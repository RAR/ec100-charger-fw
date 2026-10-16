// oem/src/hw/rs485.cpp: register values from the stock USART1 init and RX DMA
// (docs/history/notes/oem-usart-sequences.md §1-§2, bits per §7), but a free-running ring read by DMA CNT instead of
// stock's IDLE-restarted buffer, and TX by interrupt instead of polling, so a main-loop stall can't open a gap inside a
// frame (plan M2). That doesn't cover flash erase or program: Flash masks interrupts for the whole operation, which
// holds off this ISR too. So the main loop defers settings saves while rs485_tx_busy() (Task 5), and a reply is never
// interleaved with a flash write.
//
// ISR / main-loop handoff. tx_busy is the owner token for the TX state and for CTL0's TBEIE/TCIE:
// - While !tx_busy, neither TBEIE nor TCIE is set, so the USART1 interrupt has no enabled source and can't run.
//   The main loop owns everything: rs485_send fills the buffer, sets tx_busy, raises DE and enables TBEIE with rmw(),
//   and rs485_init / rs485_set_baud may rmw() CTL0.
// - While tx_busy, only the ISR touches CTL0 and the TX state. It clears tx_busy last, after disabling TCIE, and the
//   main loop can only observe that once the ISR has returned. So no rmw() of CTL0 ever overlaps and no masking is
//   needed.
// - The one exception is rs485_abort_tx (a send that never reached TC): it takes the token back with IRQ28 masked.
// The main loop only reads tx_busy and tx_done_ms. Both sides write USART1_INTC (write-1-to-clear, no rmw) and DE goes
// low via GPIOB_BC (a plain write).
#include "hw/rs485.h"

#include "board.h"
#include "hw/clock.h"
#include "hw/gpio.h"

namespace hw {
namespace {

constexpr uint32_t UEN = 1u << 0, REN = 1u << 2, TEN = 1u << 3, TCIE = 1u << 6, TBEIE = 1u << 7;  // CTL0
constexpr uint32_t DENR = 1u << 6, OVRD = 1u << 12;                                                // CTL2
constexpr uint32_t ORERR = 1u << 3, TC = 1u << 6, TBE = 1u << 7, TEA = 1u << 21;                 // STAT
constexpr uint32_t FEC = 1u << 1, NEC = 1u << 2, OREC = 1u << 3, TCC = 1u << 6;                    // INTC
constexpr uint32_t CHEN = 1u << 0, CMEN = 1u << 5, MNAGA = 1u << 7, PRIO_ULTRA = 3u << 12;         // DMA CHxCTL
constexpr uint32_t USART1_RX_REQ = 52;  // DMAMUX request ID
constexpr uint32_t USART1_BIT = 1u << 17;  // APB1EN / APB1RST
constexpr int IRQ_USART1 = 28;

constexpr size_t RING = 512;  // power of two; holds a ~300-byte JK reply plus a loop's delay
// Main loop must read before the DMA laps it; past this we can no longer trust which bytes are new.
constexpr size_t RING_LIMIT = RING - 32;
constexpr size_t TX_MAX = 256;

bool g_ready = false;  // set only by a successful rs485_init
uint8_t g_rx_ring[RING];
size_t g_rx_rd = 0;
uint32_t g_rx_overruns = 0;

uint8_t g_tx_buf[TX_MAX];
size_t g_tx_len = 0;
size_t g_tx_idx = 0;  // next byte to write to TDATA; ISR-owned while busy
volatile bool g_tx_busy = false;
volatile uint32_t g_tx_done_ms = 0;

// CK_APB1 from the live prescaler (RCU_CFG0 bits 10:8): clock_init() may have fallen back to IRC16M.
uint32_t apb1_hz() {
  const uint32_t psc = (reg(RCU_CFG0) >> 8) & 7u;
  return psc & 4u ? core_hz() >> ((psc & 3u) + 1u) : core_hz();
}

// 16x oversampling, as stock usart_baudrate_set (§4): 9600 at 30 MHz gives 0x0C35. 0 if out of range.
uint32_t baud_div(uint32_t baud) {
  if (baud == 0) return 0;
  const uint32_t udiv = (apb1_hz() + baud / 2u) / baud;
  return udiv >= 16u && udiv <= 0xFFFFu ? udiv : 0;
}

// TEA only: the one live STAT reading (stock UART4, receiving) is 0x002000C0, TEA without REA. A missing ack is
// counted, not fatal, so Modbus can never switch itself off for good.
void wait_enabled() {
  for (uint32_t t = 0; t < 0x10000; t++) {
    if (reg(USART1_STAT) & TEA) return;
  }
  g_rs485_ack_timeout++;
}

size_t ring_write_pos() { return (RING - reg(DMA_CH1CNT)) & (RING - 1); }  // CNT counts down, reloads at 0

}  // namespace

uint32_t g_rs485_ack_timeout = 0;

bool rs485_init(uint32_t baud) {
  g_ready = false;
  reg(RCU_AHBEN) |= (1u << 0) | (1u << 17) | (1u << 18);  // DMA, GPIOA, GPIOB
  pin_write(board::RS485_DE, false);
  pin_config(board::RS485_DE, Mode::OUTPUT, Pull::NONE);
  reg(RCU_APB1EN) |= USART1_BIT;
  reg(RCU_APB1RST) |= USART1_BIT;
  reg(RCU_APB1RST) &= ~USART1_BIT;
  const uint32_t div = baud_div(baud);
  if (div == 0) return false;
  pin_config({GPIOA, 2}, Mode::AF, Pull::UP, 7);  // TX; pin_config already gives OSPD 3 (50 MHz) as stock
  pin_config(board::USART1_RX, Mode::AF, Pull::UP, 7);

  // After the reset CTL0 = 0 (UEN off), so CTL1/CTL2/BAUD are writable. OVRD: no ORERR, so an overrun can never
  // block reception; a lost byte just corrupts the frame, which the CRC catches.
  reg(USART1_BAUD) = div;
  reg(USART1_CTL1) = 0;  // 1 stop bit
  reg(USART1_CTL2) = OVRD | DENR;

  reg(DMA_CH1CTL) = 0;
  reg(DMA_INTC) = 0xFu << 4;  // channel 1 flags
  reg(DMA_CH1PADDR) = USART1_RDATA;
  reg(DMA_CH1MADDR) = reinterpret_cast<uint32_t>(g_rx_ring);
  reg(DMA_CH1CNT) = RING;
  reg(DMAMUX_RM_CH1CFG) = USART1_RX_REQ;  // also clears SYNCEN and the other request-line options
  reg(DMA_CH1CTL) = PRIO_ULTRA | MNAGA | CMEN;  // 8-bit both sides, periph to memory, no DMA interrupts
  reg(DMA_CH1CTL) |= CHEN;
  g_rx_rd = 0;

  g_tx_busy = false;
  reg(USART1_CTL0) = UEN | REN | TEN;  // 8N1, 16x oversampling, no interrupts until a send
  reg(NVIC_ISER0) = 1u << IRQ_USART1;
  wait_enabled();
  g_ready = true;
  return true;
}

bool rs485_set_baud(uint32_t baud) {
  const uint32_t div = baud_div(baud);
  if (!g_ready || g_tx_busy || div == 0) return false;
  rmw(USART1_CTL0, UEN, 0);  // BAUD is only writable with UEN off
  reg(USART1_BAUD) = div;
  rmw(USART1_CTL0, UEN, UEN);
  wait_enabled();
  return true;
}

size_t rs485_read(uint8_t *buf, size_t max) {
  const size_t wr = ring_write_pos();
  __asm volatile("" ::: "memory");  // read the ring only after CNT: the DMA writes it behind the compiler's back
  if (reg(USART1_STAT) & ORERR) g_rx_overruns++;  // can't happen with OVRD; counted in case
  reg(USART1_INTC) = FEC | NEC | OREC;
  size_t avail = (wr - g_rx_rd) & (RING - 1);
  if (avail > RING_LIMIT) {
    g_rx_overruns++;
    g_rx_rd = wr;
    return 0;
  }
  if (avail > max) avail = max;
  for (size_t i = 0; i < avail; i++) {
    buf[i] = g_rx_ring[g_rx_rd];
    g_rx_rd = (g_rx_rd + 1) & (RING - 1);
  }
  return avail;
}

bool rs485_send(const uint8_t *data, size_t n) {
  if (!g_ready || g_tx_busy || n == 0 || n > TX_MAX) return false;
  for (size_t i = 0; i < n; i++) g_tx_buf[i] = data[i];
  g_tx_len = n;
  g_tx_idx = 1;
  g_tx_busy = true;
  pin_write(board::RS485_DE, true);
  for (volatile uint32_t i = 0; i < 60; i = i + 1) {  // a few us for the driver to turn on (stock waits <= 1 ms)
  }
  // TC is cleared before every TDATA write (here and in the ISR), so a TC left by the previous frame or by a stall
  // mid-frame can't end this one early: after the last byte, TC can only come from that byte.
  reg(USART1_INTC) = TCC;
  __asm volatile("" ::: "memory");  // buffer and indices are written before the ISR can run
  reg(USART1_TDATA) = g_tx_buf[0];
  rmw(USART1_CTL0, TBEIE, TBEIE);
  return true;
}

// Masks IRQ28 so the ISR can't be mid-rmw() of CTL0 while we take the TX state back.
void rs485_abort_tx() {
  reg(NVIC_ICER0) = 1u << IRQ_USART1;
  __asm volatile("dsb\n isb" ::: "memory");
  rmw(USART1_CTL0, TBEIE | TCIE, 0);
  pin_write(board::RS485_DE, false);
  g_tx_done_ms = millis();
  g_tx_busy = false;
  reg(NVIC_ISER0) = 1u << IRQ_USART1;
}

bool rs485_tx_busy() { return g_tx_busy; }
uint32_t rs485_tx_done_ms() { return g_tx_done_ms; }
uint32_t rs485_rx_overruns() { return g_rx_overruns; }

// TBE: next byte, or after the last one switch to TC. TC: the stop bit is out, so drop DE at once (as stock).
extern "C" void USART1_IRQHandler() {
  const uint32_t stat = reg(USART1_STAT), ctl0 = reg(USART1_CTL0);
  if ((ctl0 & TBEIE) && (stat & TBE)) {
    if (g_tx_idx < g_tx_len) {
      reg(USART1_INTC) = TCC;
      reg(USART1_TDATA) = g_tx_buf[g_tx_idx++];
    } else {
      rmw(USART1_CTL0, TBEIE | TCIE, TCIE);  // no TCC here: the last byte's TC may already be set
    }
  } else if ((ctl0 & TCIE) && (stat & TC)) {
    rmw(USART1_CTL0, TCIE, 0);
    reg(USART1_INTC) = TCC;
    pin_write(board::RS485_DE, false);
    g_tx_done_ms = millis();
    g_tx_busy = false;  // last: hands CTL0 back to the main loop
  }
}

}  // namespace hw
