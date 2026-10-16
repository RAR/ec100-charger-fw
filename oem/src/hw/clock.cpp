#include "hw/clock.h"

namespace hw {
namespace {

volatile uint32_t g_ms = 0;
uint32_t g_hz = 16000000;
uint32_t g_reset_flags = 0;

void spin(uint32_t n) {
  for (volatile uint32_t i = 0; i < n; i = i + 1) {
  }
}

// The PLL never locked or never became the system clock: stay on IRC16M with HXTAL and the PLL off.
bool pll_failed() {
  rmw(RCU_CFG0, 0x3u, 0x0u);
  while ((reg(RCU_CFG0) & 0xCu) != 0) {  // IRC16M is running: this switch completes
  }
  reg(RCU_CTL) &= ~((1u << 16) | (1u << 24));
  g_hz = 16000000;
  return false;
}

}  // namespace

extern "C" void SysTick_Handler() { g_ms = g_ms + 1; }

// The bootloader jumps to us with TIMER1, UART4 and its UART4 receive DMA still running
// (docs/history/notes/oem-init-sequences.md §1c). The DMA would keep writing into our RAM, and PB0 must be high-Z so it
// can't fight USART1 TX on the transceiver; its pull-up keeps it idling high, as stock drives it.
void takeover_from_bootloader() {
  reg(RCU_APB1RST) |= 1u;  // TIMER1
  reg(RCU_APB1RST) &= ~1u;
  reg(RCU_APB1RST) |= 1u << 20;  // UART4
  reg(RCU_APB1RST) &= ~(1u << 20);
  reg(DMA_CH3CTL) &= ~1u;  // CHEN: the bootloader's UART4 RX DMA (buffer 0x20000464)
  reg(RCU_CFG2) = 0;       // as stock SystemInit does
  reg(RCU_INT) = 0;
  reg(RCU_AHBEN) |= 1u << 18;  // GPIOB clock, needed to release PB0/PB1
  rmw(GPIOB + GPIO_CTL, 0xFu, 0x0u);  // PB0, PB1 inputs
  rmw(GPIOB + GPIO_PUD, 0xFu, 0x1u);  // PB0 pull-up, PB1 no pull
  reg(0xE000E280) = 0xFFFFFFFFu;  // ICPR0/1 last: a peripheral still asserting would re-pend at once
  reg(0xE000E284) = 0xFFFFFFFFu;
}

bool clock_init() {
  g_reset_flags = reg(RCU_RSTSCK) & 0xFF800000u;
  reg(RCU_RSTSCK) |= 1u << 24;  // RSTFC: clear the reset flags for the next boot

  // Step off the bootloader's PLL onto IRC16M, as stock SystemInit (0800cdf4) does, stepping AHB down first.
  reg(RCU_CTL) |= 1u;
  while (!(reg(RCU_CTL) & 2u)) {
  }
  if ((reg(RCU_CFG0) & 0xCu) == 0x8u) {
    rmw(RCU_CFG0, 0xF0u, 0x80u);
    spin(80);
    rmw(RCU_CFG0, 0xF0u, 0x90u);
    spin(80);
  }
  rmw(RCU_CFG0, 0x3u, 0x0u);
  while ((reg(RCU_CFG0) & 0xCu) != 0) {
  }
  reg(RCU_CTL) &= ~((1u << 16) | (1u << 18) | (1u << 19) | (1u << 24));  // HXTAL, bypass, monitor, PLL off
  reg(RCU_CFG0) = 0;
  reg(RCU_CFG1) &= ~0xFu;
  takeover_from_bootloader();
  g_hz = 16000000;

  // Stock clock config (0800f7dc): HXTAL x5 = 60 MHz, APB1 /2, 2 flash wait states.
  reg(RCU_CTL) |= 1u << 16;
  for (uint32_t t = 0; !(reg(RCU_CTL) & (1u << 17)); t++) {
    if (t > 0x40000) return false;
  }
  rmw(FMC_WS, 0x7u, 0x2u);
  rmw(RCU_CFG0, 0x700u, 0x400u);
  rmw(RCU_CFG0, 0x80FF0000u, 0x000D0000u);  // PLLSEL = HXTAL, PLLMF = x5
  reg(RCU_CTL) |= 1u << 24;
  for (uint32_t t = 0; !(reg(RCU_CTL) & (1u << 25)); t++) {
    if (t > 0x40000) return pll_failed();
  }
  rmw(RCU_CFG0, 0x3u, 0x2u);
  for (uint32_t t = 0; (reg(RCU_CFG0) & 0xCu) != 0x8u; t++) {
    if (t > 0x40000) return pll_failed();
  }
  g_hz = 60000000;
  return true;
}

uint32_t core_hz() { return g_hz; }

void fwdgt_init() {
  reg(RCU_RSTSCK) |= 1u;  // IRC32K clocks the FWDGT
  while (!(reg(RCU_RSTSCK) & 2u)) {
  }
  reg(FWDGT_CTL) = 0x5555;
  while (reg(FWDGT_STAT) & 1u) {
  }
  reg(FWDGT_PSC) = 3;  // /32
  reg(FWDGT_CTL) = 0x5555;
  while (reg(FWDGT_STAT) & 2u) {
  }
  reg(FWDGT_RLD) = 0x800;  // ~2.05 s
  reg(FWDGT_CTL) = 0xAAAA;
  reg(FWDGT_CTL) = 0xCCCC;
}

void systick_init() {
  reg(SYST_RVR) = g_hz / 1000u - 1u;
  reg(SYST_CVR) = 0;
  reg(SYST_CSR) = 7;  // core clock, interrupt, enable
}

uint32_t millis() { return g_ms; }
uint32_t reset_flags() { return g_reset_flags; }

void system_reset() {
  __asm volatile("dsb" ::: "memory");
  reg(SCB_AIRCR) = 0x05FA0004u;  // SYSRESETREQ
  for (;;) {
  }
}

}  // namespace hw
