// oem/src/hw/can.cpp: register sequence from the stock v1 driver (docs/history/notes/oem-can-sequences.md §9), with a
// NASN-reply filter instead of accept-all, RFD0 written rather than OR'ed, and no RX interrupt.
#include "hw/can.h"

#include "hw/can_bits.h"
#include "hw/gpio.h"

namespace hw {
namespace {

constexpr uint32_t IWMOD = 1u << 0, SLPWMOD = 1u << 1;
constexpr uint32_t IWS = 1u << 0;            // STAT: in init mode
constexpr uint32_t RFD0 = 1u << 5;           // RFIFO0: release the output mailbox
constexpr uint32_t TME0 = 1u << 26;          // TSTAT: mailbox 0 empty (1 and 2 follow)

bool wait_stat(bool want_init) {
  for (uint32_t t = 0; t < 0xFFFF; t++) {
    if (bool(reg(CAN_STAT) & IWS) == want_init) return true;
  }
  return false;
}

}  // namespace

bool can_init() {
  reg(RCU_APB1EN) |= 1u << 16;  // CANEN
  reg(RCU_AHBEN) |= 1u << 17;   // GPIOA
  pin_config({GPIOA, 4}, Mode::AF, Pull::NONE, 11);  // CAN_RX
  pin_config({GPIOA, 5}, Mode::AF, Pull::NONE, 11);  // CAN_TX
  reg(RCU_APB1RST) |= 1u << 16;
  reg(RCU_APB1RST) &= ~(1u << 16);

  reg(CAN_CTL) &= ~SLPWMOD;
  reg(CAN_CTL) |= IWMOD;
  if (!wait_stat(true)) return false;
  reg(CAN_BT) = can_bt(30, 5, 2, 1);  // 0x0014001D: 30 MHz / 30, 8 tq, 125 kbit/s, sample point 75 %
  // As stock: automatic bus-off recovery, automatic retransmission, FIFO0 keeps the oldest frames when full.
  // Unlike stock (which waits for each frame): TFO, so queued writes with the same ID leave in request order.
  rmw(CAN_CTL, CAN_CTL_CONFIG_MASK, CAN_CTL_CONFIG);
  reg(CAN_INTEN) = 0;

  reg(CAN_FCTL) |= 1u;  // filter init mode
  reg(CAN_FW) &= ~1u;
  reg(CAN_FSCFG) |= 1u;  // bank 0: 32-bit
  reg(CAN_F0DATA0) = can_filter_id_ext(NASN_REPLY_ID);
  reg(CAN_F0DATA1) = can_filter_mask_ext(NASN_REPLY_MASK);
  reg(CAN_FMCFG) &= ~1u;   // mask mode
  reg(CAN_FAFIFO) &= ~1u;  // to FIFO0
  reg(CAN_FW) |= 1u;
  reg(CAN_FCTL) &= ~1u;

  reg(CAN_CTL) &= ~IWMOD;
  return wait_stat(false);  // needs 11 recessive bits on the bus
}

bool can_send(const oem::CanFrame &f) {
  const uint32_t tstat = reg(CAN_TSTAT);
  int n = 0;
  while (n < 3 && !(tstat & (TME0 << n))) n++;
  if (n == 3) return false;
  reg(can_tmi(n)) = can_tmi_ext(f.id);
  rmw(can_tmp(n), 0xFu, 8u);
  reg(can_tmdata0(n)) = le32(f.data);
  reg(can_tmdata1(n)) = le32(f.data + 4);
  reg(can_tmi(n)) |= CAN_TEN;
  return true;
}

bool can_receive(oem::CanFrame &f) {
  const uint32_t fifo = reg(CAN_RFIFO0);
  if ((fifo & 3u) == 0 || (fifo & RFD0)) return false;  // empty, or the last release is still in progress
  const uint32_t mi = reg(CAN_RFIFOMI0);
  const uint32_t len = reg(CAN_RFIFOMP0) & 0xFu;
  const uint32_t d0 = reg(CAN_RFIFOMDATA00), d1 = reg(CAN_RFIFOMDATA10);
  reg(CAN_RFIFO0) = RFD0;
  f.id = can_rx_ext_id(mi);
  f.len = can_rx_is_ext_data(mi) && len <= 8 ? uint8_t(len) : 0;
  for (int i = 0; i < 4; i++) {
    f.data[i] = uint8_t(d0 >> (8 * i));
    f.data[4 + i] = uint8_t(d1 >> (8 * i));
  }
  return true;
}

void can_abort_tx() { reg(CAN_TSTAT) = CAN_TSTAT_MST_ALL; }

uint32_t can_error() { return reg(CAN_ERR); }

}  // namespace hw
