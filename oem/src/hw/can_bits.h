// oem/src/hw/can_bits.h: bxCAN mailbox and filter word encodings (docs/history/notes/oem-can-sequences.md §3, §4, §5).
// Pure constexpr, so the host tests pin them.
#pragma once
#include <cstdint>

namespace hw {

constexpr uint32_t CAN_TEN = 1u << 0;  // TMIx: transmit request
constexpr uint32_t CAN_FT = 1u << 1;   // remote frame
constexpr uint32_t CAN_FF = 1u << 2;   // extended frame

// TSTAT MST0/MST1/MST2 (bits 7, 15, 23): abort the pending request in mailbox 0/1/2. Writing 0 to the other bits
// leaves them alone.
constexpr uint32_t CAN_TSTAT_MST_ALL = 1u << 7 | 1u << 15 | 1u << 23;

// CTL bits we set on top of reset (init mode): ABOR bus-off recovery, RFOD FIFO0 keeps old frames, TFO transmit in
// request order. Stock has TFO = 0 because it waits for each frame; we queue several writes with the same ID, which
// TFO = 0 could reorder (lower mailbox number wins ties), leaving a module on a stale setpoint.
constexpr uint32_t CAN_CTL_TFO = 1u << 2, CAN_CTL_RFOD = 1u << 3, CAN_CTL_ABOR = 1u << 6;
constexpr uint32_t CAN_CTL_CONFIG = CAN_CTL_ABOR | CAN_CTL_RFOD | CAN_CTL_TFO;
constexpr uint32_t CAN_CTL_CONFIG_MASK = 0xFCu;  // TFO, RFOD, ARD, AWU, ABOR, TTC

// TMIx for an extended data frame, without TEN: EFID in bits 31..3, FF set.
constexpr uint32_t can_tmi_ext(uint32_t id) { return ((id & 0x1FFFFFFFu) << 3) | CAN_FF; }

// One 32-bit mask-mode filter bank uses the same layout as TMI/RFIFOMI. The mask also compares FF and FT, so only
// extended data frames pass.
constexpr uint32_t can_filter_id_ext(uint32_t id) { return ((id & 0x1FFFFFFFu) << 3) | CAN_FF; }
constexpr uint32_t can_filter_mask_ext(uint32_t mask) { return ((mask & 0x1FFFFFFFu) << 3) | CAN_FF | CAN_FT; }

// NASN replies: protocol 0x21 with the direction bit (7) clear, the same match as the ESP's on_frame.
constexpr uint32_t NASN_REPLY_ID = 0x10800000;
constexpr uint32_t NASN_REPLY_MASK = 0x1F800080;

constexpr bool can_rx_is_ext_data(uint32_t mi) { return (mi & (CAN_FF | CAN_FT)) == CAN_FF; }
constexpr uint32_t can_rx_ext_id(uint32_t mi) { return mi >> 3; }

// BT: (psc - 1) | bs1 - 1 at 16 | bs2 - 1 at 20 | sjw - 1 at 24, normal mode.
constexpr uint32_t can_bt(uint32_t psc, uint32_t bs1_tq, uint32_t bs2_tq, uint32_t sjw_tq) {
  return ((psc - 1) & 0x3FFu) | ((bs1_tq - 1) & 0xFu) << 16 | ((bs2_tq - 1) & 0x7u) << 20 | ((sjw_tq - 1) & 0x3u) << 24;
}

constexpr uint32_t le32(const uint8_t *d) {
  return uint32_t(d[0]) | uint32_t(d[1]) << 8 | uint32_t(d[2]) << 16 | uint32_t(d[3]) << 24;
}

}  // namespace hw
