#include "app/can_port.h"
#include "check.h"
#include "hw/can_bits.h"

using namespace oem;

static void test_mailbox_words() {
  // The stock firmware's pending mailbox for "read 0x023B from PSU 1" (docs/history/notes/oem-can-sequences.md §4.1).
  CHECK((hw::can_tmi_ext(0x108182FE) | hw::CAN_TEN) == 0x840C17F5);
  const uint8_t d[8] = {0x02, 0x3B, 0, 0, 0, 0, 0, 0};
  CHECK(hw::le32(d) == 0x00003B02 && hw::le32(d + 4) == 0);
  CHECK(hw::can_bt(30, 5, 2, 1) == 0x0014001D);  // stock BT: 30 MHz / 30, 1 + 5 + 2 tq = 125 kbit/s
  // Stock's ABOR | RFOD (0x48) plus TFO: the queue sends the same ID repeatedly, so mailboxes must go in request order.
  CHECK(hw::CAN_CTL_CONFIG == 0x4C && (hw::CAN_CTL_CONFIG & hw::CAN_CTL_CONFIG_MASK) == hw::CAN_CTL_CONFIG);
}

static bool passes(uint32_t mi) {
  const uint32_t id = hw::can_filter_id_ext(hw::NASN_REPLY_ID), mask = hw::can_filter_mask_ext(hw::NASN_REPLY_MASK);
  return ((mi ^ id) & mask) == 0;
}

static void test_filter_words() {
  CHECK(hw::can_filter_id_ext(hw::NASN_REPLY_ID) == 0x84000004);
  CHECK(hw::can_filter_mask_ext(hw::NASN_REPLY_MASK) == 0xFC000406);
  CHECK(passes(hw::can_tmi_ext(0x1081827E)));            // read reply from PSU 1
  CHECK(passes(hw::can_tmi_ext(0x1082817E)));            // write reply from PSU 2
  CHECK(!passes(hw::can_tmi_ext(0x108182FE)));           // a controller request (bit 7 set)
  CHECK(!passes(hw::can_tmi_ext(0x1101007E)));           // proto 0x22 module chatter, dropped in software by stock
  CHECK(!passes(hw::can_tmi_ext(0x1081827E) | hw::CAN_FT));  // remote frame
  CHECK(!passes(hw::can_filter_id_ext(hw::NASN_REPLY_ID) & ~hw::CAN_FF));  // same ID bits, standard frame
  CHECK(!passes(hw::can_tmi_ext(0x1102007E)));           // PSU 2 chatter
  CHECK(passes(hw::can_tmi_ext(0x1082827F)));            // write reply from PSU 2, other register
  CHECK(passes(hw::can_tmi_ext(0x1081817E)));            // write reply from PSU 1
}

static void test_rx_decode() {
  const uint32_t mi = hw::can_tmi_ext(0x1081827E);
  CHECK(hw::can_rx_is_ext_data(mi) && hw::can_rx_ext_id(mi) == 0x1081827E);
  CHECK(!hw::can_rx_is_ext_data(mi | hw::CAN_FT));
}

static void test_tx_queue_order_and_backpressure() {
  CanTxQueue q;
  uint8_t d[8] = {};
  for (int i = 0; i < CAN_TX_QUEUE_LEN; i++) {
    d[0] = uint8_t(i);
    CHECK(q.send(0x100u + uint32_t(i), d));
  }
  CHECK(!q.send(0x999, d));  // full
  int mailboxes = 3, next = 0;
  bool in_order = true;
  q.pump([&](const CanFrame &f) {
    if (mailboxes == 0) return false;
    mailboxes--;
    in_order = in_order && f.id == 0x100u + uint32_t(next) && f.data[0] == next && f.len == 8;
    next++;
    return true;
  });
  CHECK(in_order && next == 3 && q.size() == CAN_TX_QUEUE_LEN - 3);
  CHECK(q.send(0x200, d));
  q.pump([&](const CanFrame &) { return true; });
  CHECK(q.size() == 0);
}

static void test_abort_bits() {
  // TSTAT MST0/1/2: bit 7 of each mailbox's byte (MTF0 bit 0, MTFNERR0 bit 1 as in
  // docs/history/notes/oem-can-sequences.md §5).
  CHECK(hw::CAN_TSTAT_MST_ALL == 0x00808080);
}

static void test_tx_queue_clear() {
  CanTxQueue q;
  uint8_t d[8] = {};
  for (int i = 0; i < 5; i++) q.send(0x100u + uint32_t(i), d);
  q.clear();
  CHECK(q.size() == 0);
  CHECK(q.send(0x300, d));
  uint32_t first = 0;
  q.pump([&](const CanFrame &f) {
    first = f.id;
    return false;
  });
  CHECK(first == 0x300 && q.size() == 1);
}

static void test_outage_flush_once_per_transition() {
  OutageFlush f;
  CHECK(!f.update(false));  // nothing was ever online (boot): nothing stale to drop
  CHECK(!f.update(true));
  CHECK(f.update(false));   // the last module went away
  CHECK(!f.update(false));  // once
  CHECK(!f.update(true));
  CHECK(f.update(false));
}

int main() {
  test_mailbox_words();
  test_filter_words();
  test_rx_decode();
  test_tx_queue_order_and_backpressure();
  test_abort_bits();
  test_tx_queue_clear();
  test_outage_flush_once_per_transition();
  return test_result("test_oem_can");
}
