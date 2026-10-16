// oem/src/app/jkpb_source.h: the JK-PB BMS limit source (spec 2026-10-16-oem-jkpb-source-design.md). The board is the
// RS485 master on one pack's RS485-1 (protocol 001): one FC10 trigger request every JKPB_POLL_MS. A reply counts
// within JKPB_REPLY_MS of its request: a 300-byte record from its 55 AA EB 90 header, then the 8-byte FC10 reply naming
// the polled address and register. The pack's DIP address is found by scanning 0..15 with cell-info requests and
// locking on the first that answers; JKPB_MISSES missed polls in a row on the locked address scan again from it.
// Once locked a settings request goes first and then every JKPB_SETTINGS_MS; the others ask for cell info. Fresh
// while a valid cell-info record is at most JKPB_FRESH_MS old and a valid settings record came since the lock;
// otherwise Panel limits capped at Fallback current, flagged stale (E15). Any valid record that says stop latches
// charging off until a fresh state allows it again (charge switch off or CurBatCOC 0 count as stop). CVL = cells x
// CellRCV (CellRFV in float), CCL = CurBatCOC.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/jkpb_protocol.h"
#include "app/limit_source.h"
#include "app/polled_source.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t JKPB_BAUD = 115200;
constexpr uint32_t JKPB_POLL_MS = 1000;
constexpr uint32_t JKPB_REPLY_MS = 750;
constexpr uint32_t JKPB_FRESH_MS = 3000;
constexpr uint32_t JKPB_SETTINGS_MS = 60000;
constexpr uint32_t JKPB_HOLD_MS = JKPB_REPLY_MS + 100;  // as JK_HOLD_MS: a reply in progress holds the next request
constexpr unsigned JKPB_MISSES = 3;                      // missed polls in a row on the locked address before a re-scan
constexpr uint8_t JKPB_ADDR_COUNT = 16;                  // DIP addresses 0..15
// Alarm bits that stop charging with E12: 0-3 (wire resistance, MOSFET OT, cell count, current sensor), 5-10 (pack
// OVP, charge OCP / SCP / OT / UT, internal comm), 16 (charge MOSFET abnormal), 21-23 (battery OT, temp sensor,
// parallel module). Cell / pack UVP and the discharge bits never stop charging; bit 19 is set in normal captures.
constexpr uint32_t JKPB_ALARM_MASK = 0x00E107EF;
// Bit 4, cell OVP, fires at every normal full charge: it holds charging off but is not an alarm.
constexpr uint32_t JKPB_CELL_OVP = 0x10;

class JkpbSource final : public PolledSource {
 public:
  void reset() override { *this = JkpbSource(); }

  void push(const uint8_t *data, size_t n, uint32_t now) override {
    for (size_t i = 0; i < n; i++) {
      push_byte_(data[i], now);
      // A unit that failed may have swallowed the start of the real reply: replay its tail from the next 55. One
      // failing within the replay queues its own tail ahead of the bytes not yet replayed, keeping stream order (each
      // queue is shorter than the work it came from, so this ends).
      uint8_t work[UNIT_LEN];
      size_t m = 0, j = 0;
      while (replay_n_ != 0 || j < m) {
        if (replay_n_ != 0) {
          const size_t r = replay_n_;  // after the first, < j: it came from work[0..j)
          for (size_t k = j; k < m; k++) work[r + k - j] = work[k];
          m = r + m - j;
          for (size_t k = 0; k < r; k++) work[k] = replay_[k];
          replay_n_ = 0;
          j = 0;
        }
        push_byte_(work[j++], now);
      }
    }
  }

  size_t poll(uint32_t now, uint8_t *out) override {
    if (started_ && now - last_req_ms_ < JKPB_POLL_MS) return 0;
    if (started_ && in_frame_ && now - last_req_ms_ < JKPB_POLL_MS + JKPB_HOLD_MS) return 0;
    if (started_ && pending_) miss_();
    started_ = true;
    last_req_ms_ = now;
    pending_ = true;
    in_frame_ = false;
    len_ = 0;
    if (!locked_) {
      req_addr_ = scan_addr_;
      req_reg_ = jkpb::REG_CELL_INFO;
    } else {
      req_addr_ = addr_;
      if (!has_settings_ || now - settings_req_ms_ >= JKPB_SETTINGS_MS) {
        req_reg_ = jkpb::REG_SETTINGS;
        settings_req_ms_ = now;
      } else {
        req_reg_ = jkpb::REG_CELL_INFO;
      }
    }
    return jkpb::build_request(req_addr_, req_reg_, out);
  }

  bool fresh(uint32_t now) const { return has_cell_ && has_settings_ && now - cell_ms_ <= JKPB_FRESH_MS; }

  Limits limits(const Settings &s, uint32_t now) const override {
    if (!fresh(now)) return stale_limits(s, stopped_, SourceMode::JKPB);
    const uint32_t cell_mv = cell_.stage == 2 ? set_.rfv_mv : set_.rcv_mv;
    return {float(uint32_t(set_.cells) * cell_mv) / 1000.0f,
            float(set_.coc_ma) / 1000.0f,
            allowed_(),
            alarm_(),
            false,
            cell_.soc,
            cell_.pack_v,
            NAN,
            cell_.max_cell_v,
            true,
            false,
            SourceMode::JKPB};
  }

  uint32_t baud() const override { return JKPB_BAUD; }

  bool locked() const { return locked_; }
  uint8_t address() const { return addr_; }
  uint32_t good_replies() const { return good_; }
  uint32_t bad_replies() const { return bad_; }

 private:
  static constexpr size_t UNIT_LEN = jkpb::RECORD_LEN + jkpb::TRAILER_LEN;  // 308 bytes, ~27 ms at 115200

  bool alarm_() const { return (cell_.alarms & JKPB_ALARM_MASK) != 0; }
  // What one cell-info record says on its own: charge MOSFET on, no stopping alarm, no cell OVP.
  static bool cell_allows_(const jkpb::CellInfo &c) {
    return c.charge_mos && (c.alarms & (JKPB_ALARM_MASK | JKPB_CELL_OVP)) == 0;
  }
  // What one settings record says on its own: charge switch on and a charge current above 0 (CurBatCOC 0 holds
  // charging off, as CCL 0 does for EG4).
  static bool settings_allow_(const jkpb::ChargeSettings &st) { return st.charge_switch && st.coc_ma != 0; }
  bool allowed_() const { return cell_allows_(cell_) && settings_allow_(set_); }

  // A request went unanswered. Scanning: try the next address (15 wraps to 0). Locked: JKPB_MISSES in a row drop the
  // lock, forgetting settings and freshness (not the stop latch), and the scan restarts at the old address.
  void miss_() {
    if (!locked_) {
      scan_addr_ = uint8_t((scan_addr_ + 1) % JKPB_ADDR_COUNT);
      return;
    }
    if (++misses_ < JKPB_MISSES) return;
    locked_ = false;
    scan_addr_ = addr_;
    misses_ = 0;
    has_cell_ = false;
    has_settings_ = false;
  }

  // A unit starts at 55 AA EB 90 inside the window and is collected to RECORD_LEN + TRAILER_LEN bytes. Outside a unit
  // other bytes are dropped; a header that breaks off is dropped, the bytes after its first are re-examined as a
  // start. A unit completing after the window is dropped unchecked; one that fails its record or trailer check counts
  // bad and replays its tail from the next 55.
  void push_byte_(uint8_t c, uint32_t now) {
    if (!pending_) return;
    const bool in_window = now - last_req_ms_ <= JKPB_REPLY_MS;
    if (!in_frame_) {
      if (!in_window || c != jkpb::HEADER[0]) return;
      in_frame_ = true;
      len_ = 0;
    }
    buf_[len_++] = c;
    if (len_ <= 4 && c != jkpb::HEADER[len_ - 1]) {
      fail_();
      return;
    }
    if (len_ < UNIT_LEN) return;
    in_frame_ = false;
    len_ = 0;
    if (!in_window) return;
    const uint8_t type = req_reg_ == jkpb::REG_SETTINGS ? jkpb::TYPE_SETTINGS : jkpb::TYPE_CELL_INFO;
    if (!jkpb::record_ok(buf_, type) || !jkpb::trailer_ok(buf_ + jkpb::RECORD_LEN, req_addr_, req_reg_)) {
      bad_++;
      replay_from_(UNIT_LEN);
      return;
    }
    if (type == jkpb::TYPE_SETTINGS) {
      jkpb::ChargeSettings st;
      if (!jkpb::decode_settings(buf_, st)) {
        bad_++;
        has_settings_ = false;  // out of range: not fresh until a good one
        pending_ = false;       // but the pack did answer: not a miss
        misses_ = 0;
        return;
      }
      set_ = st;
      has_settings_ = true;
      if (!settings_allow_(st)) stopped_ = true;
    } else {
      cell_ = jkpb::decode_cell_info(buf_);
      has_cell_ = true;
      cell_ms_ = now;
      if (!locked_) {
        locked_ = true;
        addr_ = req_addr_;
        has_settings_ = false;  // settings count only from this lock on
      }
      if (!cell_allows_(cell_)) stopped_ = true;  // latched even while not fresh
    }
    good_++;
    pending_ = false;
    misses_ = 0;
    if (fresh(now) && allowed_()) stopped_ = false;
  }

  // A header that broke off: its bytes after the first get another pass (the current one may start a header).
  void fail_() {
    const size_t n = len_;
    in_frame_ = false;
    len_ = 0;
    replay_from_(n);
  }

  // Queue buf_[1..n) from the first 55 for another pass.
  void replay_from_(size_t n) {
    size_t i = 1;
    while (i < n && buf_[i] != jkpb::HEADER[0]) i++;
    for (replay_n_ = 0; i < n; i++) replay_[replay_n_++] = buf_[i];
  }

  uint8_t buf_[UNIT_LEN] = {};
  uint8_t replay_[UNIT_LEN] = {};
  size_t replay_n_ = 0;
  size_t len_ = 0;
  bool in_frame_ = false;
  bool started_ = false;
  bool pending_ = false;  // a request awaiting its reply
  uint32_t last_req_ms_ = 0;
  uint8_t req_addr_ = 0;  // what the outstanding request asked
  uint16_t req_reg_ = jkpb::REG_CELL_INFO;
  bool locked_ = false;
  uint8_t addr_ = 0;       // the locked address
  uint8_t scan_addr_ = 0;  // the next address the scan asks
  unsigned misses_ = 0;    // missed polls in a row while locked
  uint32_t settings_req_ms_ = 0;
  jkpb::CellInfo cell_{NAN, -1, NAN, 0, false, 0};
  bool has_cell_ = false;
  uint32_t cell_ms_ = 0;
  jkpb::ChargeSettings set_{0, 0, 0, 0, false};
  bool has_settings_ = false;  // a valid settings record since the current lock
  uint32_t good_ = 0, bad_ = 0;
  bool stopped_ = false;
};

}  // namespace oem
