// oem/src/app/jk_source.h: the JK BMS limit source (spec 2026-10-16-oem-eg4-jk-sources-design.md). The board is the
// RS485 master: one "read all" request every JK_POLL_MS. A reply counts within JK_REPLY_MS of its request, collected
// from its 4E 57 header to the length the frame states. Fresh while a valid reply is at most JK_FRESH_MS old;
// otherwise Panel limits capped at Fallback current, flagged stale (E15), unless the last fresh state was a stop:
// that holds charging off until the BMS is fresh again. The JK gives no charge limit, so CVL is Charge V (Panel)
// and CCL is Max current. Some units answer only the 0x02 source byte, so three polls without a valid reply flip
// the request's source byte between 0x03 and 0x02.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/jk_protocol.h"
#include "app/limit_source.h"
#include "app/polled_source.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t JK_BAUD = 115200;
constexpr uint32_t JK_POLL_MS = 1000;
constexpr uint32_t JK_REPLY_MS = 600;
constexpr uint32_t JK_FRESH_MS = 3000;
constexpr uint32_t JK_HOLD_MS = JK_REPLY_MS + 100;  // as PYLON_HOLD_MS: a reply in progress holds the next request
constexpr unsigned JK_FLIP_AFTER = 3;               // polls without a valid reply before the source byte flips
// 0x8B alarm bits that stop charging: MOS OT, charge OV, battery OT, charge OC, box OT, battery UT, cell OV, and the
// two "309_A" bits. Cell delta (bit 7) is not here: it would strand an imbalanced flat pack. Low capacity, discharge
// UV / OC and cell UV don't stop charging either.
constexpr uint16_t JK_ALARM_MASK = 0x3736;

class JkSource final : public PolledSource {
 public:
  void reset() override { *this = JkSource(); }

  void push(const uint8_t *data, size_t n, uint32_t now) override {
    for (size_t i = 0; i < n; i++) {
      push_byte_(data[i], now);
      // A frame that failed may have swallowed the start of the real reply: replay its tail from the next 4E. One
      // failing within the replay queues its own tail ahead of the bytes not yet replayed, keeping stream order (each
      // queue is shorter than the work it came from, so this ends).
      uint8_t work[jk::MAX_FRAME];
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
    if (started_ && now - last_req_ms_ < JK_POLL_MS) return 0;
    if (started_ && in_frame_ && now - last_req_ms_ < JK_POLL_MS + JK_HOLD_MS) return 0;
    if (started_ && pending_ && ++misses_ >= JK_FLIP_AFTER) {
      source_byte_ = source_byte_ == 0x03 ? 0x02 : 0x03;
      misses_ = 0;
    }
    started_ = true;
    last_req_ms_ = now;
    pending_ = true;
    in_frame_ = false;
    len_ = 0;
    return jk::build_request(source_byte_, out);
  }

  bool fresh(uint32_t now) const { return has_ && now - ok_ms_ <= JK_FRESH_MS; }

  Limits limits(const Settings &s, uint32_t now) const override {
    if (!fresh(now)) return stale_limits(s, stopped_, SourceMode::JK);
    return {float(s.get(HR_CHARGE_V)) / 100.0f,
            float(s.get(HR_MAX_CURRENT)),
            allowed_(),
            alarm_(),
            false,
            d_.soc,
            d_.pack_v,
            NAN,
            d_.max_cell_v,
            true,
            false,
            SourceMode::JK};
  }

  uint32_t baud() const override { return JK_BAUD; }

  uint32_t good_replies() const { return good_; }
  uint32_t bad_replies() const { return bad_; }
  uint8_t source_byte() const { return source_byte_; }

 private:
  bool alarm_() const { return (d_.alarms & JK_ALARM_MASK) != 0; }
  // Charge MOSFET on (bit 0), no stopping alarm. Bit 3 is not used: the source labels it "battery dropped(?)" and
  // its own normal example is 0x0007.
  bool allowed_() const { return (d_.status & 0x0001) != 0 && !alarm_(); }

  // A reply starts at 4E 57 inside the window and is collected to the total its length field states, which must be
  // plausible (MIN_FRAME..MAX_FRAME) as soon as it is read. Outside a frame other bytes are dropped. A lone 4E not
  // followed by 57 is dropped, the byte after it being re-examined as a start. A frame completing after the window is
  // dropped unparsed; one that fails replays its tail from the next 4E.
  void push_byte_(uint8_t c, uint32_t now) {
    if (!pending_) return;
    const bool in_window = now - last_req_ms_ <= JK_REPLY_MS;
    if (in_frame_ && len_ == 1 && c != 0x57) {
      if (in_window) bad_++;
      in_frame_ = false;
      len_ = 0;
    }
    if (!in_frame_) {
      if (!in_window || c != 0x4E) return;
      in_frame_ = true;
      len_ = 0;
    }
    buf_[len_++] = c;
    if (len_ == 4) {
      total_ = jk::frame_len(buf_[2], buf_[3]);
      if (total_ == 0) {
        if (in_window) bad_++;
        fail_();
        return;
      }
    }
    if (len_ < 4 || len_ < total_) return;
    in_frame_ = false;
    const size_t n = len_;
    len_ = 0;
    if (!in_window) return;
    jk::Data d;
    if (jk::parse_reply(buf_, n, d) != jk::Error::OK) {
      bad_++;
      replay_from_(n);
      return;
    }
    d_ = d;
    has_ = true;
    ok_ms_ = now;
    good_++;
    pending_ = false;
    misses_ = 0;
    stopped_ = !allowed_();
  }

  void fail_() {
    const size_t n = len_;
    in_frame_ = false;
    len_ = 0;
    replay_from_(n);
  }

  // Queue buf_[1..n) from the first 4E for another pass.
  void replay_from_(size_t n) {
    size_t i = 1;
    while (i < n && buf_[i] != 0x4E) i++;
    for (replay_n_ = 0; i < n; i++) replay_[replay_n_++] = buf_[i];
  }

  uint8_t buf_[jk::MAX_FRAME] = {};
  uint8_t replay_[jk::MAX_FRAME] = {};
  size_t replay_n_ = 0;
  size_t len_ = 0;
  size_t total_ = 0;
  bool in_frame_ = false;
  bool started_ = false;
  bool pending_ = false;  // a request awaiting its reply
  unsigned misses_ = 0;
  uint8_t source_byte_ = 0x03;
  uint32_t last_req_ms_ = 0;
  jk::Data d_{NAN, -1, NAN, 0, 0, 0, 0};
  bool has_ = false;
  uint32_t ok_ms_ = 0;
  uint32_t good_ = 0, bad_ = 0;
  bool stopped_ = false;
};

}  // namespace oem
