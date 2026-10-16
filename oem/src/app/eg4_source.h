// oem/src/app/eg4_source.h: the EG4 BMS limit source (spec 2026-10-16-oem-eg4-jk-sources-design.md). Modbus RTU
// master: one read every EG4_POLL_MS, alternating block A (pack V, cells) and block B (max charge current, SOC,
// protection). A reply counts within EG4_REPLY_MS of its request, collected to its known length. Fresh while both
// blocks have arrived within EG4_FRESH_MS; otherwise Panel limits capped at Fallback current, flagged stale (E15),
// unless the last state was a stop (alarm or CCL 0, latched by any valid block B): that holds charging off until the
// BMS is fresh again.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/eg4_protocol.h"
#include "app/limit_source.h"
#include "app/polled_source.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t EG4_BAUD = 9600;
constexpr uint32_t EG4_POLL_MS = 500;
constexpr uint32_t EG4_REPLY_MS = 400;
constexpr uint32_t EG4_FRESH_MS = 3000;
constexpr uint32_t EG4_HOLD_MS = EG4_REPLY_MS + 100;  // as PYLON_HOLD_MS: a reply in progress holds the next request
// Protection bits that stop charging: pack OV, cell OV, charge OC, MOS OT, charge OT, charge UT. Pack/cell UV are not
// here: a flat battery must stay chargeable.
constexpr uint16_t EG4_ALARM_MASK = 0x0593;

class Eg4Source final : public PolledSource {
 public:
  void reset() override { *this = Eg4Source(); }

  void push(const uint8_t *data, size_t n, uint32_t now) override {
    for (size_t i = 0; i < n; i++) {
      push_byte_(data[i], now);
      // A frame that failed its CRC may have swallowed the start of the real reply: replay its tail from the next
      // ADDR byte (each pass is shorter, so this ends).
      while (replay_n_ != 0) {
        uint8_t tmp[eg4::MAX_REPLY];
        const size_t m = replay_n_;
        for (size_t j = 0; j < m; j++) tmp[j] = replay_[j];
        replay_n_ = 0;
        for (size_t j = 0; j < m; j++) push_byte_(tmp[j], now);
      }
    }
  }

  size_t poll(uint32_t now, uint8_t *out) override {
    if (started_ && now - last_req_ms_ < EG4_POLL_MS) return 0;
    if (started_ && in_frame_ && now - last_req_ms_ < EG4_POLL_MS + EG4_HOLD_MS) return 0;
    started_ = true;
    last_req_ms_ = now;
    pending_ = next_a_ ? 'A' : 'B';
    const size_t n = next_a_ ? eg4::build_request(eg4::A_FIRST, eg4::A_COUNT, out)
                             : eg4::build_request(eg4::B_FIRST, eg4::B_COUNT, out);
    next_a_ = !next_a_;
    in_frame_ = false;
    len_ = 0;
    return n;
  }

  bool fresh(uint32_t now) const { return recent_(has_a_, a_ms_, now) && recent_(has_b_, b_ms_, now); }

  Limits limits(const Settings &s, uint32_t now) const override {
    if (!fresh(now)) return stale_limits(s, stopped_, SourceMode::EG4);
    return {float(s.get(HR_CHARGE_V)) / 100.0f,
            b_.ccl,
            !alarm_(),
            alarm_(),
            false,
            b_.soc,
            a_.pack_v,
            NAN,
            a_.max_cell_v,
            true,
            false,
            SourceMode::EG4};
  }

  uint32_t baud() const override { return EG4_BAUD; }

  uint32_t good_replies() const { return good_; }
  uint32_t bad_replies() const { return bad_; }

 private:
  static bool recent_(bool has, uint32_t at, uint32_t now) { return has && now - at <= EG4_FRESH_MS; }
  bool alarm_() const { return (b_.protection & EG4_ALARM_MASK) != 0; }
  bool stop_() const { return alarm_() || b_.ccl <= 0.0f; }  // as the Modbus source: CCL 0 holds charging off

  // A reply starts at an ADDR byte inside the window and is collected to the length the request implies. Outside a
  // frame, other bytes are dropped. The function (byte 1) and byte count (byte 2) are checked as they arrive; a
  // mismatch drops what was collected and resyncs, the mismatching byte being re-examined as a start. A frame
  // completing after the window is dropped unparsed.
  void push_byte_(uint8_t c, uint32_t now) {
    if (pending_ == 0) return;
    const bool in_window = now - last_req_ms_ <= EG4_REPLY_MS;
    const uint16_t count = pending_ == 'A' ? eg4::A_COUNT : eg4::B_COUNT;
    if (in_frame_) {
      const bool ok = len_ != 1 ? (len_ != 2 || c == 2 * count) : c == 0x03;
      if (!ok) {
        if (in_window) bad_++;
        in_frame_ = false;
        len_ = 0;
      }
    }
    if (!in_frame_) {
      if (!in_window || c != eg4::ADDR) return;
      in_frame_ = true;
      len_ = 0;
    }
    buf_[len_++] = c;
    if (len_ < eg4::reply_len(count)) return;
    in_frame_ = false;
    const size_t n = len_;
    len_ = 0;
    if (!in_window) return;
    const uint8_t *d = nullptr;
    if (eg4::parse_reply(buf_, n, count, d) != eg4::Error::OK) {
      bad_++;
      size_t i = 1;
      while (i < n && buf_[i] != eg4::ADDR) i++;
      for (replay_n_ = 0; i < n; i++) replay_[replay_n_++] = buf_[i];
      return;
    }
    if (pending_ == 'A') {
      const eg4::BlockA a = eg4::decode_a(d);
      if (!(a.pack_v > 0.0f)) {  // the all-zero reply quirk
        bad_++;
        return;
      }
      a_ = a;
      has_a_ = true;
      a_ms_ = now;
    } else {
      bool zero = true;  // the all-zero reply quirk, as for block A
      for (size_t i = 0; i < 2 * size_t(eg4::B_COUNT); i++) zero = zero && d[i] == 0;
      if (zero) {
        bad_++;
        return;
      }
      b_ = eg4::decode_b(d);
      has_b_ = true;
      b_ms_ = now;
    }
    good_++;
    const bool was_b = pending_ == 'B';
    pending_ = 0;
    // A valid block B that by itself says stop latches even while the source is not fresh; only a fresh, fine state
    // clears the latch.
    if (fresh(now)) stopped_ = stop_();
    else if (was_b && stop_()) stopped_ = true;
  }

  uint8_t buf_[eg4::MAX_REPLY] = {};
  uint8_t replay_[eg4::MAX_REPLY] = {};
  size_t replay_n_ = 0;
  size_t len_ = 0;
  bool in_frame_ = false;
  bool started_ = false;
  bool next_a_ = true;
  char pending_ = 0;  // 'A' or 'B' awaiting its reply, 0 = none
  uint32_t last_req_ms_ = 0;
  eg4::BlockA a_{NAN, NAN};
  eg4::BlockB b_{-1, 0.0f, 0};
  bool has_a_ = false, has_b_ = false;
  uint32_t a_ms_ = 0, b_ms_ = 0;
  uint32_t good_ = 0, bad_ = 0;
  bool stopped_ = false;
};

}  // namespace oem
