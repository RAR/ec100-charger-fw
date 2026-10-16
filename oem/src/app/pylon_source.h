// oem/src/app/pylon_source.h: the Pylontech limit source (spec 2026-10-15-oem-pylontech-source-design.md §4-5). The
// board is the RS485 master: one request every PYLON_POLL_MS cycling 0x61 -> 0x62 -> 0x63; a reply counts within
// PYLON_REPLY_MS of its request and is parsed as the command polled. Fresh while a 0x61 and a 0x63 have both arrived
// within PYLON_FRESH_MS; otherwise Panel limits capped at Fallback current, flagged stale (E15), unless the last
// fresh state was a BMS stop (charge enable off or an alarm): that holds charging off until the BMS is fresh again.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/limit_source.h"
#include "app/polled_source.h"
#include "app/pylon_protocol.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t PYLON_BAUD = 9600;  // the BMS line's baud, whatever HR 8 says
constexpr uint32_t PYLON_POLL_MS = 500;
constexpr uint32_t PYLON_REPLY_MS = 400;
constexpr uint32_t PYLON_FRESH_MS = 3000;
// A reply still arriving when the next request falls due holds that request (half duplex) for at most this long; a
// partial frame that never ends is then abandoned and the request sent.
constexpr uint32_t PYLON_HOLD_MS = PYLON_REPLY_MS + 100;

class PylonSource final : public PolledSource {
 public:
  // A mode change: forget the BMS (stale until it answers again) and start the cycle over at 0x61.
  void reset() override { *this = PylonSource(); }

  // Every pass, with the bytes received since the last call.
  void push(const uint8_t *data, size_t n, uint32_t now) override {
    for (size_t i = 0; i < n; i++) push_byte_(char(data[i]), now);
  }

  // Every pass while the line is free. When a request is due, writes it to out and returns its length; else 0.
  size_t poll(uint32_t now, uint8_t *out) override {
    if (started_ && now - last_req_ms_ < PYLON_POLL_MS) return 0;
    if (started_ && in_frame_ && now - last_req_ms_ < PYLON_POLL_MS + PYLON_HOLD_MS) return 0;  // a frame on the line
    started_ = true;
    last_req_ms_ = now;
    pending_ = CYCLE[next_];
    next_ = uint8_t((next_ + 1) % 3);
    in_frame_ = false;
    len_ = 0;
    return pylon::build_request(pending_, reinterpret_cast<char *>(out));
  }

  bool fresh(uint32_t now) const { return recent_(has_analog_, analog_ms_, now) && recent_(has_mgmt_, mgmt_ms_, now); }

  Limits limits(const Settings &s, uint32_t now) const override {
    if (!fresh(now)) {
      return stale_limits(s, stopped_, SourceMode::PYLONTECH);
    }
    return {mgmt_.cvl,
            mgmt_.ccl,
            mgmt_.charge_enable,
            recent_(has_alarm_, alarm_ms_, now) && alarm_,
            false,
            analog_.soc,
            analog_.battery_v,
            NAN,  // the 0x61 current's unit is unconfirmed (spec §1)
            analog_.max_cell_v,
            true,
            false,
            SourceMode::PYLONTECH};
  }

  uint32_t baud() const override { return PYLON_BAUD; }

  uint32_t good_replies() const { return good_; }
  uint32_t bad_replies() const { return bad_; }

 private:
  static constexpr uint8_t CYCLE[3] = {pylon::CID_ANALOG, pylon::CID_ALARM, pylon::CID_MGMT};

  static bool recent_(bool has, uint32_t at, uint32_t now) { return has && now - at <= PYLON_FRESH_MS; }

  // Every frame on the line is followed (in_frame_ holds the next request); only one ending inside the reply window of
  // an outstanding request is parsed.
  void push_byte_(char c, uint32_t now) {
    const bool in_window = pending_ != 0 && now - last_req_ms_ <= PYLON_REPLY_MS;
    if (c == '~') {
      in_frame_ = true;
      len_ = 0;
    }
    if (!in_frame_) return;
    if (len_ >= pylon::MAX_FRAME) {
      in_frame_ = false;
      if (in_window) bad_++;
      return;
    }
    buf_[len_++] = c;
    if (c != '\r') return;
    in_frame_ = false;
    if (!in_window) return;
    uint8_t info[pylon::MAX_INFO];
    size_t n = 0;
    if (pylon::parse_reply(buf_, len_, pylon::min_info(pending_), info, n) != pylon::Error::OK) {
      bad_++;
      return;
    }
    good_++;
    if (pending_ == pylon::CID_ANALOG) {
      analog_ = pylon::decode_analog(info);
      has_analog_ = true;
      analog_ms_ = now;
    } else if (pending_ == pylon::CID_ALARM) {
      alarm_ = pylon::decode_alarm(info);
      has_alarm_ = true;
      alarm_ms_ = now;
    } else {
      mgmt_ = pylon::decode_mgmt(info);
      has_mgmt_ = true;
      mgmt_ms_ = now;
    }
    const uint8_t pending_done = pending_;
    pending_ = 0;
    // A valid reply that by itself says stop latches even while the source is not fresh; only a fresh, fine state
    // clears the latch.
    const bool says_stop = (pending_done == pylon::CID_MGMT && !mgmt_.charge_enable) ||
                           (pending_done == pylon::CID_ALARM && alarm_);
    if (fresh(now)) stopped_ = !mgmt_.charge_enable || (recent_(has_alarm_, alarm_ms_, now) && alarm_);
    else if (says_stop) stopped_ = true;
  }

  char buf_[pylon::MAX_FRAME] = {};
  size_t len_ = 0;
  bool in_frame_ = false;
  bool started_ = false;
  uint8_t next_ = 0;
  uint8_t pending_ = 0;  // CID2 awaiting its reply, 0 = none
  uint32_t last_req_ms_ = 0;
  pylon::Analog analog_{NAN, -1, NAN};
  pylon::Mgmt mgmt_{NAN, NAN, 0.0f, 0.0f, 0, false};
  bool alarm_ = false;
  bool has_analog_ = false, has_mgmt_ = false, has_alarm_ = false;
  uint32_t analog_ms_ = 0, mgmt_ms_ = 0, alarm_ms_ = 0;
  uint32_t good_ = 0, bad_ = 0;
  bool stopped_ = false;  // the last fresh state had charge enable off or an alarm
};

}  // namespace oem
