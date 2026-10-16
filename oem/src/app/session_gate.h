// oem/src/app/session_gate.h: when charging is wanted (spec §2.3, stock semantics). The controller sees
// allowed = source.allowed && active().
#pragma once
#include <cmath>
#include <cstdint>

namespace oem {

constexpr uint32_t SESSION_START_HOLD_MS = 5000;
constexpr uint32_t SESSION_STOP_HOLD_MS = 10000;
constexpr float SESSION_STOP_MAX_A = 4.0f;  // stop by voltage only once the tail current is this low

struct GateThresholds {
  int start_soc, stop_soc;  // %
  float start_v, stop_v;    // V
};

struct GateInputs {
  int soc;           // %, -1 when unknown
  float battery_v;   // NaN when unknown
  float output_a;    // our total output
  bool charge_now;   // Switch click (or HR 113 in phase 3) since the last update
};

class SessionGate {
 public:
  bool update(const GateInputs &in, const GateThresholds &t, uint32_t now_ms) {
    const bool start = (in.soc >= 0 && in.soc <= t.start_soc) || in.battery_v < t.start_v;
    const bool stop =
        (in.soc >= 0 && in.soc >= t.stop_soc) || (in.battery_v >= t.stop_v && in.output_a <= SESSION_STOP_MAX_A);
    if (first_) {
      first_ = false;
      active_ = start;  // a battery that needs charging at power-up starts at once
    }
    if (in.charge_now) {
      active_ = true;
      stop_.reset();
    }
    if (active_) {
      start_.reset();
      if (stop_.held(stop, now_ms, SESSION_STOP_HOLD_MS)) {
        active_ = false;
        stop_.reset();
      }
    } else if (start_.held(start, now_ms, SESSION_START_HOLD_MS)) {
      active_ = true;
      start_.reset();
    }
    return active_;
  }

  bool active() const { return active_; }
  // The battery reading can't be trusted (controller E17): no session, and a new one needs the start rule again.
  // first_ is kept, so the first plausible reading after an implausible boot still counts as the power-up one.
  void end() {
    active_ = false;
    start_.reset();
    stop_.reset();
  }

 private:
  struct Hold {
    bool timing = false;
    uint32_t since = 0;
    bool held(bool cond, uint32_t now, uint32_t ms) {
      if (!cond) {
        timing = false;
        return false;
      }
      if (!timing) {
        timing = true;
        since = now;
      }
      return now - since >= ms;
    }
    void reset() { timing = false; }
  };

  bool first_ = true;
  bool active_ = false;
  Hold start_, stop_;
};

}  // namespace oem
