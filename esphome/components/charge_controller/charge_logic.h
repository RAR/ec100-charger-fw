#pragma once
// Charge-control decisions for the ESPHome charger controller (see the design spec).
// Pure C++17 with no ESPHome dependencies, so it is unit-tested on the host (tests/test_charge_logic.cpp).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <string>
#include <utility>

namespace charge {

constexpr uint32_t FRESH_MS = 60000;

// ---- BMS inputs (YamBMS over MQTT) ----

// BLOCKING (bms_blocking_charge), MAX_CELL (max_cell_voltage, volts) and BATTERY_V (total_voltage) are
// optional: freshness doesn't depend on them.
enum class Topic : uint8_t { STATUS, CVL, CCL, ALLOWED, ALARM, CURRENT, BLOCKING, MAX_CELL, BATTERY_V };
constexpr int TOPIC_COUNT = 9;

// Tracks the latest YamBMS values and whether the periodic ones are live. ESPHome's MQTT callback
// doesn't expose the retain flag, so the first message per topic after each (re)connect is treated
// as a possibly-stale retained replay: its value is stored but it doesn't count as an update.
class BmsTracker {
 public:
  void on_reconnect() {
    for (auto &t : topics_) t.seen_since_connect = false;
  }

  // Returns true when the message counted as a live update (not a retained replay or garbage).
  bool on_message(Topic topic, const std::string &payload, uint32_t now_ms) {
    float v = 0.0f;
    switch (topic) {
      case Topic::STATUS:
        v = payload == "online" ? 1.0f : 0.0f;
        break;
      case Topic::ALLOWED:
        if (payload == "ON") {
          v = 1.0f;
        } else if (payload == "OFF") {
          v = 0.0f;
        } else {
          return false;
        }
        break;
      default:
        if (!parse_float_(payload, &v)) return false;
    }
    Entry &t = topics_[int(topic)];
    const bool live = t.seen_since_connect;
    t.seen_since_connect = true;
    // A safety block may be set by a retained replay but only cleared by a live update.
    if (topic == Topic::BLOCKING && !live && v == 0.0f && t.has_value && t.value != 0.0f) return false;
    t.value = v;
    t.has_value = true;
    t.live = live;
    if (live) {
      t.last_update_ms = now_ms;
      t.has_update = true;
      t.expired = false;
    }
    return live;
  }

  // Not const: once a periodic topic has gone stale it stays stale until its next live update, so
  // millis() wrapping after 49.7 days can't make an old update look fresh again. Call it every tick.
  bool fresh(uint32_t now_ms) {
    bool expired = false;
    for (Topic p : PERIODIC) {
      Entry &t = topics_[int(p)];
      if (t.has_update && now_ms - t.last_update_ms > FRESH_MS) t.expired = true;
      expired = expired || t.expired;
    }
    return !expired && age_ms(now_ms) <= FRESH_MS && online() && has_(Topic::ALLOWED) && has_(Topic::ALARM);
  }

  // Age of the stalest periodic topic (CVL, CCL, current), or UINT32_MAX if one has never updated live.
  uint32_t age_ms(uint32_t now_ms) const {
    uint32_t worst = 0;
    for (Topic p : PERIODIC) {
      const Entry &t = topics_[int(p)];
      if (!t.has_update) return UINT32_MAX;
      worst = std::max(worst, now_ms - t.last_update_ms);
    }
    return worst;
  }

  bool online() const { return has_(Topic::STATUS) && value_(Topic::STATUS) == 1.0f; }
  bool allowed() const { return has_(Topic::ALLOWED) && value_(Topic::ALLOWED) == 1.0f; }
  bool alarm() const { return has_(Topic::ALARM) && value_(Topic::ALARM) != 0.0f; }
  float cvl() const { return value_(Topic::CVL); }
  float ccl() const { return value_(Topic::CCL); }
  float current() const { return value_(Topic::CURRENT); }
  // Some pack's BMS is blocking charge (non-zero). False until a value arrives.
  bool blocking() const { return has_(Topic::BLOCKING) && value_(Topic::BLOCKING) != 0.0f; }
  // Highest cell voltage in volts, NAN until a value arrives.
  float max_cell() const { return value_(Topic::MAX_CELL); }
  // Pack voltage from a live update at most FRESH_MS old, else NAN.
  float battery_voltage(uint32_t now_ms) const {
    const Entry &t = topics_[int(Topic::BATTERY_V)];
    return t.has_update && t.live && now_ms - t.last_update_ms <= FRESH_MS ? t.value : NAN;
  }
  // The stored max cell value came from a live update, not a retained replay.
  bool max_cell_live() const { return topics_[int(Topic::MAX_CELL)].live; }

  // Time since the topic's last live update, or UINT32_MAX if it has never had one.
  uint32_t topic_age_ms(Topic topic, uint32_t now_ms) const {
    const Entry &t = topics_[int(topic)];
    return t.has_update ? now_ms - t.last_update_ms : UINT32_MAX;
  }

 private:
  struct Entry {
    float value = NAN;
    bool has_value = false;
    bool live = false;  // the stored value came from a live update
    bool seen_since_connect = false;
    bool has_update = false;
    bool expired = false;  // went stale since the last live update
    uint32_t last_update_ms = 0;
  };

  static bool parse_float_(const std::string &s, float *out) {
    char *end = nullptr;
    float v = std::strtof(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
  }

  bool has_(Topic t) const { return topics_[int(t)].has_value; }
  float value_(Topic t) const { return topics_[int(t)].value; }

  static constexpr Topic PERIODIC[] = {Topic::CVL, Topic::CCL, Topic::CURRENT};
  Entry topics_[TOPIC_COUNT];
};

// ---- Setpoints ----

constexpr float HARD_MAX_V = 58.0f;
constexpr float MAX_A_PER_PSU = 50.0f;
constexpr float OVERVOLTAGE_MARGIN_V = 0.5f;
constexpr float OVERVOLTAGE_MIN_A = 1.0f;  // only a fault while we're actually pushing current
constexpr float RAMP_A_PER_S = 2.0f;
constexpr float RAMP_MAX_DT_S = 1.0f;  // the controller ticks at 1 Hz; longer gaps mustn't skip the ramp
constexpr uint32_t AVERAGE_MS = 10000;

inline float target_voltage(float cvl, float user_max_v) {
  if (!std::isfinite(cvl) || !std::isfinite(user_max_v)) return NAN;
  return std::min({cvl, user_max_v, HARD_MAX_V});
}

// The stock firmware caps total charge current at 60 A while AC input (0x0178) is below 180 V.
constexpr float LOW_LINE_V = 180.0f;
constexpr float LOW_LINE_MAX_A = 60.0f;

// User max current, capped at LOW_LINE_MAX_A when the lowest AC input reading is below LOW_LINE_V.
// An unknown (NaN) input voltage leaves it unchanged.
inline float line_limited_max(float user_max_a, float min_input_v) {
  if (std::isfinite(min_input_v) && min_input_v < LOW_LINE_V && user_max_a > LOW_LINE_MAX_A) return LOW_LINE_MAX_A;
  return user_max_a;
}

// Current we may add: CCL minus what everything else (e.g. the inverter) already pushes into the battery.
// A load drawing from the battery (others < 0) doesn't raise the budget past the CCL, and CCL <= 0 is no charging.
inline float current_budget(float ccl, float battery_a, float our_a, float user_max_a) {
  if (!std::isfinite(ccl) || !std::isfinite(battery_a) || !std::isfinite(our_a) || !std::isfinite(user_max_a))
    return 0.0f;
  if (ccl <= 0.0f) return 0.0f;
  const float others = std::max(0.0f, battery_a - our_a);
  return std::max(0.0f, std::min(user_max_a, ccl - others));
}

// Mean of the samples added within the last window_ms.
class MovingAverage {
 public:
  explicit MovingAverage(uint32_t window_ms = AVERAGE_MS) : window_ms_(window_ms) {}

  void add(float v, uint32_t now_ms) {
    if (!std::isfinite(v)) return;
    samples_.emplace_back(now_ms, v);
    while (!samples_.empty() && now_ms - samples_.front().first > window_ms_) samples_.pop_front();
  }

  float value() const {
    if (samples_.empty()) return NAN;
    float sum = 0.0f;
    for (const auto &s : samples_) sum += s.second;
    return sum / float(samples_.size());
  }

  void reset() { samples_.clear(); }

 private:
  uint32_t window_ms_;
  std::deque<std::pair<uint32_t, float>> samples_;
};

// Rises at most RAMP_A_PER_S, falls immediately.
class RampLimiter {
 public:
  float step(float target, uint32_t now_ms) {
    if (!started_) {
      started_ = true;
      last_ms_ = now_ms;
    }
    float dt = std::min(float(now_ms - last_ms_) / 1000.0f, RAMP_MAX_DT_S);
    last_ms_ = now_ms;
    output_ = std::max(0.0f, std::min(target, output_ + RAMP_A_PER_S * dt));
    return output_;
  }

  void reset() {
    output_ = 0.0f;
    started_ = false;
  }

  float output() const { return output_; }

 private:
  float output_ = 0.0f;
  uint32_t last_ms_ = 0;
  bool started_ = false;
};

inline float per_psu_current(float total, int healthy) {
  if (healthy <= 0 || !(total > 0.0f)) return 0.0f;
  return std::min(total / float(healthy), MAX_A_PER_PSU);
}

inline bool overvoltage(bool charging, float measured_v, float target_v, float our_a) {
  if (!std::isfinite(measured_v)) return false;
  if (measured_v > HARD_MAX_V + OVERVOLTAGE_MARGIN_V) return true;
  return charging && std::isfinite(target_v) && measured_v > target_v + OVERVOLTAGE_MARGIN_V &&
         our_a > OVERVOLTAGE_MIN_A;
}

// ---- Cell voltage limit ----

// Defaults when the HA thresholds are missing or NaN.
constexpr float CELL_LIMIT_V = 3.55f;
constexpr float CELL_RESUME_V = 3.40f;
constexpr float CELL_LIMIT_MAX_V = 3.58f;  // highest allowed user limit: stays below the 3.600 V BMS trip
constexpr float CELL_PLAUSIBLE_MIN_V = 2.0f;  // max-cell readings outside this range are ignored
constexpr float CELL_PLAUSIBLE_MAX_V = 4.5f;

struct CellThresholds {
  float limit;
  float resume;
  bool fallback;  // the inputs were invalid and the defaults are used instead
};

// User thresholds, or CELL_LIMIT_V / CELL_RESUME_V for both if either is non-finite, resume >= limit, or
// limit > CELL_LIMIT_MAX_V, so a bad setting never disables or raises the cutoff past the BMS trip.
inline CellThresholds cell_thresholds(float limit, float resume) {
  if (std::isfinite(limit) && std::isfinite(resume) && resume < limit && limit <= CELL_LIMIT_MAX_V)
    return {limit, resume, false};
  return {CELL_LIMIT_V, CELL_RESUME_V, true};
}

// Latches high when the highest cell reaches limit, releases once it falls to resume. Only a live reading
// may release it (a retained replay can be hours old). A NaN or implausible cell voltage (outside
// CELL_PLAUSIBLE_MIN_V..MAX_V), or invalid thresholds (non-finite, or resume >= limit), leave the state unchanged.
class CellLimiter {
 public:
  bool update(float max_cell, float limit, float resume, bool live = true) {
    if (!(max_cell >= CELL_PLAUSIBLE_MIN_V && max_cell <= CELL_PLAUSIBLE_MAX_V)) return high_;
    if (!std::isfinite(limit) || !std::isfinite(resume) || !(resume < limit)) return high_;
    if (max_cell >= limit) {
      high_ = true;
    } else if (max_cell <= resume && live) {
      high_ = false;
    }
    return high_;
  }

  bool high() const { return high_; }

 private:
  bool high_ = false;
};

// ---- Charging without current ----

// CHARGING with current commanded, the modules sitting at the target voltage yet delivering nothing, while the
// battery is well below that target: the DC output isn't reaching the battery (breaker, fuse, connector).
// A full battery at its target voltage doesn't match, because its voltage is at the target.
constexpr float NO_CURRENT_MIN_COMMANDED_A = 2.0f;
constexpr float NO_CURRENT_MAX_OUTPUT_A = 1.0f;
constexpr float NO_CURRENT_VOUT_MARGIN_V = 0.5f;  // module output within this of the target
constexpr float NO_CURRENT_BATTERY_GAP_V = 1.0f;  // battery at least this far below the target
constexpr uint32_t NO_CURRENT_MS = 60000;

// Any NaN input (unknown battery voltage included) makes it false.
inline bool no_current_condition(bool charging, float commanded_a, float output_a, float max_vout, float target_v,
                                 float battery_v) {
  return charging && commanded_a >= NO_CURRENT_MIN_COMMANDED_A && output_a < NO_CURRENT_MAX_OUTPUT_A &&
         max_vout >= target_v - NO_CURRENT_VOUT_MARGIN_V && battery_v < target_v - NO_CURRENT_BATTERY_GAP_V;
}

// True once the condition has held continuously for NO_CURRENT_MS; false as soon as it doesn't.
class NoCurrentDetector {
 public:
  bool update(bool condition, uint32_t now_ms) {
    if (!condition) {
      timing_ = false;
      return false;
    }
    if (!timing_) {
      timing_ = true;
      since_ms_ = now_ms;
    }
    return now_ms - since_ms_ >= NO_CURRENT_MS;
  }

 private:
  bool timing_ = false;
  uint32_t since_ms_ = 0;
};

// ---- Cooldown ----

// Standby stops a module's fans. A module that stops delivering while hot stays on instead, with its
// voltage setpoint below the battery so no current flows, until it cools or COOLDOWN_MAX_MS passes.
constexpr float COOLDOWN_START_C = 45.0f;  // heatsink at or above this when the module stops: cool down
constexpr float COOLDOWN_STOP_C = 40.0f;   // standby once the heatsink is at or below this
constexpr uint32_t COOLDOWN_MAX_MS = 600000;
constexpr float COOLDOWN_V = 48.0f;  // below the battery: the module sources nothing
constexpr float COOLDOWN_A = 1.0f;   // backstop limit; not 0 A (Huawei modules can wedge on a 0 A limit while on)
constexpr float COOLDOWN_MIN_BATTERY_V = 49.0f;  // below this, COOLDOWN_V isn't safely under the battery

// One per module. Cooling starts only on the tick the module stops running, so it never re-arms after
// timing out. allowed = module online, not faulted, controller not in FAULT, and its output (battery)
// voltage at or above COOLDOWN_MIN_BATTERY_V; it going false ends cooling at once. A NaN temperature
// never starts cooling and ends it.
class Cooldown {
 public:
  bool update(bool run, bool allowed, float heatsink_c, uint32_t now_ms) {
    if (run) {
      was_running_ = true;
      active_ = false;
      return false;
    }
    if (was_running_) {
      was_running_ = false;
      if (allowed && heatsink_c >= COOLDOWN_START_C) {
        active_ = true;
        since_ms_ = now_ms;
      }
    }
    if (active_ && (!allowed || !(heatsink_c > COOLDOWN_STOP_C) || now_ms - since_ms_ >= COOLDOWN_MAX_MS))
      active_ = false;
    return active_;
  }

  bool active() const { return active_; }

 private:
  bool was_running_ = false;
  bool active_ = false;
  uint32_t since_ms_ = 0;
};

// ---- State machine ----

constexpr float START_BUDGET_A = 2.0f;     // HOLD -> CHARGING needs at least this much headroom
constexpr uint32_t FAULT_CLEAR_MS = 30000;  // a fault must stay clear this long before auto-recovery

enum class State : uint8_t { BOOT, WAIT_BMS, CHARGING, HOLD, FAULT };

inline const char *state_name(State s) {
  switch (s) {
    case State::BOOT:
      return "BOOT";
    case State::WAIT_BMS:
      return "WAIT_BMS";
    case State::CHARGING:
      return "CHARGING";
    case State::HOLD:
      return "HOLD";
    case State::FAULT:
      return "FAULT";
  }
  return "?";
}

struct Inputs {
  bool boot_done = false;
  bool bms_fresh = false;
  bool allowed = false;
  bool bms_alarm = false;
  bool paused = false;
  bool bms_blocking = false;  // some pack's BMS is blocking charge
  bool cell_high = false;     // CellLimiter latched on the highest cell voltage
  float budget = 0.0f;
  const char *fault = nullptr;  // non-null while a fault condition is present (the reason)
  bool clear_fault = false;     // the Clear fault button was pressed since the last update
};

// Why the inputs hold charging, highest priority first; "" when none applies. The budget case also covers
// the START_BUDGET_A hysteresis; it's only published in HOLD, so CHARGING with 0 < budget < 2 A never shows it.
inline const char *hold_reason(const Inputs &in) {
  if (in.paused) return "paused";
  if (!in.allowed) return "BMS not allowing charge";
  if (in.bms_alarm) return "BMS alarm";
  if (in.bms_blocking) return "BMS blocking charge";
  if (in.cell_high) return "cell voltage high";
  if (!(in.budget >= START_BUDGET_A)) return "current budget below 2 A";
  return "";
}

class StateMachine {
 public:
  State update(const Inputs &in, uint32_t now_ms) {
    if (in.fault != nullptr) {
      state_ = State::FAULT;
      last_fault_ = in.fault;
      clear_timing_ = false;
      return state_;
    }
    if (state_ == State::FAULT) {
      if (!clear_timing_) {
        clear_timing_ = true;
        clear_since_ms_ = now_ms;
      }
      if (!in.clear_fault && now_ms - clear_since_ms_ < FAULT_CLEAR_MS) return state_;
      clear_timing_ = false;
    }
    if (!in.boot_done) {
      state_ = State::BOOT;
    } else if (!in.bms_fresh) {
      state_ = State::WAIT_BMS;
    } else if (in.paused || !in.allowed || in.bms_alarm || in.bms_blocking || in.cell_high || !(in.budget > 0.0f)) {
      state_ = State::HOLD;
    } else if (state_ == State::CHARGING || in.budget >= START_BUDGET_A) {
      state_ = State::CHARGING;
    } else {
      state_ = State::HOLD;
    }
    return state_;
  }

  State state() const { return state_; }
  // The last fault's reason, "" before the first one. in.fault must outlive the next fault (a literal or a
  // buffer that is only rewritten when a new fault is reported), since only the pointer is kept: no heap.
  const char *last_fault() const { return last_fault_; }

 private:
  State state_ = State::BOOT;
  const char *last_fault_ = "";
  bool clear_timing_ = false;
  uint32_t clear_since_ms_ = 0;
};

}  // namespace charge
