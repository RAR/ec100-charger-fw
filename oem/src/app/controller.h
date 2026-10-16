// oem/src/app/controller.h: the 1 s charge-control tick. A port of the ESP ChargeController::tick_
// (esphome/components/charge_controller) on charge_logic.h, with the source's Limits instead of YamBMS over MQTT, the
// session gate (spec §2.3) folded into "allowed", and spec §3 error codes instead of fault strings.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "app/limit_source.h"
#include "app/rectifier.h"
#include "app/session_gate.h"
#include "app/settings.h"
#include "charge_controller/charge_logic.h"

namespace oem {

enum class HoldReason : uint8_t {  // Modbus IR 2
  NONE = 0, PAUSED = 1, NOT_ALLOWED = 2, BMS_ALARM = 3, BMS_BLOCKING = 4, CELL_HIGH = 5, BUDGET = 6, WAITING = 7,
};

constexpr uint8_t E_BMS_ALARM = 12, E_MISMATCH = 13, E_OVERVOLTAGE = 14, E_SOURCE_STALE = 15, E_NO_CURRENT = 16,
                  E_BATTERY_SENSE = 17;
constexpr uint32_t CONTROLLER_TICK_MS = 1000;
// Below this (or NaN) a 48 V battery reading is a fault in the sense line, not a flat battery. Without a SOC the
// voltage is the only way a session ends, so an open sense line reading ~0 V would otherwise charge forever.
constexpr float BATTERY_SENSE_MIN_V = 20.0f;
constexpr uint32_t OVERVOLTAGE_TARGET_MS = 3000;  // "above target" must last this long; above 58.5 V faults at once
constexpr uint32_t DIAG_MAX_S = 600;  // longest diagnostic override (HR 130)

// Module soft-start. Leaving standby, a NASN module pushes ~3-4 A for a few seconds whatever its current limit
// (even 0 A), unless its voltage target is below the battery (docs/history/notes/oem-bringup.md). So a waking module is
// told to sit SOFTSTART_MARGIN_V below the battery until it is surely up, then its target rises to the charge voltage
// at SOFTSTART_V_PER_S, with the current limit in force. A module still spikes (~4 A for well under a second, whatever
// the ramp rate) when it starts to conduct, so PSU2 keeps holding until PSU1 has carried current for SOFTSTART_LEAD_MS
// (or isn't running, or SOFTSTART_MAX_HOLD_MS has passed): the two spikes don't add up. A fixed delay doesn't do it,
// because the modules' voltage calibrations differ by ~0.5 V.
constexpr float SOFTSTART_MARGIN_V = 1.0f;
constexpr uint32_t SOFTSTART_HOLD_MS = 12000;  // the modules take ~7-8 s to wake
constexpr uint32_t SOFTSTART_MAX_HOLD_MS = 30000;
constexpr float SOFTSTART_V_PER_S = 0.2f;
constexpr float SOFTSTART_LEAD_A = 0.5f;
constexpr uint32_t SOFTSTART_LEAD_MS = 2000;

class SoftStart {
 public:
  // The voltage to command this tick: target_v once the start is over or when not running.
  // may_rise: the hold may end once SOFTSTART_HOLD_MS has passed; it ends regardless at SOFTSTART_MAX_HOLD_MS.
  float update(bool run, float target_v, float battery_v, uint32_t now, bool may_rise = true) {
    if (!run) {
      active_ = false;
      return target_v;
    }
    if (!active_) {
      active_ = true;
      rising_ = false;
      since_ = now;
      start_v_ = std::isfinite(battery_v) ? battery_v - SOFTSTART_MARGIN_V : NAN;
    }
    if (!std::isfinite(start_v_) || !std::isfinite(target_v)) return target_v;
    if (!rising_) {
      const uint32_t held = now - since_;
      if (!((held >= SOFTSTART_HOLD_MS && may_rise) || held >= SOFTSTART_MAX_HOLD_MS)) return std::min(target_v, start_v_);
      rising_ = true;
      rise_since_ = now;
    }
    return std::min(target_v, start_v_ + float(now - rise_since_) / 1000.0f * SOFTSTART_V_PER_S);
  }

 private:
  bool active_ = false, rising_ = false;
  uint32_t since_ = 0, rise_since_ = 0;
  float start_v_ = NAN;
};

// E1-E11 from the 0x0183 alarm word, first match wins, as stock 08006d68 (docs/history/notes/findings-v2.md §5). E7 is
// unused.
inline uint8_t alarm_code(uint32_t a) {
  static constexpr struct {
    uint32_t bits;
    uint8_t code;
  } MAP[] = {{1u << 22, 1}, {1u << 17, 2}, {1u << 12, 3}, {1u << 4, 4},  {1u << 14, 5},
             {1u << 27, 6}, {1u << 0, 8},  {1u << 1, 9},  {1u << 20, 10}, {3u << 2, 11}};
  for (const auto &m : MAP) {
    if (a & m.bits) return m.code;
  }
  return 0;
}

inline bool is_fault_code(uint8_t e) { return (e >= 1 && e <= 14) || e == E_BATTERY_SENSE; }

struct Status {
  charge::State state = charge::State::BOOT;
  HoldReason hold = HoldReason::NONE;
  uint8_t error = 0;          // 0 or E1-E17; faults take precedence over warnings
  bool session_active = false;
  bool paused = false;
  bool cooling = false;
  bool no_current = false;
  bool stale = false;         // the configured non-panel source isn't fresh (E15)
  uint8_t source = 0;         // SourceMode of the limits in use (IR 5)
  bool diag = false;          // diagnostic override (HR 130-133) driving the modules
  int online = 0;             // modules online
  int soc = -1;
  float target_v = NAN;       // while charging
  float commanded_a = 0.0f;   // total
  float budget = 0.0f;
  float output_a = 0.0f;      // measured, total
  float power_w = 0.0f;
  float battery_v = NAN;      // in use: the source's when known, else the ADC
  float ac_v = NAN;           // highest AC input reading of the online modules
  float cvl = NAN;            // V in use, set whether or not charging
  float ccl = NAN;            // A in use, total
};

class Controller {
 public:
  Controller(Rectifier &a, Rectifier &b) : rects_{&a, &b} {}

  void charge_now() { charge_now_ = true; }
  void toggle_pause() {
    paused_ = !paused_;
    diag_.on = false;  // any pause command is the operator taking over
  }
  void set_paused(bool p) {
    paused_ = p;
    diag_.on = false;
  }
  bool paused() const { return paused_; }
  void clear_fault() { clear_fault_ = true; }
  // A Source mode change: the old source's session ends; the new one needs the start rule again.
  void end_session() { gate_.end(); }
  const Status &status() const { return st_; }
  bool module_cooling(int k) const { return cooldown_[k].active(); }

  // Diagnostic override: for `seconds`, the modules in `mask` (bit 0 = PSU1) leave standby at v / a_per_module and the
  // rest stand by, whatever the session or pause says. Ends at the timeout, on any pause command, on a fault, on a
  // BMS alarm or stop, a high cell or E17, or by stop_diag(). The total never exceeds Max current. The fault checks
  // keep running; the caller has range-checked the values.
  void start_diag(float v, float a_per_module, uint8_t mask, uint32_t seconds, uint32_t now) {
    diag_ = {v, a_per_module, mask, now + std::min(seconds, DIAG_MAX_S) * 1000u, true};
  }
  void stop_diag() { diag_.on = false; }

  // Before a restart (Up + Down held 5 s): both modules go to standby at once and stay there, with no cooldown or
  // override, until the board resets.
  void halt() {
    halting_ = true;
    diag_.on = false;
    for (Rectifier *r : rects_) r->set_standby(true);
  }
  bool halting() const { return halting_; }
  bool diag_active() const { return diag_.on; }
  uint32_t diag_remaining_s(uint32_t now) const {
    const int32_t left = int32_t(diag_.until - now);
    return diag_.on && left > 0 ? (uint32_t(left) + 999u) / 1000u : 0;
  }

  // Every CONTROLLER_TICK_MS.
  const Status &tick(const Settings &s, const Limits &lim, float adc_v, uint32_t now) {
    const float fb_v = float(s.get(HR_MODULE_FALLBACK_V)) / 100.0f, fb_a = float(s.get(HR_MODULE_FALLBACK_A));
    float our_a = 0.0f, power = 0.0f, max_vout = NAN, min_vin = NAN, max_vin = NAN;
    int healthy = 0, online = 0;
    bool boot_done = true;
    for (Rectifier *r : rects_) {
      r->set_fallback(fb_v, fb_a);
      boot_done = boot_done && r->boot_sent();
      if (!r->is_online()) continue;
      online++;
      const float v = r->output_voltage(), i = r->output_current(), vin = r->input_voltage();
      if (std::isfinite(i)) our_a += i;
      if (std::isfinite(v) && std::isfinite(i)) power += v * i;
      if (std::isfinite(v)) max_vout = std::isfinite(max_vout) ? std::max(max_vout, v) : v;
      if (std::isfinite(vin)) {
        min_vin = std::isfinite(min_vin) ? std::min(min_vin, vin) : vin;
        max_vin = std::isfinite(max_vin) ? std::max(max_vin, vin) : vin;
      }
      if (!r->faulted()) healthy++;
    }

    const float battery_v = std::isfinite(lim.battery_v) ? lim.battery_v : adc_v;
    const GateThresholds gt{int(s.get(HR_START_SOC)), int(s.get(HR_STOP_SOC)), float(s.get(HR_START_V)) / 100.0f,
                            float(s.get(HR_STOP_V)) / 100.0f};
    const bool sense_ok = lim.soc >= 0 || (std::isfinite(battery_v) && battery_v >= BATTERY_SENSE_MIN_V);
    bool session = false;
    if (sense_ok) {
      session = gate_.update({lim.soc, battery_v, our_a, charge_now_}, gt, now);
    } else {
      gate_.end();
    }
    charge_now_ = false;

    const float target_v = charge::target_voltage(lim.cvl, float(s.get(HR_CHARGE_V)) / 100.0f);
    const float max_a = charge::line_limited_max(float(s.get(HR_MAX_CURRENT)), min_vin);
    // Unknown battery current: assume nothing but us charges the battery.
    const float battery_a = std::isfinite(lim.battery_a) ? lim.battery_a : our_a;
    const float budget = charge::current_budget(lim.ccl, battery_a, our_a, max_a);

    charge::Inputs in;
    in.boot_done = boot_done;
    in.bms_fresh = lim.fresh;
    in.allowed = lim.allowed && session;
    in.bms_alarm = lim.alarm;
    in.paused = paused_;
    in.bms_blocking = lim.blocking;
    const charge::CellThresholds cell =
        charge::cell_thresholds(float(s.get(HR_MAX_CELL_MV)) / 1000.0f, float(s.get(HR_CELL_RESUME_MV)) / 1000.0f);
    in.cell_high = cell_.update(lim.max_cell_v, cell.limit, cell.resume);
    in.budget = budget;
    uint8_t fault_code = 0;
    in.fault = check_faults_(max_vout, our_a, fault_code, now);
    if (in.fault != nullptr) last_fault_code_ = fault_code;
    in.clear_fault = clear_fault_;
    clear_fault_ = false;
    const charge::State state = sm_.update(in, now);
    // The override ignores the session and a pause, but not the battery: a BMS alarm or stop, a high cell or an
    // implausible battery reading ends it as a fault does. A restart (halt()) ends it too.
    const bool battery_says_stop = lim.alarm || !lim.allowed || lim.blocking || in.cell_high || !sense_ok;
    if (diag_.on && (state == charge::State::FAULT || battery_says_stop || halting_ || int32_t(diag_.until - now) <= 0))
      diag_.on = false;
    const bool diag = diag_.on;
    // Max current caps the override's total, also when it is lowered after the start.
    const int diag_modules = (diag_.mask & 1) + (diag_.mask >> 1 & 1);
    const float diag_a = diag_modules > 0 ? std::min(diag_.a, max_a / float(diag_modules)) : 0.0f;

    const bool charging = state == charge::State::CHARGING;
    float total = 0.0f;
    if (charging && healthy > 0 && !diag) {
      total = ramp_.step(budget, now);
    } else {
      ramp_.reset();
    }
    const float per_psu = charge::per_psu_current(total, healthy);
    float commanded = charging && !diag ? per_psu * float(healthy) : 0.0f;
    int cooling = 0;
    for (int k = 0; k < 2; k++) {
      Rectifier *r = rects_[k];
      const bool run =
          !halting_ && (diag ? (diag_.mask >> k & 1) != 0 : charging) && r->is_online() && !r->faulted();
      const bool cool_ok = r->is_online() && !r->faulted() && state != charge::State::FAULT &&
                           r->output_voltage() >= charge::COOLDOWN_MIN_BATTERY_V;
      if (k == 0) {
        lead_running_ = run && !diag;
        lead_ok_ = lead_timer_.update(lead_running_ && r->output_current() >= SOFTSTART_LEAD_A, now);
      }
      const bool may_rise = k == 0 || !lead_running_ || lead_ok_;
      const float module_v =
          soft_[k].update(run && !diag, std::isfinite(target_v) ? target_v : fb_v, battery_v, now, may_rise);
      if (cooldown_[k].update(run, cool_ok && !halting_, r->heatsink_temperature(), now)) {
        cooling++;
        r->set_target(charge::COOLDOWN_V, charge::COOLDOWN_A);
        r->set_standby(false);
      } else if (diag) {
        r->set_target(diag_.v, run ? diag_a : 0.0f);
        r->set_standby(!run);
        if (run) commanded += diag_a;
      } else {
        r->set_target(module_v, run ? per_psu : 0.0f);
        r->set_standby(!run);
      }
    }
    sent_target_v_ = diag ? diag_.v : target_v;

    const bool no_current = no_current_.update(
        charge::no_current_condition(charging && !diag, commanded, our_a, max_vout, target_v, battery_v), now);

    st_.state = state;
    st_.hold = state == charge::State::HOLD ? hold_reason_(in, lim, session) : HoldReason::NONE;
    if (state == charge::State::FAULT) {
      st_.error = last_fault_code_;
    } else if (!sense_ok) {
      st_.error = E_BATTERY_SENSE;
    } else if (lim.alarm && lim.fresh) {
      st_.error = E_BMS_ALARM;
    } else if (lim.stale || !lim.fresh) {
      st_.error = E_SOURCE_STALE;
    } else {
      st_.error = no_current ? E_NO_CURRENT : 0;
    }
    st_.session_active = session;
    st_.paused = paused_;
    st_.cooling = cooling > 0;
    st_.no_current = no_current;
    st_.stale = lim.stale;
    st_.source = uint8_t(lim.source);
    st_.diag = diag;
    st_.online = online;
    st_.soc = lim.soc;
    st_.target_v = diag ? diag_.v : charging ? target_v : NAN;
    st_.commanded_a = commanded;
    st_.budget = budget;
    st_.output_a = our_a;
    st_.power_w = power;
    st_.battery_v = battery_v;
    st_.ac_v = max_vin;
    st_.cvl = target_v;
    st_.ccl = lim.ccl;
    return st_;
  }

 private:
  // A module's hardware alarm (E1-E11) on any module outranks E13 (setpoints not followed or writes rejected), which
  // outranks overvoltage (E14). The output is the same FAULT whichever code shows.
  const char *check_faults_(float max_vout, float our_a, uint8_t &code, uint32_t now) {
    // The target rule compares with the voltage the modules were last told, not this tick's new target: after Charge V
    // is lowered the output takes a moment to follow, and that isn't a fault. Timed every tick, whatever else faults.
    const bool hard = charge::overvoltage(false, max_vout, NAN, our_a);
    const bool charging = sm_.state() == charge::State::CHARGING || diag_.on;
    const bool above = ov_timer_.update(charge::overvoltage(charging, max_vout, sent_target_v_, our_a), now);
    bool faulted = false;
    for (Rectifier *r : rects_) {
      if (!r->faulted()) continue;
      faulted = true;
      code = alarm_code(r->alarms() & nasn::STOCK_ALARM_MASK);
      if (code != 0) return "PSU alarm";
    }
    if (faulted) {
      code = E_MISMATCH;
      return "PSU not following setpoints";
    }
    if (hard || above) {
      code = E_OVERVOLTAGE;
      return "Overvoltage";
    }
    return nullptr;
  }

  // charge::hold_reason's order, with "waiting for start" when it's the session, not the source, that says no.
  static HoldReason hold_reason_(const charge::Inputs &in, const Limits &lim, bool session) {
    if (in.paused) return HoldReason::PAUSED;
    if (!session) return HoldReason::WAITING;
    if (!lim.allowed) return HoldReason::NOT_ALLOWED;
    if (in.bms_alarm) return HoldReason::BMS_ALARM;
    if (in.bms_blocking) return HoldReason::BMS_BLOCKING;
    if (in.cell_high) return HoldReason::CELL_HIGH;
    if (!(in.budget >= charge::START_BUDGET_A)) return HoldReason::BUDGET;
    return HoldReason::NONE;
  }

  Rectifier *rects_[2];
  SessionGate gate_;
  charge::StateMachine sm_;
  charge::RampLimiter ramp_;
  charge::CellLimiter cell_;
  charge::Cooldown cooldown_[2];
  SoftStart soft_[2];
  nasn::PersistTimer lead_timer_{SOFTSTART_LEAD_MS};
  bool lead_running_ = false, lead_ok_ = false;
  charge::NoCurrentDetector no_current_;
  Status st_;
  nasn::PersistTimer ov_timer_{OVERVOLTAGE_TARGET_MS};
  struct {
    float v, a;
    uint8_t mask;
    uint32_t until;
    bool on;
  } diag_{NAN, 0.0f, 0, 0, false};
  float sent_target_v_ = NAN;
  bool halting_ = false;
  uint8_t last_fault_code_ = 0;
  bool charge_now_ = false, paused_ = false, clear_fault_ = false;
};

}  // namespace oem
