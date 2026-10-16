#include <cmath>

#include "app/controller.h"
#include "app/crc16.h"
#include "app/eg4_source.h"
#include "app/jkpb_protocol.h"
#include "app/jkpb_source.h"
#include "check.h"
#include "fake_nasn_bus.h"

using namespace oem;
using charge::State;

struct Rig {
  FakeBus bus;
  FakeModule m1{1}, m2{2};
  Rectifier r1{bus, 1, true}, r2{bus, 2, true};
  Controller ctl{r1, r2};
  Settings s = default_settings();
  float adc_v = 50.0f;  // between Start V (42.0) and Stop V (57.6): no session until something starts one
  uint32_t now = 1000;
  Status st;
  const Limits *lim = nullptr;  // the source's limits when set, else Panel

  void run_s(int seconds) {
    for (int t = 0; t < seconds * 1000; t++, now++) {
      r1.loop(now);
      r2.loop(now);
      bus.deliver({&m1, &m2}, {&r1, &r2}, now);
      if (now % CONTROLLER_TICK_MS == 0) st = ctl.tick(s, lim ? *lim : panel_limits(s), adc_v, now);
    }
  }
};

static void test_alarm_codes() {
  CHECK(alarm_code(0) == 0);
  CHECK(alarm_code(1u << 22) == 1);
  CHECK(alarm_code(1u << 17) == 2);
  CHECK(alarm_code(1u << 0) == 8);
  CHECK(alarm_code(1u << 3) == 11 && alarm_code(1u << 2) == 11);
  CHECK(alarm_code((1u << 22) | 1u) == 1);  // first match wins
  CHECK(is_fault_code(14) && !is_fault_code(15) && !is_fault_code(0));
}

static void test_waits_for_start_then_charge_now() {
  Rig g;
  g.run_s(5);
  CHECK(g.st.state == State::HOLD && g.st.hold == HoldReason::WAITING && !g.st.session_active);
  CHECK(g.st.online == 2 && g.m1.standby == 1 && g.m2.standby == 1);
  CHECK_NEAR(g.st.ac_v, 230.0, 0.01);
  g.ctl.charge_now();
  g.run_s(1);
  CHECK(g.st.state == State::CHARGING && g.st.session_active);
  g.run_s(1);
  CHECK(g.m1.standby == 0 && g.m2.standby == 0);
  CHECK_NEAR(g.m1.vout_set, 49.0, 0.01);  // soft-start: woken 1 V below the battery
  CHECK_NEAR(g.st.target_v, 57.6, 0.01);
  CHECK(g.st.commanded_a <= 4.01f);  // ramped at 2 A/s
  g.run_s(60);
  CHECK_NEAR(g.m1.vout_set, 57.6, 0.01);
  CHECK_NEAR(g.st.commanded_a, 10.0, 0.01);  // Max current 10 A, split 5 + 5
  CHECK_NEAR(g.m1.ilimit, 5.0, 0.1);
  CHECK_NEAR(g.m2.ilimit, 5.0, 0.1);
  CHECK_NEAR(g.st.output_a, 10.0, 0.1);
  CHECK(g.st.error == 0);
}

static void test_low_battery_starts_session_at_boot() {
  Rig g;
  g.adc_v = 41.0f;
  g.run_s(3);
  CHECK(g.st.session_active && g.st.state == State::CHARGING);
}

static void test_split_cap_and_low_line() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 100);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK_NEAR(g.m1.ilimit, 50.0, 0.1);  // 100 A over two modules, <= 50 A each
  CHECK_NEAR(g.m2.ilimit, 50.0, 0.1);
  g.m1.vin = 175.0f;  // low line: 60 A total
  g.run_s(3);
  CHECK_NEAR(g.st.commanded_a, 60.0, 0.01);
  CHECK_NEAR(g.m1.ilimit, 30.0, 0.1);
}

static void test_one_module_takes_the_budget() {
  Rig g;
  g.m2.muted = true;
  g.s.set(HR_MAX_CURRENT, 80);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.online == 1);
  CHECK_NEAR(g.m1.ilimit, 50.0, 0.1);  // capped per module
  CHECK_NEAR(g.st.commanded_a, 50.0, 0.01);
}

static void test_alarm_fault_and_auto_clear() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(10);
  g.m1.alarm = 1u << 22;
  g.run_s(3);
  CHECK(g.st.state == State::FAULT && g.st.error == 1);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
  g.m1.alarm = 0;
  g.run_s(20);
  CHECK(g.st.state == State::FAULT && g.st.error == 1);  // 30 s after the condition is gone
  g.run_s(15);
  CHECK(g.st.state == State::CHARGING && g.st.error == 0);
}

static void test_clear_fault_at_once() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(5);
  g.m2.alarm = 1u;  // E8
  g.run_s(3);
  CHECK(g.st.error == 8);
  g.m2.alarm = 0;
  g.run_s(3);
  g.ctl.clear_fault();
  g.run_s(1);
  CHECK(g.st.state == State::CHARGING && g.st.error == 0);
}

static void test_overvoltage_e14() {
  Rig g;
  g.m1.battery_v = 59.0f;  // a module reading above HARD_MAX_V + 0.5 V faults whatever the state
  g.run_s(4);
  CHECK(g.st.state == State::FAULT && g.st.error == E_OVERVOLTAGE);
}

static void test_mismatch_e13() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(3);
  g.m2.frozen = true;
  g.s.set(HR_CHARGE_V, 5600);  // module 2 ignores the new voltage
  g.run_s(15);
  CHECK(g.st.state == State::FAULT && g.st.error == E_MISMATCH);
}

static void test_cooldown_before_standby() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(10);
  g.m1.heatsink_c = 50.0f;
  g.run_s(2);
  g.ctl.toggle_pause();
  g.run_s(3);
  CHECK(g.st.state == State::HOLD && g.st.hold == HoldReason::PAUSED);
  CHECK(g.st.cooling);
  CHECK(g.m1.standby == 0 && g.m2.standby == 1);  // module 1 keeps its fans running
  CHECK_NEAR(g.m1.vout_set, 48.0, 0.01);
  CHECK_NEAR(g.m1.ilimit, 1.0, 0.1);
  g.m1.heatsink_c = 39.0f;
  g.run_s(3);
  CHECK(!g.st.cooling && g.m1.standby == 1);
}

static void test_no_current_warning_e16() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(60);
  g.m1.load_connected = g.m2.load_connected = false;  // DC breaker open
  g.run_s(55);
  CHECK(g.st.error == 0);
  g.run_s(10);
  CHECK(g.st.no_current && g.st.error == E_NO_CURRENT && g.st.state == State::CHARGING);
  g.m1.load_connected = g.m2.load_connected = true;
  g.run_s(3);
  CHECK(!g.st.no_current && g.st.error == 0);
}

static void test_session_stops_at_stop_v_with_tail_current() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 4);
  g.ctl.charge_now();
  g.run_s(5);
  g.adc_v = 57.6f;
  g.run_s(12);
  CHECK(!g.st.session_active && g.st.state == State::HOLD && g.st.hold == HoldReason::WAITING);
}

static void test_stored_defaults_follow_settings() {
  Rig g;
  g.s.set(HR_MODULE_FALLBACK_V, 5200);
  g.s.set(HR_MODULE_FALLBACK_A, 3);
  g.run_s(8);
  CHECK_NEAR(g.m1.default_v, 52.0, 0.01);
  CHECK_NEAR(g.m2.default_a, 3.0, 0.1);
}

static void test_both_modules_offline_ramp_restarts() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 20);
  g.ctl.charge_now();
  g.run_s(20);
  CHECK_NEAR(g.st.commanded_a, 20.0, 0.01);
  g.m1.muted = g.m2.muted = true;
  g.run_s(8);
  CHECK(g.st.online == 0 && g.st.commanded_a == 0.0f);
  g.m1.muted = g.m2.muted = false;
  float prev = 0.0f;
  for (int i = 0; i < 12; i++) {
    g.run_s(1);
    if (g.st.online == 2) {
      CHECK(g.st.commanded_a <= prev + 4.01f);  // never jumps back to the budget
      prev = g.st.commanded_a;
    }
  }
  CHECK(prev > 0.0f);
}

// With no SOC, the battery voltage is the only thing that ends a session: a dead ADC must not start one, even
// with Charge now, or the modules would hold Charge V forever.
static void test_nan_adc_no_session_e17() {
  Rig g;
  g.adc_v = NAN;
  g.run_s(80);
  CHECK(!g.st.session_active && g.st.error == E_BATTERY_SENSE && g.st.state == State::HOLD);
  g.ctl.charge_now();
  g.run_s(3);
  CHECK(!g.st.session_active && g.st.state == State::HOLD);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
}

static void test_implausible_battery_v_e17() {
  Rig g;
  g.adc_v = 0.2f;  // sense line open: reads as an empty battery
  g.run_s(10);
  CHECK(!g.st.session_active && g.st.error == E_BATTERY_SENSE && is_fault_code(g.st.error));
  g.ctl.charge_now();
  g.run_s(3);
  CHECK(!g.st.session_active && g.m1.standby == 1 && g.m2.standby == 1);
  g.adc_v = BATTERY_SENSE_MIN_V - 0.1f;
  g.run_s(10);
  CHECK(!g.st.session_active && g.st.error == E_BATTERY_SENSE);
  g.adc_v = 41.0f;  // the first plausible reading counts as the power-up one: below Start V starts at once
  g.run_s(3);
  CHECK(g.st.session_active && g.st.state == State::CHARGING && g.st.error == 0);
}

static void test_sense_lost_mid_session_ends_it() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(10);
  CHECK(g.st.state == State::CHARGING);
  g.adc_v = 0.0f;
  g.run_s(1);
  CHECK(!g.st.session_active && g.st.state == State::HOLD && g.st.error == E_BATTERY_SENSE);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1 && g.m1.ilimit == 0.0f);
  g.adc_v = 50.0f;  // back, between Start V and Stop V: the ended session stays ended
  g.run_s(10);
  CHECK(!g.st.session_active && g.st.error == 0);
}

static void test_rejected_writes_e13() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(3);
  g.m2.reject_writes = true;
  g.run_s(10);
  CHECK(g.st.state == State::FAULT && g.st.error == E_MISMATCH);
}

static void test_overvoltage_e14_target_rule() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 20);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.state == State::CHARGING && g.st.output_a > 1.0f);
  g.m1.frozen = true;
  g.s.set(HR_CHARGE_V, 5400);  // module 1 stays at 57.6 V, more than 0.5 V above the new target
  g.run_s(3);
  CHECK(g.st.state == State::CHARGING);  // the target rule needs 3 s
  g.run_s(3);
  CHECK(g.st.state == State::FAULT);
  CHECK(g.st.error == E_OVERVOLTAGE);
}

static void test_lowering_charge_v_while_charging_no_e14() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 40);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.state == State::CHARGING && g.st.output_a > 19.9f);
  CHECK_NEAR(g.m1.vout(), 57.6, 0.01);
  g.s.set(HR_CHARGE_V, 5500);
  for (int i = 0; i < 10; i++) {
    g.run_s(1);
    CHECK(g.st.state == State::CHARGING && g.st.error == 0);
  }
  CHECK_NEAR(g.m1.vout_set, 55.0, 0.01);  // the modules followed down
  CHECK_NEAR(g.m2.vout_set, 55.0, 0.01);
}

// Settings straight into the controller, past apply_clamps: the hard limits still hold on every frame sent.
static void test_unclamped_settings_hard_limits() {
  Rig g;
  g.s.set(HR_CHARGE_V, 6500);
  g.s.set(HR_MAX_CURRENT, 300);
  g.s.set(HR_MODULE_FALLBACK_V, 6000);
  g.s.set(HR_MODULE_FALLBACK_A, 40);
  g.m1.battery_v = g.m2.battery_v = 52.0f;
  g.ctl.charge_now();
  g.run_s(120);
  CHECK(g.st.state == State::CHARGING);
  int v = 0, a = 0, dv = 0, da = 0;
  for (const auto &f : g.bus.sent) {
    if (((f.id >> 8) & 0xFF) != 0x81) continue;
    const int reg = f.data[0] << 8 | f.data[1], raw = f.data[6] << 8 | f.data[7];
    const long v_max = std::lround(58.00 * 1023), a_max = std::lround(50.0 * 1023 / 52.0);
    const long dv_max = std::lround(54.4 * 1023), da_max = std::lround(10.0 * 1023 / 52.0);
    v += reg == 0x0100;
    a += reg == 0x022D;
    dv += reg == 0x0101;
    da += reg == 0x0104;
    CHECK(reg != 0x0100 || raw <= v_max);
    CHECK(reg != 0x022D || raw <= a_max);
    CHECK(reg != 0x0101 || raw <= dv_max);
    CHECK(reg != 0x0104 || raw <= da_max);
  }
  CHECK(v > 0 && a > 0 && dv > 0 && da > 0);
  CHECK_NEAR(g.m1.vout_set, 58.0, 0.01);  // the limits are reached, not just respected
  CHECK_NEAR(g.m1.ilimit, 50.0, 0.1);
  CHECK_NEAR(g.m2.default_v, 54.4, 0.01);
  CHECK_NEAR(g.m2.default_a, 10.0, 0.1);
}

static void test_fault_replaces_e16() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(60);
  g.m1.load_connected = g.m2.load_connected = false;
  g.run_s(70);
  CHECK(g.st.error == E_NO_CURRENT);
  g.m1.alarm = 1u << 14;  // E5
  g.run_s(3);
  CHECK(g.st.state == State::FAULT && g.st.error == 5);
}

static void test_millis_wraparound_fault() {
  Rig g;
  g.now = 4294960000u;
  g.ctl.charge_now();
  g.run_s(3);
  g.m1.alarm = 1u << 22;
  g.run_s(3);
  CHECK(g.st.state == State::FAULT && g.st.error == 1);
  CHECK(g.now > 4294960000u);  // not yet wrapped
  g.m1.alarm = 0;
  g.run_s(20);
  CHECK(g.now < 100000u);  // wrapped
  CHECK(g.st.state == State::FAULT);
  g.run_s(15);
  CHECK(g.st.state == State::CHARGING && g.st.error == 0);
}

static void test_alarm_code_outranks_mismatch() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(3);
  g.m1.frozen = true;
  g.s.set(HR_CHARGE_V, 5600);
  g.run_s(15);
  CHECK(g.st.state == State::FAULT && g.st.error == E_MISMATCH);
  g.m2.alarm = 1u << 22;
  g.run_s(3);
  CHECK(g.st.state == State::FAULT && g.st.error == 1);
}

static void test_diag_override() {
  Rig g;
  g.ctl.set_paused(true);
  g.run_s(5);
  CHECK(g.st.state == State::HOLD && g.m1.standby == 1 && g.m2.standby == 1);
  // PSU1 only, 0 A, below the battery: out of standby while paused; PSU2 stays in standby.
  g.ctl.start_diag(51.0f, 0.0f, 1, 10, g.now);
  CHECK(g.ctl.diag_remaining_s(g.now) == 10);
  g.run_s(2);
  CHECK(g.st.diag && g.st.state == State::HOLD && g.st.hold == HoldReason::PAUSED);
  CHECK(g.m1.standby == 0 && g.m2.standby == 1);
  CHECK_NEAR(g.m1.vout_set, 51.0, 0.01);
  CHECK_NEAR(g.m1.ilimit, 0.0, 0.01);
  CHECK_NEAR(g.st.target_v, 51.0, 0.01);
  CHECK(g.st.error == 0);  // no E16 at 0 A
  // Both, 1.5 A each.
  g.ctl.start_diag(53.0f, 1.5f, 3, 10, g.now);
  g.run_s(2);
  CHECK(g.m1.standby == 0 && g.m2.standby == 0);
  CHECK_NEAR(g.m2.ilimit, 1.5, 0.05);  // the module's current step
  CHECK_NEAR(g.st.commanded_a, 3.0, 0.01);
  // Timeout: back to standby (paused).
  g.run_s(9);
  CHECK(!g.st.diag && !g.ctl.diag_active() && g.ctl.diag_remaining_s(g.now) == 0);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
  // A pause command ends it.
  g.ctl.start_diag(53.0f, 1.0f, 3, 60, g.now);
  g.run_s(2);
  CHECK(g.m1.standby == 0);
  g.ctl.toggle_pause();
  CHECK(!g.ctl.diag_active());
  g.ctl.set_paused(true);
  g.run_s(2);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
  // A fault ends it.
  g.ctl.start_diag(53.0f, 1.0f, 3, 60, g.now);
  g.run_s(2);
  g.m1.alarm = 1u << 22;
  g.run_s(3);
  CHECK(g.st.state == State::FAULT && !g.ctl.diag_active());
}

static void test_diag_total_capped_by_max_current() {
  Rig g;  // Max current 10 A
  g.ctl.start_diag(53.0f, 8.0f, 3, 60, g.now);  // 16 A asked in all
  g.run_s(2);
  CHECK(g.m1.standby == 0 && g.m2.standby == 0);
  CHECK(g.m1.ilimit <= 5.05f && g.m2.ilimit <= 5.05f);
  CHECK(g.st.commanded_a <= 10.01f);
  g.s.set(HR_MAX_CURRENT, 4);  // lowered while it runs
  g.run_s(2);
  CHECK(g.m1.ilimit <= 2.05f && g.m2.ilimit <= 2.05f);
  g.ctl.start_diag(53.0f, 8.0f, 1, 60, g.now);  // one module: up to the whole Max current
  g.run_s(2);
  CHECK(g.m1.ilimit > 3.9f && g.m1.ilimit <= 4.05f && g.m2.standby == 1);
}

static void test_diag_ends_when_the_battery_says_stop() {
  struct Case {
    void (*apply)(Limits &);
  };
  const Case cases[] = {
      {[](Limits &l) { l.alarm = true; }},
      {[](Limits &l) { l.allowed = false; }},
      {[](Limits &l) { l.blocking = true; }},
      {[](Limits &l) { l.max_cell_v = 3.70f; }},  // above Max cell (3.55 V)
  };
  for (const Case &c : cases) {
    Rig g;
    Limits lim = panel_limits(g.s);
    g.lim = &lim;
    g.ctl.start_diag(53.0f, 1.0f, 3, 60, g.now);
    g.run_s(2);
    CHECK(g.ctl.diag_active() && g.m1.standby == 0);
    c.apply(lim);
    g.run_s(2);
    CHECK(!g.ctl.diag_active() && g.m1.standby == 1 && g.m2.standby == 1);
  }
  Rig g;  // E17: no SOC and an implausible battery reading
  g.ctl.start_diag(53.0f, 1.0f, 3, 60, g.now);
  g.run_s(2);
  CHECK(g.ctl.diag_active());
  g.adc_v = 5.0f;
  g.run_s(2);
  CHECK(!g.ctl.diag_active() && g.m1.standby == 1 && g.m2.standby == 1);
}

static void test_halt_puts_modules_in_standby() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(15);
  CHECK(g.st.state == State::CHARGING && g.m1.standby == 0);
  g.ctl.halt();
  CHECK(g.ctl.halting());
  g.run_s(1);  // the frames go out before the next tick
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
  g.run_s(5);  // and no tick, cooldown or override brings them back
  g.ctl.start_diag(53.0f, 1.0f, 3, 60, g.now);
  g.run_s(2);
  CHECK(!g.ctl.diag_active() && g.m1.standby == 1 && g.m2.standby == 1);
}

static void test_diag_while_charging_then_ramp_restarts() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(15);
  CHECK(g.st.state == State::CHARGING && g.m1.ilimit > 4.9f);
  g.ctl.start_diag(54.0f, 0.5f, 2, 3, g.now);
  g.run_s(2);
  CHECK(g.m1.standby == 1 && g.m2.standby == 0);
  CHECK_NEAR(g.m2.ilimit, 0.5, 0.05);
  g.run_s(2);
  CHECK(!g.st.diag && g.st.state == State::CHARGING);
  CHECK(g.st.commanded_a <= 4.01f);  // the ramp starts again from 0
}

static void test_modules_soft_start_staggered() {
  Rig g;  // ADC 50 V, so both wake at 49 V; the fake modules conduct above 52 V
  g.ctl.charge_now();
  g.run_s(31);  // PSU1: wakes at ~2 s, 12 s hold, then 0.2 V/s from 49 V: conducting from ~29 s
  CHECK(g.m1.iout() > 0.0f);
  CHECK_NEAR(g.m2.vout_set, 49.0, 0.01);  // PSU2 holds until PSU1 has carried current for 2 s
  g.run_s(10);
  CHECK(g.m2.vout_set > 49.5f);
  g.run_s(60);
  CHECK_NEAR(g.m1.vout_set, 57.6, 0.01);
  CHECK_NEAR(g.m2.vout_set, 57.6, 0.01);
}

static void test_soft_start_lead_missing() {
  Rig g;  // PSU1 offline: PSU2 doesn't wait for it
  g.m1.muted = true;
  g.ctl.charge_now();
  g.run_s(20);
  CHECK(g.m2.vout_set > 49.5f);
  Rig h;  // PSU1 running but never conducting: PSU2 rises at the 30 s cap
  h.m1.load_connected = false;
  h.ctl.charge_now();
  h.run_s(29);
  CHECK_NEAR(h.m2.vout_set, 49.0, 0.01);
  h.run_s(5);
  CHECK(h.m2.vout_set > 49.5f);
}

static void test_soft_start_ramps_after_the_hold() {
  SoftStart ss;
  CHECK_NEAR(ss.update(false, 57.6f, 53.0f, 0), 57.6, 0.001);
  CHECK_NEAR(ss.update(true, 57.6f, 53.0f, 1000), 52.0, 0.001);
  CHECK_NEAR(ss.update(true, 57.6f, 55.0f, 1000 + SOFTSTART_HOLD_MS), 52.0, 0.001);  // start fixed at the wake
  CHECK_NEAR(ss.update(true, 57.6f, 53.0f, 1000 + SOFTSTART_HOLD_MS + 5000), 53.0, 0.001);  // 0.2 V/s
  CHECK_NEAR(ss.update(true, 57.6f, 53.0f, 1000 + SOFTSTART_HOLD_MS + 60000), 57.6, 0.001);
  CHECK_NEAR(ss.update(true, 56.0f, 53.0f, 1000 + SOFTSTART_HOLD_MS + 70000), 56.0, 0.001);  // follows a new target
  CHECK_NEAR(ss.update(false, 57.6f, 53.0f, 200000), 57.6, 0.001);
  CHECK_NEAR(ss.update(true, 57.6f, 53.0f, 201000), 52.0, 0.001);  // a new wake starts over
  SoftStart held;  // may_rise false: holds past SOFTSTART_HOLD_MS, rises at SOFTSTART_MAX_HOLD_MS
  held.update(true, 57.6f, 53.0f, 0, false);
  CHECK_NEAR(held.update(true, 57.6f, 53.0f, 20000, false), 52.0, 0.001);
  CHECK_NEAR(held.update(true, 57.6f, 53.0f, 25000, true), 52.0, 0.001);  // released: rises from here
  CHECK_NEAR(held.update(true, 57.6f, 53.0f, 30000, true), 53.0, 0.001);
  SoftStart blind;
  CHECK_NEAR(blind.update(true, 57.6f, NAN, 0), 57.6, 0.001);  // no battery reading: no soft-start
  CHECK_NEAR(blind.update(true, 54.0f, 56.0f, 0), 54.0, 0.001);  // never above the target
}

static void test_stale_source_e15_and_fields() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.error == 0 && !g.st.stale && g.st.source == 0);
  Limits lim = panel_limits(g.s);
  lim.stale = true;
  lim.source = SourceMode::PYLONTECH;
  lim.ccl = 4.0f;  // as PylonSource would cap it
  for (int i = 0; i < 3; i++) {
    for (int t = 0; t < 1000; t++, g.now++) {
      g.r1.loop(g.now);
      g.r2.loop(g.now);
      g.bus.deliver({&g.m1, &g.m2}, {&g.r1, &g.r2}, g.now);
      if (g.now % CONTROLLER_TICK_MS == 0) g.st = g.ctl.tick(g.s, lim, g.adc_v, g.now);
    }
  }
  CHECK(g.st.error == E_SOURCE_STALE && g.st.stale && g.st.source == 1);
  CHECK(g.st.state == State::CHARGING);  // a warning: charging carries on within the fallback cap
  CHECK(g.st.budget <= 4.01f);
}

// An EG4 block pair pushed into a real Eg4Source: block A (pack 53.20 V), block B (CCL 30 A, SOC 75, protection p).
static void eg4_frame(uint8_t *f, uint16_t count, uint16_t reg, uint16_t v) {  // one non-zero register, then CRC
  for (int i = 0; i < 3 + 2 * count + 2; i++) f[i] = 0;
  f[0] = 0x10;
  f[1] = 0x03;
  f[2] = uint8_t(2 * count);
  f[3 + 2 * reg] = uint8_t(v >> 8);
  f[4 + 2 * reg] = uint8_t(v);
  const size_t n = 3 + 2 * count;
  const uint16_t crc = crc16_modbus(f, n);
  f[n] = uint8_t(crc);
  f[n + 1] = uint8_t(crc >> 8);
}

// One poll and its reply (block A or B, whichever the source asks for), the reply arriving in the same ms.
static void eg4_exchange(Eg4Source &src, uint32_t now, uint16_t prot) {
  uint8_t req[POLL_REQ_MAX], f[eg4::MAX_REPLY];
  CHECK(src.poll(now, req) == 8);
  if (req[3] == 0) {  // block A starts at register 0
    eg4_frame(f, eg4::A_COUNT, 0, 5320);
    src.push(f, eg4::reply_len(eg4::A_COUNT), now);
  } else {
    eg4_frame(f, eg4::B_COUNT, 0x1B - 0x12, prot);
    f[3 + 2 * (0x16 - 0x12) + 1] = 30;  // CCL 30 A
    f[3 + 2 * (0x18 - 0x12) + 1] = 75;  // SOC 75, then re-seal
    const size_t n = 3 + 2 * eg4::B_COUNT;
    const uint16_t crc = crc16_modbus(f, n);
    f[n] = uint8_t(crc);
    f[n + 1] = uint8_t(crc >> 8);
    src.push(f, eg4::reply_len(eg4::B_COUNT), now);
  }
}

static void test_eg4_source_through_controller() {
  Rig g;
  Eg4Source src;
  g.ctl.charge_now();
  auto run = [&](int seconds, uint16_t prot, bool feed) {
    for (int t = 0; t < seconds * 1000; t++, g.now++) {
      g.r1.loop(g.now);
      g.r2.loop(g.now);
      g.bus.deliver({&g.m1, &g.m2}, {&g.r1, &g.r2}, g.now);
      if (feed && g.now % 500 == 0) eg4_exchange(src, g.now, prot);
      if (g.now % CONTROLLER_TICK_MS == 0) g.st = g.ctl.tick(g.s, src.limits(g.s, g.now), g.adc_v, g.now);
    }
  };
  g.now = 1000;
  run(5, 0, true);
  const Limits l = src.limits(g.s, g.now);
  CHECK(l.fresh && !l.alarm && l.source == SourceMode::EG4 && l.ccl == 30.0f && l.soc == 75);
  CHECK(g.st.error == 0 && g.st.source == 3 && !g.st.stale);
  run(5, 0x0001, true);  // pack OV protection bit
  CHECK(g.st.error == E_BMS_ALARM);
  run(5, 0, true);  // alarm gone: the stop latch releases on the next fresh clean state
  CHECK(g.st.error == 0);
  run(5, 0, false);  // the BMS goes silent: fresh for 3 s, then stale
  CHECK(g.st.error == E_SOURCE_STALE && g.st.stale);
}

// A JK-PB source fed records: poll() asks for settings first, then cell info; the reply is a record plus its trailer.
static void jkpb_put_u32(uint8_t *r, size_t i, uint32_t v) {
  for (int k = 0; k < 4; k++) r[i + k] = uint8_t(v >> (8 * k));
}

static void jkpb_reply(JkpbSource &src, uint32_t now, uint32_t alarms) {
  uint8_t req[POLL_REQ_MAX], u[oem::jkpb::RECORD_LEN + oem::jkpb::TRAILER_LEN] = {};
  CHECK(src.poll(now, req) == oem::jkpb::REQUEST_LEN);
  const uint16_t reg = uint16_t(req[2] << 8 | req[3]);
  const bool cell = reg == oem::jkpb::REG_CELL_INFO;
  u[0] = 0x55; u[1] = 0xAA; u[2] = 0xEB; u[3] = 0x90;
  u[4] = cell ? 0x02 : 0x01;
  if (cell) {
    for (int n = 0; n < 16; n++) { u[6 + 2 * n] = 0x0C; u[7 + 2 * n] = 0xE4; }  // 0x0CE4 = 3300 mV
    jkpb_put_u32(u, 150, 52800);
    jkpb_put_u32(u, 166, alarms);
    u[173] = 75;
    u[198] = 1;
  } else {
    jkpb_put_u32(u, 38, 3600);
    jkpb_put_u32(u, 42, 3500);
    jkpb_put_u32(u, 50, 30000);
    jkpb_put_u32(u, 114, 16);
    jkpb_put_u32(u, 118, 1);
  }
  uint8_t sum = 0;
  for (size_t i = 0; i < oem::jkpb::RECORD_LEN - 1; i++) sum = uint8_t(sum + u[i]);
  u[oem::jkpb::RECORD_LEN - 1] = sum;
  uint8_t *t = u + oem::jkpb::RECORD_LEN;
  const uint8_t b[6] = {req[0], 0x10, req[2], req[3], 0x00, 0x01};
  for (int i = 0; i < 6; i++) t[i] = b[i];
  const uint16_t crc = crc16_modbus(t, 6);
  t[6] = uint8_t(crc);
  t[7] = uint8_t(crc >> 8);
  src.push(u, sizeof u, now);
}

static void test_jkpb_source_through_controller() {
  Rig g;
  JkpbSource src;
  g.ctl.charge_now();
  auto run = [&](int seconds, uint32_t alarms, bool feed) {
    for (int t = 0; t < seconds * 1000; t++, g.now++) {
      g.r1.loop(g.now);
      g.r2.loop(g.now);
      g.bus.deliver({&g.m1, &g.m2}, {&g.r1, &g.r2}, g.now);
      if (feed && g.now % 1000 == 0) jkpb_reply(src, g.now, alarms);
      if (g.now % CONTROLLER_TICK_MS == 0) g.st = g.ctl.tick(g.s, src.limits(g.s, g.now), g.adc_v, g.now);
    }
  };
  g.now = 1000;
  run(8, 0, true);
  const Limits l = src.limits(g.s, g.now);
  CHECK(src.locked() && l.fresh && !l.alarm && l.source == SourceMode::JKPB && l.ccl == 30.0f && l.soc == 75);
  CHECK(g.st.error == 0 && g.st.source == 5 && !g.st.stale);
  run(5, 0x00000001, true);  // a warning bit in the alarm mask
  CHECK(g.st.error == E_BMS_ALARM);
  run(5, 0, true);  // alarm gone: the stop latch releases on the next fresh clean state
  CHECK(g.st.error == 0);
  run(5, 0, false);  // the BMS goes silent: fresh for 3 s, then stale
  CHECK(g.st.error == E_SOURCE_STALE && g.st.stale);
}

static void test_end_session_on_mode_change() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(3);
  CHECK(g.st.session_active && g.st.state == State::CHARGING);
  g.ctl.end_session();  // main.cpp: a Source mode change
  g.run_s(1);
  CHECK(!g.st.session_active);
  g.run_s(15);  // 50 V is between Start V and Stop V: nothing starts a new one
  CHECK(!g.st.session_active && g.st.state == State::HOLD && g.st.hold == HoldReason::WAITING);
}

int main() {
  test_alarm_codes();
  test_eg4_source_through_controller();
  test_jkpb_source_through_controller();
  test_waits_for_start_then_charge_now();
  test_low_battery_starts_session_at_boot();
  test_end_session_on_mode_change();
  test_split_cap_and_low_line();
  test_one_module_takes_the_budget();
  test_alarm_fault_and_auto_clear();
  test_clear_fault_at_once();
  test_overvoltage_e14();
  test_mismatch_e13();
  test_cooldown_before_standby();
  test_no_current_warning_e16();
  test_session_stops_at_stop_v_with_tail_current();
  test_stored_defaults_follow_settings();
  test_both_modules_offline_ramp_restarts();
  test_nan_adc_no_session_e17();
  test_implausible_battery_v_e17();
  test_sense_lost_mid_session_ends_it();
  test_rejected_writes_e13();
  test_overvoltage_e14_target_rule();
  test_lowering_charge_v_while_charging_no_e14();
  test_unclamped_settings_hard_limits();
  test_fault_replaces_e16();
  test_millis_wraparound_fault();
  test_alarm_code_outranks_mismatch();
  test_diag_override();
  test_stale_source_e15_and_fields();
  test_soft_start_ramps_after_the_hold();
  test_modules_soft_start_staggered();
  test_soft_start_lead_missing();
  test_diag_while_charging_then_ramp_restarts();
  test_diag_total_capped_by_max_current();
  test_diag_ends_when_the_battery_says_stop();
  test_halt_puts_modules_in_standby();
  return test_result("test_oem_controller");
}
