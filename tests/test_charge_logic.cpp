#include "check.h"
#include "charge_controller/charge_logic.h"

using charge::Topic;

// Feeds one full YamBMS snapshot at time t (each topic's first message counts as retained).
static void feed_all(charge::BmsTracker &b, uint32_t t) {
  b.on_message(Topic::STATUS, "online", t);
  b.on_message(Topic::CVL, "57.0", t);
  b.on_message(Topic::CCL, "275", t);
  b.on_message(Topic::ALLOWED, "ON", t);
  b.on_message(Topic::ALARM, "0", t);
  b.on_message(Topic::CURRENT, "237.8", t);
}

static void test_retained_only_is_not_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 1000);  // retained replay on subscribe
  CHECK(!b.fresh(1000));
  CHECK(b.online());
  CHECK(b.allowed());
  CHECK_NEAR(b.cvl(), 57.0, 1e-6);
}

static void test_second_message_is_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 1000);
  b.on_message(Topic::CVL, "57.0", 10000);
  b.on_message(Topic::CCL, "275", 10000);
  CHECK(!b.fresh(10000));  // current not yet updated live
  b.on_message(Topic::CURRENT, "238.0", 11000);
  CHECK(b.fresh(11000));
  CHECK_NEAR(b.current(), 238.0, 1e-6);
  CHECK(b.age_ms(11000) == 1000);  // CVL/CCL last updated at 10000
}

static void test_goes_stale_after_60s() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  CHECK(b.fresh(61000));
  CHECK(!b.fresh(61001));
}

static void test_on_message_reports_live_updates() {
  charge::BmsTracker b;
  b.on_reconnect();
  CHECK(!b.on_message(Topic::CURRENT, "10", 0));    // retained replay: stored, not counted
  CHECK(b.on_message(Topic::CURRENT, "12", 1000));  // live
  CHECK(!b.on_message(Topic::CURRENT, "junk", 2000));
  b.on_reconnect();
  CHECK(!b.on_message(Topic::CURRENT, "11", 3000));  // replay after reconnect
  CHECK(b.on_message(Topic::CURRENT, "13", 4000));
}

static void test_stale_does_not_wrap_back_to_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  CHECK(!b.fresh(61001));
  CHECK(!b.fresh(uint32_t(1000ull + (1ull << 32) - 10)));
  CHECK(!b.fresh(uint32_t(1000ull + (1ull << 32) + 10)));  // millis() wrapped: age would read 10 ms
  feed_all(b, 1020);  // a new live update makes it fresh again
  CHECK(b.fresh(1030));
}

static void test_reconnect_requires_live_update_again() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  CHECK(b.fresh(2000));
  b.on_reconnect();
  feed_all(b, 3000);    // retained replay after reconnect: values stored...
  CHECK(b.fresh(3000)); // ...and the earlier live updates (t=1000) still count until they age out
  CHECK(!b.fresh(61001));
  feed_all(b, 62000);
  CHECK(b.fresh(62000));
}

static void test_status_and_flags() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  b.on_message(Topic::STATUS, "offline", 2000);
  CHECK(!b.online());
  CHECK(!b.fresh(2000));
  b.on_message(Topic::STATUS, "online", 3000);
  b.on_message(Topic::ALLOWED, "OFF", 3000);
  CHECK(!b.allowed());
  CHECK(b.fresh(3000));  // freshness doesn't depend on the allowed value
  b.on_message(Topic::ALARM, "1", 3000);
  CHECK(b.alarm());
}

static void test_garbage_is_ignored() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  b.on_message(Topic::CVL, "unavailable", 2000);
  b.on_message(Topic::CVL, "nan", 2000);
  b.on_message(Topic::CVL, "", 2000);
  b.on_message(Topic::ALLOWED, "maybe", 2000);
  CHECK_NEAR(b.cvl(), 57.0, 1e-6);
  CHECK(b.allowed());
  CHECK(!b.fresh(61001));  // ignored payloads don't refresh
}

static void test_missing_flags_not_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  for (uint32_t t : {0u, 1000u}) {
    b.on_message(Topic::STATUS, "online", t);
    b.on_message(Topic::CVL, "57", t);
    b.on_message(Topic::CCL, "275", t);
    b.on_message(Topic::CURRENT, "0", t);
  }
  CHECK(!b.fresh(1000));  // no charging_allowed / bms_in_alarm value yet
  CHECK(b.age_ms(1000) == 0);
  charge::BmsTracker never;
  CHECK(never.age_ms(5000) == UINT32_MAX);
}

static void test_target_voltage() {
  CHECK_NEAR(charge::target_voltage(57.0f, 57.0f), 57.0, 1e-6);
  CHECK_NEAR(charge::target_voltage(57.0f, 55.5f), 55.5, 1e-6);
  CHECK_NEAR(charge::target_voltage(60.0f, 60.0f), 58.0, 1e-6);  // hard cap
  CHECK(std::isnan(charge::target_voltage(NAN, 57.0f)));
}

static void test_current_budget() {
  // Inverter pushes 238 A, CCL 275, we push nothing: 37 A headroom.
  CHECK_NEAR(charge::current_budget(275, 238, 0, 100), 37, 1e-4);
  // We're now pushing 37 A, so the battery sees 275: the budget stays at 37 (no oscillation).
  CHECK_NEAR(charge::current_budget(275, 275, 37, 100), 37, 1e-4);
  // Loads draining the battery: the budget is capped by the user max.
  CHECK_NEAR(charge::current_budget(275, -50, 0, 100), 100, 1e-4);
  // A load drawing from the battery never inflates the budget past the CCL...
  CHECK_NEAR(charge::current_budget(30, -50, 0, 100), 30, 1e-4);
  // ...and a CCL of 0 means no charging at all, whatever the load.
  CHECK_NEAR(charge::current_budget(0, -20, 0, 100), 0, 1e-6);
  CHECK_NEAR(charge::current_budget(-5, -20, 0, 100), 0, 1e-6);
  // Over the CCL: no budget, never negative.
  CHECK_NEAR(charge::current_budget(100, 150, 0, 100), 0, 1e-6);
  // Missing data is never treated as unlimited.
  CHECK_NEAR(charge::current_budget(275, NAN, 0, 100), 0, 1e-6);
  CHECK_NEAR(charge::current_budget(NAN, 0, 0, 100), 0, 1e-6);
}

static void test_moving_average() {
  charge::MovingAverage avg(10000);
  CHECK(std::isnan(avg.value()));
  avg.add(10, 0);
  avg.add(20, 5000);
  CHECK_NEAR(avg.value(), 15, 1e-6);
  avg.add(30, 12000);  // the t=0 sample drops out of the 10 s window
  CHECK_NEAR(avg.value(), 25, 1e-6);
  avg.add(NAN, 13000);  // ignored
  CHECK_NEAR(avg.value(), 25, 1e-6);
  avg.reset();
  CHECK(std::isnan(avg.value()));
}

static void test_ramp() {
  charge::RampLimiter r;
  CHECK_NEAR(r.step(10, 0), 0, 1e-6);
  CHECK_NEAR(r.step(10, 1000), 2, 1e-6);
  CHECK_NEAR(r.step(10, 2000), 4, 1e-6);
  CHECK_NEAR(r.step(1, 2500), 1, 1e-6);   // drops immediately
  CHECK_NEAR(r.step(10, 3500), 3, 1e-6);
  r.reset();
  CHECK_NEAR(r.step(10, 60000), 0, 1e-6); // reset restarts from 0 without a time jump
  CHECK_NEAR(r.step(10, 60500), 1, 1e-6);
  charge::RampLimiter gap;
  gap.step(10, 0);
  CHECK_NEAR(gap.step(10, 100000), 2, 1e-6);  // a 100 s gap still only ramps 1 s worth
}

static void test_split_and_overvoltage() {
  CHECK_NEAR(charge::per_psu_current(37, 2), 18.5, 1e-6);
  CHECK_NEAR(charge::per_psu_current(150, 2), 50, 1e-6);
  CHECK_NEAR(charge::per_psu_current(37, 0), 0, 1e-6);
  CHECK_NEAR(charge::per_psu_current(-3, 2), 0, 1e-6);
  CHECK(!charge::overvoltage(true, 57.4f, 57.0f, 10.0f));
  CHECK(charge::overvoltage(true, 57.6f, 57.0f, 10.0f));
  CHECK(!charge::overvoltage(true, 57.6f, 54.0f, 0.3f));  // CVL just dropped to float: battery still high, ~0 A
  CHECK(charge::overvoltage(true, 57.6f, 54.0f, 5.0f));    // ...but pushing current above target is a fault
  CHECK(!charge::overvoltage(false, 57.6f, 57.0f, 0.0f));  // in standby the reading is just the battery
  CHECK(charge::overvoltage(false, 58.6f, 57.0f, 0.0f));   // above the hard cap is always a fault
  CHECK(!charge::overvoltage(true, NAN, 57.0f, 10.0f));
}

static charge::Inputs good() {
  charge::Inputs in;
  in.boot_done = true;
  in.bms_fresh = true;
  in.allowed = true;
  in.budget = 37.0f;
  return in;
}

static void test_boot_and_wait() {
  charge::StateMachine sm;
  CHECK(sm.state() == charge::State::BOOT);
  charge::Inputs in = good();
  in.boot_done = false;
  CHECK(sm.update(in, 0) == charge::State::BOOT);
  in = good();
  in.bms_fresh = false;
  CHECK(sm.update(in, 100) == charge::State::WAIT_BMS);
  CHECK(sm.update(good(), 200) == charge::State::CHARGING);
  CHECK(std::string(charge::state_name(charge::State::WAIT_BMS)) == "WAIT_BMS");
}

static void test_hold_reasons() {
  charge::StateMachine sm;
  sm.update(good(), 0);
  charge::Inputs in = good();
  in.paused = true;
  CHECK(sm.update(in, 1) == charge::State::HOLD);
  in = good();
  in.allowed = false;
  CHECK(sm.update(in, 2) == charge::State::HOLD);
  in = good();
  in.bms_alarm = true;
  CHECK(sm.update(in, 3) == charge::State::HOLD);
  in = good();
  in.budget = 0.0f;
  CHECK(sm.update(in, 4) == charge::State::HOLD);
  in = good();
  in.bms_blocking = true;
  CHECK(sm.update(in, 5) == charge::State::HOLD);
  in = good();
  in.cell_high = true;
  CHECK(sm.update(in, 6) == charge::State::HOLD);
  CHECK(sm.update(good(), 7) == charge::State::CHARGING);
}

static void test_hold_reason_text() {
  using charge::hold_reason;
  charge::Inputs in = good();
  CHECK(std::string(hold_reason(in)) == "");
  in.budget = 0.0f;
  CHECK(std::string(hold_reason(in)) == "current budget below 2 A");
  in.budget = NAN;
  CHECK(std::string(hold_reason(in)) == "current budget below 2 A");
  in.budget = 1.0f;  // start hysteresis: HOLD until the budget reaches START_BUDGET_A
  CHECK(std::string(hold_reason(in)) == "current budget below 2 A");
  in.budget = 2.0f;
  CHECK(std::string(hold_reason(in)) == "");
  in.budget = 0.0f;
  in.cell_high = true;
  CHECK(std::string(hold_reason(in)) == "cell voltage high");
  in.bms_blocking = true;
  CHECK(std::string(hold_reason(in)) == "BMS blocking charge");
  in.bms_alarm = true;
  CHECK(std::string(hold_reason(in)) == "BMS alarm");
  in.allowed = false;
  CHECK(std::string(hold_reason(in)) == "BMS not allowing charge");
  in.paused = true;
  CHECK(std::string(hold_reason(in)) == "paused");
  in = good();
  in.cell_high = true;
  CHECK(std::string(hold_reason(in)) == "cell voltage high");
  in = good();
  in.bms_blocking = true;
  CHECK(std::string(hold_reason(in)) == "BMS blocking charge");
}

static void test_start_hysteresis() {
  charge::StateMachine sm;
  charge::Inputs in = good();
  in.budget = 1.0f;
  CHECK(sm.update(in, 0) == charge::State::HOLD);      // needs >= 2 A to start
  in.budget = 2.0f;
  CHECK(sm.update(in, 1) == charge::State::CHARGING);
  in.budget = 0.5f;
  CHECK(sm.update(in, 2) == charge::State::CHARGING);  // keeps going while > 0
  in.budget = 0.0f;
  CHECK(sm.update(in, 3) == charge::State::HOLD);
}

static void test_fault_latch() {
  charge::StateMachine sm;
  sm.update(good(), 0);
  charge::Inputs in = good();
  in.fault = "PSU 1 fault";
  CHECK(sm.update(in, 1000) == charge::State::FAULT);
  CHECK(std::string(sm.last_fault()) == "PSU 1 fault");
  CHECK(sm.update(good(), 2000) == charge::State::FAULT);    // clear timer starts at 2000
  CHECK(sm.update(good(), 31999) == charge::State::FAULT);
  CHECK(sm.update(good(), 32000) == charge::State::CHARGING);
  CHECK(std::string(sm.last_fault()) == "PSU 1 fault");                   // kept for HA after recovery
}

static void test_fault_reappearing_restarts_timer() {
  charge::StateMachine sm;
  charge::Inputs bad = good();
  bad.fault = "Overvoltage";
  sm.update(bad, 0);
  sm.update(good(), 1000);
  sm.update(bad, 10000);
  CHECK(sm.update(good(), 11000) == charge::State::FAULT);
  CHECK(sm.update(good(), 40999) == charge::State::FAULT);
  CHECK(sm.update(good(), 41000) == charge::State::CHARGING);
}

static void test_clear_button() {
  charge::StateMachine sm;
  charge::Inputs bad = good();
  bad.fault = "PSU 2 fault";
  sm.update(bad, 0);
  bad.clear_fault = true;
  CHECK(sm.update(bad, 500) == charge::State::FAULT);  // can't clear while the condition is present
  charge::Inputs in = good();
  in.clear_fault = true;
  CHECK(sm.update(in, 1000) == charge::State::CHARGING);
}

static void test_fault_during_boot_returns_to_boot() {
  charge::StateMachine sm;
  charge::Inputs in = good();
  in.boot_done = false;
  in.fault = "PSU 1 fault";
  CHECK(sm.update(in, 0) == charge::State::FAULT);
  in.fault = nullptr;
  in.clear_fault = true;
  CHECK(sm.update(in, 100) == charge::State::BOOT);
}

static void test_line_limited_max() {
  CHECK_NEAR(charge::LOW_LINE_V, 180.0, 1e-6);
  CHECK_NEAR(charge::LOW_LINE_MAX_A, 60.0, 1e-6);
  CHECK_NEAR(charge::line_limited_max(100.0f, 230.0f), 100.0, 1e-6);  // normal mains: unchanged
  CHECK_NEAR(charge::line_limited_max(100.0f, 180.0f), 100.0, 1e-6);  // only below 180 V
  CHECK_NEAR(charge::line_limited_max(100.0f, 179.9f), 60.0, 1e-6);   // low line: 60 A total
  CHECK_NEAR(charge::line_limited_max(40.0f, 150.0f), 40.0, 1e-6);    // never raises the user max
  CHECK_NEAR(charge::line_limited_max(100.0f, NAN), 100.0, 1e-6);     // unknown AC: unchanged
  CHECK(std::isnan(charge::line_limited_max(NAN, 150.0f)));          // NaN user max stays NaN
}

static void test_blocking_and_max_cell() {
  CHECK(charge::TOPIC_COUNT == 9);
  charge::BmsTracker b;
  b.on_reconnect();
  CHECK(!b.blocking());                 // never received
  CHECK(std::isnan(b.max_cell()));
  CHECK(!b.on_message(Topic::BLOCKING, "0", 0));  // retained replay: stored, not live
  CHECK(!b.blocking());
  CHECK(b.on_message(Topic::BLOCKING, "1", 1000));
  CHECK(b.blocking());
  CHECK(!b.on_message(Topic::BLOCKING, "junk", 2000));
  CHECK(b.blocking());                  // garbage leaves the value
  CHECK(b.on_message(Topic::BLOCKING, "0", 3000));
  CHECK(!b.blocking());
  CHECK(!b.on_message(Topic::MAX_CELL, "3.335", 0));
  CHECK_NEAR(b.max_cell(), 3.335, 1e-6);
  CHECK(b.on_message(Topic::MAX_CELL, "3.401", 1000));
  CHECK_NEAR(b.max_cell(), 3.401, 1e-6);
}

static void test_new_topics_not_required_for_freshness() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  CHECK(b.fresh(2000));  // no blocking / max cell ever received
  b.on_message(Topic::MAX_CELL, "3.3", 1000);
  b.on_message(Topic::MAX_CELL, "3.3", 2000);
  b.on_message(Topic::BLOCKING, "0", 1000);
  b.on_message(Topic::BLOCKING, "0", 2000);
  CHECK(b.age_ms(2000) == 1000);  // they don't affect the age either
  CHECK(!b.fresh(61001));         // and don't keep it fresh
}

static void test_cell_limiter() {
  CHECK_NEAR(charge::CELL_LIMIT_V, 3.55, 1e-6);
  CHECK_NEAR(charge::CELL_RESUME_V, 3.40, 1e-6);
  charge::CellLimiter c;
  CHECK(!c.high());
  CHECK(!c.update(3.40f, 3.55f, 3.40f));
  CHECK(!c.update(3.549f, 3.55f, 3.40f));
  CHECK(c.update(3.55f, 3.55f, 3.40f));   // latches at the limit
  CHECK(c.high());
  CHECK(c.update(3.45f, 3.55f, 3.40f));   // stays latched between resume and limit
  CHECK(c.update(NAN, 3.55f, 3.40f));     // unknown cell voltage: unchanged
  CHECK(c.update(3.30f, NAN, 3.40f));     // invalid thresholds: unchanged
  CHECK(c.update(3.30f, 3.55f, INFINITY));
  CHECK(c.update(3.30f, 3.40f, 3.40f));   // resume >= limit: unchanged
  CHECK(c.update(3.30f, 3.40f, 3.50f));
  CHECK(!c.update(3.40f, 3.55f, 3.40f));  // releases at the resume voltage
  CHECK(!c.high());
  CHECK(!c.update(3.50f, 3.55f, 3.40f));  // and stays released below the limit
  CHECK(!c.update(3.70f, 3.40f, 3.50f));  // invalid thresholds don't latch either
  CHECK(!c.update(NAN, 3.55f, 3.40f));
}

static void test_cell_thresholds() {
  charge::CellThresholds t = charge::cell_thresholds(3.57f, 3.45f);
  CHECK(!t.fallback);
  CHECK_NEAR(t.limit, 3.57, 1e-6);
  CHECK_NEAR(t.resume, 3.45, 1e-6);
  // Any invalid input falls back to the defaults for both thresholds.
  for (auto p : {std::make_pair(NAN, 3.40f), std::make_pair(3.55f, NAN), std::make_pair(INFINITY, 3.40f),
                 std::make_pair(3.40f, 3.40f), std::make_pair(3.45f, 3.50f), std::make_pair(3.65f, 3.40f)}) {
    t = charge::cell_thresholds(p.first, p.second);
    CHECK(t.fallback);
    CHECK_NEAR(t.limit, charge::CELL_LIMIT_V, 1e-6);
    CHECK_NEAR(t.resume, charge::CELL_RESUME_V, 1e-6);
  }
  CHECK_NEAR(charge::CELL_LIMIT_MAX_V, 3.58, 1e-6);
  CHECK(!charge::cell_thresholds(3.58f, 3.40f).fallback);  // up to the cap is allowed
  CHECK(charge::cell_thresholds(3.581f, 3.40f).fallback);  // above it: below the 3.600 V BMS trip
    // With fallback thresholds the limiter still cuts off.
  charge::CellLimiter c;
  t = charge::cell_thresholds(3.40f, 3.40f);
  CHECK(c.update(3.56f, t.limit, t.resume));
}

static void test_cell_limiter_ignores_implausible() {
  charge::CellLimiter c;
  CHECK(!c.update(4.6f, 3.55f, 3.40f));   // > 4.5 V: garbage, not a cell voltage
  CHECK(!c.update(INFINITY, 3.55f, 3.40f));
  CHECK(c.update(3.56f, 3.55f, 3.40f));
  CHECK(c.update(1.9f, 3.55f, 3.40f));    // < 2.0 V: ignored, doesn't release
  CHECK(c.update(0.0f, 3.55f, 3.40f));
  CHECK(c.update(-INFINITY, 3.55f, 3.40f));
  CHECK(!c.update(2.0f, 3.55f, 3.40f));   // 2.0 V is plausible
  CHECK(c.update(4.5f, 3.55f, 3.40f));    // so is 4.5 V
}

static void test_no_current() {
  using charge::no_current_condition;
  CHECK(no_current_condition(true, 50.0f, 0.0f, 56.9f, 57.0f, 53.2f));    // the real case: DC side open
  CHECK(!no_current_condition(false, 50.0f, 0.0f, 56.9f, 57.0f, 53.2f));  // not charging
  CHECK(!no_current_condition(true, 1.0f, 0.0f, 56.9f, 57.0f, 53.2f));    // hardly anything commanded (ramp start)
  CHECK(!no_current_condition(true, 50.0f, 5.0f, 56.9f, 57.0f, 53.2f));   // current is flowing
  CHECK(!no_current_condition(true, 50.0f, 0.0f, 53.3f, 57.0f, 53.2f));   // modules pulled down to the battery
  CHECK(!no_current_condition(true, 50.0f, 0.0f, 57.0f, 57.0f, 56.5f));   // full battery at its target
  CHECK(!no_current_condition(true, 50.0f, 0.0f, 56.9f, 57.0f, NAN));     // battery voltage unknown
  CHECK(!no_current_condition(true, 50.0f, NAN, 56.9f, 57.0f, 53.2f));
  CHECK(!no_current_condition(true, 50.0f, 0.0f, NAN, 57.0f, 53.2f));

  charge::NoCurrentDetector d;
  CHECK(!d.update(true, 1000));
  CHECK(!d.update(true, 1000 + charge::NO_CURRENT_MS - 1));
  CHECK(d.update(true, 1000 + charge::NO_CURRENT_MS));
  CHECK(!d.update(false, 2000 + charge::NO_CURRENT_MS));                   // clears at once
  CHECK(!d.update(true, 3000 + charge::NO_CURRENT_MS));                    // and times again from scratch
  CHECK(d.update(true, 3000 + 2 * charge::NO_CURRENT_MS));
}

static void test_battery_voltage() {
  charge::BmsTracker b;
  b.on_reconnect();
  CHECK(std::isnan(b.battery_voltage(0)));
  b.on_message(Topic::BATTERY_V, "53.18", 1000);     // retained replay: not trusted
  CHECK(std::isnan(b.battery_voltage(1000)));
  b.on_message(Topic::BATTERY_V, "53.20", 2000);     // live
  CHECK(b.battery_voltage(2000) == 53.20f);
  CHECK(b.battery_voltage(2000 + charge::FRESH_MS) == 53.20f);
  CHECK(std::isnan(b.battery_voltage(2001 + charge::FRESH_MS)));  // stale
}

static void test_cooldown() {
  charge::Cooldown c;
  CHECK(!c.update(false, true, 60.0f, 0));      // never ran (e.g. after boot): no cooldown even if hot
  CHECK(!c.update(true, true, 60.0f, 1000));    // running
  CHECK(c.update(false, true, 50.0f, 2000));    // stops hot: cools
  CHECK(c.update(false, true, 40.5f, 3000));
  CHECK(!c.update(false, true, 40.0f, 4000));   // at 40 °C: standby
  CHECK(!c.update(false, true, 50.0f, 5000));   // doesn't re-arm without running again

  charge::Cooldown cool;
  CHECK(!cool.update(true, true, 30.0f, 0));
  CHECK(!cool.update(false, true, 44.9f, 1000));  // stops below 45 °C: straight to standby

  charge::Cooldown t;
  t.update(true, true, 60.0f, 0);
  CHECK(t.update(false, true, 60.0f, 1000));
  CHECK(t.update(false, true, 60.0f, 1000 + charge::COOLDOWN_MAX_MS - 1));
  CHECK(!t.update(false, true, 60.0f, 1000 + charge::COOLDOWN_MAX_MS));  // 10 min cap
  CHECK(!t.update(false, true, 60.0f, 2000 + charge::COOLDOWN_MAX_MS));

  charge::Cooldown na;
  na.update(true, true, 60.0f, 0);
  CHECK(!na.update(false, false, 60.0f, 1000));  // not allowed at the stop: standby
  CHECK(!na.update(false, true, 60.0f, 2000));   // and it doesn't start later

  charge::Cooldown lost;
  lost.update(true, true, 60.0f, 0);
  CHECK(lost.update(false, true, 60.0f, 1000));
  CHECK(!lost.update(false, false, 60.0f, 2000));  // allowed drops (fault, offline, low battery): standby
  CHECK(!lost.update(false, true, 60.0f, 3000));

  charge::Cooldown nan;
  nan.update(true, true, 60.0f, 0);
  CHECK(!nan.update(false, true, NAN, 1000));    // unknown temperature never starts cooling
  charge::Cooldown nan2;
  nan2.update(true, true, 60.0f, 0);
  CHECK(nan2.update(false, true, 60.0f, 1000));
  CHECK(!nan2.update(false, true, NAN, 2000));   // and ends it

  charge::Cooldown resume;
  resume.update(true, true, 60.0f, 0);
  CHECK(resume.update(false, true, 60.0f, 1000));
  CHECK(!resume.update(true, true, 60.0f, 2000));  // charging resumes: no longer cooling
  CHECK(resume.update(false, true, 60.0f, 3000));  // stops hot again: cools again
}

static void test_cell_limiter_release_needs_live() {
  charge::CellLimiter c;
  CHECK(c.update(3.60f, 3.55f, 3.40f, false));   // a retained value can set the latch...
  CHECK(c.update(3.30f, 3.55f, 3.40f, false));   // ...but not release it
  CHECK(!c.update(3.30f, 3.55f, 3.40f, true));   // a live value releases
  CHECK(!c.update(3.30f, 3.55f, 3.40f));         // live defaults to true
}

static void test_max_cell_live_and_age() {
  charge::BmsTracker b;
  b.on_reconnect();
  CHECK(!b.max_cell_live());
  CHECK(b.topic_age_ms(Topic::MAX_CELL, 5000) == UINT32_MAX);  // never updated live
  b.on_message(Topic::MAX_CELL, "3.30", 1000);  // retained
  CHECK(!b.max_cell_live());
  CHECK(b.topic_age_ms(Topic::MAX_CELL, 5000) == UINT32_MAX);
  b.on_message(Topic::MAX_CELL, "3.31", 2000);
  CHECK(b.max_cell_live());
  CHECK(b.topic_age_ms(Topic::MAX_CELL, 5000) == 3000);
  b.on_message(Topic::MAX_CELL, "junk", 3000);  // garbage changes nothing
  CHECK(b.max_cell_live());
  b.on_reconnect();
  b.on_message(Topic::MAX_CELL, "3.20", 4000);  // retained replay after reconnect
  CHECK(!b.max_cell_live());
  CHECK(b.topic_age_ms(Topic::MAX_CELL, 5000) == 3000);  // age still from the last live update
}

static void test_blocking_retained_cannot_clear() {
  charge::BmsTracker b;
  b.on_reconnect();
  b.on_message(Topic::BLOCKING, "0", 0);     // retained 0 at boot: not blocking
  CHECK(!b.blocking());
  b.on_message(Topic::BLOCKING, "1", 1000);  // live block
  CHECK(b.blocking());
  b.on_reconnect();
  b.on_message(Topic::BLOCKING, "0", 2000);  // retained 0 after reconnect: still blocking
  CHECK(b.blocking());
  b.on_message(Topic::BLOCKING, "0", 3000);  // live 0 clears
  CHECK(!b.blocking());
  b.on_reconnect();
  b.on_message(Topic::BLOCKING, "1", 4000);  // a retained non-zero still sets it
  CHECK(b.blocking());
}

int main() {
  test_cell_limiter_ignores_implausible();
  test_cell_limiter_release_needs_live();
  test_max_cell_live_and_age();
  test_blocking_retained_cannot_clear();
  test_cell_thresholds();
  test_cooldown();
  test_no_current();
  test_battery_voltage();
  test_cell_limiter();
  test_blocking_and_max_cell();
  test_new_topics_not_required_for_freshness();
  test_line_limited_max();
  test_retained_only_is_not_fresh();
  test_second_message_is_fresh();
  test_goes_stale_after_60s();
  test_stale_does_not_wrap_back_to_fresh();
  test_on_message_reports_live_updates();
  test_reconnect_requires_live_update_again();
  test_status_and_flags();
  test_garbage_is_ignored();
  test_missing_flags_not_fresh();
  test_target_voltage();
  test_current_budget();
  test_moving_average();
  test_ramp();
  test_split_and_overvoltage();
  test_boot_and_wait();
  test_hold_reasons();
  test_hold_reason_text();
  test_start_hysteresis();
  test_fault_latch();
  test_fault_reappearing_restarts_timer();
  test_clear_button();
  test_fault_during_boot_returns_to_boot();
  return test_result("charge_logic");
}
