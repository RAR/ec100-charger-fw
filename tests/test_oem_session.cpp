#include <cmath>

#include "app/pause_memory.h"
#include "app/relay.h"
#include "app/session_gate.h"
#include "check.h"

using namespace oem;

static const GateThresholds T{20, 100, 42.0f, 57.6f};

static GateInputs volts(float v, float a = 0.0f) { return {-1, v, a, false}; }

static void test_boot_active_when_start_holds() {
  SessionGate g;
  CHECK(g.update(volts(41.0f), T, 0));  // at once, no 5 s wait
  SessionGate h;
  CHECK(!h.update(volts(50.0f), T, 0));
  SessionGate u;
  CHECK(!u.update(volts(NAN), T, 0));  // unknown voltage and SOC: no session
}

static void test_start_by_voltage_after_5s() {
  SessionGate g;
  g.update(volts(50.0f), T, 0);
  CHECK(!g.update(volts(41.9f), T, 1000));
  CHECK(!g.update(volts(41.9f), T, 5999));
  CHECK(!g.update(volts(42.5f), T, 6000));  // a blip above restarts the hold
  CHECK(!g.update(volts(41.9f), T, 7000));
  CHECK(g.update(volts(41.9f), T, 12000));
}

static void test_start_by_soc() {
  SessionGate g;
  g.update({50, 50.0f, 0.0f, false}, T, 0);
  CHECK(!g.update({20, 50.0f, 0.0f, false}, T, 1000));
  CHECK(g.update({20, 50.0f, 0.0f, false}, T, 6000));  // SOC <= Start SOC, held 5 s
}

static void test_stop_needs_voltage_and_tail_current() {
  SessionGate g;
  g.update(volts(41.0f), T, 0);
  CHECK(g.update(volts(57.6f, 10.0f), T, 1000));    // at Stop V but still taking 10 A
  CHECK(g.update(volts(57.6f, 10.0f), T, 20000));
  CHECK(g.update(volts(57.6f, 4.0f), T, 21000));    // tail current reached: start the 10 s hold
  CHECK(g.update(volts(57.6f, 4.0f), T, 30999));
  CHECK(!g.update(volts(57.6f, 3.0f), T, 31000));
}

static void test_stop_by_soc() {
  SessionGate g;
  g.update({10, NAN, 0.0f, false}, T, 0);
  CHECK(g.active());
  CHECK(g.update({100, NAN, 30.0f, false}, T, 1000));
  CHECK(!g.update({100, NAN, 30.0f, false}, T, 11000));  // SOC >= Stop SOC stops whatever the current
}

static void test_charge_now() {
  SessionGate g;
  g.update(volts(50.0f), T, 0);
  CHECK(g.update({-1, 50.0f, 0.0f, true}, T, 1000));  // ignores the start thresholds
  CHECK(g.update(volts(50.0f), T, 60000));            // and stays until a stop condition
  g.update(volts(57.7f, 0.0f), T, 61000);
  CHECK(!g.update(volts(57.7f, 0.0f), T, 71000));
}

static void test_relay_auto() {
  RelayLogic r;
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, 0));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 1000));
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 2000));   // session over: held 10 s
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 10999));
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, 11000));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 12000));
  CHECK(!r.update(RelayMode::AUTO, true, true, false, false, 13000));   // paused: opens at once
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 23000));
  CHECK(!r.update(RelayMode::AUTO, true, false, true, false, 33000));   // fault: opens at once
}

static void test_relay_immediate_open() {
  RelayLogic r;
  r.update(RelayMode::AUTO, true, false, false, false, 1000);
  CHECK(!r.update(RelayMode::AUTO, true, true, false, false, 1001));    // pause
  r.update(RelayMode::AUTO, true, false, false, false, 2000);
  CHECK(!r.update(RelayMode::AUTO, true, false, true, false, 2001));    // fault
  r.update(RelayMode::AUTO, true, false, false, false, 3000);
  CHECK(!r.update(RelayMode::AUTO, false, false, true, false, 3001));   // fault during the end delay
  CHECK(!r.update(RelayMode::AUTO, false, false, true, false, 3002));   // fault, no session: stays open
  RelayLogic o;
  o.update(RelayMode::AUTO, true, false, false, false, 0);
  CHECK(!o.update(RelayMode::OFF, true, false, false, false, 1));       // Off opens at once
  CHECK(!o.update(RelayMode::OFF, true, false, false, true, 2));
}

static void test_relay_end_delay() {
  RelayLogic r;
  r.update(RelayMode::AUTO, true, false, false, false, 1000);
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 5000));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 6000));    // session returns: no chatter
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 7000));
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 15999));  // 9999 ms after the last wanted
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, 16000));
}

static void test_relay_wraparound() {
  const uint32_t base = 0xFFFFF000u;
  RelayLogic r;
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, base));
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, base + 9999u));
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, base + 10000u));
}

static void test_gate_thresholds_exact() {
  SessionGate a;
  a.update(volts(50.0f), T, 0);
  CHECK(!a.update(volts(42.0f), T, 1000));    // V < Start V is strict
  CHECK(!a.update(volts(42.0f), T, 20000));
  CHECK(!a.update(volts(41.99f), T, 21000));
  CHECK(a.update(volts(41.99f), T, 26000));
  SessionGate b;
  b.update({50, 50.0f, 0.0f, false}, T, 0);
  CHECK(!b.update({21, 50.0f, 0.0f, false}, T, 1000));  // SOC 21 > Start SOC 20
  CHECK(!b.update({21, 50.0f, 0.0f, false}, T, 20000));
  SessionGate c;
  c.update(volts(41.0f), T, 0);
  CHECK(c.update({99, NAN, 30.0f, false}, T, 1000));    // SOC 99 < Stop SOC 100
  CHECK(c.update({99, NAN, 30.0f, false}, T, 20000));
  CHECK(c.update(volts(57.59f, 0.0f), T, 21000));       // V below Stop V
  CHECK(c.update(volts(57.59f, 0.0f), T, 40000));
  CHECK(c.update(volts(57.6f, 4.01f), T, 41000));       // current above the tail limit
  CHECK(c.update(volts(57.6f, 4.01f), T, 60000));
}

static void test_gate_hold_boundaries() {
  SessionGate g;
  g.update(volts(50.0f), T, 0);
  g.update(volts(41.0f), T, 1000);                      // start hold begins
  CHECK(!g.update(volts(41.0f), T, 5999));
  CHECK(g.update(volts(41.0f), T, 6000));
  g.update(volts(57.7f, 0.0f), T, 7000);                // stop hold begins
  CHECK(g.update(volts(57.7f, 0.0f), T, 16999));
  CHECK(!g.update(volts(57.7f, 0.0f), T, 17000));
}

static void test_gate_stop_flicker() {
  SessionGate g;
  g.update(volts(41.0f), T, 0);
  CHECK(g.update(volts(57.6f, 3.0f), T, 1000));
  CHECK(g.update(volts(57.6f, 3.0f), T, 9000));
  CHECK(g.update(volts(50.0f, 3.0f), T, 10000));        // condition drops: timer restarts
  CHECK(g.update(volts(57.6f, 3.0f), T, 11000));
  CHECK(g.update(volts(57.6f, 3.0f), T, 20999));
  CHECK(!g.update(volts(57.6f, 3.0f), T, 21000));
}

static void test_gate_charge_now_with_stop_holding() {
  SessionGate g;
  g.update(volts(57.7f, 0.0f), T, 0);
  CHECK(!g.active());
  CHECK(g.update({-1, 57.7f, 0.0f, true}, T, 1000));    // active at once
  CHECK(g.update(volts(57.7f, 0.0f), T, 10999));
  CHECK(!g.update(volts(57.7f, 0.0f), T, 11000));       // ends 10 s after the click
}

static void test_gate_unknown_never_stops() {
  SessionGate g;
  g.update(volts(41.0f), T, 0);
  CHECK(g.update(volts(NAN, 0.0f), T, 1000));           // NaN V with a low current: no stop
  CHECK(g.update(volts(NAN, 0.0f), T, 100000));
  CHECK(g.update(volts(57.0f, 0.0f), T, 101000));       // unknown SOC, V below Stop V
  CHECK(g.update(volts(57.0f, 0.0f), T, 200000));
}

static void test_gate_wraparound() {
  const uint32_t base = 0xFFFFF000u;
  SessionGate g;
  g.update(volts(50.0f), T, base);
  g.update(volts(41.0f), T, base + 1000u);
  CHECK(!g.update(volts(41.0f), T, base + 5999u));
  CHECK(g.update(volts(41.0f), T, base + 6000u));
  g.update(volts(57.7f, 0.0f), T, base + 7000u);
  CHECK(g.update(volts(57.7f, 0.0f), T, base + 16999u));
  CHECK(!g.update(volts(57.7f, 0.0f), T, base + 17000u));
}

static void test_gate_end() {
  SessionGate g;
  CHECK(g.update(volts(41.0f), T, 0));
  g.end();
  CHECK(!g.active());
  CHECK(!g.update(volts(41.0f), T, 1000));  // a fresh start needs the 5 s hold again
  CHECK(g.update(volts(41.0f), T, 6000));
}

static void test_gate_end_before_first_reading_keeps_boot_rule() {
  SessionGate g;
  g.end();  // implausible sense at boot: the first plausible reading still counts as the power-up one
  CHECK(g.update(volts(41.0f), T, 0));
}

static void test_pause_memory() {
  CHECK(pause_word(true) == PAUSE_MAGIC && pause_word(false) == 0);
  CHECK(PAUSE_REBOOT_MAGIC == 0x50415552);
  CHECK(restore_pause(true, false, PAUSE_MAGIC));
  CHECK(!restore_pause(false, false, PAUSE_MAGIC));  // power-cycle: never paused
  CHECK(!restore_pause(false, true, PAUSE_MAGIC));   // a software reset only keeps a pause written for it
  CHECK(restore_pause(false, true, PAUSE_REBOOT_MAGIC));  // Modbus reboot (HR 120) while paused
  CHECK(!restore_pause(true, false, PAUSE_REBOOT_MAGIC));
  CHECK(!restore_pause(false, false, PAUSE_REBOOT_MAGIC));
  CHECK(!restore_pause(true, true, 0));
  CHECK(!restore_pause(true, false, PAUSE_MAGIC ^ 1));  // anything else in the register: not ours
  CHECK(!restore_pause(false, true, PAUSE_REBOOT_MAGIC ^ 1));
}

static void test_relay_off_and_modbus() {
  RelayLogic r;
  CHECK(!r.update(RelayMode::OFF, true, false, false, true, 0));
  CHECK(r.update(RelayMode::MODBUS, false, false, false, true, 1000));
  CHECK(!r.update(RelayMode::MODBUS, true, false, false, false, 2000));
}

int main() {
  test_boot_active_when_start_holds();
  test_start_by_voltage_after_5s();
  test_start_by_soc();
  test_stop_needs_voltage_and_tail_current();
  test_stop_by_soc();
  test_charge_now();
  test_relay_auto();
  test_relay_immediate_open();
  test_relay_end_delay();
  test_relay_wraparound();
  test_gate_thresholds_exact();
  test_gate_hold_boundaries();
  test_gate_stop_flicker();
  test_gate_charge_now_with_stop_holding();
  test_gate_unknown_never_stops();
  test_gate_wraparound();
  test_relay_off_and_modbus();
  test_gate_end();
  test_gate_end_before_first_reading_keeps_boot_rule();
  test_pause_memory();
  return test_result("test_oem_session");
}
