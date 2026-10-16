#include <cmath>

#include "check.h"
#include "fake_nasn_bus.h"

using oem::Rectifier;

struct Rig {
  explicit Rig(bool write_defaults = true) : r(bus, 1, write_defaults) {}
  FakeBus bus;
  FakeModule mod{1};
  Rectifier r;
  uint32_t now = 1000;
  int per_pass = -1;  // frames the bus accepts each pass (-1: no limit)

  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++, now++) {
      bus.accept = per_pass;
      r.loop(now);
      bus.deliver({&mod}, {&r}, now);
    }
  }
};

static void test_comes_online_and_reads_telemetry() {
  Rig g;
  g.mod.vin = 231.0f;
  g.mod.heatsink_c = 41.5f;
  CHECK(!g.r.is_online());
  g.run(400);
  CHECK(g.r.is_online());
  CHECK(g.r.boot_sent());
  CHECK_NEAR(g.r.input_voltage(), 231.0, 0.01);
  CHECK_NEAR(g.r.heatsink_temperature(), 41.5, 0.01);
  CHECK_NEAR(g.r.output_current(), 0.0, 0.01);  // boots in standby: the first setpoints say so
  CHECK(g.mod.standby == 1);
  CHECK(!g.r.faulted());
}

static void test_one_read_in_flight_with_spacing() {
  Rig g;
  g.mod.muted = true;  // nothing answers: each read waits out its 250 ms timeout
  g.run(1000);
  int reads = 0;
  for (const auto &f : g.bus.sent) reads += ((f.id >> 8) & 0xFF) == 0x82;
  CHECK(reads == 4);  // at 0, 250, 500 and 750 ms
  Rig h;             // answered at once: one read per 20 ms until the 12-register cycle is done
  h.run(1000);
  reads = 0;
  for (const auto &f : h.bus.sent) reads += ((f.id >> 8) & 0xFF) == 0x82;
  CHECK(reads == 12);
  h.run(1000);  // the next cycle starts a second after the first
  reads = 0;
  for (const auto &f : h.bus.sent) reads += ((f.id >> 8) & 0xFF) == 0x82;
  CHECK(reads == 24);
}

static void test_offline_after_5s_silence() {
  Rig g;
  g.run(500);
  CHECK(g.r.is_online());
  g.mod.muted = true;  // the last reply came at about 1220 ms
  g.run(4500);
  CHECK(g.r.is_online());
  g.run(1000);
  CHECK(!g.r.is_online());
  CHECK(std::isnan(g.r.output_voltage()) && std::isnan(g.r.input_voltage()));
  g.mod.muted = false;
  const int before = g.bus.writes(0x0100);
  g.run(300);
  CHECK(g.r.is_online());
  CHECK(g.bus.writes(0x0100) == before + 1);  // setpoints again as soon as it reappears
}

static void test_setpoints_on_change_and_keepalive() {
  Rig g;
  g.run(500);
  g.r.set_target(56.0f, 20.0f);
  g.r.set_standby(false);
  g.run(50);
  CHECK_NEAR(g.mod.vout_set, 56.0, 0.01);
  CHECK_NEAR(g.mod.ilimit, 20.0, 0.1);
  CHECK(g.mod.standby == 0);
  const int n = g.bus.writes(0x0100);
  g.r.set_target(56.005f, 20.05f);  // below the 0.01 V / 0.1 A change thresholds
  g.run(4000);
  CHECK(g.bus.writes(0x0100) == n);
  g.run(1100);
  CHECK(g.bus.writes(0x0100) == n + 1);  // 5 s keepalive
  CHECK(g.bus.writes(0x022D) == n + 1 && g.bus.writes(0x0132) == n + 1);
}

static void test_failed_send_retries_promptly() {
  Rig g;
  g.run(500);
  g.bus.full = true;
  g.r.set_target(55.0f, 10.0f);
  g.bus.refused_writes = 0;
  g.run(1000);
  CHECK(g.bus.refused_writes >= 3 * 18 && g.bus.refused_writes <= 3 * 21);  // an attempt per 50 ms, not per pass
  g.bus.full = false;
  g.run(100);
  CHECK_NEAR(g.mod.vout_set, 55.0, 0.01);  // within 100 ms of the queue having room, not at the 5 s keepalive
  CHECK_NEAR(g.mod.ilimit, 10.0, 0.1);
}

static void test_partial_send_reaches_standby() {
  Rig g;
  g.run(500);
  g.r.set_target(56.0f, 20.0f);
  g.r.set_standby(false);
  g.run(100);
  CHECK(g.mod.standby == 0);
  g.per_pass = 2;  // only two of the three writes fit each time
  g.r.set_target(56.0f, 0.0f);
  g.r.set_standby(true);
  g.run(100);
  CHECK(g.mod.standby == 1 && g.mod.ilimit < 0.05f);
  g.per_pass = 1;
  g.r.set_target(56.0f, 20.0f);
  g.r.set_standby(false);
  g.run(200);
  CHECK(g.mod.standby == 0);
  CHECK_NEAR(g.mod.ilimit, 20.0, 0.1);
}

static void test_alarm_mask() {
  Rig g;
  g.run(500);
  g.mod.alarm = 0x00000020;  // bit 5: not in the stock mask
  g.run(1000);
  CHECK(g.r.alarms() == 0x20 && !g.r.faulted());
  g.mod.alarm = 0x00400000;  // bit 22 (E1)
  g.run(1000);
  CHECK(g.r.faulted());
  g.mod.alarm = 0;
  g.run(1000);
  CHECK(!g.r.faulted());
}

static void test_setpoint_mismatch_after_10s() {
  Rig g;
  g.run(500);
  g.mod.frozen = true;
  g.r.set_target(56.0f, 20.0f);
  g.r.set_standby(false);
  g.run(9000);
  CHECK(!g.r.setpoint_mismatch());
  g.run(3000);
  CHECK(g.r.setpoint_mismatch() && g.r.faulted());
  g.mod.frozen = false;
  g.run(6000);  // the keepalive rewrites the setpoints and the readback agrees again
  CHECK(!g.r.setpoint_mismatch() && !g.r.faulted());
}

static void test_write_errors() {
  Rig g;
  g.run(500);
  g.mod.reject_writes = true;
  g.r.set_target(55.0f, 5.0f);  // 3 rejected writes (voltage, current, standby)
  g.run(100);
  CHECK(g.r.consecutive_write_errors() == 3 && g.r.faulted());
  g.mod.reject_writes = false;
  g.run(5000);
  CHECK(g.r.consecutive_write_errors() == 0 && !g.r.faulted());
}

static void test_stored_defaults() {
  Rig g;
  g.r.set_fallback(57.0f, 30.0f);  // clamped to 54.4 V / 10 A
  g.run(7000);  // 0x0101 is fixed on its first readback, 0x0104 (polled later) one keepalive after
  CHECK_NEAR(g.mod.default_v, 54.4, 0.01);
  CHECK_NEAR(g.mod.default_a, 10.0, 0.1);
  const int n = g.bus.writes(0x0101);
  CHECK(n == 1);
  g.run(10000);
  CHECK(g.bus.writes(0x0101) == n);  // the readback now matches: no more writes
  Rig off(false);
  off.run(1500);
  CHECK(off.bus.writes(0x0101) == 0 && off.bus.writes(0x0104) == 0);
}

static void test_first_voltage_write_is_the_fallback() {
  Rig g;
  g.r.set_fallback(52.0f, 3.0f);
  g.run(10);
  int first = -1;
  for (const auto &f : g.bus.sent) {
    if (((f.id >> 8) & 0xFF) == 0x81 && (f.data[0] << 8 | f.data[1]) == 0x0100) {
      first = f.data[6] << 8 | f.data[7];
      break;
    }
  }
  CHECK(first == int(std::lround(52.0 * 1023)));
  CHECK_NEAR(g.mod.vout_set, 52.0, 0.01);
}

static void test_stored_default_rewrites_capped() {
  Rig g;
  g.mod.forgets_defaults = true;
  g.r.set_fallback(52.0f, 3.0f);
  g.run(60000);
  CHECK(g.bus.writes(0x0101) == 3 && g.bus.writes(0x0104) == 3);  // not every 5 s forever
  g.r.set_fallback(52.0f, 3.0f);  // the same values: still capped
  g.run(10000);
  CHECK(g.bus.writes(0x0101) == 3 && g.bus.writes(0x0104) == 3);
  g.r.set_fallback(53.0f, 3.0f);  // a new stored default (the user changed HR 15) gets its own attempts
  g.run(30000);
  CHECK(g.bus.writes(0x0101) == 6 && g.bus.writes(0x0104) == 3);
}

// A master flipping HR 15/16 resets the per-value cap each time; the per-boot total still bounds module flash wear.
static void test_stored_default_writes_per_boot() {
  Rig g;
  g.mod.forgets_defaults = true;
  for (int i = 0; i < 40; i++) {
    g.r.set_fallback(i % 2 ? 52.0f : 53.0f, i % 2 ? 3.0f : 4.0f);
    g.run(6000);
  }
  CHECK(g.bus.writes(0x0101) + g.bus.writes(0x0104) == oem::DEFAULT_WRITES_PER_BOOT);
}

static void test_ignores_other_address() {
  FakeBus bus;
  FakeModule two(2);
  Rectifier r(bus, 1, false);
  uint32_t now = 1000;
  for (int i = 0; i < 1000; i++, now++) {
    r.loop(now);
    bus.deliver({&two}, {&r}, now);
  }
  CHECK(!r.is_online());
}

int main() {
  test_comes_online_and_reads_telemetry();
  test_one_read_in_flight_with_spacing();
  test_offline_after_5s_silence();
  test_setpoints_on_change_and_keepalive();
  test_failed_send_retries_promptly();
  test_partial_send_reaches_standby();
  test_alarm_mask();
  test_setpoint_mismatch_after_10s();
  test_write_errors();
  test_stored_defaults();
  test_first_voltage_write_is_the_fallback();
  test_stored_default_rewrites_capped();
  test_stored_default_writes_per_boot();
  test_ignores_other_address();
  return test_result("test_oem_rectifier");
}
