#include <cmath>

#include "app/modbus_source.h"
#include "check.h"

using namespace oem;

namespace {

// HR 100-107: heartbeat, CVL x100, CCL x10, flags, SOC, V x100, max cell x1000, current x10.
const uint16_t GOOD[8] = {1, 5640, 300, 1, 67, 5299, 3320, 0x7FFF};

void test_never_fresh_is_stale_fallback() {
  ModbusSource m;
  Settings s = default_settings();  // Max current 10, Fallback 10
  s.set(HR_FALLBACK_A, 4);
  const Limits l = m.limits(s, 5000);
  CHECK(!m.fresh(5000));
  CHECK(l.stale && l.fresh && l.allowed && l.source == SourceMode::MODBUS);
  CHECK_NEAR(l.ccl, 4.0, 1e-6);
  CHECK_NEAR(l.cvl, s.get(HR_CHARGE_V) / 100.0, 1e-4);
  CHECK(l.soc == -1 && std::isnan(l.battery_v));
}

void test_fresh_decode() {
  ModbusSource m;
  const Settings s = default_settings();
  m.heartbeat(GOOD, 1000);
  CHECK(m.fresh(1000) && m.fresh(1000 + EXT_FRESH_MS) && !m.fresh(1001 + EXT_FRESH_MS));
  const Limits l = m.limits(s, 2000);
  CHECK(l.fresh && !l.stale && l.source == SourceMode::MODBUS);
  CHECK_NEAR(l.cvl, 56.40, 1e-4);
  CHECK_NEAR(l.ccl, 30.0, 1e-4);  // not capped here: the controller applies Max current
  CHECK(l.allowed && !l.alarm && !l.blocking);
  CHECK(l.soc == 67);
  CHECK_NEAR(l.battery_v, 52.99, 1e-4);
  CHECK_NEAR(l.max_cell_v, 3.320, 1e-5);
  CHECK(std::isnan(l.battery_a));
}

void test_unknown_codes_and_flags() {
  ModbusSource m;
  const Settings s = default_settings();
  const uint16_t u[8] = {1, 5500, 20, 0b110, 0xFFFF, 0, 0, uint16_t(int16_t(-25))};
  m.heartbeat(u, 0);
  const Limits l = m.limits(s, 0);
  CHECK(!l.allowed && l.alarm && l.blocking);
  CHECK(l.soc == -1 && std::isnan(l.battery_v) && std::isnan(l.max_cell_v));
  CHECK_NEAR(l.battery_a, -2.5, 1e-5);
  const uint16_t soc101[8] = {1, 5500, 20, 1, 101, 0, 0, 0x7FFF};
  m.heartbeat(soc101, 0);
  CHECK(m.limits(s, 0).soc == -1);
}

void test_stale_after_window_and_stop_latch() {
  ModbusSource m;
  Settings s = default_settings();
  s.set(HR_FALLBACK_A, 0);  // 0 = no charging without the source
  m.heartbeat(GOOD, 0);
  Limits l = m.limits(s, EXT_FRESH_MS + 1);
  CHECK(l.stale && l.allowed);
  CHECK_NEAR(l.ccl, 0.0, 1e-6);

  const uint16_t off[8] = {1, 5640, 300, 0, 67, 5299, 3320, 0x7FFF};  // charge allowed off
  m.heartbeat(off, 20000);
  CHECK(!m.limits(s, 20000).allowed);
  CHECK(!m.limits(s, 20001 + EXT_FRESH_MS).allowed);  // a stop holds while stale
  m.forget_stop();                                      // a mode change forgets it
  CHECK(m.limits(s, 20001 + EXT_FRESH_MS).allowed);

  const uint16_t alarm[8] = {1, 5640, 300, 0b011, 67, 5299, 3320, 0x7FFF};  // allowed but alarm
  m.heartbeat(alarm, 40000);
  CHECK(m.limits(s, 40000).alarm);
  CHECK(!m.limits(s, 40001 + EXT_FRESH_MS).allowed);
  m.heartbeat(GOOD, 60000);  // fresh and fine again: the latch clears
  CHECK(m.limits(s, 60001 + EXT_FRESH_MS).allowed);
}

void test_blocking_and_ccl_zero_latch() {
  ModbusSource m;
  Settings s = default_settings();
  const uint16_t blocking[8] = {1, 5640, 300, 0b101, 67, 5299, 3320, 0x7FFF};  // allowed + blocking
  m.heartbeat(blocking, 0);
  CHECK(m.limits(s, 0).blocking);
  CHECK(!m.limits(s, 1 + EXT_FRESH_MS).allowed);
  m.heartbeat(GOOD, 20000);
  CHECK(m.limits(s, 20001 + EXT_FRESH_MS).allowed);

  const uint16_t ccl0[8] = {1, 5640, 0, 1, 67, 5299, 3320, 0x7FFF};
  m.heartbeat(ccl0, 40000);
  CHECK(!m.limits(s, 40001 + EXT_FRESH_MS).allowed);
  m.heartbeat(GOOD, 60000);
  CHECK(m.limits(s, 60001 + EXT_FRESH_MS).allowed);
}

void test_forget_keeps_data() {
  ModbusSource m;
  const Settings s = default_settings();
  m.heartbeat(GOOD, 1000);
  m.forget_stop();
  CHECK(m.fresh(2000) && !m.limits(s, 2000).stale);
}

}  // namespace

int main() {
  test_never_fresh_is_stale_fallback();
  test_fresh_decode();
  test_unknown_codes_and_flags();
  test_stale_after_window_and_stop_latch();
  test_blocking_and_ccl_zero_latch();
  test_forget_keeps_data();
  return test_result("test_oem_modbus_source");
}
