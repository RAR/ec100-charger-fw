#include <cmath>

#include "app/controller.h"
#include "app/modbus_map.h"
#include "app/modbus_rtu.h"
#include "app/relay.h"
#include "check.h"
#include "fake_nasn_bus.h"

using namespace oem;
using charge::State;

namespace {

struct Rig {
  FakeBus bus;
  FakeModule m1{1}, m2{2};
  Rectifier r1{bus, 1, true}, r2{bus, 2, true};
  Controller ctl{r1, r2};
  Settings s = default_settings();
  Telemetry tel{&ctl.status(), {&r1, &r2}, &ctl, false, 50.0f, 0, 0, false};
  OemRegisters regs{s, tel};
  uint32_t now = 1000;

  void run_s(int seconds) {
    for (int t = 0; t < seconds * 1000; t++, now++) {
      r1.loop(now);
      r2.loop(now);
      bus.deliver({&m1, &m2}, {&r1, &r2}, now);
      if (now % CONTROLLER_TICK_MS == 0) ctl.tick(s, panel_limits(s), tel.adc_v, now);
    }
    tel.now_ms = now;
  }

  uint16_t ir(uint16_t reg) {
    uint16_t v = 0x5A5A;
    CHECK(regs.read_input(reg, v));
    return v;
  }
  uint16_t hr(uint16_t reg) {
    uint16_t v = 0x5A5A;
    CHECK(regs.read_holding(reg, v));
    return v;
  }
  uint8_t write1(uint16_t reg, uint16_t v) { return regs.write_holding(reg, &v, 1); }
};

void test_status_registers() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(60);  // past the module soft-start
  const Status &st = g.ctl.status();
  CHECK(st.state == State::CHARGING);
  CHECK(g.ir(0) == 0x030A);
  CHECK(g.ir(1) == 2);
  CHECK(g.ir(2) == 0);
  CHECK(g.ir(3) == 0);
  CHECK(g.ir(5) == 0);
  CHECK(g.ir(6) == 5760);
  CHECK(g.ir(7) == 100);
  CHECK(g.ir(8) == 100);
  CHECK(g.ir(9) == 100);  // 10 A
  CHECK(g.ir(10) == uint16_t(std::lround(st.power_w)));
  CHECK(g.ir(11) == uint16_t(std::lround(st.battery_v * 100.0)));
  CHECK(g.ir(12) == 5000);
  CHECK(g.ir(13) == 0xFFFF);  // Panel: no SOC
  CHECK(g.ir(14) == 5760);
  CHECK(g.ir(15) == 100);
  CHECK(g.ir(18) == 0);
  // Flags: session active, AC present, both PSUs online, relay as told.
  CHECK(g.ir(4) == ((1u << 3) | (1u << 5) | (1u << 6) | (1u << 7)));
  g.tel.relay_closed = true;
  CHECK(g.ir(4) & (1u << 4));
  g.tel.uptime_s = 70000;
  CHECK(g.ir(16) == 1 && g.ir(17) == 4464);
}

// IR 5 is the source of the limits in use and IR 4 bit 2 its staleness: clear with Panel, set from a stale Pylontech.
void test_source_registers() {
  Rig g;
  g.ctl.tick(g.s, panel_limits(g.s), g.tel.adc_v, g.now);
  CHECK(g.ir(5) == 0);
  CHECK(!(g.ir(4) & (1u << 2)));
  Limits l = panel_limits(g.s);
  l.stale = true;
  l.source = SourceMode::PYLONTECH;
  g.ctl.tick(g.s, l, g.tel.adc_v, g.now + CONTROLLER_TICK_MS);
  CHECK(g.ir(5) == 1);
  CHECK(g.ir(4) & (1u << 2));
  l.source = SourceMode::MODBUS;
  g.ctl.tick(g.s, l, g.tel.adc_v, g.now + 2 * CONTROLLER_TICK_MS);
  CHECK(g.ir(5) == 2);
}

void test_modbus_source_snapshot() {
  Rig g;
  g.tel.now_ms = 1000;
  CHECK(g.regs.modbus_limits(g.s, 1000).stale);  // nothing written yet
  const uint16_t blk[8] = {1, 5500, 250, 1, 80, 5300, 3400, 0x7FFF};
  CHECK(g.regs.write_holding(100, blk, 8) == 0);
  Limits l = g.regs.modbus_limits(g.s, 1000);
  CHECK(!l.stale && l.source == SourceMode::MODBUS && l.soc == 80);
  CHECK_NEAR(l.cvl, 55.0, 1e-4);
  CHECK_NEAR(l.ccl, 25.0, 1e-4);
  CHECK(g.write1(102, 50) == 0);  // without a heartbeat: read back, not used
  CHECK(g.hr(102) == 50);
  CHECK_NEAR(g.regs.modbus_limits(g.s, 1000).ccl, 25.0, 1e-4);
  // A lone HR 100 write (or any shorter one) is not a snapshot: limits unchanged, Modbus freshness not refreshed.
  g.tel.now_ms = 1000 + EXT_FRESH_MS;
  CHECK(g.write1(100, 1) == 0);
  CHECK_NEAR(g.regs.modbus_limits(g.s, g.tel.now_ms).ccl, 25.0, 1e-4);
  g.tel.now_ms = 2000 + EXT_FRESH_MS;  // the block write at 1000 is now stale
  CHECK(g.write1(100, 1) == 0);
  CHECK(g.regs.modbus_limits(g.s, g.tel.now_ms).stale);
  const uint16_t seven[7] = {1, 5500, 250, 1, 80, 5300, 3400};
  CHECK(g.regs.write_holding(100, seven, 7) == 0);
  CHECK(g.regs.modbus_limits(g.s, g.tel.now_ms).stale);
  CHECK(g.regs.external_soc(g.tel.now_ms) == 80);  // but the Panel SOC display is refreshed
  g.tel.now_ms = 1000;
  CHECK(g.regs.write_holding(100, blk, 8) == 0);
  CHECK(!g.regs.modbus_limits(g.s, 1000).stale);
  const uint16_t stop[8] = {1, 5500, 250, 0, 80, 5300, 3400, 0x7FFF};
  CHECK(g.regs.write_holding(100, stop, 8) == 0);
  CHECK(!g.regs.modbus_limits(g.s, 1001 + EXT_FRESH_MS).allowed);
  g.regs.forget_modbus_stop();
  CHECK(g.regs.modbus_limits(g.s, 1001 + EXT_FRESH_MS).allowed);
}

// Real ModbusSource limits driving the Controller.
void test_modbus_limits_in_controller() {
  Rig g;
  g.s.set(HR_FALLBACK_A, 4);
  g.s.set(HR_START_SOC, 30);
  g.s.set(HR_STOP_SOC, 90);
  uint32_t now = 1000;
  g.tel.now_ms = now;
  const uint16_t alarm[8] = {1, 5500, 250, 0b011, 20, 5300, 3400, 0x7FFF};
  CHECK(g.regs.write_holding(100, alarm, 8) == 0);
  g.ctl.tick(g.s, g.regs.modbus_limits(g.s, now), g.tel.adc_v, now);
  CHECK(g.ctl.status().error == E_BMS_ALARM);

  // Fresh and healthy, SOC below Start SOC: the session (started at power-up) is active.
  const uint16_t good[8] = {1, 5500, 250, 1, 20, 5300, 3400, 0x7FFF};
  CHECK(g.regs.write_holding(100, good, 8) == 0);
  g.ctl.tick(g.s, g.regs.modbus_limits(g.s, now), g.tel.adc_v, now);
  CHECK(g.ctl.status().error == 0 && g.ctl.status().session_active);

  // No snapshot for more than 10 s: E15, budget capped at Fallback current (4 A), source Modbus.
  now += EXT_FRESH_MS + 1;
  g.tel.now_ms = now;
  g.ctl.tick(g.s, g.regs.modbus_limits(g.s, now), g.tel.adc_v, now);
  CHECK(g.ctl.status().error == E_SOURCE_STALE && g.ctl.status().stale);
  CHECK(g.ctl.status().budget <= 4.0f + 1e-4f);
  CHECK(g.ctl.status().source == uint8_t(SourceMode::MODBUS));

  // SOC at or above Stop SOC ends the session once held for 10 s, and a fresh snapshot does not restart it.
  const uint16_t full[8] = {1, 5500, 250, 1, 100, 5300, 3400, 0x7FFF};
  for (int i = 0; i < 12; i++, now += 1000) {
    g.tel.now_ms = now;
    CHECK(g.regs.write_holding(100, full, 8) == 0);
    g.ctl.tick(g.s, g.regs.modbus_limits(g.s, now), g.tel.adc_v, now);
  }
  CHECK(!g.ctl.status().session_active);
  CHECK(g.ctl.status().error == 0);
}

void test_waiting_and_e17() {
  Rig g;
  g.run_s(5);
  CHECK(g.ir(1) == 3 && g.ir(2) == 7);
  CHECK(g.ir(6) == 0xFFFF);   // NaN target while not charging
  CHECK(g.ir(14) == 5760);    // CVL in use is set anyway
  CHECK(g.ir(4) == ((1u << 5) | (1u << 6) | (1u << 7)));
  g.tel.adc_v = 0.0f;
  g.run_s(2);
  CHECK(g.ir(3) == 17);
}

void test_psu_block() {
  Rig g;
  g.m2.heatsink_c = 35.0f;
  g.m2.alarm = 0x00010001;
  g.run_s(5);
  CHECK(g.r1.is_online() && g.r1.standby() && !g.r1.faulted());
  CHECK(g.ir(20) == ((1u << 0) | (1u << 3)));  // online, standby
  CHECK(g.ir(21) == uint16_t(std::lround(g.r1.output_voltage() * 100.0)));
  CHECK(g.ir(22) == 0);
  CHECK(g.ir(23) == 2300);
  CHECK(g.ir(24) == 0x7FFF);
  CHECK(g.ir(25) == 350);
  CHECK(g.ir(26) == 0xFFFF && g.ir(27) == 0xFFFF);
  CHECK(g.ir(28) == 0 && g.ir(29) == 0);
  CHECK(g.ir(35) == 350);
  CHECK(g.ir(38) == 0x0001 && g.ir(39) == 0x0001);
  CHECK(g.ir(30) & (1u << 1));  // bit 0 of the alarm word is in the stock mask: faulted
  CHECK(g.ir(19) == 0xFFFF);
  uint16_t v = 0;
  CHECK(!g.regs.read_input(40, v));
  CHECK(!g.regs.read_input(0xFFFF, v));
}

void test_scaling() {
  CHECK(mb_u16(NAN, 100.0f) == 0xFFFF);
  CHECK(mb_i16(NAN, 10.0f) == 0x7FFF);
  CHECK(mb_i16(-5.0f, 10.0f) == 0xFFCE);
  CHECK(mb_i16(-0.04f, 10.0f) == 0);
  CHECK(mb_i16(-4000.0f, 10.0f) == 0x8000);
  CHECK(mb_i16(4000.0f, 10.0f) == 0x7FFF);
  CHECK(mb_u16(-1.0f, 10.0f) == 0);
  CHECK(mb_u16(1e6f, 10.0f) == 0xFFFF);
  CHECK(mb_u16(57.6f, 100.0f) == 5760);
  CHECK(mb_u16(0.149f, 100.0f) == 15);
  CHECK(mb_u16(INFINITY, 1.0f) == 0xFFFF);
}

void test_settings_writes() {
  Rig g;
  CHECK(g.hr(0) == 10 && g.hr(1) == 5760);
  CHECK(g.write1(0, 20) == 0);
  CHECK(g.s.get(HR_MAX_CURRENT) == 20 && g.hr(0) == 20);
  Commands c = g.regs.take_commands();
  CHECK(c.settings_changed && !c.line_changed);
  CHECK(!g.regs.take_commands().settings_changed);

  CHECK(g.write1(0, 101) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.s.get(HR_MAX_CURRENT) == 20);
  CHECK(g.write1(5, 5790) == MB_EX_ILLEGAL_VALUE);
  g.s.set(HR_CHARGE_V, 5500);
  CHECK(g.write1(5, 5600) == MB_EX_ILLEGAL_VALUE);  // in range, but above Charge V
  CHECK(g.s.get(HR_STOP_V) == 5760);
  g.s = default_settings();

  const Settings before = g.s;
  const uint16_t block[5] = {5700, 30, 90, 4300, 5781};
  CHECK(g.regs.write_holding(1, block, 5) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.s == before);
  const uint16_t good[5] = {5700, 30, 90, 4300, 5600};
  CHECK(g.regs.write_holding(1, good, 5) == 0);
  CHECK(g.s.get(HR_CHARGE_V) == 5700 && g.s.get(HR_STOP_V) == 5600 && g.s.get(HR_START_SOC) == 30);
  CHECK(g.regs.take_commands().settings_changed);

  CHECK(g.write1(6, 1) == MB_EX_ILLEGAL_VALUE);  // M1: Panel or Modbus; never Pylontech (it would cut this link)
  CHECK(g.write1(6, 3) == MB_EX_ILLEGAL_VALUE);  // EG4, JK and JK-PB poll too
  CHECK(g.write1(6, 4) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.write1(6, 5) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.write1(6, 2) == 0 && g.s.get(HR_SOURCE_MODE) == 2);
  CHECK(g.write1(6, 0) == 0);
  g.s.set(HR_SOURCE_MODE, 1);  // stored from the menu: it doesn't block other writes
  CHECK(g.write1(0, 20) == 0 && g.s.get(HR_MAX_CURRENT) == 20);
  // A block that spans HR 6 but leaves the stored value as it is (oem_modbus.py's read-modify-write) is accepted.
  const uint16_t keep[3] = {uint16_t(g.s.get(HR_STOP_V)), 1, 3};  // HR 5 unchanged, HR 6 still 1, HR 7 = 3
  CHECK(g.regs.write_holding(5, keep, 3) == 0 && g.s.get(HR_MODBUS_ADDR) == 3 && g.s.get(HR_SOURCE_MODE) == 1);
  CHECK(g.write1(7, 1) == 0);
  CHECK(g.write1(6, 2) == 0);  // leaving Pylontech for Modbus keeps the link, so it is accepted
  CHECK(g.write1(6, 0) == 0 && g.s.get(HR_SOURCE_MODE) == 0);
  CHECK(g.write1(0, 10) == 0);
  g.regs.take_commands();
  CHECK(g.write1(14, uint16_t(-20)) == 0);
  CHECK(g.s.get(HR_ADC_TRIM) == -20 && g.hr(14) == 0xFFEC);
  CHECK(g.write1(14, uint16_t(-51)) == MB_EX_ILLEGAL_VALUE);
  g.regs.take_commands();

  CHECK(g.write1(7, 5) == 0);
  c = g.regs.take_commands();
  CHECK(c.line_changed && c.settings_changed && g.s.get(HR_MODBUS_ADDR) == 5);
  CHECK(g.write1(8, 3) == 0);
  CHECK(g.regs.take_commands().line_changed);
  CHECK(g.write1(8, 3) == 0);  // same value: nothing changed
  c = g.regs.take_commands();
  CHECK(!c.line_changed && !c.settings_changed);

  g.tel.menu_open = true;
  CHECK(g.write1(0, 15) == MB_EX_BUSY);
  CHECK(g.s.get(HR_MAX_CURRENT) == 10);
  CHECK(g.write1(110, 1) == 0);  // runtime registers still work
  CHECK(g.hr(0) == 10);
}

void test_runtime_registers() {
  Rig g;
  CHECK(g.hr(110) == 0 && !g.regs.paused());
  CHECK(g.write1(110, 1) == 0);
  Commands c = g.regs.take_commands();
  CHECK(c.pause_set && c.pause && !c.settings_changed);
  CHECK(g.hr(110) == 1 && g.regs.paused());
  CHECK(g.write1(110, 2) == MB_EX_ILLEGAL_VALUE);
  CHECK(!g.regs.take_commands().pause_set);
  g.regs.sync_pause(false);
  CHECK(g.hr(110) == 0);

  CHECK(g.write1(111, 1) == 0);
  CHECK(g.regs.take_commands().clear_fault);
  CHECK(g.write1(111, 0) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.write1(113, 1) == 0);
  CHECK(g.regs.take_commands().charge_now);
  CHECK(g.hr(111) == 0 && g.hr(113) == 0 && g.hr(120) == 0);

  CHECK(g.write1(120, 1) == MB_EX_ILLEGAL_VALUE);
  CHECK(!g.regs.take_commands().reboot);
  CHECK(g.write1(120, REBOOT_MAGIC) == 0);
  CHECK(g.regs.take_commands().reboot);

  CHECK(g.write1(112, 1) == 0);
  CHECK(g.regs.relay_command() && g.hr(112) == 1);
  CHECK(g.write1(112, 2) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.regs.relay_command());

  CHECK(g.write1(101, 5500) == 0);
  CHECK(g.hr(101) == 5500);
  CHECK(g.write1(100, 0xFFFF) == 0 && g.hr(100) == 0xFFFF);
  const uint16_t ext[3] = {1, 2, 3};
  CHECK(g.regs.write_holding(105, ext, 3) == 0 && g.hr(107) == 3);

  // Mixed block: nothing applied when one value is bad.
  const uint16_t rt[4] = {1, 1, 0, 5};
  CHECK(g.regs.write_holding(110, rt, 4) == MB_EX_ILLEGAL_VALUE);
  c = g.regs.take_commands();
  CHECK(!c.pause_set && !c.clear_fault && !c.charge_now);

  uint16_t v = 0;
  CHECK(g.write1(99, 1) == MB_EX_ILLEGAL_ADDRESS);
  CHECK(g.write1(17, 1) == MB_EX_ILLEGAL_ADDRESS);
  CHECK(g.write1(108, 1) == MB_EX_ILLEGAL_ADDRESS);
  CHECK(!g.regs.read_holding(99, v) && !g.regs.read_holding(17, v) && !g.regs.read_holding(121, v));
  uint16_t span[85] = {};
  span[0] = 20;
  CHECK(g.regs.write_holding(16, span, 85) == MB_EX_ILLEGAL_ADDRESS);  // 16-100 crosses a gap
  CHECK(g.regs.write_holding(106, ext, 3) == MB_EX_ILLEGAL_ADDRESS);   // 106-108
  CHECK(g.regs.write_holding(0xFFFF, ext, 2) == MB_EX_ILLEGAL_ADDRESS);
  CHECK(g.s.get(HR_MODULE_FALLBACK_A) == 5);

  g.write1(110, 1);
  g.write1(111, 1);
  g.regs.take_commands();
  c = g.regs.take_commands();
  CHECK(!c.settings_changed && !c.pause_set && !c.clear_fault && !c.charge_now && !c.reboot && !c.line_changed);
}

void test_clamps() {
  Settings s = default_settings();
  CHECK(!violates_clamps(s));
  s.set(HR_STOP_V, 5700);
  s.set(HR_CHARGE_V, 5600);
  CHECK(violates_clamps(s));
  s = default_settings();
  s.set(HR_MAX_CURRENT, 101);
  CHECK(violates_clamps(s));
  s = default_settings();
  s.set(HR_ADC_TRIM, -51);
  CHECK(violates_clamps(s));
  s = default_settings();
  s.set(HR_START_SOC, 50);
  s.set(HR_STOP_SOC, 50);
  CHECK(violates_clamps(s));
  s = default_settings();
  s.set(HR_START_V, 5300);  // out of range and within 5 V of Stop V
  CHECK(violates_clamps(s));
}

void test_relay_leaving_modbus() {
  RelayLogic r;
  CHECK(r.update(RelayMode::MODBUS, false, false, false, true, 1000));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 50000));
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 51000));
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 59999));
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, 60000));

  // Leaving Modbus mode closed with no session: the end delay counts from the switch, not from a stale time.
  RelayLogic m;
  m.update(RelayMode::AUTO, true, false, false, false, 1000);
  CHECK(m.update(RelayMode::MODBUS, false, false, false, true, 2000));
  CHECK(m.update(RelayMode::AUTO, false, false, false, false, 100000));
  CHECK(m.update(RelayMode::AUTO, false, false, false, false, 109999));
  CHECK(!m.update(RelayMode::AUTO, false, false, false, false, 110000));
}

void test_wire_round_trip() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(15);
  uint8_t req[8] = {1, 4, 0, 0, 0, 40};
  const uint16_t crc = crc16_modbus(req, 6);
  req[6] = uint8_t(crc & 0xFF);
  req[7] = uint8_t(crc >> 8);
  uint8_t resp[MB_MAX_FRAME];
  const size_t n = modbus_handle(1, req, 8, resp, g.regs);
  CHECK(n == 3 + 80 + 2);
  CHECK(resp[1] == 4 && resp[2] == 80);
  CHECK(resp[3] == 0x03 && resp[4] == 0x0A);
  CHECK(resp[3 + 2 * 6] == (5760 >> 8) && resp[4 + 2 * 6] == (5760 & 0xFF));
  CHECK(resp[3 + 2 * 19] == 0xFF && resp[4 + 2 * 19] == 0xFF);
  const uint16_t rcrc = crc16_modbus(resp, n - 2);
  CHECK(resp[n - 2] == (rcrc & 0xFF) && resp[n - 1] == (rcrc >> 8));

  req[5] = 41;  // 0-40: IR 40 doesn't exist
  const uint16_t crc2 = crc16_modbus(req, 6);
  req[6] = uint8_t(crc2 & 0xFF);
  req[7] = uint8_t(crc2 >> 8);
  CHECK(modbus_handle(1, req, 8, resp, g.regs) == 5 && resp[1] == 0x84 && resp[2] == MB_EX_ILLEGAL_ADDRESS);
}

}  // namespace

void test_diag_registers() {
  Rig g;  // defaults: Charge V 57.60, Max current 10 A
  CHECK(g.hr(130) == 0 && g.hr(131) == 0 && g.hr(132) == 0 && g.hr(133) == 0);
  CHECK(g.write1(130, 10) == MB_EX_ILLEGAL_VALUE);  // no voltage or modules yet
  const uint16_t blk[4] = {30, 5300, 0, 1};
  CHECK(g.regs.write_holding(130, blk, 4) == 0);
  Commands c = g.regs.take_commands();
  CHECK(c.diag_start && !c.diag_stop && c.diag_seconds == 30 && c.diag_v100 == 5300 && c.diag_a10 == 0 &&
        c.diag_mask == 1);
  CHECK(g.hr(131) == 5300 && g.hr(133) == 1);
  g.ctl.start_diag(53.0f, 0.0f, 1, 30, g.now);
  g.tel.now_ms = g.now;
  CHECK(g.hr(130) == 30);
  g.run_s(2);
  CHECK(g.ir(4) & 0x100);
  CHECK(g.write1(130, 0) == 0);
  c = g.regs.take_commands();
  CHECK(c.diag_stop && !c.diag_start);
  // Range checks: above Charge V, above Max current, bad mask, too long; nothing stored on a refusal.
  const uint16_t hi_v[3] = {5761, 0, 1}, hi_a[3] = {5300, 101, 1}, bad_mask[3] = {5300, 0, 4};
  CHECK(g.regs.write_holding(131, hi_v, 3) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.regs.write_holding(131, hi_a, 3) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.regs.write_holding(131, bad_mask, 3) == MB_EX_ILLEGAL_VALUE);
  // HR 132 is per module: both modules (mask 3) may only ask half of Max current each.
  const uint16_t both_hi[3] = {5300, 51, 3}, both_ok[3] = {5300, 50, 3};
  CHECK(g.regs.write_holding(131, both_hi, 3) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.regs.write_holding(131, both_ok, 3) == 0);
  CHECK(g.write1(132, 51) == MB_EX_ILLEGAL_VALUE);  // with mask 3 already stored
  const uint16_t one_full[3] = {5300, 100, 1};
  CHECK(g.regs.write_holding(131, one_full, 3) == 0);
  CHECK(g.write1(133, 3) == MB_EX_ILLEGAL_VALUE);  // 10 A on each would be 20 A
  CHECK(g.write1(131, 4199) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.write1(130, 601) == MB_EX_ILLEGAL_VALUE);
  CHECK(g.hr(131) == 5300);
  CHECK(g.write1(132, 100) == 0 && g.hr(132) == 100);
  CHECK(g.write1(133, 0) == 0);
  CHECK(g.write1(130, 5) == MB_EX_ILLEGAL_VALUE);  // mask 0: nothing to wake
  CHECK(!g.regs.take_commands().diag_start);
  uint16_t v;
  CHECK(!g.regs.read_holding(134, v) && g.write1(134, 1) == MB_EX_ILLEGAL_ADDRESS);
}

void test_external_soc_display() {
  Rig g;  // Panel: the controller knows no SOC
  g.tel.now_ms = 1000;
  CHECK(g.regs.external_soc(1000) == -1 && g.ir(13) == 0xFFFF);
  CHECK(g.write1(104, 57) == 0);
  CHECK(g.regs.external_soc(1000) == -1);  // no heartbeat yet
  CHECK(g.write1(100, 1) == 0);
  CHECK(g.regs.external_soc(1000) == 57 && g.ir(13) == 57);
  g.tel.now_ms = 1000 + EXT_FRESH_MS;
  CHECK(g.regs.external_soc(g.tel.now_ms) == 57);
  g.tel.now_ms = 1001 + EXT_FRESH_MS;
  CHECK(g.regs.external_soc(g.tel.now_ms) == -1 && g.ir(13) == 0xFFFF);  // heartbeat older than 10 s
  const uint16_t both[5] = {1, 0, 0, 0, 0xFFFF};  // a heartbeat with SOC "unknown"
  CHECK(g.regs.write_holding(100, both, 5) == 0);
  CHECK(g.regs.external_soc(g.tel.now_ms) == -1);
  CHECK(g.write1(104, 101) == 0 && g.regs.external_soc(g.tel.now_ms) == -1);  // out of range: not shown
  CHECK(g.write1(104, 100) == 0 && g.regs.external_soc(g.tel.now_ms) == 100);
}

int main() {
  test_status_registers();
  test_source_registers();
  test_modbus_source_snapshot();
  test_modbus_limits_in_controller();
  test_waiting_and_e17();
  test_psu_block();
  test_scaling();
  test_settings_writes();
  test_runtime_registers();
  test_clamps();
  test_relay_leaving_modbus();
  test_wire_round_trip();
  test_diag_registers();
  test_external_soc_display();
  return test_result("test_oem_modbus_map");
}
