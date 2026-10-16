#include "check.h"
#include "nasn_rectifier/nasn_protocol.h"

static bool frame_eq(const nasn::Frame &f, std::initializer_list<int> want) {
  size_t i = 0;
  for (int b : want) {
    if (f[i++] != b) return false;
  }
  return i == 8;
}

static void test_ids() {
  CHECK(nasn::read_id(1) == 0x108182FE);
  CHECK(nasn::read_id(2) == 0x108282FE);
  CHECK(nasn::write_id(1) == 0x108181FE);
  CHECK(nasn::write_id(2) == 0x108281FE);
}

static void test_requests() {
  CHECK(frame_eq(nasn::read_request(0x023B), {0x02, 0x3B, 0, 0, 0, 0, 0, 0}));
  // Matches the stock firmware's live mailbox: 57.0 V -> 0xE3C7.
  CHECK(frame_eq(nasn::write_voltage(nasn::reg::VOUT_SET, 57.0f), {0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7}));
  CHECK(frame_eq(nasn::write_standby(true), {0x01, 0x32, 0, 1, 0, 0, 0, 0}));
  CHECK(frame_eq(nasn::write_standby(false), {0x01, 0x32, 0, 0, 0, 0, 0, 0}));
  CHECK(frame_eq(nasn::write_current(nasn::reg::ILIMIT, 26.0f), {0x02, 0x2D, 0, 0, 0, 0, 0x02, 0x00}));
}

static void test_clamps() {
  CHECK(nasn::voltage_raw(60.0f) == 59334);  // 58.0 V hard cap
  CHECK(nasn::voltage_raw(30.0f) == 42966);  // 42.0 V floor
  CHECK(nasn::voltage_raw(NAN) == 42966);
  CHECK(nasn::current_raw(50.0f) == 984);
  CHECK(nasn::current_raw(80.0f) == 984);    // 50 A per PSU cap
  CHECK(nasn::current_raw(-5.0f) == 0);
  CHECK(nasn::current_raw(NAN) == 0);
}

static void test_parse_reply() {
  const uint8_t ok[8] = {0x01, 0x75, 0, 0, 0, 0, 0xD6, 0x00};
  auto r = nasn::parse_reply(0x1081827E, true, ok, 8);
  CHECK(r.has_value());
  CHECK(r->addr == 1);
  CHECK(r->cmd == nasn::CMD_READ);
  CHECK(r->status == 0);
  CHECK(r->reg == 0x0175);
  CHECK(r->value == 0xD600);
  CHECK_NEAR(nasn::telemetry(r->value), 53.5, 1e-6);

  const uint8_t err[8] = {0x21, 0x75, 0, 0, 0, 0, 0, 0};
  auto e = nasn::parse_reply(0x1082827E, true, err, 8);
  CHECK(e.has_value());
  CHECK(e->addr == 2);
  CHECK(e->status == 2);
  CHECK(e->reg == 0x0175);

  auto w = nasn::parse_reply(0x1081817E, true, ok, 8);
  CHECK(w.has_value() && w->cmd == nasn::CMD_WRITE);
}

static void test_parse_rejects() {
  const uint8_t d[8] = {0x01, 0x00, 0, 0, 0, 0, 0, 0};
  CHECK(!nasn::parse_reply(0x108182FE, true, d, 8));   // bit 7 set: a controller request
  CHECK(!nasn::parse_reply(0x1081827E, false, d, 8));  // standard ID
  CHECK(!nasn::parse_reply(0x1081827E, true, d, 6));   // short frame
  CHECK(!nasn::parse_reply(0x1081407E, true, d, 8));   // cmd 0x40 (Huawei data) not used here
  CHECK(!nasn::parse_reply(0x1001117E, true, d, 8));   // proto 0x20 unsolicited chatter
}

static void test_setpoint_readback() {
  CHECK_NEAR(nasn::setpoint_volts(0xE3C7), 57.0, 0.001);
  CHECK_NEAR(nasn::setpoint_amps(512), 26.03, 0.01);
}

static void test_readback_mismatch() {
  using nasn::reg::ILIMIT;
  using nasn::reg::STANDBY;
  using nasn::reg::VOUT_READBACK;
  const uint16_t v57 = nasn::voltage_raw(57.0f);
  // The stock firmware confirms VOUT against 0x022F (same x1023 scale as 0x0100), not 0x0100.
  CHECK(VOUT_READBACK == 0x022F);
  CHECK(!nasn::readback_mismatch(VOUT_READBACK, v57, v57));
  CHECK(!nasn::readback_mismatch(VOUT_READBACK, nasn::voltage_raw(57.14f), v57));  // within 0.15 V
  CHECK(nasn::readback_mismatch(VOUT_READBACK, nasn::voltage_raw(57.2f), v57));
  CHECK(nasn::readback_mismatch(VOUT_READBACK, nasn::voltage_raw(54.0f), v57));
  CHECK(!nasn::readback_mismatch(nasn::reg::VOUT_SET, 0, v57));  // 0x0100 is published, never compared
  const uint16_t a20 = nasn::current_raw(20.0f);
  CHECK(!nasn::readback_mismatch(ILIMIT, nasn::current_raw(20.9f), a20));  // within 1 A
  CHECK(nasn::readback_mismatch(ILIMIT, nasn::current_raw(21.2f), a20));
  CHECK(nasn::readback_mismatch(ILIMIT, 0, a20));
  // Standby: compare as a flag, nonzero = standby, whatever the byte layout of the readback.
  CHECK(!nasn::readback_mismatch(STANDBY, 1, 1));
  CHECK(!nasn::readback_mismatch(STANDBY, 0x000100000000ull, 1));  // same layout as the write
  CHECK(!nasn::readback_mismatch(STANDBY, 0, 0));
  CHECK(nasn::readback_mismatch(STANDBY, 0, 1));
  CHECK(nasn::readback_mismatch(STANDBY, 1, 0));
  CHECK(!nasn::readback_mismatch(nasn::reg::VOUT, 0, 12345));  // not a setpoint register
}

static void test_stored_default_mismatch() {
  using nasn::reg::ILIMIT_DEFAULT;
  using nasn::reg::VOUT_DEFAULT;
  const uint16_t v54 = nasn::voltage_raw(54.0f);
  CHECK(!nasn::stored_default_mismatch(VOUT_DEFAULT, v54, v54));
  CHECK(!nasn::stored_default_mismatch(VOUT_DEFAULT, v54 + 1, v54));  // one count of rounding
  CHECK(nasn::stored_default_mismatch(VOUT_DEFAULT, nasn::voltage_raw(54.1f), v54));
  const uint16_t a5 = nasn::current_raw(5.0f);
  CHECK(!nasn::stored_default_mismatch(ILIMIT_DEFAULT, a5, a5));
  CHECK(nasn::stored_default_mismatch(ILIMIT_DEFAULT, nasn::current_raw(6.0f), a5));
  CHECK(!nasn::stored_default_mismatch(nasn::reg::VOUT_SET, 0, v54));  // not a default register
}

static void test_persist_timer() {
  nasn::PersistTimer t(10000);
  CHECK(!t.update(false, 0));
  CHECK(!t.update(true, 1000));
  CHECK(!t.update(true, 10999));
  CHECK(t.update(true, 11000));   // continuously true for 10 s
  CHECK(t.active());
  CHECK(!t.update(false, 12000));  // any good reading restarts it
  CHECK(!t.update(true, 13000));
  CHECK(!t.update(true, 22999));
  t.reset();
  CHECK(!t.update(true, 30000));
  CHECK(!t.active());
  CHECK(t.update(true, 40000));
}

static void test_alarm_word() {
  // 0x0183 reply value is data[2..7]; the stock firmware reads data[4..7] as a big-endian u32.
  CHECK(nasn::STOCK_ALARM_MASK == 0x0852501Fu);  // OR of the E1-E11 bits below
  CHECK(nasn::alarm_word(0x000000400000ull) == 0x00400000u);  // bit 22 (E1)
  CHECK((nasn::alarm_word(0x000000400000ull) & nasn::STOCK_ALARM_MASK) != 0);
  CHECK((nasn::alarm_word(0x000000000020ull) & nasn::STOCK_ALARM_MASK) == 0);  // bit 5: not a stock fault
  CHECK(nasn::alarm_word(0xABCD12345678ull) == 0x12345678u);  // data[2..3] dropped
  CHECK((nasn::alarm_word(0x000008000000ull) & nasn::STOCK_ALARM_MASK) != 0);  // bit 27 (E6), in data[4]
  uint32_t m = 0;
  for (int b : {0, 1, 2, 3, 4, 12, 14, 17, 20, 22, 27}) m |= 1u << b;
  CHECK(m == nasn::STOCK_ALARM_MASK);
  CHECK((nasn::alarm_word(0x000000020000ull) & nasn::STOCK_ALARM_MASK) != 0);  // bit 17 (E2)
  CHECK((nasn::alarm_word(0x000000050000ull) & nasn::STOCK_ALARM_MASK) == 0);  // bits 16, 18: not stock faults
}

static void test_output_voltage() {
  CHECK(nasn::reg::VOUT_ALT == 0x0184);
  CHECK_NEAR(nasn::output_voltage(53.5f, 53.7f), 53.7, 1e-6);  // the higher reading wins
  CHECK_NEAR(nasn::output_voltage(54.0f, 53.0f), 54.0, 1e-6);
  CHECK_NEAR(nasn::output_voltage(NAN, 53.0f), 53.0, 1e-6);    // either one alone is enough
  CHECK_NEAR(nasn::output_voltage(52.0f, NAN), 52.0, 1e-6);
  CHECK(std::isnan(nasn::output_voltage(NAN, NAN)));
}

int main() {
  test_ids();
  test_output_voltage();
  test_alarm_word();
  test_requests();
  test_clamps();
  test_parse_reply();
  test_parse_rejects();
  test_setpoint_readback();
  test_readback_mismatch();
  test_stored_default_mismatch();
  test_persist_timer();
  return test_result("nasn_protocol");
}
