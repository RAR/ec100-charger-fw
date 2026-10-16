#include <cmath>
#include <cstring>

#include "app/crc16.h"
#include "app/jkpb_protocol.h"
#include "app/jkpb_source.h"
#include "check.h"

using namespace oem::jkpb;
using oem::JkpbSource;
using oem::Limits;

// docs/history/notes/jk-pb-capture.txt (docs/history/notes/jk-pb-protocol.md §6): TV1, the slave 01 cell-info record
// and its FC10 trailer.
static const char *TV1_HEX =
    "55AAEB900205C40CC40CC40CC40CC40CC40CC40CC30CC40CC40CC40CC40CC40CC40CC40CC40C000000000000000000000000000000000000"
    "0000000000000000000000000000FFFF0000C40C010000053A0037003A0037003A0038003A0037003B0039003B0038003B003A003C003A00"
    "0000000000000000000000000000000000000000000000000000000000000000AE00000000003FCC00000000000000000000A700AC000000"
    "080000000040CA14030090CA0400000000001602000064000000A8FF030001010000000000000000000000000000FF0001000000A8030000"
    "00009CD73E40000000006C1400000001010100060000F46B260000000000AE00AB00A900A40380B81F09030000008051010000000100496D"
    "00000000000000FEFF7FDC2F0101B00F0000002D";
static const char *TV1_TRAILER_HEX = "011016200001044B";
// The slave 01 settings record and its trailer.
static const char *TV3_HEX =
    "55AAEB900105AC0D0000140A00003C0A0000420E0000FC0D00000A000000060E0000280A0000100E0000AC0D0000C4090000409C00000300"
    "00003C000000F04902002C0100003C00000005000000E8030000BC02000058020000BC020000580200001400000032000000E80300002003"
    "00001000000001000000010000000100000090CA0400DC050000B80B00000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010000000000000060E3"
    "160000323C3218FEFFFFFF9FE91D0200000000E6";
static const char *TV3_TRAILER_HEX = "0110161E00016587";

// docs/history/notes/jk-pb-capture-rs485-1.txt: a real PB2A16S20P (sw 19.34, DIP 4) on RS485-1, protocol 001: the
// 308-byte cell-info and settings units (record + FC10 trailer) answering the firmware's own requests.
static const char *RS1_CELL_HEX =
    "55AAEB9002001C0D210D1D0D1D0D1D0D1E0D1E0D200D1E0D1D0D1E0D1E0D1D0D1E0D200D1D0D000000000000000000000000000000000000"
    "0000000000000000000000000000FFFF00001E0D040002004E006700640073006A008500860085007A008300650077006A00680050006300"
    "0000000000000000000000000000000000000000000000000000000000000000FD0000000000E3D100007525120075560000EE00F2000000"
    "0000000000461D5E030090CA040079000000A404480264000000F1C8A20101010000000000000000000000000000FF00010000000000A600"
    "00001C0A3F4000000000FD140000000101010006000044A5740000000000FD00F200EE0010274638C60C4A00000080510100000002000000"
    "00000000000000FEFF7FDD2F0121B00F07000049041016200001041E";
static const char *RS1_SETTINGS_HEX =
    "55AAEB900100AC0D0000140A00003C0A0000420E0000DC0D00000A000000DD0D0000280A0000DE0D0000160D0000BA090000400D03000300"
    "00003C000000400D03002C0100003C0000001E000000D0070000F4010000C2010000F4010000C2010000140000004600000020030000BC02"
    "00001000000001000000010000000100000090CA040005000000B80B00000000000000000000000000000000000000000000000000000000"
    "0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
    "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000040000000000000020A1"
    "07000000F60018FEFFFFFFBFE9ED0F00DDE200210410161E000165D2";

constexpr size_t UNIT = RECORD_LEN + TRAILER_LEN;

static int hexval(char c) { return c <= '9' ? c - '0' : c - 'A' + 10; }

static size_t unhex(const char *h, uint8_t *f) {
  size_t n = 0;
  for (const char *p = h; *p; p += 2) f[n++] = uint8_t(hexval(p[0]) << 4 | hexval(p[1]));
  return n;
}

static void put_u32(uint8_t *r, size_t i, uint32_t v) {
  for (int k = 0; k < 4; k++) r[i + k] = uint8_t(v >> (8 * k));
}

static void reseal(uint8_t *r) {
  uint8_t sum = 0;
  for (size_t i = 0; i < RECORD_LEN - 1; i++) sum = uint8_t(sum + r[i]);
  r[RECORD_LEN - 1] = sum;
}

// An FC10 reply `addr 10 RH RL 00 01 CRC` with a good CRC.
static void trailer(uint8_t addr, uint16_t reg, uint8_t *t) {
  const uint8_t b[6] = {addr, 0x10, uint8_t(reg >> 8), uint8_t(reg), 0x00, 0x01};
  for (int i = 0; i < 6; i++) t[i] = b[i];
  const uint16_t crc = oem::crc16_modbus(t, 6);
  t[6] = uint8_t(crc);
  t[7] = uint8_t(crc >> 8);
}

// TV1 with patched fields, resealed, then its trailer for addr: a 308-byte unit.
struct CellOpt {
  uint32_t alarms = 0x00080000;
  uint8_t mos = 1, stage = 0, soc = 64;
};

static size_t cell_unit(uint8_t *u, uint8_t addr = 1, const CellOpt &o = CellOpt()) {
  unhex(TV1_HEX, u);
  put_u32(u, 166, o.alarms);
  u[173] = o.soc;
  u[198] = o.mos;
  u[280] = o.stage;
  reseal(u);
  trailer(addr, REG_CELL_INFO, u + RECORD_LEN);
  return UNIT;
}

// TV3 (settings) with patched fields, resealed, then its trailer for addr.
struct SetOpt {
  uint32_t cells = 16, rcv = 3600, rfv = 3500, coc = 40000, sw = 1;
};

static size_t settings_unit(uint8_t *u, uint8_t addr = 1, const SetOpt &o = SetOpt()) {
  unhex(TV3_HEX, u);
  put_u32(u, 38, o.rcv);
  put_u32(u, 42, o.rfv);
  put_u32(u, 50, o.coc);
  put_u32(u, 114, o.cells);
  put_u32(u, 118, o.sw);
  reseal(u);
  trailer(addr, REG_SETTINGS, u + RECORD_LEN);
  return UNIT;
}

static void test_requests() {
  uint8_t o[REQUEST_LEN];
  const uint8_t c1[11] = {0x01, 0x10, 0x16, 0x20, 0x00, 0x01, 0x02, 0x00, 0x00, 0xD6, 0xF1};
  const uint8_t c0[11] = {0x00, 0x10, 0x16, 0x20, 0x00, 0x01, 0x02, 0x00, 0x00, 0xDB, 0x61};
  const uint8_t s1[11] = {0x01, 0x10, 0x16, 0x1E, 0x00, 0x01, 0x02, 0x00, 0x00, 0xD2, 0x2F};
  const uint8_t s0[11] = {0x00, 0x10, 0x16, 0x1E, 0x00, 0x01, 0x02, 0x00, 0x00, 0xDF, 0xBF};
  CHECK(REQUEST_LEN == 11 && REQUEST_LEN <= oem::POLL_REQ_MAX);
  CHECK(build_request(1, REG_CELL_INFO, o) == 11 && memcmp(o, c1, 11) == 0);
  CHECK(build_request(0, REG_CELL_INFO, o) == 11 && memcmp(o, c0, 11) == 0);
  CHECK(build_request(1, REG_SETTINGS, o) == 11 && memcmp(o, s1, 11) == 0);
  CHECK(build_request(0, REG_SETTINGS, o) == 11 && memcmp(o, s0, 11) == 0);
}

static void test_tv1() {
  uint8_t r[RECORD_LEN];
  CHECK(unhex(TV1_HEX, r) == RECORD_LEN);
  CHECK(record_ok(r, TYPE_CELL_INFO));
  CHECK(!record_ok(r, TYPE_SETTINGS));
  const CellInfo c = decode_cell_info(r);
  CHECK_NEAR(c.pack_v, 52.287, 0.0005);
  CHECK(c.soc == 64 && c.alarms == 0x00080000 && c.charge_mos && c.stage == 0);
  CHECK_NEAR(c.max_cell_v, 3.268, 0.0005);
}

static void test_settings() {
  uint8_t r[RECORD_LEN];
  CHECK(unhex(TV3_HEX, r) == RECORD_LEN);
  CHECK(record_ok(r, TYPE_SETTINGS));
  CHECK(!record_ok(r, TYPE_CELL_INFO));
  ChargeSettings s{};
  CHECK(decode_settings(r, s));
  CHECK(s.cells == 16 && s.rcv_mv == 3600 && s.rfv_mv == 3500 && s.coc_ma == 40000 && s.charge_switch);
  // the build helpers reproduce the captures exactly
  uint8_t u[UNIT];
  settings_unit(u);
  CHECK(memcmp(u, r, RECORD_LEN) == 0);
  cell_unit(u);
  unhex(TV1_HEX, r);
  CHECK(memcmp(u, r, RECORD_LEN) == 0);
}

static void test_trailers() {
  uint8_t t1[TRAILER_LEN], t3[TRAILER_LEN], b[TRAILER_LEN];
  CHECK(unhex(TV1_TRAILER_HEX, t1) == TRAILER_LEN && unhex(TV3_TRAILER_HEX, t3) == TRAILER_LEN);
  CHECK(trailer_ok(t1, 1, REG_CELL_INFO));
  CHECK(trailer_ok(t3, 1, REG_SETTINGS));
  CHECK(!trailer_ok(t1, 0, REG_CELL_INFO));
  CHECK(!trailer_ok(t1, 1, REG_SETTINGS));
  CHECK(!trailer_ok(t3, 1, REG_CELL_INFO));
  for (int k = 6; k < 8; k++) {
    memcpy(b, t1, TRAILER_LEN);
    b[k] ^= 0x01;
    CHECK(!trailer_ok(b, 1, REG_CELL_INFO));
  }
  trailer(1, REG_CELL_INFO, b);
  CHECK(memcmp(b, t1, TRAILER_LEN) == 0);
  trailer(0, REG_CELL_INFO, b);  // the master's
  CHECK(trailer_ok(b, 0, REG_CELL_INFO) && !trailer_ok(b, 1, REG_CELL_INFO));
}

static void test_rejections() {
  uint8_t r[RECORD_LEN];
  unhex(TV1_HEX, r);
  r[100] ^= 0x01;
  CHECK(!record_ok(r, TYPE_CELL_INFO));  // sum
  unhex(TV1_HEX, r);
  r[4] = 0x03;
  reseal(r);
  CHECK(!record_ok(r, TYPE_CELL_INFO) && !record_ok(r, TYPE_SETTINGS));
  unhex(TV1_HEX, r);
  r[2] = 0xEC;
  reseal(r);
  CHECK(!record_ok(r, TYPE_CELL_INFO));  // header
  ChargeSettings s{};
  uint8_t u[UNIT];
  struct Case {
    SetOpt o;
    bool ok;
  };
  Case cases[8];
  for (Case &c : cases) c = {SetOpt(), true};
  cases[0].o.cells = 0, cases[0].ok = false;
  cases[1].o.cells = 33, cases[1].ok = false;
  cases[2].o.rcv = 2400, cases[2].ok = false;
  cases[3].o.rfv = 4501, cases[3].ok = false;
  cases[4].o.cells = 1;
  cases[5].o.cells = 32;
  cases[6].o.rcv = 2500, cases[6].o.rfv = 4500;
  cases[7].o.sw = 0;
  for (const Case &c : cases) {
    settings_unit(u, 1, c.o);
    CHECK(record_ok(u, TYPE_SETTINGS));
    CHECK(decode_settings(u, s) == c.ok);
  }
  CHECK(!s.charge_switch);  // the last case decodes with the switch off
}

static void test_cell_fields() {
  uint8_t u[UNIT];
  cell_unit(u);
  u[6] = 0x87, u[7] = 0x13;   // cell 1 4999 mV counts
  u[8] = 0x88, u[9] = 0x13;   // cell 2 5000 mV ignored
  u[173] = 101;               // SOC out of range
  u[198] = 2;                 // only 1 is "on"
  CellInfo c = decode_cell_info(u);
  CHECK_NEAR(c.max_cell_v, 4.999, 0.0005);
  CHECK(c.soc == -1 && !c.charge_mos);
  for (size_t i = 6; i < 70; i++) u[i] = 0;
  c = decode_cell_info(u);
  CHECK(std::isnan(c.max_cell_v));
  for (size_t i = 6; i < 70; i += 2) u[i] = 0x88, u[i + 1] = 0x13;  // all 5000 mV
  CHECK(std::isnan(decode_cell_info(u).max_cell_v));
  u[6 + 2 * 31] = 0xC4, u[7 + 2 * 31] = 0x0C;  // cell 32 counts
  CHECK_NEAR(decode_cell_info(u).max_cell_v, 3.268, 0.0005);
}

// ---- the source ----

static void expect_req(JkpbSource &p, uint32_t t, uint8_t addr, uint16_t reg) {
  uint8_t out[oem::POLL_REQ_MAX], want[REQUEST_LEN];
  CHECK(p.poll(t, out) == REQUEST_LEN);
  build_request(addr, reg, want);
  CHECK(memcmp(out, want, REQUEST_LEN) == 0);
}

static void feed(JkpbSource &p, const uint8_t *f, size_t n, uint32_t now) { p.push(f, n, now); }

// Scan 0 (silent), lock on 1 at 1000, settings at 2000, cell info at 3000: fresh from 3050.
static void lock_fresh(JkpbSource &p, const CellOpt &c = CellOpt(), const SetOpt &s = SetOpt()) {
  uint8_t u[UNIT];
  expect_req(p, 0, 0, REG_CELL_INFO);
  expect_req(p, 1000, 1, REG_CELL_INFO);
  feed(p, u, cell_unit(u, 1, c), 1050);
  expect_req(p, 2000, 1, REG_SETTINGS);
  feed(p, u, settings_unit(u, 1, s), 2050);
  expect_req(p, 3000, 1, REG_CELL_INFO);
  feed(p, u, cell_unit(u, 1, c), 3050);
}

static void test_scan_and_lock() {
  JkpbSource p;
  oem::Settings s = oem::default_settings();
  uint8_t u[UNIT], out[oem::POLL_REQ_MAX];
  CHECK(p.baud() == 115200 && !p.locked() && !p.fresh(0));
  expect_req(p, 0, 0, REG_CELL_INFO);
  CHECK(p.poll(999, out) == 0);
  expect_req(p, 1000, 1, REG_CELL_INFO);  // 0 silent past the window
  feed(p, u, cell_unit(u, 1), 1050);
  CHECK(p.locked() && p.address() == 1 && p.good_replies() == 1 && p.bad_replies() == 0);
  CHECK(!p.fresh(1050) && p.limits(s, 1050).stale);  // cell info alone
  expect_req(p, 2000, 1, REG_SETTINGS);               // first locked request
  feed(p, u, settings_unit(u, 1), 2050);
  CHECK(p.fresh(2050) && p.good_replies() == 2);
  expect_req(p, 3000, 1, REG_CELL_INFO);
  feed(p, u, cell_unit(u, 1), 3050);
  CHECK(p.fresh(3050));
  // the scan visits 0..15 and wraps
  JkpbSource q;
  for (uint32_t k = 0; k < 18; k++) expect_req(q, 1000 * k, uint8_t(k % 16), REG_CELL_INFO);
  CHECK(!q.locked());
  // address 0 locks too
  JkpbSource z;
  expect_req(z, 0, 0, REG_CELL_INFO);
  feed(z, u, cell_unit(u, 0), 30);
  CHECK(z.locked() && z.address() == 0);
  expect_req(z, 1000, 0, REG_SETTINGS);
}

static void test_limits() {
  oem::Settings s = oem::default_settings();
  {
    JkpbSource p;
    lock_fresh(p);
    const Limits l = p.limits(s, 3100);
    CHECK(l.fresh && !l.stale && l.source == oem::SourceMode::JKPB);
    CHECK_NEAR(l.cvl, 57.6, 0.001);
    CHECK_NEAR(l.ccl, 40.0, 0.001);
    CHECK(l.allowed && !l.alarm && !l.blocking && l.soc == 64);
    CHECK_NEAR(l.battery_v, 52.287, 0.001);
    CHECK(std::isnan(l.battery_a));
    CHECK_NEAR(l.max_cell_v, 3.268, 0.001);
  }
  {
    JkpbSource p;
    CellOpt c;
    c.stage = 2;  // float
    lock_fresh(p, c);
    CHECK_NEAR(p.limits(s, 3100).cvl, 56.0, 0.001);
    c.stage = 1;
    JkpbSource q;
    lock_fresh(q, c);
    CHECK_NEAR(q.limits(s, 3100).cvl, 57.6, 0.001);
  }
  {
    JkpbSource p;
    SetOpt o;
    o.cells = 15, o.rcv = 3450, o.coc = 25500;
    lock_fresh(p, CellOpt(), o);
    CHECK_NEAR(p.limits(s, 3100).cvl, 51.75, 0.001);
    CHECK_NEAR(p.limits(s, 3100).ccl, 25.5, 0.001);
  }
}

static void test_alarm_bits() {
  oem::Settings s = oem::default_settings();
  for (int b = 0; b < 32; b++) {
    JkpbSource p;
    CellOpt c;
    c.alarms = 1u << b;
    lock_fresh(p, c);
    const Limits l = p.limits(s, 3100);
    CHECK(l.fresh && !l.stale);
    const bool stop = (oem::JKPB_ALARM_MASK >> b) & 1;
    if (stop) {
      CHECK(l.alarm && !l.allowed);  // E12
    } else if (b == 4) {
      CHECK(!l.alarm && !l.allowed);  // cell OVP: hold, no E12
    } else {
      CHECK(!l.alarm && l.allowed);  // UVP, discharge bits, bit 19 ...
    }
  }
  CHECK(oem::JKPB_ALARM_MASK == 0x00E107EF && oem::JKPB_CELL_OVP == 0x10);
  {
    JkpbSource p;
    CellOpt c;
    c.alarms = 0x00080800;  // bit 19 + bit 11 (cell UVP)
    lock_fresh(p, c);
    CHECK(p.limits(s, 3100).allowed && !p.limits(s, 3100).alarm);
  }
  {
    JkpbSource p;
    CellOpt c;
    c.mos = 0;
    lock_fresh(p, c);
    const Limits l = p.limits(s, 3100);
    CHECK(l.fresh && !l.allowed && !l.alarm);
  }
  {
    JkpbSource p;
    SetOpt o;
    o.sw = 0;
    lock_fresh(p, CellOpt(), o);
    const Limits l = p.limits(s, 3100);
    CHECK(l.fresh && !l.allowed && !l.alarm);
  }
}

static void test_cadence() {
  JkpbSource p;
  uint8_t u[UNIT];
  lock_fresh(p);  // settings requested at 2000
  for (uint32_t t = 4000; t < 62000; t += 1000) {
    expect_req(p, t, 1, REG_CELL_INFO);
    feed(p, u, cell_unit(u), t + 40);
  }
  expect_req(p, 62000, 1, REG_SETTINGS);  // 60 s after the last settings request
  feed(p, u, settings_unit(u), 62040);
  expect_req(p, 63000, 1, REG_CELL_INFO);
  CHECK(p.fresh(63000) && p.bad_replies() == 0);
}

static void test_settings_retry_and_invalid() {
  oem::Settings s = oem::default_settings();
  uint8_t u[UNIT];
  JkpbSource p;
  expect_req(p, 0, 0, REG_CELL_INFO);
  feed(p, u, cell_unit(u, 0), 40);
  expect_req(p, 1000, 0, REG_SETTINGS);  // no answer: asked again
  expect_req(p, 2000, 0, REG_SETTINGS);
  SetOpt bad;
  bad.cells = 33;
  feed(p, u, settings_unit(u, 0, bad), 2040);
  CHECK(p.bad_replies() == 1 && p.good_replies() == 1);
  // the pack answered: no miss, so invalid replies never drop the lock; settings stay unknown
  for (uint32_t t = 3000; t <= 6000; t += 1000) {
    expect_req(p, t, 0, REG_SETTINGS);
    CHECK(p.locked() && !p.fresh(t) && p.limits(s, t).stale);
    feed(p, u, settings_unit(u, 0, bad), t + 40);
  }
  expect_req(p, 7000, 0, REG_SETTINGS);  // still unknown
  CHECK(p.locked() && !p.fresh(7000) && p.limits(s, 7000).stale);
  feed(p, u, settings_unit(u, 0), 7040);
  expect_req(p, 8000, 0, REG_CELL_INFO);
  feed(p, u, cell_unit(u, 0), 8040);
  CHECK(p.fresh(8040) && p.locked());
  // an invalid settings record on the periodic refresh drops the settings
  for (uint32_t t = 9000; t < 67000; t += 1000) {
    expect_req(p, t, 0, REG_CELL_INFO);
    feed(p, u, cell_unit(u, 0), t + 40);
  }
  expect_req(p, 67000, 0, REG_SETTINGS);
  SetOpt low;
  low.rcv = 2400;
  feed(p, u, settings_unit(u, 0, low), 67040);
  CHECK(!p.fresh(67040));
  expect_req(p, 68000, 0, REG_SETTINGS);
}

static void test_stale_and_latch() {
  oem::Settings s = oem::default_settings();
  uint8_t u[UNIT];
  {
    JkpbSource p;  // never heard: Panel limits, stale, allowed
    const Limits l = p.limits(s, 0);
    CHECK(l.stale && l.allowed && l.source == oem::SourceMode::JKPB);
  }
  {
    JkpbSource p;  // 3 s without a reply
    lock_fresh(p);
    CHECK(p.fresh(6050) && !p.fresh(6051));
    s.set(oem::HR_MAX_CURRENT, 25);
    s.set(oem::HR_FALLBACK_A, 4);
    const Limits l = p.limits(s, 6051);
    CHECK(l.stale && l.allowed);
    CHECK_NEAR(l.ccl, 4.0, 0.001);
    CHECK_NEAR(l.cvl, float(s.get(oem::HR_CHARGE_V)) / 100.0f, 0.001);
    s.set(oem::HR_FALLBACK_A, 30);
    CHECK_NEAR(p.limits(s, 6051).ccl, 25.0, 0.001);
    s = oem::default_settings();
  }
  {
    JkpbSource p;  // an alarm then silence: held off, though stale; a fresh allowed record clears it
    CellOpt c;
    c.alarms = 0x20;
    lock_fresh(p, c);
    CHECK(!p.limits(s, 6051).allowed && p.limits(s, 6051).stale);
    expect_req(p, 7000, 1, REG_CELL_INFO);
    feed(p, u, cell_unit(u), 7050);
    CHECK(p.limits(s, 7050).allowed && !p.limits(s, 7050).stale);
  }
  {
    JkpbSource p;  // a stop record while not fresh (no settings yet) still latches
    CellOpt c;
    c.alarms = oem::JKPB_CELL_OVP;
    expect_req(p, 0, 0, REG_CELL_INFO);
    feed(p, u, cell_unit(u, 0, c), 40);
    CHECK(!p.fresh(40) && !p.limits(s, 40).allowed);
    expect_req(p, 1000, 0, REG_SETTINGS);
    feed(p, u, settings_unit(u, 0), 1040);  // fresh now, but the record says stop
    CHECK(p.fresh(1040) && !p.limits(s, 1040).allowed);
    expect_req(p, 2000, 0, REG_CELL_INFO);
    feed(p, u, cell_unit(u, 0), 2040);
    CHECK(p.limits(s, 2040).allowed);
  }
  {
    JkpbSource p;  // MOS off latches
    CellOpt c;
    c.mos = 0;
    lock_fresh(p, c);
    CHECK(!p.limits(s, 6051).allowed);
  }
  {
    JkpbSource p;  // the charge switch off latches
    SetOpt o;
    o.sw = 0;
    lock_fresh(p, CellOpt(), o);
    CHECK(!p.limits(s, 6051).allowed);
  }
  {
    JkpbSource p;  // CurBatCOC 0 means don't charge: held off while fresh, latched through silence
    SetOpt o;
    o.coc = 0;
    lock_fresh(p, CellOpt(), o);
    CHECK(p.fresh(3050) && !p.limits(s, 3050).allowed);
    CHECK(p.limits(s, 6051).stale && !p.limits(s, 6051).allowed);
    for (uint32_t t = 4000; t <= 7000; t += 1000) p.poll(t, u);  // 3 misses: unlocked, scanning from 1
    CHECK(!p.locked() && !p.limits(s, 7000).allowed);
    feed(p, u, cell_unit(u, 1), 7050);  // relocks: still held off
    CHECK(p.locked() && !p.limits(s, 7050).allowed);
    expect_req(p, 8000, 1, REG_SETTINGS);
    feed(p, u, settings_unit(u, 1), 8050);  // COC back above 0 and fresh: allowed
    CHECK(p.fresh(8050) && p.limits(s, 8050).allowed);
    CHECK_NEAR(p.limits(s, 8050).ccl, 40.0, 0.001);
  }
  {
    JkpbSource p;  // losing the lock keeps the latch; reset() forgets everything
    CellOpt c;
    c.alarms = 0x01;
    lock_fresh(p, c);
    for (uint32_t t = 4000; t <= 7000; t += 1000) p.poll(t, u);
    CHECK(!p.locked() && !p.limits(s, 7000).allowed);
    p.reset();
    CHECK(!p.locked() && p.good_replies() == 0 && p.bad_replies() == 0 && !p.fresh(7000));
    CHECK(p.limits(s, 7000).allowed && p.limits(s, 7000).stale);
    expect_req(p, 7000, 0, REG_CELL_INFO);  // scanning from 0 again
  }
}

static void test_unlock_and_rescan() {
  uint8_t u[UNIT];
  JkpbSource p;
  lock_fresh(p);                          // locked on 1, answered at 3000
  expect_req(p, 4000, 1, REG_CELL_INFO);  // no answer from here
  expect_req(p, 5000, 1, REG_CELL_INFO);  // miss 1
  expect_req(p, 6000, 1, REG_CELL_INFO);  // miss 2
  CHECK(p.locked());
  expect_req(p, 7000, 1, REG_CELL_INFO);  // miss 3: unlocked, scanning from 1
  CHECK(!p.locked() && !p.fresh(7000));
  expect_req(p, 8000, 2, REG_CELL_INFO);
  expect_req(p, 9000, 3, REG_CELL_INFO);
  feed(p, u, cell_unit(u, 3), 9040);
  CHECK(p.locked() && p.address() == 3 && !p.fresh(9040));  // settings forgotten
  expect_req(p, 10000, 3, REG_SETTINGS);
  // a good reply between misses resets the count
  JkpbSource q;
  lock_fresh(q);
  expect_req(q, 4000, 1, REG_CELL_INFO);
  expect_req(q, 5000, 1, REG_CELL_INFO);
  expect_req(q, 6000, 1, REG_CELL_INFO);
  feed(q, u, cell_unit(u), 6040);
  expect_req(q, 7000, 1, REG_CELL_INFO);
  expect_req(q, 8000, 1, REG_CELL_INFO);
  expect_req(q, 9000, 1, REG_CELL_INFO);
  CHECK(q.locked());
}

static void test_window_and_rejects() {
  uint8_t u[UNIT], out[oem::POLL_REQ_MAX];
  {
    JkpbSource p;
    feed(p, u, cell_unit(u, 0), 5);  // no request yet
    CHECK(p.good_replies() == 0 && p.bad_replies() == 0);
    expect_req(p, 100, 0, REG_CELL_INFO);
    feed(p, u, UNIT, 851);  // 751 ms after the request: too late
    CHECK(p.good_replies() == 0 && !p.locked());
    expect_req(p, 1100, 1, REG_CELL_INFO);
    feed(p, u, cell_unit(u, 1), 1850);  // 750 ms: in time
    CHECK(p.good_replies() == 1 && p.locked());
    feed(p, u, UNIT, 1860);  // already answered
    CHECK(p.good_replies() == 1 && p.bad_replies() == 0);
  }
  {
    JkpbSource p;  // a master record (trailer address 00) while polling 01
    expect_req(p, 0, 0, REG_CELL_INFO);
    expect_req(p, 1000, 1, REG_CELL_INFO);
    feed(p, u, cell_unit(u, 0), 1040);
    CHECK(p.bad_replies() == 1 && p.good_replies() == 0 && !p.locked());
    feed(p, u, cell_unit(u, 1), 1080);  // the real one still counts
    CHECK(p.good_replies() == 1 && p.locked() && p.address() == 1);
  }
  {
    JkpbSource p;  // wrong record type for the request, and a trailer naming the other register
    expect_req(p, 0, 0, REG_CELL_INFO);
    feed(p, u, settings_unit(u, 0), 40);
    CHECK(p.bad_replies() == 1 && !p.locked());
    cell_unit(u, 0);
    trailer(0, REG_SETTINGS, u + RECORD_LEN);
    feed(p, u, UNIT, 60);
    CHECK(p.bad_replies() == 2 && !p.locked());
    cell_unit(u, 0);
    u[150] ^= 0x01;  // sum
    feed(p, u, UNIT, 80);
    CHECK(p.bad_replies() == 3 && !p.locked());
    feed(p, u, cell_unit(u, 0), 100);
    CHECK(p.locked() && p.good_replies() == 1);
  }
  {
    JkpbSource p;  // a record with no trailer times out cleanly
    expect_req(p, 0, 0, REG_CELL_INFO);
    cell_unit(u, 0);
    feed(p, u, RECORD_LEN, 40);
    CHECK(p.good_replies() == 0 && !p.locked());
    CHECK(p.poll(1000, out) == 0);  // in progress: held
    CHECK(p.poll(1000 + oem::JKPB_HOLD_MS - 1, out) == 0);
    expect_req(p, 1000 + oem::JKPB_HOLD_MS, 1, REG_CELL_INFO);  // a miss: the scan moves on
    CHECK(p.good_replies() == 0 && !p.locked());
    feed(p, u, cell_unit(u, 1), 1000 + oem::JKPB_HOLD_MS + 40);
    CHECK(p.good_replies() == 1 && p.locked());
  }
}

static void test_resync() {
  uint8_t u[UNIT], g[2 * UNIT];
  {
    JkpbSource p;  // stray bytes and partial headers before the reply
    expect_req(p, 0, 0, REG_CELL_INFO);
    const uint8_t pre[7] = {0x00, 0x55, 0x55, 0xAA, 0xEB, 0x55, 0xAA};
    feed(p, pre, 7, 5);
    feed(p, u, cell_unit(u, 0), 10);
    CHECK(p.good_replies() == 1 && p.locked());
  }
  {
    JkpbSource p;  // a broken unit that swallowed the start of the real one: replay finds it
    expect_req(p, 0, 0, REG_CELL_INFO);
    size_t n = 0;
    const uint8_t h[5] = {0x55, 0xAA, 0xEB, 0x90, 0x02};
    for (uint8_t b : h) g[n++] = b;
    for (int i = 0; i < 100; i++) g[n++] = uint8_t(i == 50 ? 0x55 : 0x11);  // a lone 55 inside too
    n += cell_unit(g + n, 0);
    feed(p, g, n, 10);
    CHECK(p.good_replies() == 1 && p.bad_replies() >= 1 && p.locked());
  }
  {
    JkpbSource p;  // the replay itself breaks on a stray 55 right before the real header: still in stream order
    expect_req(p, 0, 0, REG_CELL_INFO);
    size_t n = 0;
    const uint8_t h[5] = {0x55, 0xAA, 0xEB, 0x90, 0x02};
    for (uint8_t b : h) g[n++] = b;
    for (int i = 0; i < 50; i++) g[n++] = 0x11;
    g[n++] = 0x55;
    n += cell_unit(g + n, 0);
    feed(p, g, n, 10);
    CHECK(p.good_replies() == 1 && p.locked());
  }
  {
    JkpbSource p;  // split across pushes, byte by byte
    expect_req(p, 0, 0, REG_CELL_INFO);
    const size_t n = cell_unit(u, 0);
    for (size_t i = 0; i < n; i++) feed(p, u + i, 1, uint32_t(10 + i / 10));
    CHECK(p.good_replies() == 1 && p.locked());
  }
  {
    JkpbSource p;  // header-like garbage never overflows; the next valid reply is accepted
    expect_req(p, 0, 0, REG_CELL_INFO);
    for (size_t i = 0; i < sizeof g; i++) g[i] = (i % 4 == 0) ? 0x55 : (i % 4 == 1) ? 0xAA : (i % 4 == 2) ? 0xEB : 0x90;
    feed(p, g, sizeof g, 5);
    expect_req(p, 1000 + oem::JKPB_HOLD_MS, 1, REG_CELL_INFO);
    feed(p, u, cell_unit(u, 1), 1900);
    CHECK(p.good_replies() == 1 && p.locked());
  }
}

static void test_rs485_1_capture() {
  uint8_t c[UNIT], st[UNIT];
  CHECK(unhex(RS1_CELL_HEX, c) == UNIT && unhex(RS1_SETTINGS_HEX, st) == UNIT);
  CHECK(record_ok(c, TYPE_CELL_INFO) && trailer_ok(c + RECORD_LEN, 4, REG_CELL_INFO));
  CHECK(record_ok(st, TYPE_SETTINGS) && trailer_ok(st + RECORD_LEN, 4, REG_SETTINGS));
  const CellInfo ci = decode_cell_info(c);
  CHECK_NEAR(ci.pack_v, 53.731, 0.0005);
  CHECK_NEAR(ci.max_cell_v, 3.361, 0.0005);
  CHECK(ci.soc == 70 && ci.alarms == 0 && ci.charge_mos && ci.stage == 0);
  ChargeSettings cs{};
  CHECK(decode_settings(st, cs));
  CHECK(cs.cells == 16 && cs.rcv_mv == 3550 && cs.rfv_mv == 3350 && cs.coc_ma == 200000 && cs.charge_switch);
  // the source scans 0..3 (silent), locks on 4, reads settings, then limits come from the pack
  JkpbSource p;
  oem::Settings s = oem::default_settings();
  for (uint32_t k = 0; k < 5; k++) expect_req(p, 1000 * k, uint8_t(k), REG_CELL_INFO);
  feed(p, c, UNIT, 4030);
  CHECK(p.locked() && p.address() == 4);
  expect_req(p, 5000, 4, REG_SETTINGS);
  feed(p, st, UNIT, 5030);
  CHECK(p.fresh(5030) && p.bad_replies() == 0);
  const Limits l = p.limits(s, 5030);
  CHECK_NEAR(l.cvl, 56.8, 0.001);
  CHECK_NEAR(l.ccl, 200.0, 0.001);
  CHECK(l.allowed && !l.alarm && l.soc == 70 && !l.stale);
  CHECK_NEAR(l.battery_v, 53.731, 0.0005);
  CHECK_NEAR(l.max_cell_v, 3.361, 0.0005);
}

int main() {
  test_requests();
  test_tv1();
  test_settings();
  test_rs485_1_capture();
  test_trailers();
  test_rejections();
  test_cell_fields();
  test_scan_and_lock();
  test_limits();
  test_alarm_bits();
  test_cadence();
  test_settings_retry_and_invalid();
  test_stale_and_latch();
  test_unlock_and_rescan();
  test_window_and_rejects();
  test_resync();
  return test_result("test_oem_jkpb");
}
