#include <cmath>
#include <cstring>

#include "app/jk_protocol.h"
#include "app/jk_source.h"
#include "check.h"

using namespace oem::jk;
using oem::JkSource;
using oem::Limits;

// docs/history/notes/jk-bms-protocol.md §5, TV1: a real 14S reply, 285 bytes, checksum 0x54D1, length field 0x011B.
static const char *TV1_HEX =
    "4E57011B00000000060001792A010EED020EFA030EF7040EEC050EF8060EFA070EF1080EF8090EE30A0EFA0B0EF10C0EFB0D0EFB0E0EF2"
    "80001D81001E82001C8314EF8480D0850F860287000489000000008A000E8B00008C00078E16268F10AE900FD2910FA0920005930BEA94"
    "0C1C95000596012C9700079800039900059A00059B0CE49C00089D019E005A9F0046A00064A10064A20014A30046A40046A5FFECA6FFF6"
    "A7FFECA8FFF6A90EAA0000000EAB01AC01AD0411AE01AF01B0000AB114B231323334353600000000B300B4496E707574205573B5323130"
    "31B60000E200B748362E585F5F53362E312E33535F5FB800B900000000BA425433303732303230313230303030323030353231303031C0"
    "010000000068000054D1";

static int hexval(char c) { return c <= '9' ? c - '0' : c - 'A' + 10; }

static size_t tv1(uint8_t *f) {
  size_t n = 0;
  for (const char *p = TV1_HEX; *p; p += 2) f[n++] = uint8_t(hexval(p[0]) << 4 | hexval(p[1]));
  return n;
}

static void reseal(uint8_t *f, size_t n) {
  f[2] = uint8_t((n - 2) >> 8);
  f[3] = uint8_t(n - 2);
  uint16_t sum = 0;
  for (size_t i = 0; i < n - 2; i++) sum = uint16_t(sum + f[i]);
  f[n - 2] = uint8_t(sum >> 8);
  f[n - 1] = uint8_t(sum);
}

struct Rec {
  uint8_t id, len;
  const uint8_t *v;
};

// Header, the given records (0x79 supplied with its LEN byte as part of v), then the 4-byte record number and trailer.
static size_t frame(uint8_t *f, const Rec *r, size_t nr) {
  memset(f, 0, 4);
  f[0] = 0x4E;
  f[1] = 0x57;
  f[8] = 0x06;
  f[10] = 0x01;
  size_t n = 11;
  for (size_t i = 0; i < nr; i++) {
    f[n++] = r[i].id;
    for (size_t j = 0; j < r[i].len; j++) f[n++] = r[i].v[j];
  }
  const uint8_t tail[9] = {0, 0, 0, 0, 0x68, 0, 0, 0, 0};
  for (uint8_t t : tail) f[n++] = t;
  reseal(f, n);
  return n;
}

// 4 cells 3300 3341 0 5100 (0 and 5100 ignored -> max 3341), pack 53.20 V, SOC 75, 0x8A = 4.
struct Opt {
  uint16_t alarms = 0, status = 0x000B, pack = 5320, n8a = 4;
  uint8_t soc = 75, len79 = 12;
  bool with_8a = true;
};

static size_t reply(uint8_t *f, const Opt &o = Opt()) {
  const uint8_t c79[13] = {o.len79, 1, 0x0C, 0xE4, 2, 0x0D, 0x0D, 3, 0, 0, 4, 0x13, 0xEC};
  const uint8_t v83[2] = {uint8_t(o.pack >> 8), uint8_t(o.pack)};
  const uint8_t v85[1] = {o.soc};
  const uint8_t v8a[2] = {uint8_t(o.n8a >> 8), uint8_t(o.n8a)};
  const uint8_t v8b[2] = {uint8_t(o.alarms >> 8), uint8_t(o.alarms)};
  const uint8_t v8c[2] = {uint8_t(o.status >> 8), uint8_t(o.status)};
  Rec r[6];
  size_t n = 0;
  r[n++] = {0x79, 13, c79};
  r[n++] = {0x83, 2, v83};
  r[n++] = {0x85, 1, v85};
  if (o.with_8a) r[n++] = {0x8A, 2, v8a};
  r[n++] = {0x8B, 2, v8b};
  r[n++] = {0x8C, 2, v8c};
  return frame(f, r, n);
}

static void test_requests() {
  uint8_t o[REQUEST_LEN];
  const uint8_t a[21] = {0x4E, 0x57, 0, 0x13, 0, 0, 0, 0, 0x06, 0x03, 0, 0, 0, 0, 0, 0, 0x68, 0, 0, 0x01, 0x29};
  const uint8_t b[21] = {0x4E, 0x57, 0, 0x13, 0, 0, 0, 0, 0x06, 0x02, 0, 0, 0, 0, 0, 0, 0x68, 0, 0, 0x01, 0x28};
  CHECK(REQUEST_LEN == 21 && build_request(0x03, o) == 21 && memcmp(o, a, 21) == 0);
  CHECK(build_request(0x02, o) == 21 && memcmp(o, b, 21) == 0);
}

static void test_tv1() {
  uint8_t f[MAX_FRAME];
  const size_t n = tv1(f);
  CHECK(n == 285 && f[2] == 0x01 && f[3] == 0x1B && f[n - 2] == 0x54 && f[n - 1] == 0xD1);
  uint16_t sum = 0;
  for (size_t i = 0; i < n - 2; i++) sum = uint16_t(sum + f[i]);
  CHECK(sum == 0x54D1);
  Data d{};
  CHECK(parse_reply(f, n, d) == Error::OK);
  CHECK_NEAR(d.pack_v, 53.59, 0.0005);
  CHECK(d.soc == 15 && d.cells == 14 && d.alarms == 0 && d.status == 0x0007);
  CHECK_NEAR(d.max_cell_v, 3.835, 0.0005);
  CHECK(d.walk_end == n - 9);  // every record consumed up to the trailer
}

static void test_max_cell_bounds() {
  uint8_t f[MAX_FRAME];
  Data d{};
  const uint8_t v83[2] = {0x14, 0xEF}, v8b[2] = {0, 0}, v8c[2] = {0, 7}, v85[1] = {50};
  const uint8_t pad[15] = {};
  const uint8_t hi[2][10] = {{9, 1, 0x13, 0x87, 2, 0x0C, 0xE4, 3, 0, 0}, {9, 1, 0x13, 0x88, 2, 0x0C, 0xE4, 3, 0, 0}};
  const float want[2] = {4.999f, 3.300f};  // 4999 mV counts, 5000 mV is ignored
  for (int k = 0; k < 2; k++) {
    const Rec r[] = {{0x79, 10, hi[k]}, {0x83, 2, v83}, {0x85, 1, v85}, {0x8B, 2, v8b}, {0x8C, 2, v8c}, {0xB7, 15, pad}};
    const size_t n = frame(f, r, 6);
    CHECK(parse_reply(f, n, d) == Error::OK);
    CHECK_NEAR(d.max_cell_v, want[k], 0.0005);
  }
}

static void test_decode_and_rejections() {
  uint8_t f[MAX_FRAME];
  Data d{};
  size_t n = reply(f);
  CHECK(n == 11 + 14 + 3 + 2 + 3 + 3 + 3 + 9);
  CHECK(parse_reply(f, n, d) == Error::OK);
  CHECK_NEAR(d.pack_v, 53.20, 0.0005);
  CHECK(d.soc == 75 && d.cells == 4);
  CHECK_NEAR(d.max_cell_v, 3.341, 0.0005);  // 0 and 5100 ignored
  n = reply(f, Opt{0x0001, 0x0007});
  CHECK(parse_reply(f, n, d) == Error::OK && d.alarms == 1 && d.status == 0x0007);
  { Opt o; o.soc = 101; n = reply(f, o); CHECK(parse_reply(f, n, d) == Error::OK && d.soc == -1); }
  { Opt o; o.with_8a = false; n = reply(f, o); CHECK(parse_reply(f, n, d) == Error::OK && d.cells == 4); }
  { Opt o; o.n8a = 5; n = reply(f, o); CHECK(parse_reply(f, n, d) == Error::CELLS); }  // 0x8A vs 0x79 mismatch
  { Opt o; o.len79 = 11; n = reply(f, o); CHECK(parse_reply(f, n, d) != Error::OK); }   // not a multiple of 3
  // checksum
  n = reply(f);
  f[20] ^= 1;
  CHECK(parse_reply(f, n, d) == Error::CHECKSUM);
  // length field wrong (checksum fixed up so only the length is wrong)
  n = reply(f);
  f[3] += 1;
  CHECK(parse_reply(f, n, d) == Error::LENGTH);
  n = reply(f);
  CHECK(parse_reply(f, n - 1, d) == Error::LENGTH);  // truncated
  CHECK(parse_reply(f, 3, d) == Error::SHORT);
  CHECK(parse_reply(f, 0, d) == Error::SHORT);
  n = reply(f);
  f[0] = 0x4F;
  CHECK(parse_reply(f, n, d) == Error::HEADER);
  n = reply(f);
  f[8] = 0x03;
  reseal(f, n);
  CHECK(parse_reply(f, n, d) == Error::COMMAND);
  // missing required ID (each dropped in turn); a 0xB7 record pads the frame to a plausible size
  {
    const uint8_t c79[10] = {9, 1, 0x0C, 0xE4, 2, 0x0C, 0xE5, 3, 0x0C, 0xE6};
    const uint8_t v83[2] = {0x14, 0xEF}, v8b[2] = {0, 0}, v8c[2] = {0, 7}, v85[1] = {50};
    const uint8_t pad[15] = {};
    const Rec all[] = {{0x79, 10, c79}, {0x83, 2, v83}, {0x85, 1, v85}, {0x8B, 2, v8b}, {0x8C, 2, v8c}, {0xB7, 15, pad}};
    n = frame(f, all, 6);
    CHECK(parse_reply(f, n, d) == Error::OK && d.cells == 3);
    CHECK_NEAR(d.max_cell_v, 3.302, 0.0005);
    for (int k = 0; k < 5; k++) {
      Rec some[5];
      size_t m = 0;
      for (int i = 0; i < 6; i++) if (i != k) some[m++] = all[i];
      n = frame(f, some, m);
      CHECK(parse_reply(f, n, d) == Error::MISSING);
    }
    // unknown ID mid-frame stops the walk: fine if every required one came before it, rejected if one is after it
    const uint8_t junk[2] = {1, 2};
    const Rec before[] = {{0x79, 10, c79}, {0x83, 2, v83}, {0x85, 1, v85}, {0x8B, 2, v8b}, {0x8C, 2, v8c}, {0xF0, 2, junk}, {0xB7, 15, pad}};
    n = frame(f, before, 7);
    CHECK(parse_reply(f, n, d) == Error::OK);
    const Rec after[] = {{0x79, 10, c79}, {0x83, 2, v83}, {0xF0, 2, junk}, {0x85, 1, v85}, {0x8B, 2, v8b}, {0x8C, 2, v8c}, {0xB7, 15, pad}};
    n = frame(f, after, 7);
    CHECK(parse_reply(f, n, d) == Error::MISSING);
    // a record that runs past the end of the record area stops the walk (0xB2 wants 10 bytes, 2 given)
    const uint8_t big[2] = {0, 0};
    const Rec cut[] = {{0x79, 10, c79}, {0x83, 2, v83}, {0x85, 1, v85}, {0x8B, 2, v8b}, {0xB7, 15, pad}, {0xB2, 2, big}, {0x8C, 2, v8c}};
    n = frame(f, cut, 7);
    CHECK(parse_reply(f, n, d) == Error::MISSING);
  }
}

static void feed(JkSource &p, const uint8_t *f, size_t n, uint32_t now) { p.push(f, n, now); }

static void expect_req(JkSource &p, uint32_t t, uint8_t src) {
  uint8_t out[oem::POLL_REQ_MAX], want[REQUEST_LEN];
  CHECK(p.poll(t, out) == REQUEST_LEN);
  build_request(src, want);
  CHECK(memcmp(out, want, REQUEST_LEN) == 0);
}

static void run(JkSource &p, uint32_t t0, const Opt &o = Opt()) {
  uint8_t f[MAX_FRAME];
  expect_req(p, t0, 0x03);
  feed(p, f, reply(f, o), t0 + 50);
}

static void test_poller_fresh_limits() {
  JkSource p;
  oem::Settings s = oem::default_settings();
  CHECK(!p.fresh(0) && p.baud() == 115200);
  run(p, 0);
  CHECK(p.fresh(100) && p.good_replies() == 1 && p.bad_replies() == 0);
  const Limits l = p.limits(s, 100);
  CHECK(l.fresh && !l.stale && l.source == oem::SourceMode::JK);
  CHECK_NEAR(l.cvl, 57.6, 0.001);
  CHECK_NEAR(l.ccl, 10.0, 0.001);  // Max current
  CHECK(l.allowed && !l.alarm && !l.blocking && l.soc == 75);
  CHECK_NEAR(l.battery_v, 53.2, 0.001);
  CHECK(std::isnan(l.battery_a));
  CHECK_NEAR(l.max_cell_v, 3.341, 0.001);
  s.set(oem::HR_MAX_CURRENT, 25);
  CHECK_NEAR(p.limits(s, 100).ccl, 25.0, 0.001);
}

static void test_poller_tv1() {
  JkSource p;
  uint8_t f[MAX_FRAME], out[oem::POLL_REQ_MAX];
  p.poll(0, out);
  const size_t n = tv1(f);
  feed(p, f, n, 90);
  CHECK(p.good_replies() == 1);
  const Limits l = p.limits(oem::default_settings(), 100);
  CHECK(l.allowed && !l.alarm && l.soc == 15);  // 0x8C = 0x0007: charge MOSFET on
  CHECK_NEAR(l.battery_v, 53.59, 0.001);
  CHECK_NEAR(l.max_cell_v, 3.835, 0.001);
}

static void test_poller_cadence_and_freshness() {
  JkSource p;
  oem::Settings s = oem::default_settings();
  uint8_t f[MAX_FRAME], out[oem::POLL_REQ_MAX];
  expect_req(p, 0, 0x03);
  feed(p, f, reply(f), 50);
  CHECK(p.poll(999, out) == 0);
  expect_req(p, 1000, 0x03);
  CHECK(p.poll(1999, out) == 0);
  CHECK(p.fresh(3050) && !p.limits(s, 3050).stale);  // reply is 3000 ms old
  CHECK(!p.fresh(3051) && p.limits(s, 3051).stale);
}

static void test_poller_alarm_mask() {
  oem::Settings s = oem::default_settings();
  const uint16_t bits[] = {0x0002, 0x0004, 0x0010, 0x0020, 0x0100, 0x0200, 0x0400, 0x1000, 0x2000, 0x3736};
  for (uint16_t b : bits) {
    JkSource p;
    Opt o;
    o.alarms = b;
    run(p, 0, o);
    const Limits l = p.limits(s, 100);
    CHECK(l.alarm && !l.allowed && l.fresh);
  }
  const uint16_t quiet[] = {0x0001, 0x0008, 0x0040, 0x0080, 0x0800, 0x4000, 0x8000, 0xC849};
  for (uint16_t b : quiet) {
    JkSource p;
    Opt o;
    o.alarms = b;
    run(p, 0, o);
    const Limits l = p.limits(s, 100);
    CHECK(!l.alarm && l.allowed);
  }
}

static void test_poller_status_bits() {
  oem::Settings s = oem::default_settings();
  const uint16_t ok[] = {0x0001, 0x0007, 0x0009, 0x000B, 0xFFF9};
  for (uint16_t st : ok) {
    JkSource p;
    Opt o;
    o.status = st;
    run(p, 0, o);
    CHECK(p.limits(s, 100).allowed);
  }
  const uint16_t no[] = {0x0000, 0x0006, 0x0008, 0xFFFE};  // needs bit 0 only
  for (uint16_t st : no) {
    JkSource p;
    Opt o;
    o.status = st;
    run(p, 0, o);
    const Limits l = p.limits(s, 100);
    CHECK(!l.allowed && !l.alarm && l.fresh);
  }
}

static void test_alarm_with_normal_status() {
  JkSource p;
  Opt o;
  o.alarms = 0x0002;
  o.status = 0x0007;
  run(p, 0, o);
  const Limits l = p.limits(oem::default_settings(), 100);
  CHECK(l.alarm && !l.allowed);
}

static void test_poller_stale_and_latch() {
  oem::Settings s = oem::default_settings();
  {
    JkSource p;
    Limits l = p.limits(s, 0);
    CHECK(l.fresh && l.stale && l.allowed && l.source == oem::SourceMode::JK);
    CHECK_NEAR(l.ccl, 10.0, 0.001);
    s.set(oem::HR_FALLBACK_A, 4);
    CHECK_NEAR(p.limits(s, 0).ccl, 4.0, 0.001);
    s.set(oem::HR_FALLBACK_A, 10);
  }
  {
    JkSource p;
    run(p, 0);
    const Limits l = p.limits(s, 3051);
    CHECK(l.stale && l.allowed);
    CHECK_NEAR(l.cvl, 57.6, 0.001);
  }
  {
    JkSource p;  // an alarm, then silence: held off, though stale
    Opt o;
    o.alarms = 0x0002;
    run(p, 0, o);
    CHECK(p.limits(s, 3051).stale && !p.limits(s, 3051).allowed);
    run(p, 5000);  // recovery
    const Limits r = p.limits(s, 5100);
    CHECK(!r.stale && r.allowed);
  }
  {
    JkSource p;  // the charge MOSFET off also latches
    Opt o;
    o.status = 0x0006;
    run(p, 0, o);
    CHECK(!p.limits(s, 3051).allowed);
  }
  {
    JkSource p;
    Opt o;
    o.alarms = 0x0002;
    run(p, 0, o);
    p.reset();
    const Limits l = p.limits(s, 3051);
    CHECK(l.stale && l.allowed);
    CHECK(p.good_replies() == 0 && !p.fresh(10));
    expect_req(p, 10, 0x03);
  }
}

static void test_poller_window_and_rejects() {
  JkSource p;
  uint8_t f[MAX_FRAME], out[oem::POLL_REQ_MAX];
  size_t n = reply(f);
  feed(p, f, n, 5);  // no request yet
  CHECK(p.good_replies() == 0 && p.bad_replies() == 0);
  expect_req(p, 100, 0x03);
  feed(p, f, n, 701);  // 601 ms after the request: too late
  CHECK(p.good_replies() == 0);
  expect_req(p, 1100, 0x03);
  f[30] ^= 0x40;
  feed(p, f, n, 1110);  // checksum
  CHECK(p.bad_replies() == 1 && p.good_replies() == 0);
  n = reply(f);
  feed(p, f, n, 1120);  // still outstanding
  CHECK(p.good_replies() == 1);
  feed(p, f, n, 1130);  // already answered
  CHECK(p.good_replies() == 1 && p.bad_replies() == 1);
  (void)out;
  JkSource q;  // a 0x8A mismatch counts as bad
  expect_req(q, 0, 0x03);
  Opt o;
  o.n8a = 5;
  n = reply(f, o);
  feed(q, f, n, 20);
  CHECK(q.bad_replies() == 1 && q.good_replies() == 0 && !q.fresh(20));
}

static void test_source_flip() {
  JkSource p;
  uint8_t f[MAX_FRAME];
  expect_req(p, 0, 0x03);
  expect_req(p, 1000, 0x03);
  expect_req(p, 2000, 0x03);
  expect_req(p, 3000, 0x02);  // 3 polls without a valid reply
  expect_req(p, 4000, 0x02);
  expect_req(p, 5000, 0x02);
  expect_req(p, 6000, 0x03);  // and back
  feed(p, f, reply(f), 6050);
  expect_req(p, 7000, 0x03);  // a valid reply resets the count
  expect_req(p, 8000, 0x03);
  expect_req(p, 9000, 0x03);
  feed(p, f, reply(f), 9050);
  expect_req(p, 10000, 0x03);
  expect_req(p, 11000, 0x03);
  expect_req(p, 12000, 0x03);
  expect_req(p, 13000, 0x02);
  // a bad reply is not a valid reply
  JkSource q;
  expect_req(q, 0, 0x03);
  const size_t qn = reply(f);
  f[30] ^= 1;
  feed(q, f, qn, 10);
  expect_req(q, 1000, 0x03);
  expect_req(q, 2000, 0x03);
  expect_req(q, 3000, 0x02);
}

static void test_resync() {
  uint8_t f[MAX_FRAME], g[2 * MAX_FRAME], out[oem::POLL_REQ_MAX];
  {
    JkSource p;  // stray prefix bytes
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    const uint8_t pre[3] = {0x00, 0xFF, 0x57};
    feed(p, pre, 3, 5);
    feed(p, f, n, 10);
    CHECK(p.good_replies() == 1 && p.bad_replies() == 0);
  }
  {
    JkSource p;  // a lone 0x4E directly before the real header
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    const uint8_t pre[1] = {0x4E};
    feed(p, pre, 1, 5);
    feed(p, f, n, 10);
    CHECK(p.good_replies() == 1);
  }
  {
    JkSource p;  // 4E 4E 57 ... split mid-header
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    feed(p, f, 1, 5);
    feed(p, f, n, 6);  // 4E | 4E 57 ...
    CHECK(p.good_replies() == 1);
  }
  {
    JkSource p;  // an implausible length field is dropped early and the real frame behind it still reads
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    const uint8_t bad[4] = {0x4E, 0x57, 0xFF, 0xFF};
    feed(p, bad, 4, 5);
    feed(p, f, n, 10);
    CHECK(p.good_replies() == 1);
    expect_req(p, 1000, 0x03);
    const uint8_t small[4] = {0x4E, 0x57, 0x00, 0x05};
    feed(p, small, 4, 1005);
    feed(p, f, n, 1010);
    CHECK(p.good_replies() == 2);
  }
  {
    JkSource p;  // a frame failing its checksum that swallowed the real reply's start: replay finds it
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    // junk header claiming a 40-byte total whose body holds the whole real reply
    uint8_t h[4] = {0x4E, 0x57, 0x00, 0x26};
    feed(p, h, 4, 5);
    feed(p, f, n, 10);
    CHECK(p.good_replies() == 1 && p.bad_replies() >= 1);
  }
  {
    JkSource p;  // the replay itself breaks on a stray 4E 57 right before the real header: still in stream order
    expect_req(p, 0, 0x03);
    size_t m = 0;
    const uint8_t h[4] = {0x4E, 0x57, 0x00, 0x26};  // a broken frame claiming 40 bytes
    for (uint8_t b : h) g[m++] = b;
    for (int i = 0; i < 10; i++) g[m++] = 0x11;
    g[m++] = 0x4E;
    g[m++] = 0x57;
    m += reply(g + m);
    feed(p, g, m, 10);
    CHECK(p.good_replies() == 1 && p.bad_replies() >= 1);
  }
  {
    JkSource p;  // a late reply (after the window) is dropped
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    feed(p, f, 20, 500);
    CHECK(p.poll(1000, out) == 0);
    feed(p, f + 20, n - 20, 650);
    CHECK(p.good_replies() == 0);
    expect_req(p, 1000 - 0 + 1, 0x03);
  }
  {
    JkSource p;  // long garbage with 4E / 57 runs never overflows; the next valid reply is accepted
    expect_req(p, 0, 0x03);
    for (size_t i = 0; i < sizeof g; i++) g[i] = (i % 3 == 0) ? 0x4E : (i % 3 == 1) ? 0x57 : 0x24;
    feed(p, g, sizeof g, 5);
    const size_t n = reply(f);
    feed(p, f, n, 20);
    CHECK(p.good_replies() == 1);
  }
  {
    JkSource p;  // garbage of header pairs with plausible lengths, then valid
    expect_req(p, 0, 0x03);
    for (size_t i = 0; i < sizeof g; i++) g[i] = (i % 4 == 0) ? 0x4E : (i % 4 == 1) ? 0x57 : (i % 4 == 2) ? 0x00 : 0x30;
    feed(p, g, sizeof g, 5);
    const size_t n = reply(f);
    feed(p, f, n, 20);
    CHECK(p.good_replies() == 1);
  }
}

static void test_poller_split_and_hold() {
  uint8_t f[MAX_FRAME], out[oem::POLL_REQ_MAX];
  {
    JkSource p;  // split across pushes, byte by byte at the end
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    feed(p, f, 10, 10);
    feed(p, f + 10, 1, 11);
    feed(p, f + 11, n - 11, 20);
    CHECK(p.good_replies() == 1);
  }
  {
    JkSource p;  // TV1 split in three
    uint8_t t[MAX_FRAME];
    const size_t n = tv1(t);
    p.poll(0, out);
    feed(p, t, 100, 10);
    feed(p, t + 100, 100, 20);
    feed(p, t + 200, n - 200, 30);
    CHECK(p.good_replies() == 1);
  }
  {
    JkSource p;  // partial reply at the 1000 ms mark holds the next request
    expect_req(p, 0, 0x03);
    const size_t n = reply(f);
    feed(p, f, 20, 580);
    CHECK(p.poll(1000, out) == 0);
    CHECK(p.poll(1000 + oem::JK_HOLD_MS - 1, out) == 0);
    feed(p, f + 20, n - 20, 1100);  // ends late: not parsed
    CHECK(p.good_replies() == 0);
    expect_req(p, 1101, 0x03);
  }
  {
    JkSource p;  // never ends: abandoned after the hold
    expect_req(p, 0, 0x03);
    reply(f);
    feed(p, f, 20, 580);
    CHECK(p.poll(1000 + oem::JK_HOLD_MS - 1, out) == 0);
    expect_req(p, 1000 + oem::JK_HOLD_MS, 0x03);
    const size_t n = reply(f);
    feed(p, f, n, 1750);
    CHECK(p.good_replies() == 1);
  }
}

int main() {
  test_requests();
  test_tv1();
  test_max_cell_bounds();
  test_decode_and_rejections();
  test_poller_fresh_limits();
  test_poller_tv1();
  test_poller_cadence_and_freshness();
  test_poller_alarm_mask();
  test_poller_status_bits();
  test_alarm_with_normal_status();
  test_poller_stale_and_latch();
  test_poller_window_and_rejects();
  test_source_flip();
  test_poller_split_and_hold();
  test_resync();
  return test_result("test_oem_jk");
}
