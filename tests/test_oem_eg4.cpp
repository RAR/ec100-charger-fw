#include <cmath>
#include <cstring>

#include "app/crc16.h"
#include "app/eg4_protocol.h"
#include "app/eg4_source.h"
#include "check.h"

using namespace oem::eg4;
using oem::Limits;
using oem::Eg4Source;

static void put(uint8_t *f, size_t reg_off, uint16_t v) {
  f[3 + 2 * reg_off] = uint8_t(v >> 8);
  f[3 + 2 * reg_off + 1] = uint8_t(v);
}

static size_t seal(uint8_t *f, uint16_t count) {
  f[0] = ADDR;
  f[1] = 0x03;
  f[2] = uint8_t(2 * count);
  const size_t n = 3 + 2 * count;
  const uint16_t crc = oem::crc16_modbus(f, n);
  f[n] = uint8_t(crc);
  f[n + 1] = uint8_t(crc >> 8);
  return n + 2;
}

// Block A: 53.20 V, current register 0x0F00, cells 3325 3330 3341 3320 then 0 and one bogus 5100.
static size_t reply_a(uint8_t *f, uint16_t pack = 5320) {
  memset(f, 0, MAX_REPLY);
  put(f, 0, pack);
  put(f, 1, 0x0F00);  // current: 3840, above every cell and below 5000 as mV, so a cell loop starting a register early would count it
  const uint16_t cells[6] = {3325, 3330, 3341, 3320, 0, 5100};
  for (int i = 0; i < 6; i++) put(f, 2 + i, cells[i]);
  if (pack == 5321) put(f, 17, 3399);  // max cell in the last cell register
  return seal(f, A_COUNT);
}

// Block B: max charge current 30 A (0x16), SOC 75 (0x18), protection p (0x1B).
static size_t reply_b(uint8_t *f, uint16_t prot = 0, uint16_t soc = 75, uint16_t ccl = 30) {
  memset(f, 0, MAX_REPLY);
  put(f, 0x16 - 0x12, ccl);
  put(f, 0x18 - 0x12, soc);
  put(f, 0x1A - 0x12, 0xFFFF);  // warnings are ignored
  put(f, 0x1B - 0x12, prot);
  return seal(f, B_COUNT);
}

static void test_requests() {
  uint8_t o[REQUEST_LEN];
  const uint8_t a[8] = {0x10, 0x03, 0x00, 0x00, 0x00, 0x12, 0xC6, 0x86};
  const uint8_t b[8] = {0x10, 0x03, 0x00, 0x12, 0x00, 0x15, 0x27, 0x41};
  CHECK(build_request(0x0000, 0x12, o) == 8 && memcmp(o, a, 8) == 0);
  CHECK(build_request(0x0012, 0x15, o) == 8 && memcmp(o, b, 8) == 0);
}

static void test_decode() {
  uint8_t f[MAX_REPLY];
  const uint8_t *d = nullptr;
  size_t n = reply_a(f);
  CHECK(n == 41 && parse_reply(f, n, A_COUNT, d) == Error::OK && d == f + 3);
  BlockA a = decode_a(d);
  CHECK_NEAR(a.pack_v, 53.20, 0.0005);
  CHECK_NEAR(a.max_cell_v, 3.341, 0.0005);  // 0 and 5100 ignored
  n = reply_b(f, 0x0008, 75, 30);
  CHECK(n == 47 && parse_reply(f, n, B_COUNT, d) == Error::OK);
  BlockB b = decode_b(d);
  CHECK(b.soc == 75 && b.protection == 0x0008);
  CHECK_NEAR(b.ccl, 30.0, 0.001);
  n = reply_b(f, 0, 101);
  CHECK(parse_reply(f, n, B_COUNT, d) == Error::OK && decode_b(d).soc == -1);
  memset(f, 0, MAX_REPLY);
  n = seal(f, A_COUNT);
  CHECK(parse_reply(f, n, A_COUNT, d) == Error::OK && std::isnan(decode_a(d).max_cell_v));
}

static void test_rejections() {
  uint8_t f[MAX_REPLY];
  const uint8_t *d = nullptr;
  size_t n = reply_b(f);
  f[0] = 0x01;
  CHECK(parse_reply(f, n, B_COUNT, d) == Error::ADDR && d == nullptr);
  n = reply_b(f);
  f[1] = 0x83;
  CHECK(parse_reply(f, n, B_COUNT, d) == Error::FUNC);
  n = reply_b(f);
  f[2] = 40;
  CHECK(parse_reply(f, n, B_COUNT, d) == Error::COUNT);
  n = reply_b(f);
  f[10] ^= 1;
  CHECK(parse_reply(f, n, B_COUNT, d) == Error::CRC);
  n = reply_b(f);
  CHECK(parse_reply(f, n - 1, B_COUNT, d) == Error::SHORT);
  CHECK(parse_reply(f, 3, B_COUNT, d) == Error::SHORT);
  CHECK(parse_reply(f, 0, B_COUNT, d) == Error::SHORT);
  n = reply_a(f);
  CHECK(parse_reply(f, n, B_COUNT, d) == Error::COUNT);  // a block A reply to a block B request
}

static void feed(Eg4Source &p, const uint8_t *f, size_t n, uint32_t now) { p.push(f, n, now); }

static void expect_req(Eg4Source &p, uint32_t t, uint16_t first, uint16_t count) {
  uint8_t out[oem::POLL_REQ_MAX], want[REQUEST_LEN];
  CHECK(p.poll(t, out) == REQUEST_LEN);
  build_request(first, count, want);
  CHECK(memcmp(out, want, REQUEST_LEN) == 0);
}

// Requests A at t0, B at t0+500, each answered 50 ms later.
static void run_cycle(Eg4Source &p, uint32_t t0, uint16_t prot, uint16_t pack = 5320) {
  uint8_t f[MAX_REPLY], out[oem::POLL_REQ_MAX];
  expect_req(p, t0, A_FIRST, A_COUNT);
  size_t n = reply_a(f, pack);
  feed(p, f, n, t0 + 50);
  CHECK(p.poll(t0 + 499, out) == 0);
  expect_req(p, t0 + 500, B_FIRST, B_COUNT);
  n = reply_b(f, prot);
  feed(p, f, n, t0 + 550);
}

static void run(Eg4Source &p, uint32_t t0, uint16_t prot) { run_cycle(p, t0, prot); }

static void test_poller_fresh_limits() {
  Eg4Source p;
  oem::Settings s = oem::default_settings();
  CHECK(!p.fresh(0) && p.baud() == 9600);
  run(p, 0, 0);
  CHECK(p.fresh(600) && p.good_replies() == 2 && p.bad_replies() == 0);
  const Limits l = p.limits(s, 600);
  CHECK(l.fresh && !l.stale && l.source == oem::SourceMode::EG4);
  CHECK_NEAR(l.cvl, 57.6, 0.001);  // Charge V (Panel)
  CHECK_NEAR(l.ccl, 30.0, 0.001);
  CHECK(l.allowed && !l.alarm && !l.blocking && l.soc == 75);
  CHECK_NEAR(l.battery_v, 53.2, 0.001);
  CHECK(std::isnan(l.battery_a));
  CHECK_NEAR(l.max_cell_v, 3.341, 0.001);
}

static void test_poller_cadence_and_freshness() {
  Eg4Source p;
  oem::Settings s = oem::default_settings();
  uint8_t f[MAX_REPLY], out[oem::POLL_REQ_MAX];
  expect_req(p, 0, A_FIRST, A_COUNT);
  feed(p, f, reply_a(f), 50);
  CHECK(!p.fresh(100));  // block A alone is not enough
  CHECK(p.poll(499, out) == 0);
  expect_req(p, 500, B_FIRST, B_COUNT);
  feed(p, f, reply_b(f), 550);
  CHECK(p.fresh(550));
  expect_req(p, 1000, A_FIRST, A_COUNT);  // alternates back
  expect_req(p, 1500, B_FIRST, B_COUNT);
  CHECK(p.fresh(3050) && !p.limits(s, 3050).stale);   // A is 3000 ms old
  CHECK(!p.fresh(3051) && p.limits(s, 3051).stale);   // A is 3001 ms old
}

static void test_poller_alarm_mask() {
  oem::Settings s = oem::default_settings();
  const uint16_t bits[] = {0x0001, 0x0002, 0x0010, 0x0080, 0x0100, 0x0400, 0x0593};
  for (uint16_t b : bits) {
    Eg4Source p;
    run(p, 0, b);
    const Limits l = p.limits(s, 600);
    CHECK(l.alarm && !l.allowed && l.fresh);
  }
  const uint16_t quiet[] = {0x0004, 0x0008, 0x000C, 0x0020, 0x0040, 0x0200, 0x0800, 0x1000, 0x2000};
  for (uint16_t b : quiet) {
    Eg4Source p;
    run(p, 0, b);
    const Limits l = p.limits(s, 600);
    CHECK(!l.alarm && l.allowed);
  }
}

static void test_poller_stale_and_latch() {
  oem::Settings s = oem::default_settings();  // Max 10 A, Fallback 10 A
  {
    Eg4Source p;
    Limits l = p.limits(s, 0);
    CHECK(l.fresh && l.stale && l.allowed && l.source == oem::SourceMode::EG4);
    CHECK_NEAR(l.ccl, 10.0, 0.001);
    s.set(oem::HR_FALLBACK_A, 4);
    CHECK_NEAR(p.limits(s, 0).ccl, 4.0, 0.001);
    s.set(oem::HR_FALLBACK_A, 10);
  }
  {
    Eg4Source p;  // normal then silence: Panel fallback, still allowed
    run(p, 0, 0);
    const Limits l = p.limits(s, 3551);
    CHECK(l.stale && l.allowed);
    CHECK_NEAR(l.cvl, 57.6, 0.001);
  }
  {
    Eg4Source p;  // an alarm, then silence: held off, though stale
    run(p, 0, 0x0001);
    const Limits l = p.limits(s, 3551);
    CHECK(l.stale && !l.allowed);
    run(p, 5000, 0);  // recovery
    const Limits r = p.limits(s, 5600);
    CHECK(!r.stale && r.allowed);
  }
  {
    Eg4Source p;  // latch then reset forgets
    run(p, 0, 0x0001);
    p.reset();
    const Limits l = p.limits(s, 3551);
    CHECK(l.stale && l.allowed);
    CHECK(p.good_replies() == 0 && !p.fresh(10));
    expect_req(p, 10, A_FIRST, A_COUNT);  // restarts at block A
  }
}

// A valid block B that by itself says stop latches even though block A never arrived (so the source is not fresh).
static void b_only(Eg4Source &p, uint16_t prot, uint16_t ccl) {
  uint8_t f[MAX_REPLY], out[oem::POLL_REQ_MAX];
  expect_req(p, 0, A_FIRST, A_COUNT);
  feed(p, f, reply_a(f, 0), 50);  // zero pack V: rejected
  CHECK(p.poll(499, out) == 0);
  expect_req(p, 500, B_FIRST, B_COUNT);
  feed(p, f, reply_b(f, prot, 75, ccl), 550);
}

static void test_poller_latch_without_fresh() {
  oem::Settings s = oem::default_settings();
  {
    Eg4Source p;
    b_only(p, 0x0002, 30);
    CHECK(!p.fresh(600) && p.bad_replies() == 1);
    const Limits l = p.limits(s, 600);
    CHECK(l.stale && !l.allowed);
  }
  {
    Eg4Source p;  // CCL 0 stops too
    b_only(p, 0, 0);
    CHECK(p.limits(s, 600).stale && !p.limits(s, 600).allowed);
  }
  {
    Eg4Source p;  // a clean block B alone does not stop (nor clear)
    b_only(p, 0, 30);
    CHECK(p.limits(s, 600).stale && p.limits(s, 600).allowed);
  }
  {
    Eg4Source p;  // fresh with CCL 0: latched, held off once stale, cleared by a fresh good state
    run_cycle(p, 0, 0);
    uint8_t f[MAX_REPLY], out[oem::POLL_REQ_MAX];
    expect_req(p, 1000, A_FIRST, A_COUNT);
    feed(p, f, reply_a(f), 1050);
    expect_req(p, 1500, B_FIRST, B_COUNT);
    feed(p, f, reply_b(f, 0, 75, 0), 1550);
    CHECK(p.limits(s, 1600).ccl == 0.0f);
    CHECK(p.limits(s, 5000).stale && !p.limits(s, 5000).allowed);
    run(p, 6000, 0);
    CHECK(!p.limits(s, 6600).stale && p.limits(s, 6600).allowed);
    (void)out;
  }
}

static void test_poller_window_and_rejects() {
  Eg4Source p;
  uint8_t f[MAX_REPLY], out[oem::POLL_REQ_MAX];
  size_t n = reply_a(f);
  feed(p, f, n, 5);  // no request yet: dropped
  CHECK(p.good_replies() == 0 && p.bad_replies() == 0);
  expect_req(p, 100, A_FIRST, A_COUNT);
  feed(p, f, n, 501);  // 401 ms after the request: too late
  CHECK(p.good_replies() == 0);
  CHECK(p.poll(600, out) == REQUEST_LEN);  // block B now outstanding
  n = reply_b(f);
  f[2] = 40;
  feed(p, f, n, 610);  // wrong byte count
  CHECK(p.bad_replies() == 1 && p.good_replies() == 0);
  n = reply_b(f);
  f[20] ^= 0x40;
  feed(p, f, n, 620);
  CHECK(p.bad_replies() == 2);
  n = reply_b(f);
  feed(p, f, n, 630);  // still in the window, still outstanding
  CHECK(p.good_replies() == 1);
  feed(p, f, n, 640);  // already answered: dropped
  CHECK(p.good_replies() == 1 && p.bad_replies() == 2);
  // zero pack voltage rejects block A
  Eg4Source q;
  expect_req(q, 0, A_FIRST, A_COUNT);
  n = reply_a(f, 0);
  feed(q, f, n, 20);
  CHECK(q.bad_replies() == 1 && q.good_replies() == 0 && !q.fresh(20));
}

static void test_resync() {
  uint8_t f[MAX_REPLY], g[2 * MAX_REPLY], out[oem::POLL_REQ_MAX];
  {
    Eg4Source p;  // stray prefix bytes
    expect_req(p, 0, A_FIRST, A_COUNT);
    const size_t n = reply_a(f);
    const uint8_t pre[2] = {0x00, 0xFF};
    feed(p, pre, 2, 5);
    feed(p, f, n, 10);
    CHECK(p.good_replies() == 1 && p.bad_replies() == 0);
  }
  {
    Eg4Source p;  // a late block A reply while B is pending, then the valid B in the window
    expect_req(p, 0, A_FIRST, A_COUNT);
    CHECK(p.poll(500, out) == REQUEST_LEN);
    size_t n = reply_a(f);
    feed(p, f, n, 510);
    CHECK(p.good_replies() == 0);
    n = reply_b(f);
    feed(p, f, n, 520);
    CHECK(p.good_replies() == 1);
  }
  {
    Eg4Source p;  // an abandoned false start does not hold longer than Pylon's rule
    expect_req(p, 0, A_FIRST, A_COUNT);
    const uint8_t junk[2] = {0x10, 0x03};
    feed(p, junk, 2, 10);
    CHECK(p.poll(500, out) == 0);
    CHECK(p.poll(500 + oem::EG4_HOLD_MS, out) == REQUEST_LEN);
  }
  {
    Eg4Source p;  // long garbage (incl. 0x10 / 0x03 runs) never overflows; the next valid reply is accepted
    expect_req(p, 0, A_FIRST, A_COUNT);
    for (size_t i = 0; i < sizeof g; i++) g[i] = (i % 3 == 0) ? 0x10 : (i % 3 == 1) ? 0x03 : 0x24;
    feed(p, g, sizeof g, 5);
    const size_t n = reply_a(f);
    feed(p, f, n, 20);
    CHECK(p.good_replies() == 1);
  }
  {
    Eg4Source p;  // an all-zero block B is rejected
    expect_req(p, 0, A_FIRST, A_COUNT);
    feed(p, f, reply_a(f), 10);
    expect_req(p, 500, B_FIRST, B_COUNT);
    memset(f, 0, MAX_REPLY);
    feed(p, f, seal(f, B_COUNT), 510);
    CHECK(p.bad_replies() == 1 && p.good_replies() == 1 && !p.fresh(510));
  }
  {
    Eg4Source p;  // the last cell register can hold the maximum
    run_cycle(p, 0, 0, 5321);
    CHECK_NEAR(p.limits(oem::default_settings(), 600).max_cell_v, 3.399, 0.0005);
  }
}

static void test_poller_split_and_hold() {
  uint8_t f[MAX_REPLY], out[oem::POLL_REQ_MAX];
  {
    Eg4Source p;  // split across pushes
    expect_req(p, 0, A_FIRST, A_COUNT);
    const size_t n = reply_a(f);
    feed(p, f, 10, 10);
    feed(p, f + 10, n - 10, 20);
    CHECK(p.good_replies() == 1);
  }
  {
    Eg4Source p;  // partial reply at the 500 ms mark holds the next request
    expect_req(p, 0, A_FIRST, A_COUNT);
    const size_t n = reply_a(f);
    feed(p, f, 20, 380);
    CHECK(p.poll(500, out) == 0);
    CHECK(p.poll(500 + oem::EG4_HOLD_MS - 1, out) == 0);
    feed(p, f + 20, n - 20, 570);  // ends late: not parsed, line free
    CHECK(p.good_replies() == 0);
    expect_req(p, 571, B_FIRST, B_COUNT);
  }
  {
    Eg4Source p;  // never ends: abandoned after the hold
    expect_req(p, 0, A_FIRST, A_COUNT);
    reply_a(f);
    feed(p, f, 20, 380);
    CHECK(p.poll(500 + oem::EG4_HOLD_MS - 1, out) == 0);
    expect_req(p, 500 + oem::EG4_HOLD_MS, B_FIRST, B_COUNT);
    const size_t n = reply_b(f);
    feed(p, f, n, 1050);
    CHECK(p.good_replies() == 1);
  }
}

int main() {
  test_requests();
  test_decode();
  test_rejections();
  test_poller_fresh_limits();
  test_poller_cadence_and_freshness();
  test_poller_alarm_mask();
  test_poller_stale_and_latch();
  test_poller_latch_without_fresh();
  test_poller_window_and_rejects();
  test_poller_split_and_hold();
  test_resync();
  return test_result("test_oem_eg4");
}
