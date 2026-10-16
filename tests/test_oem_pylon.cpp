#include <cmath>
#include <cstring>

#include "app/pylon_protocol.h"
#include "app/pylon_source.h"
#include "check.h"

using namespace oem::pylon;

static const char *const A61 =
    "~200246008062D098012C4B000A000A64640D1101010D0201080BBE0BD701030BA501090BBE0BD701010BA501010BBE0BD701010BA50101"
    "E8C4\r";
static const char *const A62_CLEAR = "~20024600800800000000FC22\r";
static const char *const A62_ALARM = "~20024600800800000200FC20\r";
static const char *const A63_ON = "~20024600D012D548ABE0012C03E8C0F995\r";
static const char *const A63_OFF = "~20024600D012D098ABE0000003E840F9BA\r";

static Error parse(const char *f, uint8_t cid2, uint8_t *info, size_t &n) {
  return parse_reply(f, strlen(f), min_info(cid2), info, n);
}

static void test_length_and_checksum() {
  CHECK(length_field(18) == 0xD012);
  CHECK(length_field(0) == 0x0000);
  CHECK(length_field(8) == 0x8008);
  CHECK(checksum("201246600000", 12) == 0xFDAB);  // stock's captured 0x60 request
}

static void test_requests() {
  char out[REQUEST_LEN + 1] = {};
  CHECK(build_request(CID_ANALOG, out) == REQUEST_LEN && strcmp(out, "~201246610000FDAA\r") == 0);
  CHECK(build_request(CID_ALARM, out) == REQUEST_LEN && strcmp(out, "~201246620000FDA9\r") == 0);
  CHECK(build_request(CID_MGMT, out) == REQUEST_LEN && strcmp(out, "~201246630000FDA8\r") == 0);
}

static void test_decode_analog() {
  uint8_t info[MAX_INFO];
  size_t n = 0;
  CHECK(parse(A61, CID_ANALOG, info, n) == Error::OK && n == 49);
  const Analog a = decode_analog(info);
  CHECK_NEAR(a.battery_v, 53.4, 0.0005);
  CHECK(a.soc == 75);
  CHECK_NEAR(a.max_cell_v, 3.345, 0.0005);
  info[4] = 101;
  CHECK(decode_analog(info).soc == -1);
}

static void test_decode_alarm() {
  uint8_t info[MAX_INFO];
  size_t n = 0;
  CHECK(parse(A62_CLEAR, CID_ALARM, info, n) == Error::OK && n == 4 && !decode_alarm(info));
  CHECK(parse(A62_ALARM, CID_ALARM, info, n) == Error::OK && decode_alarm(info));
  const uint8_t only_low_bits[4] = {0xFF, 0xFF, 0x55, 0x97};  // nothing under 0xAA / 0x68
  CHECK(!decode_alarm(only_low_bits));
  const uint8_t byte3[4] = {0, 0, 0, 0x40};
  CHECK(decode_alarm(byte3));
}

static void test_decode_mgmt() {
  uint8_t info[MAX_INFO];
  size_t n = 0;
  CHECK(parse(A63_ON, CID_MGMT, info, n) == Error::OK && n == 9);
  Mgmt m = decode_mgmt(info);
  CHECK_NEAR(m.cvl, 54.6, 0.0005);
  CHECK_NEAR(m.dvl, 44.0, 0.0005);
  CHECK_NEAR(m.ccl, 30.0, 0.001);
  CHECK_NEAR(m.dcl, 100.0, 0.001);
  CHECK(m.status == 0xC0 && m.charge_enable);
  CHECK(parse(A63_OFF, CID_MGMT, info, n) == Error::OK);
  m = decode_mgmt(info);
  CHECK_NEAR(m.ccl, 0.0, 0.001);
  CHECK(!m.charge_enable);
}

static void test_rejections() {
  uint8_t info[MAX_INFO];
  size_t n = 0;
  char f[300];
  strcpy(f, A63_ON);
  f[strlen(f) - 2] = '6';  // checksum F995 -> F996
  CHECK(parse(f, CID_MGMT, info, n) == Error::CHKSUM);
  strcpy(f, A63_ON);
  f[9] = 'C';  // LENGTH D012 -> C012: LCHKSUM wrong
  CHECK(parse(f, CID_MGMT, info, n) == Error::LCHKSUM);
  CHECK(parse("~20024700D012D548ABE0012C03E8C0F994\r", CID_MGMT, info, n) == Error::CID1);
  CHECK(parse("~20024601D012D548ABE0012C03E8C0F994\r", CID_MGMT, info, n) == Error::RTN);
  CHECK(parse(A62_CLEAR, CID_MGMT, info, n) == Error::SHORT);  // 4 INFO bytes where 0x63 needs 9
  CHECK(parse("~201246630000FDA8\r", CID_MGMT, info, n) == Error::RTN);  // our own request echoed back
  strcpy(f, A63_ON);
  f[20] = 'G';
  CHECK(parse(f, CID_MGMT, info, n) == Error::FORMAT);
  strcpy(f, A63_ON);
  f[strlen(f) - 1] = '\n';
  CHECK(parse(f, CID_MGMT, info, n) == Error::FORMAT);
  CHECK(parse("~20024600D014D548ABE0012C03E8C0F995\r", CID_MGMT, info, n) == Error::LENGTH);  // LENID 20, 18 sent
  CHECK(parse("~2002", CID_MGMT, info, n) == Error::FORMAT);
  CHECK(n == 0);
}

using oem::Limits;
using oem::PylonSource;

static void feed(PylonSource &p, const char *frame, uint32_t now) {
  p.push(reinterpret_cast<const uint8_t *>(frame), strlen(frame), now);
}

// Requests at 0, 500, 1000 ms (0x61, 0x62, 0x63), each answered 50 ms later.
static void run_cycle(PylonSource &p, uint32_t t0, const char *a61, const char *a62, const char *a63) {
  uint8_t out[REQUEST_LEN + 1] = {};
  CHECK(p.poll(t0, out) == REQUEST_LEN && strncmp(reinterpret_cast<const char *>(out), "~201246610000FDAA\r", REQUEST_LEN) == 0);
  feed(p, a61, t0 + 50);
  CHECK(p.poll(t0 + 499, out) == 0);
  CHECK(p.poll(t0 + 500, out) == REQUEST_LEN && strncmp(reinterpret_cast<const char *>(out), "~201246620000FDA9\r", REQUEST_LEN) == 0);
  feed(p, a62, t0 + 550);
  CHECK(p.poll(t0 + 1000, out) == REQUEST_LEN && strncmp(reinterpret_cast<const char *>(out), "~201246630000FDA8\r", REQUEST_LEN) == 0);
  feed(p, a63, t0 + 1050);
}

static void test_poller_fresh_limits() {
  PylonSource p;
  oem::Settings s = oem::default_settings();
  CHECK(!p.fresh(0));
  run_cycle(p, 0, A61, A62_CLEAR, A63_ON);
  CHECK(p.fresh(1100) && p.good_replies() == 3 && p.bad_replies() == 0);
  const Limits l = p.limits(s, 1100);
  CHECK(l.fresh && !l.stale && l.source == oem::SourceMode::PYLONTECH);
  CHECK_NEAR(l.cvl, 54.6, 0.001);
  CHECK_NEAR(l.ccl, 30.0, 0.001);
  CHECK(l.allowed && !l.alarm && !l.blocking);
  CHECK(l.soc == 75);
  CHECK_NEAR(l.battery_v, 53.4, 0.001);
  CHECK(std::isnan(l.battery_a));
  CHECK_NEAR(l.max_cell_v, 3.345, 0.001);
}

static void test_poller_charge_enable_and_alarm() {
  PylonSource p;
  oem::Settings s = oem::default_settings();
  run_cycle(p, 0, A61, A62_ALARM, A63_OFF);
  Limits l = p.limits(s, 1100);
  CHECK(l.fresh && !l.allowed && l.alarm);
  CHECK_NEAR(l.ccl, 0.0, 0.001);
  // Keep 0x61 and 0x63 fresh but stop answering 0x62: after 3 s the source is stale, and as the last fresh state
  // was an alarm, charging stays off (no silent return to charging without the alarm channel).
  uint8_t out[REQUEST_LEN + 1];
  for (uint32_t t = 1500; t <= 5000; t += 500) {
    const size_t n = p.poll(t, out);
    CHECK(n == REQUEST_LEN);
    if (strncmp(reinterpret_cast<const char *>(out), "~201246610000", 13) == 0) feed(p, A61, t + 20);
    if (strncmp(reinterpret_cast<const char *>(out), "~201246630000", 13) == 0) feed(p, A63_ON, t + 20);
  }
  l = p.limits(s, 5100);
  CHECK(l.stale && !l.allowed);
}

static void test_poller_needs_the_alarm_reply() {
  PylonSource p;
  oem::Settings s = oem::default_settings();
  s.set(oem::HR_FALLBACK_A, 10);  // the default (3.13) is 0: no charging without the BMS
  run_cycle(p, 0, A61, A62_CLEAR, A63_ON);
  CHECK(p.limits(s, 1100).fresh && !p.limits(s, 1100).stale && p.limits(s, 1100).allowed);
  // 0x62 goes unanswered (or answers with an error) while 0x61 and 0x63 keep coming: stale (E15) after 3 s, on the
  // fallback limits; the last fresh state was fine, so the fallback may charge.
  uint8_t out[REQUEST_LEN + 1];
  for (uint32_t t = 1500; t <= 5000; t += 500) {
    CHECK(p.poll(t, out) == REQUEST_LEN);
    if (strncmp(reinterpret_cast<const char *>(out), "~201246610000", 13) == 0) feed(p, A61, t + 20);
    if (strncmp(reinterpret_cast<const char *>(out), "~201246630000", 13) == 0) feed(p, A63_ON, t + 20);
  }
  const Limits l = p.limits(s, 5100);
  CHECK(l.stale && l.allowed);
  CHECK_NEAR(l.ccl, 10.0, 0.001);  // Fallback current
  // 0x62 answers again: fresh as soon as it does.
  for (uint32_t t = 5500; t <= 6500; t += 500) {
    CHECK(p.poll(t, out) == REQUEST_LEN);
    const char *cmd = reinterpret_cast<const char *>(out);
    feed(p, strncmp(cmd, "~201246610000", 13) == 0 ? A61 : strncmp(cmd, "~201246620000", 13) == 0 ? A62_CLEAR : A63_ON,
         t + 20);
  }
  CHECK(!p.limits(s, 6600).stale);
}

static void test_poller_stale_fallback() {
  PylonSource p;
  oem::Settings s = oem::default_settings();  // Max current 10 A, Fallback current 0 A (3.13)
  Limits l = p.limits(s, 0);  // nothing heard yet: a fresh install charges nothing without the BMS
  CHECK(l.fresh && l.stale && l.source == oem::SourceMode::PYLONTECH && l.allowed);
  CHECK_NEAR(l.ccl, 0.0, 0.001);
  s.set(oem::HR_FALLBACK_A, 10);
  CHECK_NEAR(p.limits(s, 0).ccl, 10.0, 0.001);
  s.set(oem::HR_FALLBACK_A, 4);
  CHECK_NEAR(p.limits(s, 0).ccl, 4.0, 0.001);
  s.set(oem::HR_FALLBACK_A, 0);
  CHECK_NEAR(p.limits(s, 0).ccl, 0.0, 0.001);  // HR 9 = 0: no charging without the BMS
  s.set(oem::HR_FALLBACK_A, 10);
  run_cycle(p, 0, A61, A62_CLEAR, A63_ON);
  CHECK(!p.limits(s, 3050).stale);   // 0x61 at 50 ms: still within 3000 ms
  CHECK(p.limits(s, 3051).stale);    // 0x61 now older than 3 s
  CHECK_NEAR(p.limits(s, 3051).cvl, 57.6, 0.001);  // Panel Charge V while stale
}

// After fresh cycles at 0 and 1500 ms the BMS goes silent: stale from 4551 ms (the 0x61 at 1550 is then 3001 ms old).
static void go_silent_after(PylonSource &p, const char *a62, const char *a63) {
  run_cycle(p, 0, A61, A62_CLEAR, A63_ON);
  run_cycle(p, 1500, A61, a62, a63);
  uint8_t out[REQUEST_LEN + 1];
  for (uint32_t t = 3000; t <= 4000; t += 500) CHECK(p.poll(t, out) == REQUEST_LEN);  // unanswered; next is 0x61
}

static void test_poller_stale_after_a_bms_stop() {
  oem::Settings s = oem::default_settings();
  s.set(oem::HR_FALLBACK_A, 10);  // so "stopped" is told apart from a 0 A fallback
  {
    PylonSource p;  // charge enable off, then silence: still off, though stale
    go_silent_after(p, A62_CLEAR, A63_OFF);
    CHECK(!p.limits(s, 4550).stale && !p.limits(s, 4550).allowed);
    const Limits l = p.limits(s, 4551);
    CHECK(l.stale && !l.allowed);
    // Recovery: a fresh normal cycle allows charging again.
    run_cycle(p, 5000, A61, A62_CLEAR, A63_ON);
    CHECK(!p.limits(s, 6100).stale && p.limits(s, 6100).allowed);
  }
  {
    PylonSource p;  // an alarm, then silence
    go_silent_after(p, A62_ALARM, A63_ON);
    const Limits l = p.limits(s, 4551);
    CHECK(l.stale && !l.allowed);
    run_cycle(p, 5000, A61, A62_CLEAR, A63_ON);
    CHECK(!p.limits(s, 6100).stale && p.limits(s, 6100).allowed);
  }
  {
    PylonSource p;  // normal, then silence: the capped Panel fallback as before
    go_silent_after(p, A62_CLEAR, A63_ON);
    const Limits l = p.limits(s, 4551);
    CHECK(l.stale && l.allowed);
    CHECK_NEAR(l.ccl, 10.0, 0.001);
  }
}

// A valid reply that by itself says stop latches though the source never became fresh (no 0x61).
static void test_poller_latch_without_fresh() {
  oem::Settings s = oem::default_settings();
  uint8_t out[REQUEST_LEN + 1];
  {
    PylonSource p;  // 0x61 unanswered, 0x62 clear, then 0x63 charge enable off
    CHECK(p.poll(0, out) == REQUEST_LEN);
    CHECK(p.poll(500, out) == REQUEST_LEN);
    feed(p, A62_CLEAR, 550);
    CHECK(p.poll(1000, out) == REQUEST_LEN);
    feed(p, A63_OFF, 1050);
    CHECK(!p.fresh(1100));
    const Limits l = p.limits(s, 1100);
    CHECK(l.stale && !l.allowed);
  }
  {
    PylonSource p;  // 0x61 unanswered, a valid 0x62 alarm
    CHECK(p.poll(0, out) == REQUEST_LEN);
    CHECK(p.poll(500, out) == REQUEST_LEN);
    feed(p, A62_ALARM, 550);
    CHECK(p.limits(s, 600).stale && !p.limits(s, 600).allowed);
  }
  {
    PylonSource p;  // a clean 0x63 alone does not stop
    CHECK(p.poll(0, out) == REQUEST_LEN);
    CHECK(p.poll(500, out) == REQUEST_LEN);
    CHECK(p.poll(1000, out) == REQUEST_LEN);
    feed(p, A63_ON, 1050);
    CHECK(p.limits(s, 1100).stale && p.limits(s, 1100).allowed);
  }
}

static void test_poller_holds_a_request_while_a_frame_arrives() {
  uint8_t out[REQUEST_LEN + 1];
  const size_t half = strlen(A61) / 2;
  {
    PylonSource p;  // a reply half-received at the 500 ms mark: the 0x62 waits for its end
    CHECK(p.poll(0, out) == REQUEST_LEN);
    p.push(reinterpret_cast<const uint8_t *>(A61), half, 380);
    CHECK(p.poll(500, out) == 0);
    CHECK(p.poll(560, out) == 0);
    p.push(reinterpret_cast<const uint8_t *>(A61) + half, strlen(A61) - half, 570);  // ends late: not parsed
    CHECK(p.good_replies() == 0);
    CHECK(p.poll(571, out) == REQUEST_LEN && strncmp(reinterpret_cast<const char *>(out), "~201246620000FDA9\r", REQUEST_LEN) == 0);
    CHECK(p.poll(1070, out) == 0);  // the cycle restarts from the delayed request
    CHECK(p.poll(1071, out) == REQUEST_LEN);
  }
  {
    PylonSource p;  // a partial frame that never ends is abandoned PYLON_HOLD_MS after the request fell due
    CHECK(p.poll(0, out) == REQUEST_LEN);
    p.push(reinterpret_cast<const uint8_t *>(A61), half, 380);
    CHECK(p.poll(500, out) == 0);
    CHECK(p.poll(500 + oem::PYLON_HOLD_MS - 1, out) == 0);
    CHECK(p.poll(500 + oem::PYLON_HOLD_MS, out) == REQUEST_LEN);
    feed(p, A62_CLEAR, 1050);  // the new request's reply still parses
    CHECK(p.good_replies() == 1);
  }
}

static void test_poller_reply_window_and_noise() {
  PylonSource p;
  uint8_t out[REQUEST_LEN + 1];
  CHECK(p.poll(0, out) == REQUEST_LEN);
  feed(p, A61, 401);  // too late for the 0x61 request
  CHECK(p.good_replies() == 0);
  CHECK(p.poll(500, out) == REQUEST_LEN);  // 0x62 now outstanding
  feed(p, "garbage", 510);
  feed(p, A61, 520);  // a valid frame answering the outstanding request is parsed as the command polled (0x62)
  CHECK(p.good_replies() == 1);
  feed(p, A62_CLEAR, 530);  // the 0x62 is already answered: nothing outstanding, dropped
  CHECK(p.good_replies() == 1);
  CHECK(p.poll(1000, out) == REQUEST_LEN);  // 0x63
  char bad[64];
  strcpy(bad, A63_ON);
  bad[strlen(bad) - 2] = '6';
  feed(p, bad, 1010);
  CHECK(p.bad_replies() == 1);
  feed(p, "~201246630000FDA8\r", 1020);  // an echo of our own request: RTN != 0
  CHECK(p.bad_replies() == 2);
  feed(p, A63_ON, 1030);  // still within the window and still outstanding
  CHECK(p.good_replies() == 2);
}

static void test_poller_frame_split_and_reset() {
  PylonSource p;
  uint8_t out[REQUEST_LEN + 1];
  CHECK(p.poll(0, out) == REQUEST_LEN);
  const size_t half = strlen(A61) / 2;
  p.push(reinterpret_cast<const uint8_t *>(A61), half, 10);
  p.push(reinterpret_cast<const uint8_t *>(A61) + half, strlen(A61) - half, 20);
  CHECK(p.good_replies() == 1);
  p.reset();
  CHECK(p.good_replies() == 0 && !p.fresh(30));
  CHECK(p.poll(30, out) == REQUEST_LEN && strncmp(reinterpret_cast<const char *>(out), "~201246610000FDAA\r", REQUEST_LEN) == 0);  // restarts at 0x61
}

int main() {
  test_length_and_checksum();
  test_requests();
  test_decode_analog();
  test_decode_alarm();
  test_decode_mgmt();
  test_rejections();
  test_poller_fresh_limits();
  test_poller_charge_enable_and_alarm();
  test_poller_needs_the_alarm_reply();
  test_poller_stale_fallback();
  test_poller_stale_after_a_bms_stop();
  test_poller_latch_without_fresh();
  test_poller_holds_a_request_while_a_frame_arrives();
  test_poller_reply_window_and_noise();
  test_poller_frame_split_and_reset();
  return test_result("test_oem_pylon");
}
