#include "app/modbus_rtu.h"

#include "check.h"

using namespace oem;

namespace {

struct WriteLog {
  uint16_t start, count, first;
};

class FakeBank final : public RegisterBank {
 public:
  uint16_t hr[130] = {};
  uint16_t ir[50] = {};
  WriteLog log[8] = {};
  int writes = 0, reads = 0;
  uint8_t inject = 0;

  bool read_input(uint16_t reg, uint16_t &v) override {
    reads++;
    if (reg >= 50) return false;
    v = ir[reg];
    return true;
  }
  bool read_holding(uint16_t reg, uint16_t &v) override {
    reads++;
    if (reg >= 130) return false;
    v = hr[reg];
    return true;
  }
  uint8_t write_holding(uint16_t start, const uint16_t *values, uint16_t count) override {
    if (inject) return inject;
    if (writes < 8) log[writes] = {start, count, values[0]};
    writes++;
    return 0;
  }
};

// Builds a request frame from payload bytes (address first), appending the CRC.
size_t frame(uint8_t *out, const uint8_t *payload, size_t n) {
  for (size_t i = 0; i < n; i++) out[i] = payload[i];
  uint16_t crc = crc16_modbus(out, n);
  out[n] = uint8_t(crc & 0xFF);
  out[n + 1] = uint8_t(crc >> 8);
  return n + 2;
}

bool same_with_crc(const uint8_t *resp, size_t len, const uint8_t *expect, size_t n) {
  uint8_t e[MB_MAX_FRAME];
  size_t m = frame(e, expect, n);
  if (len != m) return false;
  for (size_t i = 0; i < m; i++)
    if (resp[i] != e[i]) return false;
  return true;
}

}  // namespace

static void test_crc() {
  FakeBank b;
  uint8_t req[16], resp[MB_MAX_FRAME];
  const uint8_t p[] = {1, 3, 0, 0, 0, 10};
  size_t n = frame(req, p, 6);
  CHECK(req[6] == 0xC5 && req[7] == 0xCD);
  CHECK(modbus_handle(1, req, n, resp, b) > 0);
  req[3] ^= 1;
  CHECK(modbus_handle(1, req, n, resp, b) == 0);
}

static void test_read() {
  FakeBank b;
  b.hr[0] = 10, b.hr[1] = 5760, b.hr[2] = 20;
  b.ir[3] = 0x1234;
  uint8_t req[16], resp[MB_MAX_FRAME];
  const uint8_t r3[] = {1, 3, 0, 0, 0, 3};
  size_t n = modbus_handle(1, req, frame(req, r3, 6), resp, b);
  const uint8_t e3[] = {1, 3, 6, 0x00, 0x0A, 0x16, 0x80, 0x00, 0x14};
  CHECK(same_with_crc(resp, n, e3, sizeof e3));

  const uint8_t r4[] = {1, 4, 0, 3, 0, 1};
  n = modbus_handle(1, req, frame(req, r4, 6), resp, b);
  const uint8_t e4[] = {1, 4, 2, 0x12, 0x34};
  CHECK(same_with_crc(resp, n, e4, sizeof e4));

  const uint8_t big[] = {1, 4, 0, 0, 0, 126};
  n = modbus_handle(1, req, frame(req, big, 6), resp, b);
  const uint8_t eb[] = {1, 0x84, 3};
  CHECK(same_with_crc(resp, n, eb, sizeof eb));

  const uint8_t zero[] = {1, 3, 0, 0, 0, 0};
  n = modbus_handle(1, req, frame(req, zero, 6), resp, b);
  const uint8_t ez[] = {1, 0x83, 3};
  CHECK(same_with_crc(resp, n, ez, sizeof ez));
}

static void test_read_range() {
  FakeBank b;
  uint8_t req[16], resp[MB_MAX_FRAME];
  const uint8_t r[] = {1, 3, 0, 129, 0, 2};  // 129 ok, 130 missing
  size_t n = modbus_handle(1, req, frame(req, r, 6), resp, b);
  const uint8_t e[] = {1, 0x83, 2};
  CHECK(same_with_crc(resp, n, e, sizeof e));
}

static void test_read_limits() {
  FakeBank b;
  b.hr[0] = 0xABCD, b.hr[124] = 0x1234;
  uint8_t req[300], resp[MB_MAX_FRAME];
  const uint8_t r[] = {1, 3, 0, 0, 0, 125};
  size_t n = modbus_handle(1, req, frame(req, r, 6), resp, b);
  CHECK(n == 255 && resp[2] == 250 && resp[3] == 0xAB && resp[251] == 0x12 && resp[252] == 0x34);
  const uint8_t e[] = {1, 0x83, 3};
  const uint8_t f3[] = {1, 3, 0, 0, 0, 1, 0};
  const uint8_t f4[] = {1, 4, 0, 0, 0, 1, 0};
  const uint8_t f6[] = {1, 6, 0, 0, 0, 1, 0};
  n = modbus_handle(1, req, frame(req, f3, 7), resp, b);
  CHECK(same_with_crc(resp, n, e, sizeof e));
  const uint8_t e4[] = {1, 0x84, 3}, e6[] = {1, 0x86, 3};
  n = modbus_handle(1, req, frame(req, f4, 7), resp, b);
  CHECK(same_with_crc(resp, n, e4, sizeof e4));
  n = modbus_handle(1, req, frame(req, f6, 7), resp, b);
  CHECK(same_with_crc(resp, n, e6, sizeof e6));
  CHECK(b.writes == 0);
}

static void test_broadcast_malformed() {
  FakeBank b;
  uint8_t req[32], resp[MB_MAX_FRAME];
  const uint8_t w6[] = {0, 6, 0, 9, 0, 42, 0};
  CHECK(modbus_handle(1, req, frame(req, w6, 7), resp, b) == 0);
  const uint8_t w16[] = {0, 16, 0, 7, 0, 2, 3, 0, 1, 0, 2};
  CHECK(modbus_handle(1, req, frame(req, w16, sizeof w16), resp, b) == 0);
  CHECK(b.writes == 0);
}

static void test_write_single() {
  FakeBank b;
  uint8_t req[16], resp[MB_MAX_FRAME];
  const uint8_t w[] = {1, 6, 0, 5, 0x12, 0x34};
  size_t rl = frame(req, w, 6);
  size_t n = modbus_handle(1, req, rl, resp, b);
  CHECK(n == rl);
  for (size_t i = 0; i < n; i++) CHECK(resp[i] == req[i]);
  CHECK(b.writes == 1 && b.log[0].start == 5 && b.log[0].count == 1 && b.log[0].first == 0x1234);

  FakeBank f;
  f.inject = 3;
  n = modbus_handle(1, req, rl, resp, f);
  const uint8_t e[] = {1, 0x86, 3};
  CHECK(same_with_crc(resp, n, e, sizeof e));
  CHECK(f.writes == 0);
}

static void test_write_multiple() {
  FakeBank b;
  uint8_t req[32], resp[MB_MAX_FRAME];
  const uint8_t w[] = {1, 16, 0, 7, 0, 2, 4, 0, 1, 0, 2};
  size_t n = modbus_handle(1, req, frame(req, w, sizeof w), resp, b);
  const uint8_t e[] = {1, 16, 0, 7, 0, 2};
  CHECK(same_with_crc(resp, n, e, sizeof e));
  CHECK(b.writes == 1 && b.log[0].start == 7 && b.log[0].count == 2 && b.log[0].first == 1);

  const uint8_t bad[] = {1, 16, 0, 7, 0, 2, 3, 0, 1, 0, 2};  // byte count mismatch
  n = modbus_handle(1, req, frame(req, bad, sizeof bad), resp, b);
  const uint8_t e3[] = {1, 0x90, 3};
  CHECK(same_with_crc(resp, n, e3, sizeof e3));
  CHECK(b.writes == 1);

  const uint8_t shortf[] = {1, 16, 0, 7, 0, 2, 4, 0, 1};  // frame shorter than the count says
  n = modbus_handle(1, req, frame(req, shortf, sizeof shortf), resp, b);
  CHECK(same_with_crc(resp, n, e3, sizeof e3));

  b.inject = 6;
  n = modbus_handle(1, req, frame(req, w, sizeof w), resp, b);
  const uint8_t e6[] = {1, 0x90, 6};
  CHECK(same_with_crc(resp, n, e6, sizeof e6));
}

static void test_other_fc() {
  FakeBank b;
  uint8_t req[16], resp[MB_MAX_FRAME];
  const uint8_t p[] = {1, 5, 0, 0, 0xFF, 0};
  size_t n = modbus_handle(1, req, frame(req, p, 6), resp, b);
  const uint8_t e[] = {1, 0x85, 1};
  CHECK(same_with_crc(resp, n, e, sizeof e));
}

static void test_address() {
  FakeBank b;
  uint8_t req[16], resp[MB_MAX_FRAME];
  const uint8_t other[] = {2, 3, 0, 0, 0, 1};
  CHECK(modbus_handle(1, req, frame(req, other, 6), resp, b) == 0);
  const uint8_t bw[] = {0, 6, 0, 9, 0, 42};
  CHECK(modbus_handle(1, req, frame(req, bw, 6), resp, b) == 0);
  CHECK(b.writes == 1 && b.log[0].start == 9 && b.log[0].first == 42);
  const uint8_t br[] = {0, 3, 0, 0, 0, 1};
  CHECK(modbus_handle(1, req, frame(req, br, 6), resp, b) == 0);
  CHECK(b.reads == 0);
  const uint8_t bf[] = {0, 5, 0, 0, 0, 0};  // unknown FC broadcast: silent too
  CHECK(modbus_handle(1, req, frame(req, bf, 6), resp, b) == 0);
}

static void test_short() {
  FakeBank b;
  uint8_t resp[MB_MAX_FRAME];
  const uint8_t s[] = {1, 3, 0};
  CHECK(modbus_handle(1, s, 3, resp, b) == 0);
}

static void test_gap() {
  CHECK(modbus_gap_ms(9600) == 6);
  CHECK(modbus_gap_ms(19200) == 5);  // never under 5: a 1-2 ms main-loop pass mustn't split a frame
  CHECK(modbus_gap_ms(38400) == 5);
  CHECK(modbus_gap_ms(115200) == 5);
  CHECK(modbus_gap_ms(0) == 5);
}

static void test_assembler() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  const uint8_t d[] = {1, 2, 3, 4};
  a.push(d, 2, 0);
  a.push(d + 2, 2, 3);
  CHECK(a.poll(8, f) == 0);
  CHECK(a.poll(9, f) == 4);
  CHECK(f[0] == 1 && f[3] == 4);
  CHECK(a.poll(20, f) == 0);  // delivered once

  const uint8_t x[] = {9, 8}, y[] = {7, 6, 5};
  a.push(x, 2, 100);
  CHECK(a.poll(106, f) == 2 && f[0] == 9);
  a.push(y, 3, 110);
  CHECK(a.poll(116, f) == 3 && f[0] == 7);

  uint8_t big[100] = {};
  a.push(big, 100, 200);
  a.push(big, 100, 201);
  a.push(big, 100, 202);  // 300 bytes, no gap
  CHECK(a.poll(208, f) == 0);
  a.push(x, 2, 300);
  CHECK(a.poll(306, f) == 2 && f[0] == 9 && f[1] == 8);
}

static void test_assembler_sizes() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  uint8_t big[257] = {};
  a.push(big, 256, 0);
  CHECK(a.poll(6, f) == 256);
  a.push(big, 257, 100);
  CHECK(a.poll(106, f) == 0);
}

static void test_assembler_overflow_unpolled_gap() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  uint8_t big[300] = {};
  a.push(big, 300, 0);  // overflow, never polled
  const uint8_t d[] = {5, 6, 7};
  a.push(d, 3, 50);
  CHECK(a.poll(56, f) == 3 && f[0] == 5);
}

static void test_assembler_wrap() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  const uint8_t d[] = {1, 2};
  a.push(d, 2, 0xFFFFFFFEu);
  CHECK(a.poll(3, f) == 0);  // 5 ms later
  CHECK(a.poll(4, f) == 2);  // 6 ms later, past the wrap
}

static void test_discard_floor_expires() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  const uint8_t d[] = {1};
  a.discard_until(10);
  a.push(d, 1, 10);
  CHECK(a.poll(16, f) == 1);
  a.push(d, 1, 10u + 0x80000000u);
  CHECK(a.poll(16u + 0x80000000u, f) == 1);
}

// A floor nobody pushed past is dropped by poll() once reached, so a long silence can't wrap it into the future.
static void test_discard_floor_expires_on_poll() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  const uint8_t d[] = {1};
  a.discard_until(10);
  CHECK(a.poll(11, f) == 0);
  a.push(d, 1, 11u + 0x80000000u);
  CHECK(a.poll(17u + 0x80000000u, f) == 1);
}

static void test_discard() {
  FrameAssembler a;
  a.set_gap_ms(6);
  const uint8_t *f = nullptr;
  const uint8_t d[] = {1, 2, 3};
  a.push(d, 3, 5);
  a.discard_until(10);
  a.push(d, 1, 8);  // before the cut
  CHECK(a.poll(30, f) == 0);
  a.push(d + 1, 2, 12);
  CHECK(a.poll(18, f) == 2 && f[0] == 2);
}

int main() {
  test_crc();
  test_read();
  test_read_range();
  test_read_limits();
  test_broadcast_malformed();
  test_write_single();
  test_write_multiple();
  test_other_fc();
  test_address();
  test_short();
  test_gap();
  test_assembler();
  test_discard();
  test_discard_floor_expires();
  test_discard_floor_expires_on_poll();
  test_assembler_sizes();
  test_assembler_overflow_unpolled_gap();
  test_assembler_wrap();
  return test_result("oem_modbus");
}
