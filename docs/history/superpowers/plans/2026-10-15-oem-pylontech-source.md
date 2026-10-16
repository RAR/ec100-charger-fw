# OEM Pylontech Source Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** In Source mode Pylontech (HR 6 = 1, set from the panel menu), the OEM charger polls a BMS on its RS485 jack in the stock charger's dialect (CID2 0x61/0x62/0x63) and charges within the BMS's CVL/CCL/charge-enable, falling back to capped Panel limits (E15) when the BMS goes quiet.

**Architecture:** A pure framing/decoding header (`pylon_protocol.h`) and a pure poller (`pylon_source.h`, request cadence, reply assembly, freshness, `Limits`) are host-tested; `Limits`/`Status` gain `stale` and `source`; `main.cpp` gives USART1 to either the existing Modbus server (Panel/Modbus) or the poller (Pylontech), following HR 6 at run time. A Python BMS simulator drives the live test through the WiFi bridge's Passthrough.

**Tech Stack:** C++17 (GD32 app + host tests under `make -C tests`), Python 3 tools with pytest.

**Spec:** `docs/history/superpowers/specs/2026-10-15-oem-pylontech-source-design.md` (parent: `docs/history/superpowers/specs/2026-10-14-oem-board-firmware-design.md`)

## Global Constraints

- Dialect: requests VER 0x20, ADR 0x12, CID1 0x46, CID2 0x61/0x62/0x63, empty INFO, 9600 8N1. Exact request strings: `~201246610000FDAA\r`, `~201246620000FDA9\r`, `~201246630000FDA8\r`.
- Reply accepted only if: `~` first, `\r` last, all else hex, length = 18 + LENID, LENID even, LCHKSUM and CHKSUM correct, CID1 = 0x46, RTN = 0x00, INFO bytes ≥ 13 (0x61) / 4 (0x62) / 9 (0x63). VER and ADR not checked.
- Units: 0x61 total V mV (bytes 0-1), SOC % (byte 4; > 100 = unknown), max cell V mV (bytes 11-12); 0x61 current NOT used (battery_a stays NaN); 0x62 alarm = `INFO[2] & 0xAA || INFO[3] & 0x68`; 0x63 CVL mV (0-1), DVL mV (2-3), CCL 0.1 A (4-5), DCL 0.1 A (6-7), status byte 8, bit 7 = charge enable.
- Poll one request every 500 ms cycling 0x61 → 0x62 → 0x63; a reply counts only within 400 ms of its request. Fresh = valid 0x61 and valid 0x63 both within 3000 ms; 0x62 older than 3000 ms = no alarm.
- Stale (incl. before the first replies): Panel limits with ccl = min(Max current, Fallback current HR 9), `stale` true → E15.
- Source mode: 0 Panel and 2 Modbus → Modbus server (as today); 1 → Pylontech poller, Modbus server off. Modbus writes of HR 6 keep refusing non-Panel (don't change `modbus_map.h`'s rule).
- Firmware version becomes 3.4 (`OEM_FW_VERSION = 0x0304`).
- Host tests: `make -C tests test` (all suites ok, `-Wall -Wextra -Werror`); Python: `.venv/bin/python -m pytest -q`; firmware build: `cmake --build oem/build` (writes `oem/build/charger.dat`).
- Never flash, never push, never open `esphome/secrets.yaml`. Commit messages end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Q88Pr4eMBeeEdBYp5fsSSu
  ```

**Test vectors** (computed with the user's YamBMS charger proxy encoder; used verbatim below):
- 0x61, 53.400 V, 3.0 A, SOC 75, max cell 3.345 V: `~200246008062D098012C4B000A000A64640D1101010D0201080BBE0BD701030BA501090BBE0BD701010BA501010BBE0BD701010BA50101E8C4\r`
- 0x62 clear: `~20024600800800000000FC22\r`; 0x62 with INFO[2] = 0x02 (alarm): `~20024600800800000200FC20\r`
- 0x63 on (CVL 54.600, DVL 44.000, CCL 30.0, DCL 100.0, status 0xC0): `~20024600D012D548ABE0012C03E8C0F995\r`
- 0x63 off (CVL 53.400, CCL 0.0, status 0x40): `~20024600D012D098ABE0000003E840F9BA\r`

---

### Task 1: Pylontech framing and decoding (`pylon_protocol.h`)

**Files:**
- Create: `oem/src/app/pylon_protocol.h`
- Create: `tests/test_oem_pylon.cpp`
- Modify: `tests/Makefile` (append `test_oem_pylon` to `TESTS`), `.gitignore` (add `tests/test_oem_pylon` next to the other `tests/test_*` lines)

**Interfaces:**
- Produces (namespace `oem::pylon`): constants `CID1`, `CID_ANALOG`, `CID_ALARM`, `CID_MGMT`, `REQUEST_LEN` (18), `FRAME_OVERHEAD` (18), `MAX_FRAME` (256), `MAX_INFO`; `size_t min_info(uint8_t cid2)`; `uint16_t length_field(uint16_t lenid)`; `uint16_t checksum(const char *s, size_t n)`; `size_t build_request(uint8_t cid2, char *out)`; `enum class Error { OK, FORMAT, LENGTH, LCHKSUM, CHKSUM, CID1, RTN, SHORT }`; `Error parse_reply(const char *f, size_t n, size_t need, uint8_t *info, size_t &info_len)`; `struct Analog { float battery_v; int soc; float max_cell_v; }` + `Analog decode_analog(const uint8_t *info)`; `bool decode_alarm(const uint8_t *info)`; `struct Mgmt { float cvl, dvl, ccl, dcl; uint8_t status; bool charge_enable; }` + `Mgmt decode_mgmt(const uint8_t *info)`.

- [ ] **Step 1: Write the failing test** — `tests/test_oem_pylon.cpp`:

```cpp
#include <cstring>

#include "app/pylon_protocol.h"
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

int main() {
  test_length_and_checksum();
  test_requests();
  test_decode_analog();
  test_decode_alarm();
  test_decode_mgmt();
  test_rejections();
  return test_result("test_oem_pylon");
}
```

The hand-edited CID1 and RTN frames carry their correct checksum (F994, computed), so they fail on CID1/RTN, not CHKSUM; the `D014` frame fails on LENGTH (checked before CHKSUM).

- [ ] **Step 2: Run it to verify it fails**

Run: `make -C tests test_oem_pylon`
Expected: compile error, `app/pylon_protocol.h: No such file or directory`.

- [ ] **Step 3: Implement** — `oem/src/app/pylon_protocol.h`:

```cpp
// oem/src/app/pylon_protocol.h: Pylontech RS485 framing (ASCII) and the stock charger's dialect: the CID2
// 0x61/0x62/0x63 requests and the reply fields we use. docs/history/superpowers/specs/2026-10-15-oem-pylontech-source-design.md
// §2-3; layouts from docs/history/notes/pylon-stock-protocol.md and the user's YamBMS charger proxy.
#pragma once
#include <cstddef>
#include <cstdint>

namespace oem {
namespace pylon {

constexpr uint8_t CID1 = 0x46;
constexpr uint8_t CID_ANALOG = 0x61, CID_ALARM = 0x62, CID_MGMT = 0x63;
constexpr size_t REQUEST_LEN = 18;     // "~201246xx0000CCCC\r"
constexpr size_t FRAME_OVERHEAD = 18;  // every character but INFO
constexpr size_t MAX_FRAME = 256;
constexpr size_t MAX_INFO = (MAX_FRAME - FRAME_OVERHEAD) / 2;  // decoded INFO bytes

// The fewest INFO bytes each reply must carry for the fields we read.
inline size_t min_info(uint8_t cid2) { return cid2 == CID_ANALOG ? 13 : cid2 == CID_ALARM ? 4 : 9; }

// LENGTH: LCHKSUM (top nibble) over LENID's three nibbles, then LENID = ASCII characters of INFO.
inline uint16_t length_field(uint16_t lenid) {
  const unsigned s = ((lenid >> 8) & 0xFu) + ((lenid >> 4) & 0xFu) + (lenid & 0xFu);
  return uint16_t((((~s + 1u) & 0xFu) << 12) | (lenid & 0x0FFFu));
}

// CHKSUM: two's complement of the ASCII sum of everything between '~' and the checksum.
inline uint16_t checksum(const char *s, size_t n) {
  uint32_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += uint8_t(s[i]);
  return uint16_t((~sum + 1u) & 0xFFFFu);
}

inline char hex_digit(unsigned v) { return "0123456789ABCDEF"[v & 0xFu]; }
inline int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}
// The byte in two hex characters at p (callers have checked they are hex).
inline int hex_byte(const char *p) { return (hex_value(p[0]) << 4) | hex_value(p[1]); }

// The stock request: VER 0x20, ADR 0x12, CID1 0x46, cid2, empty INFO. Writes REQUEST_LEN characters (no NUL).
inline size_t build_request(uint8_t cid2, char *out) {
  const char head[12] = {'2', '0', '1', '2', '4', '6', hex_digit(cid2 >> 4), hex_digit(cid2), '0', '0', '0', '0'};
  out[0] = '~';
  for (int i = 0; i < 12; i++) out[1 + i] = head[i];
  const uint16_t c = checksum(out + 1, 12);
  for (int i = 0; i < 4; i++) out[13 + i] = hex_digit(unsigned(c) >> (12 - 4 * i));
  out[17] = '\r';
  return REQUEST_LEN;
}

enum class Error : uint8_t { OK, FORMAT, LENGTH, LCHKSUM, CHKSUM, CID1, RTN, SHORT };

// One reply frame, '~' to '\r' (n characters). VER and ADR aren't checked. On OK, the INFO bytes are decoded into
// info[] (info_len of them, at most MAX_INFO); otherwise info_len is 0.
inline Error parse_reply(const char *f, size_t n, size_t need, uint8_t *info, size_t &info_len) {
  info_len = 0;
  if (n < FRAME_OVERHEAD || n > MAX_FRAME || f[0] != '~' || f[n - 1] != '\r') return Error::FORMAT;
  for (size_t i = 1; i < n - 1; i++) {
    if (hex_value(f[i]) < 0) return Error::FORMAT;
  }
  const uint16_t length = uint16_t((hex_byte(f + 9) << 8) | hex_byte(f + 11));
  const uint16_t lenid = length & 0x0FFFu;
  if (n != FRAME_OVERHEAD + lenid || (lenid & 1u)) return Error::LENGTH;
  if (length_field(lenid) != length) return Error::LCHKSUM;
  const uint16_t sent = uint16_t((hex_byte(f + n - 5) << 8) | hex_byte(f + n - 3));
  if (checksum(f + 1, n - 6) != sent) return Error::CHKSUM;
  if (hex_byte(f + 5) != CID1) return Error::CID1;
  if (hex_byte(f + 7) != 0) return Error::RTN;
  if (lenid / 2u < need) return Error::SHORT;
  info_len = lenid / 2u;
  for (size_t i = 0; i < info_len; i++) info[i] = uint8_t(hex_byte(f + 13 + 2 * i));
  return Error::OK;
}

inline uint16_t be16(const uint8_t *p) { return uint16_t((p[0] << 8) | p[1]); }

struct Analog {
  float battery_v;   // V
  int soc;           // %, -1 when out of range
  float max_cell_v;  // V
};
// 0x61. The current at bytes 2-3 isn't used: its unit is unconfirmed (spec §1).
inline Analog decode_analog(const uint8_t *info) {
  return {be16(info) / 1000.0f, info[4] <= 100 ? int(info[4]) : -1, be16(info + 11) / 1000.0f};
}

// 0x62: stock v2's error-12 mask over the protection bytes.
inline bool decode_alarm(const uint8_t *info) { return (info[2] & 0xAAu) != 0 || (info[3] & 0x68u) != 0; }

struct Mgmt {
  float cvl, dvl;  // V
  float ccl, dcl;  // A
  uint8_t status;
  bool charge_enable;  // status bit 7
};
// 0x63.
inline Mgmt decode_mgmt(const uint8_t *info) {
  return {be16(info) / 1000.0f, be16(info + 2) / 1000.0f, be16(info + 4) / 10.0f, be16(info + 6) / 10.0f, info[8],
          (info[8] & 0x80u) != 0};
}

}  // namespace pylon
}  // namespace oem
```

- [ ] **Step 4: Run all host tests**

Run: `make -C tests test`
Expected: every suite `ok`, including `test_oem_pylon: ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/pylon_protocol.h tests/test_oem_pylon.cpp tests/Makefile .gitignore
git commit -m "OEM: Pylontech framing and the stock dialect's reply decoding, host-tested"
```
(with the attribution lines)

---

### Task 2: Pylontech poller and limits; `stale`/`source` through the controller

**Files:**
- Create: `oem/src/app/pylon_source.h`
- Modify: `oem/src/app/limit_source.h` (add `stale`, `source` to `Limits`; update `panel_limits`)
- Modify: `oem/src/app/controller.h` (`Status` gains `uint8_t source`, `bool stale`; E15 from `lim.stale`)
- Test: `tests/test_oem_pylon.cpp` (append), `tests/test_oem_controller.cpp` (append)

**Interfaces:**
- Consumes: Task 1's `oem::pylon` API.
- Produces: `oem::PylonSource` with `void push(const uint8_t *data, size_t n, uint32_t now)`, `size_t poll(uint32_t now, char *out)` (returns `pylon::REQUEST_LEN` when a request is due, else 0), `bool fresh(uint32_t now) const`, `Limits limits(const Settings &s, uint32_t now) const`, `void reset()`, `uint32_t good_replies() const`, `uint32_t bad_replies() const`; constants `PYLON_POLL_MS = 500`, `PYLON_REPLY_MS = 400`, `PYLON_FRESH_MS = 3000`. `Limits` field order becomes `cvl, ccl, allowed, alarm, blocking, soc, battery_v, battery_a, max_cell_v, fresh, stale, source`. `Status::source` (0 Panel, 1 Pylontech) and `Status::stale`.

- [ ] **Step 1: Extend `Limits`** — in `oem/src/app/limit_source.h`, append two fields to `struct Limits` after `fresh`:

```cpp
  bool stale;         // a non-panel source is configured but not fresh: these are its Panel fallback limits
  SourceMode source;  // the source these came from
```

and make `panel_limits` return them (Panel is never stale):

```cpp
inline Limits panel_limits(const Settings &s) {
  return {float(s.get(HR_CHARGE_V)) / 100.0f, float(s.get(HR_MAX_CURRENT)), true, false, false, -1,
          NAN, NAN, NAN, true, false, SourceMode::PANEL};
}
```

Update the file's header comment: the Pylontech source is in `pylon_source.h`; Modbus is still unbuilt.

- [ ] **Step 2: Write the failing poller tests** — append to `tests/test_oem_pylon.cpp` (before `main`), add `#include "app/pylon_source.h"` at the top, and call the new tests from `main`:

```cpp
using oem::Limits;
using oem::PylonSource;

static void feed(PylonSource &p, const char *frame, uint32_t now) {
  p.push(reinterpret_cast<const uint8_t *>(frame), strlen(frame), now);
}

// Requests at 0, 500, 1000 ms (0x61, 0x62, 0x63), each answered 50 ms later.
static void run_cycle(PylonSource &p, uint32_t t0, const char *a61, const char *a62, const char *a63) {
  char out[REQUEST_LEN + 1] = {};
  CHECK(p.poll(t0, out) == REQUEST_LEN && strncmp(out, "~201246610000FDAA\r", REQUEST_LEN) == 0);
  feed(p, a61, t0 + 50);
  CHECK(p.poll(t0 + 499, out) == 0);
  CHECK(p.poll(t0 + 500, out) == REQUEST_LEN && strncmp(out, "~201246620000FDA9\r", REQUEST_LEN) == 0);
  feed(p, a62, t0 + 550);
  CHECK(p.poll(t0 + 1000, out) == REQUEST_LEN && strncmp(out, "~201246630000FDA8\r", REQUEST_LEN) == 0);
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
  // Keep 0x61 and 0x63 fresh but stop answering 0x62: the alarm expires after 3 s.
  char out[REQUEST_LEN + 1];
  for (uint32_t t = 1500; t <= 5000; t += 500) {
    const size_t n = p.poll(t, out);
    CHECK(n == REQUEST_LEN);
    if (strncmp(out, "~201246610000", 13) == 0) feed(p, A61, t + 20);
    if (strncmp(out, "~201246630000", 13) == 0) feed(p, A63_ON, t + 20);
  }
  l = p.limits(s, 5100);
  CHECK(l.fresh && l.allowed && !l.alarm);
}

static void test_poller_stale_fallback() {
  PylonSource p;
  oem::Settings s = oem::default_settings();  // Max current 10 A, Fallback current 10 A
  Limits l = p.limits(s, 0);  // nothing heard yet
  CHECK(l.fresh && l.stale && l.source == oem::SourceMode::PYLONTECH && l.allowed);
  CHECK_NEAR(l.ccl, 10.0, 0.001);
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

static void test_poller_reply_window_and_noise() {
  PylonSource p;
  char out[REQUEST_LEN + 1];
  CHECK(p.poll(0, out) == REQUEST_LEN);
  feed(p, A61, 401);  // too late for the 0x61 request
  CHECK(p.good_replies() == 0);
  CHECK(p.poll(500, out) == REQUEST_LEN);  // 0x62 now outstanding
  feed(p, "garbage", 510);
  feed(p, A61, 520);  // a valid frame, but 0x62 needs 4+ bytes and this one is parsed as 0x62: it has 49, fine...
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
  char out[REQUEST_LEN + 1];
  CHECK(p.poll(0, out) == REQUEST_LEN);
  const size_t half = strlen(A61) / 2;
  p.push(reinterpret_cast<const uint8_t *>(A61), half, 10);
  p.push(reinterpret_cast<const uint8_t *>(A61) + half, strlen(A61) - half, 20);
  CHECK(p.good_replies() == 1);
  p.reset();
  CHECK(p.good_replies() == 0 && !p.fresh(30));
  CHECK(p.poll(30, out) == REQUEST_LEN && strncmp(out, "~201246610000FDAA\r", REQUEST_LEN) == 0);  // restarts at 0x61
}
```

In `test_poller_reply_window_and_noise`, the A61 frame answering the outstanding 0x62 request is accepted (it is parsed as 0x62, and 49 ≥ 4 bytes) — exactly stock's "parse as the command polled" rule; the test pins that behaviour. Add `#include <cmath>` for `std::isnan`.

- [ ] **Step 3: Run to verify failure**

Run: `make -C tests test_oem_pylon`
Expected: compile error, `app/pylon_source.h: No such file or directory`.

- [ ] **Step 4: Implement** — `oem/src/app/pylon_source.h`:

```cpp
// oem/src/app/pylon_source.h: the Pylontech limit source (spec 2026-10-15-oem-pylontech-source-design.md §4-5). The
// board is the RS485 master: one request every PYLON_POLL_MS cycling 0x61 -> 0x62 -> 0x63; a reply counts within
// PYLON_REPLY_MS of its request and is parsed as the command polled. Fresh while a 0x61 and a 0x63 have both arrived
// within PYLON_FRESH_MS; otherwise Panel limits capped at Fallback current, flagged stale (E15).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/limit_source.h"
#include "app/pylon_protocol.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t PYLON_POLL_MS = 500;
constexpr uint32_t PYLON_REPLY_MS = 400;
constexpr uint32_t PYLON_FRESH_MS = 3000;

class PylonSource {
 public:
  // A mode change: forget the BMS (stale until it answers again) and start the cycle over at 0x61.
  void reset() { *this = PylonSource(); }

  // Every pass, with the bytes received since the last call.
  void push(const uint8_t *data, size_t n, uint32_t now) {
    for (size_t i = 0; i < n; i++) push_byte_(char(data[i]), now);
  }

  // Every pass while the line is free. When a request is due, writes it to out and returns its length; else 0.
  size_t poll(uint32_t now, char *out) {
    if (started_ && now - last_req_ms_ < PYLON_POLL_MS) return 0;
    started_ = true;
    last_req_ms_ = now;
    pending_ = CYCLE[next_];
    next_ = uint8_t((next_ + 1) % 3);
    in_frame_ = false;
    len_ = 0;
    return pylon::build_request(pending_, out);
  }

  bool fresh(uint32_t now) const { return recent_(has_analog_, analog_ms_, now) && recent_(has_mgmt_, mgmt_ms_, now); }

  Limits limits(const Settings &s, uint32_t now) const {
    if (!fresh(now)) {
      Limits l = panel_limits(s);
      l.ccl = std::min(l.ccl, float(s.get(HR_FALLBACK_A)));
      l.stale = true;
      l.source = SourceMode::PYLONTECH;
      return l;
    }
    return {mgmt_.cvl,
            mgmt_.ccl,
            mgmt_.charge_enable,
            recent_(has_alarm_, alarm_ms_, now) && alarm_,
            false,
            analog_.soc,
            analog_.battery_v,
            NAN,  // the 0x61 current's unit is unconfirmed (spec §1)
            analog_.max_cell_v,
            true,
            false,
            SourceMode::PYLONTECH};
  }

  uint32_t good_replies() const { return good_; }
  uint32_t bad_replies() const { return bad_; }

 private:
  static constexpr uint8_t CYCLE[3] = {pylon::CID_ANALOG, pylon::CID_ALARM, pylon::CID_MGMT};

  static bool recent_(bool has, uint32_t at, uint32_t now) { return has && now - at <= PYLON_FRESH_MS; }

  void push_byte_(char c, uint32_t now) {
    if (pending_ == 0 || now - last_req_ms_ > PYLON_REPLY_MS) {
      in_frame_ = false;
      return;
    }
    if (c == '~') {
      in_frame_ = true;
      len_ = 0;
    }
    if (!in_frame_) return;
    if (len_ >= pylon::MAX_FRAME) {
      in_frame_ = false;
      bad_++;
      return;
    }
    buf_[len_++] = c;
    if (c != '\r') return;
    in_frame_ = false;
    uint8_t info[pylon::MAX_INFO];
    size_t n = 0;
    if (pylon::parse_reply(buf_, len_, pylon::min_info(pending_), info, n) != pylon::Error::OK) {
      bad_++;
      return;
    }
    good_++;
    if (pending_ == pylon::CID_ANALOG) {
      analog_ = pylon::decode_analog(info);
      has_analog_ = true;
      analog_ms_ = now;
    } else if (pending_ == pylon::CID_ALARM) {
      alarm_ = pylon::decode_alarm(info);
      has_alarm_ = true;
      alarm_ms_ = now;
    } else {
      mgmt_ = pylon::decode_mgmt(info);
      has_mgmt_ = true;
      mgmt_ms_ = now;
    }
    pending_ = 0;
  }

  char buf_[pylon::MAX_FRAME] = {};
  size_t len_ = 0;
  bool in_frame_ = false;
  bool started_ = false;
  uint8_t next_ = 0;
  uint8_t pending_ = 0;  // CID2 awaiting its reply, 0 = none
  uint32_t last_req_ms_ = 0;
  pylon::Analog analog_{NAN, -1, NAN};
  pylon::Mgmt mgmt_{NAN, NAN, 0.0f, 0.0f, 0, false};
  bool alarm_ = false;
  bool has_analog_ = false, has_mgmt_ = false, has_alarm_ = false;
  uint32_t analog_ms_ = 0, mgmt_ms_ = 0, alarm_ms_ = 0;
  uint32_t good_ = 0, bad_ = 0;
};

}  // namespace oem
```

- [ ] **Step 5: Controller: `stale`, `source`, E15** — in `oem/src/app/controller.h`:
  - add to `struct Status`, after `no_current`: `bool stale = false;  // the configured non-panel source isn't fresh (E15)` and `uint8_t source = 0;  // SourceMode of the limits in use (IR 5)`;
  - in `tick`, set `st_.stale = lim.stale;` and `st_.source = uint8_t(lim.source);` next to the other `st_` assignments;
  - change the E15 branch from `} else if (!lim.fresh) {` to `} else if (lim.stale || !lim.fresh) {` (keep `E_SOURCE_STALE`).

  Append to `tests/test_oem_controller.cpp` (and call it from `main`):

```cpp
static void test_stale_source_e15_and_fields() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.error == 0 && !g.st.stale && g.st.source == 0);
  Limits lim = panel_limits(g.s);
  lim.stale = true;
  lim.source = SourceMode::PYLONTECH;
  lim.ccl = 4.0f;  // as PylonSource would cap it
  for (int i = 0; i < 3; i++) {
    for (int t = 0; t < 1000; t++, g.now++) {
      g.r1.loop(g.now);
      g.r2.loop(g.now);
      g.bus.deliver({&g.m1, &g.m2}, {&g.r1, &g.r2}, g.now);
      if (g.now % CONTROLLER_TICK_MS == 0) g.st = g.ctl.tick(g.s, lim, g.adc_v, g.now);
    }
  }
  CHECK(g.st.error == E_SOURCE_STALE && g.st.stale && g.st.source == 1);
  CHECK(g.st.state == State::CHARGING);  // a warning: charging carries on within the fallback cap
  CHECK(g.st.budget <= 4.01f);
}
```

- [ ] **Step 6: Run all host tests**

Run: `make -C tests test`
Expected: all suites `ok` (the new `test_oem_pylon` cases and `test_stale_source_e15_and_fields` included). Existing tests that build `Limits` go through `panel_limits`, so they need no change; if any constructs `Limits` directly, add `false, SourceMode::PANEL` to it.

- [ ] **Step 7: Commit**

```bash
git add oem/src/app/pylon_source.h oem/src/app/limit_source.h oem/src/app/controller.h tests/test_oem_pylon.cpp tests/test_oem_controller.cpp
git commit -m "OEM: Pylontech poller with freshness and the capped Panel fallback; E15 from a stale source"
```
(with the attribution lines)

---

### Task 3: Firmware integration — Source mode picks Modbus server or Pylontech poller (3.4)

**Files:**
- Modify: `oem/src/main.cpp` (RS485 block, controller tick, mode switching)
- Modify: `oem/src/app/modbus_map.h` (IR 4 bit 2, IR 5, `OEM_FW_VERSION`)
- Test: `tests/test_oem_modbus_map.cpp` (version check and IR 4 b2 / IR 5)

**Interfaces:**
- Consumes: `PylonSource` (Task 2), `Status::stale`/`source` (Task 2), `hw::rs485_*` (`oem/src/hw/rs485.h`), the existing Modbus block in `main.cpp` (around `if (modbus) { ... }`, roughly lines 200-262).
- Produces: firmware 3.4 in `oem/build/charger.dat`.

Behaviour to implement (spec §6):
- A file-scope `PylonSource g_pylon;` and a "line role" derived from settings: `const bool want_pylon = settings.get(HR_SOURCE_MODE) == int32_t(SourceMode::PYLONTECH);` (values 0 and 2 → Modbus server).
- Keep a `bool pylon_active` (initially `want_pylon` at boot, after `rs485_init`). Each pass, when the line is not busy (`!hw::rs485_tx_busy()`), if `want_pylon != pylon_active`:
  - to Pylontech: `set_line_baud(9600)` (Pylontech is always 9600), `g_mb_rx.discard_until(now)`, `g_pylon.reset()`, `pylon_active = true`;
  - to Modbus: `set_line_baud(modbus_baud(settings))`, set `mb_baud` and `g_mb_rx.set_gap_ms(...)` exactly as the existing baud-change code does, `g_mb_rx.discard_until(now)`, `pylon_active = false`.
  At boot, if HR 6 is Pylontech, init the line at 9600 instead of the Modbus baud (and `g_pylon.reset()`).
- Inside `if (modbus) { ... }` (the flag means "the line came up"; rename it `rs485` if that reads better, everywhere it's used): keep the TX-timeout abort for both roles. Then branch:
  - `pylon_active`: read with `hw::rs485_read` into a buffer and `g_pylon.push(buf, n, now)`; when not busy, `char req[pylon::REQUEST_LEN]; const size_t n = g_pylon.poll(now, req); if (n) hw::rs485_send(reinterpret_cast<const uint8_t *>(req), n)` (record `mb_tx_start = now` so the timeout guard covers it). No Modbus parsing, no commands, no Modbus baud changes, no `mb_reboot` (unreachable in this mode).
  - otherwise: the existing Modbus code unchanged.
- Controller tick: `const Limits lim = pylon_active ? g_pylon.limits(settings, now) : panel_limits(settings); st = g_ctl.tick(settings, lim, adc_v, now);` and drop the "Phase 2: the Panel source whatever Source mode says" comment for one that says Source mode now selects Panel or Pylontech (Modbus = Panel until built).
- The settings save gate (`!hw::rs485_tx_busy()`) is unchanged and applies to both roles.
- Update the file header comment (phase 3b: Pylontech source).

`modbus_map.h`:
- `OEM_FW_VERSION = 0x0304`.
- IR 4 bit 2 = `st.stale` (replace the `false` placeholder at bit 2 of the `bits[]` array).
- IR 5 = `st.source` (replace `case 5: v = 0;` and its comment).
- `tests/test_oem_modbus_map.cpp`: update the version checks (`0x0303` → `0x0304`, both the `ir(0)` check and the raw-response bytes `resp[3] == 0x03 && resp[4] == 0x04`), and add to `test_status_registers` (or a new test) that IR 5 reads 0 and IR 4 bit 2 is clear with Panel limits.

- [ ] **Step 1:** Make the `modbus_map.h` and test changes; run `make -C tests test` → all ok.
- [ ] **Step 2:** Make the `main.cpp` changes; build: `cmake --build oem/build 2>&1 | tail -5` → `wrote charger.dat: ...`, no warnings (the build uses `-Werror`-equivalent flags; fix any warning).
- [ ] **Step 3:** Run `make -C tests test` and `.venv/bin/python -m pytest -q` → all pass.
- [ ] **Step 4: Commit**

```bash
git add oem/src/main.cpp oem/src/app/modbus_map.h tests/test_oem_modbus_map.cpp
git commit -m "OEM 3.4: Source mode Pylontech polls the BMS on RS485 (Modbus server off); IR 5 / IR 4 b2 report it"
```
(with the attribution lines)

---

### Task 4: BMS simulator (`tools/fake_pylon.py`) and docs

**Files:**
- Create: `tools/fake_pylon.py`, `tools/test_fake_pylon.py`
- Modify: `README.md` (new subsection under "## OEM board firmware", after "### Modbus" and before "### Remote access through the WiFi bridge": "### Pylontech BMS source")

**Interfaces:**
- Consumes: `tools/oem_modbus.py`'s `open_port(serial, name, baud)`, `resolve_bridge(host)`, `bridge_url(host)`, `set_bridge_passthrough(host, on)`, `set_bridge_baud(host, baud)`, `PASSTHROUGH_SETTLE_S`.
- Produces: `fake_pylon.py` with pure helpers `length_field(lenid) -> str`, `checksum(text) -> str`, `frame(info_hex) -> str` (header `20024600`, as the user's proxy), `parse_request(text) -> int | None` (the CID2 of a valid `~20xx46xx0000cccc\r` request, else None), `reply_for(cid2, state) -> str | None`, a `State` dataclass (`voltage=53.4, current=0.0, soc=75, max_cell=3.345, cvl=54.6, dvl=44.0, ccl=30.0, dcl=100.0, charge_enable=True, alarm=False, silent=False`), and `main()`.

Encoding (mirror the user's YamBMS charger proxy, `packages/charger/charger_proxy_rs485.yaml`):
- 0x61 INFO = `%04X` V mV, `%04X` current ×100 (two's complement), `%02X` SOC, `%04X%04X` cycles ×2 (10), `%02X%02X` SOH ×2 (100), `%04X` max cell mV, `0101`, `%04X` min cell mV (= max cell − 15 mV), `0108`, then three groups of `avg_K max_K id min_K id` with K = int((°C + 273.15) × 10) for 27.5 / 30.0 / 25.0 °C and ids `0103`/`0109`, `0101`/`0101`, `0101`/`0101` — so that `State()` with current 3.0 A reproduces this plan's 0x61 test vector exactly.
- 0x62 INFO = `00` `00` then the protection bytes: `0200` when `alarm` else `0000`.
- 0x63 INFO = `%04X%04X%04X%04X%02X` of CVL mV, DVL mV, CCL ×10, DCL ×10, status (`0x80` if charge_enable, `| 0x40`).
- `silent` → no reply.

CLI:
- `fake_pylon.py (--port DEV | --bridge HOST)` plus initial values (`--voltage --soc --max-cell --cvl --ccl --no-charge-enable --alarm`).
- With `--bridge`: `resolve_bridge`, `set_bridge_passthrough(host, True)`, sleep `PASSTHROUGH_SETTLE_S`, `set_bridge_baud(host, 9600)`, open `bridge_url(host)`; turn Passthrough off in a `finally`.
- Main loop: read bytes, assemble `~…\r`, `parse_request`, reply with `reply_for` (write the reply immediately); print one line per request answered (`0x61 -> V=53.40 SOC=75` etc., quiet mode `-q` prints only changes).
- Live control from stdin (non-blocking, a reader thread is fine): lines like `set ccl 2`, `set cvl 53.8`, `set soc 90`, `enable off|on`, `alarm on|off`, `silent on|off`, `show`, `quit`.

Tests (`tools/test_fake_pylon.py`): `frame` reproduces the plan's 0x62/0x63 vectors from the corresponding states, `reply_for(0x61, State(current=3.0))` equals the plan's 0x61 vector, `parse_request` accepts the three stock requests (and 0x60) and rejects a bad checksum / non-request; `silent` returns None.

README "### Pylontech BMS source": how to switch (menu item 7 Source mode → Pylontech; back to Panel the same way; Modbus can't change it), that in this mode the RS485 jack is the BMS's (no Modbus, no HA, no `--modbus-reboot`: install updates with a power-cycle), the dialect (stock's 0x61-0x63 at 9600, as Eco-Worthy JBD batteries and the YamBMS charger proxy speak), what is honoured (CVL, CCL, charge enable, alarm → E12, SOC thresholds, max cell, battery V) and not (0x61 current), the stale rule (3 s → E15, Panel limits capped at Fallback current; set it to 0 to stop charging without the BMS), and the simulator usage (`fake_pylon.py --bridge eco-charger-a.local --soc 60`).

- [ ] **Step 1:** Write `tools/test_fake_pylon.py` with the tests above; run `.venv/bin/python -m pytest -q tools/test_fake_pylon.py` → fails (module missing).
- [ ] **Step 2:** Implement `tools/fake_pylon.py`; run the full Python suite → all pass.
- [ ] **Step 3:** README section.
- [ ] **Step 4: Commit**

```bash
git add tools/fake_pylon.py tools/test_fake_pylon.py README.md
git commit -m "tools: fake_pylon.py BMS simulator (stock Pylontech dialect, through the bridge); README Pylontech source"
```
(with the attribution lines)

---

### Live verification (controller, after the final review)

Needs the user at one charger's panel. Through that charger's bridge:
1. Install 3.4 with `oem_update.py --bridge eco-charger-X.local --modbus-reboot oem/build/charger.dat` (Panel mode, as today); check `status` shows 3.4.
2. Start `fake_pylon.py --bridge eco-charger-X.local --soc 60 --ccl 3 --cvl 54.6`; the user sets Source mode → Pylontech and unpauses / charge-now on the panel.
3. Cases, watched with the clamp/shunt and the panel: charges ≤ 3 A; `set ccl 1.5` → current follows; `set cvl 53.8` → target follows; `enable off` → stops (not allowed); `alarm on` → E12, stops; `silent on` → E15 after ~3 s, fallback current; `silent off` → back to fresh.
4. User sets Source mode → Panel; bridge/HA return (`status` works).
