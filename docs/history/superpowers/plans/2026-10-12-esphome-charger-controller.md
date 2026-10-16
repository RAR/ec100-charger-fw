# ESPHome Charger Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Eco-Worthy charger's stock control board with a Waveshare ESP32-S3-RS485-CAN running ESPHome. It drives two NASN RM4850-B rectifiers over CAN, using YamBMS charge limits from MQTT and Home Assistant overrides.

**Architecture:** There are two ESPHome external components.
- `nasn_rectifier` (one instance per PSU address) owns all CAN traffic for one module: polling, setpoints and keepalive.
- `charge_controller` subscribes to YamBMS over MQTT and runs the state machine. Each second it hands every rectifier a target voltage and current and an on/standby flag.

All decision logic and frame encoding live in two headers with no ESPHome dependency (`nasn_protocol.h`, `charge_logic.h`), unit-tested on the Mac. The ESPHome glue stays thin. A Python fake (`tools/fake_nasn.py`) on the CANable emulates both PSUs for bench tests.

**Tech Stack:** ESPHome 2026.8.1 (ESP-IDF, ESP32-S3), C++17, host clang for unit tests, Python 3 with python-can and gs_usb, plus pytest for the bench fake.

**Spec:** `docs/history/superpowers/specs/2026-10-12-esphome-charger-controller-design.md`

## Global Constraints

- Board: Waveshare ESP32-S3-RS485-CAN, `esp32-s3-devkitc-1`, `flash_size: 16MB`, `framework: esp-idf`. CAN TX is `GPIO15` and RX is `GPIO16` (confirmed in Task 1).
- CAN: 125 kbit/s, 29-bit extended IDs. Read = `0x108082FE | addr<<16` (cmd 0x82). Write = `0x108081FE | addr<<16` (cmd 0x81). A reply has the request ID with bit 7 cleared.
- In a reply, the status is the **high** nibble of byte 0 (0 = ok), the register is `(byte0 & 0x0F)<<8 | byte1`, and bytes 2–7 are a 48-bit big-endian value.
- Setpoint encoding: voltage raw = V × 1023 (reg 0x0100). Current raw = A × 1023 / 52 (reg 0x022D). Standby: reg 0x0132, `data[3]` = 0 for on, 1 for standby. Telemetry readings are raw / 1024.
- Hard caps that nothing can override: 58.0 V, and 50 A per PSU. The minimum voltage setpoint is 42.0 V.
- Fallback stored defaults: 54.0 V / 5 A per PSU.
- MQTT broker: `homeassistant.local`, credentials go in `esphome/secrets.yaml` only, which is gitignored. Never commit them.
- MQTT topics:
  - `yambms/status`
  - `yambms/yambms_1/sensor/requested_charge_voltage/state`
  - `yambms/yambms_1/sensor/requested_charge_current/state`
  - `yambms/yambms_1/binary_sensor/charging_allowed/state`
  - `yambms/yambms_1/sensor/bms_in_alarm/state`
  - `yambms/yambms_1/sensor/current/state` (positive = charging)
- Never run `fake_psu.py` or `fake_nasn.py` on a bus with real PSUs attached.
- Run Python tools with the repo venv: `.venv/bin/python`. Run ESPHome as `esphome` (pipx, on PATH).
- Commit messages end with `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`.

## File Structure

```
esphome/
  eco-charger.yaml                          device config (grows over Tasks 1, 7, 8)
  secrets.yaml                              gitignored: wifi_ssid, wifi_password, api_key, ota_password, mqtt_username, mqtt_password
  components/nasn_rectifier/
    __init__.py                             config schema + codegen (sensors inline)
    nasn_protocol.h                         pure frame encode/decode (host-tested)
    nasn_rectifier.h / .cpp                 PollingComponent: CAN I/O for one PSU
  components/charge_controller/
    __init__.py                             config schema + codegen
    charge_logic.h                          pure: BmsTracker, setpoint math, StateMachine (host-tested)
    charge_controller.h / .cpp              Component: MQTT subscribe, 1 Hz tick, entity publishing
tests/
  Makefile, check.h                         tiny assert harness, no dependencies
  test_nasn_protocol.cpp, test_charge_logic.cpp
tools/
  fake_nasn.py, test_fake_nasn.py           NASN emulator for the bench + its pytest
  psu_ctl.py (existing; status-nibble fix already applied in the working tree)
docs/history/notes/findings.md                           Task 9 appends the real-hardware results
```

---

### Task 1: Bench bring-up and CAN pin check

Proves that the toolchain, the board, the GPIO15/16 mapping and the CANable wiring all work before any real code exists.

**Files:**
- Create: `esphome/eco-charger.yaml`, `esphome/secrets.yaml` (gitignored)
- Commit (existing, untracked): `tools/`, `docs/history/notes/`

**Interfaces:**
- Produces: `esphome/eco-charger.yaml` with `canbus` id `can`. Later tasks append to this file.

- [ ] **Step 1: Commit the existing tools and notes.** `psu_ctl.py` already contains the status-nibble fix. The `dumps/`, `re/` and `captures/` directories stay untracked: the user decides about those.

```bash
cd /Users/rar/eco-charger
git add tools/*.py docs/history/notes/findings.md
git commit -m "Add CAN bench tools and firmware findings

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

- [ ] **Step 2: Create `esphome/secrets.yaml`.** Ask the user for the WiFi SSID and password. Generate the API key with `openssl rand -base64 32`. The MQTT values come from the user (the YamBMS broker login).

```yaml
wifi_ssid: "<ask user>"
wifi_password: "<ask user>"
api_key: "<openssl rand -base64 32>"
ota_password: "<openssl rand -hex 16>"
mqtt_username: "<ask user>"
mqtt_password: "<ask user>"
```

Check that it is ignored: `git check-ignore esphome/secrets.yaml` must print the path.

- [ ] **Step 3: Write the pin-check config `esphome/eco-charger.yaml`.**

```yaml
esphome:
  name: eco-charger
  friendly_name: Eco Charger

esp32:
  board: esp32-s3-devkitc-1
  flash_size: 16MB
  framework:
    type: esp-idf

logger:
  level: DEBUG

api:
  encryption:
    key: !secret api_key

ota:
  - platform: esphome
    password: !secret ota_password

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  ap: {}

canbus:
  - platform: esp32_can
    id: can
    tx_pin: GPIO15
    rx_pin: GPIO16
    can_id: 0
    use_extended_id: true
    bit_rate: 125kbps
    rx_queue_len: 32
    tx_queue_len: 16
    on_frame:
      - can_id: 0
        can_id_mask: 0
        use_extended_id: true
        then:
          - lambda: |-
              ESP_LOGI("pincheck", "RX %08X len %u", can_id, (unsigned) x.size());

interval:
  - interval: 1s
    then:
      - canbus.send:
          canbus_id: can
          can_id: 0x1FFFFFF0
          use_extended_id: true
          data: [0xDE, 0xAD, 0xBE, 0xEF]
```

- [ ] **Step 4: Validate the config.**

Run: `cd esphome && esphome config eco-charger.yaml > /dev/null && echo OK`
Expected: `OK`

- [ ] **Step 5: Flash over USB** (the first flash takes several minutes).

Run: `cd esphome && esphome run eco-charger.yaml --device /dev/cu.usbmodem1201 --no-logs`
Expected: ends with `Successfully uploaded program.` If the port has changed, re-list it with `ls /dev/cu.usbmodem*`. If the upload can't connect, hold BOOT, tap RESET, and retry.

- [ ] **Step 6: ESP32 → CANable.** Fit the Waveshare 120 Ω jumper. Run in the background:

Run: `.venv/bin/python tools/sniff.py 10`
Expected: about 10 lines of `1FFFFFF0#DE AD BE EF`. If there are 0 frames, swap CAN-H/L at one end and retry. If it's still 0, the pins are wrong: check the Waveshare schematic for the TWAI pins before going further.

- [ ] **Step 7: CANable → ESP32.** Run `esphome logs eco-charger.yaml --device /dev/cu.usbmodem1201` in the background, and send frames from the Mac:

Run: `.venv/bin/python tools/psu_ctl.py read --addr 1` (every register shows `no reply`, which is fine)
Expected: the ESP log shows `[pincheck] RX 108182FE len 8` lines.

- [ ] **Step 8: Commit.**

```bash
git add esphome/eco-charger.yaml
git commit -m "ESPHome bench config: CAN pin check on Waveshare ESP32-S3 (GPIO15/16)

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Host test harness and `nasn_protocol.h`

**Files:**
- Create: `tests/check.h`, `tests/Makefile`, `tests/test_nasn_protocol.cpp`
- Create: `esphome/components/nasn_rectifier/nasn_protocol.h`
- Modify: `.gitignore` (add the test binaries)

**Interfaces:**
- Produces (namespace `nasn`):
  - constants `READ_BASE`, `WRITE_BASE`, `PROTO_SMU`, `CMD_WRITE`, `CMD_READ`, `SETPOINT_SCALE`, `TELEMETRY_SCALE`, `NOMINAL_A`, `MIN_V`, `MAX_V`, `MAX_A_PER_PSU`
  - `nasn::reg::{VOUT_SET, VOUT_DEFAULT, ILIMIT_DEFAULT, STANDBY, INPUT_POWER, INPUT_FREQ, INPUT_CURRENT, OUTPUT_POWER, VOUT, INPUT_VOLTAGE, TEMP_OUT, TEMP_IN, IOUT, ALARMS, ILIMIT}`
  - `using Frame = std::array<uint8_t, 8>`
  - `uint32_t read_id(uint8_t addr)`, `uint32_t write_id(uint8_t addr)`
  - `Frame read_request(uint16_t reg)`, `Frame write_u16(uint16_t reg, uint16_t raw)`
  - `uint16_t voltage_raw(float)`, `uint16_t current_raw(float)`
  - `Frame write_voltage(uint16_t reg, float volts)`, `Frame write_current(uint16_t reg, float amps)`, `Frame write_standby(bool)`
  - `struct Reply { uint8_t addr, cmd, status; uint16_t reg; uint64_t value; }`
  - `std::optional<Reply> parse_reply(uint32_t id, bool extended, const uint8_t *data, size_t len)`
  - `float telemetry(uint64_t)`, `float setpoint_volts(uint64_t)`, `float setpoint_amps(uint64_t)`

- [ ] **Step 1: Write the harness `tests/check.h`.**

```cpp
#pragma once
// Minimal test harness: CHECK / CHECK_NEAR record failures, main() returns test_result().
#include <cmath>
#include <cstdio>

inline int g_failures = 0;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::printf("%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);   \
      g_failures++;                                                          \
    }                                                                        \
  } while (0)

#define CHECK_NEAR(a, b, eps)                                                          \
  do {                                                                                 \
    double a_ = (a), b_ = (b);                                                         \
    if (!(std::fabs(a_ - b_) <= (eps))) {                                              \
      std::printf("%s:%d: %s = %g, expected %g\n", __FILE__, __LINE__, #a, a_, b_);   \
      g_failures++;                                                                    \
    }                                                                                  \
  } while (0)

inline int test_result(const char *name) {
  std::printf("%s: %s\n", name, g_failures ? "FAILED" : "ok");
  return g_failures ? 1 : 0;
}
```

- [ ] **Step 2: Write `tests/Makefile`.**

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components
TESTS = test_nasn_protocol test_charge_logic
HEADERS = check.h $(wildcard ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

Append to `.gitignore`:

```
tests/test_nasn_protocol
tests/test_charge_logic
```

- [ ] **Step 3: Write the failing test `tests/test_nasn_protocol.cpp`.**

```cpp
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

int main() {
  test_ids();
  test_requests();
  test_clamps();
  test_parse_reply();
  test_parse_rejects();
  test_setpoint_readback();
  return test_result("nasn_protocol");
}
```

Also create a placeholder `tests/test_charge_logic.cpp` so `make` has both targets (Task 3 replaces it):

```cpp
#include "check.h"
int main() { return test_result("charge_logic"); }
```

- [ ] **Step 4: Run it and confirm it fails.**

Run: `make -C tests`
Expected: a compile error, `'nasn_rectifier/nasn_protocol.h' file not found`

- [ ] **Step 5: Implement `esphome/components/nasn_rectifier/nasn_protocol.h`.**

```cpp
#pragma once
// NASN RM4850-B CAN framing, as used by the stock Eco-Worthy controller firmware (see docs/history/notes/findings.md).
// Pure C++17 with no ESPHome dependencies, so it is unit-tested on the host (tests/test_nasn_protocol.cpp).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace nasn {

constexpr uint32_t READ_BASE = 0x108082FE;   // cmd 0x82 register get
constexpr uint32_t WRITE_BASE = 0x108081FE;  // cmd 0x81 register set (Huawei docs use 0x80)
constexpr uint8_t PROTO_SMU = 0x21;
constexpr uint8_t CMD_WRITE = 0x81;
constexpr uint8_t CMD_READ = 0x82;
constexpr float SETPOINT_SCALE = 1023.0f;   // stock firmware scales setpoints by 1023
constexpr float TELEMETRY_SCALE = 1024.0f;  // Huawei scale for readings (confirm in Task 9)
constexpr float NOMINAL_A = 52.0f;
constexpr float MIN_V = 42.0f;
constexpr float MAX_V = 58.0f;          // hard cap, never exceeded
constexpr float MAX_A_PER_PSU = 50.0f;  // hard cap per module

namespace reg {
constexpr uint16_t VOUT_SET = 0x0100;
constexpr uint16_t VOUT_DEFAULT = 0x0101;
constexpr uint16_t ILIMIT_DEFAULT = 0x0104;
constexpr uint16_t STANDBY = 0x0132;
constexpr uint16_t INPUT_POWER = 0x0170;
constexpr uint16_t INPUT_FREQ = 0x0171;
constexpr uint16_t INPUT_CURRENT = 0x0172;
constexpr uint16_t OUTPUT_POWER = 0x0173;
constexpr uint16_t VOUT = 0x0175;
constexpr uint16_t INPUT_VOLTAGE = 0x0178;
constexpr uint16_t TEMP_OUT = 0x017F;
constexpr uint16_t TEMP_IN = 0x0180;
constexpr uint16_t IOUT = 0x0182;
constexpr uint16_t ALARMS = 0x0183;
constexpr uint16_t ILIMIT = 0x022D;
}  // namespace reg

using Frame = std::array<uint8_t, 8>;

inline uint32_t read_id(uint8_t addr) { return READ_BASE | (uint32_t(addr & 0x7F) << 16); }
inline uint32_t write_id(uint8_t addr) { return WRITE_BASE | (uint32_t(addr & 0x7F) << 16); }

inline Frame read_request(uint16_t r) { return {uint8_t(r >> 8), uint8_t(r & 0xFF), 0, 0, 0, 0, 0, 0}; }

inline Frame write_u16(uint16_t r, uint16_t raw) {
  return {uint8_t(r >> 8), uint8_t(r & 0xFF), 0, 0, 0, 0, uint8_t(raw >> 8), uint8_t(raw & 0xFF)};
}

inline uint16_t voltage_raw(float volts) {
  if (!std::isfinite(volts)) volts = MIN_V;
  return uint16_t(std::lround(std::clamp(volts, MIN_V, MAX_V) * SETPOINT_SCALE));
}

inline uint16_t current_raw(float amps) {
  if (!std::isfinite(amps)) amps = 0.0f;
  return uint16_t(std::lround(std::clamp(amps, 0.0f, MAX_A_PER_PSU) * SETPOINT_SCALE / NOMINAL_A));
}

inline Frame write_voltage(uint16_t r, float volts) { return write_u16(r, voltage_raw(volts)); }
inline Frame write_current(uint16_t r, float amps) { return write_u16(r, current_raw(amps)); }

inline Frame write_standby(bool standby) {
  return {uint8_t(reg::STANDBY >> 8), uint8_t(reg::STANDBY & 0xFF), 0, uint8_t(standby ? 1 : 0), 0, 0, 0, 0};
}

struct Reply {
  uint8_t addr;
  uint8_t cmd;     // CMD_READ or CMD_WRITE
  uint8_t status;  // 0 = ok
  uint16_t reg;
  uint64_t value;  // 48-bit big-endian payload
};

inline std::optional<Reply> parse_reply(uint32_t id, bool extended, const uint8_t *data, size_t len) {
  if (!extended || len != 8) return std::nullopt;
  if (((id >> 23) & 0x3F) != PROTO_SMU || (id & 0x80)) return std::nullopt;  // bit 7 set = sent by a controller
  uint8_t cmd = (id >> 8) & 0xFF;
  if (cmd != CMD_READ && cmd != CMD_WRITE) return std::nullopt;
  Reply r{};
  r.addr = (id >> 16) & 0x7F;
  r.cmd = cmd;
  r.status = data[0] >> 4;
  r.reg = uint16_t((data[0] & 0x0F) << 8 | data[1]);
  for (size_t i = 2; i < 8; i++) r.value = (r.value << 8) | data[i];
  return r;
}

inline float telemetry(uint64_t raw) { return float(raw) / TELEMETRY_SCALE; }
inline float setpoint_volts(uint64_t raw) { return float(raw) / SETPOINT_SCALE; }
inline float setpoint_amps(uint64_t raw) { return float(raw) * NOMINAL_A / SETPOINT_SCALE; }

}  // namespace nasn
```

- [ ] **Step 6: Run the tests and confirm they pass.**

Run: `make -C tests`
Expected: `nasn_protocol: ok` and `charge_logic: ok`

- [ ] **Step 7: Commit.**

```bash
git add tests/ esphome/components/nasn_rectifier/nasn_protocol.h .gitignore
git commit -m "nasn_protocol.h: host-tested NASN CAN frame encoding

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: `charge_logic.h`: BMS input tracking

**Files:**
- Create: `esphome/components/charge_controller/charge_logic.h`
- Modify: `tests/test_charge_logic.cpp` (replaces the placeholder)

**Interfaces:**
- Produces (namespace `charge`):
  - `constexpr uint32_t FRESH_MS = 60000`
  - `enum class Topic : uint8_t { STATUS, CVL, CCL, ALLOWED, ALARM, CURRENT }`, `constexpr int TOPIC_COUNT = 6`
  - `class BmsTracker` with:
    - `void on_reconnect()`
    - `void on_message(Topic, const std::string &payload, uint32_t now_ms)`
    - `bool fresh(uint32_t now_ms) const`
    - `uint32_t age_ms(uint32_t now_ms) const`
    - `bool online() const`, `bool allowed() const`, `bool alarm() const`
    - `float cvl() const`, `float ccl() const`, `float current() const`

- [ ] **Step 1: Write the failing tests.** Replace `tests/test_charge_logic.cpp` with:

```cpp
#include "check.h"
#include "charge_controller/charge_logic.h"

using charge::Topic;

// Feeds one full YamBMS snapshot at time t (each topic's first message counts as retained).
static void feed_all(charge::BmsTracker &b, uint32_t t) {
  b.on_message(Topic::STATUS, "online", t);
  b.on_message(Topic::CVL, "57.0", t);
  b.on_message(Topic::CCL, "275", t);
  b.on_message(Topic::ALLOWED, "ON", t);
  b.on_message(Topic::ALARM, "0", t);
  b.on_message(Topic::CURRENT, "237.8", t);
}

static void test_retained_only_is_not_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 1000);  // retained replay on subscribe
  CHECK(!b.fresh(1000));
  CHECK(b.online());
  CHECK(b.allowed());
  CHECK_NEAR(b.cvl(), 57.0, 1e-6);
}

static void test_second_message_is_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 1000);
  b.on_message(Topic::CVL, "57.0", 10000);
  b.on_message(Topic::CCL, "275", 10000);
  CHECK(!b.fresh(10000));  // current not yet updated live
  b.on_message(Topic::CURRENT, "238.0", 11000);
  CHECK(b.fresh(11000));
  CHECK_NEAR(b.current(), 238.0, 1e-6);
  CHECK(b.age_ms(11000) == 1000);  // CVL/CCL last updated at 10000
}

static void test_goes_stale_after_60s() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  CHECK(b.fresh(61000));
  CHECK(!b.fresh(61001));
}

static void test_reconnect_requires_live_update_again() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  CHECK(b.fresh(2000));
  b.on_reconnect();
  feed_all(b, 3000);    // retained replay after reconnect: values stored...
  CHECK(b.fresh(3000)); // ...and the earlier live updates (t=1000) still count until they age out
  CHECK(!b.fresh(61001));
  feed_all(b, 62000);
  CHECK(b.fresh(62000));
}

static void test_status_and_flags() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  b.on_message(Topic::STATUS, "offline", 2000);
  CHECK(!b.online());
  CHECK(!b.fresh(2000));
  b.on_message(Topic::STATUS, "online", 3000);
  b.on_message(Topic::ALLOWED, "OFF", 3000);
  CHECK(!b.allowed());
  CHECK(b.fresh(3000));  // freshness doesn't depend on the allowed value
  b.on_message(Topic::ALARM, "1", 3000);
  CHECK(b.alarm());
}

static void test_garbage_is_ignored() {
  charge::BmsTracker b;
  b.on_reconnect();
  feed_all(b, 0);
  feed_all(b, 1000);
  b.on_message(Topic::CVL, "unavailable", 2000);
  b.on_message(Topic::CVL, "nan", 2000);
  b.on_message(Topic::CVL, "", 2000);
  b.on_message(Topic::ALLOWED, "maybe", 2000);
  CHECK_NEAR(b.cvl(), 57.0, 1e-6);
  CHECK(b.allowed());
  CHECK(!b.fresh(61001));  // ignored payloads don't refresh
}

static void test_missing_flags_not_fresh() {
  charge::BmsTracker b;
  b.on_reconnect();
  for (uint32_t t : {0u, 1000u}) {
    b.on_message(Topic::STATUS, "online", t);
    b.on_message(Topic::CVL, "57", t);
    b.on_message(Topic::CCL, "275", t);
    b.on_message(Topic::CURRENT, "0", t);
  }
  CHECK(!b.fresh(1000));  // no charging_allowed / bms_in_alarm value yet
  CHECK(b.age_ms(1000) == 0);
  charge::BmsTracker never;
  CHECK(never.age_ms(5000) == UINT32_MAX);
}

int main() {
  test_retained_only_is_not_fresh();
  test_second_message_is_fresh();
  test_goes_stale_after_60s();
  test_reconnect_requires_live_update_again();
  test_status_and_flags();
  test_garbage_is_ignored();
  test_missing_flags_not_fresh();
  return test_result("charge_logic");
}
```

- [ ] **Step 2: Run it and confirm it fails.**

Run: `make -C tests`
Expected: a compile error, `'charge_controller/charge_logic.h' file not found`

- [ ] **Step 3: Implement `esphome/components/charge_controller/charge_logic.h`.**

```cpp
#pragma once
// Charge-control decisions for the ESPHome charger controller (see the design spec).
// Pure C++17 with no ESPHome dependencies, so it is unit-tested on the host (tests/test_charge_logic.cpp).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

namespace charge {

constexpr uint32_t FRESH_MS = 60000;

// ---- BMS inputs (YamBMS over MQTT) ----

enum class Topic : uint8_t { STATUS, CVL, CCL, ALLOWED, ALARM, CURRENT };
constexpr int TOPIC_COUNT = 6;

// Tracks the latest YamBMS values and whether the periodic ones are live. ESPHome's MQTT callback
// doesn't expose the retain flag, so the first message per topic after each (re)connect is treated
// as a possibly-stale retained replay: its value is stored but it doesn't count as an update.
class BmsTracker {
 public:
  void on_reconnect() {
    for (auto &t : topics_) t.seen_since_connect = false;
  }

  void on_message(Topic topic, const std::string &payload, uint32_t now_ms) {
    float v = 0.0f;
    switch (topic) {
      case Topic::STATUS:
        v = payload == "online" ? 1.0f : 0.0f;
        break;
      case Topic::ALLOWED:
        if (payload == "ON") {
          v = 1.0f;
        } else if (payload == "OFF") {
          v = 0.0f;
        } else {
          return;
        }
        break;
      default:
        if (!parse_float_(payload, &v)) return;
    }
    Entry &t = topics_[int(topic)];
    t.value = v;
    t.has_value = true;
    if (t.seen_since_connect) {
      t.last_update_ms = now_ms;
      t.has_update = true;
    }
    t.seen_since_connect = true;
  }

  bool fresh(uint32_t now_ms) const {
    return age_ms(now_ms) <= FRESH_MS && online() && has_(Topic::ALLOWED) && has_(Topic::ALARM);
  }

  // Age of the stalest periodic topic (CVL, CCL, current), or UINT32_MAX if one has never updated live.
  uint32_t age_ms(uint32_t now_ms) const {
    uint32_t worst = 0;
    for (Topic p : {Topic::CVL, Topic::CCL, Topic::CURRENT}) {
      const Entry &t = topics_[int(p)];
      if (!t.has_update) return UINT32_MAX;
      worst = std::max(worst, now_ms - t.last_update_ms);
    }
    return worst;
  }

  bool online() const { return has_(Topic::STATUS) && value_(Topic::STATUS) == 1.0f; }
  bool allowed() const { return has_(Topic::ALLOWED) && value_(Topic::ALLOWED) == 1.0f; }
  bool alarm() const { return has_(Topic::ALARM) && value_(Topic::ALARM) != 0.0f; }
  float cvl() const { return value_(Topic::CVL); }
  float ccl() const { return value_(Topic::CCL); }
  float current() const { return value_(Topic::CURRENT); }

 private:
  struct Entry {
    float value = NAN;
    bool has_value = false;
    bool seen_since_connect = false;
    bool has_update = false;
    uint32_t last_update_ms = 0;
  };

  static bool parse_float_(const std::string &s, float *out) {
    char *end = nullptr;
    float v = std::strtof(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
  }

  bool has_(Topic t) const { return topics_[int(t)].has_value; }
  float value_(Topic t) const { return topics_[int(t)].value; }

  Entry topics_[TOPIC_COUNT];
};

}  // namespace charge
```

- [ ] **Step 4: Run the tests and confirm they pass.**

Run: `make -C tests`
Expected: `nasn_protocol: ok` and `charge_logic: ok`

- [ ] **Step 5: Commit.**

```bash
git add tests/test_charge_logic.cpp esphome/components/charge_controller/charge_logic.h
git commit -m "charge_logic.h: YamBMS freshness tracking with retained-replay handling

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: `charge_logic.h`: setpoint math

**Files:**
- Modify: `esphome/components/charge_controller/charge_logic.h` (append before the closing `}  // namespace charge`)
- Modify: `tests/test_charge_logic.cpp`

**Interfaces:**
- Consumes: the `charge` namespace from Task 3.
- Produces:
  - constants `HARD_MAX_V = 58.0f`, `MAX_A_PER_PSU = 50.0f`, `OVERVOLTAGE_MARGIN_V = 0.5f`, `RAMP_A_PER_S = 2.0f`, `AVERAGE_MS = 10000`
  - `float target_voltage(float cvl, float user_max_v)`: NAN if the inputs aren't finite
  - `float current_budget(float ccl, float battery_a, float our_a, float user_max_a)`: 0 if any input isn't finite
  - `class MovingAverage { explicit MovingAverage(uint32_t window_ms = AVERAGE_MS); void add(float, uint32_t now_ms); float value() const; void reset(); }`: `value()` is NAN when empty
  - `class RampLimiter { float step(float target, uint32_t now_ms); void reset(); float output() const; }`
  - `float per_psu_current(float total, int healthy)`
  - `bool overvoltage(bool charging, float measured_v, float target_v, float our_a)`: above the hard cap is always a fault. Above target + 0.5 V is a fault only while charging **and** sourcing more than 1 A, because when CVL drops to float the output terminal just reads the still-high battery.

- [ ] **Step 1: Add the failing tests.** Insert these functions above `main()` in `tests/test_charge_logic.cpp`, and add the calls to `main()` before `return`:

```cpp
static void test_target_voltage() {
  CHECK_NEAR(charge::target_voltage(57.0f, 57.0f), 57.0, 1e-6);
  CHECK_NEAR(charge::target_voltage(57.0f, 55.5f), 55.5, 1e-6);
  CHECK_NEAR(charge::target_voltage(60.0f, 60.0f), 58.0, 1e-6);  // hard cap
  CHECK(std::isnan(charge::target_voltage(NAN, 57.0f)));
}

static void test_current_budget() {
  // Inverter pushes 238 A, CCL 275, we push nothing: 37 A headroom.
  CHECK_NEAR(charge::current_budget(275, 238, 0, 100), 37, 1e-4);
  // We're now pushing 37 A, so the battery sees 275: the budget stays at 37 (no oscillation).
  CHECK_NEAR(charge::current_budget(275, 275, 37, 100), 37, 1e-4);
  // Loads draining the battery: the budget is capped by the user max.
  CHECK_NEAR(charge::current_budget(275, -50, 0, 100), 100, 1e-4);
  // Over the CCL: no budget, never negative.
  CHECK_NEAR(charge::current_budget(100, 150, 0, 100), 0, 1e-6);
  // Missing data is never treated as unlimited.
  CHECK_NEAR(charge::current_budget(275, NAN, 0, 100), 0, 1e-6);
  CHECK_NEAR(charge::current_budget(NAN, 0, 0, 100), 0, 1e-6);
}

static void test_moving_average() {
  charge::MovingAverage avg(10000);
  CHECK(std::isnan(avg.value()));
  avg.add(10, 0);
  avg.add(20, 5000);
  CHECK_NEAR(avg.value(), 15, 1e-6);
  avg.add(30, 12000);  // the t=0 sample drops out of the 10 s window
  CHECK_NEAR(avg.value(), 25, 1e-6);
  avg.add(NAN, 13000);  // ignored
  CHECK_NEAR(avg.value(), 25, 1e-6);
  avg.reset();
  CHECK(std::isnan(avg.value()));
}

static void test_ramp() {
  charge::RampLimiter r;
  CHECK_NEAR(r.step(10, 0), 0, 1e-6);
  CHECK_NEAR(r.step(10, 1000), 2, 1e-6);
  CHECK_NEAR(r.step(10, 2000), 4, 1e-6);
  CHECK_NEAR(r.step(1, 2500), 1, 1e-6);   // drops immediately
  CHECK_NEAR(r.step(10, 3500), 3, 1e-6);
  r.reset();
  CHECK_NEAR(r.step(10, 60000), 0, 1e-6); // reset restarts from 0 without a time jump
  CHECK_NEAR(r.step(10, 60500), 1, 1e-6);
}

static void test_split_and_overvoltage() {
  CHECK_NEAR(charge::per_psu_current(37, 2), 18.5, 1e-6);
  CHECK_NEAR(charge::per_psu_current(150, 2), 50, 1e-6);
  CHECK_NEAR(charge::per_psu_current(37, 0), 0, 1e-6);
  CHECK_NEAR(charge::per_psu_current(-3, 2), 0, 1e-6);
  CHECK(!charge::overvoltage(true, 57.4f, 57.0f, 10.0f));
  CHECK(charge::overvoltage(true, 57.6f, 57.0f, 10.0f));
  CHECK(!charge::overvoltage(true, 57.6f, 54.0f, 0.3f));   // CVL just dropped to float: battery still high, we push ~0 A
  CHECK(charge::overvoltage(true, 57.6f, 54.0f, 5.0f));    // ...but pushing current above target is a fault
  CHECK(!charge::overvoltage(false, 57.6f, 57.0f, 0.0f));  // in standby the reading is just the battery
  CHECK(charge::overvoltage(false, 58.6f, 57.0f, 0.0f));   // above the hard cap is always a fault
  CHECK(!charge::overvoltage(true, NAN, 57.0f, 10.0f));
}
```

Add to `main()`:

```cpp
  test_target_voltage();
  test_current_budget();
  test_moving_average();
  test_ramp();
  test_split_and_overvoltage();
```

- [ ] **Step 2: Run it and confirm it fails.**

Run: `make -C tests`
Expected: compile errors, `no member named 'target_voltage' in namespace 'charge'` (and similar)

- [ ] **Step 3: Implement.** Add `#include <deque>` and `#include <utility>` to the includes. Then append before `}  // namespace charge`:

```cpp
// ---- Setpoints ----

constexpr float HARD_MAX_V = 58.0f;
constexpr float MAX_A_PER_PSU = 50.0f;
constexpr float OVERVOLTAGE_MARGIN_V = 0.5f;
constexpr float OVERVOLTAGE_MIN_A = 1.0f;  // only a fault while we're actually pushing current
constexpr float RAMP_A_PER_S = 2.0f;
constexpr uint32_t AVERAGE_MS = 10000;

inline float target_voltage(float cvl, float user_max_v) {
  if (!std::isfinite(cvl) || !std::isfinite(user_max_v)) return NAN;
  return std::min({cvl, user_max_v, HARD_MAX_V});
}

// Current we may add: CCL minus what everything else (e.g. the inverter) already pushes into the battery.
inline float current_budget(float ccl, float battery_a, float our_a, float user_max_a) {
  if (!std::isfinite(ccl) || !std::isfinite(battery_a) || !std::isfinite(our_a) || !std::isfinite(user_max_a))
    return 0.0f;
  float others = battery_a - our_a;
  return std::max(0.0f, std::min(user_max_a, ccl - others));
}

// Mean of the samples added within the last window_ms.
class MovingAverage {
 public:
  explicit MovingAverage(uint32_t window_ms = AVERAGE_MS) : window_ms_(window_ms) {}

  void add(float v, uint32_t now_ms) {
    if (!std::isfinite(v)) return;
    samples_.emplace_back(now_ms, v);
    while (!samples_.empty() && now_ms - samples_.front().first > window_ms_) samples_.pop_front();
  }

  float value() const {
    if (samples_.empty()) return NAN;
    float sum = 0.0f;
    for (const auto &s : samples_) sum += s.second;
    return sum / float(samples_.size());
  }

  void reset() { samples_.clear(); }

 private:
  uint32_t window_ms_;
  std::deque<std::pair<uint32_t, float>> samples_;
};

// Rises at most RAMP_A_PER_S, falls immediately.
class RampLimiter {
 public:
  float step(float target, uint32_t now_ms) {
    if (!started_) {
      started_ = true;
      last_ms_ = now_ms;
    }
    float dt = float(now_ms - last_ms_) / 1000.0f;
    last_ms_ = now_ms;
    output_ = std::max(0.0f, std::min(target, output_ + RAMP_A_PER_S * dt));
    return output_;
  }

  void reset() {
    output_ = 0.0f;
    started_ = false;
  }

  float output() const { return output_; }

 private:
  float output_ = 0.0f;
  uint32_t last_ms_ = 0;
  bool started_ = false;
};

inline float per_psu_current(float total, int healthy) {
  if (healthy <= 0 || !(total > 0.0f)) return 0.0f;
  return std::min(total / float(healthy), MAX_A_PER_PSU);
}

inline bool overvoltage(bool charging, float measured_v, float target_v, float our_a) {
  if (!std::isfinite(measured_v)) return false;
  if (measured_v > HARD_MAX_V + OVERVOLTAGE_MARGIN_V) return true;
  return charging && std::isfinite(target_v) && measured_v > target_v + OVERVOLTAGE_MARGIN_V &&
         our_a > OVERVOLTAGE_MIN_A;
}
```

- [ ] **Step 4: Run the tests and confirm they pass.**

Run: `make -C tests`
Expected: `nasn_protocol: ok` and `charge_logic: ok`

- [ ] **Step 5: Commit.**

```bash
git add tests/test_charge_logic.cpp esphome/components/charge_controller/charge_logic.h
git commit -m "charge_logic.h: voltage target, CCL budget, averaging, ramp, split

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: `charge_logic.h`: state machine

**Files:**
- Modify: `esphome/components/charge_controller/charge_logic.h` (append before `}  // namespace charge`)
- Modify: `tests/test_charge_logic.cpp`

**Interfaces:**
- Produces:
  - `constexpr float START_BUDGET_A = 2.0f`, `constexpr uint32_t FAULT_CLEAR_MS = 30000`
  - `enum class State : uint8_t { BOOT, WAIT_BMS, CHARGING, HOLD, FAULT }`, `const char *state_name(State)`
  - `struct Inputs { bool boot_done, bms_fresh, allowed, bms_alarm, paused; float budget; const char *fault; bool clear_fault; }`: `fault` is non-null while a fault condition is present and holds the reason text
  - `class StateMachine { State update(const Inputs &, uint32_t now_ms); State state() const; const std::string &last_fault() const; }`

- [ ] **Step 1: Add the failing tests** above `main()`, with the calls added to `main()`:

```cpp
static charge::Inputs good() {
  charge::Inputs in;
  in.boot_done = true;
  in.bms_fresh = true;
  in.allowed = true;
  in.budget = 37.0f;
  return in;
}

static void test_boot_and_wait() {
  charge::StateMachine sm;
  CHECK(sm.state() == charge::State::BOOT);
  charge::Inputs in = good();
  in.boot_done = false;
  CHECK(sm.update(in, 0) == charge::State::BOOT);
  in = good();
  in.bms_fresh = false;
  CHECK(sm.update(in, 100) == charge::State::WAIT_BMS);
  CHECK(sm.update(good(), 200) == charge::State::CHARGING);
  CHECK(std::string(charge::state_name(charge::State::WAIT_BMS)) == "WAIT_BMS");
}

static void test_hold_reasons() {
  charge::StateMachine sm;
  sm.update(good(), 0);
  charge::Inputs in = good();
  in.paused = true;
  CHECK(sm.update(in, 1) == charge::State::HOLD);
  in = good();
  in.allowed = false;
  CHECK(sm.update(in, 2) == charge::State::HOLD);
  in = good();
  in.bms_alarm = true;
  CHECK(sm.update(in, 3) == charge::State::HOLD);
  in = good();
  in.budget = 0.0f;
  CHECK(sm.update(in, 4) == charge::State::HOLD);
}

static void test_start_hysteresis() {
  charge::StateMachine sm;
  charge::Inputs in = good();
  in.budget = 1.0f;
  CHECK(sm.update(in, 0) == charge::State::HOLD);      // needs >= 2 A to start
  in.budget = 2.0f;
  CHECK(sm.update(in, 1) == charge::State::CHARGING);
  in.budget = 0.5f;
  CHECK(sm.update(in, 2) == charge::State::CHARGING);  // keeps going while > 0
  in.budget = 0.0f;
  CHECK(sm.update(in, 3) == charge::State::HOLD);
}

static void test_fault_latch() {
  charge::StateMachine sm;
  sm.update(good(), 0);
  charge::Inputs in = good();
  in.fault = "PSU 1 fault";
  CHECK(sm.update(in, 1000) == charge::State::FAULT);
  CHECK(sm.last_fault() == "PSU 1 fault");
  CHECK(sm.update(good(), 2000) == charge::State::FAULT);    // clear timer starts at 2000
  CHECK(sm.update(good(), 31999) == charge::State::FAULT);
  CHECK(sm.update(good(), 32000) == charge::State::CHARGING);
  CHECK(sm.last_fault() == "PSU 1 fault");                   // kept for HA after recovery
}

static void test_fault_reappearing_restarts_timer() {
  charge::StateMachine sm;
  charge::Inputs bad = good();
  bad.fault = "Overvoltage";
  sm.update(bad, 0);
  sm.update(good(), 1000);
  sm.update(bad, 10000);
  CHECK(sm.update(good(), 11000) == charge::State::FAULT);
  CHECK(sm.update(good(), 40999) == charge::State::FAULT);
  CHECK(sm.update(good(), 41000) == charge::State::CHARGING);
}

static void test_clear_button() {
  charge::StateMachine sm;
  charge::Inputs bad = good();
  bad.fault = "PSU 2 fault";
  sm.update(bad, 0);
  bad.clear_fault = true;
  CHECK(sm.update(bad, 500) == charge::State::FAULT);  // can't clear while the condition is present
  charge::Inputs in = good();
  in.clear_fault = true;
  CHECK(sm.update(in, 1000) == charge::State::CHARGING);
}

static void test_fault_during_boot_returns_to_boot() {
  charge::StateMachine sm;
  charge::Inputs in = good();
  in.boot_done = false;
  in.fault = "PSU 1 fault";
  CHECK(sm.update(in, 0) == charge::State::FAULT);
  in.fault = nullptr;
  in.clear_fault = true;
  CHECK(sm.update(in, 100) == charge::State::BOOT);
}
```

Add to `main()`:

```cpp
  test_boot_and_wait();
  test_hold_reasons();
  test_start_hysteresis();
  test_fault_latch();
  test_fault_reappearing_restarts_timer();
  test_clear_button();
  test_fault_during_boot_returns_to_boot();
```

- [ ] **Step 2: Run it and confirm it fails.**

Run: `make -C tests`
Expected: compile errors, `no type named 'StateMachine' in namespace 'charge'` (and similar)

- [ ] **Step 3: Implement.** Append before `}  // namespace charge`:

```cpp
// ---- State machine ----

constexpr float START_BUDGET_A = 2.0f;     // HOLD -> CHARGING needs at least this much headroom
constexpr uint32_t FAULT_CLEAR_MS = 30000;  // a fault must stay clear this long before auto-recovery

enum class State : uint8_t { BOOT, WAIT_BMS, CHARGING, HOLD, FAULT };

inline const char *state_name(State s) {
  switch (s) {
    case State::BOOT:
      return "BOOT";
    case State::WAIT_BMS:
      return "WAIT_BMS";
    case State::CHARGING:
      return "CHARGING";
    case State::HOLD:
      return "HOLD";
    case State::FAULT:
      return "FAULT";
  }
  return "?";
}

struct Inputs {
  bool boot_done = false;
  bool bms_fresh = false;
  bool allowed = false;
  bool bms_alarm = false;
  bool paused = false;
  float budget = 0.0f;
  const char *fault = nullptr;  // non-null while a fault condition is present (the reason)
  bool clear_fault = false;     // the Clear fault button was pressed since the last update
};

class StateMachine {
 public:
  State update(const Inputs &in, uint32_t now_ms) {
    if (in.fault != nullptr) {
      state_ = State::FAULT;
      last_fault_ = in.fault;
      clear_timing_ = false;
      return state_;
    }
    if (state_ == State::FAULT) {
      if (!clear_timing_) {
        clear_timing_ = true;
        clear_since_ms_ = now_ms;
      }
      if (!in.clear_fault && now_ms - clear_since_ms_ < FAULT_CLEAR_MS) return state_;
      clear_timing_ = false;
    }
    if (!in.boot_done) {
      state_ = State::BOOT;
    } else if (!in.bms_fresh) {
      state_ = State::WAIT_BMS;
    } else if (in.paused || !in.allowed || in.bms_alarm || !(in.budget > 0.0f)) {
      state_ = State::HOLD;
    } else if (state_ == State::CHARGING || in.budget >= START_BUDGET_A) {
      state_ = State::CHARGING;
    } else {
      state_ = State::HOLD;
    }
    return state_;
  }

  State state() const { return state_; }
  const std::string &last_fault() const { return last_fault_; }

 private:
  State state_ = State::BOOT;
  std::string last_fault_;
  bool clear_timing_ = false;
  uint32_t clear_since_ms_ = 0;
};
```

- [ ] **Step 4: Run the tests and confirm they pass.**

Run: `make -C tests`
Expected: `nasn_protocol: ok` and `charge_logic: ok`

- [ ] **Step 5: Commit.**

```bash
git add tests/test_charge_logic.cpp esphome/components/charge_controller/charge_logic.h
git commit -m "charge_logic.h: charger state machine with fault latch and start hysteresis

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: `tools/fake_nasn.py`, a NASN emulator for the bench

**Files:**
- Create: `tools/fake_nasn.py`, `tools/test_fake_nasn.py`

**Interfaces:**
- Produces:
  - `FakeNasn(addr, battery_v=52.0, default_v=54.0, default_a=5.0)` with:
    - `.handle(arb, data) -> (reply_id, reply_bytes) | None`
    - `.reset_to_defaults()`, `.registers() -> dict[int, int]`
    - attributes `muted`, `alarm`, `standby`, `vout_set`, `ilimit`
  - CLI: `fake_nasn.py [--seconds N] [--addrs 1,2] [--mute A] [--alarm A] [--fallback-after S] [--battery-v V]`

- [ ] **Step 1: Install pytest in the venv.**

Run: `.venv/bin/pip install -q pytest`

- [ ] **Step 2: Write the failing tests in `tools/test_fake_nasn.py`.**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from fake_nasn import FakeNasn  # noqa: E402


def frame(*b):
    return bytes(b)


def test_read_voltage_setpoint():
    m = FakeNasn(1)
    rid, data = m.handle(0x108182FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0))
    assert rid == 0x1081827E
    assert data[:2] == bytes([0x01, 0x00])
    assert int.from_bytes(data[2:], "big") == round(54.0 * 1023)


def test_write_voltage_current_standby():
    m = FakeNasn(2)
    rid, data = m.handle(0x108281FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))
    assert rid == 0x1082817E and data == frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7)
    assert abs(m.vout_set - 57.0) < 0.001
    m.handle(0x108281FE, frame(0x02, 0x2D, 0, 0, 0, 0, 0x02, 0x00))
    assert abs(m.ilimit - 26.03) < 0.01
    m.handle(0x108281FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))
    assert m.standby == 1


def test_telemetry_follows_state():
    m = FakeNasn(1, battery_v=52.0)
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))  # standby
    _, d = m.handle(0x108182FE, frame(0x01, 0x75, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 52 * 1024
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 0
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 0, 0, 0, 0, 0))  # on, 54 V > 52 V battery, 5 A default
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 5 * 1024


def test_ignores_other_traffic():
    m = FakeNasn(1)
    assert m.handle(0x108282FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None  # addr 2
    assert m.handle(0x1081827E, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None  # a reply, not a request
    assert m.handle(0x108182FE, frame(0x01, 0x00)) is None                      # short
    m.muted = True
    assert m.handle(0x108182FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None


def test_unknown_register_status_2():
    m = FakeNasn(1)
    _, d = m.handle(0x108182FE, frame(0x07, 0x77, 0, 0, 0, 0, 0, 0))
    assert d[0] >> 4 == 2 and (d[0] & 0x0F) == 0x07 and d[1] == 0x77


def test_fallback_resets_setpoints():
    m = FakeNasn(1)
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))
    m.reset_to_defaults()
    assert abs(m.vout_set - 54.0) < 0.001 and m.ilimit == 5.0 and m.standby == 0
```

- [ ] **Step 3: Run it and confirm it fails.**

Run: `.venv/bin/python -m pytest tools/test_fake_nasn.py -q`
Expected: `ModuleNotFoundError: No module named 'fake_nasn'`

- [ ] **Step 4: Implement `tools/fake_nasn.py`.**

```python
#!/usr/bin/env python3
"""Emulate NASN RM4850-B rectifiers (the dialect the stock Eco-Worthy firmware speaks) on the CAN bus.

Answers cmd 0x82 register reads and cmd 0x81 register writes for each emulated address. If the
controller goes quiet for --fallback-after seconds, the modules revert to their stored defaults,
as real modules are expected to. Bench use only: never run this on a bus with real PSUs attached.

  fake_nasn.py [--seconds 600] [--addrs 1,2] [--mute 2] [--alarm 1] [--fallback-after 60] [--battery-v 52]
"""
import argparse
import time

import can

import sniff  # noqa: F401  (macOS gs_usb detach patch)

SETPOINT_SCALE = 1023
TELEMETRY_SCALE = 1024
NOMINAL_A = 52.0
WRITE_CMD, READ_CMD = 0x81, 0x82


class FakeNasn:
    """One emulated module. handle() takes a controller request and returns the reply, or None."""

    def __init__(self, addr, battery_v=52.0, default_v=54.0, default_a=5.0):
        self.addr = addr
        self.battery_v = battery_v
        self.default_v = default_v
        self.default_a = default_a
        self.alarm = 0
        self.muted = False
        self.reset_to_defaults()

    def reset_to_defaults(self):
        self.vout_set = self.default_v
        self.ilimit = self.default_a
        self.standby = 0  # modules come up running at their stored defaults

    def registers(self):
        on = not self.standby
        iout = self.ilimit if on and self.vout_set > self.battery_v else 0.0
        vout = self.vout_set if on else self.battery_v
        p_out = vout * iout
        p_in = p_out / 0.96 + 15 if on else 5

        def t(x):
            return round(x * TELEMETRY_SCALE)

        return {
            0x0100: round(self.vout_set * SETPOINT_SCALE),
            0x0101: round(self.default_v * SETPOINT_SCALE),
            0x0104: round(self.default_a * SETPOINT_SCALE / NOMINAL_A),
            0x0132: self.standby,
            0x0170: t(p_in),
            0x0171: t(50.0),
            0x0172: t(p_in / 230.0),
            0x0173: t(p_out),
            0x0175: t(vout),
            0x0178: t(230.0),
            0x017F: t(35.0),
            0x0180: t(30.0),
            0x0182: t(iout),
            0x0183: self.alarm,
            0x022D: round(self.ilimit * SETPOINT_SCALE / NOMINAL_A),
        }

    def handle(self, arb, data):
        if self.muted or len(data) != 8 or (arb >> 23) & 0x3F != 0x21 or not arb & 0x80:
            return None
        if (arb >> 16) & 0x7F != self.addr:
            return None
        cmd = (arb >> 8) & 0xFF
        reg = (data[0] & 0x0F) << 8 | data[1]
        reply_id = arb & ~0x80
        error = bytes([0x20 | (data[0] & 0x0F), data[1]])  # status 2 in the high nibble
        if cmd == READ_CMD:
            regs = self.registers()
            if reg not in regs:
                return reply_id, error + bytes(6)
            return reply_id, bytes(data[:2]) + regs[reg].to_bytes(6, "big")
        if cmd == WRITE_CMD:
            raw = int.from_bytes(data[6:8], "big")
            if reg == 0x0100:
                self.vout_set = raw / SETPOINT_SCALE
            elif reg == 0x022D:
                self.ilimit = raw * NOMINAL_A / SETPOINT_SCALE
            elif reg == 0x0132:
                self.standby = data[3]
            elif reg == 0x0101:
                self.default_v = raw / SETPOINT_SCALE
            elif reg == 0x0104:
                self.default_a = raw * NOMINAL_A / SETPOINT_SCALE
            else:
                return reply_id, error + bytes(data[2:])
            return reply_id, bytes(data)
        return None

    def status(self):
        return (f"addr {self.addr}: {'STANDBY' if self.standby else 'ON     '} vset={self.vout_set:6.2f} V "
                f"ilim={self.ilimit:5.1f} A defaults={self.default_v:.2f} V/{self.default_a:.1f} A"
                f"{' MUTED' if self.muted else ''}{f' ALARM={self.alarm:#06x}' if self.alarm else ''}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=600)
    ap.add_argument("--addrs", default="1,2")
    ap.add_argument("--mute", type=int, action="append", default=[], help="address that never answers")
    ap.add_argument("--alarm", type=int, action="append", default=[], help="address reporting alarm bit 0x0001")
    ap.add_argument("--fallback-after", type=float, default=60)
    ap.add_argument("--battery-v", type=float, default=52.0)
    a = ap.parse_args()

    mods = {int(x): FakeNasn(int(x), battery_v=a.battery_v) for x in a.addrs.split(",")}
    for addr in a.mute:
        mods[addr].muted = True
    for addr in a.alarm:
        mods[addr].alarm = 0x0001

    bus = can.Bus(interface="gs_usb", channel=0, index=0, bitrate=125000)
    t0 = last_rx = next_status = time.time()
    fell_back = False
    print(f"fake NASN at {sorted(mods)} for {a.seconds:.0f}s", flush=True)
    try:
        while time.time() - t0 < a.seconds:
            msg = bus.recv(timeout=0.2)
            now = time.time()
            if msg is not None and msg.is_rx and msg.is_extended_id and not msg.is_error_frame:
                data = bytes(msg.data)
                last_rx, fell_back = now, False
                for mod in mods.values():
                    reply = mod.handle(msg.arbitration_id, data)
                    if reply is None:
                        continue
                    bus.send(can.Message(arbitration_id=reply[0], data=reply[1], is_extended_id=True))
                    if (msg.arbitration_id >> 8) & 0xFF == WRITE_CMD:
                        print(f"({now - t0:8.3f}) write {msg.arbitration_id:08X}#{data.hex(' ')}  -> {mod.status()}", flush=True)
            if not fell_back and now - last_rx > a.fallback_after:
                for mod in mods.values():
                    mod.reset_to_defaults()
                fell_back = True
                print(f"({now - t0:8.3f}) CAN silent {a.fallback_after:.0f}s: modules revert to stored defaults", flush=True)
            if now >= next_status:
                for mod in mods.values():
                    print(f"({now - t0:8.3f}) {mod.status()}", flush=True)
                next_status = now + 5
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Run the tests and confirm they pass.**

Run: `.venv/bin/python -m pytest tools/test_fake_nasn.py -q`
Expected: `6 passed`

- [ ] **Step 6: Smoke-test against `psu_ctl.py`.** Two CAN processes can't share the CANable, so this is pytest-only. Check the CLI parses:

Run: `.venv/bin/python tools/fake_nasn.py -h`
Expected: the usage text, listing `--mute`, `--alarm` and `--fallback-after`

- [ ] **Step 7: Commit.**

```bash
git add tools/fake_nasn.py tools/test_fake_nasn.py
git commit -m "fake_nasn.py: NASN dialect emulator for bench-testing the ESP32

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: `nasn_rectifier` ESPHome component

**Files:**
- Create: `esphome/components/nasn_rectifier/__init__.py`, `nasn_rectifier.h`, `nasn_rectifier.cpp`
- Modify: `esphome/eco-charger.yaml`: remove the Task 1 pin-check `on_frame` and `interval` blocks, then add the rectifiers

**Interfaces:**
- Consumes: `nasn_protocol.h` (Task 2).
- Produces (namespace `esphome::nasn_rectifier`): class `NasnRectifier : public PollingComponent`, with this public API used by Task 8:
  - `void set_target(float volts, float amps)`: per-PSU amps, clamped by the protocol layer
  - `void set_standby(bool standby)`
  - `void set_fallback(float volts, float amps)`
  - `bool is_online() const`, `bool faulted() const`, `bool boot_sent() const`
  - `float output_voltage() const`, `float output_current() const`: NAN when unknown
  - `uint16_t alarms() const`, `uint32_t consecutive_write_errors() const`, `uint8_t address() const`
- YAML: `nasn_rectifier:` list items with:
  - `id`, `canbus_id`, `address`, `update_interval` (2s), `keepalive_interval` (5s)
  - `write_stored_defaults` (false), `fallback_voltage` (54.0), `fallback_current` (5.0), `alarm_mask` (0)
  - optional sensors: `output_voltage`, `output_current`, `output_power`, `input_voltage`, `input_current`, `input_power`, `input_frequency`, `input_temperature`, `output_temperature`, `voltage_setpoint`, `current_setpoint`, `alarms`, `raw_021d`, `raw_022f`, `raw_0239`, `raw_023a`, `raw_023b`, `raw_024e`
  - binary sensor `online`

- [ ] **Step 1: Write `esphome/components/nasn_rectifier/nasn_rectifier.h`.**

```cpp
#pragma once

#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "nasn_protocol.h"

namespace esphome::nasn_rectifier {

enum ValueKind : uint8_t { KIND_TELEMETRY = 0, KIND_VOLTS = 1, KIND_AMPS = 2, KIND_RAW = 3 };

// One NASN RM4850-B module: polls its registers, and writes the setpoints on change and every
// keepalive interval (the module reverts to its stored defaults when CAN goes quiet).
class NasnRectifier : public PollingComponent {
 public:
  explicit NasnRectifier(canbus::Canbus *canbus) : canbus_(canbus) {}

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_address(uint8_t address) { this->address_ = address; }
  void set_keepalive_interval(uint32_t ms) { this->keepalive_ms_ = ms; }
  void set_write_stored_defaults(bool write) { this->write_stored_defaults_ = write; }
  void set_alarm_mask(uint16_t mask) { this->alarm_mask_ = mask; }
  void add_sensor(uint16_t reg, uint8_t kind, sensor::Sensor *sensor);
  void set_online_binary_sensor(binary_sensor::BinarySensor *sensor) { this->online_sensor_ = sensor; }

  // Control interface used by charge_controller.
  void set_target(float volts, float amps);
  void set_standby(bool standby);
  void set_fallback(float volts, float amps);
  bool is_online() const { return this->online_; }
  bool faulted() const {
    return this->online_ && ((this->alarms_ & this->alarm_mask_) != 0 || this->consecutive_write_errors_ >= 3);
  }
  bool boot_sent() const { return this->boot_sent_; }
  float output_voltage() const { return this->vout_; }
  float output_current() const { return this->iout_; }
  uint16_t alarms() const { return this->alarms_; }
  uint32_t consecutive_write_errors() const { return this->consecutive_write_errors_; }
  uint8_t address() const { return this->address_; }

 protected:
  struct Entry {
    uint16_t reg;
    uint8_t kind;
    sensor::Sensor *sensor;
  };

  void on_frame_(uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data);
  void send_(uint32_t can_id, const nasn::Frame &frame);
  void send_setpoints_();
  void set_online_(bool online);
  void add_poll_reg_(uint16_t reg);

  canbus::Canbus *canbus_;
  uint8_t address_{1};
  uint32_t keepalive_ms_{5000};
  bool write_stored_defaults_{false};
  uint16_t alarm_mask_{0};
  float fallback_v_{54.0f};
  float fallback_a_{5.0f};
  bool defaults_dirty_{true};

  std::vector<Entry> entries_;
  std::vector<uint16_t> poll_regs_;
  size_t poll_pos_{0};
  uint32_t last_poll_send_{0};

  float target_v_{54.0f};
  float target_a_{0.0f};
  bool standby_{true};
  bool dirty_{true};
  bool boot_sent_{false};
  uint32_t last_setpoint_send_{0};

  bool online_{false};
  uint32_t last_reply_{0};
  float vout_{NAN};
  float iout_{NAN};
  uint16_t alarms_{0};
  uint32_t consecutive_write_errors_{0};
  binary_sensor::BinarySensor *online_sensor_{nullptr};
};

}  // namespace esphome::nasn_rectifier
```

- [ ] **Step 2: Write `esphome/components/nasn_rectifier/nasn_rectifier.cpp`.**

```cpp
#include "nasn_rectifier.h"

#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::nasn_rectifier {

static const char *const TAG = "nasn_rectifier";
static const uint32_t POLL_SPACING_MS = 20;

static float convert(uint8_t kind, uint64_t raw) {
  switch (kind) {
    case KIND_VOLTS:
      return nasn::setpoint_volts(raw);
    case KIND_AMPS:
      return nasn::setpoint_amps(raw);
    case KIND_RAW:
      return float(raw);
    default:
      return nasn::telemetry(raw);
  }
}

void NasnRectifier::setup() {
  // Registers the control loop needs, whether or not they have sensors.
  for (uint16_t reg : {nasn::reg::VOUT, nasn::reg::IOUT, nasn::reg::ALARMS}) this->add_poll_reg_(reg);
  this->target_v_ = this->fallback_v_;
  this->canbus_->add_callback([this](uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data) {
    this->on_frame_(can_id, extended, rtr, data);
  });
  if (this->online_sensor_ != nullptr) this->online_sensor_->publish_state(false);
}

void NasnRectifier::add_sensor(uint16_t reg, uint8_t kind, sensor::Sensor *sensor) {
  this->entries_.push_back({reg, kind, sensor});
  this->add_poll_reg_(reg);
}

void NasnRectifier::add_poll_reg_(uint16_t reg) {
  for (uint16_t r : this->poll_regs_) {
    if (r == reg) return;
  }
  this->poll_regs_.push_back(reg);
}

void NasnRectifier::update() { this->poll_pos_ = 0; }

void NasnRectifier::loop() {
  const uint32_t now = millis();
  if (this->defaults_dirty_) {
    if (this->write_stored_defaults_) {
      this->send_(nasn::write_id(this->address_), nasn::write_voltage(nasn::reg::VOUT_DEFAULT, this->fallback_v_));
      this->send_(nasn::write_id(this->address_), nasn::write_current(nasn::reg::ILIMIT_DEFAULT, this->fallback_a_));
    }
    this->defaults_dirty_ = false;
  }
  if (this->dirty_ || now - this->last_setpoint_send_ >= this->keepalive_ms_) {
    this->send_setpoints_();
    this->last_setpoint_send_ = now;
    this->dirty_ = false;
    this->boot_sent_ = true;
  }
  if (this->poll_pos_ < this->poll_regs_.size() && now - this->last_poll_send_ >= POLL_SPACING_MS) {
    this->send_(nasn::read_id(this->address_), nasn::read_request(this->poll_regs_[this->poll_pos_++]));
    this->last_poll_send_ = now;
  }
  if (this->online_ && now - this->last_reply_ > 3 * this->get_update_interval()) this->set_online_(false);
}

void NasnRectifier::send_setpoints_() {
  const uint32_t id = nasn::write_id(this->address_);
  this->send_(id, nasn::write_voltage(nasn::reg::VOUT_SET, this->target_v_));
  this->send_(id, nasn::write_current(nasn::reg::ILIMIT, this->target_a_));
  this->send_(id, nasn::write_standby(this->standby_));
}

void NasnRectifier::send_(uint32_t can_id, const nasn::Frame &frame) {
  std::vector<uint8_t> data(frame.begin(), frame.end());
  this->canbus_->send_data(can_id, true, data);
}

void NasnRectifier::set_target(float volts, float amps) {
  if (!std::isfinite(volts) || !std::isfinite(amps)) return;
  if (std::fabs(volts - this->target_v_) >= 0.01f || std::fabs(amps - this->target_a_) >= 0.1f) {
    this->target_v_ = volts;
    this->target_a_ = amps;
    this->dirty_ = true;
  }
}

void NasnRectifier::set_standby(bool standby) {
  if (standby != this->standby_) {
    this->standby_ = standby;
    this->dirty_ = true;
  }
}

void NasnRectifier::set_fallback(float volts, float amps) {
  if (!std::isfinite(volts) || !std::isfinite(amps)) return;
  if (volts != this->fallback_v_ || amps != this->fallback_a_) {
    this->fallback_v_ = volts;
    this->fallback_a_ = amps;
    this->defaults_dirty_ = true;
  }
}

void NasnRectifier::on_frame_(uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data) {
  auto reply = nasn::parse_reply(can_id, extended, data.data(), data.size());
  if (!reply || reply->addr != this->address_) return;
  this->last_reply_ = millis();
  if (!this->online_) this->set_online_(true);

  if (reply->cmd == nasn::CMD_WRITE) {
    if (reply->status != 0) {
      this->consecutive_write_errors_++;
      ESP_LOGW(TAG, "PSU %u: write to 0x%04X rejected (status %u)", this->address_, reply->reg, reply->status);
    } else {
      this->consecutive_write_errors_ = 0;
    }
    return;
  }
  if (reply->status != 0) {
    ESP_LOGV(TAG, "PSU %u: read of 0x%04X failed (status %u)", this->address_, reply->reg, reply->status);
    return;
  }
  if (reply->reg == nasn::reg::VOUT) {
    this->vout_ = nasn::telemetry(reply->value);
  } else if (reply->reg == nasn::reg::IOUT) {
    this->iout_ = nasn::telemetry(reply->value);
  } else if (reply->reg == nasn::reg::ALARMS) {
    this->alarms_ = uint16_t(reply->value);
  }
  for (auto &e : this->entries_) {
    if (e.reg == reply->reg) e.sensor->publish_state(convert(e.kind, reply->value));
  }
}

void NasnRectifier::set_online_(bool online) {
  this->online_ = online;
  ESP_LOGI(TAG, "PSU %u %s", this->address_, online ? "online" : "offline");
  if (this->online_sensor_ != nullptr) this->online_sensor_->publish_state(online);
  if (online) {
    this->dirty_ = true;  // a module that just appeared needs our setpoints now
    this->defaults_dirty_ = true;
  } else {
    this->vout_ = NAN;
    this->iout_ = NAN;
    this->alarms_ = 0;
    this->consecutive_write_errors_ = 0;
    for (auto &e : this->entries_) e.sensor->publish_state(NAN);
  }
}

void NasnRectifier::dump_config() {
  ESP_LOGCONFIG(TAG, "NASN rectifier:");
  ESP_LOGCONFIG(TAG, "  Address: %u", this->address_);
  ESP_LOGCONFIG(TAG, "  Keepalive: %u ms", (unsigned) this->keepalive_ms_);
  ESP_LOGCONFIG(TAG, "  Fallback: %.1f V / %.1f A (write stored defaults: %s)", this->fallback_v_, this->fallback_a_,
                YESNO(this->write_stored_defaults_));
  ESP_LOGCONFIG(TAG, "  Alarm mask: 0x%04X", this->alarm_mask_);
  ESP_LOGCONFIG(TAG, "  Polled registers: %u", (unsigned) this->poll_regs_.size());
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace esphome::nasn_rectifier
```

- [ ] **Step 3: Write `esphome/components/nasn_rectifier/__init__.py`.**

```python
import esphome.codegen as cg
from esphome.components import binary_sensor, sensor
from esphome.components.canbus import CanbusComponent
import esphome.config_validation as cv
from esphome.const import (
    CONF_ADDRESS,
    CONF_ID,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_VOLT,
    UNIT_WATT,
)

DEPENDENCIES = ["canbus"]
AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = True

CONF_CANBUS_ID = "canbus_id"
CONF_KEEPALIVE_INTERVAL = "keepalive_interval"
CONF_WRITE_STORED_DEFAULTS = "write_stored_defaults"
CONF_FALLBACK_VOLTAGE = "fallback_voltage"
CONF_FALLBACK_CURRENT = "fallback_current"
CONF_ALARM_MASK = "alarm_mask"
CONF_ONLINE = "online"

nasn_rectifier_ns = cg.esphome_ns.namespace("nasn_rectifier")
NasnRectifier = nasn_rectifier_ns.class_("NasnRectifier", cg.PollingComponent)

# ValueKind in nasn_rectifier.h
TELEMETRY, VOLTS, AMPS, RAW = 0, 1, 2, 3


def _measure(unit, device_class, decimals):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        device_class=device_class,
        accuracy_decimals=decimals,
        state_class=STATE_CLASS_MEASUREMENT,
    )


def _raw():
    return sensor.sensor_schema(accuracy_decimals=0, entity_category=ENTITY_CATEGORY_DIAGNOSTIC)


# config key: (register, ValueKind, schema)
SENSORS = {
    "output_voltage": (0x0175, TELEMETRY, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    "output_current": (0x0182, TELEMETRY, _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 2)),
    "output_power": (0x0173, TELEMETRY, _measure(UNIT_WATT, DEVICE_CLASS_POWER, 0)),
    "input_voltage": (0x0178, TELEMETRY, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 1)),
    "input_current": (0x0172, TELEMETRY, _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 2)),
    "input_power": (0x0170, TELEMETRY, _measure(UNIT_WATT, DEVICE_CLASS_POWER, 0)),
    "input_frequency": (0x0171, TELEMETRY, _measure(UNIT_HERTZ, DEVICE_CLASS_FREQUENCY, 1)),
    "input_temperature": (0x0180, TELEMETRY, _measure(UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, 1)),
    "output_temperature": (0x017F, TELEMETRY, _measure(UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, 1)),
    "voltage_setpoint": (0x0100, VOLTS, _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    "current_setpoint": (0x022D, AMPS, _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1)),
    "alarms": (0x0183, RAW, _raw()),
    "raw_021d": (0x021D, RAW, _raw()),
    "raw_022f": (0x022F, RAW, _raw()),
    "raw_0239": (0x0239, RAW, _raw()),
    "raw_023a": (0x023A, RAW, _raw()),
    "raw_023b": (0x023B, RAW, _raw()),
    "raw_024e": (0x024E, RAW, _raw()),
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(NasnRectifier),
            cv.Required(CONF_CANBUS_ID): cv.use_id(CanbusComponent),
            cv.Required(CONF_ADDRESS): cv.int_range(min=1, max=127),
            cv.Optional(CONF_KEEPALIVE_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_WRITE_STORED_DEFAULTS, default=False): cv.boolean,
            cv.Optional(CONF_FALLBACK_VOLTAGE, default=54.0): cv.float_range(min=42.0, max=58.0),
            cv.Optional(CONF_FALLBACK_CURRENT, default=5.0): cv.float_range(min=0.0, max=50.0),
            cv.Optional(CONF_ALARM_MASK, default=0): cv.hex_uint16_t,
            cv.Optional(CONF_ONLINE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            **{cv.Optional(key): schema for key, (_, _, schema) in SENSORS.items()},
        }
    )
    .extend(cv.polling_component_schema("2s"))
)


async def to_code(config):
    canbus = await cg.get_variable(config[CONF_CANBUS_ID])
    var = cg.new_Pvariable(config[CONF_ID], canbus)
    await cg.register_component(var, config)
    cg.add(var.set_address(config[CONF_ADDRESS]))
    cg.add(var.set_keepalive_interval(config[CONF_KEEPALIVE_INTERVAL]))
    cg.add(var.set_write_stored_defaults(config[CONF_WRITE_STORED_DEFAULTS]))
    cg.add(var.set_alarm_mask(config[CONF_ALARM_MASK]))
    cg.add(var.set_fallback(config[CONF_FALLBACK_VOLTAGE], config[CONF_FALLBACK_CURRENT]))
    for key, (reg, kind, _) in SENSORS.items():
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(var.add_sensor(reg, kind, sens))
    if CONF_ONLINE in config:
        online = await binary_sensor.new_binary_sensor(config[CONF_ONLINE])
        cg.add(var.set_online_binary_sensor(online))
```

- [ ] **Step 4: Update `esphome/eco-charger.yaml`.**
  - Delete the `on_frame:` block under `canbus` and the whole `interval:` block.
  - Insert after `esphome:`:

```yaml
external_components:
  - source:
      type: local
      path: components
```

  - Append:

```yaml
nasn_rectifier:
  - id: psu1
    canbus_id: can
    address: 1
    online:
      name: PSU 1 online
    output_voltage:
      name: PSU 1 output voltage
    output_current:
      name: PSU 1 output current
    output_power:
      name: PSU 1 output power
    input_voltage:
      name: PSU 1 AC voltage
    input_current:
      name: PSU 1 AC current
    input_power:
      name: PSU 1 AC power
    input_temperature:
      name: PSU 1 inlet temperature
    output_temperature:
      name: PSU 1 outlet temperature
    voltage_setpoint:
      name: PSU 1 voltage setpoint
    current_setpoint:
      name: PSU 1 current setpoint
    alarms:
      name: PSU 1 alarm bits
    raw_021d: {name: PSU 1 reg 021D}
    raw_022f: {name: PSU 1 reg 022F}
    raw_0239: {name: PSU 1 reg 0239}
    raw_023a: {name: PSU 1 reg 023A}
    raw_023b: {name: PSU 1 reg 023B}
    raw_024e: {name: PSU 1 reg 024E}
  - id: psu2
    canbus_id: can
    address: 2
    online:
      name: PSU 2 online
    output_voltage:
      name: PSU 2 output voltage
    output_current:
      name: PSU 2 output current
    output_power:
      name: PSU 2 output power
    input_voltage:
      name: PSU 2 AC voltage
    input_current:
      name: PSU 2 AC current
    input_power:
      name: PSU 2 AC power
    input_temperature:
      name: PSU 2 inlet temperature
    output_temperature:
      name: PSU 2 outlet temperature
    voltage_setpoint:
      name: PSU 2 voltage setpoint
    current_setpoint:
      name: PSU 2 current setpoint
    alarms:
      name: PSU 2 alarm bits
    raw_021d: {name: PSU 2 reg 021D}
    raw_022f: {name: PSU 2 reg 022F}
    raw_0239: {name: PSU 2 reg 0239}
    raw_023a: {name: PSU 2 reg 023A}
    raw_023b: {name: PSU 2 reg 023B}
    raw_024e: {name: PSU 2 reg 024E}
```

- [ ] **Step 5: Compile.**

Run: `cd esphome && esphome compile eco-charger.yaml 2>&1 | tail -5`
Expected: `[SUCCESS]`. Fix any compile errors before going on: ESPHome API names can drift between releases, so check the installed headers under `~/.local/pipx/venvs/esphome/lib/python3.14/site-packages/esphome/components/`.

- [ ] **Step 6: Flash and run the bench test with the fake.** Upload with `esphome upload eco-charger.yaml --device /dev/cu.usbmodem1201`. Then run `.venv/bin/python tools/fake_nasn.py --seconds 120` in the background and `esphome logs eco-charger.yaml --device /dev/cu.usbmodem1201` in the background. Expected:
  - The fake logs a `write` of `0x0132 … 01` (standby) for each address within 1 s of boot, then the same three writes every 5 s.
  - The ESP log shows `PSU 1 online` and `PSU 2 online`, and `output voltage` sensor values of 52.00 V (fake battery, standby).

- [ ] **Step 7: Offline detection.** Restart the fake with `--mute 2`. Expected: within 6 s the ESP logs `PSU 2 offline`, and PSU 2's sensors publish NaN. PSU 1 stays online.

- [ ] **Step 8: Commit.**

```bash
git add esphome/components/nasn_rectifier esphome/eco-charger.yaml
git commit -m "nasn_rectifier: ESPHome component for NASN RM4850-B over CAN

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: `charge_controller` ESPHome component

**Files:**
- Create: `esphome/components/charge_controller/__init__.py`, `charge_controller.h`, `charge_controller.cpp`
- Modify: `esphome/eco-charger.yaml`

**Interfaces:**
- Consumes:
  - `charge_logic.h` (Tasks 3–5): `BmsTracker`, `target_voltage`, `current_budget`, `MovingAverage`, `RampLimiter`, `per_psu_current`, `overvoltage`, `StateMachine`, `Inputs`, `state_name`
  - `NasnRectifier` (Task 7): public API
- Produces: `ChargeController::clear_fault()`, called from a template button. YAML keys:
  - `rectifiers`, `max_voltage`, `max_current`, `fallback_voltage`, `fallback_current`, `pause`, `topics`
  - sensors `target_voltage`, `target_current`, `budget`, `output_power`, `bms_age`
  - text sensors `state`, `last_fault`

- [ ] **Step 1: Write `esphome/components/charge_controller/charge_controller.h`.**

```cpp
#pragma once

#include <string>
#include <vector>

#include "charge_logic.h"
#include "esphome/components/nasn_rectifier/nasn_rectifier.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome::charge_controller {

// Turns YamBMS limits (MQTT) and HA caps into per-rectifier targets once a second.
class ChargeController : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void add_rectifier(nasn_rectifier::NasnRectifier *rectifier) { this->rectifiers_.push_back(rectifier); }
  void set_max_voltage_number(number::Number *n) { this->max_v_ = n; }
  void set_max_current_number(number::Number *n) { this->max_a_ = n; }
  void set_fallback_voltage_number(number::Number *n) { this->fallback_v_ = n; }
  void set_fallback_current_number(number::Number *n) { this->fallback_a_ = n; }
  void set_pause_switch(switch_::Switch *s) { this->pause_ = s; }
  void set_topic(uint8_t topic, const std::string &name) { this->topics_[topic] = name; }

  void set_target_voltage_sensor(sensor::Sensor *s) { this->target_v_sensor_ = s; }
  void set_target_current_sensor(sensor::Sensor *s) { this->target_a_sensor_ = s; }
  void set_budget_sensor(sensor::Sensor *s) { this->budget_sensor_ = s; }
  void set_output_power_sensor(sensor::Sensor *s) { this->power_sensor_ = s; }
  void set_bms_age_sensor(sensor::Sensor *s) { this->bms_age_sensor_ = s; }
  void set_state_text_sensor(text_sensor::TextSensor *s) { this->state_sensor_ = s; }
  void set_last_fault_text_sensor(text_sensor::TextSensor *s) { this->fault_sensor_ = s; }

  void clear_fault() { this->clear_fault_ = true; }

 protected:
  void tick_(uint32_t now);
  const char *check_faults_(float target_v, float max_vout, float our_a);
  static float number_or_(number::Number *n, float fallback);

  std::vector<nasn_rectifier::NasnRectifier *> rectifiers_;
  number::Number *max_v_{nullptr};
  number::Number *max_a_{nullptr};
  number::Number *fallback_v_{nullptr};
  number::Number *fallback_a_{nullptr};
  switch_::Switch *pause_{nullptr};
  std::string topics_[charge::TOPIC_COUNT];

  charge::BmsTracker bms_;
  charge::StateMachine sm_;
  charge::MovingAverage battery_avg_;
  charge::MovingAverage ours_avg_;
  charge::RampLimiter ramp_;
  bool mqtt_was_connected_{false};
  bool clear_fault_{false};
  uint32_t last_tick_{0};
  char fault_text_[96]{};
  std::string published_state_;
  std::string published_fault_;

  sensor::Sensor *target_v_sensor_{nullptr};
  sensor::Sensor *target_a_sensor_{nullptr};
  sensor::Sensor *budget_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *bms_age_sensor_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};
  text_sensor::TextSensor *fault_sensor_{nullptr};
};

}  // namespace esphome::charge_controller
```

- [ ] **Step 2: Write `esphome/components/charge_controller/charge_controller.cpp`.**

```cpp
#include "charge_controller.h"

#include <cmath>
#include <cstdio>

#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::charge_controller {

static const char *const TAG = "charge_controller";
static const uint32_t TICK_MS = 1000;

void ChargeController::setup() {
  for (int i = 0; i < charge::TOPIC_COUNT; i++) {
    const auto topic = charge::Topic(i);
    mqtt::global_mqtt_client->subscribe(this->topics_[i], [this, topic](const std::string &, const std::string &payload) {
      const uint32_t now = millis();
      this->bms_.on_message(topic, payload, now);
      if (topic == charge::Topic::CURRENT) this->battery_avg_.add(this->bms_.current(), now);
    });
  }
}

void ChargeController::loop() {
  const uint32_t now = millis();
  const bool connected = mqtt::global_mqtt_client->is_connected();
  if (connected && !this->mqtt_was_connected_) this->bms_.on_reconnect();
  this->mqtt_was_connected_ = connected;
  if (now - this->last_tick_ >= TICK_MS) {
    this->last_tick_ = now;
    this->tick_(now);
  }
}

float ChargeController::number_or_(number::Number *n, float fallback) {
  return (n != nullptr && std::isfinite(n->state)) ? n->state : fallback;
}

void ChargeController::tick_(uint32_t now) {
  const float fb_v = number_or_(this->fallback_v_, 54.0f);
  const float fb_a = number_or_(this->fallback_a_, 5.0f);
  float our_a = 0.0f;
  float power = 0.0f;
  float max_vout = NAN;
  int healthy = 0;
  bool boot_done = true;
  for (auto *r : this->rectifiers_) {
    r->set_fallback(fb_v, fb_a);
    boot_done = boot_done && r->boot_sent();
    if (!r->is_online()) continue;
    const float v = r->output_voltage(), i = r->output_current();
    if (std::isfinite(i)) our_a += i;
    if (std::isfinite(v) && std::isfinite(i)) power += v * i;
    if (std::isfinite(v)) max_vout = std::isfinite(max_vout) ? std::max(max_vout, v) : v;
    if (!r->faulted()) healthy++;
  }
  this->ours_avg_.add(our_a, now);

  const float target_v = charge::target_voltage(this->bms_.cvl(), number_or_(this->max_v_, 57.0f));
  const float budget = charge::current_budget(this->bms_.ccl(), this->battery_avg_.value(), this->ours_avg_.value(),
                                              number_or_(this->max_a_, 0.0f));

  charge::Inputs in;
  in.boot_done = boot_done;
  in.bms_fresh = this->bms_.fresh(now);
  in.allowed = this->bms_.allowed();
  in.bms_alarm = this->bms_.alarm();
  in.paused = this->pause_ == nullptr || this->pause_->state;
  in.budget = budget;
  in.fault = this->check_faults_(target_v, max_vout, our_a);
  in.clear_fault = this->clear_fault_;
  this->clear_fault_ = false;
  const charge::State state = this->sm_.update(in, now);

  const bool charging = state == charge::State::CHARGING;
  float total = 0.0f;
  if (charging) {
    total = this->ramp_.step(budget, now);
  } else {
    this->ramp_.reset();
  }
  const float per_psu = charge::per_psu_current(total, healthy);
  for (auto *r : this->rectifiers_) {
    const bool run = charging && r->is_online() && !r->faulted();
    r->set_target(std::isfinite(target_v) ? target_v : fb_v, run ? per_psu : 0.0f);
    r->set_standby(!run);
  }

  if (this->target_v_sensor_ != nullptr) this->target_v_sensor_->publish_state(charging ? target_v : NAN);
  if (this->target_a_sensor_ != nullptr) this->target_a_sensor_->publish_state(total);
  if (this->budget_sensor_ != nullptr) this->budget_sensor_->publish_state(budget);
  if (this->power_sensor_ != nullptr) this->power_sensor_->publish_state(power);
  if (this->bms_age_sensor_ != nullptr) {
    const uint32_t age = this->bms_.age_ms(now);
    this->bms_age_sensor_->publish_state(age == UINT32_MAX ? NAN : age / 1000.0f);
  }
  const std::string state_str = charge::state_name(state);
  if (this->state_sensor_ != nullptr && state_str != this->published_state_) {
    ESP_LOGI(TAG, "State %s -> %s", this->published_state_.c_str(), state_str.c_str());
    this->state_sensor_->publish_state(state_str);
    this->published_state_ = state_str;
  }
  if (this->fault_sensor_ != nullptr && this->sm_.last_fault() != this->published_fault_) {
    this->fault_sensor_->publish_state(this->sm_.last_fault());
    this->published_fault_ = this->sm_.last_fault();
  }
}

const char *ChargeController::check_faults_(float target_v, float max_vout, float our_a) {
  for (auto *r : this->rectifiers_) {
    if (r->faulted()) {
      snprintf(this->fault_text_, sizeof(this->fault_text_), "PSU %u fault: alarms 0x%04X, write errors %u",
               r->address(), r->alarms(), (unsigned) r->consecutive_write_errors());
      return this->fault_text_;
    }
  }
  if (charge::overvoltage(this->sm_.state() == charge::State::CHARGING, max_vout, target_v, our_a)) {
    snprintf(this->fault_text_, sizeof(this->fault_text_), "Overvoltage: %.2f V measured, target %.2f V", max_vout,
             target_v);
    return this->fault_text_;
  }
  return nullptr;
}

void ChargeController::dump_config() {
  ESP_LOGCONFIG(TAG, "Charge controller:");
  ESP_LOGCONFIG(TAG, "  Rectifiers: %u", (unsigned) this->rectifiers_.size());
  for (int i = 0; i < charge::TOPIC_COUNT; i++) ESP_LOGCONFIG(TAG, "  Topic %d: %s", i, this->topics_[i].c_str());
}

}  // namespace esphome::charge_controller
```

- [ ] **Step 3: Write `esphome/components/charge_controller/__init__.py`.**

```python
import esphome.codegen as cg
from esphome.components import number, sensor, switch, text_sensor
from esphome.components.nasn_rectifier import NasnRectifier
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_SECOND,
    UNIT_VOLT,
    UNIT_WATT,
)

DEPENDENCIES = ["mqtt", "nasn_rectifier"]
AUTO_LOAD = ["sensor", "text_sensor"]

CONF_RECTIFIERS = "rectifiers"
CONF_MAX_VOLTAGE = "max_voltage"
CONF_MAX_CURRENT = "max_current"
CONF_FALLBACK_VOLTAGE = "fallback_voltage"
CONF_FALLBACK_CURRENT = "fallback_current"
CONF_PAUSE = "pause"
CONF_TOPICS = "topics"

charge_controller_ns = cg.esphome_ns.namespace("charge_controller")
ChargeController = charge_controller_ns.class_("ChargeController", cg.Component)

# Order matches charge::Topic in charge_logic.h.
TOPICS = {
    "status": "yambms/status",
    "charge_voltage": "yambms/yambms_1/sensor/requested_charge_voltage/state",
    "charge_current": "yambms/yambms_1/sensor/requested_charge_current/state",
    "charging_allowed": "yambms/yambms_1/binary_sensor/charging_allowed/state",
    "alarm": "yambms/yambms_1/sensor/bms_in_alarm/state",
    "current": "yambms/yambms_1/sensor/current/state",
}

NUMBERS = {
    CONF_MAX_VOLTAGE: "set_max_voltage_number",
    CONF_MAX_CURRENT: "set_max_current_number",
    CONF_FALLBACK_VOLTAGE: "set_fallback_voltage_number",
    CONF_FALLBACK_CURRENT: "set_fallback_current_number",
}


def _measure(unit, device_class, decimals):
    return sensor.sensor_schema(
        unit_of_measurement=unit,
        device_class=device_class,
        accuracy_decimals=decimals,
        state_class=STATE_CLASS_MEASUREMENT,
    )


SENSORS = {
    "target_voltage": ("set_target_voltage_sensor", _measure(UNIT_VOLT, DEVICE_CLASS_VOLTAGE, 2)),
    "target_current": ("set_target_current_sensor", _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1)),
    "budget": ("set_budget_sensor", _measure(UNIT_AMPERE, DEVICE_CLASS_CURRENT, 1)),
    "output_power": ("set_output_power_sensor", _measure(UNIT_WATT, DEVICE_CLASS_POWER, 0)),
    "bms_age": (
        "set_bms_age_sensor",
        sensor.sensor_schema(
            unit_of_measurement=UNIT_SECOND,
            device_class=DEVICE_CLASS_DURATION,
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    ),
}
TEXT_SENSORS = {
    "state": "set_state_text_sensor",
    "last_fault": "set_last_fault_text_sensor",
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ChargeController),
        cv.Required(CONF_RECTIFIERS): cv.All(cv.ensure_list(cv.use_id(NasnRectifier)), cv.Length(min=1)),
        **{cv.Required(key): cv.use_id(number.Number) for key in NUMBERS},
        cv.Required(CONF_PAUSE): cv.use_id(switch.Switch),
        cv.Optional(CONF_TOPICS, default={}): cv.Schema(
            {cv.Optional(key, default=topic): cv.subscribe_topic for key, topic in TOPICS.items()}
        ),
        **{cv.Optional(key): schema for key, (_, schema) in SENSORS.items()},
        **{cv.Optional(key): text_sensor.text_sensor_schema() for key in TEXT_SENSORS},
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for rid in config[CONF_RECTIFIERS]:
        cg.add(var.add_rectifier(await cg.get_variable(rid)))
    for key, setter in NUMBERS.items():
        cg.add(getattr(var, setter)(await cg.get_variable(config[key])))
    cg.add(var.set_pause_switch(await cg.get_variable(config[CONF_PAUSE])))
    for index, key in enumerate(TOPICS):
        cg.add(var.set_topic(index, config[CONF_TOPICS][key]))
    for key, (setter, _) in SENSORS.items():
        if key in config:
            cg.add(getattr(var, setter)(await sensor.new_sensor(config[key])))
    for key, setter in TEXT_SENSORS.items():
        if key in config:
            cg.add(getattr(var, setter)(await text_sensor.new_text_sensor(config[key])))
```

- [ ] **Step 4: Append to `esphome/eco-charger.yaml`.**

```yaml
mqtt:
  broker: homeassistant.local
  username: !secret mqtt_username
  password: !secret mqtt_password
  discovery: false
  topic_prefix: eco-charger
  log_topic: null

number:
  - platform: template
    id: max_v
    name: Max charge voltage
    unit_of_measurement: V
    entity_category: config
    min_value: 48
    max_value: 58
    step: 0.1
    initial_value: 57.0
    optimistic: true
    restore_value: true
  - platform: template
    id: max_a
    name: Max charge current
    unit_of_measurement: A
    entity_category: config
    min_value: 0
    max_value: 100
    step: 1
    initial_value: 100
    optimistic: true
    restore_value: true
  - platform: template
    id: fb_v
    name: Fallback voltage
    unit_of_measurement: V
    entity_category: config
    min_value: 48
    max_value: 58
    step: 0.1
    initial_value: 54.0
    optimistic: true
    restore_value: true
  - platform: template
    id: fb_a
    name: Fallback current (per PSU)
    unit_of_measurement: A
    entity_category: config
    min_value: 0
    max_value: 50
    step: 1
    initial_value: 5
    optimistic: true
    restore_value: true

switch:
  - platform: template
    id: pause_sw
    name: Pause charging
    optimistic: true
    restore_mode: RESTORE_DEFAULT_ON

button:
  - platform: template
    name: Clear fault
    on_press:
      - lambda: id(ctl).clear_fault();

charge_controller:
  id: ctl
  rectifiers: [psu1, psu2]
  max_voltage: max_v
  max_current: max_a
  fallback_voltage: fb_v
  fallback_current: fb_a
  pause: pause_sw
  state:
    name: Charger state
  last_fault:
    name: Last fault
  target_voltage:
    name: Target voltage
  target_current:
    name: Target current
  budget:
    name: Current budget
  output_power:
    name: Charger output power
  bms_age:
    name: BMS data age
```

- [ ] **Step 5: Compile.**

Run: `cd esphome && esphome compile eco-charger.yaml 2>&1 | tail -5`
Expected: `[SUCCESS]`

- [ ] **Step 6: Bench test with live MQTT and the fake PSUs.** Upload over USB or OTA. Run `fake_nasn.py --seconds 300` and `esphome logs` in the background. Expected:
  1. After boot: `State  -> BOOT`, then `WAIT_BMS` (data not fresh yet).
  2. Within about 20 s, `HOLD`, because `Pause charging` defaults to on. `BMS data age` reads under 15 s.
  3. Add the ESPHome device in HA (auto-discovered; it needs `api_key` from secrets) and turn `Pause charging` off. Expected: `CHARGING`. The fake logs `0x0132 … 00` (on) and a `0x0100` write at `min(CVL, 57.0)`. `Target current` ramps by about 2 A/s, up to the budget of `CCL − (YamBMS current − our output)`. This reads the live bank: with the inverter at about 238 A and a CCL of 275, the budget is about 37 A, split as about 18.5 A per PSU.
  4. Set `Max charge current` to 4. Expected: the current setpoints drop to 2 A per PSU immediately.
  5. Restart the fake with `--alarm 1` and set `alarm_mask: 0x0001` on psu1 temporarily (recompile and upload). Expected: `FAULT`, `Last fault` = `PSU 1 fault: alarms 0x0001…`, and both PSUs in standby. Restart the fake without `--alarm`. Expected: back to `CHARGING` 30 s later. **Then revert `alarm_mask` to its default.**
  6. Keepalive check: with the fake running, the line `CAN silent 60s: modules revert…` must never appear while the ESP32 runs. Unplug the ESP32's USB for 70 s: the line appears, and the fake's status shows `ON vset=54.00` (stored defaults).
  7. Turn `Pause charging` back on before moving on.

- [ ] **Step 7: Commit.**

```bash
git add esphome/components/charge_controller esphome/eco-charger.yaml
git commit -m "charge_controller: YamBMS-driven charge control with HA caps and fault latch

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 9: First real PSU (manual, with the user present)

This task runs on mains-powered hardware. **Do each step only with the user's explicit go-ahead,** and record every result in `docs/history/notes/findings.md` under a new `## Real-hardware verification (date)` heading.

**Files:**
- Modify: `docs/history/notes/findings.md`, `esphome/eco-charger.yaml` (only the gating settings below)

- [ ] **Step 1: Read-only register check.** One PSU in its backplane on mains, the stock board and the ESP32 disconnected, the CANable on the backplane CAN. No battery, and a dummy load if one is available.

Run: `.venv/bin/python tools/psu_ctl.py read --addr 1`
Record:
  - every register value;
  - whether `0x0175` reads about 53.5 V (default) at /1024, which confirms `TELEMETRY_SCALE`. Compare the **raw** values (`psu_ctl.py` prints them for `0x0175`/`0x0178`) against a multimeter on the output, since /1023 and /1024 differ by only 0.1 %;
  - the raw `0x0101`/`0x0104` values (stored defaults);
  - `0x0183` in the normal state;
  - whether any register returns status ≠ 0.

- [ ] **Step 2: Standby bits.** Run `psu_ctl.py off`, then `read`, then `on`, then `read`. Record `0x0183` and `0x0132` in each state, including the **raw 6-byte layout of the `0x0132` readback** (the controller treats any nonzero value as standby; confirm that holds). From this, choose `alarm_mask` as the bits that are neither 0 in normal operation nor standby-related. If that's unclear, leave the mask at 0 and record why.

- [ ] **Step 3: CAN-silence fallback, from standby, into the dummy load.**
  - Connect the dummy load. Run `psu_ctl.py off` (standby), confirm with `read`, then send nothing for 90 s and read again.
  - Record **whether the module turns itself ON** at fallback (`0x0132` and the output), and after how long.
  - Record **the fallback current into the dummy load** (clamp meter and `0x0182`) and the output voltage, i.e. what the module would push into a battery unattended.
  - Repeat from running: `psu_ctl.py set-voltage 50`, read back `0x0100`, send nothing for 90 s, read again. Record whether it reverted to the stored default, and after how long.
  - **If it never reverts:** stop. The spec's fallback layer 1 fails, and the external AC relay safeguard needs designing before connecting a battery.
  - **Gate for Step 8 (battery):** continue to the battery only if the fallback defaults are safe for the battery unattended (≤ 54.4 V and ≤ 10 A per module, as now enforced by the controller, and acceptable at the battery's resting voltage), **or** the external AC relay with a hardware timeout is fitted. Record which condition was met.

- [ ] **Step 4: Stored defaults write.** Only if Step 1 showed `0x0101` ≈ V × 1023:
  - Write 54.0 V via `psu_ctl.py`. That needs a small `set-default-voltage` addition: `write_reg(bus, a.addr, 0x0101, bytes([0, 0, 0, 0, raw >> 8, raw & 0xFF]))`.
  - Power-cycle the PSU and read `0x0100`.
  - If it comes up at 54.0, set `write_stored_defaults: true` for both rectifiers in the YAML. The controller then polls `0x0101`/`0x0104` and writes them only when the readback differs (at most once per keepalive), with the fallback capped at 54.4 V / 10 A.
  - Repeat the same for `0x0104` with 5 A, using the encoding Step 1 suggests.

- [ ] **Step 5: ESP32 drive into a dummy load.** Connect the ESP32 in place of the CANable. Set `Max charge current` to 2 and turn `Pause charging` off. Expected: PSU 1 on, at `min(CVL, 57)` V, with the current limited to 2 A. Watch the HA entities for 5 minutes (no `PSU n not following setpoints` fault should appear), then pause again.

- [ ] **Step 6: Resilience while driving the dummy load.** With PSU 1 driving the dummy load as in Step 5 (CANable on the bus in listen-only mode via `sniff.py`, if available, to log the frames):
  - **Write ACKs:** confirm the write replies for `0x0100`, `0x022D` and `0x0132` all carry status 0 (high nibble of byte 0), and that no `write ... rejected` warnings appear in the ESP log.
  - **ESP reboot:** restart the ESP32 from HA. The PSU must go to standby (current 0) on shutdown, before the reboot. Record the output current through the reboot and how long it takes to return to the commanded state.
  - **OTA:** run an OTA upload while it is driving the load. The PSU must go to standby at the start of the upload (`on_begin` → `stop_now()`), and stay there until the new firmware is running. Record the output during the upload.
  - **CAN unplug:** unplug CAN-H/L at the ESP side for ~10 s, then replug. Record that the PSU is marked offline, that the ESP's CAN controller recovers from bus-off without a reboot, and that the PSU comes back online and follows the setpoints again. Repeat with ~90 s unplugged and record whether the PSU fell back to its stored defaults in between.

- [ ] **Step 7: Lock down MQTT commands.** Before any battery is connected, set a broker ACL that **denies writes** to `eco-charger/+/+/command` for every client except an admin account, so nothing on the network can raise the caps or unpause charging. Confirm with `mosquitto_pub` from an ordinary account that the command is rejected (and the HA entity doesn't change).

- [ ] **Step 8: Battery (both PSUs, through a fuse).** Only once the Step 3 gate is met and Step 7 is in place. With the user present:
  - Start at `Max charge current` = 10, then raise it step by step.
  - Confirm that the battery current reported by YamBMS never exceeds the CCL while the inverter is also charging.
  - Confirm that pulling the ESP32's WiFi (or stopping MQTT) drops the charger into `WAIT_BMS`/standby within 60 s.

- [ ] **Step 9: Commit the findings and any gating changes.**

```bash
git add docs/history/notes/findings.md esphome/eco-charger.yaml tools/psu_ctl.py
git commit -m "Real-hardware verification of NASN registers, fallback and alarm bits

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
