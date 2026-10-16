# OEM Bridge Home Assistant Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The Waveshare bridge polls the OEM charger over Modbus for Home Assistant by default, and a Passthrough switch (flipped by the tools, behind web-server auth) hands the RS485 line to the TCP stream for `oem_modbus.py` / `oem_update.py`.

**Architecture:** A small ESPHome external component, `rs485_mux`, owns the real UART and exposes two virtual UART ports; only the active one receives bytes and may transmit. `modbus`/`modbus_controller` sit on one port, `stream_server` on the other. The switch, the idle timeout and the baud reset are YAML (template switch + script + interval) around two mux methods. The tools turn Passthrough on/off over HTTP with basic auth.

**Tech Stack:** ESPHome 2026.8.1 (ESP-IDF, ESP32-S3), C++17, Python 3 tools with pytest, host C++ tests under `make -C tests`.

**Spec:** `docs/history/superpowers/specs/2026-10-15-oem-bridge-home-assistant-design.md`

## Global Constraints

- **Never print, cat, grep or commit `esphome/secrets.yaml`.** Never run `esphome config` (it prints the resolved config, secrets included). Validate with `esphome compile` only, and don't paste its full output into reports.
- The user adds `web_username` and `web_password` to `esphome/secrets.yaml` before Task 2 can compile. If `esphome compile` reports them missing, stop and report NEEDS_CONTEXT.
- ESPHome is at `~/.local/bin/esphome` (2026.8.1). Compile with `~/.local/bin/esphome compile esphome/oem-bridge.yaml`. Never `run`/`upload`/OTA: flashing the bridge is done by the controller after review.
- The bridge's charger link: Modbus RTU, 9600 8N1, address 1, on the UART at TX GPIO17 / RX GPIO18 / direction GPIO21.
- HA owns the line by default and after every reboot; PASSTHROUGH only while the switch is on. Idle timeout 10 min without a TCP client. Turning passthrough off sets the baud back to 9600.
- Writable settings: HR 0-6 and 9-16. HR 7 (Modbus address) and HR 8 (Modbus baud) are read-only in HA.
- Tools read web credentials from `OEM_BRIDGE_USER` / `OEM_BRIDGE_PASSWORD`; never put credentials in code, tests or docs.
- Commit messages end with:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Q88Pr4eMBeeEdBYp5fsSSu
  ```
- Never push. Don't touch `oem/` (charger firmware) in this plan.

**Rulings against the spec (recorded here, binding for the tasks):**
- Spec §3 shows `client_connected`, `on_passthrough`/`on_modbus` triggers and a `switch` platform in the component. The plan does these in YAML instead: the stream_server "connected" binary sensor's `on_state` calls `id(mux).set_client_connected(x)`, a template switch + `passthrough_off` script do the suspend/resume and baud reset, and a 1 s `interval` polls `id(mux).idle_expired()`. Same behaviour, no codegen for triggers or a switch platform. Spec §7's `switch/` directory is therefore not created.

---

## File structure

```
esphome/components/rs485_mux/__init__.py   config schema + codegen (component, two ports)
esphome/components/rs485_mux/rs485_mux.h   RS485Mux component and MuxPort (virtual UART)
esphome/components/rs485_mux/rs485_mux.cpp routing, port I/O
esphome/components/rs485_mux/mux_logic.h   IdleTimer (no ESPHome dependency; host-tested)
tests/test_rs485_mux.cpp                   host test for IdleTimer
tests/Makefile                             add test_rs485_mux
esphome/oem-bridge.yaml                    mux, modbus_controller entities, Passthrough switch, web auth
tools/oem_modbus.py, tools/oem_update.py   passthrough on/off around --bridge; basic auth
tools/test_oem_modbus.py, tools/test_oem_update.py
README.md                                  "Remote access through the WiFi bridge"
```

---

### Task 1: IdleTimer (mux_logic.h) with a host test

**Files:**
- Create: `esphome/components/rs485_mux/mux_logic.h`
- Create: `tests/test_rs485_mux.cpp`
- Modify: `tests/Makefile` (TESTS list)

**Interfaces:**
- Produces: `rs485_mux::IdleTimer` with `explicit IdleTimer(uint32_t timeout_ms)`, `bool update(bool passthrough, bool client, uint32_t now_ms)` (true when passthrough should be turned off), `void set_timeout(uint32_t timeout_ms)`.

- [ ] **Step 1: Write the failing test** — `tests/test_rs485_mux.cpp`:

```cpp
#include "check.h"
#include "rs485_mux/mux_logic.h"

using rs485_mux::IdleTimer;

static void test_off_never_fires() {
  IdleTimer t(1000);
  CHECK(!t.update(false, false, 0));
  CHECK(!t.update(false, false, 5000));
}

static void test_on_without_client_fires_after_timeout() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 100));  // switched on at 100
  CHECK(!t.update(true, false, 1099));
  CHECK(t.update(true, false, 1100));
}

static void test_client_holds_it_and_disconnect_restarts_the_count() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(true, true, 500));    // client connects
  CHECK(!t.update(true, true, 50000));  // never times out while connected
  CHECK(!t.update(true, false, 50001)); // disconnect: count from here
  CHECK(!t.update(true, false, 51000));
  CHECK(t.update(true, false, 51001));
}

static void test_reconnect_inside_the_window_restarts() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(true, true, 900));
  CHECK(!t.update(true, false, 950));
  CHECK(!t.update(true, false, 1900));
  CHECK(t.update(true, false, 1950));
}

static void test_turning_off_resets() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(false, false, 900));  // turned off by hand
  CHECK(!t.update(true, false, 950));   // on again: a fresh 1000 ms
  CHECK(!t.update(true, false, 1900));
  CHECK(t.update(true, false, 1950));
}

static void test_millis_wraparound() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0xFFFFFF00u));
  CHECK(!t.update(true, false, 0x000002E7u));  // 999 ms later
  CHECK(t.update(true, false, 0x000002E8u));   // 1000 ms later
}

static void test_set_timeout() {
  IdleTimer t(1000);
  t.set_timeout(600000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(true, false, 599999));
  CHECK(t.update(true, false, 600000));
}

int main() {
  test_off_never_fires();
  test_on_without_client_fires_after_timeout();
  test_client_holds_it_and_disconnect_restarts_the_count();
  test_reconnect_inside_the_window_restarts();
  test_turning_off_resets();
  test_millis_wraparound();
  test_set_timeout();
  return test_result("test_rs485_mux");
}
```

In `tests/Makefile`, append `test_rs485_mux` to the end of the `TESTS =` list (after `test_oem_modbus_map`). `HEADERS` already matches `../esphome/components/*/*_logic.h`.

- [ ] **Step 2: Run it to verify it fails**

Run: `make -C tests test_rs485_mux`
Expected: compile error, `rs485_mux/mux_logic.h: No such file or directory`.

- [ ] **Step 3: Implement** — `esphome/components/rs485_mux/mux_logic.h`:

```cpp
// rs485_mux idle timeout: passthrough turns itself off after timeout_ms without a TCP client, counted from the
// switch-on or from the last client leaving. Plain C++ (no ESPHome) so it builds in the host tests.
#pragma once
#include <cstdint>

namespace rs485_mux {

class IdleTimer {
 public:
  explicit IdleTimer(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}
  void set_timeout(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

  // Call every loop. True when passthrough should be turned off; it keeps returning true until it is.
  bool update(bool passthrough, bool client, uint32_t now_ms) {
    if (!passthrough || client) {
      armed_ = false;
      return false;
    }
    if (!armed_) {
      armed_ = true;
      since_ms_ = now_ms;
    }
    return now_ms - since_ms_ >= timeout_ms_;
  }

 private:
  uint32_t timeout_ms_;
  uint32_t since_ms_ = 0;
  bool armed_ = false;
};

}  // namespace rs485_mux
```

- [ ] **Step 4: Run all host tests**

Run: `make -C tests test`
Expected: every suite prints `ok`, including `test_rs485_mux: ok`.

- [ ] **Step 5: Commit**

```bash
git add esphome/components/rs485_mux/mux_logic.h tests/test_rs485_mux.cpp tests/Makefile
git commit -m "rs485_mux: IdleTimer for the passthrough idle timeout, host-tested"
```
(with the Co-Authored-By / Claude-Session lines from Global Constraints)

---

### Task 2: rs485_mux component and the bridge wiring (passthrough switch, idle timeout, web auth)

**Files:**
- Create: `esphome/components/rs485_mux/__init__.py`
- Create: `esphome/components/rs485_mux/rs485_mux.h`
- Create: `esphome/components/rs485_mux/rs485_mux.cpp`
- Modify: `esphome/oem-bridge.yaml` (full replacement below)

**Interfaces:**
- Consumes: `rs485_mux::IdleTimer` (Task 1).
- Produces (used from YAML lambdas in Tasks 2-3): `id(mux).set_passthrough(bool)`, `id(mux).passthrough()`, `id(mux).set_client_connected(bool)`, `id(mux).idle_expired()`; port IDs `mux_modbus`, `mux_stream`; `modbus_controller` id `charger`; template select `rs485_baud` (name "RS485 baud"); template switch name "Passthrough"; script `passthrough_off`.

- [ ] **Step 1: Write the component header** — `esphome/components/rs485_mux/rs485_mux.h`:

```cpp
// rs485_mux: one RS485 UART shared by modbus_controller (Home Assistant polling) and stream_server (raw TCP for the
// tools). Both read every byte they're given, so the mux hands the line to one side at a time: only the active port
// receives bytes and may transmit. MODBUS by default; PASSTHROUGH while set_passthrough(true).
// docs/history/superpowers/specs/2026-10-15-oem-bridge-home-assistant-design.md §3.
#pragma once
#include <deque>

#include "esphome/components/uart/uart_component.h"
#include "esphome/core/component.h"
#include "mux_logic.h"

namespace esphome {
namespace rs485_mux {

class RS485Mux;

// One side of the mux. Looks like a UART to its user; settings (baud, bits) mirror the real UART.
class MuxPort : public uart::UARTComponent {
 public:
  void write_array(const uint8_t *data, size_t len) override;
  bool peek_byte(uint8_t *data) override;
  bool read_array(uint8_t *data, size_t len) override;
  size_t available() override { return this->rx_.size(); }
  uart::UARTFlushResult flush() override;
#if defined(USE_ESP8266) || defined(USE_ESP32)
  void load_settings(bool dump_config) override {}  // the real UART owns the line settings
#endif

  void set_mux(RS485Mux *mux) { this->mux_ = mux; }
  void push(const uint8_t *data, size_t len);
  void clear() { this->rx_.clear(); }

 protected:
  void check_logger_conflict() override {}

  static constexpr size_t MAX_RX = 1024;
  RS485Mux *mux_{nullptr};
  std::deque<uint8_t> rx_;
};

class RS485Mux : public Component {
 public:
  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_ports(MuxPort *modbus, MuxPort *stream);
  void set_idle_timeout(uint32_t ms) { this->idle_.set_timeout(ms); }

  void setup() override;
  void loop() override;
  void dump_config() override;
  // After the UART (BUS) and before modbus (BUS - 1), whose setup() divides by the port's baud rate.
  float get_setup_priority() const override { return setup_priority::BUS - 0.5f; }

  void set_passthrough(bool on);
  bool passthrough() const { return this->passthrough_; }
  void set_client_connected(bool connected) { this->client_ = connected; }
  // True once passthrough has been on for the idle timeout with no TCP client; the YAML then turns it off.
  bool idle_expired() { return this->idle_.update(this->passthrough_, this->client_, millis()); }

  bool is_active(const MuxPort *port) const { return port == (this->passthrough_ ? this->stream_ : this->modbus_); }
  uart::UARTComponent *uart() { return this->uart_; }

 protected:
  void sync_settings_(MuxPort *port);

  uart::UARTComponent *uart_{nullptr};
  MuxPort *modbus_{nullptr};
  MuxPort *stream_{nullptr};
  ::rs485_mux::IdleTimer idle_{600000};
  bool passthrough_{false};
  bool client_{false};
};

}  // namespace rs485_mux
}  // namespace esphome
```

Note the two namespaces: the host-testable `IdleTimer` lives in the top-level `::rs485_mux`, the ESPHome classes in `esphome::rs485_mux`.

- [ ] **Step 2: Write the implementation** — `esphome/components/rs485_mux/rs485_mux.cpp`:

```cpp
#include "rs485_mux.h"

#include "esphome/core/log.h"

namespace esphome {
namespace rs485_mux {

static const char *const TAG = "rs485_mux";

void MuxPort::write_array(const uint8_t *data, size_t len) {
  if (this->mux_ != nullptr && this->mux_->is_active(this)) this->mux_->uart()->write_array(data, len);
}

bool MuxPort::peek_byte(uint8_t *data) {
  if (this->rx_.empty()) return false;
  *data = this->rx_.front();
  return true;
}

bool MuxPort::read_array(uint8_t *data, size_t len) {
  if (this->rx_.size() < len) return false;
  for (size_t i = 0; i < len; i++) {
    data[i] = this->rx_.front();
    this->rx_.pop_front();
  }
  return true;
}

uart::UARTFlushResult MuxPort::flush() {
  if (this->mux_ != nullptr && this->mux_->is_active(this)) return this->mux_->uart()->flush();
  return uart::UARTFlushResult::UART_FLUSH_RESULT_ASSUMED_SUCCESS;
}

void MuxPort::push(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (this->rx_.size() >= MAX_RX) this->rx_.pop_front();  // keep the newest bytes
    this->rx_.push_back(data[i]);
  }
}

void RS485Mux::set_ports(MuxPort *modbus, MuxPort *stream) {
  this->modbus_ = modbus;
  this->stream_ = stream;
  modbus->set_mux(this);
  stream->set_mux(this);
  // Also here, not only in setup(): the ports must never report baud 0 to a user that sets up first.
  if (this->uart_ != nullptr) {
    this->sync_settings_(modbus);
    this->sync_settings_(stream);
  }
}

void RS485Mux::sync_settings_(MuxPort *port) {
  port->set_baud_rate(this->uart_->get_baud_rate());
  port->set_data_bits(this->uart_->get_data_bits());
  port->set_stop_bits(this->uart_->get_stop_bits());
  port->set_parity(this->uart_->get_parity());
  port->set_rx_buffer_size(this->uart_->get_rx_buffer_size());
}

void RS485Mux::setup() {
  this->sync_settings_(this->modbus_);
  this->sync_settings_(this->stream_);
}

void RS485Mux::loop() {
  // The baud select changes the real UART at run time; modbus reads get_baud_rate() for its frame timing.
  const uint32_t baud = this->uart_->get_baud_rate();
  if (this->modbus_->get_baud_rate() != baud) {
    this->sync_settings_(this->modbus_);
    this->sync_settings_(this->stream_);
  }
  MuxPort *active = this->passthrough_ ? this->stream_ : this->modbus_;
  uint8_t buf[64];
  size_t n;
  while ((n = std::min(this->uart_->available(), sizeof(buf))) > 0) {
    if (!this->uart_->read_array(buf, n)) break;
    active->push(buf, n);
  }
}

void RS485Mux::set_passthrough(bool on) {
  if (on == this->passthrough_) return;
  this->passthrough_ = on;
  // A half-received frame must not cross over.
  this->modbus_->clear();
  this->stream_->clear();
  ESP_LOGI(TAG, "RS485 line -> %s", on ? "TCP passthrough" : "Modbus polling");
}

void RS485Mux::dump_config() {
  ESP_LOGCONFIG(TAG, "RS485 mux: line to %s", this->passthrough_ ? "TCP passthrough" : "Modbus polling");
}

}  // namespace rs485_mux
}  // namespace esphome
```

Add `#include <algorithm>` at the top of the .cpp (for `std::min`).

- [ ] **Step 3: Write the codegen** — `esphome/components/rs485_mux/__init__.py`:

```python
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_UART_ID

DEPENDENCIES = ["uart"]

CONF_MODBUS_PORT = "modbus_port"
CONF_PASSTHROUGH_PORT = "passthrough_port"
CONF_IDLE_TIMEOUT = "idle_timeout"

rs485_mux_ns = cg.esphome_ns.namespace("rs485_mux")
RS485Mux = rs485_mux_ns.class_("RS485Mux", cg.Component)
# A UARTComponent subclass, so other components accept its ID as their uart_id.
MuxPort = rs485_mux_ns.class_("MuxPort", uart.UARTComponent)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(RS485Mux),
            cv.Required(CONF_MODBUS_PORT): cv.Schema({cv.Required(CONF_ID): cv.declare_id(MuxPort)}),
            cv.Required(CONF_PASSTHROUGH_PORT): cv.Schema({cv.Required(CONF_ID): cv.declare_id(MuxPort)}),
            cv.Optional(CONF_IDLE_TIMEOUT, default="10min"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_uart(await cg.get_variable(config[CONF_UART_ID])))  # before set_ports: it syncs the settings
    modbus = cg.new_Pvariable(config[CONF_MODBUS_PORT][CONF_ID])
    stream = cg.new_Pvariable(config[CONF_PASSTHROUGH_PORT][CONF_ID])
    cg.add(var.set_ports(modbus, stream))
    cg.add(var.set_idle_timeout(config[CONF_IDLE_TIMEOUT]))
```

- [ ] **Step 4: Rewire the bridge** — replace `esphome/oem-bridge.yaml` with:

```yaml
# Waveshare ESP32-S3-RS485-CAN as the OEM charger's bridge (spec phase 4 and
# docs/history/superpowers/specs/2026-10-15-oem-bridge-home-assistant-design.md).
# Home Assistant owns the RS485 line: modbus_controller polls the charger. The "Passthrough" switch hands the line to
# the raw TCP stream on port 8899 instead, for tools/oem_modbus.py and tools/oem_update.py (--bridge), which flip it
# themselves over the web server (basic auth). It turns itself off after 10 min without a TCP client, and turning it
# off puts the line back to 9600. CAN is not used: the OEM board owns the module bus.
esphome:
  name: oem-bridge
  friendly_name: OEM Bridge

esp32:
  board: esp32-s3-devkitc-1
  flash_size: 16MB
  framework:
    type: esp-idf

external_components:
  - source: github://oxan/esphome-stream-server
    components: [stream_server]
  - source:
      type: local
      path: components
    components: [rs485_mux]

logger:
  level: INFO

api:
  encryption:
    key: !secret api_key
  reboot_timeout: 0s

ota:
  - platform: esphome
    password: !secret ota_password

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password
  domain: !secret domain
  ap: {}

# The tools switch Passthrough and the baud here: POST /switch/Passthrough/turn_on, /select/RS485%20baud/set?option=...
web_server:
  port: 80
  version: 3
  auth:
    username: !secret web_username
    password: !secret web_password

uart:
  - id: rs485
    tx_pin: GPIO17
    rx_pin: GPIO18
    flow_control_pin: GPIO21  # the transceiver's direction enable (RS485 half-duplex mode)
    baud_rate: 9600
    rx_buffer_size: 1024

rs485_mux:
  id: mux
  uart_id: rs485
  modbus_port:
    id: mux_modbus
  passthrough_port:
    id: mux_stream
  idle_timeout: 10min

stream_server:
  uart_id: mux_stream
  port: 8899
  buffer_size: 1024

modbus:
  id: modbus_bus
  uart_id: mux_modbus

modbus_controller:
  - id: charger
    modbus_id: modbus_bus
    address: 0x1
    update_interval: 5s
    offline_skip_updates: 2

script:
  - id: passthrough_off
    then:
      - select.set:
          id: rs485_baud
          option: "9600"
      - lambda: id(mux).set_passthrough(false);
      - component.resume: charger

switch:
  - platform: template
    id: passthrough
    name: Passthrough
    icon: mdi:lan-connect
    restore_mode: ALWAYS_OFF
    lambda: return id(mux).passthrough();
    turn_on_action:
      - component.suspend: charger
      - lambda: id(mux).set_passthrough(true);
    turn_off_action:
      - script.execute: passthrough_off

interval:
  - interval: 1s
    then:
      - if:
          condition:
            lambda: return id(mux).idle_expired();
          then:
            - logger.log: "Passthrough idle for the timeout: back to Modbus polling"
            - script.execute: passthrough_off

select:
  - platform: template
    id: rs485_baud
    name: RS485 baud
    options: ["9600", "19200", "38400", "115200", "460800"]
    initial_option: "9600"
    optimistic: true
    set_action:
      - lambda: |-
          id(rs485).set_baud_rate(atoi(x.c_str()));
          id(rs485).load_settings(false);

sensor:
  - platform: wifi_signal
    name: WiFi signal
    update_interval: 60s
  - platform: uptime
    name: Uptime
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Charger firmware
    register_type: read
    address: 0
    value_type: U_WORD
    accuracy_decimals: 0
    entity_category: diagnostic

binary_sensor:
  - platform: stream_server
    connected:
      name: Bridge client connected
      on_state:
        - lambda: id(mux).set_client_connected(x);
```

(Task 3 replaces the single "Charger firmware" sensor with the full entity set.)

- [ ] **Step 5: Compile**

Run: `cd esphome && ~/.local/bin/esphome compile oem-bridge.yaml 2>&1 | tail -5`
Expected: `[SUCCESS]` / `Successfully compiled program.` If it fails on missing `web_username`/`web_password`, stop and report NEEDS_CONTEXT (the user adds them; never edit or print secrets.yaml). For other errors, fix the component, not the YAML's intent.

- [ ] **Step 6: Run the host tests** (nothing should change): `make -C tests test` → all `ok`.

- [ ] **Step 7: Commit**

```bash
git add esphome/components/rs485_mux esphome/oem-bridge.yaml
git commit -m "rs485_mux: share the bridge's RS485 line between Modbus polling and the TCP passthrough"
```
(with the attribution lines)

---

### Task 3: Home Assistant entities

**Files:**
- Modify: `esphome/oem-bridge.yaml` (replace the `sensor:` and `binary_sensor:` blocks; add `text_sensor:`, `number:`, `button:`; add one select and one switch to the existing lists)

**Interfaces:**
- Consumes: `charger` (modbus_controller), `mux`, `passthrough`, `rs485_baud` from Task 2.
- Produces: nothing used later.

Register map (spec §4 and `docs/history/superpowers/specs/2026-10-14-oem-board-firmware-design.md` §4): input registers via `register_type: read`, holding via `register_type: holding`. Not-available reads as 0xFFFF (unsigned) or 0x7FFF (signed); publish NAN for them.

- [ ] **Step 1: Replace the `sensor:` block** with:

```yaml
sensor:
  - platform: wifi_signal
    name: WiFi signal
    update_interval: 60s
  - platform: uptime
    name: Uptime
  # --- status, IR 0-17 ---
  - platform: modbus_controller
    modbus_controller_id: charger
    id: ir_version
    internal: true
    register_type: read
    address: 0
    value_type: U_WORD
    on_value:
      - lambda: |-
          char s[8];
          snprintf(s, sizeof(s), "%u.%u", (unsigned) x >> 8, (unsigned) x & 0xFF);
          id(fw_version).publish_state(s);
  - platform: modbus_controller
    modbus_controller_id: charger
    id: ir_state
    internal: true
    register_type: read
    address: 1
    value_type: U_WORD
    on_value:
      - lambda: |-
          static const char *const N[] = {"BOOT", "WAIT_BMS", "CHARGING", "HOLD", "FAULT"};
          id(state_text).publish_state(x < 5 ? N[(int) x] : "unknown");
  - platform: modbus_controller
    modbus_controller_id: charger
    id: ir_hold
    internal: true
    register_type: read
    address: 2
    value_type: U_WORD
    on_value:
      - lambda: |-
          static const char *const N[] = {"none", "paused", "not allowed", "BMS alarm", "BMS blocking",
                                          "cell high", "budget < 2 A", "waiting for start"};
          id(hold_text).publish_state(x < 8 ? N[(int) x] : "unknown");
  - platform: modbus_controller
    modbus_controller_id: charger
    id: ir_error
    internal: true
    register_type: read
    address: 3
    value_type: U_WORD
    on_value:
      - lambda: |-
          const int e = (int) x;
          const char *what = e == 0 ? "none" : e <= 11 ? "module alarm" : e == 12 ? "BMS alarm"
                           : e == 13 ? "setpoint mismatch" : e == 14 ? "output overvoltage"
                           : e == 15 ? "source stale (warning)" : e == 16 ? "charging without current (warning)"
                           : e == 17 ? "battery reading implausible" : "unknown";
          char s[48];
          if (e == 0) snprintf(s, sizeof(s), "none");
          else snprintf(s, sizeof(s), "E%d %s", e, what);
          id(error_text).publish_state(s);
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Target voltage
    register_type: read
    address: 6
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 2
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.01f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Target current
    register_type: read
    address: 7
    value_type: U_WORD
    unit_of_measurement: A
    device_class: current
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Current budget
    register_type: read
    address: 8
    value_type: U_WORD
    unit_of_measurement: A
    device_class: current
    state_class: measurement
    accuracy_decimals: 1
    entity_category: diagnostic
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Output current
    register_type: read
    address: 9
    value_type: U_WORD
    unit_of_measurement: A
    device_class: current
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Output power
    register_type: read
    address: 10
    value_type: U_WORD
    unit_of_measurement: W
    device_class: power
    state_class: measurement
    accuracy_decimals: 0
    filters:
      - lambda: return x == 65535 ? NAN : x;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Battery voltage
    register_type: read
    address: 11
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 2
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.01f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Battery voltage ADC
    register_type: read
    address: 12
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 2
    entity_category: diagnostic
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.01f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: CVL in use
    register_type: read
    address: 14
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    accuracy_decimals: 2
    entity_category: diagnostic
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.01f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: CCL in use
    register_type: read
    address: 15
    value_type: U_WORD
    unit_of_measurement: A
    device_class: current
    accuracy_decimals: 1
    entity_category: diagnostic
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Charger uptime
    register_type: read
    address: 16
    value_type: U_DWORD
    unit_of_measurement: s
    device_class: duration
    accuracy_decimals: 0
    entity_category: diagnostic
  # --- per module: PSU1 at IR 20, PSU2 at IR 30 ---
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 voltage
    register_type: read
    address: 21
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 2
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.01f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 current
    register_type: read
    address: 22
    value_type: U_WORD
    unit_of_measurement: A
    device_class: current
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 AC voltage
    register_type: read
    address: 23
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 heatsink
    register_type: read
    address: 25
    value_type: S_WORD
    unit_of_measurement: °C
    device_class: temperature
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 32767 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    id: psu1_alarm_raw
    internal: true
    register_type: read
    address: 28
    value_type: U_DWORD
    on_value:
      - lambda: |-
          char s[12];
          snprintf(s, sizeof(s), "%08X", (unsigned) x);
          id(psu1_alarm).publish_state(s);
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 voltage
    register_type: read
    address: 31
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 2
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.01f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 current
    register_type: read
    address: 32
    value_type: U_WORD
    unit_of_measurement: A
    device_class: current
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 AC voltage
    register_type: read
    address: 33
    value_type: U_WORD
    unit_of_measurement: V
    device_class: voltage
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 65535 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 heatsink
    register_type: read
    address: 35
    value_type: S_WORD
    unit_of_measurement: °C
    device_class: temperature
    state_class: measurement
    accuracy_decimals: 1
    filters:
      - lambda: return x == 32767 ? NAN : x * 0.1f;
  - platform: modbus_controller
    modbus_controller_id: charger
    id: psu2_alarm_raw
    internal: true
    register_type: read
    address: 38
    value_type: U_DWORD
    on_value:
      - lambda: |-
          char s[12];
          snprintf(s, sizeof(s), "%08X", (unsigned) x);
          id(psu2_alarm).publish_state(s);
  # --- read-only link settings, HR 7-8 (every 30 s) ---
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Modbus address
    register_type: holding
    address: 7
    value_type: U_WORD
    skip_updates: 5
    accuracy_decimals: 0
    entity_category: diagnostic
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Modbus baud code
    register_type: holding
    address: 8
    value_type: U_WORD
    skip_updates: 5
    accuracy_decimals: 0
    entity_category: diagnostic
```

- [ ] **Step 2: Add `text_sensor:`** (new top-level block):

```yaml
text_sensor:
  - platform: template
    id: fw_version
    name: Charger firmware
    entity_category: diagnostic
  - platform: template
    id: state_text
    name: Charger state
  - platform: template
    id: hold_text
    name: Hold reason
  - platform: template
    id: error_text
    name: Error
  - platform: template
    id: psu1_alarm
    name: PSU1 alarm word
    entity_category: diagnostic
  - platform: template
    id: psu2_alarm
    name: PSU2 alarm word
    entity_category: diagnostic
```

- [ ] **Step 3: Replace the `binary_sensor:` block** with:

```yaml
binary_sensor:
  - platform: stream_server
    connected:
      name: Bridge client connected
      on_state:
        - lambda: id(mux).set_client_connected(x);
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Cooling
    register_type: read
    address: 4
    bitmask: 0x1
  - platform: modbus_controller
    modbus_controller_id: charger
    name: No-current warning
    register_type: read
    address: 4
    bitmask: 0x2
    device_class: problem
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Session active
    register_type: read
    address: 4
    bitmask: 0x8
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Relay closed
    register_type: read
    address: 4
    bitmask: 0x10
  - platform: modbus_controller
    modbus_controller_id: charger
    name: AC present
    register_type: read
    address: 4
    bitmask: 0x20
    device_class: power
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Diagnostic override
    register_type: read
    address: 4
    bitmask: 0x100
    entity_category: diagnostic
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 online
    register_type: read
    address: 20
    bitmask: 0x1
    device_class: connectivity
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 faulted
    register_type: read
    address: 20
    bitmask: 0x2
    device_class: problem
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 cooling
    register_type: read
    address: 20
    bitmask: 0x4
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU1 standby
    register_type: read
    address: 20
    bitmask: 0x8
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 online
    register_type: read
    address: 30
    bitmask: 0x1
    device_class: connectivity
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 faulted
    register_type: read
    address: 30
    bitmask: 0x2
    device_class: problem
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 cooling
    register_type: read
    address: 30
    bitmask: 0x4
  - platform: modbus_controller
    modbus_controller_id: charger
    name: PSU2 standby
    register_type: read
    address: 30
    bitmask: 0x8
```

- [ ] **Step 4: Add `number:`** (new top-level block). All holding, FC 06, read every 30 s (`skip_updates: 5` at a 5 s interval). `multiply` scales the written value and divides the read one. Ranges are `HR_RANGES` in `oem/src/app/settings.h`:

```yaml
number:
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Max current
    address: 0
    value_type: U_WORD
    min_value: 2
    max_value: 100
    step: 1
    unit_of_measurement: A
    mode: box
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Charge voltage
    address: 1
    value_type: U_WORD
    multiply: 100
    min_value: 47.0
    max_value: 57.8
    step: 0.01
    unit_of_measurement: V
    mode: box
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Start SOC
    address: 2
    value_type: U_WORD
    min_value: 0
    max_value: 99
    step: 1
    unit_of_measurement: "%"
    mode: box
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Stop SOC
    address: 3
    value_type: U_WORD
    min_value: 1
    max_value: 100
    step: 1
    unit_of_measurement: "%"
    mode: box
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Start voltage
    address: 4
    value_type: U_WORD
    multiply: 100
    min_value: 42.0
    max_value: 52.8
    step: 0.01
    unit_of_measurement: V
    mode: box
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Stop voltage
    address: 5
    value_type: U_WORD
    multiply: 100
    min_value: 47.0
    max_value: 57.8
    step: 0.01
    unit_of_measurement: V
    mode: box
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Fallback current
    address: 9
    value_type: U_WORD
    min_value: 0
    max_value: 20
    step: 1
    unit_of_measurement: A
    mode: box
    entity_category: config
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Backlight timeout
    address: 11
    value_type: U_WORD
    min_value: 0
    max_value: 3600
    step: 1
    unit_of_measurement: s
    mode: box
    entity_category: config
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Max cell voltage
    address: 12
    value_type: U_WORD
    multiply: 1000
    min_value: 3.4
    max_value: 3.58
    step: 0.001
    unit_of_measurement: V
    mode: box
    entity_category: config
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Cell resume voltage
    address: 13
    value_type: U_WORD
    multiply: 1000
    min_value: 3.2
    max_value: 3.55
    step: 0.001
    unit_of_measurement: V
    mode: box
    entity_category: config
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: ADC trim
    address: 14
    value_type: S_WORD
    min_value: -50
    max_value: 50
    step: 1
    unit_of_measurement: "‰"
    mode: box
    entity_category: config
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Module fallback voltage
    address: 15
    value_type: U_WORD
    multiply: 100
    min_value: 48.0
    max_value: 54.4
    step: 0.01
    unit_of_measurement: V
    mode: box
    entity_category: config
    skip_updates: 5
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Module fallback current
    address: 16
    value_type: U_WORD
    min_value: 0
    max_value: 10
    step: 1
    unit_of_measurement: A
    mode: box
    entity_category: config
    skip_updates: 5
```

- [ ] **Step 5: Add the selects** to the existing `select:` list (after `rs485_baud`):

```yaml
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Source mode
    address: 6
    value_type: U_WORD
    skip_updates: 5
    entity_category: config
    optionsmap:
      "Panel": 0
      "Pylontech": 1
      "Modbus": 2
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Relay mode
    address: 10
    value_type: U_WORD
    skip_updates: 5
    entity_category: config
    optionsmap:
      "Auto": 0
      "Off": 1
      "Modbus": 2
```

- [ ] **Step 6: Add the Pause switch** to the existing `switch:` list (after `passthrough`). A holding-register switch writes `0xFFFF & bitmask`, so `bitmask: 1` writes 1 / 0:

```yaml
  - platform: modbus_controller
    modbus_controller_id: charger
    name: Pause
    register_type: holding
    address: 110
    bitmask: 1
    icon: mdi:pause-circle
```

- [ ] **Step 7: Add `button:`** (new top-level block):

```yaml
button:
  - platform: template
    name: Charge now
    icon: mdi:battery-charging
    on_press:
      - lambda: |-
          auto cmd = modbus_controller::ModbusCommandItem::create_write_single_command(id(charger), 113, 1);
          id(charger)->queue_command(cmd);
  - platform: template
    name: Clear fault
    icon: mdi:alert-remove
    on_press:
      - lambda: |-
          auto cmd = modbus_controller::ModbusCommandItem::create_write_single_command(id(charger), 111, 1);
          id(charger)->queue_command(cmd);
```

- [ ] **Step 8: Compile**

Run: `cd esphome && ~/.local/bin/esphome compile oem-bridge.yaml 2>&1 | tail -5`
Expected: success. If `create_write_single_command`'s signature differs in 2026.8.1, read
`~/.local/pipx/venvs/esphome/lib/python3.14/site-packages/esphome/components/modbus_controller/modbus_controller.h` (around `create_write_single_command`) and adapt the two button lambdas only. If a key is rejected (for example `skip_updates` on a platform), check that platform's `__init__.py` in the same directory and adapt; keep the register, scale and range.

- [ ] **Step 9: Commit**

```bash
git add esphome/oem-bridge.yaml
git commit -m "OEM bridge: Home Assistant entities for status, modules, settings, pause and commands"
```
(with the attribution lines)

---

### Task 4: Tools turn Passthrough on/off around --bridge, with basic auth

**Files:**
- Modify: `tools/oem_modbus.py` (`set_bridge_baud`, new helpers, `main`)
- Modify: `tools/oem_update.py` (`main`)
- Test: `tools/test_oem_modbus.py`, `tools/test_oem_update.py`

**Interfaces:**
- Consumes: the bridge's web endpoints `POST /switch/Passthrough/turn_on|turn_off` and `POST /select/RS485%20baud/set?option=N` (Task 2).
- Produces: `oem_modbus.auth_header(env=None) -> dict`, `oem_modbus.bridge_post(url, env=None)`, `oem_modbus.set_bridge_passthrough(host, on, post=None)`, `oem_modbus.bridge_session(host, post=None)` (context manager: on at entry, off at exit, also on exceptions). `set_bridge_baud(host, baud, post=None)` keeps its signature.

- [ ] **Step 1: Write the failing tests** — append to `tools/test_oem_modbus.py`:

```python
def test_set_bridge_passthrough_posts_to_switch():
    calls = []
    om.set_bridge_passthrough("10.0.0.5", True, post=calls.append)
    om.set_bridge_passthrough("10.0.0.5", False, post=calls.append)
    assert calls == ["http://10.0.0.5/switch/Passthrough/turn_on", "http://10.0.0.5/switch/Passthrough/turn_off"]


def test_bridge_session_turns_passthrough_off_even_on_error():
    calls = []
    with om.bridge_session("h", post=calls.append):
        assert calls == ["http://h/switch/Passthrough/turn_on"]
    assert calls[-1] == "http://h/switch/Passthrough/turn_off"
    calls.clear()
    with pytest.raises(RuntimeError):
        with om.bridge_session("h", post=calls.append):
            raise RuntimeError("tool failed")
    assert calls == ["http://h/switch/Passthrough/turn_on", "http://h/switch/Passthrough/turn_off"]


def test_auth_header_from_env():
    assert om.auth_header({}) == {}
    h = om.auth_header({"OEM_BRIDGE_USER": "u", "OEM_BRIDGE_PASSWORD": "p"})
    assert h == {"Authorization": "Basic dTpw"}  # base64("u:p")


def test_main_bridge_wraps_the_run_in_passthrough(monkeypatch):
    events = []
    monkeypatch.setattr(om, "resolve_bridge", lambda h: "10.0.0.5")
    monkeypatch.setattr(om, "set_bridge_passthrough", lambda host, on: events.append(("pass", host, on)))

    class Port:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            events.append(("close",))

    monkeypatch.setattr(om, "open_port", lambda serial, name, baud: events.append(("open", name)) or Port())
    monkeypatch.setattr(om, "run", lambda port, a: events.append(("run", a.cmd)))
    om.main(["--bridge", "oem-bridge.local", "status"])
    assert events == [("pass", "10.0.0.5", True), ("open", "socket://10.0.0.5:8899"), ("run", "status"),
                      ("close",), ("pass", "10.0.0.5", False)]
```

And in `tools/test_oem_update.py`, in `test_bridge_switches_baud_around_the_install`, add right after the `set_bridge_baud` monkeypatch:

```python
    monkeypatch.setattr(ou, "resolve_bridge", lambda h: h)
    monkeypatch.setattr(ou, "set_bridge_passthrough", lambda host, on: events.append(("pass", host, on)))
```

and replace the expected `events` list with:

```python
    assert events == [("pass", "br", True), ("baud", "br", 9600), ("open", url, 9600), ("reboot",), ("close", url),
                      ("baud", "br", 460800), ("open", url, 460800), ("install", True), ("close", url),
                      ("baud", "br", 9600), ("pass", "br", False)]
```

- [ ] **Step 2: Run them to verify they fail**

Run: `.venv/bin/python -m pytest -q tools/test_oem_modbus.py tools/test_oem_update.py 2>&1 | tail -5`
Expected: failures for the missing `set_bridge_passthrough`, `bridge_session`, `auth_header`, and the changed event list.

- [ ] **Step 3: Implement in `tools/oem_modbus.py`.** Replace the existing `set_bridge_baud` function with:

```python
def auth_header(env=None):
    """HTTP basic auth for the bridge's web server, from OEM_BRIDGE_USER / OEM_BRIDGE_PASSWORD (empty if unset)."""
    import base64
    import os
    env = os.environ if env is None else env
    user, password = env.get("OEM_BRIDGE_USER"), env.get("OEM_BRIDGE_PASSWORD")
    if not user or password is None:
        return {}
    return {"Authorization": "Basic " + base64.b64encode(f"{user}:{password}".encode()).decode()}


def bridge_post(url, env=None):
    import urllib.request
    req = urllib.request.Request(url, data=b"", method="POST", headers=auth_header(env))
    urllib.request.urlopen(req, timeout=5).read()


def set_bridge_baud(host, baud, post=None):
    """Switches the WiFi bridge's RS485 line rate (its "RS485 baud" select, through the ESPHome web server)."""
    (post or bridge_post)(f"http://{host}/select/RS485%20baud/set?option={baud}")  # ESPHome addresses entities by name


def set_bridge_passthrough(host, on, post=None):
    """Hands the bridge's RS485 line to the TCP stream (on) or back to its Modbus polling (off)."""
    (post or bridge_post)(f"http://{host}/switch/Passthrough/turn_{'on' if on else 'off'}")


@contextlib.contextmanager
def bridge_session(host, post=None):
    """Passthrough on for the block, off afterwards, also when the block fails."""
    set_bridge_passthrough(host, True, post)
    try:
        yield
    finally:
        set_bridge_passthrough(host, False, post)
```

Add `import contextlib` to the module's imports. Then change `main()` so the bridge case wraps the port in passthrough. Replace the body after `a = build_parser().parse_args(argv)` with:

```python
    if a.bridge:
        a.bridge = resolve_bridge(a.bridge)
        set_bridge_passthrough(a.bridge, True)
    try:
        try:
            port = open_port(serial, port_name(a), a.baud)
        except serial.SerialException as e:
            print(f"error: cannot open {port_name(a)}: {e}", file=sys.stderr)
            sys.exit(3)
        with port:
            try:
                run(port, a)
            except ModbusException as e:
                sys.exit(f"{e}")
            except ModbusError as e:
                print(f"error: {e}", file=sys.stderr)
                sys.exit(2)
    finally:
        if a.bridge:
            set_bridge_passthrough(a.bridge, False)
```

(`set_bridge_passthrough` is looked up at call time, so the test's monkeypatch applies; `bridge_session` is for `oem_update.py`, which has more steps.) The test monkeypatches `open_port` with a 3-argument lambda that ignores `serial`; `main` still imports `serial` at its top as today.

- [ ] **Step 4: Implement in `tools/oem_update.py`.** Add `set_bridge_passthrough` to the `from oem_modbus import (...)` list. In `main()`, right after `name = bridge_url(a.bridge) if a.bridge else a.port` and `dat = ...`, wrap everything that follows (the `check_image` / Modbus-reboot `try` and the install `try`/`finally`) so passthrough is on for the whole sequence and off last:

```python
    if a.bridge:
        set_bridge_passthrough(a.bridge, True)  # the bridge's Modbus poller stays off the line throughout
    try:
        # ... the existing check_image / --modbus-reboot try-block, unchanged ...
        # ... the existing install try/finally (ending with set_bridge_baud(a.bridge, a.modbus_baud)), unchanged ...
    finally:
        if a.bridge:
            set_bridge_passthrough(a.bridge, False)
```

Keep the existing code inside exactly as it is; only indent it into the new `try`. The `sys.exit` calls inside raise `SystemExit`, so the outer `finally` still runs.

- [ ] **Step 5: Run the tests**

Run: `.venv/bin/python -m pytest -q 2>&1 | tail -3`
Expected: all pass (82 before this task, 86 after).

- [ ] **Step 6: Commit**

```bash
git add tools/oem_modbus.py tools/oem_update.py tools/test_oem_modbus.py tools/test_oem_update.py
git commit -m "Bridge tools: turn the bridge's Passthrough on/off around --bridge use; basic auth from env"
```
(with the attribution lines)

---

### Task 5: README

**Files:**
- Modify: `README.md`, section `### Remote access through the WiFi bridge`

**Interfaces:** none.

- [ ] **Step 1: Rewrite the section's opening.** Replace the bullet list and paragraph that begin with "`esphome/oem-bridge.yaml` turns the Waveshare ESP32-S3-RS485-CAN into a WiFi bridge" (up to, not including, "Both tools take `--bridge HOST`") with:

```markdown
`esphome/oem-bridge.yaml` turns the Waveshare ESP32-S3-RS485-CAN into the charger's bridge:
- Wire its RS485 A+/B− to the jack's A/B (as the FTDI adapter). Leave its CAN unconnected; the OEM board owns the module bus.
- The RS485 transceiver is on GPIO17 (TX), GPIO18 (RX) and GPIO21 (direction).
- **Home Assistant owns the line.** The bridge polls the charger over Modbus every 5 s (settings every 30 s) and exposes
  the state, hold reason, error, voltages, currents, power, flags, per-module data, all settings except the Modbus
  address and baud, a Pause switch, and Charge now / Clear fault buttons.
- **Passthrough.** A "Passthrough" switch hands the line to raw TCP on port 8899 instead. The tools below flip it
  themselves; it turns itself off after 10 minutes without a TCP client, and turning it off puts the line back to
  9600. With it off, a TCP client neither hears nor reaches the charger.
- The web server (port 80, which takes the Passthrough and "RS485 baud" requests) needs `web_username` and
  `web_password` in `esphome/secrets.yaml`. The tools read them from `OEM_BRIDGE_USER` and `OEM_BRIDGE_PASSWORD`.
- Line sharing is done by the local `rs485_mux` component (`esphome/components/rs485_mux`).
- Flash it once over USB: `esphome run esphome/oem-bridge.yaml`. After that, use OTA.
```

Also delete the paragraph or line, if present in that section, that says the bridge has no authentication.

- [ ] **Step 2: Check nothing else in the README contradicts it.** Run `grep -n "no authentication\|8899" README.md` and make each hit consistent with the new text.

- [ ] **Step 3: Commit**

```bash
git add README.md
git commit -m "README: the bridge's Home Assistant entities, Passthrough switch and web auth"
```
(with the attribution lines)

---

### Live verification (controller, after the final review)

Not a subagent task: needs the OTA flash, the user's credentials in the environment, and the charger in the garage.

1. `cd esphome && ~/.local/bin/esphome upload oem-bridge.yaml --device oem-bridge.local` (OTA).
2. Without credentials: `curl -s -o /dev/null -w "%{http_code}" -X POST http://oem-bridge.local/switch/Passthrough/turn_on` → 401.
3. With Passthrough off: a raw `socket://<ip>:8899` FC04 request gets no reply.
4. HA / web UI values against `oem_modbus.py --bridge oem-bridge.local status` taken right after.
5. A setting write from HA (e.g. Max current 3 → 4 → 3) reads back over the CLI; Stop voltage above Charge voltage is refused and snaps back.
6. Pause from HA shows in `status`; un-pause; Charge now starts a session.
7. `oem_modbus.py --bridge` with polling on: Passthrough flips on for the run and off after; polling resumes.
8. Passthrough on by hand, no client: off by itself after 10 min.
9. `oem_update.py --bridge oem-bridge.local --modbus-reboot oem/build/charger.dat` (same 3.3 image) with polling on: installs; polling resumes.
