# OEM Modbus Limit Source Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Source mode 2 (Modbus): the charger takes CVL/CCL/flags/SOC/V/cell from HR 101-107, snapshotted on each
HR 100 heartbeat, with the Pylontech-style stale fallback; the WiFi bridge relays YamBMS's limits from MQTT into them.

**Architecture:** A pure `ModbusSource` (header-only, like `PylonSource`) owned by `OemRegisters`, fed on every block
write that starts at HR 100. `main.cpp` picks Panel / Pylontech / Modbus limits by HR 6 and clears the stop latch on a
mode change. The bridge YAML gets MQTT sensors and one FC 16 write of HR 100-107 every 5 s.

**Tech Stack:** C++17 header-only app code, host tests via `make -C tests`, firmware via CMake/arm-none-eabi-gcc
(`cmake --build oem/build` — check the existing build dir/instructions in README "Building"), ESPHome 2026.8.1 YAML.

**Spec:** `docs/history/superpowers/specs/2026-10-15-oem-modbus-source-design.md`

## Global Constraints

- Firmware version 3.6: `OEM_FW_VERSION = 0x0306`.
- `EXT_FRESH_MS` (10000) is the Modbus freshness window; it already exists in `modbus_map.h`.
- Stale fallback = Panel limits, ccl = min(Max current, HR 9 Fallback current), `stale` true, `fresh` true,
  `allowed = !stopped_`, `source = SourceMode::MODBUS`.
- HR 6 over Modbus: 0 and 2 accepted, changing to 1 refused (`MB_EX_ILLEGAL_VALUE`).
- Never run `esphome config` and never print, cat or grep `esphome/secrets.yaml`. Use `esphome compile` only.
- No Eco-Worthy / EC100 name in `docs/oem-controls.md`.
- Commit trailer on every commit:
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Q88Pr4eMBeeEdBYp5fsSSu
  ```
- Do not push, flash or talk to the chargers/bridges on the network.

---

### Task 1: `ModbusSource` unit

**Files:**
- Create: `oem/src/app/modbus_source.h`
- Create: `tests/test_oem_modbus_source.cpp`
- Modify: `tests/Makefile` (add `test_oem_modbus_source` to `TESTS`)
- Modify: `.gitignore` (add the `tests/test_oem_modbus_source` binary next to the other test binaries)
- Modify: `oem/src/app/limit_source.h` (header comment: "Modbus is in modbus_source.h")

**Interfaces:**
- Produces: `oem::ModbusSource` with `void heartbeat(const uint16_t *ext, uint32_t now)` (ext = HR 100..107, 8 words),
  `void forget_stop()`, `bool fresh(uint32_t now) const`, `Limits limits(const Settings &s, uint32_t now) const`;
  constants `EXT_UNKNOWN_I16 = 0x7FFF`. Uses `EXT_FRESH_MS` — move that constant (and nothing else) from
  `modbus_map.h` into `modbus_source.h` so both see it (`modbus_map.h` will include `modbus_source.h` in Task 2; for
  now define it in `modbus_source.h` and delete it from `modbus_map.h`, adding `#include "app/modbus_source.h"` to
  `modbus_map.h` so it still compiles).

- [ ] **Step 1: Write the failing test** `tests/test_oem_modbus_source.cpp`:

```cpp
#include <cmath>

#include "app/modbus_source.h"
#include "check.h"

using namespace oem;

namespace {

// HR 100-107: heartbeat, CVL x100, CCL x10, flags, SOC, V x100, max cell x1000, current x10.
const uint16_t GOOD[8] = {1, 5640, 300, 1, 67, 5299, 3320, 0x7FFF};

void test_never_fresh_is_stale_fallback() {
  ModbusSource m;
  Settings s = default_settings();  // Max current 10, Fallback 10
  s.set(HR_FALLBACK_A, 4);
  const Limits l = m.limits(s, 5000);
  CHECK(!m.fresh(5000));
  CHECK(l.stale && l.fresh && l.allowed && l.source == SourceMode::MODBUS);
  CHECK_NEAR(l.ccl, 4.0, 1e-6);
  CHECK_NEAR(l.cvl, s.get(HR_CHARGE_V) / 100.0, 1e-4);
  CHECK(l.soc == -1 && std::isnan(l.battery_v));
}

void test_fresh_decode() {
  ModbusSource m;
  const Settings s = default_settings();
  m.heartbeat(GOOD, 1000);
  CHECK(m.fresh(1000) && m.fresh(1000 + EXT_FRESH_MS) && !m.fresh(1001 + EXT_FRESH_MS));
  const Limits l = m.limits(s, 2000);
  CHECK(l.fresh && !l.stale && l.source == SourceMode::MODBUS);
  CHECK_NEAR(l.cvl, 56.40, 1e-4);
  CHECK_NEAR(l.ccl, 30.0, 1e-4);  // not capped here: the controller applies Max current
  CHECK(l.allowed && !l.alarm && !l.blocking);
  CHECK(l.soc == 67);
  CHECK_NEAR(l.battery_v, 52.99, 1e-4);
  CHECK_NEAR(l.max_cell_v, 3.320, 1e-5);
  CHECK(std::isnan(l.battery_a));
}

void test_unknown_codes_and_flags() {
  ModbusSource m;
  const Settings s = default_settings();
  const uint16_t u[8] = {1, 5500, 20, 0b110, 0xFFFF, 0, 0, uint16_t(int16_t(-25))};
  m.heartbeat(u, 0);
  const Limits l = m.limits(s, 0);
  CHECK(!l.allowed && l.alarm && l.blocking);
  CHECK(l.soc == -1 && std::isnan(l.battery_v) && std::isnan(l.max_cell_v));
  CHECK_NEAR(l.battery_a, -2.5, 1e-5);
  const uint16_t soc101[8] = {1, 5500, 20, 1, 101, 0, 0, 0x7FFF};
  m.heartbeat(soc101, 0);
  CHECK(m.limits(s, 0).soc == -1);
}

void test_stale_after_window_and_stop_latch() {
  ModbusSource m;
  Settings s = default_settings();
  s.set(HR_FALLBACK_A, 0);  // 0 = no charging without the source
  m.heartbeat(GOOD, 0);
  Limits l = m.limits(s, EXT_FRESH_MS + 1);
  CHECK(l.stale && l.allowed);
  CHECK_NEAR(l.ccl, 0.0, 1e-6);

  const uint16_t off[8] = {1, 5640, 300, 0, 67, 5299, 3320, 0x7FFF};  // charge allowed off
  m.heartbeat(off, 20000);
  CHECK(!m.limits(s, 20000).allowed);
  CHECK(!m.limits(s, 20001 + EXT_FRESH_MS).allowed);  // a stop holds while stale
  m.forget_stop();                                      // a mode change forgets it
  CHECK(m.limits(s, 20001 + EXT_FRESH_MS).allowed);

  const uint16_t alarm[8] = {1, 5640, 300, 0b011, 67, 5299, 3320, 0x7FFF};  // allowed but alarm
  m.heartbeat(alarm, 40000);
  CHECK(m.limits(s, 40000).alarm);
  CHECK(!m.limits(s, 40001 + EXT_FRESH_MS).allowed);
  m.heartbeat(GOOD, 60000);  // fresh and fine again: the latch clears
  CHECK(m.limits(s, 60001 + EXT_FRESH_MS).allowed);
}

void test_forget_keeps_data() {
  ModbusSource m;
  const Settings s = default_settings();
  m.heartbeat(GOOD, 1000);
  m.forget_stop();
  CHECK(m.fresh(2000) && !m.limits(s, 2000).stale);
}

}  // namespace

int main() {
  test_never_fresh_is_stale_fallback();
  test_fresh_decode();
  test_unknown_codes_and_flags();
  test_stale_after_window_and_stop_latch();
  test_forget_keeps_data();
  return test_result("test_oem_modbus_source");
}
```

- [ ] **Step 2: Run, expect a compile failure** (`make -C tests test_oem_modbus_source`): `app/modbus_source.h` missing.

- [ ] **Step 3: Implement** `oem/src/app/modbus_source.h`:

```cpp
// oem/src/app/modbus_source.h: the Modbus limit source (spec 2026-10-15-oem-modbus-source-design.md). A master writes
// HR 101-107 and the HR 100 heartbeat; a block write starting at HR 100 hands all eight words here, and the limits
// are that snapshot while it is at most EXT_FRESH_MS old. Otherwise Panel limits capped at Fallback current, flagged
// stale (E15), unless the last snapshot was a stop (charge allowed off or the alarm bit): that holds charging off until
// a fresh snapshot or a mode change (forget_stop).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "app/limit_source.h"
#include "app/settings.h"

namespace oem {

// HR 104 (external SOC) and the Modbus source count while HR 100 (heartbeat) was written this recently.
constexpr uint32_t EXT_FRESH_MS = 10000;
constexpr uint16_t EXT_UNKNOWN_I16 = 0x7FFF;  // HR 107: current unknown

class ModbusSource {
 public:
  // ext = HR 100..107 as just written.
  void heartbeat(const uint16_t *ext, uint32_t now) {
    const uint16_t flags = ext[3];
    l_ = {float(ext[1]) / 100.0f,
          float(ext[2]) / 10.0f,
          (flags & 1u) != 0,
          (flags & 2u) != 0,
          (flags & 4u) != 0,
          ext[4] <= 100 ? int(ext[4]) : -1,
          ext[5] != 0 ? float(ext[5]) / 100.0f : NAN,
          ext[7] != EXT_UNKNOWN_I16 ? float(int16_t(ext[7])) / 10.0f : NAN,
          ext[6] != 0 ? float(ext[6]) / 1000.0f : NAN,
          true,
          false,
          SourceMode::MODBUS};
    seen_ = true;
    at_ms_ = now;
    stopped_ = !l_.allowed || l_.alarm;
  }

  void forget_stop() { stopped_ = false; }

  bool fresh(uint32_t now) const { return seen_ && now - at_ms_ <= EXT_FRESH_MS; }

  Limits limits(const Settings &s, uint32_t now) const {
    if (fresh(now)) return l_;
    Limits l = panel_limits(s);
    l.ccl = std::min(l.ccl, float(s.get(HR_FALLBACK_A)));
    l.allowed = !stopped_;
    l.stale = true;
    l.source = SourceMode::MODBUS;
    return l;
  }

 private:
  Limits l_{};
  bool seen_ = false;
  uint32_t at_ms_ = 0;
  bool stopped_ = false;  // the last snapshot had charge allowed off or the alarm bit
};

}  // namespace oem
```

In `modbus_map.h`: delete the `EXT_FRESH_MS` definition (keep its comment's meaning in the new file), add
`#include "app/modbus_source.h"`.

- [ ] **Step 4: Run** `make -C tests test` — all suites ok, including `test_oem_modbus_source: ok`.
- [ ] **Step 5: Commit** `OEM: ModbusSource (HR 100-107 snapshot, stale fallback, stop latch)`.

---

### Task 2: wire it in (map, main, version)

**Files:**
- Modify: `oem/src/app/modbus_map.h`
- Modify: `oem/src/main.cpp` (source selection ~line 213-217, header comment line 1-3)
- Modify: `oem/src/app/settings.h:15` comment (`0 Panel, 1 Pylontech, 2 Modbus`)
- Modify: `tests/test_oem_modbus_map.cpp`

**Interfaces:**
- Consumes: `ModbusSource` from Task 1.
- Produces: `OemRegisters::modbus_limits(const Settings &s, uint32_t now) const` → `Limits`;
  `OemRegisters::forget_modbus_stop()`.

- [ ] **Step 1: Tests first** in `tests/test_oem_modbus_map.cpp`:
  - `test_status_registers`: `g.ir(0) == 0x0306`.
  - In `test_settings_writes` (the block around the `// M1: Panel only` line), replace the HR 6 checks so they say:
    `write1(6, 1)` → `MB_EX_ILLEGAL_VALUE`; `write1(6, 2)` → 0 and `get(HR_SOURCE_MODE) == 2`; `write1(6, 0)` → 0.
    Then with a stored 1 (set from the menu): other writes still accepted, the keep-block still accepted,
    `write1(6, 2)` → 0 (leaving Pylontech for Modbus is a change to a link-keeping mode — accepted), then back to 0.
    Update the comment "M1: Panel only" to "M1: Panel or Modbus; never Pylontech (it would cut this link)".
  - New `test_modbus_source_snapshot`:
    ```cpp
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
      CHECK(g.write1(100, 1) == 0);   // a heartbeat alone snapshots what is stored
      CHECK_NEAR(g.regs.modbus_limits(g.s, 1000).ccl, 5.0, 1e-4);
      const uint16_t stop[8] = {1, 5500, 250, 0, 80, 5300, 3400, 0x7FFF};
      CHECK(g.regs.write_holding(100, stop, 8) == 0);
      CHECK(!g.regs.modbus_limits(g.s, 1001 + EXT_FRESH_MS).allowed);
      g.regs.forget_modbus_stop();
      CHECK(g.regs.modbus_limits(g.s, 1001 + EXT_FRESH_MS).allowed);
    }
    ```
    and in `test_source_registers` add: a tick with `l.source = SourceMode::MODBUS` → `ir(5) == 2`.
  - Register the new test in `main()`.
- [ ] **Step 2:** `make -C tests test` fails (no `modbus_limits`, version, HR 6 = 2 refused).
- [ ] **Step 3: Implement** in `modbus_map.h`:
  - `OEM_FW_VERSION = 0x0306`.
  - Member `ModbusSource modbus_;`. In `write_holding`'s HR 100-107 branch, after storing, when
    `start == HR_HEARTBEAT`: set `heartbeat_ms_`/`heartbeat_seen_` as today and call `modbus_.heartbeat(ext_, t_.now_ms)`.
  - `Limits modbus_limits(const Settings &s, uint32_t now) const { return modbus_.limits(s, now); }`,
    `void forget_modbus_stop() { modbus_.forget_stop(); }`.
  - `write_settings_`: refuse only `changes_source && c.get(HR_SOURCE_MODE) == int32_t(SourceMode::PYLONTECH)`; update
    the M1 comment accordingly. Out-of-range values (> 2) stay refused by `violates_clamps` — verify with a check
    `write1(6, 3) == MB_EX_ILLEGAL_VALUE` in the test; if `violates_clamps` doesn't refuse 3, refuse
    `c.get(HR_SOURCE_MODE) > 2` here too.
  - IR 5 comment: `0 Panel, 1 Pylontech, 2 Modbus`. `ext_` comment: `100-107: the Modbus source's registers; HR 104
    is also the Panel SOC display (3.5)`.
- [ ] **Step 4: Implement** in `main.cpp`, at the controller tick:
  ```cpp
      const SourceMode mode = SourceMode(settings.get(HR_SOURCE_MODE));
      if (mode != last_mode) {
        g_regs.forget_modbus_stop();  // a mode change forgets a source's stop (spec §1)
        last_mode = mode;
      }
      const Limits lim = pylon_active                ? g_pylon.limits(settings, now)
                         : mode == SourceMode::MODBUS ? g_regs.modbus_limits(settings, now)
                                                      : panel_limits(settings);
  ```
  with `SourceMode last_mode = SourceMode(settings.get(HR_SOURCE_MODE));` declared next to `pylon_active` before the
  loop. Replace the comment "Source mode selects Panel or Pylontech limits (Modbus is Panel until it's built)." with
  "Source mode selects Panel, Pylontech or Modbus limits." Update the file header (lines 1-3) to mention Modbus source.
  The external-SOC display line (`if (shown.soc < 0) shown.soc = g_regs.external_soc(now);`) stays.
- [ ] **Step 5:** `make -C tests test` all ok; build the firmware the way README "Building" says and confirm it links
  (note flash/RAM size from the build output in the report).
- [ ] **Step 6: Commit** `OEM 3.6: Modbus source mode (HR 100-107 from the master; HR 6 may switch Panel/Modbus)`.

---

### Task 3: bridge relays YamBMS limits

**Files:**
- Modify: `esphome/oem-bridge.yaml`
- Modify: `esphome/eco-charger-a.yaml`, `esphome/eco-charger-b.yaml` (add `ccl_share: "0.5"` substitution)

Read `esphome/oem-bridge.yaml` fully first; keep its style (comments explain *why*, entity_category diagnostic for
the YamBMS mirrors).

- [ ] **Step 1:** Substitutions in `oem-bridge.yaml` (with defaults, overridable by the per-bridge files):
  `yambms_prefix: yambms/yambms_1`, `ccl_share: "1.0"`. Change the existing SOC topic to
  `${yambms_prefix}/sensor/battery_soc/state`.
- [ ] **Step 2:** MQTT mirrors (all `internal: false`, `entity_category: diagnostic`, each `on_value` stamping its own
  `uint32_t` global with `millis()` — only the three required ones need globals: `yambms_cvl_ms`, `yambms_ccl_ms`,
  `yambms_allowed_ms`; replace `yambms_soc_ms` usage only where needed):
  - `sensor` `mqtt_subscribe`: `yambms_cvl` (`sensor/requested_charge_voltage/state`, V, 1 decimal, name
    "YamBMS CVL"), `yambms_ccl` (`sensor/requested_charge_current/state`, A, name "YamBMS CCL"),
    `yambms_alarm_bits` (`sensor/errors_bitmask_alarm/state`, internal: true), `yambms_blocking`
    (`sensor/bms_blocking_charge/state`, internal: true), `yambms_v` (`sensor/total_voltage/state`, internal: true),
    `yambms_max_cell` (`sensor/max_cell_voltage/state`, internal: true), existing `yambms_soc`.
  - `charging_allowed`: `text_sensor` `mqtt_subscribe` id `yambms_allowed`, topic
    `${yambms_prefix}/binary_sensor/charging_allowed/state`, name "YamBMS charging allowed", diagnostic; `on_value`
    stamps `yambms_allowed_ms`. (`binary_sensor` has no mqtt_subscribe platform; compare `x == "ON"`.)
- [ ] **Step 3:** Replace the 5 s SOC relay interval with the limits relay:
  ```yaml
    # YamBMS's limits for Source mode Modbus (firmware 3.6), and its SOC for the Panel display (3.5): one FC 16 of
    # HR 100-107 (heartbeat first) every 5 s while the line is Modbus's and CVL, CCL and charging_allowed have all
    # arrived within 60 s. Otherwise nothing is written, and the charger falls back (E15) 10 s after the last one.
    # CCL is the bank's: each charger gets ccl_share of it. Bank current isn't sent (HR 107 unknown): it includes the
    # other charger and the inverter.
    - interval: 5s
      then:
        - if:
            condition:
              lambda: |-
                const uint32_t now = millis();
                auto recent = [&](uint32_t t) { return t != 0 && now - t < 60000; };
                return !id(mux).passthrough() && recent(id(yambms_cvl_ms)) && recent(id(yambms_ccl_ms)) &&
                       recent(id(yambms_allowed_ms)) && !std::isnan(id(yambms_cvl).state) &&
                       !std::isnan(id(yambms_ccl).state);
            then:
              - lambda: |-
                  auto u16 = [](float x, float scale, uint16_t unknown) -> uint16_t {
                    if (std::isnan(x) || x < 0) return unknown;
                    const float v = x * scale;
                    return v >= 65535.0f ? 65535 : uint16_t(std::lround(v));
                  };
                  const float soc = id(yambms_soc).state;
                  const float share = ${ccl_share};
                  const bool allowed = id(yambms_allowed).state == "ON";
                  const bool alarm = !std::isnan(id(yambms_alarm_bits).state) && id(yambms_alarm_bits).state != 0;
                  const bool blocking = !std::isnan(id(yambms_blocking).state) && id(yambms_blocking).state > 0;
                  std::vector<uint16_t> regs = {
                      1,
                      u16(id(yambms_cvl).state, 100.0f, 0),
                      u16(id(yambms_ccl).state * share, 10.0f, 0),
                      uint16_t((allowed ? 1 : 0) | (alarm ? 2 : 0) | (blocking ? 4 : 0)),
                      std::isnan(soc) ? uint16_t(0xFFFF) : uint16_t(soc < 0 ? 0 : soc > 100 ? 100 : std::lround(soc)),
                      u16(id(yambms_v).state, 100.0f, 0),
                      u16(id(yambms_max_cell).state, 1000.0f, 0),
                      0x7FFF,
                  };
                  auto cmd = modbus_controller::ModbusCommandItem::create_write_multiple_command(id(charger), 100,
                                                                                                regs.size(), regs);
                  id(charger)->queue_command(cmd);
  ```
  Note the SOC no longer needs its own freshness check (it rides with the limits; a SOC that stopped updating while
  the limits update is still YamBMS's latest). Remove `yambms_soc_ms` if nothing uses it any more.
- [ ] **Step 4:** Source mode select: keep `optionsmap` with all three (the board refuses Pylontech; leave a comment
  saying so: "Pylontech is menu-only: the board refuses it here (it would cut this link)").
- [ ] **Step 5:** Diagnostic HA sensors for what the charger uses: if `oem-bridge.yaml` doesn't already expose IR 14
  (CVL in use) and IR 15 (CCL in use), add them inside the existing IR 0-39 block pattern (same `ir_block` approach as
  the others). Check before adding.
- [ ] **Step 6:** `cd esphome && esphome compile eco-charger-a.yaml` and the same for `eco-charger-b.yaml` (never
  `esphome config`, never print secrets). Both must compile. Do NOT upload.
- [ ] **Step 7: Commit** `bridge: relay YamBMS limits to HR 100-107 for Source mode Modbus (ccl_share 0.5 per charger)`.

---

### Task 4: docs

**Files:** `README.md` (Pylontech section's Switching bullet, the "Remote access through the WiFi bridge" section's
SOC-relay bullet around line 334, and a new "Modbus source (YamBMS through the bridge)" section after the Pylontech
one), `docs/oem-controls.md` (menu item 7 row: `2 Modbus (limits from Home Assistant / YamBMS through the WiFi
bridge)`; Source mode section: a **2 Modbus** bullet; E15 row: "the BMS / the bridge isn't answering (Pylontech or
Modbus mode)"; version 3.6 in the intro), `docs/history/notes/oem-bringup.md` (a short "3.6 Modbus source: built, not yet flashed"
entry).

- [ ] **Step 1:** Write the README section from the spec §1-2 (registers table, freshness, fallback, latch, HR 6 rule,
  the bridge relay with `ccl_share`, why current isn't relayed). Keep the existing tone: short sentences, tables.
- [ ] **Step 2:** Update `docs/oem-controls.md` as listed. No Eco-Worthy/EC100 name.
- [ ] **Step 3:** Commit `docs: Modbus source mode (3.6)`.
