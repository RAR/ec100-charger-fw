# OEM EG4 and JK BMS Sources Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Source modes 3 (EG4, Modbus RTU master at 9600) and 4 (JK-B `4E 57` protocol at 115200): the board polls the
BMS on RS485 for SOC, pack V, max cell V, charge allowed / alarm and (EG4) CCL, with Pylontech's stale fallback.

**Architecture:** A `PolledSource` interface over the three BMS pollers; `PylonSource` adopts it unchanged in
behaviour; `Eg4Source` and `JkSource` are new header-only units with pure protocol helpers (`eg4_protocol.h`,
`jk_protocol.h`) unit-tested on the host. `main.cpp` holds one active poller chosen by HR 6, at its baud.

**Tech Stack:** C++17 header-only app code (oem/src/app), host tests `make -C tests test` (-Werror), firmware
`cmake --build oem/build` (must link), Python 3 tools in tools/ with `python3 -m pytest tools` (or the existing
test runner — check how tools/test_fake_pylon.py is run), ESPHome bridge YAML.

**Spec:** `docs/history/superpowers/specs/2026-10-16-oem-eg4-jk-sources-design.md`. Byte-level protocol detail:
`docs/history/notes/eg4-bms-protocol.md`, `docs/history/notes/jk-bms-protocol.md` (read the relevant sections; they cite their sources).

## Global Constraints

- Firmware version 3.7: `OEM_FW_VERSION = 0x0307` (Task 4).
- `SourceMode` values: 0 PANEL, 1 PYLONTECH, 2 MODBUS, 3 EG4, 4 JK.
- EG4: 9600 8N1, slave 0x10, block A `10 03 00 00 00 12` + CRC, block B `10 03 00 12 00 15` + CRC (CRC16/Modbus, low
  byte first; reuse `oem/src/app/crc16.h`), alarm mask 0x0593 on 0x001B, reject pack V 0, CCL = reg 0x0016 A.
- JK: 115200 8N1, request `4E 57 00 13 00 00 00 00 06 SS 00 00 00 00 00 00 68 00 00 CK CK` with SS = 0x03 (checksum
  `01 29`) or 0x02 (`01 28`); checksum = 16-bit big-endian sum of all preceding bytes; alarm = 0x8B & 0x37B6;
  allowed = 0x8C bit0 && !alarm (bit 3 unused: TV1 reports 0x0007); CCL = Max current; required IDs 0x79 0x83 0x85 0x8B 0x8C.
- Both: CVL = Charge V (Panel); battery_a NaN; SOC 0-100 else -1; poll 500 ms (JK 1000 ms); reply window 400 ms
  (JK 600 ms); fresh 3 s; stale = Panel limits, ccl = min(Max current, Fallback HR 9), stale true, allowed = !stopped,
  source = the mode; stop latch set when fresh and (!allowed || alarm); reset() forgets everything.
- Never run `esphome config`; never read `esphome/secrets.yaml`; never run `tools/fake_nasn.py`; no flashing, no
  network to the chargers/bridges, no push.
- Commit trailer (implementers: your own model name in Co-Authored-By):
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Q88Pr4eMBeeEdBYp5fsSSu
  ```
- `git add` explicit paths only (untracked vendor files exist; never add PDFs, .dat, captures/, dumps/, re/).
- Add each new test binary to `tests/Makefile` TESTS and to `.gitignore` next to the others.

---

### Task 1: `PolledSource` interface and the shared stale fallback (no behaviour change)

**Files:** Create `oem/src/app/polled_source.h`; modify `oem/src/app/limit_source.h`, `oem/src/app/pylon_source.h`,
`oem/src/app/modbus_source.h`, `oem/src/main.cpp`, `tests/test_oem_pylon.cpp`.

**Produces:**
```cpp
// limit_source.h
// A configured source that isn't fresh: Panel limits with ccl capped at Fallback current, flagged stale; a source
// whose last fresh state was a stop keeps charging off.
inline Limits stale_limits(const Settings &s, bool stopped, SourceMode source);

// polled_source.h — a BMS the board polls on RS485 (the board is the master; no Modbus server meanwhile).
constexpr size_t POLL_REQ_MAX = 32;  // longest request any poller writes
class PolledSource {
 public:
  virtual void reset() = 0;                                           // a mode change: stale, start over
  virtual void push(const uint8_t *data, size_t n, uint32_t now) = 0;  // bytes received since the last call
  virtual size_t poll(uint32_t now, uint8_t *out) = 0;                // a due request into out, else 0
  virtual Limits limits(const Settings &s, uint32_t now) const = 0;
  virtual uint32_t baud() const = 0;
 protected:
  ~PolledSource() = default;  // never deleted through the base (no heap)
};
```
- `PylonSource` derives from `PolledSource` (`final`), `poll` takes `uint8_t *out` (adapt `pylon::build_request`'s
  char buffer with a cast or change its parameter — keep `pylon_protocol.h` tests passing), `baud()` returns 9600
  (move `PYLON_BAUD` from main.cpp into pylon_source.h). Its stale branch and `ModbusSource`'s call `stale_limits`.
- main.cpp: replace `wants_pylon`/`pylon_active`/`g_pylon` usage with `PolledSource *poller_for(const Settings &)`
  (returns `&g_pylon` for PYLONTECH, else nullptr) and `PolledSource *active` (nullptr = Modbus server). The role
  switch compares pointers; the line baud is `active->baud()`. The source selection becomes
  `active ? active->limits(...) : mode == MODBUS ? g_regs.modbus_limits(...) : panel_limits(...)`. Keep every existing
  guard (tx busy, pending reboot, discard_until, mb_was_busy) and comment.
- Verify: `make -C tests test` all ok, firmware links (note size). Pure refactor — no test expectation changes except
  the poll buffer type in test_oem_pylon.cpp.
- Commit: `OEM: PolledSource interface and shared stale fallback (no behaviour change)`.

### Task 2: EG4 source

**Files:** Create `oem/src/app/eg4_protocol.h`, `oem/src/app/eg4_source.h`, `tests/test_oem_eg4.cpp`.

**Produces:** `namespace oem::eg4 { constexpr uint8_t ADDR = 0x10; constexpr size_t REQUEST_LEN = 8;
size_t build_request(uint16_t first_reg, uint16_t count, uint8_t *out); enum class Error {...};
Error parse_reply(const uint8_t *f, size_t n, uint16_t count, const uint8_t *&data);  // validates addr, fn 03, byte
count == 2*count, CRC; data → first register byte
struct BlockA { float pack_v; float max_cell_v; };  BlockA decode_a(const uint8_t *data);  // 0x00-0x11
struct BlockB { int soc; float ccl; uint16_t protection; };  BlockB decode_b(const uint8_t *data);  // 0x12-0x26 }`
and `class Eg4Source final : public PolledSource` (constants `EG4_POLL_MS = 500`, `EG4_REPLY_MS = 400`,
`EG4_FRESH_MS = 3000`, `EG4_BAUD = 9600`, `EG4_ALARM_MASK = 0x0593`).
- Reply framing: Modbus RTU replies have a known length (5 + 2 × count); collect bytes after a request until that
  length (or the window ends), then validate. Bytes outside a window are dropped. A pack voltage of 0 rejects block A.
- Register offsets: block A starts at 0x0000 (pack V u16 ×0.01, current s16 unused, cells 0x0002-0x0011 mV — max of
  cells ignoring 0 and ≥ 5000); block B starts at 0x0012: SOC at 0x0018, max charging current 0x0016, protection
  0x001B (offsets = (reg − 0x12) × 2). Confirm each against docs/history/notes/eg4-bms-protocol.md §4.
- Tests: request bytes equal the notes' `10 03 00 00 00 12 C6 86` and `10 03 00 12 00 15 27 41`; a synthetic reply
  built in the test with a CRC computed by `crc16` decodes to the chosen values; each rejection (wrong address,
  function, byte count, CRC, truncated, zero pack V); poller alternation A/B, 500 ms cadence, 400 ms window, fresh
  needs both blocks within 3 s; limits fresh (cvl = Charge V, ccl = reg 0x16, allowed, alarm with each mask bit and
  not with a non-mask bit e.g. 0x0008), stale fallback, latch on alarm, reset.
- Commit: `OEM: EG4 BMS source (Modbus RTU master, blocks A/B)`.

### Task 3: JK source

**Files:** Create `oem/src/app/jk_protocol.h`, `oem/src/app/jk_source.h`, `tests/test_oem_jk.cpp`.

**Produces:** `namespace oem::jk { constexpr size_t REQUEST_LEN = 21, MAX_FRAME = 400;
size_t build_request(uint8_t source_byte, uint8_t *out); enum class Error {...};
Error parse_reply(const uint8_t *f, size_t n, Data &d);  struct Data { float pack_v; int soc; float max_cell_v;
uint16_t alarms; uint16_t status; int cells; };` and `class JkSource final : public PolledSource` (`JK_POLL_MS = 1000`,
`JK_REPLY_MS = 600`, `JK_FRESH_MS = 3000`, `JK_BAUD = 115200`, `JK_ALARM_MASK = 0x37B6`).
- Framing: starts `4E 57`, length field (bytes 2-3, BE) = total − 2, checksum = last 2 bytes? — follow the notes §3
  exactly (checksum position and coverage, and the 4-byte trailer before it if any; the TV1 frame ends
  `…68 00 00 54 D1`). Records start at byte 11. Walk IDs with a length table from notes §4 (0x79 variable); an
  unknown ID stops the walk; required IDs must have been seen.
- Collect a reply from `4E 57` until the length field's total (bounded by MAX_FRAME) within the reply window.
- Source byte: start 0x03; after 3 consecutive polls without a valid reply, flip 0x03 ↔ 0x02.
- Tests: request checksums (`01 29` / `01 28`); TV1 (copy the hex from notes §5 into the test) decodes to the notes'
  values (pack V, SOC, cells 14, max cell, alarms 0, status 0x0007); corrupt checksum, wrong length, truncated, missing
  required ID, unknown ID mid-frame (fields before kept → still rejected if a required one is after it); alarm mask
  (a mask bit set → alarm; a non-mask bit e.g. bit 0 → not); allowed needs status bit 0 (bit 3 ignored); poller cadence, window,
  freshness, source-byte flip after 3 misses; stale fallback, latch, reset.
- Commit: `OEM: JK BMS source (4E 57 protocol, ID-tagged records)`.

### Task 4: wire modes 3 and 4 (firmware 3.7)

**Files:** `oem/src/app/limit_source.h` (enum), `oem/src/app/settings.h` (HR 6 range 0-4, comment),
`oem/src/app/modbus_map.h` (version 0x0307; HR 6 Modbus writes refuse changing to any polled mode 1/3/4; IR 5
comment), `oem/src/main.cpp` (`poller_for`: EG4 → g_eg4, JK → g_jk; header comment), `oem/src/app/ui.h` only if the
menu range for item 7 isn't already taken from HR_RANGES, tests (`test_oem_modbus_map.cpp`: version, HR 6 3/4
refused, 2 still accepted; `test_oem_settings.cpp`/`test_oem_ui.cpp`: item 7 reaches 4; a controller-level test that
a fresh EG4 alarm → E12 and stale → E15 if cheap using the source classes with pushed frames).
- Commit: `OEM 3.7: Source modes EG4 and JK`.

### Task 5: simulators, CLI and bridge

**Files:** Create `tools/fake_eg4.py`, `tools/test_fake_eg4.py`, `tools/fake_jk.py`, `tools/test_fake_jk.py`; modify
`tools/oem_modbus.py` (`SOURCES` + any help text), `tools/test_oem_modbus.py` if it checks names, `esphome/oem-bridge.yaml`
(Source mode select options add `"EG4": 3`, `"JK": 4`; comment: polled modes are menu-only, refused here).
- Model the simulators on `tools/fake_pylon.py` (same CLI shape: `--port` / `--bridge`, value flags, `--control FILE`,
  `-q`, silence command; `--bridge` sets the bridge baud — 9600 EG4, 115200 JK — the way fake_pylon does). EG4 answers
  reads of 0x0000+0x12 and 0x0012+0x15 at address 0x10 (others ignored). JK answers the read-all request (either
  source byte) with a frame built from a record list (14 or 16 cells) whose checksum and length are right.
- Tests: encoders reproduce the firmware test vectors (the EG4 request CRCs; JK TV1 checksum when fed TV1's values,
  or at least a self-consistent frame that `jk` parsing rules accept — mirror the C++ validation in Python).
- `esphome compile` eco-charger-a.yaml and eco-charger-b.yaml (from esphome/), both must succeed. Never upload.
- Commit: `tools: EG4 and JK BMS simulators; bridge/CLI know Source modes 3-4`.

### Task 6: docs

**Files:** `README.md` (new "EG4 and JK BMS sources" section next to Pylontech: wiring/baud/address, what's read,
CVL = Charge V, CCL, stop conditions, stale fallback, simulators, JK-PB not supported), `docs/oem-controls.md` (item 7
row 0-4; Source mode bullets for 3 and 4; E12/E15 rows mention EG4/JK; version 3.7; no Eco-Worthy/EC100 name),
`docs/history/notes/oem-bringup.md` ("3.7 EG4/JK sources: built and host-tested, not flashed", live checklist).
- Commit: `docs: EG4 and JK BMS sources (3.7)`.
