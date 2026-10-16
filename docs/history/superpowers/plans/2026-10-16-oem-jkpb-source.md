# OEM JK-PB BMS Source Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Source mode 5 (JK-PB): the board is the Modbus master on a JK-PB pack's RS485-1 (115200), finds the pack's
address, reads the 300-byte cell-info and settings records and turns them into Limits (CVL/CCL from the pack's settings).

**Architecture:** Same shape as 3.7's JK-B source: a pure protocol header (`jkpb_protocol.h`: request frames, record
validation, field decode) and a `JkpbSource final : public PolledSource` (`jkpb_source.h`: scan/lock, cadence, reply
collection, freshness, limits, stop latch), host-tested; `main.cpp` adds it to `poller_for`.

**Tech Stack:** C++17 header-only app code (oem/src/app), host tests `make -C tests test` (-Werror), firmware
`cmake --build oem/build` (must link), Python 3 tools (`.venv/bin/python -m pytest tools/test_fake_jkpb.py`), ESPHome YAML.

**Spec:** `docs/history/superpowers/specs/2026-10-16-oem-jkpb-source-design.md` (binding). Protocol detail with citations:
`docs/history/notes/jk-pb-protocol.md` §3-6. Real bytes: `docs/history/notes/jk-pb-capture.txt` (lines: master cell info, its FC10 trailer, master
settings, trailer, request to 01, slave 01 cell info (TV1), its trailer `01 10 16 20 00 01 04 4B`, request, slave 01
settings, trailer `01 10 16 1E 00 01 65 87`). Templates to mirror: `oem/src/app/jk_source.h`, `jk_protocol.h`,
`tests/test_oem_jk.cpp`, `tools/fake_jk.py`, `tools/test_fake_jk.py`.

## Global Constraints

- Firmware 3.9: `OEM_FW_VERSION = 0x0309`. `SourceMode::JKPB = 5`; `is_polled()` true for it; HR 6 range 0-5; Modbus
  writes of HR 6 refuse 5 (as 1, 3, 4); IR 5 reads 5.
- 115200 8N1. Requests: FC 0x10, one register, value 0x0000, CRC-16/MODBUS low byte first (`oem/src/app/crc16.h`):
  cell info register 0x1620 (addr 1: `01 10 16 20 00 01 02 00 00 D6 F1`; addr 0: `00 10 16 20 00 01 02 00 00 DB 61`),
  settings 0x161E (addr 1: `01 10 16 1E 00 01 02 00 00 D2 2F`; addr 0: `00 10 16 1E 00 01 02 00 00 DF BF`).
- Reply: 300-byte record `55 AA EB 90 TT ..` (TT 0x02 cell info, 0x01 settings), byte 299 = sum of bytes 0..298 mod 256;
  then an 8-byte trailer `AA 10 RH RL 00 01 CRClo CRChi` with the polled address and register and a good CRC. Only a record
  plus its matching trailer counts. Reply window 750 ms from the request.
- Poll every 1000 ms. Address scan: 0, then 1..15 (one cell-info request each), lock on the first valid reply; 3 missed
  polls in a row on the locked address → scan again starting at the locked address. After a lock: a settings request
  first, then every 60 000 ms; otherwise cell info.
- Fresh = valid cell info within 3000 ms AND a valid settings record since the lock. Not fresh → `stale_limits(s,
  stopped_, SourceMode::JKPB)`.
- Cell info (LE): cell n u16 mV at 6+2n (n 0..31, 0 = absent); pack u32 mV at 150; alarms u32 at 166; SOC u8 at 173
  (>100 → -1); charge MOS u8 at 198; charge stage u8 at 280. Settings (u32 LE): CellRCV mV at 38, CellRFV mV at 42,
  CurBatCOC mA at 50, cell count at 114, charge switch at 118. Settings invalid if cell count ∉ 1..32 or RCV/RFV ∉
  2500..4500 mV.
- CVL = cells × RCV / 1000 (stage 2 → cells × RFV / 1000); CCL = COC / 1000; alarm = alarms & 0x00E107EF ≠ 0; allowed =
  MOS == 1 && switch == 1 && !alarm && !(alarms & 0x10); battery_a NaN; max cell = max present cell (0 and ≥ 5000 mV
  ignored), NaN if none. Stop latch: any valid cell-info record whose own fields say stop (alarm, bit 4, charge MOS off) sets it,
  even while not fresh, as does a valid settings record with the charge switch off; a fresh allowed state clears it; reset() forgets everything.
- Never run `esphome config`; never read `esphome/secrets.yaml`; never run `tools/fake_nasn.py`; no flashing, no network to
  the chargers/bridges, no push.
- Commit trailer (implementers: your own model name in Co-Authored-By):
  ```
  Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Q88Pr4eMBeeEdBYp5fsSSu
  ```
- `git add` explicit paths only (untracked vendor files exist; never add PDFs, .dat, captures/, dumps/, re/). New test
  binaries go in `tests/Makefile` TESTS and `.gitignore` next to the others. Comment lines ≤ 120 columns.

---

### Task 1: JK-PB protocol and source

**Files:** Create `oem/src/app/jkpb_protocol.h`, `oem/src/app/jkpb_source.h`, `tests/test_oem_jkpb.cpp`; modify
`tests/Makefile`, `.gitignore`, `oem/src/app/limit_source.h` (add `JKPB = 5` to the enum and to `is_polled`, comment).

**Produces:**
```cpp
namespace oem::jkpb {
constexpr size_t RECORD_LEN = 300, TRAILER_LEN = 8, REQUEST_LEN = 11;
constexpr uint16_t REG_CELL_INFO = 0x1620, REG_SETTINGS = 0x161E;
constexpr uint8_t TYPE_SETTINGS = 0x01, TYPE_CELL_INFO = 0x02;
size_t build_request(uint8_t addr, uint16_t reg, uint8_t *out);           // 11 bytes
bool record_ok(const uint8_t *r, uint8_t type);                           // header, type, sum (r has RECORD_LEN bytes)
bool trailer_ok(const uint8_t *t, uint8_t addr, uint16_t reg);            // TRAILER_LEN bytes
struct CellInfo { float pack_v; int soc; float max_cell_v; uint32_t alarms; bool charge_mos; uint8_t stage; };
struct ChargeSettings { int cells; uint32_t rcv_mv, rfv_mv, coc_ma; bool charge_switch; };
CellInfo decode_cell_info(const uint8_t *r);
bool decode_settings(const uint8_t *r, ChargeSettings &s);               // false when out of range
}
namespace oem {
constexpr uint32_t JKPB_BAUD = 115200, JKPB_POLL_MS = 1000, JKPB_REPLY_MS = 750, JKPB_FRESH_MS = 3000,
                   JKPB_SETTINGS_MS = 60000;
constexpr unsigned JKPB_MISSES = 3;
constexpr uint32_t JKPB_ALARM_MASK = 0x00E107EF, JKPB_CELL_OVP = 0x10;
class JkpbSource final : public PolledSource { /* PolledSource API, plus: */
  bool fresh(uint32_t now) const; bool locked() const; uint8_t address() const;
  uint32_t good_replies() const; uint32_t bad_replies() const; };
}
```

**Behaviour** (spec §1; mirror `jk_source.h`'s structure and comments):
- Reply collection: after a request, inside the window, wait for `55 AA EB 90`, collect RECORD_LEN bytes, then
  TRAILER_LEN bytes; check `record_ok(type of the request)` and `trailer_ok(addr, reg)`; on failure count `bad_` and resync
  from the next `55` in what was collected (replay, as JkSource does). Bytes outside a pending request or the window are
  dropped. A complete good unit clears `pending_`. A reply in progress holds the next request (as JK_HOLD_MS).
- Scan: unlocked → each poll is a cell-info request to `scan_addr_` (start 0); a miss advances `scan_addr_` (15 wraps to
  0). A valid cell-info reply locks `addr_ = scan_addr_`. Locked: 3 misses in a row → unlocked, `scan_addr_ = addr_`
  (settings and freshness forgotten; the latch kept).
- Cadence locked: next request is settings if none received since the lock or 60 s since the last settings request,
  else cell info. A settings reply that fails `decode_settings` counts bad and leaves settings unknown.
- Limits: as Global Constraints. `fresh()` false until both records are known.

**Tests** (`tests/test_oem_jkpb.cpp`, `check.h` style as `test_oem_jk.cpp`; copy the hex of TV1 (slave 01 cell info), its
trailer, and the slave 01 settings record + trailer from `docs/history/notes/jk-pb-capture.txt` into the test as byte arrays):
- `build_request` for addr 0/1 × both registers equals the four frames above.
- TV1: `record_ok(…, 0x02)`; decodes to pack 52.287 V, SOC 64, max cell 3.268, alarms 0x00080000, charge_mos true,
  stage 0. Settings: `decode_settings` → cells 16, RCV 3600, RFV 3500, COC 40000, switch true. Trailers ok for (1, 0x1620)
  and (1, 0x161E); not for addr 0, nor the other register; one CRC byte flipped → false.
- Rejections: one data byte flipped (sum), wrong type, cell count 0 or 33, RCV 2400.
- Source: scan (addr 0 silent past the window → next poll asks addr 1; reply from 1 → locked, address() == 1); first locked
  request is settings; not fresh after cell info alone; fresh after settings + cell info; limits CVL 57.6, CCL 40, soc 64,
  battery_v 52.287, max_cell 3.268, allowed, !alarm; settings re-requested after 60 s; stage 2 → CVL 56.0; alarm bit 5 →
  alarm + !allowed (E12 path); bit 4 alone → !allowed, !alarm; bit 11 (cell UVP) and bit 19 → allowed; charge_mos 0 →
  !allowed; 3 s without a reply → stale limits (ccl = min(Max current, Fallback)); stop latch kept while stale and
  cleared by a fresh allowed record; 3 misses → unlocked and scanning from the old address; a record with no trailer
  times out cleanly; a master record (trailer addr 00) while polling 01 is rejected; reset() forgets everything.
- Build helper: a function in the test that builds a valid record from TV1 with patched fields (fix the sum) and a
  trailer with the right CRC, so cases don't need more captured hex.
- Commit: `OEM: JK-PB BMS source (Modbus trigger, 300-byte records)`.

### Task 2: wire mode 5 (firmware 3.9)

**Files:** `oem/src/app/settings.h` (HR 6 range 0-5, comment), `oem/src/app/modbus_map.h` (version 0x0309; HR 6
refuses 5; IR 5 comment), `oem/src/main.cpp` (`g_jkpb`, `poller_for` → JKPB; header comment), `oem/src/app/ui.h` only if
item 7's range isn't taken from the settings range; tests: `test_oem_modbus_map.cpp` (version, both places it is checked
— IR 0 and the raw FC04 response bytes; HR 6 = 5 refused, 2 accepted), `test_oem_settings.cpp`/`test_oem_ui.cpp` (item 7
reaches 5, 6 rejected), `test_oem_controller.cpp` (a JK-PB source fed records: alarm → E12, stale → E15, mirroring
`test_eg4_source_through_controller`).
- `cmake --build oem/build` links; report FLASH/RAM.
- Commit: `OEM 3.9: Source mode JK-PB`.

### Task 3: simulator, CLI and bridge

**Files:** Create `tools/fake_jkpb.py`, `tools/test_fake_jkpb.py`; modify `tools/oem_modbus.py` (`SOURCES` adds
`"JK-PB"`), `tools/test_oem_modbus.py` if it checks names, `esphome/oem-bridge.yaml` (Source mode select adds
`"JK-PB": 5`; comment lists JK-PB among the menu-only modes).
- Model on `tools/fake_jk.py`: `--port`/`--bridge` (Passthrough at 115200), `--address` (DIP, default 1), `--voltage`,
  `--soc`, `--max-cell`, `--alarms`, `--charge-mos`, `--stage`, `--rcv`, `--rfv`, `--coc`, `--cells`, `--control FILE`
  (set NAME VALUE, silent on|off, show, quit), `-q`. Answers only well-formed FC10 requests (good CRC) to its address for
  0x1620/0x161E with record + trailer, built from the TV1 / settings templates with fields patched and the sum fixed.
- Tests: unpatched templates reproduce the captured bytes exactly (record and trailer); patched fields decode as the C++
  does (mirror the offsets); requests with a bad CRC, another address or another register get no reply.
- `esphome compile eco-charger-a.yaml` and `eco-charger-b.yaml` (from esphome/), both succeed. Never upload.
- Commit: `tools: JK-PB BMS simulator; bridge/CLI know Source mode 5`.

### Task 4: docs

**Files:** `README.md` (a JK-PB section next to "EG4 and JK BMS sources": RS485-1, protocol 001, 115200, DIP address found
automatically, parallel bank → master pack only, CVL/CCL from the pack's settings, stop conditions incl. cell OVP without
E12, stale fallback, simulator, unproven trigger on RS485-1; remove "JK-PB not supported"), `docs/oem-controls.md` (item 7
row 0-5; a mode 5 bullet; E12/E15 rows mention JK-PB; version 3.9; mode 4 bullet no longer says JK-PB is unsupported; no
Eco-Worthy/EC100 name), `docs/history/notes/oem-bringup.md` ("3.9 JK-PB source: built and host-tested, not flashed" + live checklist:
`fake_jkpb.py --bridge`, item 7 = 5, SOC shown, alarm bit 5 → E12, bit 4 → no error, silent → E15, back).
- Commit: `docs: JK-PB BMS source (3.9)`.
