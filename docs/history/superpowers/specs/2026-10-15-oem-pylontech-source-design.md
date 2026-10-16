# OEM firmware: Pylontech limit source (stock dialect)

**Status:** design, approved in chat on 2026-10-15. Implements the Pylontech row of
[`2026-10-14-oem-board-firmware-design.md`](2026-10-14-oem-board-firmware-design.md) §2.1 (phase 3b, Pylontech half;
the Modbus source stays unbuilt).

## 1. Decisions

- **Dialect:** the stock charger's, CID2 0x61 / 0x62 / 0x63 at ADR 0x12, VER 0x20, CID1 0x46, empty INFO, 9600 8N1. It is
  the inverter-facing Pylontech RS485 dialect that the user's YamBMS charger proxy (`RAR/esphome-yambms-addons`,
  `packages/charger/charger_proxy_rs485.yaml`) answers; Eco-Worthy's JBD-based batteries are reported to answer it, with
  units unconfirmed until a capture. The Pylontech *low-voltage* set (0x42 / 0x44 / 0x92) is not built now.
- **Line ownership:** in Pylontech mode the RS485 line belongs to the BMS: the board is the master and the Modbus server
  is off. The mode is chosen with the buttons (menu item 7, Source mode, HR 6). Modbus writes of HR 6 keep refusing
  anything but Panel (`modbus_map.h`), because switching remotely would cut the link the command came over.
- **Source mode values:** 0 Panel (Modbus server on, as today), 1 Pylontech, 2 Modbus = behaves as Panel (not built).
- **Battery current from 0x61 is not used** (its unit is unconfirmed: the proxy sends 0.01 A, the low-voltage document's
  equivalent field is 0.1 A, and stock never used it). `Limits::battery_a` stays NaN, as for Panel.
- **CCL unit: 0.1 A, as the YamBMS proxy encodes it, and that conflicts with stock.** Stock v2's follow mode implies
  0.01 A (`docs/history/notes/pylon-stock-protocol.md`, 0x63 layout). The firmware reads 0.1 A; a BMS that sends 0.01 A would read
  ten times too high, still capped by Max current. A capture from a real Eco-Worthy BMS settles it.
- **Battery voltage:** while fresh, the 0x61 voltage is the battery voltage the session gate and soft start use, in
  place of the measured one. A simulator with a fixed voltage therefore stands in for the bank; give it the bank's
  voltage and supervise.

Sources for the layouts: `docs/history/notes/pylon-stock-protocol.md` (stock's parsers, [V]/[I]), the proxy's encoder (units), and the
Pylontech low-voltage V3.3 document (framing and checksums; its 0x92 has the same layout and units as 0x63).

## 2. Frames

ASCII: `~` VER ADR CID1 CID2 LENGTH INFO CHKSUM `\r`, every field except SOI/EOI as upper-case hex text.

- **LENGTH:** 16 bits, LENID (low 12 bits) = number of ASCII characters in INFO; LCHKSUM (top 4 bits) =
  `(~(n2 + n1 + n0) + 1) & 0xF` over the three LENID nibbles. LENID 0 gives LENGTH `0000`.
- **CHKSUM:** `(~sum + 1) & 0xFFFF` over the ASCII codes of everything between `~` and the checksum.
  Test vector: stock's captured request `~201246600000FDAB\r` (FDAB). LENGTH for LENID 18 = `D012`. (The Pylontech
  low-voltage document's own CHKSUM example, `~1203400456ABCEFEFC72\r`, is off by one: its ASCII sum is 0x38F, so FC71.)
- **Requests** (fixed strings): `~201246610000FDAA\r`, `~201246620000FDA9\r`, `~201246630000FDA8\r`.
- **Reply acceptance:** starts with `~`, ends with `\r`, all other characters hex, total length = 18 + LENID, LCHKSUM and
  CHKSUM correct, CID1 = 0x46, RTN (the CID2 position) = 0x00, and LENID ≥ the minimum for the command polled. VER and
  ADR are not checked (the proxy answers with ADR 0x02; stock ignored both). A reply is parsed as the command that was
  polled (as stock does).

## 3. INFO layouts used (decoded bytes, big-endian)

| CID2 | Offset | Field | Unit / use |
|---|---|---|---|
| 0x61 (≥ 13 bytes) | 0-1 | total voltage | mV → `battery_v` |
| | 2-3 | current | not used (§1) |
| | 4 | SOC | % → `soc` (0-100; anything above 100 = unknown) |
| | 11-12 | max cell voltage | mV → `max_cell_v` |
| 0x62 (≥ 4 bytes) | 2, 3 | protection bytes | alarm if `INFO[2] & 0xAA` or `INFO[3] & 0x68` (stock v2's mask) |
| 0x63 (≥ 9 bytes) | 0-1 | CVL | mV → `cvl` |
| | 4-5 | CCL | 0.1 A → `ccl` (total) |
| | 8 | status | bit 7 charge enable → `allowed` |

DVL, DCL and status bits 6-4 are decoded for the record only.

## 4. Polling and freshness

- One request every 500 ms, cycling 0x61 → 0x62 → 0x63 (1.5 s per cycle), whether or not the previous one was answered
  (stock repeated a command until it got a valid reply; cycling keeps all three refreshed and can't get stuck).
- The reply is collected from `~` to `\r`; a reply is considered only within 400 ms of its request. Bytes outside a
  frame, or arriving with no request outstanding, are dropped.
- **Fresh** while a valid 0x61 *and* a valid 0x63 have both arrived within the last 3 s. 0x62 is optional: no valid 0x62
  within 3 s means "no alarm" (the alarm latch of stock is not copied).

## 5. Limits and the stale fallback

`Limits` gains `bool stale` ("a non-panel source is configured but not fresh") and `SourceMode source` (the source
that produced it), which the controller copies into `Status` for IR 4 bit 2 and IR 5.

- **Fresh:** `cvl`, `ccl`, `allowed`, `alarm`, `soc`, `battery_v`, `max_cell_v` from §3, `battery_a` NaN, `blocking`
  false, `fresh` true, `stale` false. The controller's existing caps apply: target V = min(CVL, Charge V, 58.0 V),
  budget from min(Max current, CCL).
- **Stale** (including before the first reply after boot or a mode change): Panel limits with `ccl` =
  min(Max current, Fallback current HR 9), `fresh` true, `stale` true. HR 9 = 0 therefore means "no charging without the
  BMS". The controller reports **E15** (source stale, a warning) from `stale`, replacing today's `!fresh` test.
  A BMS stop is not overridden: if the last fresh state had charge enable off or an alarm, the stale limits keep
  `allowed` false (still `stale`, E15) until the BMS is fresh again. A mode change forgets it (the poller resets).
- SOC known → the session gate's Start/Stop SOC thresholds apply (existing behaviour).

## 6. Mode switching

- At boot, and whenever HR 6 changes (the menu saves settings), the RS485 role follows HR 6: Pylontech → stop the Modbus
  server (drop its receive buffer), reset the poller (stale until fresh replies arrive), line at 9600; Panel or Modbus →
  stop the poller, Modbus server at the HR 8 baud, as today.
- IR 5 (active source) reports 1 while Pylontech is active, and IR 4 bit 2 (source stale) mirrors `stale`. These are
  only readable in Panel mode today, but keep the register map truthful.

## 7. Testing

- Host (`make -C tests`):
  - framing: LCHKSUM/LENGTH (`D012`, `0000`), CHKSUM (`FDAB` for stock's 0x60 request), the three request strings, reply acceptance and each
    rejection (bad LCHKSUM, bad CHKSUM, wrong CID1, RTN ≠ 0, length mismatch, non-hex, too short for the command);
  - decoding: the synthetic vectors in `docs/history/notes/pylon-stock-protocol.md` and frames built like the proxy's encoder;
  - poller: request cadence and order, 400 ms reply window, fresh/stale timing, mode change resets;
  - limits: fresh mapping, stale fallback (Fallback current cap, E15, HR 9 = 0 holds), charge-enable off → not allowed,
    alarm → E12.
- `tools/fake_pylon.py`: a BMS simulator on a serial port or pyserial URL (so through the bridge's Passthrough) that
  answers 0x61-0x63 with settable values (voltage, SOC, max cell, CVL, CCL, charge enable, alarm) and can go silent; its
  encoder unit-tested against the same vectors.
- Live, on one charger: bridge Passthrough on, the simulator on the Mac plays the BMS; the user switches Source mode to
  Pylontech on the panel; cases: normal (charges within CCL, target ≤ CVL), CCL lowered, CVL lowered, charge enable off,
  alarm, silence → E15 and fallback current; the user switches back to Panel. Optionally the user's YamBMS proxy as the
  real BMS.
