# OEM firmware: EG4 and JK BMS limit sources

**Status:** design, 2026-10-16, built overnight on the user's invitation ("if you want to do jk bms protocol … and eg4 …
you're welcome to"); rulings made without the user (asleep) are marked **Ruling**; the user confirmed all of them on 2026-10-16. Host-tested
only: no flashing, no pushing until the user is back. Firmware 3.7.

Protocol sources: `docs/history/notes/eg4-bms-protocol.md` (from RAR/esphome-eg4-bms) and `docs/history/notes/jk-bms-protocol.md` (from
syssi/esphome-jk-bms), with file:line citations.

## 1. Decisions

- **Two new Source modes**, like Pylontech (the board is the RS485 master; no Modbus server; menu-only):
  - **3 EG4**: EG4 LL/LifePower battery BMS, Modbus RTU 9600 8N1, slave address **0x10** (a single battery or the
    master battery: DIP switches all on). **Ruling:** fixed address, not a setting (YAGNI; documented).
  - **4 JK**: JK-B BMS "JK RS485/UART" protocol (frames `4E 57`, 16-bit sum checksum), 115200 8N1, through the JK
    RS485 adapter. **Ruling:** the JK-PB inverter series (its own Modbus-style protocol, DIP addresses) is **not** built;
    it is a different protocol, listed as a follow-up.
- **No CVL from either BMS.** Neither sends a charge-voltage limit. **Ruling:** CVL = the panel's Charge V (the user's
  chemistry setting), not a derivation from protection thresholds (OVP-recovery × cells is too close to the trip and
  varies by setup). The existing max-cell guard (HR 12/13, Max cell V / Cell resume V) uses the BMS's max cell voltage.
- **CCL:** EG4 = register 0x0016 "Max charging current" (A; unit assumed by the source repo, as the user's MQTT shows 200
  for these batteries). JK = Max current (Panel): **Ruling:** 0x99 is the charge over-current *trip*, not a limit.
- **Charge allowed / alarm:**
  - EG4: alarm = Protection (0x001B) & **0x0593** ≠ 0 (the repo's bits 0x0513: pack OV, cell OV, charge OC, charge OT, charge UT; plus 0x0080 MOS over-temperature. Never pack/cell UV: a flat battery must charge);
    allowed = !alarm; a valid block B with CCL ≤ 0 also sets the stop latch (as the Modbus source). **Ruling:** the Status word's "protect" nibble is not used (it also covers discharge protections,
    which must not stop charging). Warnings (0x001A) are ignored. A reply with pack voltage 0.00 V is rejected (the
    all-zero reply quirk, source repo issue #7).
  - JK: alarm = 0x8B & **0x3736** ≠ 0 (the notes' charge-relevant protection bits, **Ruling:** without bit 7 cell delta, which would strand an imbalanced flat pack; the top of charge is covered by bit 10 cell OV, 0x8C bit 0 and the max-cell guard; unknown bits 12/13 stay stops); allowed = 0x8C bit 0 (charging on) and !alarm. **Ruling:** bit 3 ("battery dropped(?)" in the source) is not used: the real 14S capture, a healthy BMS, reports 0x0007 (bit 3 clear).
  - The stop latch is Pylontech's: a stale source keeps `allowed` false if the last state was not allowed or an
    alarm; a mode change forgets it. **Ruling:** the latch is set by any valid reply that says stop by itself, even
    while the source is not fresh (EG4: a valid block B with a mask bit set or CCL ≤ 0; Pylontech: a valid 0x63 with
    charge enable off, or a valid 0x62 alarm; JK and Modbus carry everything in one reply/snapshot); only a fresh, fine
    state clears it.
- **Other values:** SOC (EG4 0x0018, JK 0x85), pack voltage (EG4 0x0000 × 0.01, JK 0x83 × 0.01) as the battery voltage
  in use (as Pylontech), max cell voltage (max of the cells, 0 and ≥ 5 V ignored). Battery current is **not** used
  (NaN), as with Pylontech: it is the bank's, not the charger's.
- **Polling and freshness:** one request every 500 ms. EG4 alternates block A (`10 03 00 00 00 12`, pack V, current,
  16 cells) and block B (`10 03 00 12 00 15`, temps … SOC, status, warning, protection); fresh while a valid A and a
  valid B arrived within 3 s. JK: the read-all request every 1000 ms (**Ruling:** gentler than 500 ms for a ~300-byte
  reply); fresh while a valid reply arrived within 3 s. A reply counts within 400 ms of its request (JK: 600 ms).
  Stale → Panel limits capped at Fallback current, E15, with the stop latch (Pylontech spec §5).
- **JK source byte:** the request is sent with source byte 0x03 (the captured frame); after 3 polls in a row without a
  valid reply it alternates to 0x02 (the ESPHome code's), and back. **Ruling** (the repo is inconsistent; both are
  reported to work).
- **JK decode:** walk the ID-tagged records with a per-ID length table (0x79 is variable: `79 LEN (idx hi lo)*`); stop
  at an unknown ID (fields after it are not trusted, those before it are kept). Required for a valid reply: 0x79, 0x83,
  0x85, 0x8B, 0x8C. Checksum = 16-bit sum of all bytes before it; length field = frame length − 2.
- **Mode switching** (main.cpp): the RS485 role generalises from "Pylontech or Modbus server" to "a polled BMS source
  (Pylontech, EG4, JK), each with its baud, or the Modbus server". A small `PolledSource` interface (reset, push, poll,
  limits, baud) lets main.cpp hold one active poller. HR 6 Modbus writes keep refusing every polled mode (1, 3, 4):
  they would cut the link. Menu item 7 range becomes 0-4. IR 5 reports 3 / 4.
- **Tools:** `tools/fake_eg4.py`, `tools/fake_jk.py` BMS simulators (serial port or pyserial URL, settable values,
  silence), like `fake_pylon.py`, for the live test through the bridge's Passthrough (JK needs the bridge baud at
  115200). `oem_modbus.py` names the new sources. The bridge's Source mode select lists EG4 and JK (the board refuses
  them over Modbus).

## 2. Testing

- Host: EG4 frame build/CRC (requests match the notes' hex), reply validation (address, function, byte count, CRC,
  zero-voltage rejection), decode (synthetic vectors), poller cadence/alternation/reply window/freshness, limits
  (fresh, alarm mask, stale fallback, latch). JK: request checksum (`…01 29` for 0x03, `…01 28` for 0x02), the real
  14S reply (TV1 in the notes, checksum 0x54D1) decoded to the notes' values, rejections (checksum, length, truncated,
  unknown ID cut), current sign variants are not needed (current unused), poller and limits as EG4, source-byte
  alternation. Controller: E12 from a BMS alarm, E15 stale. modbus_map: HR 6 refuses 3 and 4.
- Simulators unit-tested against the same vectors.
- Live (when the user is back): one charger, simulator through Passthrough, as the Pylontech live test; then a real EG4
  or JK battery if the user wants.
