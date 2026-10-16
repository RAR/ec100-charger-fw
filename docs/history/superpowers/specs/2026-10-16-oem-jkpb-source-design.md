# OEM firmware: JK-PB BMS limit source

**Status:** design, approved in chat on 2026-10-16. Firmware 3.9. Host-tested and simulator-tested only until someone
connects a real JK-PB pack (the user has none). Audience: other users with JK-PB packs and no YamBMS (the user's own bank
reaches the chargers through Source mode 2).

Protocol source: `docs/history/notes/jk-pb-protocol.md` (from syssi/esphome-jk-bms, txubelaxu's PB work, JK's Modbus V1.0 document and
captures in the PH README), with citations and real test vectors.

## 1. Decisions

- **Source mode 5, JK-PB.** Same kind of source as Pylontech, EG4 and JK-B: the board is the RS485 master, there is no
  Modbus server in this mode, and it can only be selected from the menu. HR 6 Modbus writes refuse 5. Menu item 7's range
  becomes 0-5. IR 5 reads 5.
- **Wiring:** the charger is the only master on one pack's **RS485-1** port. The user sets that port to UART protocol
  **001** (JK RS485 Modbus V1.0) at **115200 8N1**. In a parallel bank: the DIP-0 master pack's RS485-1. The charger then
  sees that pack only (documented). The charger never talks on RS485-2 (the parallel bus).
- **Address discovery, no setting.** The pack answers only on its DIP address (0-15; 0 is valid). The poller tries 0, then
  1..15 in turn (one cell-info request per address), and locks on the first address that gives a valid record. After 3
  missed polls in a row on the locked address it scans again, starting from the locked address. Address 0 is not treated
  as a broadcast.
- **Requests** (FC 0x10, one register, value 0x0000, CRC-16/MODBUS low byte first):
  - cell info: register **0x1620** → a 300-byte record `55 AA EB 90 02 ..` then an 8-byte FC10 reply
    `AA 10 16 20 00 01 CRC`;
  - settings: register **0x161E** → `55 AA EB 90 01 ..` then `AA 10 16 1E 00 01 CRC`.
  E.g. address 1 cell info `01 10 16 20 00 01 02 00 00 D6 F1`, settings `01 10 16 1E 00 01 02 00 00 D2 2F`; address 0
  `00 10 16 20 00 01 02 00 00 DB 61`, `00 10 16 1E 00 01 02 00 00 DF BF`.
- **Reply validation:** header `55 AA EB 90`, the expected type byte, byte 299 = sum of bytes 0..298 mod 256. The trailer
  must follow, with the polled address, function 0x10, the polled register and a good CRC. Anything else is discarded
  (resync on the next `55 AA EB 90`). A reply counts within **750 ms** of its request (no source gives a latency).
- **Polling:** one request per **1 s**. A settings request on first contact (after a lock) and then every **60 s**; all
  other polls ask for cell info. Fresh = a valid cell-info record within **3 s** AND a valid settings record since the
  lock. Not fresh → E15, Panel limits capped at Fallback current, stop latch (as the other polled sources).
- **Cell-info fields** (record byte = 6 + register offset, little-endian): cells u16 mV at 6 + 2n (n = 0..31, 0 = absent);
  pack voltage u32 mV at 150; alarms u32 at 166; SOC u8 at 173; charge MOSFET on u8 at 198; charge stage u8 at 280
  (0 bulk, 1 absorption, 2 float; syssi only).
- **Settings fields** (u32 LE): CellRCV mV at 38, CellRFV mV at 42, CurBatCOC (max charge current) mA at 50, cell count at
  114, charge switch setting at 118.
- **Limits:**
  - CVL = cell count × CellRCV; when charge stage = 2 (float), cell count × CellRFV. The panel's Charge V and the 58.0 V
    ceiling still cap it. A cell count outside 1-32 or a per-cell voltage outside 2.5-4.5 V makes the settings record
    invalid (not fresh).
  - CCL = CurBatCOC / 1000 A, capped by Max current as always.
  - allowed = charge MOSFET on (byte 198 = 1) AND charge switch setting = 1 AND no stop alarm AND not cell OVP.
  - CurBatCOC 0 counts as a stop: allowed false, latched (as EG4/Modbus), like a settings record with the switch off.
  - alarm (E12) = alarms & **0x00E107EF** ≠ 0: bits 0-3 (wire resistance, MOSFET OT, cell count, current sensor), 5-10
    (pack OVP, charge OCP, charge SCP, charge OT, charge UT, internal comm), 16 (charge MOSFET abnormal), 21 (battery OT),
    22 (temp sensor), 23 (parallel module). Never cell/pack UVP or the discharge bits; bit 19 ("modify password", set in
    the captures) is ignored.
  - **Bit 4, cell OVP:** fires at every normal full charge, so it makes allowed false but is not an alarm (no E12).
  - The stop latch is set by any valid cell-info record that says stop (any of the above), even while not fresh; only a
    fresh, fine state clears it; a mode change forgets it.
  - SOC from byte 173; pack voltage as the battery voltage in use; max cell = max of the present cells (0 and ≥ 5 V
    ignored), for the existing max-cell guard. Battery current is not used (NaN), as with the other BMS sources.
- **Trigger on RS485-1:** confirmed 2026-10-16 on a JK-PB2A16S20P sw 19.34 (protocol 001, DIP 4): both records come back as
  300 + 8 bytes (`docs/history/notes/jk-pb-capture-rs485-1.txt`). The FC 0x03 fallback is not needed.
- **Tools:** `tools/fake_jkpb.py` simulator (serial port or `--bridge`, Passthrough at 115200; DIP address, settable values,
  silence; replays the captured records as templates). `oem_modbus.py` and the bridge's Source mode select name mode 5.

## 2. Testing

- Host: request frames for addresses 0 and 1 match §1; the captured cell-info record (TV1, slave 01) decodes to 16 × 3.268 V,
  52.287 V, SOC 64 %, alarms 0x00080000, charge MOS 1; the captured settings record (TV3) to RCV 3600, RFV 3500,
  40 000 mA, 16 cells, switch 1; rejections (bad sum, wrong type, wrong trailer address, missing trailer, truncated);
  address scan (0 silent, 1 answers → locked on 1; re-scan after 3 misses); poll cadence (settings first, then every 60 s);
  freshness (needs settings; 3 s); limits (CVL 57.6 / float 56.0, CCL 40, alarm mask → E12, bit 4 stops without E12,
  UVP doesn't stop, bit 19 ignored, latch). Controller: E12 and E15 through the controller. modbus_map: HR 6 refuses 5.
- Simulator unit-tested against the same vectors.
- Live: one charger with `fake_jkpb.py` through the bridge's Passthrough, the user reading the panel, as the 3.7 test.
