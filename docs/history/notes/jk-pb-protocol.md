# JK-PB ("inverter BMS") RS485 protocol notes for a charger (master or listener)

Covers JK-PB1A16S15P / PB2A16S15P / PB2A16S20P class packs (hw 14.x / 15.x / 19.x). This is a **different protocol from the
JK-B 0x4E 0x57 frames** in `jk-bms-protocol.md`. The PB uses Modbus RTU framing. Bulk data comes back as the same 300-byte
`55 AA EB 90` "JK02_32S" record that the PB sends over BLE.

Sources were cloned read-only under
`/private/tmp/claude-501/-Users-rar-eco-charger/2718133d-2bcc-4a2a-8752-fca8dedceecf/scratchpad/research/jkpb/`. Nothing was
built or run from them. The two scripts that verified the CRCs and checksums are in `../scratchpad/scripts/` (`jkpb_decode.py`,
`crcgen.py`, run with `python3 -I`).

| Tag | Source |
|---|---|
| `SY` | `esphome-jk-bms/` = syssi/esphome-jk-bms @25822a8 (2026-10-14) |
| `PBY` | `SY/esp32-jk-pb-modbus-example.yaml` (ESPHome `modbus_controller`, FC03 register reads) |
| `BLE` | `SY/components/jk_bms_ble/jk_bms_ble.cpp` (JK02_32S 300-byte record decoder) |
| `MB10` / `MB11` | JK's own "极空BMS RS485 Modbus通用协议" V1.0 / V1.1: `SY/docs/pb2a16s20p/JK_BMS.RS485.Modbus.v1_0.pdf`, `.../BMS RS485 Modbus V1.1.pdf`. Cited as `pdftotext -layout` line numbers |
| `TX` | `txubelaxu-jk-bms/` = txubelaxu/esphome-jk-bms @10ce725 (2026-08-19). `TXS` = `components/jk_rs485_sniffer/jk_rs485_sniffer.cpp`, `TXB` = `components/jk_rs485_bms/jk_rs485_bms.cpp`, `TXR` = `components/jk_rs485_bms/README.md` |
| `PBM` | JK "User Manual for PB series" (`TX/docs/JK-BMS-User-Manual-for-PB-series-jkbms.com_.pdf`, text lines) |
| `PH` | `phinix/` = phinix-org/Multiple-JK-BMS-by-Modbus-RS485 @247d4f2. `PH/README.md` holds a **real RS485-2 bus capture** |
| `#n` | GitHub issue or discussion n on syssi/esphome-jk-bms |

Grades: **[V]** = vendor doc says so. **[C]** = shown by code or a real capture (CRC/checksum recomputed by me where marked
VERIFIED). **[G]** = guess or inference. **[F]** = field report in an issue, not checked by me.

---

## 0. Answers in brief

1. **Read a pack, recommended way: bulk trigger.** Send Modbus FC 0x10 "write 1 register at 0x1620, value 0000":
   `AA 10 16 20 00 01 02 00 00 CRClo CRChi` (address 01 gives `01 10 16 20 00 01 02 00 00 D6 F1`). The pack replies with a
   300-byte `55 AA EB 90 02 ...` cell-info record (8-bit sum checksum at byte 299), then the normal 8-byte FC10 echo
   `AA 10 16 20 00 01 CRC`. Use 0x161E for the settings record (type 01) and 0x161C for device info (type 03). Standard
   FC 0x03 reads of the 0x1000/0x1200 register blocks also work, but their byte order on the wire is not captured anywhere I
   could find (§3.2). **[C]**, capture VERIFIED.
2. All offsets are in §4 (cell info) and §5 (settings). The record is a little-endian copy of the Modbus register block:
   **record byte = 6 + register offset**. The PB has real charge-limit settings: `CellRCV` (request charge voltage per cell),
   `CellRFV` (float), `CurBatCOC` ("continuous charge current", i.e. max charge current), plus OVP and recovery values.
3. **115200 8N1** (Modbus doc). A 9600 variant is selectable on RS485-1 only. Address = 4-position DIP, binary 1/2/4/8.
   Address **0 = parallel-bus master**. On RS485-2 (the parallel bus) a master pack polls continuously, so **the charger must
   not transmit there if a pack has address 0**. Listen passively, or talk Modbus on the pack's **RS485-1** port (protocol
   001), where the charger is the only master.
4. Yes, the PB can be set to **Pylontech LV RS485 V3.5** on RS485-1 (option 005; option 014 = the 9600-baud variant) and to
   **Pylon LV CAN V1.2** on CAN (option 002). Nobody has shown that it answers the 0x60-0x63 system commands with ADR 0x12
   that our stock Pylon master sends at 9600 (`pylon-stock-protocol.md`). **Unverified. It needs a bench test.**

---

## 1. Ports, addressing, roles

| Item | Value | Source |
|---|---|---|
| Electrical | RS485 half duplex, UART 115200 bps, 8 data bits, 1 stop bit, no parity | [V] MB10:7-14 |
| Roles | Strict master/slave. "Only the master can start a request via a unique slave address. The BMS (slave) only responds" | [V] MB10:3-6 |
| Bad address or CRC | Request is silently ignored, no reply | [V] MB10:36-37, 48-51 |
| Exception reply | `addr, func|0x80, code, CRC` with code 01 illegal function, 02 illegal register, 03 illegal data, **04 CRC error** | [V] MB10:35-46 |
| Function codes | only 0x03 read and 0x10 write | [V] MB10:26-28; PH README:"JK-BMS support only commands 0x03 (read) and 0x10 (write multiple registers)" |
| DIP switch | 4 switches. Values 1, 2, 4, 8 summed for the ON positions. Address 0 = master, 1-15 = slaves, addresses must be unique | [V] PBM:65-72, 80-90 |
| Master | "In a parallel battery pack system, one and only one master is required (the device address is set to 0)." Packs talk through RS485-2 | [V] PBM:66-72 |
| RS485-1 | "RS485 interface to communicate with the inverter, optional" (the text is cut off in the PDF) | [V] PBM:108 |
| RS485-2 | "communication interface during parallel op[eration]" (2 RJ45, daisy chain) | [V] PBM:109; TXR:83-105 |
| Other ports | CAN, RS232, dry contacts, ADD (DIP), RST | [V] PBM:95-109 |
| RS485-2 protocol | Fixed. Address 0 gives "UART Protocol 015" (master mode). Address 1-15 gives 001 (Modbus slave). It cannot be selected | [F] PH README:37-44, 94-129 |
| RS485-1 protocol | Selectable list 000-015, see §7. **001 = "JK BMS RS485 Modbus V1.0"** is needed to talk Modbus here | [F] PH README:46-67; [C] SY/components/jk_bms_ble/select/__init__.py:22-44 (register 0xA5 = UART1 protocol, 0xA8 = UART2) |
| RS485-1 slave address | **Follows the DIP address.** Field test: master (DIP 0) answers on Modbus address 0x00 and slave (DIP 1) only on 0x01 | [F] discussion #753 (RaW79 replies), #882 (iamvlanone: ids 0 and 1 on the RS485-1 of each pack). syssi's earlier "fixed 0x00" was refuted there |
| RS485-1 vs CAN | V11 boards: "pick one of two". V15: "all can be used at same time". The settings register 0x1114 bit3 "Port Switch 1: RS485 / 0: CAN" shows a multiplexed port exists on some hardware | [V] PBM:21-31; [V] MB11 (0x0114 bit3) |
| Modbus address 0 | Plain Modbus treats 0 as broadcast, but the JK answers on 0. Some masters refuse it (ESPHome 2026.8 blocks address 0) | [F] #882 (iamvlanone) |
| Firmware updates | Need slave mode, address != 0 | [V] PBM:229-231 |

**Pinout.** RS485-2 RJ45 (either jack): pin 1 = B, pin 2 = A, pin 3 = GND (TXR:94-105) [F, drawn by a contributor]. PH
README warns that "Some available documents and manuals contain incorrect description of ports". Verify pins with a meter
and scope before connecting. A/B naming varies, so swap them if the bus stays silent.

## 2. What happens on RS485-2 (parallel bus)

Real capture from PH README:101-124. Every CRC and checksum below was VERIFIED with `jkpb_decode.py`:

```
master    >>> 55 AA EB 90 02 00 ...(300 B, master's own cell info)...      checksum OK
master    >>> 00 10 16 20 00 01 05 9A          <- FC10 "echo" from address 00, closes the master's own record
master    >>> 55 AA EB 90 01 00 ...(300 B, master's own settings)...
master    >>> 00 10 16 1E 00 01 64 56
master    >>> 01 10 16 20 00 01 02 00 00 D6 F1 <- request: slave 01, cell info
slave  01 <<< 55 AA EB 90 02 05 ...(300 B)...
slave  01 <<< 01 10 16 20 00 01 04 4B          <- slave's FC10 response
master    >>> 01 10 16 1E 00 01 02 00 00 D2 2F <- request: slave 01, settings
slave  01 <<< 55 AA EB 90 01 05 ...(300 B)...
slave  01 <<< 01 10 16 1E 00 01 65 87
master    >>> 02 10 16 20 00 01 02 00 00 C2 01 <- polls 02..0F, no answer (absent)
...       >>> 0F 10 16 20 00 01 02 00 00 9A 91
```

- The master **broadcasts its own records unprompted**, then polls 0x01-0x0F. PH: "During every 15 sec. status of the master
  device is send to the bus" [F]. TX says the master never sends its type-03 record and does not answer type-03 requests
  (TXR:44-45) [F].
- Each master or slave record is followed by an 8-byte FC10 reply. **Its first byte is the sender's address**, which is how a
  listener tells packs apart. The sniffer reads it at byte 300 of the 308-byte unit (TXS:11,18,874-876). For type-01 records
  the address is also inside the record at byte 270 (= 6 + 0x108 DevAddr) (TXS:16,873-874) [C], VERIFIED: master 00, slave 01.
- The master's record holds **its own pack only, not an aggregate.** Master 52.336 V / SOC 65 %, slave 52.287 V / SOC 64 %
  in the capture [C].
- **The charger must not transmit here while a DIP-0 master exists** ([F] #517, #706 syssi: "You aren't allowed to talk to the
  bus. Sniffing the traffic is the only option here"). TX still squeezes requests into gaps of more than 250 ms
  (TXS:20-21,585-596), but has open issues with lost or slow nodes (TX issues #47, #54, #55). #599 reports parallel comms lost
  while polling the master (over BLE).
- If **no** pack has address 0, the bus has no master, and the charger may act as master on RS485-2. TX does this after 2 s of
  silence (TXS:25,588-594, TXR:47) [C]. But then the packs have no master for their own parallel functions, which costs the
  inverter link (#882 danielschramm) [F].
- TX polling timings, a usable starting point [C]: cell info every 5 s per pack, settings every 10 s, device info hourly,
  at least 2.5 s between requests to the same slave, 750 ms between polls, node offline after 10 s silence (TXS:23-33).

## 3. Request frames (charger as master)

CRC is CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF), sent **low byte first** (TXS:72-84 swaps the bytes then sends the
high byte of the swapped value; MB10:65-67 example `01 03 00 05 00 02 D4 0A`). Bytes below VERIFIED with `crcgen.py` and
against the capture.

### 3.1 Bulk trigger (FC 0x10, write 0x0000 to one register) [C]

| Purpose | Register | Addr 0x01 frame | Addr 0x00 frame | Reply record |
|---|---|---|---|---|
| Settings (0x1000 block) | 0x161E | `01 10 16 1E 00 01 02 00 00 D2 2F` | `00 10 16 1E 00 01 02 00 00 DF BF` | `55 AA EB 90 01 ..` |
| **Cell info / live** (0x1200 block) | **0x1620** | `01 10 16 20 00 01 02 00 00 D6 F1` | `00 10 16 20 00 01 02 00 00 DB 61` | `55 AA EB 90 02 ..` |
| Device info (0x1400 block) | 0x161C | `01 10 16 1C 00 01 02 00 00 D3 CD` | `00 10 16 1C 00 01 02 00 00 DE 5D` | `55 AA EB 90 03 ..` |

Layout: addr, 0x10, reg hi, reg lo, qty 00 01, bytecount 02, value 00 00, CRC. Sources: TXS:374-399 (type 1 → 0x1E,
2 → 0x20, 3 → 0x1C), TX/scripts/script_rs485.py:86-89 ("#OK" against a real pack), PH README:420-426 (0x1622/0x1624 give
log record types 06/05).

Reply unit = **308 bytes**: 300-byte record plus 8-byte FC10 response `addr 10 16 2x 00 01 CRClo CRChi` (TXS:9-18). At
115200 that is about 27 ms. Reply latency is not documented. TX uses 750 ms between polls. **[G] timeout 500 ms, retry once,
mark the pack stale after 3 misses or 15 s.**

**Proven on RS485-2 (slave mode), and on RS485-1 with protocol 001 (2026-10-16, PB2A16S20P sw 19.34, DIP 4: `jk-pb-capture-rs485-1.txt`).** PH says writes to 0x161C-0x1624 return records and that "this
method is used by master - slave communication and JK-BMS-MONITOR" (the PC tool, which uses a USB-RJ45 cable, PBM:160-169).
That suggests the same Modbus handler serves RS485-1 with protocol 001. **[G] Test it on the bench. Fall back to §3.2.**

### 3.2 Plain FC 0x03 register reads [V]/[F]

Register address = block base + **byte** offset. The quantity counts 16-bit words ("JK Modbus protocol is not the same as
standard Modbus RTU. The register addresses ... are byte offsets", PBY:7-14). Blocks: 0x1000 settings (RW), 0x1200 live (R),
0x1400 device info, 0x1600 commands (W) (MB10:115-363).

| Purpose | Addr 0x01 frame | Addr 0x00 frame | Reply |
|---|---|---|---|
| Cells 1-32, 64 B | `01 03 12 00 00 20 41 6A` | `00 03 12 00 00 20 40 BB` | 3+64+2 |
| Live 0x1290-0x12C3 (V, I, temps, alarms, SOC, MOS states) | `01 03 12 90 00 1A C1 54` | `00 03 12 90 00 1A C0 85` | 3+52+2 |
| Settings 0x1000-0x1057 | `01 03 10 00 00 2C 40 D7` | `00 03 10 00 00 2C 41 06` | 3+88+2 |

- Field use: PBY reads 0x1290 as U_DWORD, big-endian, high word first, and gets plausible values (#753: "53238.00" →
  53.238 V) [F]. Cells are parsed big-endian per word (PBY:788-794).
- **Byte-pair fields, inferred from PBY's bitmasks** [G]: 0x12A6 = SOC in the low byte (`bitmask 0x00FF`, PBY:1295-1306),
  0x12C0 = charge MOS state in the high byte, discharge in the low byte (PBY:1396-1413), 0x12B8 SOH = first byte
  (PBY:1368-1379). So the FC03 path seems to send each field big-endian, with the first-listed UINT8 first. PBY's own
  "balancing" `bitmask 0x01` on 0x12A6 (PBY:1310-1315) contradicts this. No raw FC03 reply hex exists in any source. **Check
  against a known value (cell count 0x106C = 16) before trusting it.**
- Largest block read known to work: 64 bytes (PBY:775-782). #882 shows a 27-register read at 0x12A6 failing with exception 04
  while another master shared the bus [F]. Keep reads ≤ 32 words.
- A known address-map trap: 0x12AC is the *full charge capacity*, not the design capacity (#1011) [F].

## 4. Cell-info record (type 0x02, 300 B) and 0x1200 register map

Record framing: `55 AA EB 90`, [4] type, [5] frame counter, [6..298] data, [299] = **sum of bytes 0..298 mod 256** (BLE:300-305,
619-627; TXS:13-15). VERIFIED on 4 captured records. All multi-byte values in the record are **little-endian**
(BLE:670-673). Index = 6 + 0x1200 offset, checked field by field against MB11 for all rows below [C].

| Rec idx | Reg | Type | Field | Notes / source |
|---|---|---|---|---|
| 6 + 2n | 0x1200+2n | u16 mV | cell n+1 voltage, n = 0..31 | unused = 0. BLE:734 |
| 70 | 0x1240 | u32 | cell-present bitmask | 0x0000FFFF = 16S |
| 74 / 76 | 0x1244/46 | u16 mV | average / max delta | MB11 |
| 78 / 79 | 0x1248 | u8,u8 | max-V cell no. / min-V cell no. (**0-based**, TXB:574-576 adds 1) | |
| 144 | 0x128A | s16 0.1 °C | MOSFET temperature | BLE:793-795 (112+32) |
| 150 | 0x1290 | u32 mV | **pack voltage** | BLE:803 (118+32) |
| 154 | 0x1294 | u32 mW | power (unsigned, do not use for direction) | BLE:806-810 |
| 158 | 0x1298 | **s32 mA** | **current, + = charging** | BLE:808; PBY:1217-1221 |
| 162 / 164 | 0x129C/9E | s16 0.1 °C | battery temp 1 / 2 | BLE:818-824 |
| **166** | **0x12A0** | **u32** | **alarm bitmask** (table below) | BLE:827-831 (134+32); PBY:1269-1279 |
| 170 | 0x12A4 | s16 mA | balance current | |
| 172 | 0x12A6 lo | u8 | balancer 0 off / 1 charge / 2 discharge | BLE:852-853 |
| **173** | 0x12A6 hi | u8 % | **SOC** | BLE:858 (141+32) |
| 174 | 0x12A8 | s32 mAh | remaining capacity | |
| 178 | 0x12AC | u32 mAh | full-charge capacity | |
| 182 | 0x12B0 | u32 | cycle count | |
| 190 / 191 | 0x12B8 | u8,u8 | SOH % / precharge on | |
| 192 | 0x12BA | u16 | "UserAlarm" (meaning undocumented) | |
| 194 | 0x12BC | u32 s | runtime | |
| **198** | 0x12C0 lo | u8 | **charge MOSFET on (actual state)** | BLE:886 (166+32); MB11 "充电状态Charge 1:open" |
| 199 | 0x12C0 hi | u8 | discharge MOSFET on | BLE:889 |
| 200 | 0x12C2 | u16 | "UserAlarm2" (undocumented) | |
| 206 / 212 | 0x12C8/CE | u16 s | charge-OCP / cell-OVP release countdown | MB11; BLE:898-903 |
| 214 / 215 | 0x12D0 | u8,u8 | temp-sensor present bits (1 = OK) / heating on | |
| 234 | 0x12E4 | u16 10 mV | pack voltage (second copy) | BLE:922 |
| 245 | 0x12EF | u8 | charger plugged | MB11; BLE:926 (213+32) |
| 254/256/258 | 0x12F8.. | s16 0.1 °C | temps 3, 4, 5 (BLE assigns them in reverse order) | BLE:942-948 |
| 280 | (0x1312) | u8 | charge stage 0 bulk / 1 absorption / 2 float | **[C] syssi only**, not in MB11. BLE:960-964 |

### Alarm bitmask, record 166..169 (u32 LE) = 0x12A0 (MB10:275-297, MB11, PBY:1653-1677, BLE errors table `SY/components/jk_bms_ble/__init__.py:46-78`)

| Bit | Meaning (JK doc) | Block charging? [G] |
|---|---|---|
| 0 | wire resistance too high | yes (sensing fault) |
| 1 | MOSFET over-temp protection | yes |
| 2 | cell count does not match setting | yes |
| 3 | current sensor error (syssi now blanks this label) | yes |
| **4** | **cell OVP**. syssi labels it "Battery is fully charged". Expect it at every normal end of charge | **yes (stop/hold)** |
| 5 | pack OVP | yes |
| 6 | charge OCP | yes, retry after 0x12C8 countdown |
| 7 | charge short circuit | yes |
| 8 | charge over-temp | yes |
| 9 | charge under-temp | yes |
| 10 | internal (coprocessor) comm error | yes |
| 11 / 12 | cell UVP / pack UVP | no (charging is the cure) |
| 13-15 | discharge OCP / SCP / OTP | no |
| 16 | charge MOSFET abnormal | yes |
| 17 | discharge MOSFET abnormal | no |
| 18 | GPS disconnected | no |
| 19 | "modify password in time" (**set in the capture**, harmless) | no |
| 20 | discharge on failed | no |
| 21 | battery over-temp alarm | yes |
| 22 | temp sensor anomaly (MB11 lists it without a bit number; syssi says bit 22) | yes |
| 23 | parallel current-limit module fault (MB11 lists it without a bit number; syssi says bit 23) | yes (conservative) |
| 24-28 | SCP release failed, discharge OCP II/III, discharge UT, GPS remote lock (syssi only) | no |

Suggested stop mask [G] = bits 0-10, 16, 21-23 = **0x00E107FF**. Bit 4 means "full / hold CV". Do not latch it as a fault.
**TXB bug:** TX decodes the alarms from bytes 134-136 with no +32 offset (TXB:636-724). In a 16S record those bytes are wire-resistance
slots, so do not copy TX's alarm parsing. syssi (166) and the register map agree [C].

## 5. Settings record (type 0x01) and 0x1000 map, charge-relevant subset

Index = 6 + offset, u32 LE, from MB10:119-196 and BLE settings decoder (e.g. BLE:1227-1240 reads 38 = RCV, 50 = max charge current).
Values are from the captured master record (PH README:103), VERIFIED checksum.

| Rec idx | Reg | Field | Unit | Capture |
|---|---|---|---|---|
| 10 | 0x1004 | cell UVP | mV | 2580 |
| 18 | 0x100C | **cell OVP** (trip) | mV | 3650 |
| 22 | 0x1010 | **cell OVP recovery** | mV | 3580 |
| 30 | 0x1018 | "SOC-100 %" voltage | mV | 3590 |
| **38** | **0x1020** | **CellRCV, recommended (request) charge voltage per cell** | mV | **3600** |
| **42** | **0x1024** | **CellRFV, float voltage per cell** | mV | **3500** |
| **50** | **0x102C** | **CurBatCOC, "continuous charge current" = max charge current** | mA | **40000** |
| 54 / 58 | 0x1030/34 | charge OCP delay / release | s | 3 / 60 |
| 82 / 86 | 0x104C/50 | charge OT protect / recover | s32 0.1 °C | 700 / 600 |
| 98 / 102 | 0x105C/60 | charge UT protect / recover | s32 0.1 °C | 20 / 70 |
| 114 | 0x106C | cell count | | 16 |
| 118 | 0x1070 | charge switch *setting* (not the MOSFET state) | 0/1 | 1 |
| 130 | 0x107C | design capacity | mAh | 314000 |
| 270 | 0x1108 | DevAddr (= DIP) | | 0 (slave: 1) |
| 282 | 0x1114 | bit flags: b0 heat, b3 port RS485/CAN, b5 special charger, b9 **charging float mode** | | 0x3200 |
| (0x1400 block) | 0x1504 | RCV time / RFV time (u8 each, 0.1 h) | | PBY:1564-1589 |

**Charge limits for the charger** [G]: CVL = N × CellRCV (57.6 V here), dropping to N × CellRFV (56.0 V) once the record
reports charge stage float (rec 280 = 2). The 0x1504 timers set how long the BMS holds RCV. CCL = CurBatCOC (40 A here),
0 when not allowed. These are the values JK's own inverter protocols presumably report as CVL/CCL, but I found no code
proving how JK computes its CAN/RS485 CVL/CCL (e.g. any taper near full or cold). Keep the max-cell guard: back off as max
cell nears CellOVP (3.65 V), and never exceed N × CellOVPR.

**Charge allowed** [G] = fresh record (< 15 s) AND rec198 (charge MOS on) == 1 AND (alarms & stop mask) == 0 AND
setting 0x1070 == 1 AND temps inside [ChgUT recover, ChgOT]. The JK-B version of these notes found that the setting and the
actual MOSFET state can disagree, so gate on rec198.

## 6. Test vectors (real captures, PH README:101-110)

Decoded with `jkpb_decode.py`. All sums and CRCs VERIFIED.

**TV1, slave 01 cell info** (`55 AA EB 90 02 05 ... 2D`, response to `01 10 16 20 00 01 02 00 00 D6 F1`, followed by
`01 10 16 20 00 01 04 4B`). 16 cells at 3.268 V (cell 8 = 3.267). Cell bitmask 0000FFFF. MOS 17.4 °C, T1 16.7 °C,
T2 17.2 °C. **Pack 52 287 mV** (rec 150 = `3F CC 00 00`). Current 0 mA. **Alarm 0x00080000** (bit 19 only, "modify
password"). **SOC 64 %** (rec 173 = `40`). Remaining 201 930 mAh / full 314 000 mAh. SOH 100. **Charge MOS 1, discharge
MOS 1** (rec 198/199 = `01 01`). Second voltage copy 5228 × 10 mV. Charge stage 0.
Full hex: PH README line 106. Copy it verbatim into a unit test.

**TV2, master 00 cell info** (PH README:101, trailer `00 10 16 20 00 01 05 9A`). 3.271/3.272 V cells, **52 336 mV**,
**SOC 65 %**, alarm 0x00080000, MOS 1/1.

**TV3, settings** (master PH:103 / slave PH:109): table §5. The slave differs only in ChgUTPR 5.0 °C, MosOT 100/80 °C and
DevAddr 1.

**TV4, BLE copy of a PB2A16S20P hw19A sw19.27 record** with non-zero current and SOC 100 %: SY/esp32-ble-v19-example-faker.yaml
lines 30-82 (settings, cell info). The body format is identical. It is a BLE capture, not RS485.

Negative tests: flip one data byte (sum fails). Feed a 300-byte record without the 8-byte trailer (must time out cleanly).
Feed a master record with trailer address 00 when you expected 01 (must not be attributed to slave 01).

## 7. Inverter protocols on the PB's ports (for "do we need a new protocol at all?")

UART1 (RS485-1) options (`SY/components/jk_bms_ble/select/__init__.py:22-44`, PH README:46-67, same numbering):
000 4G-GPS, **001 JK RS485 Modbus V1.0**, 002 NIU, 003 China tower V1.1, 004 PACE Modbus V1.3, **005 PYLON low-voltage RS485
V3.5**, 006 Growatt, 007 Voltronic, 008 China tower V2.0, 009 WOW (SRNE), 010 JK LCD V2.0, 011/012 user, **013 (9600) JK
Modbus**, **014 (9600) PYLON LV RS485 V3.5**, 015 (internal parallel master), 016-020 newer firmware.
CAN options: 000 JK CAN 250K, 001 Deye, **002 PYLON-Low-voltage-V1.2**, 003 Growatt, 004 Victron, 005 Megarevo, 006 JK CAN 500K,
007 INVT, 008 GoodWe, 009 FSS, 010 MUST, 011 Luxpower.

- That 005 runs at 115200 is **[G]**, inferred from 014 being the explicit "(9600)" variant.
- Our charger's Source mode 1 is a Pylon RS485 **master at 9600, ADR 0x12, CID2 0x60→0x63** (`pylon-stock-protocol.md` §1.1).
  The PB with UART1 = 014 is the natural candidate. **Unverified:** (a) whether JK's V3.5 implementation answers 0x60-0x63
  (the "system" commands) rather than only 0x42/0x44/0x92 per pack, (b) which ADR it answers (0x12 is not confirmed anywhere
  for JK), (c) that only the master pack (DIP 0) speaks for the bank on RS485-1. Bench-test with the stock firmware before
  writing anything new. If it works, JK-PB users need nothing new. They wire RS485-1 to the charger and set UART1 = 014.
- If RS485-1 is used for an inverter, it is not free for a Modbus charger. Then the options are passive listening on RS485-2,
  CAN, or BLE.

## 8. Recommendation and risks

**Recommended:** the charger is the **only master on the RS485-1 port of one pack** (UART1 protocol 001, 115200, Modbus address
= that pack's DIP value). Poll `0x1620` every 2-5 s and `0x161E` every 60 s (or after a change). Parse the 300-byte record by
fixed offsets, check the sum and the trailer CRC and address, and implement §5 for allowed/CVL/CCL. In a parallel bank, connect to
the **DIP-0 master pack's** RS485-1, but note it reports **only its own pack** (§2). For bank-wide cell/SOC data, add a
**receive-only** RS485-2 listener mode: DE held low, frames attributed by trailer address, 308-byte units, no TX while a master
is seen.

**Risks / open items**
1. ~~The 0x1620 trigger is proven only on RS485-2.~~ Shown on RS485-1 with protocol 001 on 2026-10-16 [C]. The FC03 fallback has
   unverified byte order for UINT8 pairs (§3.2).
2. RS485-1 may be taken by an inverter protocol, or multiplexed with CAN on older (V11) boards (0x1114 bit3).
3. Modbus address 0 is non-standard. Our code must allow it.
4. No reply-latency figure from any source. Allow at least 500 ms.
5. RS485-2 pinout and A/B labels come from contributors. Manuals are said to be wrong in places (PH).
6. Alarm bit 4 fires at normal full charge. Bits 22/23 are numbered only by syssi.
7. JK's own CVL/CCL algorithm for inverter protocols is unknown. Our CVL from CellRCV/RFV is an inference.
8. Old-format (non-PB) JK boards use `jk-bms-protocol.md` (4E 57). Detect by model string in the 0x161C record or by which frame
   type answers.
