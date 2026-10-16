# JK-B RS485/UART protocol (0x4E 0x57 frames) for a master charger

Source: third-party clone `syssi/esphome-jk-bms` at
`/private/tmp/claude-501/-Users-rar-eco-charger/2718133d-2bcc-4a2a-8752-fca8dedceecf/scratchpad/ext/esphome-jk-bms`
(paths below are relative to it). `JM` = components/jk_modbus/jk_modbus.cpp, `JB` = components/jk_bms/jk_bms.cpp,
`PD` = docs/protocol-design-uart-ttl-gps-port.md, `PDF1` = docs/"Communication protocol between monitoring platform and BMS.pdf",
`PDF2` = docs/"RS485 Communication example.pdf". Where I say VERIFIED I recomputed it myself.

## 1. Physical / session

| Item | Value | Source |
|---|---|---|
| Baud | 115200 (ESPHome examples set only baud_rate, so 8N1 by ESPHome default; not stated in repo) | esp32-example.yaml:47-52, PDF1 sec.2 |
| Who initiates | Master polls; BMS replies. Spec also allows BMS "active upload" (type 2) and "broadcast regularly", but ESPHome never relies on it | PDF1 sec.4.1; JM:136-170 comments |
| Wake | Command word 0x01 "activation" must be sent first if BMS is asleep; ESPHome never sends it, it just polls. Treat first poll(s) after idle as possibly unanswered | PDF1 sec.4.2.4; JB:16,392-400 (offline after 5 missed polls) |
| Addressing | None on the wire for reads: "terminal number" bytes 4-7 are sent as 00 00 00 00. The `address` in ESPHome (default 0x4E) is just raw[0] used to route frames to a component, not a bus address | JM:138-145 vs JM:72,113; components/jk_bms/__init__.py:100 |
| Min gap / max reply | Packets >= 100 ms apart; reply within 5 s worst case | PDF1 sec.4.1 |
| Observed reply latency | ~93 ms (send 17:15:40.815, reply 17:15:40.908) | PDF2 log |
| Request interval used | `update_interval: 5s` | esp32-example.yaml:62 |
| RX framing timeout | 50 ms inter-byte; buffer flushed after that | JM:34-39, jk_modbus/__init__.py:26 |
| TX turnaround | optional DE/RE GPIO: high before write, `flush()`, low after. Reply latency (~90 ms) is far above turnaround needs | JM:206-213 |
| Reply size | 285 bytes for 14 cells, 269 for 20 cells (grows 3 B/cell). At 115200 8N1 = ~25 ms | PD, PDF2 |

RS485 vs TTL: the repo only ever talks UART-TTL (3.3 V logic) to the JK **4-pin JST 1.25 mm "GPS/UART-TTL" socket**
(pins GND, RX, TX, VBAT; cross TX/RX). README says "some JK-BMS units label this socket `RS485`" (README.md:96-110).
README never mentions a JK RS485 adapter. docs/display-port.md:25-57 shows a separate 4-pin `RS485` connector next to
the display port on several models, but the diagram is ambiguous (labels A/B/VCC/GND appear on the 6-pin display
connector in the drawing). The flow_control_pin option (JM:206) shows RS485 transceivers have been used with the same bytes.
UNVERIFIED: whether the BMS's "RS485" port is electrically RS485 A/B at 115200 with identical framing, or needs the
JK RS485 adapter (that adapter is presumably a TTL<->RS485 converter). Check the user's actual model manual/port before wiring.
Do NOT confuse with docs/RS485-english-OBSOLETE.pdf: that is a different protocol (0xDD...0x77, 9600 baud, Xiaoxiang-style) and not the 4E 57 one.

## 2. Request frame (read all)

21 bytes, built at JM:180-214. Length field = frame length - 2 (counts everything after the 2 start bytes, including itself and the checksum; PDF1 sec.4.2.2).

| Idx | Byte | Meaning | Source |
|---|---|---|---|
| 0-1 | 4E 57 | start | JM:182-183 |
| 2-3 | 00 13 | length = 19, **big-endian** (ESPHome comments "lb/hb" are swapped) | JM:184-185, 93 |
| 4-7 | 00 00 00 00 | terminal number | JM:186-189 |
| 8 | 06 | command: 0x06 read all (0x01 activate, 0x02 write, 0x03 read one, 0x05 password) | JM:17-20, 190 |
| 9 | 02 | frame source GPS (0 BMS, 1 BT, 2 GPS, 3 PC) | JM:24, 192 |
| 10 | 00 | transport type: 0 request, 1 reply, 2 active upload | JM:193 |
| 11 | 00 | register id (0x00 = all) | JM:194 |
| 12-15 | 00 00 00 00 | record number | JM:195-198 |
| 16 | 68 | end byte | JM:199 |
| 17-18 | 00 00 | unused (CRC16 high part, not enabled) | JM:201-202 |
| 19-20 | crc_hi crc_lo | **plain 16-bit sum** of bytes 0..16, big-endian | JM:54-60,200,203-204 |

Checksum algorithm: `sum = 0; for b in bytes[0 .. LEN-1]: sum += b; sum &= 0xFFFF` where LEN = value in length field
(= number of bytes before the 2 checksum bytes). Not a CRC.

**Request, as documented (VERIFIED sum 0x0129 = 4E+57+13+06+03+68), byte 9 = 03:**
`4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29`
Seen in PD line 4, JB:71 and the PDF2 capture (real BMS answered it).
**Request as ESPHome actually sends it (byte 9 = 02 = GPS source, JM:192, sum 0x0128):**
`4E 57 00 13 00 00 00 00 06 02 00 00 00 00 00 00 68 00 00 01 28`
The repo is inconsistent on the source byte (03 in docs/captures, 02 in code); both evidently work, but I could not confirm
that from the repo for every firmware. Use 03 (matches the real capture) and fall back to 02 if no reply.

Write path (not needed for a read-only charger): password frame cmd 0x05, 150 ms delay, then cmd 0x02 22-byte frame (JM:136-178).

## 3. Reply framing

| Idx | Content | Source |
|---|---|---|
| 0-1 | 4E 57 | JM:67-84 |
| 2-3 | length N, big-endian (0x011B = 283 for 14 cells). Total frame = N + 2 | JM:93,96 |
| 4-7 | terminal number (00 00 00 00 seen) | PD line 8 |
| 8 | command echo (06) | JM:99 |
| 9 | frame source (00 = BMS) | PD |
| 10 | transport type 01 = reply | PD |
| 11 .. N-4 | ID-tagged records (section 4), then 4 bytes record number | JM:109 |
| N-3 | 68 end byte | PD tail `00 00 00 00 68 00 00 54 D1` |
| N-2..N-1 | 00 00 (unused) | |
| N, N+1 | checksum hi, lo = 16-bit sum of bytes 0..N-1 | JM:102-103 |

Validate: header 4E 57; read length; wait for N+2 bytes (timeout 50 ms between bytes); sum bytes[0..N-1] and compare with
bytes[N]<<8|bytes[N+1]; check command echo 0x06; discard on any failure (JM:96-107). ESPHome passes records starting at
byte 11 up to index N-4 inclusive of the 4 record-number bytes (JM:109, slice end is exclusive data_len-3) so
trailing 4 bytes are just ignored.
Checksum VERIFIED on the 14S frame below (sum of 283 bytes = 0x54D1).

## 4. Record layout (starts at byte 11)

Records are `ID, value...` with fixed per-ID lengths, in ascending ID order, except 0x79 which is `79, LEN, (idx, hi, lo)*`.
**ESPHome decodes by fixed byte offsets** (`offset = data[1] + 3` etc., JB:130 onward) and requires
`size >= data[1] + 223` (JB:97), so it rejects short/older frames (e.g. the 20S PDF2 frame has no 0xB8..0xC0 and would fail).
A charger MCU should walk the IDs with the length table instead. All multi-byte values are big-endian.

| ID | Len | Meaning / scaling / sign | Source |
|---|---|---|---|
| 79 | 1+3n | LEN = 3*cells; each item: cell idx (1-based), mV hi, lo. mV, unsigned. max cell V = max of all items | JB:95,107-121 |
| 80 | 2 | MOSFET temp | JB:132-134 |
| 81 | 2 | battery box temp | JB:136-137 |
| 82 | 2 | battery temp (sensor 2) | JB:139-140 |
| 83 | 2 | pack voltage, 0.01 V, unsigned (14EF = 53.59 V) | JB:142-144 |
| 84 | 2 | current, see below | JB:146-149 |
| 85 | 1 | SOC %, 0-100 | JB:156-158 |
| 86 | 1 | number of temp sensors | JB:160 |
| 87 | 2 | cycle count | JB:163 |
| 89 | 4 | total cycle capacity | JB:166 |
| 8A | 2 | cell (string) count | JB:169-170 |
| 8B | 2 | alarm bitmask | JB:172-198 |
| 8C | 2 | status bitmask | JB:200-213 |
| 8E | 2 | total voltage OVP, 0.01 V (1626 = 56.70 V) | JB:215-217 |
| 8F | 2 | total voltage UVP, 0.01 V | JB:219-221 |
| 90 | 2 | cell OVP (trip), mV (0FD2 = 4050) | JB:223-225 |
| 91 | 2 | cell OVP recovery, mV (0FA0 = 4000) | JB:227-229 |
| 92 | 2 | cell OVP delay, s | JB:231 |
| 93 / 94 / 95 | 2 each | cell UVP mV / UVP recovery mV / delay s | JB:234-243 |
| 96 | 2 | cell delta protection mV | JB:245 |
| 97 / 98 | 2 each | discharge OC protection A / delay s | JB:249-253 |
| 99 | 2 | charge overcurrent protection **trip value, A** (integer) | JB:255-256 |
| 9A | 2 | charge OC delay s | JB:258 |
| 9B..9D | 2,2,1 | balance start mV, balance delta mV, balance switch | JB:261-269 |
| 9E..A2 | 2 each | MOSFET OT trip/recovery, box OT trip/recovery, temp diff (degC) | JB:271-285 |
| A3 | 2 | **charging high-temp protection**, degC | JB:287-288 |
| A4 | 2 | discharge high-temp protection | JB:290 |
| A5 / A6 | 2 each | **charging low-temp protection / recovery**, int16 degC (FFEC = -20) | JB:293-299 |
| A7 / A8 | 2 each | discharge low-temp protect / recovery, int16 | JB:301-307 |
| A9 | 1 | cell count setting | JB:309 |
| AA | 4 | capacity setting, Ah (note JB:311 comment says 560 Ah but frame says 0x0E = 14) | JB:311-313 |
| AB | 1 | **charging MOSFET switch** setting (0/1) | JB:316-317 |
| AC | 1 | discharge MOSFET switch | JB:319-320 |
| AD | 2 | current calibration, mA | JB:322-323 |
| AE | 1 | protection board address (reserved for cascading, PD register table) | JB:325 |
| AF | 1 | battery type 0 LFP, 1 ternary, 2 LTO | JB:328-335 |
| B0 | 2 | sleep wait s | JB:337 |
| B1 | 1 | low-SOC alarm threshold % | JB:340 |
| B2 | 10 | password | JB:343 |
| B3 | 1 | dedicated charger switch | JB:348 |
| B4 | 8 | device id; B5 4 mfg date; B6 4 runtime min; B7 15 sw version | JB:351-367 |
| B8 | 1 | start current calibration | JB:369 |
| B9 | 4 | actual capacity, needs fw >= 10.10 | JB:370-373 |
| BA | 24 | manufacturer id string | JB:375-379 |
| C0 | 1 | protocol version (selects 0x84 encoding) | JB:381-382 |

**There is no field for "max charge current limit" or "charge voltage setpoint" in this register map.**
Only protection trip values (90,91,8E,99) and the temperature windows exist (PD register table).

### 0x84 current (and bit 15)
- Value = 16-bit, unit 0.01 A. With protocol version 0xC0 == 0x01: **bit 15 = 1 means charging (+), 0 means discharging (-)**; magnitude = value & 0x7FFF. Charging 20 A = 0x87D0 (34768), discharging 20 A = 0x07D0 (PD 0x84 and 0xC0 rows; jk_bms.h:353-364).
- ESPHome returns **0 A if C0 != 1** (jk_bms.h:353-356) and it reads C0 at a fixed offset (JB:148). Older firmware (no C0, e.g. PDF2 frame): 0x84 = 10000 + offset encoding: "(10000-11000)*0.01 = -10.00 A discharge, (10000-9500)*0.01 = +5.00 A charge" (PD 0x84 row), i.e. A = (10000 - raw)*0.01 with charge positive. Example PDF2 `84 27 10` = 10000 = 0 A. That older formula is not implemented in the repo.
- Sign convention after decode: positive = charging, negative = discharging (JB:151-154).

### Temperature encoding (0x80-0x82)
Spec: 0-100 degC literal; above 100 is negative as `100 - raw` (101 = -1 degC) (PD 0x80 row, JB:133).
Code uses `99 - raw` for raw > 99 (jk_bms.h:346-351), which disagrees with the spec by 1 degC at/below zero. Treat as a known inconsistency; use the spec (100 - raw) unless a real BMS shows otherwise.

### 0x8B alarm bitmask (bit n = 1 means active) (JB:172-189, PD ^1, components/jk_bms/__init__.py:36-52)
| Bit | Alarm | Stop charging? (my judgement; repo only labels severity) |
|---|---|---|
| 0 | Low capacity (warning) | no |
| 1 | MOSFET overtemp | yes |
| 2 | Charge overvoltage | **yes** |
| 3 | Discharge undervoltage | no (but consider: pack is deeply discharged) |
| 4 | Battery overtemp | yes |
| 5 | Charge overcurrent | **yes** |
| 6 | Discharge overcurrent | no |
| 7 | Cell delta | yes (conservative) |
| 8 | Box overtemp | yes |
| 9 | Battery undertemp | **yes** (charging below A5) |
| 10 | Cell overvoltage | **yes** |
| 11 | Cell undervoltage | no / hold low current |
| 12,13 | "309_A protection" (unknown) | yes (unknown = fail safe) |
| 14,15 | reserved | ignore |
Suggested stop mask: bits 1,2,4,5,8,9,10,12,13 = **0x3736** (derived). Bit 7 (cell delta) is left out on purpose: it would strand an imbalanced flat pack that must still charge; the top of charge is covered by bit 10 (cell OV), 0x8C bit 0 and the firmware's max-cell guard. (The first derivation, 0x37B6, included it.)

### 0x8C status bitmask (JB:200-213, PD 0x8C row)
bit0 charging MOSFET on, bit1 discharging MOSFET on, bit2 balancer on, bit3 1 = battery online ("dropped" if 0). Bits 4-15 reserved.
Caveat: JB:211-213 passes bit values 1,2,4 to `check_bit_(mask, flag)`, which is correct bit0/1/2. In the PDF2 frame 8C = 000B (bits 0,1,3) while 0xAB = 00 (switch setting off), so 8C (actual state) and AB (command/setting) can disagree on some firmware: use 8C bit0 as the actual state.

### Firmware/hardware differences
- Older firmware (PDF2: "NW_1_0_0_200428", 20S): no 0xB8/B9/BA/C0 tail; 0x84 offset encoding; 0xAA only 40 Ah.
- 0xB9 needs fw >= 10.10 (JB:371). C0 selects 0x84 encoding. Number of cells varies, so every offset after 0x79 shifts by 3/cell.
- README lists hw 10.xW/11.xW, UART-TTL working on JK-BD6A*, JK-B1A20S15P, B2A24S15P, B2A8S20P etc (README.md:25-65).

## 5. Test vectors

**TV1 (VERIFIED checksum 0x54D1, length 0x011B, 285 bytes)** from PD lines 6-90; same data as tests/components/jk_bms/common.h `STATUS_FRAME_14S` (records only, 265 bytes, starts at 0x79).
```
4E57011B00000000060001792A010EED020EFA030EF7040EEC050EF8060EFA070EF1080EF8090EE30A0EFA0B0EF10C0EFB0D0EFB0E0EF280001D81001E82001C8314EF8480D0850F860287000489000000008A000E8B00008C00078E16268F10AE900FD2910FA0920005930BEA940C1C95000596012C9700079800039900059A00059B0CE49C00089D019E005A9F0046A00064A10064A20014A30046A40046A5FFECA6FFF6A7FFECA8FFF6A90EAA0000000EAB01AC01AD0411AE01AF01B0000AB114B231323334353600000000B300B4496E707574205573B532313031B60000E200B748362E585F5F53362E312E33535F5FB800B900000000BA425433303732303230313230303030323030353231303031C0010000000068000054D1
```
Decoded: 14 cells 3.821 3.834 3.831 3.820 3.832 3.834 3.825 3.832 3.811 3.834 3.825 3.835 3.835 3.826 V (min 3.811 #9, max 3.835 #12, delta 24 mV);
80=29 degC, 81=30, 82=28; 83=14EF=53.59 V; 84=80D0 with C0=01 -> +2.08 A (charging); 85=0F=15 %; 86=2; 87=4 cycles; 8A=14 cells;
8B=0000 no alarms; 8C=0007 (charge on, discharge on, balancer); 8E=1626=56.70 V; 8F=10AE=42.70 V; 90=0FD2=4.050 V; 91=0FA0=4.000 V;
99=0005=5 A charge OC; A3=0046=70 degC; A5=FFEC=-20 degC; A9=0E; AA=14 Ah; AB=01; AC=01; AF=01 ternary; B3=00; B7="H6.X__S6.1.3S__"; C0=01.
Expected unit-test outputs also in tests/components/jk_bms/jk_bms_test.cpp:11-230 (e.g. current 2.08, :73) and header comment in common.h.

**TV2 (decode only, checksum does NOT validate)**: 20-cell older-firmware frame from PDF2, length 0x010B, 269 bytes, trailer says 4923 but my sum is 0x485C (diff 0xC7; PDF text transcription or edited sample). Do not use as a checksum vector.
Key decode: 20 cells ~3.8 V; 83=1DBC=76.12 V; 84=2710=10000 (0 A, old encoding); 85=47=71 %; 8A=0014; 8B=0000; 8C=000B; 8E=20D0=84.00 V; 8F=15E0=56.00 V; 90=1068=4.200 V; 91=1036=4.150 V; 93=0AF0=2.800 V; 99=0014=20 A; AA=40 Ah; AB=00; AF=01.
Hex (only if you want a parser/tolerance test): see PDF2 text; request bytes there: `4E 57 00 13 00 00 00 00 06 03 00 00 00 00 00 00 68 00 00 01 29`.

No standalone error/alarm frames exist in the repo; build 8B/8C cases by editing TV1 and recomputing the sum (tests/components/jk_bms/jk_bms_errors_test.cpp covers bit decoding; tests/components/jk_modbus/jk_modbus_test.cpp covers bad header/CRC/partial frames, helper `make_frame`).

## 6. Recommended mapping to charger parameters

| Charger input | Source | Kind |
|---|---|---|
| Pack V | 0x83 * 0.01 | BMS value |
| SOC | 0x85 | BMS value |
| Max cell V | max of 0x79 items (mV) | BMS value |
| Cell count N | 0x8A; cross-check 0x79 LEN/3 and 0xA9; reject if they differ | BMS value |
| Charge allowed | 0x8C bit0 AND (bit3 NOT used: TV1, a healthy BMS, reports 0x0007) AND (0x8B & stop_mask)==0 AND fresh reply AND temps inside [A5 recovery.. A3] | **derived** |
| Alarm | (0x8B & stop_mask) != 0, or no valid reply for > ~15 s (3 polls) | derived |
| CVL | `min(user_chem_max, N * 0x91, 0x8E*0.01)`; refine: fold CVL down if max cell V >= (0x90 - margin e.g. 50 mV) | **derived**, BMS gives no setpoint. In TV1, 0x8E == N*0x90 (56.70 = 14*4.05) |
| CCL | `min(user_max, k * 0x99)` with k ~0.8; 0 when charge not allowed; taper by temperature and max cell V | **derived**, 0x99 is a trip threshold, not a limit |

Reasoning: JK exposes only protection thresholds. Charging up to cell OVP recovery (0x91) is below the point where the BMS re-closes the
MOSFET, so a charger holding it keeps clear of trips; charging to 0x90 risks tripping. Always cap by the user's chemistry-specific
setpoint (LFP typically far below 4.0). Use max cell V as a closed-loop guard since the BMS opens the MOSFET (output goes dead) on trip,
which a CC/CV charger will see as a load dump. Also treat reply gaps: ESPHome flags offline after 5 missed 5 s polls (JB:16, 392-400).
Charge-side fields A3, A5/A6 give the allowed temperature window; the repo's temperature decode is off-by-one for negatives (see section 4).

## 7. JK-PB (inverter BMS) series

Different protocol: the `esp32-jk-pb-modbus-example.yaml` uses ESPHome `modbus_controller` (real Modbus RTU framing, not 4E 57), 115200 baud,
slave address 1-15 set by DIP switches (0 = master mode, do not use), byte-offset register addresses like 0x1000/0x1004/0x1114,
15 s poll, optional DE/RE flow_control_pin (lines 7-14, 27-31, 70-100, 125-148). It says explicitly "JK Modbus protocol is not the
same as standard Modbus RTU" (line 7-9). It lives in this same repo (`modbus_controller` YAML), not in `jk_modbus`; the README has no
jk_rs485_bms mention. Not researched further.
