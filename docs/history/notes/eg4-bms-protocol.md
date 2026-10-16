# EG4 LL BMS RS485 protocol notes (for a bare-metal master)

Source repo: RAR/esphome-eg4-bms (clone in scratchpad `ext/esphome-eg4-bms`). Paths below are relative to it.
Vendor doc: `docs/EG4-LL-MODBUS-Communication-Protocol.pdf` (V01.06), text-extracted; cited as `PDF:<line>` of `pdftotext -layout` output.

Evidence grades: **[V]** = vendor PDF says so; **[C]** = coded in the repo, which says it follows a "working YamBMS" setup (`eg4_bms.cpp:12`); **[G]** = guess/derived.
The repo itself says **"Pending: Real hardware testing"** (`PROJECT_SUMMARY.md:158`), so nothing is hardware-verified by this repo. The only field-derived items are the sample frames and config values in REGISTER_MAP.md, and the all-zero-reply quirk (issue #7, `eg4_bms.cpp:35-38`).

## 1. Physical and link layer

| Item | Value | Source |
|---|---|---|
| Baud | 9600 default, range 9600-19200 | PDF:31; README.md:329 |
| Framing | 8 data, 1 stop, no parity (8N1) | PDF:26 (start+8+stop, no parity bit shown); README.md:330-332 |
| Initiator | Master = UPS/SMPS/charger; BMS only answers | PDF:36 |
| Reading interval | "Reading time interval: 100 mS" (vendor figure for min spacing) | PDF:33 |
| Interval used by repo | `update_interval` default 10 s, alternating 2 requests; README advises >= 1 s | `eg4_bms/__init__.py:30`; README.md:314 |
| Response timeout in repo | 1000 ms (no byte at all, or mid-frame gap); one request in flight at a time | `eg4_modbus.cpp:13,49-61` |
| Reply latency | Not documented anywhere. Use >= 1 s timeout like the repo, then retry | [G] |
| Inter-frame gap | Repo has none (no 3.5 char silence); it sends back-to-back as soon as the previous reply is parsed | `eg4_modbus.cpp:63-66` [C] |
| Direction control | DE/RE pin driven high around write, flushed, then low | `eg4_modbus.cpp:124-133` |
| Port | RJ45 "RS485 port"; CAN for inverter is a separate port, not on this connector | README.md:27,58 |
| Termination | 120 ohm at both bus ends, common GND | WIRING.md:121,157-160 |

RJ45 pinout (README.md:46-56, WIRING.md:8-24; T-568B colours). Pairs are bridged, use either end:

| Pin | Signal |
|---|---|
| 1 and 8 | RS485 B (-) |
| 2 and 7 | RS485 A (+) |
| 3 and 6 | GND |
| 4, 5 | NOT RS485 (an earlier guide wrongly said 4/5; WIRING.md:72) |

Polarity: A/B naming is vendor-side; if silent, swap (WIRING.md:149-150, 179).

### Addressing (DIP switch, 4 positions)

Valid addresses 0x01-0x10 (PDF:39; PDF:100). Default / master = **0x10** (PDF:39; `eg4_bms/__init__.py:11`; README.md:100,166).

| Addr | DIP 1-4 | | Addr | DIP 1-4 |
|---|---|---|---|---|
| 0x01 | ON ON ON OFF | | 0x09 | OFF ON ON OFF |
| 0x02 | ON ON OFF ON | | 0x0A | OFF ON OFF ON |
| 0x03 | ON ON OFF OFF | | 0x0B | OFF ON OFF OFF |
| 0x04 | ON OFF ON ON | | 0x0C | OFF OFF ON ON |
| 0x05 | ON OFF ON OFF | | 0x0D | OFF OFF ON OFF |
| 0x06 | ON OFF OFF ON | | 0x0E | OFF OFF OFF ON |
| 0x07 | ON OFF OFF OFF | | 0x0F | OFF OFF OFF OFF |
| 0x08 | OFF ON ON ON | | **0x10 (master, module ID 0)** | **ON ON ON ON** |

Source: PDF:42-74. The PDF column order (switch 1..4) is not labelled; the pattern is address-1 inverted in binary with ON=0 (0x01 -> 1110, 0x0F -> 0000, 0x10 = 1111 breaks the pattern), so for a single battery set all four ON for 0x10 or accept the current address (check the label). [G on switch numbering].

Multi-battery behaviour: the PDF only says module 0 is "Master BMS" at 0x10 and others are slaves 0x01-0x0F (PDF:38-40). Neither the PDF nor the repo says whether the master's registers aggregate the slaves. The repo treats each address as an independent device and polls them separately (README multi-BMS, WIRING.md:119-123); `eg4_modbus.cpp:21-31` only warns on duplicate configured addresses. **For a single battery: poll its own address (0x10 if it is the lone master).** Unknown: whether 0x10 pack voltage/current/limits are per-module or system-wide in a parallel bank. [G, needs hardware check].

## 2. Protocol

| Question | Answer | Source |
|---|---|---|
| Standard Modbus RTU? | Yes: addr, func, 16-bit big-endian regs, CRC16-Modbus (poly 0xA001, init 0xFFFF) sent low byte first | PDF:76-79,97-108; `eg4_modbus.cpp:76-89,120-122` |
| Function codes | 0x03 read holding (all data). 0x10 write multiple (PDF); repo docs also show 0x06 write single with echo reply | PDF:94,112; REGISTER_MAP.md:152-163 |
| Error reply | func|0x80, 1 byte code, CRC: 5 bytes. (PDF shows `0x090` for write error, likely typo for 0x90) | `eg4_modbus.cpp:170-173`; PDF:140 |
| Count field | PDF says 0x00-0xFF registers (high byte 0). Repo successfully reads 91 regs (182 B reply) | PDF:100; REGISTER_MAP.md:139-146 |
| Max registers per read | No documented limit beyond one byte count (<= 125 by Modbus, <= 255 B payload) | [G] |
| Address offsets | **Vendor PDF register numbers are DECIMAL** (e.g. "0018" = 0x0012). Repo uses hex constants of the same registers. Registers are 0-based, no +1 offset | PDF:158-248 vs `eg4_bms.cpp:13-30` |
| Words in 32-bit values | Hi word first, big-endian (cycle count) | `eg4_bms.cpp:165-168,306` [C] |
| Quirk: all-zero reply | Some LL-V2 units intermittently return a CRC-valid 03 reply with all-zero data. Reject if pack voltage == 0 (or capacity, SOH, full cap all 0); do not treat as a valid reading or a "charge OK" | `eg4_bms.cpp:35-45,176-179,237-241` (issue #7) |
| Quirk: reply sizing | Repo identifies blocks purely by byte count (0x24, 0x2A, ...); a master should key on its own pending request instead | `eg4_bms.cpp:172-339` |
| Quirk: parser | Repo reads length from byte 2 and needs >= 5 bytes before checking; with byte-count 0 a `len<5` stall is possible. Validate addr/func/len/CRC in your own parser | `eg4_modbus.cpp:153-175` |

## 3. Request frames (CRC computed locally with the algorithm above; matches the repo's one sample)

Frames are `addr 03 start_hi start_lo cnt_hi cnt_lo crc_lo crc_hi`. Shown for address 0x10 (master) and 0x01.

| Purpose | Request, addr 0x10 | Request, addr 0x01 | Reply len |
|---|---|---|---|
| **Proven pair, block A**: regs 0x00-0x11 (V, I, 16 cells) | `10 03 00 00 00 12 C6 86` | (recompute) | 18 regs: 3+36+2 = 41 B |
| **Proven pair, block B**: regs 0x12-0x26 (temps, cap, max chg I, SOH, SOC, status, warn, prot, err, ... balance) | `10 03 00 12 00 15 27 41` | (recompute) | 21 regs: 3+42+2 = 47 B |
| **Single request, everything**: regs 0x00-0x26 (39 regs) | `10 03 00 00 00 27 06 91` | `01 03 00 00 00 27 05 D0` | 3+78+2 = 83 B |
| Cell count: reg 0x24 only | `10 03 00 24 00 01 C7 40` | | 7 B |
| Protection thresholds 0x43-0x48 (cell OV warn/prot/release, pack OV warn/prot/release) | `10 03 00 43 00 06 37 5D` | | 3+12+2 = 17 B |

Notes:
- Block A and B are what the repo actually sends (`eg4_bms.cpp:85,89`, from "working YamBMS", [C]). The 39-register single read is **[G] unverified**: it is within vendor limits and the BMS answers a 91-register read (REGISTER_MAP.md:139-146), but no log shows it. Recommend: start with A+B, switch to the single read only after confirming on hardware.
- Repo sample frame, checked: `01 03 00 2D 00 5B 94 38` reads 91 regs from 0x2D (config/RTC/identity) (REGISTER_MAP.md:139-144). My CRC calc gives 94 38.
- Spacing: leave >= 100 ms between a reply and the next request (PDF:33).
- Do not use the 0x06/0x10 write frames; they change BMS config (REGISTER_MAP.md:150).

## 4. Register map (reply data is big-endian 16-bit words; "off" = byte offset in the payload after the 3-byte header, for block A starting 0x00 and block B starting 0x12)

Vendor PDF numbers decimal in parentheses.

### Real-time block 0x0000-0x0026 (vendor-documented [V] and implemented [C]; not hardware-tested)

| Reg | (dec) | Field | Type | Scale | Notes / sign | Source |
|---|---|---|---|---|---|---|
| 0x0000 | 0 | Pack voltage | u16 | 0.01 V (10 mV) | PDF range 0..9000 -> 90.00 V max | PDF:158; `eg4_bms.cpp:13,182` |
| 0x0001 | 1 | Pack current | **s16** | 0.01 A (10 mA) | **positive = charge, negative = discharge** | PDF:160-162; `eg4_bms.cpp:186-187,193-194` |
| 0x0002-0x0011 | 2-17 | Cell 1..16 voltage | u16 each | 1 mV | Unused cells read 0 (repo ignores 0 or >=5000 mV) | PDF:164-166; `eg4_bms.cpp:204-206` |
| 0x0012 | 18 | PCB/MOS temp | s16 | 1 C | direct, no +50 offset | PDF:169; `eg4_bms.cpp:244` |
| 0x0013 | 19 | Avg temp | s16 | 1 C | | PDF:173 |
| 0x0014 | 20 | Max temp | s16 | 1 C | cell/pack max temp (exact scope not stated) [G] | PDF:177; `eg4_bms.cpp:252` |
| 0x0015 | 21 | Remaining capacity | u16 | Ah (repo) | PDF gives no unit | PDF:181; `eg4_bms.cpp:256` |
| **0x0016** | 22 | **Max charging current** | u16 | **1 A** (repo; PDF no unit) | BMS-requested charge current limit | PDF:182; `eg4_bms.cpp:20,260` |
| 0x0017 | 23 | SOH | u16 | % | | PDF:185 |
| 0x0018 | 24 | SOC | u16 | % | 0-100 | PDF:187 |
| 0x0019 | 25 | Status | u16 | bits | low nibble: 0 standby, 1 charging, 2 discharging, 4 protect, 8 charging limited; bit15 = heater on | PDF:189-202; `eg4_bms.cpp:354-370` |
| 0x001A | 26 | Warning | u16 | bits | see below | PDF:206-221 |
| 0x001B | 27 | Protection | u16 | bits | see below | PDF:222-237 |
| 0x001C | 28 | Error code | u16 | bits | 0001 voltage err, 0002 temp err, 0004 current-flow err, 0010 cell unbalance | PDF:238-242 |
| 0x001D-0x001E | 29-30 | Cycle count | u32 | 1 | | PDF:244-247; `eg4_bms.cpp:306` |
| 0x001F-0x0020 | 31-32 | "Full capacity" | u32 RW | mAs (= mAh/3600?) | PDF only; repo skips it [G] | PDF:248 |
| 0x0021-0x0023 | 33-35 | Temps 1-6 | 6 x s8 | 1 C | 1 byte each, high byte first | PDF:251-254; `eg4_bms.cpp:312-317` |
| **0x0024** | 36 | **Cell count** | u16 RW | cells | In PDF only; repo does NOT read it, it counts non-zero cell regs. Block B's offset is 34 | PDF:256; `eg4_bms.cpp:223` |
| 0x0025 | 37 | Designed capacity | u16 | 0.1 Ah | repo labels it "full capacity" | PDF:259; `eg4_bms.cpp:30,321` |
| 0x0026 | 38 | Cell balance bits | u16 | bit n = cell n+1 | repo declares but never reads | PDF:261-270; `eg4_bms.cpp:29` |

Cell count in practice: if reg 0x24 is not trusted, count cells with 0 < mV < 5000 (`eg4_bms.cpp:206`), which is the only coded method. Model "ZTR-16S200A-10KW-STD2" (16 cells) is the sample (REGISTER_MAP.md:131).

### Warning (0x001A) and Protection (0x001B) bit definitions (identical layout; PDF:206-237, `eg4_bms.cpp:378-414`)

| Bit | Warning | Protection | Repo treats as blocking charge |
|---|---|---|---|
| 0x0001 | Pack OV | Pack OV | yes |
| 0x0002 | Cell OV | Cell OV | yes |
| 0x0004 | Pack UV | Pack UV | no (discharge) |
| 0x0008 | Cell UV | Cell UV | no (discharge) |
| 0x0010 | Charge OC | Charge OC | yes |
| 0x0020 | Discharge OC | Discharge OC | no |
| 0x0040 | Abnormal ambient temp | same | no (not in repo's mask) |
| 0x0080 | MOS overheating | same | no (not in repo's mask) [G: should stop charge] |
| 0x0100 | Charge OT | Charge OT | yes |
| 0x0200 | Discharge OT | Discharge OT | no |
| 0x0400 | Charge UT | Charge UT | yes |
| 0x0800 | Discharge UT | Discharge UT | no |
| 0x1000 | Low capacity | Low capacity | no |
| 0x2000 | "Float stopped" | Discharge SC (short circuit) | no |
| 0x4000 | listed, unnamed | (n/a) | |

Repo's charge-allowed rule: `!(prot & (0x0001|0x0002|0x0010|0x0100|0x0400))` and it uses the Protection register only, not Warning (`eg4_bms.cpp:299`). That is the repo's choice, labelled "permit" not "flowing" (README.md:178-181). [C, not verified.]

There is **no charge-MOSFET-state register** in the PDF or repo; the Status word (0x0019 value 0x04 "Protect", 0x08 "Charging Limited") plus Protection bits are the only signals.

### Not provided by the BMS: charge VOLTAGE limit

No register in the real-time block or in the PDF is a charge voltage limit (CVL). Closest data (config block, **read-only use**, from one unit's sample in REGISTER_MAP.md; "Based on analysis of Modbus response", unverified against vendor):

| Reg | (dec) | Field | Raw -> value | Unit as stated | Source |
|---|---|---|---|---|---|
| 0x0038 | 56 | Balance start voltage | 3400 | 1 mV | REGISTER_MAP.md:33 |
| 0x0043 | 67 | Cell OV warning | 3700 | 1 mV (3.700 V) | :48 |
| 0x0044 | 68 | Cell OV protection | 3800 | 1 mV (3.800 V) | :49 |
| 0x0045 | 69 | Cell OV release | 3450 | 1 mV | :50 |
| 0x0046 | 70 | Pack OV warning | 5920 | stated "0.1 V" but 5920 = 59.20 V -> really **0.01 V** | :53 |
| 0x0047 | 71 | Pack OV protection | 6000 | 0.01 V (60.00 V) | :54 |
| 0x0048 | 72 | Pack OV release | 5520 | 0.01 V (55.20 V) | :55 |
| 0x0050 | 80 | Charge OC1 protect | 20500 | 0.01 A (205.00 A) | :58 |
| 0x0053 | 83 | Charge OC2 protect | 22500 | 0.01 A | :60 |
| 0x0049 / 0x004A | 73/74 | unknown (555, 500) | | "55.5A or 5.55A?" unknown | :124-125 |

The REGISTER_MAP.md heading calls pack voltage thresholds "0.1V units", which contradicts its own example values (46.50 V); treat as 0.01 V (matches reg 0x0000 scale). [G]
The unit sample is a 16S (48 V nominal LFP) pack. Values are per-unit settings that the owner can change; read them from the actual BMS rather than hard-coding.

## 5. Test vectors

The repo contains **no complete real read-response frame** (searched md, yaml, h, cpp). Only these complete frames exist:

| Frame | Hex | Decoding | Source |
|---|---|---|---|
| Config read request | `01 03 00 2D 00 5B 94 38` | addr 1, read 91 regs from 0x2D; CRC 0x3894 on wire `94 38` | REGISTER_MAP.md:139 |
| Write single (do not use) | `01 06 00 39 00 14 59 C8` | set reg 0x39 = 20 | REGISTER_MAP.md:157 |
| Write multiple, RTC (do not use) | `01 10 00 2D 00 04 08 67 11 18 6E 00 08 05 00 F4 5A` | 2025-12-08 18:00:00 | REGISTER_MAP.md:170 |

The response to the config read is described ("182-byte payload", REGISTER_MAP.md:146) but not printed. I verified the request CRCs above by recomputation. I did not verify the 0x06 and 0x10 CRCs beyond the 0x06 one (59 C8 matches).

Synthetic vector (derived, not from a BMS, for CRC/decode unit tests): request `10 03 00 00 00 12 C6 86`. A decode test should build a payload with pack = 5320 (53.20 V), current = 0xFFF6 (-0.10 A), cells 3325 mV ... and compute the CRC with the algorithm in section 2.

## 6. Recommended mapping to charger inputs

| Charger input | Take from | Derivation | Confidence |
|---|---|---|---|
| **CCL (A)** | reg 0x0016 "Max charging current" | direct, unit A (repo scales x1). Clamp to charger max. Treat 0 as "no charging", but also treat an all-zero block as invalid (not zero limit) | [C], unit assumed. Check on hardware |
| **CVL (V)** | **not supplied by BMS** | Use fixed config: `cells x cell_target`, e.g. 16 x 3.45 V = 55.2 V, kept below BMS cell-OV release (3.45 V sample) and well below cell-OV protection 3.800 V / pack OV protection 60.00 V. Optionally read OV thresholds from 0x43-0x48 once at startup to cap CVL | [G] derivation |
| **Charge allowed** | Protection (0x001B) `& 0x0513` == 0 AND Status low nibble != 0x04 AND block valid AND reply fresh | 0x0513 = 0x0001+0x0002+0x0010+0x0100+0x0400 (corrected: the sum is 0x0513, not 0x0517) (repo's mask). Add 0x0080 (MOS OT) as extra conservative. Also require CCL > 0 | [C] mask; extras [G] |
| **Alarm** | Warning != 0 OR Protection != 0 OR Error != 0 OR comms timeout/no reply for N polls | Repo's online tracker uses `max_no_response_count` default 5 consecutive missed polls | [C] (`eg4_bms/__init__.py:12`, `eg4_bms.cpp:69-79`) |
| **SOC** | reg 0x0018, u16 % | direct; validate 0-100 | [V] |
| **Pack V** | reg 0x0000, u16 x 0.01 V | direct | [V] |
| **Max cell V** | max over regs 0x0002-0x0011, ignoring 0 and >=5000 mV | in mV; the BMS provides no precomputed max. Use for CVL back-off: reduce CVL/CCL as max cell nears the OV warning (3.700 V sample) | [C]/[G] |
| Cell count | reg 0x0024, else count non-zero cells | | PDF only / [C] |
| Temps | 0x0012-0x0014 s16 C, 0x0021-0x0023 s8 | for charge-UT/OT derating, BMS already trips Charge UT/OT bits | [V] |

Safety design notes (derived):
- Fail safe: if no valid reply within ~1 s x 3 retries, or any all-zero block, force CCL = 0 and raise alarm. Never act on the zero-block quirk.
- Poll cadence: block A+B, or one 83 B read, at 1-2 Hz is conservative versus the vendor 100 ms minimum and repo's 10 s; pick >= 1 s and reject stale data (> 5 s).
- Apply Warning bits as derate triggers (e.g. Cell OV warn, Charge OT warn) before Protection trips.
- Status 0x08 "Charging Limited" means the BMS itself is limiting charge: honour the lower CCL (reg 0x0016) rather than treating it as fault. [G]
- Items needing a hardware check: unit of 0x0016, current sign on this unit, whether 0x10 aggregates a parallel bank, the single 39-reg read, reply latency, and bit meanings beyond the repo's protection mask.
