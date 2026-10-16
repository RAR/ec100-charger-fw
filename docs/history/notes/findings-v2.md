# EC100 firmware v2.0.1.1 vs v1.01.1105: behaviour diff

Inputs: `dumps/ec100-v2.0.1.1/decrypted.bin` (app only, base 0x08005000) and the app half of
`dumps/gd32_flash_20261012.bin` (v1.01.1105). Decompiled with `tools/ghidra_analyze_app.py` into `re/v2/`
(untracked). Literals come from `tools/dat.py --v2`, and `.data` from `tools/keil_data.py` (v2's `.data` is
LZ-compressed by armlink, so plain `strings` shows broken fragments).

Tags: **[V]** = verified, seen in the decompiled code at the address given. **[I]** = inferred.
Addresses are `v1 → v2`. State-struct offsets are given for v2, with v1 offsets in brackets where they matter.

## 0. Orientation

- The v2 image identifies itself as `DZ-26-004-V2.0.1.1` (@0x080183FE). **[V]**
- Of the 273 v1 app functions, 183 are byte-for-byte equivalent after renaming. v2 adds about 210 functions, mostly the
  WiFi/BLE stack. **[V]** (via a function matcher over the normalised decompiled code)
- The timebase is unchanged: TIMER1 IRQ21 (0x0800cf2c → 0x0801403c) runs every 1 ms, with an inner block every 10 ticks.
  The prescaler/period is 0x3B/0x3E7 in both. **[V]** That gives 1 ms only if the timer clock is 60 MHz. **[I]**
- Charger state struct: 0x200005D8 → 0x20001148. The PSU structs move from 0x20000178/0x20000314 to 0x20000BC0/0x20000D5C.
  The CAN RX buffer moves from 0x20000050 to 0x200005D8 (the old state address). **[V]**

| Role | v1.01.1105 | v2.0.1.1 |
|---|---|---|
| CAN init / filter / RX read / RX ISR / TX | 0800dba0 / 0800da88 / 0800dd78 / 0800d274 / 0800de88 | 08014d18 / 08014c00 / 08014ef0 / 08014454 / 08015000 (all identical) |
| send-and-wait | 08008328 | 0800fe68 (identical) |
| poll builder / scheduler | 0800bf4c / 0800c52c | 08012bf8 / 080131fc (via 0800afc8) |
| reply dispatcher / readback store | 0800b688 / 08008c54 | 080122c4 / 08010b18 (identical) |
| write transaction / builder / packer | 08007ce0 / 0800c428 / 08008ce0 | 0800f9c0 / 080130e0 / 08010ba4 |
| PSU register table init | 080061b8 | 0800922c (identical) |
| VOUT setter / current limit + ramp | 0800d2b8 / 08008f64 | 08014498 / 08010e0c |
| on/off decision | 08008dc0 | 08010c90 + 08007acc (start) + 08007908 (keep running) + 08012b88 (hold, 0x132) |
| measurements + error codes | 08006d68 | 0800bbf0 |
| BMS poll / Pylon TX / CID2 dispatch | 08006bb0 / 0800982c / 080097cc | 0800b9b4 / 080117c8 / 08011768 |
| Pylon 0x61 / 0x62 / 0x63 parsers | 08007a54 / 080077d0 / 08007bc8 | 0800f7ac / 0800f4f8 / 0800f920 |
| settings load + defaults / menu keys | 08005c6c / 08009b60 | 08007f74 / 08011afc |

## 1. CAN to the PSUs

- **IDs are unchanged.** Reads use `0x108082FE | addr<<16` with data `01 rr 00…`. Writes use cmd 0x81, `0x108081FE | addr<<16`.
  A write counts as acknowledged when the reply ID's byte 1 is 0x81. **[V]** (080130e0, 0800f9c0)
- **New: 0x0132 is now sent to address 0**, ID `0x108081FE` (080130e0: `if reg==0x132` drops the address). It is sent
  fire-and-forget: 0800f9c0 doesn't wait for an ACK and doesn't record a pending register for it. **[V]**
  Whether address 0 means "all modules" is **[I]**.
- **Poll list: identical.** Same order (`0100 0101 0102 0103 0104 010C 0239 0132 … 022F 024E | 023A 023B`), and the
  address still toggles 1↔2 after 0x024E. **[V]** (08012bf8 diff)
- **Encoding: unchanged.** The packer and the per-register type table are identical. **[V]**
  - 0x0100: `01 00 00 00 00 00 HI LO`, raw = trunc(V × 0.1 × 1023), where V is the setpoint in 0.1 V units. **[V]**
  - 0x022D: raw = trunc(A × 1023 / 52 **+ 2.0**). The `+2` is a soft-float add (FUN_08005328 is `fadd`), which earlier notes
    missed: a ~0.1 A offset. A is halved per PSU when both are present. **[V]**
  - 0x0132: data[3] = 0 → on, 1 → standby (type 5 → byte at data[3]). **[V]**
- **Only 0x0100, 0x022D and 0x0132 are ever written, in both versions.** 080130e0 only knows how to encode those three, and every
  caller of 0800f9c0 passes one of them. **0x0101 / 0x0104 are never written.** **[V]**
- **Resend / keepalive:**
  - 0x0100 and 0x022D are still written only when the readback differs from the setpoint by ≥0.1 V / ≥0.1 A. **[V]**
  - The comparison uses readback **0x022F** for VOUT (PSU struct +0x188) and 0x022D for current (+0x17C). The v1 notes
    called +0x188 "VOUT", but the table maps +0x184/+0x188 to register 0x022F. **[V]**
  - **New:** 0x0132 is broadcast **every 2 s** (08012b88: counter +0x68 = 200 × 10 ms) whenever the CAN layer is idle,
    whether or not anything has changed. **[V]**
    - From boot until a charge is requested, that broadcast is "standby" (data[3] = 1). **[V]**
    - v1 sent 0x0132 only to a module whose 0x0132 readback was non-zero (08008dc0). So v1 only ever re-enabled a module
      and never commanded standby. **[V]**
- **Timing:**
  - Poll rate: v1 issued one request every >10 ms; v2 every >20 ms (scheduler counter +0x4E, 1 ms). **[V]**
  - **ACK/poll timeout went from 0.5 s to 5 s.** v1 counted +0x3A in 10 ms units up to 500 ms. v2 counts +0x4C
    in single 10 ms steps up to 500 steps. **[V]**
  - On timeout the module is marked absent (0x20000EF8[n] = 0) and the address flips. **[V]**
- The CAN echo/test mode (08008bc8 → 08010a88, RX frame re-sent when 0x20000A50 ≠ 0) is unchanged. **[V]**

## 2. Charge logic

**Measurements (0800bbf0):** **[V]** for the scaling; register meanings **[I]**.
- Battery V (+0x54, 0.1 V) = reg 0x0184 / 1024 × 10, averaged across modules when both are on.
- Current (+0x58, A) = reg 0x0182 / 1024, summed.
- AC input (+0x3A) = reg 0x0178 / 1024. v2 adds a 10-sample moving average; values below 51 are forced to 0.
- With no module present, the board's own ADC is used (+0x1C = ADC × 0.16923, from 0800c964).

**Soft-start / current limit** (08008f64 → 08010e0c): **[V]**
- v1: every 2000 ms, setpoint +1 A, but only while (setpoint − measured) < 4 A. It starts at 2 A.
- v2: every **1000 ms**, setpoint = **measured + 1 A** (minimum 2 A), capped at the user maximum (+0x30).
  This is a measured-current tracker rather than a fixed ramp.
- Caps in both versions:
  - 60 A when AC input < 180 V (v1: < 181 V). This is a low-line derate. **[I]** for the reason.
  - 50 A with a single module.
- Split: with both modules present, each gets setpoint × 0.5. **[V]**
- When there's no charge request (+0xB = 0), the setpoint is held at 2 A.

**New BMS current-follow mode** (app register 0x1510 → +0x14, default 0): **[V]**
- Applies when this flag is set and the BMS is online.
- Per-module limit = BMS CCL (0x63 field, 0.01 A units) × 0.005. That is half the CCL with two modules.
- With a single module, the code still halves it when CCL < 50 A, and pins 50 A otherwise. This looks like a vendor bug. **[I]**
- This mode bypasses the soft-start and the user maximum.

**On/off** (08008dc0 → 08010c90/08007acc/08007908): **[V]**

v2 drops the stay-on condition entirely while the debounce or hold is running. Thresholds (0.1 V / %): start SOC +4, stop SOC +5,
start V +0x18, stop V +0x1A, charge V +0x36.

| | v1 | v2 |
|---|---|---|
| Start, SOC mode | SOC < start | SOC ≤ start |
| Start, voltage mode | V < start V | **36.0 V < V** < start V (0x168) |
| Start, any mode | **V < 42.3 V** (0x1A7), regardless of mode | removed |
| Stop, SOC mode | SOC ≥ stop, or V > 57.7 V (0x241) | SOC ≥ stop, **or V ≥ charge V − 0.1 V** |
| Stop, voltage mode | V ≥ stop V **and I ≤ 4 A** (tail current) | V ≥ stop V (no tail-current test), or V ≥ charge V − 0.1 V |
| Debounce | off: 10 s | start 5 s, stop 5 s (+0x1E/+0x22 = 500 × 10 ms) |
| Output hold after stop | none | **30 s** (+0x64 = 3000 × 10 ms), then 0x0132 = standby |
| Error code ≠ 0 | blocks start / forces stop | forces stop immediately |

- v2 also adds a forced "charge now" (+0x13, a key press or app register 0x150F), which ignores the start thresholds.
  There's also an inhibit flag (+0x2). **[V]**
- v2 adds a time-of-day schedule (+0x12 = 1 once / 2 repeat): five HHMM windows set through the app (0800c198, 08007d4c),
  ANDed with the SOC/V conditions. **[V]**
- VOUT (08014498): normally the user charge voltage (+0x36). In voltage mode it switches to stop V once V ≥ stop V − 0.2 V
  (v1: −0.1 V) and current is flowing. **[V]**
- **The BMS CVL is never used for VOUT, in either version.** It is only copied into the app status frame (080061f4). **[V]**
- Defaults are the same in both:

  | Setting | Default |
  |---|---|
  | Max current | 10 A |
  | Charge V | 57.6 V |
  | Start / stop SOC | 20 % / 100 % |
  | Start / stop V | 42.0 V / 57.6 V |
  | Charge-V adjustable range | 47.0–57.8 V |
  | Max current adjustable range | 2–100 A |
  | Start V floor | 42.0 V |

  **[V]** (08007f74, 08011afc). The settings page moved from 0x0801F000 to 0x0801FC00. **[V]**

## 3. BMS (Pylontech RS485, USART1 9600)

- **Commands: unchanged.** It cycles through CID2 **0x60 → 0x61 → 0x62 → 0x63** every 500 ms. The header is VER 0x20, ADR 0x12,
  CID1 0x46, LENGTH 0, sent as `~%s%04X\r`. **[V]** (jump tables at 0x080097e8 / 0x08011784, `.data` identical)
- **0x61 (analog):** total V, current, SOC (byte 10, used in SOC mode), cycles, SOH, cell min/max, temperatures. Unchanged. **[V]**
- **0x63 (charge management):** CVL, DVL, CCL, DCL, plus status bits 4–7. **[V]**
  - v1 stored these but never used them for control.
  - v2 uses only the CCL, and only in follow mode.
  - The charge-enable / charge-immediately bits are display-only in both versions. **[V]**
- **0x62 (alarms): new.** v1's parser just cleared its flag bytes and ignored the reply.
  - v2 decodes INFO bytes 6..9 into 0x2000052C and a word at 0x20000530.
  - If `word & 0xAA68` (bits in INFO bytes 8–9) is set while in BMS mode, it raises **error 12** and charging stops. **[V]**
  - What those bits mean in Pylon terms is **[I]**: likely the over-voltage/over-temperature/over-current "protect" bits.
- **Link loss:** BMS mode drops to voltage mode (+0x9 = 1) after 4 missed polls (about 2 s). v1 did this after 2 (about 1 s).
  A bad checksum still trips it after 2. **[V]**

## 4. New in v2

- **WiFi + BLE module on UART3 at 230400** (9600 in v1, with nothing driving it). Firmware support only: the EC100 this
  project runs on has no such module fitted and no connectivity at all (user, 2026-10-16). It uses AT commands of the
  Ai-Thinker style. **[V]** (strings); the module type is **[I]**.
  - Commands seen: `AT+WJAP`, `AT+SOCKET`, `AT+BLE*`, `+EVENT:WIFI_GOT_IP`.
  - BLE name `ECO-WORTHY 24_xxxx`, service 0xFFF0, characteristics FFF1/FFF2.
  - Cloud TCP connection: `AT+SOCKET=4,47.254.32.119,9017,0,1`.
  - A factory-test join: `AT+WJAP="YAN","12345678"`.
- **"4.32.11" is not a version string.** It is a fragment of the LZ-compressed `.data` copy of that server IP,
  **4**7.25**4.32.11**9. The decompressed `.data` has no such string. **[V]** (`tools/keil_data.py`)
- **App protocol:**
  - Writes to registers 0x1507–0x151A (08005d4c) set charge V, max current, SOC/V thresholds, schedule, force-on and
    CCL-follow. Each write is range-checked and saved to flash. **[V]**
  - Status frames start `F7 7F` (080061f4) and carry V/I/P, AC, settings, the error bitmask (`1 << (code−1)`) and BMS data. **[V]**
- **Factory reset** (08006d54): sends `AT+WDISCONNECT`, erases 0x0801E000 and 0x0801F800, then resets the MCU. **[V]**
- **Error codes on +0x50** (`E` + two digits on the display): codes 1–11 are unchanged from v1 (see §5). Code 12 (BMS alarm) is new. **[V]**
- UART4 at 460800 is in both versions. **[V]** It is the RS485 service/factory-test/update port, not a display link: the display is the
  MCU's own segment-LCD controller (corrected 2026-10-14, see `docs/history/notes/oem-hw-map.md`).
- GPIO PC1 follows the "charging" state in both versions. v2 holds it for 10 s after stopping. **[V]** What it drives is **[I]**.

## 5. Bearing on safe operation from our own controller

- **0x0183 alarms (both versions, identical).**
  - The firmware stores reply data[2..3] separately, and reads data[4..7] as a **big-endian u32 A**. **[V]** (080122c4)
  - These bits are treated as faults, with the first match winning. **[V]** (0800bbf0 / 08006d68):

  | Bit of A | Code |
  |---|---|
  | 22 | E1 |
  | 17 | E2 |
  | 12 | E3 |
  | 4 | E4 |
  | 14 | E5 |
  | 27 | E6 |
  | 0 | E8 |
  | 1 | E9 |
  | 20 | E10 |
  | 2 or 3 | E11 |

  - E7 is never set.
  - The fault mask on A is therefore **0x0852_501F**. Bits 17, 20, 22 and 27 sit in data[4..5].
  - Every code forces a stop, and the code is cleared once its bits clear.
- **Stored defaults:** neither firmware writes 0x0101/0x0104. **[V]** That the stock controller never touches them is therefore
  certain. What a module does on CAN silence is still unknown. v2's 2 s 0x0132 broadcast is the closest thing to a keepalive. **[I]**
- **Timeouts:** v2 allows 5 s per request; v1 allowed 0.5 s. One request is outstanding at a time, with ≥20 ms spacing. **[V]**
- **0x021D, 0x0239–0x024E:** polled and stored but **never read** by either version. v2 adds nothing about them. **[V]** (no reads)
- **0x022F:** the firmware treats it as the **VOUT setpoint readback** (same raw ×1023 scale as 0x0100). **[V]** for the use,
  **[I]** for the meaning.
- Control only ever reads the following registers. **[V]** Their names are **[I]**:

  | Register | Used as |
  |---|---|
  | 0x0178 | AC input |
  | 0x0182 | output current |
  | 0x0183 | alarms |
  | 0x0184 | output voltage |
  | 0x022D | current-limit readback |
  | 0x022F | VOUT readback |

  0x0175 and 0x0132's readback are not used in v2.

## Implications for the ESPHome controller (before the first real-PSU test)

1. **Widen the 0x0183 alarm decode.** `nasn_rectifier.cpp:212` keeps `uint16_t(value)`, which is data[6..7] only.
   - That loses stock fault bits 17, 20, 22 and 27, and `alarm_mask` defaults to 0.
   - Store data[4..7] as a u32 and default the mask to 0x0852501F, or at least log the raw value.
2. **Also poll 0x022F and log it next to 0x0100.** The stock controller confirms VOUT against 0x022F, not 0x0100. Until we've seen
   both on real hardware, don't let a 0x0100 readback mismatch fault a module.
3. **Use a 5 s reply timeout** and keep one request in flight with ≥20 ms spacing.
4. Our 5 s keepalive is fine, since the stock controller resends 0x0132 every 2 s.
   - The stock controller only resends 0x0100/0x022D on mismatch.
   - Keep `write_stored_defaults` off for the first test. Nothing in stock firmware writes 0x0101/0x0104, so writing them is
     untested territory.
5. Optional, for parity:
   - Stock current raw is `trunc(A·1023/52 + 2)`; ours is `round(A·1023/52)`, a ~0.1 A difference.
   - Stock derates to 60 A total when AC input (0x0178) < 180 V.
   - Stock caps each module at 50 A, matching our `MAX_A_PER_PSU`.
6. Nothing in v2 changes the IDs, the cmd 0x81 write or the setpoint encodings that the component already uses.
