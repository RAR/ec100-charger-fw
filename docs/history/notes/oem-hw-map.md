# Eco-Worthy charger OEM control board: hardware/interface map

This map is for writing replacement firmware. It was extracted from the stock bootloader, from app v1.01.1105 (the one on the
bench board) and from app v2.0.1.1 (`DZ-26-004-V2.0.1.1`).

Tags: **[V]** = verified in code or literals at the address given. **[I]** = inferred.
Addresses are v1 app / v2 app / boot as labelled. GPIO call arguments were recovered from the disassembly, because Ghidra
truncated the call-site arguments. The extractor (constant propagation over `objdump` output) is in the scratchpad as `bin/calls.py`.

> **Two premises in the brief turned out wrong:**
> 1. **There is no UART display panel.** The LCD is a segment glass driven directly by the MCU's on-chip **SLCD** controller
>    (8 COM × 19 SEG). UART4 @ 460800 with `B5 5B` frames is the **RS485 service/factory-test and firmware-update port**.
> 2. **The CAN pins are PA4/PA5 (AF11), not PA11/PA12** as `docs/history/notes/findings.md` says. PA11 and PA12 are two of the four push-buttons.
>
> **v2.0.1.1 targets a different PCB revision and a different bootloader from the bench board.** The RS485 DE pin, LEDs,
> backlight and WiFi control pins all differ (§2.3). v2 also checks its anti-clone hash at a different flash address (§6.7).
> Don't flash v2 onto the v1 board, and use the **v1 pin map** for our firmware.

---

## 1. MCU identification

**Part: GigaDevice GD32L235RBT6** (LQFP64, 128 KB flash, 24 KB SRAM, Cortex-M23, ≤64 MHz). **[V]** by peripheral fingerprint. The package
is **[I]**; a QFN64 "RBHx" variant is also possible.

The evidence below was checked against the GD32L235xx datasheet Rev 2.2 (pin and AF tables):

| Evidence | Value | Fits |
|---|---|---|
| Core | Cortex-M23 r1p0 (CPUID 0x411CD200) | GD32E23x / L23x / C2x1 |
| Flash/SRAM register 0x1FFFF7E0 | 0x0018_0080 → 128 KB / 24 KB | L235: up to 128 KB / 24 KB (SRAM0 8 K + SRAM1 16 K) |
| GPIO on AHB at 0x4800_0000 (A/B/C/D) | `&GPIOA`, 0x48000400/800/C00 | E23x/L23x/C2x style, not F1/F3/E50x |
| **SLCD at 0x4000_2400** (DATA regs at +0x14 … +0x4C, stride 8) | v2 080099c8, 08010354; v1 0800cafc | **L23x only** |
| **CAN at 0x4000_6400** | v2 08014d18 | L235 adds CAN over L233 (AN184) |
| RTC at 0x4000_2800 (L0-style: WPK +0x24, BKP from +0x50) | v2 080168c0…, 08010114 | L23x |
| USART0 0x40013800, USART1 0x40004400, UART3 0x40004C00, UART4 0x40005000 | v2 080172c4 | L235 "2 USART + 2 UART" |
| UART4 on PB0/PB1 (AF4) | boot 08000e1c | datasheet: "available on GD32L235**RB** only" |
| PD4/PD8/PD9 as SLCD segments | v1 0800cafc | exist on the 64-pin L235 |
| HXTAL_VALUE constant in `rcu_clock_freq_get` | 0x00B71B00 = **12 MHz** (v2 08015d84, literal @0x080161d4) | |
| IRC16M / IRC48M constants | 0x00F42400 / 0x02DC6C00 | L23x clock tree (not IRC8M) |

**Clock setup** (v2 `SystemInit`-like 08016edc / 08013f04; boot 080020e0 is the same):
- HXTAL on, then wait for HXTALSTB (timeout 0xFFFF; it hangs forever if the crystal fails). **[V]**
- FMC wait states = 2. **[V]**
- AHB /1, **APB1 /2** (CFG0 \|= 0x400), APB2 /1. **[V]**
- CFG1 PREDV = /1. **[V]**
- **PLLSEL = HXTAL, PLLMF code 3 = ×5** (CFG0 \|= 0xD0000). **[V]**
- SYSCLK = PLL.

So SYSCLK = 12 MHz × 5 = **60 MHz**, AHB 60, **APB1 30 MHz**, and the APB1 timer clock is 60 MHz. **[V]** for the code; that the crystal really is
12 MHz is **[I]** (it's the compiled-in HXTAL_VALUE). Consistency checks:
- TIMER1 prescaler 0x3B / period 0x3E7 gives 1 ms. **[V]**
- CAN BRP 30 with 8 tq gives 125 kbit/s. **[V]**

The VTOR is set to 0x08005000 by the **app itself** (v2 08013f04 → 08015cb8(0x08000000, 0x5000); v1 0800b50c → 0800ea58). **[V]**

Other peripherals in use:
- TIMER1 (IRQ21, 1 ms tick)
- DMA channels 1/3/4 (USART RX)
- ADC (0x40012400)
- FWDGT (0x40003000)
- FMC
- PMU (0x40007000, backup-domain write enable)
- RTC (time of day, and backup registers used as reboot flags)

**Uncertain:** the exact package and marking, whether the crystal is 12 MHz, and whether a 32.768 kHz LXTAL is fitted.
- v2 RTC init 08010114 selects RTCSRC = LXTAL (BDCTL 0x100).
- v2 SLCD init 08010354 then asks for IRC32K (0x200), so it may fall back.

---

## 2. Pin map

### 2.1 Pins common to v1 (bench board) and v2

| Pin | Config | Function | Evidence |
|---|---|---|---|
| PA4 | AF11 | **CAN_RX** | v1 08008254, v2 0800fd98 **[V]** |
| PA5 | AF11, PP | **CAN_TX** | same **[V]** |
| PA2 | AF7, PP, pull-up | **USART1_TX** → BMS RS485 (Pylon, 9600 8N1) | v1 08008824(1,9600), v2 08010614 **[V]** |
| PA3 | AF7, pull-up | **USART1_RX** | **[V]** |
| PA0 | AF8, PP, pull-up | **UART3_TX** → WiFi/BLE module (v1 9600, unused; v2 230400) | **[V]** |
| PA1 | AF8, pull-up | **UART3_RX** | **[V]** |
| PB0 | AF4, PP, pull-up | **UART4_TX** → RS485 service/update port, 460800 8N1 | boot 08000e1c, v1/v2 08010614 **[V]** |
| PB1 | AF4, pull-up | **UART4_RX** (DMA + IDLE interrupt) | **[V]** |
| PC6 | input, pull-down | **Key 0** (active high) | v2 0800bb38 / v1 08006cc0 **[V]** |
| PB15 | input, pull-down | **Key 1** (UP) | **[V]** |
| PA12 | input, pull-down | **Key 2** (DOWN) | **[V]** |
| PA11 | input, pull-down | **Key 3** (MENU/SET) | **[V]** |
| PC0 | analog | **ADC_IN10**: 10-sample DMA scan, voltage sense. ×0.16923 gives 0.1 V units (full scale ≈ 69 V, so a divider of about 1:21) | v2 0800fc38 / 0800c964 **[V]**; that it measures the battery/bus voltage is **[I]** |
| PC1 | output PP | **"charging" output**: high while charging, forced on, or for 10 s after stop. Drives an unknown load (relay/fan/PSU enable?) | v1 08007754, v2 0800f48c **[V]**; load **[I]** |
| PD1 | output PP | **Status LED**: off (high) idle; blinks 500 ms with a module present; blinks 100 ms while ramping; solid (low) while charging with current | v1 0800b444, v2 08012070 **[V]**; active-low is **[I]** |
| PD6 | analog | **VSLCD** (external LCD bias input) | v1 0800cafc, v2 080139e0 **[V]** |
| PA8, PA9, PA10, PB9, PC10, PC11, PC12, PD2 | AF3, pull-down | **SLCD COM0…COM7** | **[V]** |
| PA6, PA7, PA15, PB3, PB4, PB5, PB8, PB11, PB12, PB13, PB14, PC4, PC5, PC7, PC8, PC9, PD4, PD8, PD9 | AF3, pull-down | **SLCD SEG3, 4, 17, 7, 8, 9, 16, 11, 12, 13, 14, 22, 23, 25, 26, 27, 28(remap), 30, 31** | **[V]** for the AF3 config; SEG numbers are from the datasheet |
| PA13 / PA14 | default | SWD | untouched **[V]** |

### 2.2 Pins that differ by board revision

| Function | **v1 board (bench, v1.01.1105 + stock boot)** | v2 board (v2.0.1.1) |
|---|---|---|
| **RS485 DE/RE** (shared by USART1 BMS and UART4 service) | **PB10**, output PP. Set before TX, reset after TC. Also used by the bootloader. | PB2 |
| **LCD backlight** | **PC2** (high = on) | PB6 |
| **Error LED** (low when error code ≠ 0) | **PD0** | PC13 |
| WiFi module reset | none (module free-running) | PB7 (active low, 50 ms pulse at init and on AT failure) |
| WiFi module enable/power | none | PD5 driven high at init **[I]** |
| Bootloader heartbeat | PB8 toggles every 500 ms, PB9 low; PB8 is set on HardFault. These are LCD SEG16/COM3 pins, so the LCD sees DC for about 1.5 s per boot. | n/a |

All rows are **[V]** from the GPIO call sites (v1 080084a4/08008504/08008548/08008470, v2 08010030/0801009c/080100d8/0800fffc/0800fd20,
boot 08000c10/08000ca8). Each pin's role comes from its use.

### 2.3 Other wiring facts

- **One DE pin serves two UARTs.** Both USART1 (BMS, 9600) and UART4 (service, 460800) raise the same DE pin around TX.
  This is v1: Pylon TX 0800982c and UART4 handler 080069ac both use PB10. That means the BMS RS485 port and the update port
  are either **one transceiver with both UART TX lines combined**, or two transceivers sharing a DE line. **[I]** Trace this on the board.
  The vendor update almost certainly goes in through the same RJ45 "BMS/RS485" jack.
- DE timing: DE high → `delay 1 ms` → polled TX of each byte, waiting for TC → DE low. **[V]** (v2 08005cbc, 0800b564, 080117c8)
- Interrupt use (v2): IRQ20 CAN RX0, IRQ21 TIMER1, IRQ28 USART1, IRQ29 UART3, IRQ30 UART4 (`nvic_irq_enable` 08015c86). **[V]**

---

## 3. "Display protocol" on UART4 (B5 5B): actually a service/factory-test port

### 3.1 The real display is the on-chip SLCD

Configuration **[V]** register writes (v2 08010354, v1 0800cafc + 08008588); field meanings **[I]**, based on the STM32L-style
SLCD layout:
- CTL: DUTY = 1/8 (CTL bits 4:2 = 100), BIAS field = 0 (1/4 bias), VSRC bit set (external VSLCD on PD6).
- CFG: PS = 1, DEAD = 1, HDEN on, and blink fields (0x20000 | 0xE000).
- Frame buffer: 8 words at RAM 0x20000B68… (COM0…COM7). It is copied to SLCD_DATA0…7 (0x40002414 + 8·n) **every 200 ms**
  after waiting for UPRF, then an update request is made (v2 080099c8). **[V]**
- Backlight: on while the activity timer +0x60 > 0. The default timeout is 120 000 ms (0x1D4C0); −1 means always on. When it
  expires the buffer is cleared, the backlight pin goes low, and the menu exits. **[V]** (v2 080099c8, 08007f74)

**Digit encoding.** Each 7-segment digit occupies **one SEG column**. Font bit *k* goes to **COM *k***. **[V]**
- Font A @0x0801839c: `0-9 = EE 24 D6 B6 3C BA FA 26 FE BE`, `10 = blank 00`, `11 = '-' 10`, `12 = 'E' DA`. **[V]**
- Font B @0x080183a9 (mirrored digits): `EE 48 D6 DA 78 BA BE C8 FE FA 00 10`. **[V]**

**Field map (v2 render chain 08009c38).** **[V]** for the SEG columns and fields; label names are **[I]**.

| Field | SEG columns (digit order) | Content |
|---|---|---|
| AC input V | 26, 27, 17 (font B) | ST+0x3A (V). `---` when 0. |
| Current | 22, 4, 3 | ST+0x58 output A. Menu 1: max current (+0x30, blinks 300/600 ms). |
| Power | 11, 13, 14, 23 | ST+0x40 (W, 4 digits, leading zeros blanked) |
| SOC / error | 9, 8, 7 | SOC % (ST+7) with a BMS, else `---`. Menu 3/4: start/stop SOC. Error: `E` + 2 digits of ST+0x50. |
| Battery V | 31?, 30, 12 (+DP) | ST+0x54 in 0.1 V. Menu 2: charge V (+0x36). Menu 5: start V (+0x18). Menu 6: stop V (+0x1A). |
| SOC bar graph | SEG16 COM7…COM0 | 8 bars at SOC > 0/13/27/39/51/63/75/87 %, animated every 300 ms while charging |
| Icons | SEG28 COM7 / COM6 / COM5 / COM2 / COM1 / COM0 | error / idle / charging / BMS-online (ST+9 == 0) / always on / always on |
| Menu indicators | SEG25 COM3 / COM4 / COM5, SEG28 COM4 | which menu item is active |

The remaining fixed segments (unit labels) are set unconditionally in each render function: 08009c58, 08009eb4, 0800a1b0, 0800a4f8,
0800ab58 and 0800abd4. To reuse the glass, replicate these functions or run them under an emulator to dump the bit map.

### 3.2 UART4 frames (460800 8N1, RS485 via DE pin, RX = DMA + IDLE line)

UART4 is only ever used for the following:

**(a) `B5 5B` factory-test protocol.** It is Modbus-RTU-like with a 2-byte prefix. **[V]**
```
B5 5B | addr | 03 | 10 | reg | 02 | 00 | val | crcL crcH        (11 bytes)
  addr 01 = tester→charger, 02 = charger→tester
  CRC = Modbus CRC-16 (poly 0xA001, init 0xFFFF) over bytes [2..8], i.e. excluding B5 5B, low byte first.
```
- `B5 5B 01 03 10 00 02 00 01 AA 30`: enter test mode. The reply is `B5 5B 02 03 10 00 02 00 01 99 30`; both LEDs turn on
  and flag 0x20000A50 = 1 (v2 0800b564, v1 080069ac). In test mode the main loop runs 08007df0 instead of normal control.
- In test mode, each key press sends `B5 5B 02 03 10 01 02 00 0k crc` with k = 1..4 for PC6/PB15/PA12/PA11
  (v2 08011960, e.g. k=1 → `98 CC`).
- `B5 5B 01 03 10 05 02 00 01 F5 14` → PC1 on, reply `…05 02 00 01 99 FC`. `…00 34 D4` → PC1 off (v2 08013804).
- Every 500 ms the board sends `B5 5B 02 03 10 03 02 [ST+0x1D] [ST+0x1C] crc` (v2 08005cbc).
- More test registers are handled in v2 08013434 (lengths 0x24, 5).
- **Nothing in normal operation is sent unsolicited on UART4**, and the panel does not send anything (there is no panel).

**(b) Enter-bootloader command.** The host sends `F9 45 F9 53 23` ("ùEùS#").
- v1 (080069ac): accepted only once ST+0x16 > 400 (uptime gate). Replies `F9 45 03 00 21 9D`, writes RTC BKP @0x40002854 = 1,
  then calls `NVIC_SystemReset`. **[V]** statically, but **on the bench (2026-10-15) v1 1.01.1105 ignored it**: SWD showed UART4 receiving
  the bytes (DMA count moving, buffer consumed), yet there was no reply, no reset flag and BKP @0x40002854 stayed 0, well past the
  uptime gate. Install by power-cycling into the bootloader instead.
- v2 (0800b564): replies `F9 45 03 10 20 51`, writes BKP @0x4000285C = 3, then resets. **[V]**
- The bootloader never reads these BKP registers (§6). The reset itself is the handoff.

**(c) `FC AA …`** with a checksum (v1 08005b0c → 08007e94; v2 08006ec8): status/info frames for the vendor app/gateway.
The v2 reply is `FC AA 00 0A 00 20 12` + version string + CRC. **[V]** for the framing; its use is **[I]**.

**(d) `FA 17 00 5E 01`** (v2 only): replies with the BLE name `ECO-WORTHY 24_xxxx` and the module MAC. **[V]**

---

## 4. Buttons and menu

There are **4 GPIO keys on the MCU**, all **active high** with internal pull-downs (§2.1). Nothing comes from a panel. **[V]**

**Driver.** The key driver is a FlexibleButton-style library (v2 080111c4 register, 08011330 FSM, 08011728 scan; v1 equivalent at 08006cc0). **[V]**
- Scanned every **10 ms** from the TIMER1 1 ms ISR (v2 app_IRQ21 → 08011728). **[V]**
- Debounce: 2 consecutive equal samples (20 ms). **[V]**
- Events: 1 = click (short press released), 5 = long-press hold (>100 scans = **1 s**). Double-click is supported by the library but unused. **[V]**
- **Wake-only press:** if the backlight timer has ≤100 ms left (i.e. it's off), the press only refreshes the backlight. **[V]** (v2 0800961c)

**Key actions** (v2 0800961c; v1 uses the same pins and is **[I]** to behave the same):

| Key | Click | Long press / hold |
|---|---|---|
| K0 PC6 | "Charge now" (+0x13 = 1) when idle | flag set, unused |
| K1 PB15 | UP: +1 on the current menu item | auto-repeat every >150 ms |
| K2 PA12 | DOWN: −1 | auto-repeat |
| K3 PA11 | next item 1→2→…→6, then exit and **save** | exit menu and save |

The menu also auto-exits and saves after **30 s** without a key press (0x20000B64 ≥ 3001 × 10 ms). **[V]**

**Menu items** (v2 08011afc, state struct 0x20001148; v1 offsets in brackets): **[V]**

| # | Setting | Unit | Range / rule |
|---|---|---|---|
| 1 | Max charge current, +0x30 [v1 +0x24] | A | 2…100; wraps 100→2 and 1→100 |
| 2 | Charge (CV) voltage, +0x36 [+0x28] | 0.1 V | 47.0…57.8 (0x1D6…0x242), wraps. Stop V is clamped ≤ charge V; start V ≤ stop V − 5.0 V. |
| 3 | Start SOC, +4 [+3] | % | 0 … < stop SOC |
| 4 | Stop SOC, +5 [+4] | % | > start SOC … 100 |
| 5 | Start voltage, +0x18 [+0x12] | 0.1 V | ≥ 42.0 (0x1A4), ≤ stop V − 5.0 V |
| 6 | Stop voltage, +0x1A [+0x14] | 0.1 V | 47.0 (≥45.0 at load) … 57.8, ≤ charge V |

The value being edited blinks (300 ms off / 300 ms on, counter +0x2C).

---

## 5. Settings persistence

The flash page size is **1 KB**: both the bootloader and the app erase in 0x400 steps (boot 08000574, v1 08005e44). **[V]**
Pages are wear-levelled logs: records are appended, and the last non-erased record wins (v1 08005f28 / v2 08008c68 scan). **[V]**

### v1 (bench board)

- **0x0801F000–0x0801F3FF**: settings log, 2 doublewords (16 bytes) per record (v1 08005c6c load, 08006018 append). **[V]**

  | Byte | Field |
  |---|---|
  | 0 | start SOC % |
  | 1 | stop SOC % |
  | 2–3 | max current A (LE) |
  | 4–5 | charge V (0.1 V, LE) |
  | 6–7 | start V (0.1 V, LE) |
  | 8–9 | stop V (0.1 V, LE) |
  | 10 | flag → state +1 (0/1) |
  | 11–15 | 0 |

  - The record is validated on load. On any error it falls back to defaults 20 % / 100 % / 10 A / 57.6 V / 42.0 V / 57.6 V and rewrites the record.
  - The bench board currently holds 8 records. The latest is `63 64 64 00 3A 02 A4 01 3A 02 01`: 99 % / 100 % / 100 A / 57.0 V / 42.0 V / 57.0 V / flag 1. **[V]**
- **0x0801E000**: written by the **bootloader**, not the app. It holds the image length (§6). **[V]**
- At boot, v1 also computes a byte-sum and CRC over 0x08000000…0x0801E008 (07c68/07cb8); this is only reported. **[V]**

### v2

Region roles are **[I]**:

| Region | Record size | Role |
|---|---|---|
| 0x0801FC00–0x0801FFFF | — | main settings, including +0x12/0x13/0x14 and the backlight timeout +0x5C (v2 07f74) |
| 0x0801EC00–0x0801EFFF | 7 dwords | charge-session stats |
| 0x0801F000–0x0801F3FF | 8 dwords | 31-day history |
| 0x0801F400–0x0801F7FF | 3 dwords | |
| 0x0801E400–0x0801E7FF | 3 dwords | RTC/time |
| 0x0801F800–0x0801FBFF | 9 dwords | WiFi/BLE configuration |
| 0x0801E000 | — | **v2 writes a 6-byte module MAC here and erases the page if it changes** (0800e880). That conflicts with the v1 bootloader's length word, which is more evidence that v2 is paired with a different bootloader. |

Factory reset (v2 08006d54) erases 0x0801E000 and 0x0801F800.

### For our firmware

- Keep the app ≤ 0x0801DFFF.
- Never touch the 0x0801E000 page.
- Put settings in **0x0801F000–0x0801FFFF**. The bootloader's update erase stops at 0x0801EFFF, so these pages **survive updates**. **[V]**

---

## 6. Bootloader (0x08000000–0x08004FFF)

### 6.1 Startup and stay-vs-jump decision

The sequence (boot 080020cc → 08000fb0 init, then loop 08001038):

1. Clock init, the same 60 MHz.
2. SysTick 1 ms.
3. **Anti-clone check** (0800082c). It computes three CRC-16s over permutations of the 96-bit UID (0x1FFFF7E8/EC/F0) and compares them with
   the halfwords at **0x080036A0/2/4/6**.
   - If all four slots are erased, it writes the first CRC (first-boot provisioning).
   - If **none match, it erases the app area 0x08005000–0x0801EFFF**.
   - The bench chip has `30 6A F9 21 53 2E 11 93` there. **[V]**
4. Init PB8/PB9/PB10 as outputs and UART4 PB0/PB1 at 460800 8N1 (RX by DMA + IDLE IRQ30, buffer 0x20000464, 0x212 = 530 bytes).
5. TIMER1 1 ms (prescaler 0x3B, period 0x3E7).
6. **FWDGT started: PSC /32, RLD 0x800 ≈ 2.0 s at IRC32K** (08000bec). **[V]** for the code; the timing is **[I]**.
7. Main loop: every received frame resets the idle counter (0x20000240). When **idle ≥ 1501 ms**, it tries to jump (080005e4). **[V]**

Jump rules (080005e4). **[V]**
- Read `len = BE32(*(u32*)0x0801E000)`. If 0xFFFFFFFF: set BKP3 (0x4000285C) = 1, wait 300 ms, **system reset**. It stays effectively in the
  bootloader: 1.5 s listen, then reset, forever.
- If `len < 9` **or** `*(u32*)(0x08005000 + len − 8) == 0xFFFFFFFF`: BKP3 = 1, then keep looping in the bootloader.
- Otherwise: BKP2 (0x40002858) = 0, NVIC ICER = 0xFFFFFFFF, require `(*(u32*)0x08005000 & 0x2FFE0000) == 0x20000000`, set MSP,
  then jump to `*(u32*)0x08005004`.
- The app's vector table is therefore at **0x08005000**. The bootloader does **not** set VTOR, does not stop SysTick or FWDGT, and checks
  **no whole-image CRC**.
- **No RAM or BKP magic is read.** The bootloader only *writes* RTC BKP2/BKP3 (08000b60), so the BKP values the app writes don't matter to it.
  The only way in is: reset, then traffic within 1.5 s. **[V]**

### 6.2 Update protocol (UART4, 460800 8N1, half-duplex RS485)

**Framing.** One frame is one burst on the wire, delimited by the UART IDLE interrupt. **[V]**

**CRC.** Standard **Modbus CRC-16**: tables at 0x08003562/0x08003462 start `00 C1 81 40…` / `00 C0 C1 01…`, init 0xFFFF.
It is sent **low byte first**. **[V]** (0800021c; checked against the replies below)

| Step | Host → board | Board → host |
|---|---|---|
| Handshake | `F9 45 F9 53 23` | `F9 45 03 00 21 9D` (bootloader; the v1 app sends the same bytes then resets) |
| Header (exactly 13 bytes; **CRC not checked**) | `FB 00 04 03 00 00 00 L3 L2 L1 L0 cL cH`. L = image length, **big-endian**. | `FB 03 00 00 00 C1 90` (CRC over 5 bytes; reply builder 0x080006c4) |
| Data | `FB nH nL 03 00 sH sL <n bytes TEA-encrypted> cL cH`. CRC is over the first n+7 bytes (header + ciphertext). | `FB 03 00 sH sL cL cH` (seq echoed) |
| Finish | stop sending | after 1.5 s idle → jump |

**Header handling** (08001038). **[V]**
- On the **first header since reset** only (flag 0x2000013C), it erases 0x08005000–0x0801EFFF, 1 KB pages.
- It sets the write offset (0x20000138) to 0.
- It sends the ACK, clears BKP3/BKP2, then programs the 4 length bytes plus 4 zero bytes as one doubleword at **0x0801E000**.
- The bench flash holds `00 00 BF 58 00 00 00 00` there (v1 app = 48 984 bytes).

**Data handling** (state machine 0800036c, states 3…8 on 0x20000000). **[V]**
- It checks `FB` and that n ≠ 0, checks `03 00`, latches seq, and copies n bytes to 0x20000038.
- It TEA-decrypts in place (8 rounds, delta 0x9E3779B9, key ASCII `"1234567123456789"` as 4 LE words; see `tools/tea_decrypt.py`).
- If the CRC matches, it programs n bytes at `0x08005000 + offset` as 64-bit doublewords (08000a34; trailing bytes are zero-padded),
  then sets `offset += n` and ACKs.
- A bad CRC gets **no reply**, and the state resets.

**Limits and caveats.** **[V]** from the RAM layout.
- **n ≤ 256** (decrypt buffer 0x20000038…0x20000137, immediately followed by the offset variable; staging buffer 0x20000140 has 32 dwords)
  and **n must be a multiple of 8**.
- **The sequence number is not checked.** The offset advances on every CRC-good frame, so **resending a frame whose ACK was lost will corrupt the
  image**. Since the erase happens only once per boot session, recovery is: reset the board, then restart from the handshake.
- Image limit: 0x0801E000 − 0x08005000 = **0x19000 (100 KB)**.

### 6.2.1 Vendor updater (static decompile, 2026-10-14)

Eco-Worthy's Windows updater, read from a static decompile. Our frames match its frames byte for byte.
- It sends one handshake to the **running stock app** (no power-cycle), waits a fixed 600 ms, then sends the header. On the bench
  v1 1.01.1105 doesn't answer that handshake (§3.2b), so a v1 board must be power-cycled into the bootloader.
- It sleeps 50 ms after each data frame.
- It checks the ack CRC.
- It resends a frame once on a garbled ack. That is unsafe, because the bootloader doesn't check sequence numbers.
- It uses one timeout budget of about 1.5 to 3 s for the whole session.
- It keeps RTS and DTR off, and flushes its buffers before every write. It can't handle echoing adapters.
- The upgrade is done with the charger on AC, through the rear "RS485 BMS" RJ45 jack.
- It downloads the firmware from the vendor server at run time. Only V1.01.1002 and V1.01.1105 are listed.
- Our `oem_image.py --stock-from-dump` output is byte-identical to the vendor's `ECOCHARGER 100 V1.01.1105.dat`
  (sha256 ed0f3d46…8175).

### 6.3 Building an image for our own app (keeping the stock bootloader)

1. Link at **0x08005000** with the vector table first. The app must, early on:
   - set **VTOR = 0x08005000**
   - **feed or keep feeding FWDGT within ~2 s** (it is already running and can't be stopped)
   - disable or reconfigure **SysTick**, which the bootloader left running at 1 ms. Until VTOR is set it vectors to the bootloader's
     handler, so set VTOR first thing in `Reset_Handler`.
   - re-init clocks and peripherals, and avoid the 0x0801E000 page.
2. Pad the image to a multiple of 8. **Make sure the last 8-byte word is not all 0xFF**, e.g. append an 8-byte non-FF trailer or
   build ID; otherwise the bootloader refuses to jump.
3. Encrypt the whole image with `tools/tea_decrypt.py in.bin out.dat --encrypt`. The vendor `.dat` is exactly this: raw ciphertext, no
   header, 80 032 bytes for v2. **[V]**
4. Host sequence at 460800 8N1 on the RS485 port:
   - power-cycle the board (v1 ignores `F9 45 F9 53 23` while running; see §3.2b)
   - repeat the handshake every ~50–100 ms until `F9 45 03 00 21 9D` (wait ~200 ms and repeat once, since v1 answers identically before
     it resets)
   - send the header and wait for its ACK (erasing 100 pages takes a moment)
   - stream 256-byte data frames, one at a time, each waiting for its ACK
   - then stay quiet for >1.5 s
5. To keep vendor-tool updates possible from our firmware, implement the `F9 45 F9 53 23` → reset command on UART4 (PB0/PB1, DE PB10).
6. The chip-UID hash at 0x080036A0 lives inside the bootloader and stays valid on this chip. Never erase the bootloader region. If it is
   ever restored from the dump onto the **same** chip, it still matches. The v1 app runs its own copy of this check (0800742c) and
   self-erases on mismatch; that is irrelevant to our app.

### 6.4 SWD alternative

The flash is unprotected (SPC = 0xA5). Our app can simply be written at 0x08005000 over SWD with a GD32L23x-aware flash algorithm. The
`stm32f1x` driver must not be used. Also write the 0x0801E000 length word, BE, followed by 4 zero bytes, so the bootloader will jump.

---

## 7. RS485 direction control and the WiFi/BLE module

**RS485 direction** (v1 board) is **PB10**: high = transmit. It is shared by USART1 (BMS, 9600) and UART4 (service/update, 460800), and the
bootloader uses it too. **[V]** The receiver is presumably enabled while DE is low (/RE tied to DE). **[I]**
If there is one transceiver, BMS polling and service traffic must never overlap. Stock firmware is single-threaded and polled, so
they don't. **[I]**

**WiFi/BLE module** (v1 board):
- Its UART3 (PA0/PA1) is initialised at 9600, but **v1 never sends anything to it**, and v1 configures **no reset or enable pin**.
- So the module just runs unattended on the bench board, and ignoring it (leave PA0/PA1 as inputs or UART idle) is safe. **[V]** for
  the firmware; whether the module is even fitted is **[I]**.

**WiFi/BLE module** (v2 board): PB7 = module reset (active low) and PD5 = probably enable/power (driven high). Holding PB7 low (or PD5
low) would silence it. **[I]**

---

## 8. Corrections to existing notes

- `docs/history/notes/findings.md` "CAN pins are PA11/PA12" is wrong: CAN is **PA4 (RX) / PA5 (TX), AF11**. PA11/PA12 are keys. **[V]**
- `docs/history/notes/findings-v2.md` "UART4 at 460800 (the B5 5B panel frames) … drives the display **[I]**" is wrong. UART4 is the RS485 service,
  factory-test and update port, and the display is the on-chip SLCD. **[V]**
- The "firmware update over RS485" uses **UART4 @ 460800**, not the 9600 BMS USART1. **[V]**

## 9. Open items needing eyes on the board

1. **Chip marking** (expect GD32L235RBT6) and the **crystal** frequencies: HXTAL should be 12 MHz. Is a 32.768 kHz LXTAL fitted?
2. **RS485 topology**: one transceiver or two? How do the USART1 TX (PA2) and UART4 TX (PB0) lines reach the transceiver(s)? Which
   connector pins carry them? What is the termination?
3. What **PC1** drives (relay, fan, PSU enable or remote-on line?), and what PC2 (backlight) and PD0/PD1 (LEDs) drive, including polarity.
4. **PC0 divider** ratio, and what it measures (battery terminal or PSU bus).
5. **CAN transceiver** supply (findings say its bus side is unpowered on the bench) and whether there is a standby/S pin. **No GPIO
   controls one in firmware.** **[V]**
6. Whether the WiFi module is fitted on the v1 board, and its power path.
7. LCD glass segment legend: the field map in §3.1 is from code, and the unit/icon labels need a look at the glass.

## 10. From board photos (2026-10-14)

The silkscreen reads "100A充电器-主控板_V1.1-250911" (100 A charger main control board, V1.1, 2025-09-11).

| Item | Seen |
|---|---|
| MCU | Partly worn marking, but the GigaDevice logo and "GD32…" are legible. Consistent with GD32L235RBT6. |
| Crystals | Y2 (metal can, expected 12 MHz) is fitted. Y1 appears unpopulated, so there is probably no 32.768 kHz LXTAL. |
| RS485 | **One transceiver** (U5) with test points `RS485_RO`, `RS485_DI`, `RS485_EN`, going to J5 (GND / B / A). Both UART TX lines must therefore share DI. |
| CAN | U4 (SOIC-8) with test points `CAN_TX`, `CAN_RX`, `CAN_H`, `CAN_L`. |
| Power | +48 V comes in on the connector beside CAN_H/CAN_L. U2 is a buck (330 inductor) to +5 V, and U1 an LDO to +3.3 V. |
| ADC | A test point labelled `ADC` sits by the +48 V divider (R72/R74/R73/C36), so PC0 most likely senses the 48 V supply, i.e. the battery/output side. |
| Relay | HF3FF 005-1Z3F (5 V coil, 10 A 250 VAC), with NO/COM/NC terminals: a generator-start dry contact. Its driver (Q1, D2, R5–R7) sits by a test point labelled **PB6**. |
| LEDs | Discrete "Charge" (green, LED1) and "Fault" (red, LED2) through-hole LEDs. Their pins come through near test points **PC1** (ESD3) and **PC2** (ESD4). |
| Backlight | `LCD_3V` / `LCD_DM` test points near Q2/Q3 are probably the backlight switch. |
| Keys | S1 "Switch", S2 "Up", S3 "Down", S4 "Enter", each with an RC filter (R16–R27, C24–C27). |
| SWD | J3, `+ DIO CLK −`. |
| WiFi/BLE | The top-right module footprint looks empty, apart from the J2 TX/RX pads. |

**Conflicts with §2 to resolve at bring-up:** §2 assigns PC1 = "charging output", PC2 = backlight, PD0/PD1 = LEDs, and PB6 is v2's
backlight. The photos suggest PB6 = relay and PC1/PC2 = LEDs. Phase 1 toggles PB6, PC1, PC2, PD0 and PD1 one at a time and records
what each one drives.
