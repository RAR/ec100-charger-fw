# OEM control board firmware: design

**Date:** 2026-10-14
**Status:** approved in chat, awaiting spec review

## Goal

Replacement firmware for the Eco-Worthy charger's own control board, a GD32L235RBT6 with a segment LCD, 4 keys, CAN and RS485.
With it, the board charges on its own from the two NASN RM4850-B modules, using the same charge logic as the ESPHome controller.
RS485 provides configuration and telemetry (Modbus RTU), or polls a Pylontech BMS for its limits.

The stock firmware stays recoverable. We keep the stock bootloader, and the stock app can be reinstalled through it at any time.

## Hardware facts this design relies on

The full map is in `docs/history/notes/oem-hw-map.md`, with evidence tags. Summary:

| What | Detail |
|---|---|
| MCU | GD32L235RBT6: Cortex-M23, no FPU, 128 KB flash (1 KB pages), 24 KB SRAM. 12 MHz HXTAL × 5 = 60 MHz; APB1 30 MHz. |
| CAN | PA4 RX / PA5 TX (AF11). 125 kbps, 29-bit IDs. |
| RS485 | **One transceiver**, DE on PB10 (high = transmit). USART1 (PA2/PA3) and UART4 (PB0/PB1) both reach it. |
| LCD | On-chip SLCD: 8 COM × 19 SEG, 1/8 duty, external bias on PD6. One digit per SEG column, font bit *k* on COM *k*. |
| Keys | Active high, internal pull-downs: PC6 = Switch, PB15 = Up, PA12 = Down, PA11 = Enter. |
| ADC | PC0 (ADC_IN10) senses the 48 V supply. Stock scale: raw × 0.16923 = 0.1 V units. |
| Outputs | Relay (generator dry contact), Charge LED, Fault LED, backlight. Candidate pins are PB6, PC1, PC2, PD0 and PD1; the exact roles are confirmed in phase 1. |
| Bootloader | 0x08000000–0x08004FFF, left untouched. Listens on UART4 at 460800 for 1.5 s after reset, then jumps to 0x08005000 if the length word at 0x0801E000 is valid. It starts a ~2 s FWDGT that can't be stopped, and leaves SysTick running. |

## Decisions

| Topic | Decision |
|---|---|
| Role | Standalone charger. The ESP32 becomes an optional Modbus bridge. |
| Limit source | Selectable: **Panel**, **Pylontech** (board is the bus master) or **Modbus** (board is a slave). |
| Updates | Through the stock bootloader over RS485. SWD only for debugging or recovery. |
| Language | C++17, bare metal, arm-none-eabi-gcc + CMake, no vendor HAL. GigaDevice's CMSIS device header is used for register definitions only. |
| Config protocol | Modbus RTU. |
| Structure | A 1 ms cooperative super-loop. No RTOS. |
| Relay | Generator contact. *Auto* by default (closed while a charge session is wanted). |

## 1. Structure

```
oem/
  CMakeLists.txt, cmake/arm-none-eabi.cmake
  ld/app.ld                       FLASH 0x08005000 len 0x19000 (100 KB), RAM 0x20000000 len 24K
  third_party/gd32l23x/           CMSIS core + device header, with its BSD-3 license
  src/startup.cpp                 vector table; Reset_Handler sets VTOR first, then .data/.bss, then main
  src/hw/                         clock, systick, fwdgt, gpio, can, uart (+ RS485 DE), slcd, keys, adc, flash
  src/app/                        hardware-free logic, host-tested
    rectifier.h/.cpp              one NASN module: poll schedule, setpoints, readback checks, stored defaults
    controller.h/.cpp             wires the sources, gate, state machine, rectifiers and relay together each tick
    session_gate.h                start/stop rules (§2.3)
    limit_source.h                Panel / Pylontech / Modbus → common Limits struct, with freshness
    pylon_master.h/.cpp           Pylontech request builder and parser
    modbus_slave.h/.cpp           RTU framing, CRC, register map (§4)
    settings.h/.cpp               serialisation and the two-page store (§5.2)
    ui.h/.cpp                     screens, menu, key events
    lcd_render.h/.cpp             screen model → 8-word SLCD framebuffer
    keys.h                        debounce / click / long-press / repeat state machine
  src/main.cpp                    init, then loop: run each task when due, feed FWDGT once per pass
tools/oem_image.py                pad, trailer, TEA-encrypt → .dat
tools/oem_update.py               install a .dat through the stock bootloader
tests/                            new test_oem_*.cpp files, built by the existing Makefile
esphome/oem-bridge.yaml           phase 4
```

**Shared code.** `charge_logic.h` and `nasn_protocol.h` are compiled into the GD32 build from their current directories, through an include
path. The GD32 build can't afford heap use on hot paths, so small portability changes to these headers are allowed. One example is replacing
`std::string last_fault_` with `const char *`, since every fault reason is a literal. Any such change must keep the ESP build and
`make -C tests` passing. No other ESP code changes.

**Main loop tasks:**

| Task | Period |
|---|---|
| CAN RX drain | Every pass |
| Rectifier poll | One request in flight per module, ≥ 20 ms spacing, 5 s reply timeout |
| RS485 | Every pass: Modbus RX/TX, or the Pylontech poll cycle |
| Keys | 10 ms |
| UI / LCD | 200 ms |
| ADC | 100 ms, 10-sample average |
| Controller | 1 s, matching the ESP's tick |
| Settings save | When dirty, 5 s after the last change |
| FWDGT | Fed at the end of each full pass. Never fed from an interrupt. |

**Interrupts** are used only for SysTick (1 ms time base), CAN RX (into a ring buffer), and UART RX/TX (ring buffers, DE control on TC).

## 2. Charging

### 2.1 Limit sources

Every source produces the same `Limits` struct:

- CVL, CCL;
- charge allowed, alarm, blocking;
- SOC, battery V, battery current and max cell V, each optional;
- `fresh`.

| Source | Where the values come from | Fresh while |
|---|---|---|
| **Panel** | CVL = *Charge V*, CCL = *Max current*, allowed = true. No SOC, battery current or cell data. | Always |
| **Pylontech** | USART1, 9600 8N1. The board polls CID2 0x60 → 0x61 → 0x62 → 0x63 every 500 ms, at ADR 0x12, VER 0x20, CID1 0x46.<br>0x61 gives total V, current, SOC and max cell V. 0x62 gives the alarm bits. 0x63 gives CVL, CCL and the charge-enable bit. | A good reply within the last 2 s |
| **Modbus** | Holding registers 101–107 (§4), written by the master. | Heartbeat (HR 100) written within the last 10 s |

Unlike stock, the Pylontech **charge-enable bit and CVL are honoured**.

**Panel caps always apply:**
- Target V = min(CVL, *Charge V*, `HARD_MAX_V` 58.0).
- The current cap is *Max current*.

**Stale non-panel source:**
- Charging falls back to Panel limits, with the total current capped at *Fallback current* (default 10 A).
- The display shows warning E15 and the hold reason reflects it.

### 2.2 Control (shared with the ESP)

These come from `charge_logic.h` and behave exactly as on the ESP. `Rectifier` reimplements the ESP component's per-module behaviour with the
same constants from `nasn_protocol.h`.

- **Budget:**
  - `min(Max current, CCL − max(0, battery current − our output))`, with battery current taken from the source when known.
  - Split across healthy modules at ≤ 50 A each, capped at 60 A total below 180 V AC.
  - Ramped at 2 A/s.
- **State machine:** BOOT → WAIT_BMS → CHARGING ⇄ HOLD, plus FAULT. In Panel mode the source is always fresh.
- **Faults:**
  - 0x0183 alarm bits under mask 0x0852501F;
  - a setpoint readback mismatch lasting 10 s;
  - output overvoltage.

  A fault auto-clears 30 s after its condition is gone. *Clear fault* clears it at once.
- **Cooldown before standby, and the no-current warning:** same constants as the ESP. The battery V input is the source's value when fresh,
  else the ADC.
- **Stored defaults:** 0x0101/0x0104 are written to *Fallback voltage* (≤ 54.4 V) and *Fallback current per module* (≤ 10 A). If this
  board hangs, the FWDGT resets it within about 2 s. If it stays dead, the modules fall back to those stored values.
- **Battery voltage:** source value if fresh, else ADC on PC0. The ADC scale has a trim setting (HR 14). When a module is on, its
  0x0184 reading is shown in telemetry next to the ADC value, so the trim can be checked.

### 2.3 Session gate (stock semantics, new pure class)

A **session** is the period in which charging is wanted. The state machine sees `allowed = source.allowed && session.active`. When the
session is inactive, the hold reason is "waiting for start condition".

| Event | Condition |
|---|---|
| Start | (SOC known and SOC ≤ *Start SOC*) **or** battery V < *Start V*, held for 5 s; **or** a *Switch* click (charge now) |
| Stop | (SOC known and SOC ≥ *Stop SOC*) **or** (battery V ≥ *Stop V* **and** output ≤ 4 A), held for 10 s |
| Clamps | *Stop V* ≤ *Charge V*; *Start V* ≤ *Stop V* − 5.0 V; *Start SOC* < *Stop SOC*. Applied on every settings change. |

- The session starts **active** after boot if the start condition holds.
- *Pause* (a 2 s hold on *Switch*, or HR 110) is separate from the session. It maps to the existing `paused` input.
  A pause survives a watchdog reset (kept in RTC backup register BKP4), so a hang can't resume charging. It also
  survives a Modbus reboot (HR 120), which writes `PAUR` to BKP4 just before the software reset. A power-cycle or any
  other reset clears it.

### 2.4 Relay (generator contact)

| *Relay mode* | Behaviour |
|---|---|
| **Auto** (default) | Closed while a session is active, whether or not AC is present: the generator is what brings AC. Opens 10 s after the session ends, on pause, or on a fault that isn't caused by missing AC. |
| **Off** | Always open. |
| **Modbus** | Follows HR 112. |

## 3. Display and keys

**Main screen** (stock glass layout, `docs/history/notes/oem-hw-map.md` §3.1):
- AC V, output A, W, battery V;
- SOC or `---`, and `Exx` on an error;
- the SOC bars (animated while charging);
- the error, idle, charging and BMS-online icons.

The fixed unit/label segment bits are extracted from the stock render functions in phase 1. They are recorded as a table in `lcd_render.cpp`.

**Error codes:**

| Code | Meaning |
|---|---|
| E1–E11 | Module alarm bits, using the stock mapping |
| E12 | BMS alarm |
| E13 | Setpoint readback mismatch |
| E14 | Output overvoltage |
| E15 | Limit source stale (warning: charging continues on the fallback) |
| E16 | Charging without current (warning) |
| E17 | Battery reading implausible (NaN or below 20 V with no SOC known): no session can start, a running one ends, and the relay opens. Added after the 2026-10-15 bench showed an open sense line (≈0 V) starting a session that could never end. |

Faults take precedence over warnings. The Fault LED is on for E1–E14 and E17, and blinks for E15–E16.

**Keys** (debounce over 2 × 10 ms samples, long press at 1 s, auto-repeat every 150 ms):

| Key | Click | Hold |
|---|---|---|
| Switch | Charge now (starts a session) | 2 s: pause / resume |
| Up / Down | Change the value in the menu | Auto-repeat |
| Enter | Enter the menu, then go to the next item. After the last item, exit and save. | Exit and save |

The menu also exits and saves after 30 s idle. Any key press wakes the backlight first; that wake press does nothing else. The backlight
turns off after *Backlight timeout*.

**Menu items:**

| # | Item | Range | Shown in |
|---|---|---|---|
| 1 | Max current | 2–100 A | A field |
| 2 | Charge V | 47.0–57.8 V | V field |
| 3 | Start SOC | 0–99 % | SOC field |
| 4 | Stop SOC | 1–100 % | SOC field |
| 5 | Start V | 42.0 V – (Stop V − 5.0) | V field |
| 6 | Stop V | 47.0 V – Charge V | V field |
| 7 | Source mode | 0 Panel / 1 Pylontech / 2 Modbus | SOC field |
| 8 | Modbus address | 1–247 | SOC field |

## 4. Modbus RTU

- **Line:** USART1 on PA2/PA3, 8N1. Default 9600, address 1, both configurable.
- **Function codes:** 03 (read holding), 04 (read input), 06 (write single), 16 (write multiple).
- **Errors:**
  - Unknown register → exception 02.
  - Value out of range → exception 03, and the value is not stored.
  - A frame that doesn't pass CRC is ignored.
- **Values:** signed values are int16. Unknown or not-available values read as 0x7FFF (signed) or 0xFFFF (unsigned).

### Input registers (FC 04)

| Reg | Value | Unit / scale |
|---|---|---|
| 0 | Firmware version | major << 8 \| minor |
| 1 | State | 0 BOOT, 1 WAIT_BMS, 2 CHARGING, 3 HOLD, 4 FAULT |
| 2 | Hold reason | 0 none, 1 paused, 2 not allowed, 3 BMS alarm, 4 BMS blocking, 5 cell high, 6 budget < 2 A, 7 waiting for start |
| 3 | Error code | 0 or 1–16 (§3) |
| 4 | Flags | b0 cooling, b1 no current, b2 source stale, b3 session active, b4 relay closed, b5 AC present, b6 PSU1 online, b7 PSU2 online |
| 5 | Active source | 0 Panel, 1 Pylontech, 2 Modbus |
| 6 | Target voltage | V × 100 |
| 7 | Target current | A × 10 |
| 8 | Current budget | A × 10 |
| 9 | Output current (total) | A × 10 |
| 10 | Output power (total) | W |
| 11 | Battery voltage (in use) | V × 100 |
| 12 | Battery voltage (ADC) | V × 100 |
| 13 | SOC | % |
| 14 | CVL in use | V × 100 |
| 15 | CCL in use | A × 10 |
| 16–17 | Uptime | s, u32, high word first |
| 18 | Source age | s (0xFFFF = never) |
| 20–29 | PSU 1 | see below |
| 30–39 | PSU 2 | see below |

PSU block layout, at base 20 or 30:

| Offset | Value |
|---|---|
| +0 | Status: b0 online, b1 faulted, b2 cooling, b3 standby |
| +1 | Vout, V × 100 |
| +2 | Iout, A × 10 |
| +3 | AC V × 10 |
| +4 | Inlet temperature, °C × 10, signed |
| +5 | Heatsink temperature, °C × 10, signed |
| +6 | Fan 1 rpm |
| +7 | Fan 2 rpm |
| +8 | Alarm word, high 16 bits |
| +9 | Alarm word, low 16 bits |

### Holding registers (FC 03/06/16)

Settings, persisted:

| Reg | Setting | Range | Default |
|---|---|---|---|
| 0 | Max current | 2–100 A | 10 |
| 1 | Charge V, V × 100 | 4700–5780 | 5760 |
| 2 | Start SOC % | 0–99 | 20 |
| 3 | Stop SOC % | 1–100 | 100 |
| 4 | Start V, V × 100 | 4200–5280 | 4200 |
| 5 | Stop V, V × 100 | 4700–5780 | 5760 |
| 6 | Source mode | 0–2 | 0 (Panel) |
| 7 | Modbus address | 1–247 | 1 |
| 8 | Modbus baud | 0 = 9600, 1 = 19200, 2 = 38400, 3 = 115200 | 0 |
| 9 | Fallback current (stale source, total) | 0–20 A | 10 |
| 10 | Relay mode | 0 Auto, 1 Off, 2 Modbus | 0 |
| 11 | Backlight timeout, s | 0 (always on) – 3600 | 120 |
| 12 | Max cell V, V × 1000 | 3400–3580 | 3550 |
| 13 | Cell resume V, V × 1000 | 3200–3550, must be below reg 12 | 3400 |
| 14 | ADC trim, per mille | −50 … +50 | 0 |
| 15 | Module fallback V (stored default), V × 100 | 4800–5440 | 5400 |
| 16 | Module fallback A per module (stored default) | 0–10 | 5 |

The clamps from §2.3 apply to these writes too. A write that breaks a clamp gets exception 03. Mode and baud changes take effect after the
reply has been sent.

Runtime, not persisted:

| Reg | Value |
|---|---|
| 100 | Heartbeat. Any write refreshes freshness, so the master writes it at least every 5 s. |
| 101 | External CVL, V × 100 |
| 102 | External CCL, A × 10 |
| 103 | External flags: b0 charge allowed, b1 alarm, b2 blocking |
| 104 | External SOC % (0xFFFF = unknown) |
| 105 | External battery V, V × 100 (0 = use the ADC) |
| 106 | External max cell V, V × 1000 (0 = none) |
| 107 | External battery current, A × 10, signed, positive = charging (0x7FFF = unknown) |
| 110 | Pause (0/1) |
| 111 | Write 1 to clear the fault |
| 112 | Relay command (0/1, used in Modbus relay mode) |
| 113 | Write 1 to charge now |
| 120 | Write 0xB007 to reboot into the bootloader (after the reply has been sent) |
| 130 | Diagnostic override (firmware 3.1): seconds left; write 1–600 to start with HR 131–133, 0 to stop. The modules in the mask leave standby at that voltage and current whatever the session or pause says; any pause command, a fault or the timeout ends it. IR 4 bit 8 is set while it runs |
| 131 | Override voltage, V ×100: 42.00 V up to Charge V |
| 132 | Override current per module, A ×10: 0 up to Max current |
| 133 | Override modules: bit 0 = PSU1, bit 1 = PSU2 |

Registers 101–107 are used only in Modbus source mode. In the other modes they can be written but are ignored.

## 5. Image, updates and settings

### 5.1 Image and startup

- **Linking:** the app is linked at 0x08005000 and must end before 0x0801E000 (100 KB). The link fails if it doesn't fit.
- **`Reset_Handler`, in order:**
  1. Set VTOR = 0x08005000, then disable SysTick until the clock is set up again.
  2. Feed FWDGT.
  3. Copy `.data` and zero `.bss`.
  4. Set up the clocks (12 MHz HXTAL × 5).
  5. Call `main`.
- **`tools/oem_image.py`:**
  - Takes the raw binary, pads it to a multiple of 8, and appends an 8-byte trailer: `'OEM'`, version, and a CRC-16 of the image. This
    guarantees the last word is never all-0xFF.
  - TEA-encrypts the result using `tools/tea_decrypt.py`'s routine, giving `build/oem.dat`.

### 5.2 Settings store

- **Location:** two 1 KB pages at **0x0801F800 and 0x0801FC00**.
  - The stock v1 settings log at 0x0801F000 is left alone, so reinstalling the stock app finds its own settings.
  - The bootloader's 0x0801E000 page is never touched.
- **Record format:** magic, version, sequence number, the settings block, then a CRC-16.
- **Writes:** each write appends a record to the active page. When the page is full, the other page is erased and the next record goes there.
- **Load:** the record with the highest sequence number and a valid CRC wins.
- **First boot:** if neither page holds a valid record, the latest stock v1 record at 0x0801F000 is imported if one is present. Otherwise
  the defaults apply.

### 5.3 Updates (`tools/oem_update.py`)

1. **Get into the bootloader:**
   - If our app is running, write 0xB007 to HR 120 at the configured baud.
   - If the stock app is running, send `F9 45 F9 53 23` at 460800.
   - Otherwise, ask the user to power-cycle the board.
2. **Handshake:** at 460800 8N1, send `F9 45 F9 53 23` every 100 ms until the reply `F9 45 03 00 21 9D`. Wait 200 ms and send it once more,
   because the stock app answers the same way just before it resets.
3. **Header:** send the 13-byte header with the image length (big-endian) and wait for its ACK. This step erases the app area, so allow
   up to 5 s.
4. **Data:** send 256-byte encrypted frames, one at a time, each waiting for its ACK (1 s timeout).
5. **Finish:** stay silent for more than 1.5 s. The bootloader then jumps.

**Any error means starting again from step 1** (power-cycle if needed). The bootloader doesn't check sequence numbers, so retrying a
single frame corrupts the image. A board left half-flashed stays in the bootloader, which is safe.

The same tool installs the vendor `.dat` or a re-encrypted stock dump, so the stock firmware is always one command away.

**Shared RS485 transceiver:**
- Our app uses USART1 for both Modbus and Pylontech.
- UART4's TX pin (PB0) stays a high-Z input, so it can't fight USART1's TX (PA2) on the transceiver's DI line. Its
  pull-up keeps it idling high, as stock does.
- If phase 1 shows UART4 RX is also wired to RO, the app additionally listens on UART4 RX at 460800 for `F9 45 F9 53 23` and resets on it.
  This keeps the vendor updater working against our firmware.

## 6. Safety

- **Hard limits**, enforced no matter what any setting or source says:
  - 58.0 V target;
  - 50 A per module;
  - the 0x0183 alarm mask;
  - the overvoltage fault.
- **Watchdog:** fed only from the main loop, after every task has run. A stuck task resets the board within about 2 s.
- **Module defaults:** the stored defaults keep the modules at ≤ 54.4 V and ≤ 10 A each if the board stops talking to them.
- **Stale source:** charging falls back to conservative limits rather than stopping. It doesn't continue at the last external limits.
- **Last line of defence:** the BMS and its contactor. Nothing here overrides them.

## 7. Phases

Each phase ends with something usable on the bench.

**Phase 1: bring-up.**
- Toolchain, linker script, startup code, clock and watchdog.
- `oem_image.py` and `oem_update.py`. The first install goes through the stock bootloader; SWD is the fallback.
- GPIO identification: toggle PB6, PC1, PC2, PD0 and PD1 one at a time and record which drives the relay, the LEDs and the backlight.
- An SLCD test pattern, the segment label table, and keys shown on the display.
- Checking whether UART4 RX is wired to the transceiver.
- *Exit:* our image boots, survives the watchdog, shows the test pattern, and both updating and reverting to stock work.

**Phase 2: charging, Panel source.**
- CAN driver, `Rectifier`, `Controller`, `SessionGate`, the main screen and the menu.
- Testing goes bench first against `fake_nasn.py` on a CANable, then the real modules.
- *Exit:* the board charges from panel settings, and the faults, cooldown and E-codes all behave.

**Phase 3: RS485.**
- Modbus slave, Pylontech master, the source-mode switch, the stale fallback, the relay modes, and the settings import from stock.
- *Exit:* all registers are readable and writable from a USB-RS485 adapter, and a Pylontech-protocol BMS (or a simulator) drives the limits.

**Phase 4: ESP bridge (optional).**
- `esphome/oem-bridge.yaml` on the Waveshare board, using `modbus_controller`.
- It relays the YamBMS MQTT values into HR 101–107 and writes the heartbeat.
- It exposes telemetry and settings to Home Assistant.

## 8. Testing

- **Host tests** under `make -C tests`, compiled with `-Werror`:
  - `SessionGate`: start/stop, clamps, charge now;
  - `keys`: debounce, click, long press, repeat;
  - Modbus framing, CRC and the register map, including range rejects;
  - the Pylontech request builder and the parser, against captured reply bytes;
  - settings: serialisation, page rotation, the torn-write case, the stock import;
  - `lcd_render`: golden framebuffers for representative screens;
  - `Rectifier` and `Controller` against a fake CAN bus;
  - the relay logic.
- **Size:** the CMake build reports flash and RAM use, and fails above 100 KB of flash or 20 KB of static RAM (leaving 4 KB of stack).
- **Bench:** a 48 V bench supply (or a lower voltage the buck tolerates; check in phase 1), an ST-Link on J3, a CANable running
  `fake_nasn.py`, and a USB-RS485 adapter on J5.

## Out of scope

- WiFi/BLE: the module is not fitted on this board.
- The vendor cloud and app protocols.
- The `B5 5B` factory-test mode.
- Time-of-day schedules.
- Replacing the bootloader.
