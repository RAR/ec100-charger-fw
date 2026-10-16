# OEM board bring-up log

## Bootloader round trip (phase 1 Task 3, 2026-10-15)

- **Adapters.** FTDI FT232R USB-RS485 (serial A50285BI) on J5. It doesn't echo its own transmission, and it drives the line with
  RTS and DTR off. ST-Link/V2 clone (V2J37S7) on J3, reading a target voltage of 3.28 V.
- **SWD backup.** `dumps/gd32_flash_20261015-pre-oem.bin` is byte-identical to `dumps/gd32_flash_20261012.bin`, settings page
  included. CPUID 0x411CD200, DBG_ID 0x23050410, version string "1.01.1105".
- **RS485 receive.** The stock app polls a Pylontech BMS at 9600 8N1 with `~201246600000FDAB\r` about every 0.5 s.
- **Stock-app hand-over doesn't happen.** v1 1.01.1105 ignores `F9 45 F9 53 23` while it's running. SWD showed UART4 receiving the bytes:
  the DMA ch3 count moved and the buffer at 0x20000bb9 was consumed. There was still no reply, no new reset flag, and BKP @0x40002854
  stayed 0. The README, `oem_update.py` and `oem-hw-map.md` §3.2b were corrected to say so.
- **Install.** `oem_update.py --wait 90 build/stock-v1.dat`, then a power-cycle. The handshake was caught on the way up and
  192/192 frames (256 bytes each) were acked in a few seconds.
- **Result.** The stock UI came back and the 9600 BMS polling resumed. An SWD read of 0x08005000 + 0xBF58 bytes matched the dump.

## First image (phase 1 Task 5, 2026-10-15)

- `oem/build/blink.dat` (1384 bytes, 6 frames) was installed with a power-cycle. The bootloader jumped to it.
- The green **Charge** LED blinks at 1 Hz, so **PD1 = Charge (green)** and, by elimination, PD0 = Fault (red).
- SWD, read-only with the core running:
  - VTOR = 0x08005000;
  - RCU_CFG0 = 0x000D040A, so SCSS = PLL and the crystal path is up (not the 5 Hz HXTAL-failure blink);
  - GPIOD OCTL bit 1 toggles between samples.

  (RSTSCK reads 0 here, but that proves nothing. Our startup latches the flags into `g_reset_flags` and clears them. Steady
  blinking is the evidence that no watchdog reset happened.)

## Bring-up app (phase 1 Task 9, 2026-10-15)

- `oem/build/bringup.dat` (15 frames) was installed with a power-cycle.
- STATUS screen: AC 60 (HXTAL/PLL up), battery field 0.1 (reset cause 1 = power-on), uptime counting.
- **Keys.** The bench held the board upside down and named the keys by position, so the reads came out mirrored: PA11 = 8 was
  "Switch" … PC6 = 1 was "Enter". Ruling (user): follow the stock firmware's assignment:
  - PC6 = Switch (charge now)
  - PB15 = Up
  - PA12 = Down
  - PA11 = Enter (menu/save)

  This matches `board.h`, which is unchanged. All four keys read correctly, as active high.
- **GPIO probe** (Enter steps through the outputs):
  1. PC1 high → **the relay clicks**. PC1 is the relay (generator contact), as `board::RELAY` already had it.
  2. PB6 high → nothing visible. It stays unused.
  3. PD0 low → **red** LED (Fault).
  4. PD1 low → **green** LED (Charge).
  5. PC2 low → **the backlight goes off** (active high).
- **RS485 RX wiring.** The board sits display-down in a probe fixture, so this was checked over SWD instead of on the RX screen.
  GPIO IDR was sampled 8× idle and 8× while streaming 0x00 at 9600 (line low ~90%):
  - PA3: `11111111` → `00010000`
  - PB1: `11111111` → `00000000`

  **Both USART1 RX (PA3) and UART4 RX (PB1) are wired to the transceiver's RO**, as the spec assumes.
- **Watchdog.** Up+Down held for about 5 s, then the board restarted.
  - RSTSCK is cleared by our startup, so the latched copy was read instead: `g_reset_flags` (0x20000004 in bringup.elf) = 0x20000000,
    which is **FWDGTRSTF**.
  - Starving the watchdog resets the board, and the app sees the cause on the next boot.
- **Bootloader re-entry and revert.** `oem_update.py --wait 90 build/stock-v1.dat` was started, then Switch+Enter was held for 3 s in the
  bring-up app. The software reset opened the bootloader window and 192/192 frames were acked. An SWD read of the app area matched the
  dump, and the stock app resumed its 9600 BMS polling.
- **Segment walk** was not done: the board sits display-down in the probe fixture. The glass legend will instead be checked by comparing
  the charger app's screens with the stock screens (phase 2, Task 13).

## Phase 2: fake modules (Task 13, partial, 2026-10-15)

Bench: the board was on **5 V only** (no 48 V supply), display-down in the probe fixture. The CANable (candleLight/gs_usb) ran
`fake_nasn.py` as the only other node. Checks were made from the fake's log and SWD, not the display.

- **Install.** `charger.dat` (88 frames) was installed with a power-cycle. Before the CANable was opened, the board's CAN showed all
  3 mailboxes pending, TEC 128 and ACK errors (nothing ACKing), which also proved no live modules were on the bus.
- **Online.** Both fake modules were seen within ms of the fake starting. The first frames were standby / 0x0100 / 0 A only. The
  frames queued before the fake started held no charging command.
- **Auto-start on 5 V (finding).** The panel session started by itself: the ADC reads ≈0 V on 5 V, and the gate starts when
  battery V < Start V. The modules went ON at 57.60 V and ramped 1 A/s to 5 A each (Max current 10 A default). Stop needs
  battery V ≥ Stop V, so on a dead sense line it would never stop. This matches Opus review I-2; a plausibility guard or a
  module-vout cross-check is proposed.
- **Keepalive.** Setpoints were re-sent every 5 s.
- **Pause** (hold Switch 2 s): within about 10 ms both modules got STANDBY, then 0 A, and the relay clicked open at once, as designed
  (it opens immediately on pause or fault).
- **Resume** (hold Switch 2 s): 0 A, then ON, then a 1 A/s ramp back to 5 A. The relay clicked closed.
- **Outage and stale replay.**
  - The fake was stopped while charging.
  - A process kill doesn't take a gs_usb CANable off the bus: it keeps ACKing until it is closed. Closing it from python-can on
    macOS wedged the adapter until it was replugged. Use a USB replug to simulate an outage.
  - With nothing ACKing, the board's 3 mailboxes stayed pending (error-passive).
  - On return, the first frames each module saw were STANDBY, the voltage setpoint and 0 A. There was no stale ON or 5 A. About 1 s
    later the modules were switched ON at 0 A and ramped from 0. **The stale-replay fix is confirmed on hardware.**
- **Relay while modules are offline.** PC1 stayed high (relay closed) while the session was active with the modules gone. That is
  intended: the generator contact keeps asking for AC.
- **CANable on macOS.** gs_usb's `start()` issued a USB port reset; macOS re-enumerates on it, so after any closed run the adapter
  either failed to open ("No such device") or ACKed on CAN while passing no frames up until it was replugged. `tools/sniff.py` now
  replaces that reset with a channel-mode reset on darwin (603b0ad). Three back-to-back clean runs all received frames. The CANable
  firmware (canable.io candleLight, sw 2 / hw 1) was not updated.
- **Fault injection** (`--alarm 1 --alarm-from 20 --alarm-until 40`):
  - alarm at 20.08 s;
  - both modules in standby at 20.31 s;
  - alarm cleared at 40.05 s;
  - both back ON at 70.31 s, exactly the 30 s auto-clear.
- **Cooldown** (`--heatsink 50 --heatsink-until 100`; resume and pause driven over SWD by writing `g_ctl.paused_`,
  0x20000222 in this charger.elf, to keep a human out of the timing):
  - ramp to 5 A;
  - pause at 25.47 s → 48.00 V / 1.0 A, ON;
  - heatsink cooled at 100.03 s → standby at 100.47 s, then 0 A, with no return to charging.
  - A first, key-driven run looked like a resume during cooldown. It was extra Switch holds on the bench (user confirmed), not a bug.
- **One module missing** (`--mute 2`, Max current 10 A): module 1 alone ramped 2 A/s to 10 A. Module 2 got no writes.
- **Watchdog under the charger app.** Up+Down held 5 s → reset. `g_reset_flags` (0x20000114) = 0x20000000 (FWDGTRSTF), uptime
  restarted. Pause is RAM-only, so after the reset the board came back un-paused, with a session active (0 V ADC on 5 V).
  Whether pause should survive a reset is a phase-3 question.
- **Not done yet** (they need the display or a 48 V supply): menu walk and persistence, the ADC against a real battery voltage, and the Max current 40 A variant of the one-module test.

## E17 guard and pause memory (a822c59, 2026-10-15)

- **Install.** The new `charger.dat` (89 frames) went in through the bootloader window opened by an Up+Down watchdog reset, with no
  power-cycle.
- **E17 on 5 V** (ADC ≈ 0 V): status HOLD/WAITING with error 0x11. There is no session, the relay (PC1) is low, and the Fault LED (PD0)
  is on. A Switch hold paused the board (`paused_` = 1, hold reason PAUSED) with E17 still shown. The relay didn't click because it was
  already open.
- **Pause memory.**
  - After the pause, BKP4 (0x40002860) read 0x50415553 "PAUS".
  - An Up+Down watchdog reset came back with FWDGTRSTF, `paused_` = 1 and BKP4 still "PAUS".
  - A power-cycle came back with the power-on flags (0x0C800000), `paused_` = 0 and BKP4 = 0.

## Modbus (phase 3a Task 7, 2026-10-15; board on 5 V in the probe fixture, FTDI on the RS485 jack)

- **First try:** no reply. SWD showed the board was still running the phase-2 charger: the length word was 0x58F0 and USART1
  was untouched. With the current `charger.dat` installed (105 frames), `oem_modbus.py status` answered at once:
  - v3.0, HOLD, waiting for start, E17 (0 V ADC), 57.60 V / 10 A, both PSUs n/a.
  - **So USART1 TX through the shared transceiver works, PB0 pull-up included.**
- **USART1 STAT idle is 0x00621090:** REA (bit 22) does set on USART1. `g_rs485_ack_timeout` = 0 and `g_rs485_tx_aborts` = 0
  throughout.
- **Settings:**
  - `get 0 1 5` returned 10 / 5760 / 5760.
  - `set 0=12` took.
  - `set 5=5790` gives exception 03 every time. The first attempt got "no reply"; see the next bullet.
  - `set 6=1` gives exception 03.
- **No echo, and a too-wide echo floor.**
  - The receive ring advanced by exactly the request length across one exchange (8 bytes, with no 7-byte reply echoed), so this
    transceiver doesn't hear its own TX.
  - The firmware's floor of tx_done + gap (6 ms) dropped back-to-back requests: 3 of 20 failed with no delay, 0 of 20 with a
    20 ms delay.
  - The floor is now the busy→idle edge pass (`discard_until(now)`). Back-to-back failures dropped to 0 of 60.
- **Soak:** 150 FC 04 reads of IR 0-39 at 10 Hz with a settings write every 2.5 s, so saves landed mid-poll: 150/150 OK.
- **Pause:** `pause on` gives HR 110 = 1 and BKP4 "PAUS". The hold reason shows paused on the next 1 s controller tick.
- **`oem_update.py --modbus-reboot`:** a full 105-frame install with no power-cycle in 4.5 s. Afterwards:
  - the pause survived (PAUR / SWRSTF works through the bootloader);
  - HR 0 kept its value;
  - uptime restarted.
- **Power-cycle (5 V unplugged and held off).**
  - Flags read 0x0C800000 (power-on), pause off (HR 110 = 0, BKP4 = 0), HR 0 = 12 kept.
  - A first, brief unplug didn't reset the MCU: the flags still showed the Modbus reboot's SWRSTF and the uptime ran on. The
    board's capacitors carry it over a short drop.

## First real-module charge, over the WiFi bridge (2026-10-15)

Board back in the unit with both NASN modules on AC (~250 V), battery 54.17 V. Waveshare running `esphome/oem-bridge.yaml`
wired to the rear RS485 jack; everything below ran from the desk with `--bridge oem-bridge.local`.

- `status` answered first try (fw 3.0); the board came up paused, both modules online in standby.
- Settings 0=3 1=5460 5=5440 (3 A, 54.6 V charge, 54.4 V stop) written in one FC 16 and read back.
- `pause off` -> CHARGING within one poll: target 54.60 V / 3.0 A, output 3.0 A ~159 W, shared 1.5 A / 1.5 A, both
  alarm words 0, heatsinks 25.7 -> 27.6 C over a minute, no errors.
- The modules report ~53.2 V output while the board's ADC reads 54.19 V at the battery (~1 V apart); the ADC agrees with the
  display. Open question: module-side measurement offset vs. a drop between module and battery terminal.
- Previous settings to restore afterwards: 0=12 1=5760 5=5760.
- ADC calibration: with 3 A flowing, a battery shunt right after the charger read 53.46 V while the board ADC read 54.19 V
  (+1.35 %). Set HR 14 (ADC trim) = -13 per mille; the board then read 53.52 V. The modules' self-reported ~53.2 V is
  ~0.3 V low. `oem_modbus.py get` now shows HR 14 signed.
- Current steps (remote, HR 0): 6 A -> 3.0/3.0 A per module, 318 W, heatsinks 31.7 -> 34.4 C over 2.5 min; 12 A ->
  6.0/6.0 A, ~638 W, heatsinks 34.7 -> 41.5 C over 4 min and flattening (~+0.2 C per 15 s at the end). Sharing exact,
  alarm words 0 throughout, battery steady at ~53.55 V. Modules report no fan data (n/a).
- Stop logic: stop V 53.50 with 12 A flowing did not end the session (by design: the voltage stop needs output <= 4 A for
  10 s, session_gate.h). Dropping HR 0 to 3 A ended it ~10 s later: HOLD "waiting for start", relay open, 0 A, modules
  online. Resting battery 53.53 V vs 53.55 V at 12 A. Stop V restored to 54.40, `charge-now` restarted the session; the
  first poll showed 3.9 A against a 3.0 A target (start-up overshoot), 3.0 A from the next poll on.
- Start-up overshoot, traced at ~5 Hz over the bridge (pause -> unpause; module currents refresh about once a second):
  the firmware ramp (2 A/s) reaches its target in ~2 s, but the modules deliver nothing for ~6-7 s after standby-off,
  then come on with a fixed burst before their current limit takes hold:
  | setting | headroom (target - battery) | peak (P1 + P2) | back to setting |
  | 3 A | ~1.0 V | 3.7 + 4.0 = 7.7 A | ~2 s |
  | 2 A | ~1.0 V | 3.7 + 3.8 = 7.5 A | ~3 s |
  | 3 A | ~0.1 V (charge = stop = 53.70) | 3.7 + 2.2 = 5.7 A | ~2 s |
  The burst doesn't scale with ILIMIT and only partly with voltage headroom: ~3.5-4 A per module for 1-2 s per wake.
  Not yet tested: whether it follows the modules' stored default current (HR 16, 5 A per module now). Settings restored
  to 3 A / 54.60 / 54.40 afterwards.
- Module stored defaults ruled out as the burst source (each set, then a reboot, then a `charge-now` traced):
  HR 15 = 48.00 V (below the battery) -> 4.1 + 4.1 A; HR 15 = 54.00 V, HR 16 = 2 A -> 4.1 + 4.1 A. HR 16 left at 2.
  The burst reads the same ~4 A per module whatever the current limit, voltage headroom or stored defaults, and the
  battery-side ADC moves only ~0.02 V between rest and a full 12 A, so it can't tell a real 8 A from a bad reading.
  Leading hypothesis now: the modules' current measurement is invalid for the first reading(s) after leaving standby.
- Gotcha: after HR 120 the stock bootloader only jumps to the app after 1.5 s of RS485 silence, so polling every
  < 1.5 s keeps the board in the bootloader indefinitely. Wait ~3 s before the first request after a reboot.
- The burst is real current: a DC clamp on MAX read 8.65 A when the charger was switched on at a 3 A setting.
- First remote install (firmware 3.1, diagnostic override HR 130-133) through the bridge with
  `oem_update.py --bridge oem-bridge.local --modbus-reboot`. The first try missed the bootloader window: macOS spent ~5 s
  per new connection resolving `oem-bridge.local` (HTTP baud switch and TCP connect each), so the tools now resolve the
  name once (ec52187). Second try: 107 frames, 5.5 s end to end; 3.1 came up with the settings and the pause kept.
- Diagnostic override (3.1) tests on PSU1 alone, clamp on MAX (no peak mode):
  | wake at | limit | then | module-reported | clamp MAX |
  | 54.60 V (above battery) | 0 A | - | 2.6-2.8 A for ~3 s | 2.88 A |
  | 53.00 V (below battery) | 0 A | - | 0 A | ~0 |
  | 53.00 V | 1.5 A | step to 54.60 V after 12 s | 0.2, 1.9, 1.5 A | 4.41 A |
  | battery - 1 V | 1.5 A | ramp 0.2 V/s after 12 s | 0.3, 2.4, 1.5 A | 4.4 A |
  | battery - 1 V | 1.5 A | ramp 0.05 V/s after 12 s | 3.7, 1.6, 1.5 A | 4.4 A |
  So two things: a wake burst (~3 A for ~3 s, ignoring the limit) that a below-battery wake removes, and a short spike
  (~4.4 A, one sample) when the module starts to conduct, whatever the ramp rate. Firmware 3.2: soft-start (wake 1 V
  below the battery, hold 12 s, 0.2 V/s up) with PSU2's hold 5 s longer so the two spikes don't coincide.
- 3.2 full start (both modules, 3 A): no wake burst, but PSU2 started conducting ~1 s after PSU1, not 5 s: it
  conducted with its target at 53.58 V, below the 53.78 V battery, while PSU1 needed ~0.5 V more (calibrations
  differ). Clamp MAX 7 A (8.65 A before). 3.3: PSU2 holds until PSU1 has carried >= 0.5 A for 2 s (or PSU1 isn't
  running, or 30 s have passed).
- 3.3 full start: PSU2 still conducted in the same reading as PSU1 (2.4 + 2.6 A) although its target was held 1 V
  below the battery, so a held module follows the other one once that one conducts (current sharing between the
  modules?). Clamp MAX 7 A. Diag: PSU1 alone at 54.60 V / 1.5 A (burst 4.2 A, then 1.5 A), then PSU2 added at the
  same values while PSU1 carried 1.5 A: PSU2 burst 4.1-4.2 A for ~3 s, PSU1 unchanged; clamp MAX 6 A.
- Decision (2026-10-15): keep 3.3. The sustained wake burst is gone; the coinciding conduction spikes (clamp 7 A on a
  3 A start, < 1 s) are documented as a known limitation in the README ("Module start-up"). Untried: keep PSU2 in standby
  until PSU1 conducts.
- Menu walk on the unit (user, 3.3): menus looked right; afterwards HR 0-16 read back unchanged over the bridge
  (3 A, 54.60/54.40 V, trim -13, HR 16 = 2) and charging carried on at 3.0 A with no error.

## Bridge: Home Assistant over Modbus with an authenticated passthrough (2026-10-15)

`rs485_mux` + HA entities (plan docs/history/superpowers/plans/2026-10-15-oem-bridge-home-assistant.md), OTA to the garage bridge,
charger paused by the user throughout. Live checks:
- web server without credentials: 401; raw TCP to 8899 with Passthrough off: 0 bytes back;
- HA entities: firmware 3.3, state/hold/error text, battery 53.68 V, PSU1 alarm word "00000200";
- `oem_modbus.py --bridge` with polling on: Passthrough on -> status -> off, 1.35 s;
- from HA: Max current 3 -> 4 (read back over the CLI) -> 3; Stop voltage 55.00 (above Charge V) refused, stayed 54.40
  and the entity snapped back on the next read; Pause entity showed the user's pause;
- `oem_update.py --bridge --modbus-reboot` (same 3.3 image) with polling on: 109 frames, 9.2 s; Passthrough back off,
  baud 9600, HA reading again at charger uptime 13 s, pause kept;
- Passthrough on by hand with no client: off by itself after 606 s, baud 9600.
Not checked (charger paused on purpose): Pause and Charge now from HA.

Two chargers on one battery bank, one bridge each (own RS485 line: no shared bootloader, both keep address 1). The garage
bridge is now `eco-charger-b` (esphome/eco-charger-b.yaml); `eco-charger-a` is the second Waveshare, for the other charger
(still on the stock v1.01.1105 firmware both chargers came with; V2.0.1.1 was only downloaded from the vendor to study).
- Bridge A (second Waveshare) flashed over USB as `eco-charger-a` (a LAN address), MQTT added to the shared config.
  Charger A taken off stock remotely: `oem_update.py --bridge eco-charger-a.local` + a power-cycle, 109 frames, 3.3
  booted. Set to 3 A / 54.60 / 54.40 V straight away (defaults were 10 A / 57.60 / 57.60). Modules online after AC came
  back; left waiting for start. ADC not yet trimmed (reads 54.22 V, likely ~1.3 % high like B).

## Pylontech source, live on charger A (firmware 3.4, 2026-10-15)

3.4 installed on charger A over its bridge (`--modbus-reboot`, 117 frames) in Panel mode; Fallback current set to 0
for the test. `tools/fake_pylon.py --bridge eco-charger-a.local --voltage 53.16 --soc 60 --cvl 54.6 --ccl 3` played the
BMS through the bridge's Passthrough, driven by `--control FILE`; the user switched Source mode to Pylontech on the panel,
unpaused (Switch held 2 s) and started a session (Switch click). Observed on the panel (one-digit current):
- CVL 54.6 / CCL 3 -> 3 A; CCL 2.5 -> 2-3 A; CVL 53.0 (below the 53.16 V bank) -> 0 A; CVL back -> charging.
- Charge enable off -> 0 A, no error; then silent -> E15, stayed stopped (stop latch); BMS back with enable on -> E15
  cleared, charging resumed.
- Alarm (0x62 INFO[2] = 0x02) -> E12, stopped; cleared -> resumed.
- Silent while normal, Fallback 0 -> E15, stopped.
- A paused charger stays paused across the mode switch; a CCL below 2 A can't start a session (START_BUDGET_A).
Back to Panel from the menu: Modbus answers again at once (charging 3.0 A, source 0). Fallback current restored to 10 A.

## 3.6 Modbus source: live on charger A (firmware 3.7, 2026-10-16)

3.7 installed on charger A over its bridge (`--modbus-reboot`, 135 frames; settings and pause kept), then bridge A
OTA'd with the YamBMS relay. Charger paused throughout. Live:
- Panel mode: HR 100-107 = 1 / 5700 / 1375 (YamBMS 275 A x 0.5) / 1 / 49 / 5238 / 3278 / 0x7FFF, rewritten every 5 s.
- HR 6 = 2 over Modbus: source 2, CVL 54.60 (YamBMS 57.00 capped by Charge V), CCL 137.5 A, budget 3 A (Max current),
  battery V from HR 105; HR 6 = 1, 3, 4 refused (illegal value).
- Passthrough on: E15 + "source stale" within 15 s, CCL 3 A (min(Max current, Fallback 10)), ADC voltage, SOC n/a;
  Passthrough off: fresh again within 12 s, E15 gone.
- SOC 49 <= Start SOC 80 opened a session at once in Modbus mode (as specified); the session stays active after the
  switch back to Panel (the gate isn't reset on a mode change), so unpausing in Panel would charge to Stop V.
- Back to Panel (HR 6 = 0).

### Original plan (2026-10-15)

Firmware 3.6 (Source mode 2, limits from HR 100-107) and the bridge's YamBMS relay (ESPHome 2026.9.1, `ccl_share` 0.5 on
both bridges) are built and host-tested. The chargers still run 3.5 and the bridges the previous config. The live test will
check:
- flash one charger and its bridge; Source mode Modbus from HA; IR 14/15 follow YamBMS CVL/CCL x share;
- stop the relay (Passthrough on): E15 after 10 s, limits fall back to Fallback current;
- back to Panel (HR 6 = 0), and HR 6 = 1 is refused.

## 3.7 EG4/JK sources: live on charger A with the simulators (2026-10-16)

Charger A on 3.7, paused; the user switched Source mode on the panel and read the display; simulators through bridge A's
Passthrough. All passed:
- EG4 (`fake_eg4.py`, 9600): SOC 60 shown, no error; protection 0x0001 (pack OV) -> E12; 0x000C (pack + cell UV) -> no
  error; silent -> E15, SOC blank; back -> cleared; pack V 0.00 replies rejected -> E15.
- JK (`fake_jk.py`, 115200): SOC 70 shown, no error; alarms 0x0400 (cell OV) -> E12; 0x0080 (cell delta) -> no error;
  silent -> E15; back -> cleared, the poller had flipped to source byte 0x02 and was answered.
- Back to Panel from the menu; Modbus and the YamBMS relay answered at once.
Not observable while paused (host-tested only): the stop latch and charging current following the BMS.

### Original notes

Firmware 3.7 adds Source mode 3 (EG4 BMS, Modbus RTU 9600, address 0x10) and 4 (JK-B BMS, 115200, "4E 57"), built
overnight on the user's invitation, host-tested only (spec `docs/history/superpowers/specs/2026-10-16-oem-eg4-jk-sources-design.md`).
Pushed 2026-10-16; charger A runs 3.7 (Modbus source live-tested, EG4/JK not yet), charger B 3.5.

Rulings made without the user, confirmed by the user on 2026-10-16:
- EG4 address is fixed at 0x10, not a setting.
- JK-PB inverter series (different protocol) is not built; listed as a follow-up.
- CVL = the panel's Charge V for both; no derivation from the BMS's protection thresholds.
- JK CCL = Max current (0x99 is the over-current trip, not a limit). EG4 CCL = register 0x0016, taken as amps.
- EG4 stop mask 0x0593 (pack OV, cell OV, charge OC, MOS OT, charge OT, charge UT; never UV); the Status word's protect
  nibble and warnings are not used; a pack-voltage-0.00 reply is rejected.
- JK alarm mask 0x3736 (cell delta, bit 7, dropped); "battery dropped" bit 3 is not used (a healthy real capture shows it clear).
- JK polled every 1 s (EG4 every 500 ms); JK source byte 0x03, flipping to 0x02 after 3 silent polls.
- HR 6 refuses 1, 3 and 4; the polled modes are menu-only. IR 5 reads 3 / 4.
- JK alarm mask drops cell delta (bit 7); unknown bits 12/13 kept as stops.
- Downgrading to 3.6 clamps a stored HR 6 = 3 or 4 to 2 (Modbus) and gives E15 on Fallback current: set Source mode to Panel before downgrading.
- Pinouts (EG4 RJ45 pins, JK RS485 adapter) are from the source repos and unverified on this charger.

Live checklist (one charger plus its bridge):
- watch `g_rx_overruns` (rs485_rx_overruns) and the JK/EG4 bad-reply counters (bad_replies) during the live test; the RX ring is 512 bytes;
- flash 3.7 over the bridge; Source mode EG4 from the panel; `fake_eg4.py --bridge ... --ccl 3` through Passthrough
  (bridge baud 9600); limits follow Charge V and the CCL; protection 0x0001 gives E12 and stops; `silent on` gives E15
  and the stop latch; a zero-voltage reply is ignored;
- the same for JK with the bridge at 115200: `fake_jk.py`, `--alarms` and `--status` bit 0 off stops; silent gives E15;
- HR 6 = 3 and 4 refused over Modbus; IR 5 reads 3 / 4; back to Panel from the menu;
- a real EG4 or JK battery if the user wants, after checking the wiring.

## 3.8: a Source mode change ends the session (2026-10-16)

User's call after the 3.6 live test. Installed on both chargers over their bridges (settings and pause kept). Live on A:
Panel -> Modbus opened a session (SOC 48 <= Start SOC 80), Modbus -> Panel ended it at once.

## 3.9 JK-PB source: built and host-tested, not flashed (2026-10-16)

Firmware 3.9 adds Source mode 5 (JK-PB BMS: FC 0x10 trigger writes to 0x1620 cell info and 0x161E settings, 115200 8N1,
DIP address scanned 0, 1..15), spec `docs/history/superpowers/specs/2026-10-16-oem-jkpb-source-design.md`. Host tests and
`tools/fake_jkpb.py` only at first. Live results below.

Live checklist (one charger plus its bridge):
- flash 3.9 over the bridge; bridge Passthrough baud 115200; `fake_jkpb.py --bridge ... --soc 60`;
- Source mode (item 7) = 5 from the panel; SOC 60 shown, no error; HR 6 = 5 refused over Modbus, IR 5 reads 5;
- `--alarms 0x20` (alarm bit 5) -> E12 and stop; `--alarms 0x10` (bit 4) -> no error, charging not allowed;
- `silent on` -> E15 (Fallback current, stop latch if it was stopped); `silent off` -> cleared;
- the address scan: `--address 7` locks within about 16 s; `set address` mid-run rescans;
- back to Panel from the menu; Modbus answers again;
- a real pack only after checking the RS485-1 pinout with a meter and the port protocol (001, 115200); if it stays silent,
  that is the RS485-1 trigger question, not necessarily the wiring.

### Real pack, 2026-10-16

JK-PB2A16S20P hw 19A sw 19.34 ("JK-01" in the user's bank, a parallel member polled by YamBMS on RS485-2), DIP 4, UART1
protocol 001. Charger A (3.9) wired straight to the pack's RS485-1, Source mode 5: the panel showed 53.7 V and SOC 70 %,
matching the pack. Then bridge A on the pack's RS485-1 instead, with the firmware's own requests (scratch probe): silent on
0-3 and 5-15, answered on 4 with 308-byte units (record + trailer, sums and CRCs good) for cell info and settings:
16 cells 3.357-3.360 V, 53.73 V, +22.0 A, SOC 70, alarms 0, charge MOS 1; RCV 3550, RFV 3350, COC 200 A, switch 1.
So CVL 56.80 V (capped by Charge V 54.60) and CCL 200 A (capped by Max current 3 A). Capture: `docs/history/notes/jk-pb-capture-rs485-1.txt`.

### Same pack in Pylontech mode (UART1 "PYLON LV RS485", 2026-10-16)

With the 115200 Pylon option: silent at 9600; at 115200 it answers the stock charger's own requests (VER 20, ADR 12, CID1 46, CID2
61/62/63) and replies with ADR 02 (DIP 4; the firmware doesn't check ADR):
- 0x61: `D1EA 1D0B 45 ...0D22...`: 53.738 V, SOC 69, max cell 3.362 V (current 0x1D0B = 74.35 A at 0.01 A; unexplained);
- 0x62: all zero;
- 0x63: `DDE0 A280 1900 1A90 C0`: CVL 56.800 V (= 16 x RCV 3.550), DVL 41.6 V, CCL raw 6400, DCL raw 6800, charge and
  discharge enable. CCL 6400: 640 A at 0.1 A (our 3.4+ decode) or 64 A at 0.01 A. It is a bank total (see below).
- the JK Modbus trigger is silent while the port is in Pylon mode (one protocol per port).
- Off the parallel bus (pack alone, same Pylon mode): 0x61 current raw 2173 = 21.73 A at 0.01 A (JK protocol: +22.0 A;
  10 mA is the usual 0x61 unit); 0x63 CCL raw 1600, DCL raw 1700, CVL 56.8 V. On the bus the same pack reported bank
  totals (CCL 6400 = 4 x 1600). The pack's settings: max charge and max discharge current both 200 A. So 0x63 at 0.1 A
  gives CCL 160 A (0.8 x 200 A) per pack and 640 A for 4 packs, which is plausible and matches the Pylontech spec as
  commonly read and the YamBMS proxy: **0.1 A is kept** (an earlier note here claimed 0.01 A; that was wrong, 640 A is a
  4-pack total, not one pack). The stock Eco-Worthy v2 code's 0.01 A is that vendor's dialect.
- The pack's 9600-baud Pylon option (UART1 "(9600) PYLON LV RS485 V3.5") works with Source mode 1 as is (user, charger A
  on 3.9, live): the charger showed and used it. Re-checked over the bridge: 0x61/0x62/0x63 all answer at 9600.

### Eco-Worthy 48 V 100 Ah pack (100 A BMS), Pylon RS485 (2026-10-16)

Pack "id 2" (not the master) on bridge A, Passthrough at 9600, the stock charger's requests: silent on the line when
idle; answers ADR 0x12 only (replies ADR 12), not 0x02 or 0x01.
- 0x61: `D1E2 0229 3C ... 0D20`: 53.730 V, current 5.53 A (0.01 A), SOC 60, max cell 3.360 V;
- 0x62: all zero;
- 0x63: `E420 AF00 03E8 03E8 C0`: CVL 58.40 V (16 x 3.65), DVL 44.80 V, CCL = DCL raw 1000, charge/discharge enable.
CCL 1000 = 100 A at 0.1 A = the pack's 100 A BMS rating, so **the 0x63 current unit is 0.1 A** for Eco-Worthy as for
the JK-PB: the firmware's decode (since 3.4) is right. Stock v2's CCL x 0.005 reading of the same value is just its
own (conservative) scaling. Source mode 1 works with this pack as is.

## First full-power run, both chargers in Modbus mode (3.9, 2026-10-16)

User asked to match the FlexBOSS inverters (closed loop on YamBMS: bulk 57.0 / float 54.0, rebulk 53 V / 85 %). Both
chargers: Source mode 2, Charge V 57.00, Stop V 57.00, Start SOC 85, Stop SOC 100. First real charging in Modbus mode:
- 75 A each: 74.9 A (37.4-37.5 A per module, exact sharing), ~4.0 kW each, no errors or module alarms; hottest heatsink
  59 -> 85 C over ~6 min, levelling (fans confirmed running by the user).
- 100 A each: 99.9 A, ~5.4 kW each, no errors; hottest heatsink 85 -> 91 C over ~2 min, levelling in the low 90s.
- Back to 75 A (user: both together stay under a 10 kW genset). Paused: all four modules "cooling" with fans on,
  heatsinks 90 -> 62-66 C within a minute, as designed (standby at <= 40 C or 10 min).

## Round trip to stock and back over the bridge, charger B (3.10, 2026-10-16)

- **To stock.** `build/stock-v1.dat` (v1.01.1105 rebuilt from `dumps/gd32_flash_20261012.bin`, 48984 bytes, 192
  frames) with `oem_update.py --bridge eco-charger-b.local --modbus-reboot`. The first attempt stopped at frame 187: an
  ack came back later than the then 1 s DATA_ACK_S over WiFi. The tool did as designed (no resend, board left in the
  bootloader). A retry started before the power-cycle reached the still-running bootloader at once and failed on frame
  1 with a stale ack for frame 187 (`FB 03 00 00 BB ...`), which is why the power-cycle has to come first. After the
  user's power-cycle the board stayed in the bootloader (unfinished app), and an install with DATA_ACK_S = 3 s took all
  192 frames. Stock came up: the stock screen (user), and its Pylontech poll `~201246600000FDAB\r` every 0.5 s on the
  bridge.
- **Back.** `oem_update.py --bridge eco-charger-b.local oem/build/charger.dat` (3.10): the bootloader answered about
  3 s after the handshake started, with no power-cycle (the user thinks the stock app notices the handshake and resets;
  on the bench, 2026-10-15, it ignored it). 146/146 frames; 3.10 came up paused, and HR 0-6, 9, 14 and 16 read back
  exactly as before the round trip.
- `oem_update.py` DATA_ACK_S is now 3 s.
