# ESPHome charger controller: design

Date: 2026-10-12
Status: approved in brainstorming, pending spec review

## Goal

Replace the Eco-Worthy charger's stock GD32 control board with an ESP32 running ESPHome. The ESP32 drives the two NASN RM4850-B rectifiers (a rebadged Huawei R4850G2) directly over CAN. Charge voltage and current come from YamBMS over MQTT, and Home Assistant can override them.

## Decisions

| Topic | Decision |
|---|---|
| Role | Full replacement; the stock board is removed from the CAN bus |
| Charge decisions | Taken by the BMS (YamBMS), with HA overrides (caps and pause) |
| Battery | Several packs in parallel, aggregated by YamBMS |
| BMS link | MQTT from YamBMS on `homeassistant.local`; the RS485 port is not used in v1 |
| Fail-safe | Stored PSU defaults of 54.0 V / 5 A; stop charging when BMS data is lost |
| Hardware | Waveshare ESP32-S3-RS485-CAN, prototyped on the bench first |

## Hardware

- **Controller:** Waveshare ESP32-S3-RS485-CAN, with 16 MB flash and an isolated CAN transceiver with TVS protection.
  - TWAI TX is on GPIO15 and RX on GPIO16. These come from community reports of `WS_GPIO.h` and are confirmed by bench step 2.
  - Fit the onboard 120 Ω CAN termination jumper.
- **Power:** the board accepts 7–36 V. Use USB-C on the bench. For the install, add a 48 V → 12 V buck rated for at least 60 V input, feeding the screw terminal.
- **CAN wiring:** CAN-H and CAN-L go to the backplane connector the stock board used. The isolated transceiver avoids ground loops through the rectifier backplane.
- **Bus:** 125 kbit/s, 29-bit extended IDs. PSU addresses 1 and 2 are set by the backplane address board.

## CAN protocol (NASN dialect, from the stock firmware)

See `docs/history/notes/findings.md` for the derivation.

| Operation | ID | Payload |
|---|---|---|
| Read register | `0x108082FE \| addr<<16` | `RR RR 00 00 00 00 00 00` |
| Write register | `0x108081FE \| addr<<16` | `RR RR` + 6-byte value |
| Reply | Same as the request, with ID bit 7 cleared | `byte0 >> 4` = status (0 = ok), register = `(byte0 & 0x0F) << 8 \| byte1`, bytes 2–7 = big-endian value |

| Register | Meaning | Encoding |
|---|---|---|
| `0x0100` | Output voltage setpoint | V × 1023, in the low 2 bytes (`00 00 00 00 HI LO`) |
| `0x022D` | Current limit | A × 1023 / 52 |
| `0x022F` | Output voltage setpoint readback (the register the stock firmware checks VOUT against) | V × 1023, the same scale as `0x0100` |
| `0x0132` | Standby | `data[3]`: 0 = on, 1 = standby (payload `00 val 00 00 00 00`) |
| `0x0175`, `0x0182`, `0x0178`, `0x0180` … | Vout, Iout, AC V, temperature | As decoded in `tools/psu_ctl.py` |
| `0x0184` | Output voltage, the alternate reading (the stock firmware measures Vout here) | ÷ 1024. Always polled next to `0x0175`, because it isn't yet known which one is right on the NASN. The controller uses the higher of the two finite readings, which is conservative for the overvoltage fault |
| `0x0183` | Alarms | `data[4..7]` as a big-endian u32 (the stock firmware's decode). It stops on mask `0x0852501F`, which is bits 0–4, 12, 14, 17, 20, 22 and 27 (E1–E11). The findings quoted `0x0855501F`, but those bits OR to `0x0852501F` |

The PSU falls back to its stored defaults after about 60 s without CAN traffic. Two points still need checking on real hardware:
- whether this applies to the NASN units,
- which registers hold the stored defaults (Huawei uses `0x0101` and `0x0104`).

## Architecture

```
YamBMS --MQTT--> [ESP32: charge_controller] --targets--> [nasn_rectifier x2] --CAN--> PSU 1, PSU 2
                          ^                                     |
         HA (native API): caps, pause, clear fault       telemetry to HA
```

There are two ESPHome external components in `esphome/components/`.

### `nasn_rectifier` (one instance per PSU address)

- Polls the telemetry registers. Each PSU exposes:
  - output V, output A, and power;
  - AC V and A;
  - input and output temperatures;
  - alarm bits, an online flag, and the current setpoints.
- Accepts `set_target(voltage, current)` and `set_standby(bool)`. It writes the setpoints when they change, and re-sends them every 5 s as a keepalive.
- At boot, it puts the PSU in standby. With `write_stored_defaults` on, it polls the stored defaults (`0x0101`/`0x0104`) and rewrites them only when the readback differs from the fallback, at most once per keepalive (they live in the module's flash).
- The fallback is capped at 54.4 V and 10 A per PSU, because the module runs on it unattended.
- Always polls the live setpoints and compares them with what was sent: the voltage written to `0x0100` against its `0x022F` readback (as the stock firmware does), plus `0x022D` and `0x0132`. `0x0100` is still polled and published, so the two can be compared on real hardware. A mismatch for 10 s continuously while online (> 0.15 V, > 1.0 A, or a different standby flag, where any nonzero `0x0132` counts as standby) marks the PSU faulted.
- `stop_now()` sends current 0 and standby at once. It runs on shutdown and at the start of an OTA, which blocks the main loop.
- Marks the PSU offline after 3 missed polls.
- The pure frame encoding and decoding lives in a header with no ESPHome dependency (`nasn_protocol.h`), so it can be unit tested.

### `charge_controller`

- Subscribes over MQTT to the topics below. The top-level `yambms/sensor/…` topics are stale retained values and are never used. All topics are retained. Observed on 2026-10-12:

  | Topic | Use | Observed behaviour |
  |---|---|---|
  | `yambms/status` | Must be `online` | Published on change |
  | `yambms/yambms_1/sensor/requested_charge_voltage/state` | CVL | About every 10 s (`57.0`) |
  | `yambms/yambms_1/sensor/requested_charge_current/state` | CCL | About every 10 s (`275`) |
  | `yambms/yambms_1/binary_sensor/charging_allowed/state` | Charge permission | `ON`/`OFF`, published on change only |
  | `yambms/yambms_1/sensor/bms_in_alarm/state` | Alarm when ≠ 0 | About every 11 s (`0`) |
  | `yambms/yambms_1/sensor/current/state` | Battery current | About every 11 s; **positive = charging** (`237.8` while in Bulk) |
  | `yambms/yambms_1/sensor/bms_blocking_charge/state` | Some pack's BMS is blocking charge when ≠ 0 (HOLD) | `0` |
  | `yambms/yambms_1/sensor/max_cell_voltage/state` | Highest cell voltage, V (cell-voltage limit) | About every 10 s (`3.335`) |
  | `yambms/yambms_1/sensor/charging_instruction/state` | Shown in HA as `BMS charge stage` only; not used for control | Text: `Bulk`, `Absorption`, `Float`, … |

  `sensor/max_voltage_cell` is a cell ID (e.g. `916`), not a voltage, and is not used. The blocking and max-cell topics are not part of the freshness check: until they arrive, blocking reads as false and the cell limit does nothing.

- **Freshness:** ESPHome's MQTT callback does not expose the retain flag. Instead, the first message on each topic after every MQTT (re)connect is treated as possibly retained: its value is stored but it does not count as an update. Data is fresh when all of the following hold:
  - the CVL, CCL and current topics have each had a counted update within the last 60 s;
  - `status` is `online`;
  - `charging_allowed` and `bms_in_alarm` have stored values.

  The `charging_allowed` and `status` topics are on-change only, so their retained value is used as-is.
- **Cell-voltage limit:** when the highest cell reaches `Max cell voltage` (default 3.55 V), the controller latches into HOLD until it falls to `Cell resume voltage` (default 3.40 V). If either threshold is non-finite, resume ≥ max, or max > 3.58 V (kept below the BMS's 3.600 V trip), both fall back to 3.55 / 3.40 V (with a logged warning), so a bad setting never disables the cutoff. A missing cell reading, or one outside 2.0–4.5 V, leaves the latch as it was. A retained replay can set the latch, and can set `bms_blocking_charge`, but only a live update can release either.
  - **If no cell reading has ever been received, the cutoff is inactive.** It is not part of the freshness check, so charging continues without it. This shows in HA as an unknown `Max cell age`, and the controller logs a warning once the reading has been missing or stale for more than 60 s.
- **Voltage target:** `min(CVL, user max V, 58.0 hard cap)`.
- **Current budget:**
  - `budget = min(user max A, CCL − max(0, battery current − our output current))`, and 0 when CCL ≤ 0. Other chargers, such as the inverter, share the CCL; a house load drawing from the battery never raises the budget above the CCL.
  - Inputs are 10 s averages.
  - Low-line derate: while any online PSU reports AC input (`0x0178`, always polled) below 180 V, the user max A is capped at 60 A total, matching the stock firmware. An unknown AC reading leaves it unchanged.
  - Ramp up at no more than 2 A/s; drop immediately.
  - Split evenly across online, non-alarmed PSUs, with a cap of 50 A per PSU.

## State machine

| State | Entered when | PSU action |
|---|---|---|
| `BOOT` | Power-up | Write stored defaults (54.0 V / 5 A), then standby |
| `WAIT_BMS` | BMS data missing or stale, or `status` ≠ online | Standby |
| `CHARGING` | Fresh data, charging allowed, no alarm, not paused, and budget ≥ 2 A to enter (stays while budget > 0) | On; push targets every 5 s |
| `HOLD` | Paused, charging not allowed, BMS alarm, BMS blocking charge, cell voltage high (latched, see the cell-voltage limit), or budget ≤ 0 | Standby |
| `FAULT` | PSU alarm bit, persistent CAN errors, a PSU not following its setpoints for 10 s, measured V > 58.5 V, or measured V > target + 0.5 V while sourcing > 1 A | Standby; latched until all conditions have been clear for 30 s, or until the `Clear fault` button is pressed |

## Fail-safe layers

1. The PSUs revert to their stored defaults (54.0 V / 5 A) about 60 s after CAN goes silent (bench check required).
2. Stale BMS data puts the PSUs in standby.
3. Hard firmware caps that HA cannot override: 58.0 V, 50 A per PSU, and 60 A total while AC input is below 180 V (low-line derate, as the stock firmware does).
4. A hardware watchdog on the ESP32.
5. The PSUs are put in standby at the start of an OTA and on shutdown.

Writing the stored defaults stays disabled by default (`write_stored_defaults: false`) until bench step 3 confirms it on a real module, because the NASN encoding of `0x0101`/`0x0104` is unverified and the stock firmware never writes them.

PSU alarm bits cause a FAULT by default (`alarm_mask: 0x0852501F`, applied to `0x0183` `data[4..7]` as a u32). This is the set of bits the stock firmware stops on. The stock firmware also commands standby, so standby can't be what sets these bits.

The `Pause charging` switch defaults to on at first flash, so nothing charges until it is switched off in HA.

If the bench test shows the NASN does not revert on CAN silence, add an external AC relay on the PSU input, driven by the controller with a hardware timeout.

## Home Assistant entities

- **Per PSU:**
  - output V, output A, and power;
  - AC V and A;
  - input and output temperatures;
  - alarm bits, online flag, and current setpoint.
- **Charger:**
  - state (`BOOT`, `WAIT_BMS`, `CHARGING`, `HOLD`, or `FAULT`; shown as `CHARGING (no PSU)` while charging with no healthy PSU);
  - target V, target A, and current budget;
  - total output power;
  - BMS data age;
  - max cell age (time since the last live `max_cell_voltage` update; diagnostic);
  - last fault reason;
  - hold reason: in HOLD, the first of `paused`, `BMS not allowing charge`, `BMS alarm`, `BMS blocking charge`, `cell voltage high`, or `current budget below 2 A` (budget below the 2 A start threshold, including ≤ 0); `waiting for BMS data` in WAIT_BMS; empty otherwise;
  - BMS charge stage: the raw YamBMS `charging_instruction` text, published on change.
- **Controls:** all are restored on boot and clamped to the hard caps.

  | Control | Type | Default |
  |---|---|---|
  | `Max charge voltage` | number | 57.0 V |
  | `Max charge current` | number, total | 100 A |
  | `Pause charging` | switch | — |
  | `Fallback voltage` | number, at most 54.4 V | 54.0 V |
  | `Fallback current` | number, per PSU, at most 10 A | 5 A |
  | `Max cell voltage` | number, 3.40–3.58 V | 3.55 V |
  | `Cell resume voltage` | number, 3.20–3.55 V | 3.40 V |
  | `Clear fault` | button | — |

- **Diagnostics:**
  - read and write counters;
  - CAN error counter;
  - raw values of the registers not yet understood (`0x021D`, `0x022F`, `0x0239`–`0x024E`).

## Repository layout

```
esphome/
  components/nasn_rectifier/     __init__.py, nasn_protocol.h, nasn_rectifier.{h,cpp}, sensor.py, number.py, switch.py
  components/charge_controller/  __init__.py, charge_logic.h (pure), charge_controller.{h,cpp}, sensor.py, number.py, switch.py, button.py
  eco-charger.yaml               device config (esp32-s3-devkitc-1, 16 MB, ESP-IDF)
  secrets.yaml                   gitignored: wifi, mqtt (dedicated user), api key
tests/                           host C++ unit tests for nasn_protocol.h and charge_logic.h
tools/                           existing Python tools (sniff, fake_psu, psu_ctl) as the bench kit
```

## Testing

1. **Host unit tests (C++ on the Mac):**
   - frame encoding and decoding;
   - target and budget calculations, with ramping and averaging;
   - every state-machine transition, including stale data, retained-only data, BMS alarms and PSU alarms.
2. **Bench with no PSUs:** the ESP32 is wired to the CANable. Extend `fake_psu.py` to emulate the NASN dialect (cmd `0x81` writes, ×1023 scaling) at both addresses. Verify:
   - GPIO15/16;
   - polling and setpoint writes;
   - the 5 s keepalive;
   - that PSUs show offline when the fake stops responding.
3. **One real PSU, no battery:**
   - First run `psu_ctl.py read` with the ESP32 off the bus, to confirm the registers and stored defaults.
   - Test the CAN-silence fallback.
   - Then let the ESP32 drive the PSU into a dummy load with `Max charge current` = 2 A.
4. **Battery:**
   - Connect both PSUs through a fuse, starting with a low `Max charge current`.
   - Confirm the CCL budget is respected while the inverter is also charging.

## Out of scope for v1

- Direct Pylon link to the BMS over RS485 as an MQTT backup.
- Reflashing or reusing the stock GD32 board.
- Firmware updates for the PSUs.
