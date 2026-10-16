# OEM bridge: Home Assistant over Modbus, with the TCP passthrough kept

**Status:** design, approved in chat on 2026-10-15. This is phase 4 of
[`2026-10-14-oem-board-firmware-design.md`](2026-10-14-oem-board-firmware-design.md), minus the YamBMS relay (that waits
for the firmware's Modbus limit source, phase 3b).

## 1. Goal

The Waveshare ESP32-S3-RS485-CAN on the charger's RS485 jack (`esphome/oem-bridge.yaml`) already passes the line through
raw on TCP port 8899, which `tools/oem_modbus.py` and `tools/oem_update.py` use with `--bridge`. This adds Home Assistant:
the bridge polls the charger's Modbus registers itself and exposes status, per-module data, settings and commands as
ESPHome entities. The passthrough, and with it remote firmware updates, keeps working unchanged.

Out of scope: relaying BMS limits (HR 100-107), the diagnostic override (HR 130-133, CLI only), the relay command
(HR 112), reboot (HR 120), and any charger firmware change.

## 2. The constraint

Both consumers read the UART continuously: `stream_server` drains every received byte into its ring buffer whether or not
a client is connected (`StreamServerComponent::read()`), and ESPHome's `modbus` component parses every byte in its
`loop()`. On one UART each would steal the other's bytes, and suspending `modbus_controller` only stops it sending. So the
line needs an owner that hands it to one side at a time.

## 3. `rs485_mux` component

A local external component, `esphome/components/rs485_mux`. Home Assistant owns the line by default; a **Passthrough**
switch hands it to the TCP stream. The mux exists because neither reader can be paused at runtime, not to make decisions:
the switch alone sets the mode.

**Shape.** It takes the real UART (`uart_id`) and declares two ports, each a `uart::UARTComponent` subclass, so they can be
given as `uart_id` to other components:

```yaml
rs485_mux:
  id: mux
  uart_id: rs485
  modbus_port: {id: mux_modbus}
  passthrough_port: {id: mux_stream}
  idle_baud: 9600
  idle_timeout: 10min
  on_passthrough: [component.suspend: charger]
  on_modbus: [component.resume: charger]

switch:
  - platform: rs485_mux
    mux_id: mux
    name: Passthrough
```

`modbus` gets `uart_id: mux_modbus`; `stream_server` gets `uart_id: mux_stream`. The ports report the real UART's baud
rate and settings.

**Routing.** In `loop()` the mux moves bytes from the real UART to the active port's receive buffer only; the inactive
port sees no bytes. Writes from the active port go to the real UART; writes from the inactive port are dropped. On every
switch both port buffers are cleared, so a half-received frame never crosses over. In MODBUS mode (the default, and the
mode after a reboot) a TCP client can connect to port 8899 but neither hears nor reaches the charger.

**Mode.** PASSTHROUGH while the switch is on, MODBUS otherwise. Two safety nets:
- **Idle timeout.** The switch turns itself off after `idle_timeout` (10 min) with no TCP client connected (the mux is
  told by `stream_server`'s "connected" binary sensor), so a crashed tool or a forgotten switch can't leave HA blind.
- **Baud reset.** Turning passthrough off sets the real UART back to `idle_baud` (9600) and the baud select to match, so a
  tool that died mid-update can't leave the poller running at 460800.

Each transition fires `on_passthrough` / `on_modbus`; the YAML uses them to suspend and resume `modbus_controller`, so it
doesn't queue reads or writes into a muted port. The switch state shows in HA.

**The tools flip it.** `oem_modbus.py --bridge` and `oem_update.py --bridge` turn Passthrough on over HTTP (as they already
set the baud) before opening the TCP stream, and off in a `finally`. The update keeps passthrough on from the Modbus reboot
through the install, so the poller never touches the bootloader's window.

**Access.** The web server (which takes the Passthrough and baud requests) gets HTTP basic auth: `web_username` /
`web_password` in `secrets.yaml`, read by the tools from `OEM_BRIDGE_USER` / `OEM_BRIDGE_PASSWORD`. Without them the
switch can't be turned on, so port 8899 is inert. Home Assistant uses the encrypted native API and is unaffected.

**Logic split.** The idle-timeout rule (client connected / disconnected / switch on → when to turn off) lives in a plain
header, `mux_logic.h`, with no ESPHome dependency, and gets a host test under `make -C tests` like the other components.

## 4. Home Assistant entities

`modbus_controller` at address 1 on `mux_modbus`, 9600 8N1, `update_interval: 5s`. Names follow
`tools/oem_modbus.py`.

**Status (IR 0-39, FC 04, one read every 5 s):**
- text sensors: state (IR 1), hold reason (IR 2), error (IR 3, "E13 setpoint mismatch" style), firmware version (IR 0);
- sensors: target V (IR 6, /100), target current (IR 7, /10), budget (IR 8, /10), output current (IR 9, /10), output
  power (IR 10, W), battery V in use (IR 11, /100), battery V ADC (IR 12, /100), CVL (IR 14, /100), CCL (IR 15, /10),
  uptime (IR 16-17, u32, s);
- binary sensors from IR 4: cooling (b0), no-current warning (b1), session active (b3), relay closed (b4), AC present (b5),
  diagnostic override (b8);
- per module (PSU1 at IR 20, PSU2 at IR 30): online / faulted / cooling / standby binary sensors (offset 0, b0-b3), output V
  (+1, /100), output current (+2, /10), AC V (+3, /10), heatsink °C (+5, int16, /10), alarm word (+8-9, u32, hex text);
- 0xFFFF / 0x7FFF (not available) become unknown, not 6553.5.

**Settings (HR 0-16, FC 03, every 30 s via `skip_updates`), writable with FC 06** except Modbus address (HR 7) and baud
(HR 8), which are read-only sensors because changing either from HA cuts the link:

| HR | Entity | Kind, range, step |
|---|---|---|
| 0 | Max current | number, 2-100 A, 1 |
| 1 | Charge V | number, 47.00-57.80 V, 0.01 |
| 2 | Start SOC | number, 0-99 %, 1 |
| 3 | Stop SOC | number, 1-100 %, 1 |
| 4 | Start V | number, 42.00-52.80 V, 0.01 |
| 5 | Stop V | number, 47.00-57.80 V, 0.01 |
| 6 | Source mode | select Panel / Pylontech / Modbus (the firmware refuses anything but Panel today) |
| 9 | Fallback current | number, 0-20 A, 1 |
| 10 | Relay mode | select Auto / Off / Modbus |
| 11 | Backlight timeout | number, 0-3600 s, 1 |
| 12 | Max cell V | number, 3.400-3.580 V, 0.001 |
| 13 | Cell resume V | number, 3.200-3.550 V, 0.001 |
| 14 | ADC trim | number, -50-50 ‰, 1 (int16) |
| 15 | Module fallback V | number, 48.00-54.40 V, 0.01 (applies after the next reboot) |
| 16 | Module fallback A | number, 0-10 A per module, 1 (applies after the next reboot) |

The ranges are the firmware's own (`oem/src/app/settings.h`), so HA can't offer a value the firmware rejects for range;
the cross-setting clamps (Stop V ≤ Charge V, Start V ≤ Stop V − 5 V, cell resume < max cell) are still checked by the
firmware.

**Commands:** a Pause switch (HR 110, read back every 5 s, so a panel pause shows), and Charge now (HR 113 = 1) and Clear
fault (HR 111 = 1) buttons.

## 5. Errors

- The charger doesn't answer: `modbus_controller` marks the device offline and the entities go unavailable.
- A write is refused (exception 03 for a clamp, 06 while the menu is open): ESPHome logs it, and the entity returns to the
  charger's value on the next read.
- Passthrough on: the status entities keep their last values (no polling) and the Passthrough switch shows on. Turning
  it off (the tool, HA, or the idle timeout) resumes polling at 9600.
- A tool that dies mid-update: passthrough stays on and the baud at 460800 until the idle timeout (10 min after the TCP
  connection drops) turns it off and resets the baud.
- The tools without credentials, or with wrong ones: the HTTP request fails with 401 and the tool stops before touching
  the line.

## 6. Testing

- **Host:** `tests/test_rs485_mux.cpp` on `mux_logic.h`: the switch turns off `idle_timeout` after the last client
  leaves, or after being turned on with no client ever connecting; a client connecting stops the countdown; a reconnect
  restarts it; nothing times out while a client stays connected. `tools/test_oem_modbus.py`: `--bridge` turns
  Passthrough on before connecting and off afterwards, also on errors; credentials go in the Authorization header.
- **Compile:** `esphome config` and `esphome compile esphome/oem-bridge.yaml`.
- **Live, against the charger in the garage (OTA to `oem-bridge.local`):**
  - HA (or the ESPHome web UI / API) values agree with `oem_modbus.py --bridge ... status` taken right after;
  - a setting write from HA reads back over the CLI; a write that breaks a clamp is refused and snaps back;
  - Pause from HA shows on the charger and in `status`; Charge now starts a session;
  - `oem_modbus.py --bridge` works with HA polling (Passthrough on for the run, off after, polling resumes);
  - a raw TCP connection with Passthrough off gets nothing back from the charger;
  - the web server refuses the Passthrough switch without credentials;
  - Passthrough turned on by hand with no client goes off by itself after 10 min;
  - `oem_update.py --bridge --modbus-reboot` with polling on installs the current `charger.dat` (same version: no
    behaviour change), and polling resumes afterwards.

## 7. Files

```
esphome/components/rs485_mux/__init__.py     config schema, ports, triggers, codegen
esphome/components/rs485_mux/rs485_mux.h/.cpp component and the two ports
esphome/components/rs485_mux/switch/         the Passthrough switch platform
esphome/components/rs485_mux/mux_logic.h     idle-timeout rule (host-tested)
esphome/oem-bridge.yaml                       mux, modbus_controller entities, Passthrough switch, web server auth
tests/test_rs485_mux.cpp                      host test
tools/oem_modbus.py, tools/oem_update.py      turn Passthrough on/off around --bridge use; basic auth from env
README.md                                      "Remote access through the WiFi bridge": HA entities and the mux
```
