# EC100 charger replacement firmware

[![CI](https://github.com/RAR/ec100-charger-fw/actions/workflows/ci.yml/badge.svg)](https://github.com/RAR/ec100-charger-fw/actions/workflows/ci.yml)

Replacement firmware for the stock GD32 control board of the Eco-Worthy 48 V 100 A charger (EC100). It keeps the
stock panel, buttons and display, runs behind the untouched stock bootloader (so the stock firmware can always be
reinstalled), and drives the charger's two NASN RM4850-B rectifier modules over CAN.

> [!WARNING]
> **Not official Eco-Worthy firmware.** This is an independent project, not made, endorsed or supported by Eco-Worthy.
> Installing it will likely **void your warranty**. Use it **at your own risk**: no warranty of any kind is given.
>
> **Safety.** This drives about 5 kW of DC into a lithium battery. The BMS and its contactor are the last line of
> defence; this firmware never overrides them.

- **Charges from the panel settings, or from a BMS:** Pylontech RS485 (including Eco-Worthy packs and JK-PB packs in
  their Pylon mode), EG4 LL/LifePower, JK-B, JK-PB, or YamBMS through a WiFi bridge (Modbus).
- **Honours the BMS:** charge voltage and current limits, charge enable, alarms, SOC start/stop, a highest-cell guard;
  E15 and a capped fallback if the BMS goes quiet, staying stopped if its last word was "stop".
- **Gentler on the modules:** a soft start (no wake-up burst), a cooldown that keeps the fans running after a charge,
  module alarm codes as stock plus checks the stock firmware lacks (E13-E17).
- **Remote control:** a Modbus RTU server on the RS485 jack, and an optional ESPHome WiFi bridge for Home Assistant,
  the YamBMS relay and firmware updates (installed from GitHub releases with Home Assistant's update button).

**Status.** In daily use on two chargers charging a 16S LFP bank, with limits from YamBMS through the WiFi bridge.
Each has run at 75 A and 100 A, with both modules sharing the load evenly and the heatsinks levelling off at about
85-90 °C. Tested with real batteries: Eco-Worthy and JK-PB packs over Pylontech, and a JK-PB pack directly. EG4 and JK-B
are tested with simulators only. Test logs: [docs/history/notes/oem-bringup.md](docs/history/notes/oem-bringup.md).

## Quick start

1. Download `charger-vX.Y.dat` from the latest [release](https://github.com/RAR/ec100-charger-fw/releases).
2. Set up the tools: `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`, and connect a USB-RS485
   adapter to the charger's rear RJ45 RS485 jack.
3. Install: `.venv/bin/python tools/oem_update.py --port /dev/cu.usbserial-XXXX charger-vX.Y.dat`, then power-cycle the
   charger while it waits. Settings start at the stock defaults (10 A, 57.6 V).
4. Read the [panel guide](docs/oem-controls.md), and set Max current and Charge V for your battery before charging.

To go back to the stock firmware, see [Installing](docs/installing.md#building-and-installing) ("Revert to stock").

## Documentation

| Page | What's in it |
|---|---|
| [Panel guide](docs/oem-controls.md) | Buttons, display, LEDs, the settings menu, source modes and error codes. |
| [Installing](docs/installing.md) | Releases, building from source, installing over RS485 or the bridge, reverting to stock. |
| [Limit sources](docs/limit-sources.md) | Charging from a BMS: Pylontech (incl. Eco-Worthy and JK-PB packs), EG4, JK-B, JK-PB, and YamBMS through the bridge. |
| [WiFi bridge](docs/wifi-bridge.md) | The ESPHome bridge: Home Assistant, the YamBMS relay, Passthrough and remote updates. |
| [Modbus](docs/modbus.md) | The Modbus RTU server on the RS485 jack and the `oem_modbus.py` client. |
| [Rectifier modules](docs/modules.md) | How the modules are started (soft-start) and their known quirks. |
| [Development](docs/development.md) | Tools and simulators, repository layout, CI and releases. |
| [ESP32 controller](docs/esp32-controller.md) | The project's other path: replacing the stock board with an ESP32 running ESPHome (not in daily use). |
| [History](docs/history/) | Reverse-engineering notes, the bench and live test log, and the design specs and plans. |

## Future

- **LilyGO T-2CAN:** a two-bus board, to keep feeding the stock front-panel display while this controller drives the
  modules.
- **CAN-silence fallback:** still to be tested on real hardware.

## License

[GPL-3.0](LICENSE). Eco-Worthy is a trademark of its owner; this project is not affiliated with it.
