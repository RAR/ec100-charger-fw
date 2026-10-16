# WiFi bridge (Home Assistant, YamBMS relay, remote updates)

Back to the [README](../README.md).

## Remote access through the WiFi bridge

`esphome/oem-bridge.yaml` turns the Waveshare ESP32-S3-RS485-CAN into a charger's bridge. There is one bridge per
charger, each on its own RS485 line: `esphome/eco-charger-a.yaml` and `esphome/eco-charger-b.yaml` only set the name
(`eco-charger-a` / `eco-charger-b`) and include it.
- Wire its RS485 A+/B− to the jack's A/B (as the FTDI adapter). Leave its CAN unconnected; the OEM board owns the module bus.
- The RS485 transceiver is on GPIO17 (TX), GPIO18 (RX) and GPIO21 (direction).
- **Home Assistant owns the line.** The bridge polls the charger over Modbus every 5 s (settings too: `skip_updates` has no
  effect in ESPHome 2026.9) and exposes
  the state, hold reason, error, voltages, currents, power, flags, per-module data, all settings except the Modbus
  address and baud, a Pause switch, and Charge now / Clear fault buttons.
- **Passthrough.** A "Passthrough" switch hands the line to raw TCP on port 8899 instead. The tools below flip it
  themselves; it turns itself off after 10 minutes without a TCP client, and turning it off puts the line back to
  9600. With it off, a TCP client neither hears nor reaches the charger, and the "RS485 baud" select
  ignores any rate other than 9600.
- The web server (port 80, which takes the Passthrough and "RS485 baud" requests) needs `web_username` and
  `web_password` in `esphome/secrets.yaml`. The tools read them from `OEM_BRIDGE_USER` and `OEM_BRIDGE_PASSWORD`.
- **YamBMS relay.** Each bridge subscribes to YamBMS over MQTT and writes HR 100-107 every 5 s (see
  [Modbus source](limit-sources.md#modbus-source-yambms-through-the-bridge)). In Panel mode the charger only shows the SOC (HR 104) on
  its LCD and IR 13; in Modbus mode it charges within the relayed limits.
- Line sharing is done by the local `rs485_mux` component (`esphome/components/rs485_mux`).
- Flash it once over USB: `esphome run esphome/eco-charger-a.yaml` (or `-b`). After that, use OTA.

Both tools take `--bridge HOST` in place of `--port`:

```bash
.venv/bin/python tools/oem_modbus.py --bridge eco-charger-a.local status
.venv/bin/python tools/oem_update.py --bridge eco-charger-a.local --modbus-reboot oem/build/charger.dat
```

For an update, the tool sets the bridge to 9600 for the Modbus reboot, then 460800 for the bootloader, then back to 9600.
