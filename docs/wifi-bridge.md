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

## Charger firmware updates from Home Assistant

The bridge shows the charger's firmware as an update entity, **Charger firmware update** (the `charger_update`
component). Shortly after the bridge starts, every 6 hours, and when you press the **Check charger firmware** button,
it reads the latest release's `manifest.json` from GitHub (`RAR/ec100-charger-fw` by default; set `repo:` for a
fork). When the release is newer than the charger's version, Home Assistant offers it.

**Install** (only ever when you press it):
1. The bridge downloads the release's `.dat` and checks its size and SHA-256 against the manifest.
2. It takes the RS485 line from Modbus polling and reads the charger's version and state. **While the charger is
   charging it refuses**: pause it first. It also refuses while the panel menu is open (3.12 and later) or Passthrough is on.
3. It asks the charger to reboot into the bootloader (HR 120; the charger puts its modules in standby first), installs
   the image at 460800 baud, and gives the line back to Modbus. Progress shows in Home Assistant; the whole install
   takes about 15 s. Settings and a pause are kept.

If an install fails part way, the entity says so: **power-cycle the charger, then press Install again** (the
bootloader erases the app area only once per boot, so it must be a fresh boot). A charger that doesn't answer Modbus
(stuck in the bootloader, or running the stock firmware) keeps showing the last version the bridge read (or none,
after a bridge restart), so Install stays offered; it then waits up to 60 s for you to power-cycle the charger. The
bridge talks to Modbus address 1 (the charger's default); with another address it can't ask the charger to reboot,
and you have to power-cycle it. While an install runs, the Passthrough switch and the RS485 baud select are refused.
