# Bridge installs charger firmware from GitHub releases (design)

Approved by the user 2026-10-16 ("yes, refuse while charging, go ahead").

## Goal

Home Assistant shows the charger's firmware as an update entity on its bridge. **Install** (never automatic) makes the
bridge download the latest release's `.dat` from GitHub and install it through the stock bootloader, as
`tools/oem_update.py --bridge --modbus-reboot` does from a PC.

## 1. Release side

The CI release job also publishes `manifest.json`:

```json
{"version": "3.12", "file": "charger-v3.12.dat", "size": 37400, "sha256": "<64 hex>"}
```

The bridge reads `https://github.com/<repo>/releases/latest/download/manifest.json` and then
`https://github.com/<repo>/releases/download/v<version>/<file>`. GitHub answers both with redirects whose `Location`
is about 900 characters, so the bridge's `http_request` needs `buffer_size_rx` / `buffer_size_tx` of 2048. `latest`
skips pre-releases and drafts.

## 2. Charger side (firmware 3.12)

HR 120 = 0xB007 (reboot into the bootloader) goes through the same path as the Up + Down restart: both modules to
standby (`Controller::halt()`), then the reset once the reply and the standby frames have gone out (at most 3 s).
Before, the reset came straight after the reply and the modules ran on their last setpoints through the install.

## 3. Bridge side: `esphome/components/charger_update`

- `update:` platform `charger_update`; options `mux_id`, `uart_id`, `modbus_controller_id`, `http_request_id`,
  `version_sensor` (the bridge's `Charger firmware` text sensor), `repo` (default `RAR/ec100-charger-fw`), polling
  interval (default 6 h).
- **Check** (a FreeRTOS task, as ESPHome's `http_request` update): fetch the manifest; latest = its version;
  installed = the text sensor. State AVAILABLE when latest > installed (numeric major.minor), or the installed version
  is unknown (charger silent: the recovery path); NO_UPDATE otherwise.
- **Install** (`perform`): a task downloads the `.dat` (size ≤ 0x19000, a multiple of 8, equal to the manifest's) and
  checks its SHA-256. Then the main loop runs the installer state machine:
  1. Refuse if Passthrough is on.
  2. Take the line: polling suspended, the mux hands the UART to the updater (a third owner beside Modbus and
     Passthrough; the Modbus and stream ports are deaf and mute meanwhile).
  3. At 9600: read IR 0-1. **State CHARGING (2): refuse** and give the line back ("pause the charger first"). A
     Modbus exception 06 (menu open): refuse. No reply: the charger isn't running our firmware; wait up to 60 s for
     the bootloader (the user power-cycles). Otherwise write HR 120 = 0xB007 and expect the echo.
  4. At 460800: handshake (`F9 45 F9 53 23`, two `F9 45 03 00 21 9D` replies within 1 s; an echoing adapter is
     handled), the header frame (ack within 5 s), 256-byte data frames (ack within 3 s each, sequence and CRC checked,
     **never resent**), 2 s quiet.
  5. Back to 9600, line to Modbus, polling resumed; check again 15 s later.
  - Any failure after the header: error "power-cycle the charger, then Install again" (the bootloader erases only
    once per boot); the line goes back to Modbus.
- Progress in `update_info.progress` (published every 10 frames).
- The protocol core (`installer.h`) is plain C++ with an I/O interface, so it has host tests with a fake charger
  (`tests/test_charger_update.cpp`), mirroring `tools/test_oem_update.py`.

## 4. Not in scope

Automatic installs; installing the stock firmware; release channels other than `latest`.
