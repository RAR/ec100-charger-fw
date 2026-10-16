# Development

Tools, repository layout and CI. Back to the [README](../README.md).

## Tools (`tools/`)

Run with `.venv/bin/python` (tests: `.venv/bin/python -m pytest tools`):

| Script | What it does |
|---|---|
| `fake_nasn.py` | Emulates two modules, including the CAN-silence fallback, for bench-testing the firmware. **Never run it on a bus with real modules.** |
| `fake_pylon.py`, `fake_eg4.py`, `fake_jk.py`, `fake_jkpb.py` | BMS simulators for Source modes 1, 3, 4 and 5, on a serial port or through the bridge's Passthrough (see each source's section). |
| `gs_usb_macos.py` | macOS workarounds for candleLight/gs_usb adapters, imported by `fake_nasn.py`. |
| `oem_image.py` | Builds an install image (`.dat`) for the stock bootloader: pads, adds a trailer, and encrypts. It can also re-encrypt the stock app from a flash dump. |
| `oem_update.py` | Installs a `.dat` through the stock bootloader over RS485. `--modbus-reboot` first asks the running app to reset into the bootloader. |
| `oem_modbus.py` | Modbus RTU client for the OEM firmware: decoded status, register reads and writes, and commands. |
| `tea_decrypt.py` | The stock bootloader's update cipher (TEA, 8 rounds), used by `oem_image.py`. |

## Repository layout

| Path | What it is |
|---|---|
| `oem/` | The firmware: CMake build, linker script, startup, drivers (`src/hw/`) and the header-only app (`src/app/`). |
| `tests/` | Host C++ unit tests (`make -C tests test`, `-Werror`). |
| `tools/` | Python tools: installer, Modbus client, image builder, BMS and module simulators, with their tests. |
| `esphome/` | The WiFi bridge (`oem-bridge.yaml`, `eco-charger-a/-b.yaml`, `components/rs485_mux`, `components/charger_update`) and the ESP32-controller path. |
| `docs/oem-controls.md` | The panel guide. |
| `docs/esp32-controller.md` | The ESP32-controller path. |
| `docs/history/notes/` | Reverse-engineering and bring-up notes: the stock firmware analysis, hardware map, init sequences, BMS protocol notes and captures, the module register map, and the bench/live log. |
| `docs/history/superpowers/` | The design specs and implementation plans the firmware was built from. |

Kept out of the repo on purpose: vendor firmware images and decrypted dumps (`dumps/`, `*.dat`), Ghidra projects and
decompiled output (`ghidra/`, `re/`), raw CAN captures (`captures/`) and vendor PDFs.

## Continuous integration

`.github/workflows/ci.yml` runs on every push to `main` and every pull request:
- **Host tests:** `make -C tests test` (all C++ unit tests, `-Werror`).
- **Tool tests:** `pytest tools` on Python 3.13 with `requirements.txt`.
- **OEM firmware:** builds `oem/` with the Arm GNU Toolchain 15.3.rel2 (downloaded from Arm, SHA-256 checked, cached),
  reports FLASH/RAM in the run summary and uploads `charger.dat`/`.elf`/`.bin`.
- **Release:** pushing a tag `vX.Y` that matches `OEM_FW_VERSION` (`oem/src/app/modbus_map.h`, e.g. `0x030A` = `v3.10`)
  publishes a GitHub release with the firmware and a `manifest.json` (version, file, size, SHA-256) that the WiFi
  bridge's `charger_update` reads. A mismatched tag fails the release job.

The ESPHome configs aren't built in CI (they need `esphome/secrets.yaml`); build them locally with `esphome compile`.
