# Installing the firmware

How to get, build, install and remove the replacement firmware. Back to the [README](../README.md).

## Getting the firmware

- **Prebuilt:** each tag `vX.Y` publishes a GitHub release with `charger-vX.Y.dat` (the image to install), the
  `.elf`/`.bin` and `SHA256SUMS`, built by CI from that tag. Each CI run on `main` also attaches its build as the
  `oem-firmware` artifact.
- **Tools:** `python3 -m venv .venv && .venv/bin/pip install -r requirements.txt`. You need a USB-RS485 adapter (an
  FTDI one works) on the charger's rear RJ45 RS485 jack, or the WiFi bridge below.

## Building and installing

The app is linked at `0x08005000`, behind the untouched stock bootloader. It has a maximum size of 100 KB. The original
design is in [`docs/history/superpowers/specs/2026-10-14-oem-board-firmware-design.md`](history/superpowers/specs/2026-10-14-oem-board-firmware-design.md).

**Toolchain.** The build uses the Arm GNU Toolchain 15.3.Rel2. By default it is expected at
`~/.local/arm-gnu-toolchain-15.3.rel2`. To use another location, set the `ARM_TOOLCHAIN` environment variable.

**Build.**

```bash
cmake -S oem -B oem/build && cmake --build oem/build
```

This writes `oem/build/charger.elf`, `oem/build/charger.bin` and `oem/build/charger.dat`. The app is `charger`
(the bring-up `blink` and `bringup` apps were removed after bring-up; see the git history).
With `charger` running, holding Up + Down for 5 s (the SOC field counts down) puts both modules in standby and then
restarts the board, which opens the bootloader's 1.5 s window without a power-cycle.

**Install.** Install the `.dat` that the build (or a release) produced, through the stock bootloader:

```bash
.venv/bin/python tools/oem_update.py --port /dev/cu.usbserial-XXXX oem/build/charger.dat
```

Start `oem_update.py`, then power-cycle the board while it waits. The bootloader listens for 1.5 s after reset, at
460800 baud on the RS485 jack J5. On a charger in the unit, the stock v1 app (1.01.1105) has also handed over by itself
within a few seconds of the handshake starting (2026-10-16, over the bridge); on the bench it ignored the handshake. If
nothing happens within a few seconds, power-cycle.
If the install fails on the first frame with a small-FIFO USB-RS485 adapter, retry with `--frame-bytes 64`.

**Revert to stock.** Rebuild the stock app image from a flash dump of the stock board (the stock firmware isn't in this
repository), then install it the same way. Our firmware hands over by itself, so `--modbus-reboot` (or Up + Down for
5 s) is enough; no power-cycle:

```bash
mkdir -p build && .venv/bin/python tools/oem_image.py --stock-from-dump dumps/gd32_flash_20261012.bin build/stock-v1.dat
.venv/bin/python tools/oem_update.py --port /dev/cu.usbserial-XXXX --modbus-reboot build/stock-v1.dat
# or over the WiFi bridge: tools/oem_update.py --bridge eco-charger.local --modbus-reboot build/stock-v1.dat
```

The stock app comes back with its own settings (it keeps them in a page this firmware doesn't use) and isn't paused,
so it may start charging at once. Our settings and pause survive in their own pages: installing this firmware again
later picks them up. Tested on a charger in the unit, over the bridge, both ways (2026-10-16).

**If an install fails, never resend a frame.** The bootloader doesn't check sequence numbers, so a resent frame is
programmed a second time at the next offset. Power-cycle the board (the bootloader erases only once per boot) and run `oem_update.py` again from the start. Once
the bootloader has accepted the header, the board stays in it until an install completes; an abort before the header
leaves the old app installed.
