# Modbus RTU server

The Modbus RTU server on the RS485 jack, and the `oem_modbus.py` client. Back to the [README](../README.md).

## Modbus

The `charger` app runs a Modbus RTU server on the rear RJ45 RS485 jack (J5), the same jack the bootloader uses. The line is
9600 8N1 with address 1 (both are settings, HR 7 and 8). From a Mac, use an FTDI USB-RS485 adapter. `oem_modbus.py` opens the
port with RTS and DTR off.

```bash
T="--port /dev/cu.usbserial-XXXX"
.venv/bin/python tools/oem_modbus.py $T status --watch 2    # decoded IR 0-39
.venv/bin/python tools/oem_modbus.py $T get 0 1 4 5          # holding registers
.venv/bin/python tools/oem_modbus.py $T set 0=20             # max current, A
.venv/bin/python tools/oem_modbus.py $T set 1=5700 5=5700    # Charge V and Stop V, checked together
.venv/bin/python tools/oem_modbus.py $T pause on             # also: charge-now, clear-fault, relay on|off
```

The register map (input registers, settings HR 0-16, and the runtime registers at HR 100-120) is in
[spec §4](history/superpowers/specs/2026-10-14-oem-board-firmware-design.md). Values are raw integers, so Charge V is volts × 100.
`not available` values (0xFFFF unsigned, 0x7FFF signed) print as `n/a`. An exception reply prints its name and exits 1; no
reply exits 2. Exception 06 means the menu is open on the panel.

A write that breaks a clamp is refused with exception 03 and nothing is stored. A `set` of settings only (HR 0-16) first
reads HR 0-16, then writes the span from the lowest to the highest given register as one FC 16. The firmware checks
the result as a whole, so the order of the values doesn't matter: `set 1=5700 5=5700` works from the defaults, although
Charge V alone below Stop V would be refused. Other registers are written one run of consecutive registers at a time;
if a later write fails, the tool prints the registers it had already applied.

Relay mode 2 (Modbus, HR 10) has no dead-man until the phase-3b heartbeat: the contact stays as the master last set it
(HR 112), even if the master goes away, and also through a fault (Auto opens it on a fault; Modbus leaves that to the
master).

Input register 18 is reserved and reads 0 (the spec's "source age" was never implemented). IR 4 bit 2 says whether the
configured source is stale, and E15 in IR 3 shows the fallback.

**Diagnostic override (HR 130-133).** For bench work on the modules themselves: `oem_modbus.py ... diag 53.2 0 psu1 60`
holds PSU1 out of standby at 53.20 V and 0 A for 60 s (PSU2 stands by), whatever the session or pause says. The current
(HR 132) is per module, and the total can't exceed Max current: with both modules (`both`, mask 3) each may ask at most half
of it, and a later lower Max current caps it again. The voltage can't exceed Charge V. `diag off`, any pause command, a
fault, a BMS alarm or stop, a high cell, E17, the Up + Down restart or the timeout ends it.

A pause (HR 110 or the panel) survives a watchdog reset and a reboot by HR 120 (including `--modbus-reboot`), but not a
power-cycle.

The module fallback settings (HR 15 and 16) are sent to the modules at boot, so a change takes effect after the next reboot
(for example HR 120 = 0xB007). After a reboot keep the line quiet for ~3 s: the stock bootloader only starts the app
after 1.5 s without RS485 traffic, so polling faster keeps the board in the bootloader.

**Update without a power-cycle.** `oem_update.py --modbus-reboot` (with `--modbus-baud 9600` and `--modbus-addr 1` as defaults)
first writes HR 120 = 0xB007 at the Modbus settings, then runs the usual handshake at 460800 baud. Since 3.12 the board
first puts both modules in standby and resets about a second after the reply (at most 3 s); the bootloader window is
only 1.5 s, so the handshake starts right after the reply and keeps trying. If the board doesn't answer (it may be running stock), the tool warns
and carries on, and you power-cycle as before. If the board acked but the bootloader hasn't answered within about 5 s,
the tool says so: power-cycle the board then, while it keeps waiting. `oem_modbus.py reboot-bootloader` sends the same
request on its own.
