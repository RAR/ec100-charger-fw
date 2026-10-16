# Limit sources (BMS and YamBMS)

Where the charging limits come from: Source mode (menu item 7, HR 6). 0 Panel uses the panel settings; the others are below. The panel side is in [oem-controls.md](oem-controls.md#source-mode). Back to the [README](../README.md).

## Pylontech BMS source

The OEM firmware can take its limits from a BMS on the RS485 jack instead of the panel settings.
- **Switching.** Menu item 7, Source mode: Pylontech. Back to Panel (or Modbus) the same way. Modbus can't switch to it
  (HR 6 refuses 1 over Modbus, because it would cut the link the write came over; 0 and 2 are accepted).
- **The jack belongs to the BMS.** In this mode the charger is the master on the RS485 line: no Modbus server, so no
  Home Assistant, no bridge polling and no `--modbus-reboot`. Install updates with a power-cycle.
- **Dialect.** The stock charger's: requests `~201246{61,62,63}0000cccc` (CID1 0x46, empty INFO) at 9600 8N1, one every
  500 ms, ASCII frames. The YamBMS charger proxy (`packages/charger/charger_proxy_rs485.yaml`) speaks it; Eco-Worthy JBD
  batteries answer it (confirmed on a 100 Ah pack, 2026-10-16).
- **CCL unit.** Read as 0.1 A, as the YamBMS proxy encodes it. Stock v2's code implies 0.01 A
  (`docs/history/notes/pylon-stock-protocol.md`): a BMS sending 0.01 A would read ten times too high, still capped by Max current.
  Confirmed on real packs: an Eco-Worthy 100 Ah / 100 A pack sends 1000 (= 100 A), and a JK-PB in its Pylon mode fits
  0.1 A too (160 A per 200 A pack, 640 A for a 4-pack bank; `docs/history/notes/oem-bringup.md`).
  JK-PB packs work in this mode with their "(9600) PYLON LV RS485 V3.5" UART1 option.
- **Honoured:** charge voltage limit (CVL), charge current limit (CCL), charge enable (status bit 7), the alarm bits (a
  BMS alarm gives E12), SOC for the start/stop thresholds, the highest cell voltage and the battery voltage.
  **Not used:** the current in 0x61.
- **Stale rule.** Without a valid 0x61, 0x62 and 0x63 within 3 s the charger raises E15 (since 3.11 the alarm reply
  counts too: a BMS whose alarms can't be read isn't charged from at its own limits) and charges on the Panel limits,
  with the current capped at Fallback current (HR 9), unless the BMS's last fresh word was a stop (charge enable off or
  an alarm): then it stays stopped, with E15, until the BMS answers again. Set Fallback current to 0 to stop charging
  when the BMS is gone.
- **No bridge polling with a real BMS.** The WiFi bridge's Modbus polls would collide with the charger's requests on the
  jack. With a real BMS connected, disconnect the bridge; leave Passthrough on only while using `fake_pylon.py`.
- **Simulator.** `tools/fake_pylon.py` plays the BMS, on a serial port or through the bridge's Passthrough:

```bash
.venv/bin/python tools/fake_pylon.py --bridge eco-charger-a.local --soc 60
```

  Starting values: `--voltage --soc --max-cell --cvl --ccl --no-charge-enable --alarm`; `-q` prints only changes. Live
  commands on stdin: `set ccl 2`, `set cvl 53.8`, `set soc 90`, `enable off|on`, `alarm on|off`, `silent on|off` (stop
  answering, to see E15), `show`, `quit`. It keeps serving after stdin ends; `--control FILE` takes the same commands
  appended to FILE, for driving it from other shells: `echo "set ccl 1.5" >> FILE`.
  **On a real bank, give it `--voltage` close to the bank's and supervise:** while it is fresh, its fixed voltage
  replaces the measured battery voltage for the session gate and soft start.

## EG4 and JK BMS sources

Firmware 3.7, Source mode 3 (EG4) and 4 (JK). Live-tested on a charger with the simulators below (alarms, under-voltage
not stopping, silence, recovery); not yet run on a real EG4 or JK-B battery. Like
Pylontech, the board is the RS485 master and the jack belongs to the BMS: no Modbus server, no bridge polling, no
`--modbus-reboot`. Both are menu-only: HR 6 over Modbus refuses 1, 3, 4 and 5 (JK-PB, below). IR 5 reads 3 (EG4) or 4 (JK).
Protocol notes: [`docs/history/notes/eg4-bms-protocol.md`](history/notes/eg4-bms-protocol.md), [`docs/history/notes/jk-bms-protocol.md`](history/notes/jk-bms-protocol.md).

| | EG4 (mode 3) | JK (mode 4) |
|---|---|---|
| Battery | EG4 LL / LifePower | JK-B BMS (the JK-PB inverter series is mode 5, below) |
| Link | Modbus RTU, 9600 8N1 | JK "4E 57" frames, 115200 8N1, through the JK RS485 adapter |
| Address | 0x10, fixed: a single battery, or the master battery (DIP switches all on) | none |
| Requests | alternating reads of 0x0000 x 18 and 0x0012 x 21, one every 500 ms | read-all, one every 1 s |
| CVL | Charge V (the BMS sends none) | Charge V (the BMS sends none) |
| CCL | register 0x0016, Max charging current (A); 0 or less also stops (latched) | Max current (the BMS sends none) |
| Charge allowed | no bit of 0x0593 set (under-voltage bits still allow charging) | charging bit (0x8C bit 0) set and no alarm |
| Alarm (E12) | protection (0x001B) & 0x0593 | alarm (0x8B) & 0x3736 |
| Also read | SOC, pack V, max cell V | SOC, pack V, max cell V |

- **CVL is Charge V.** Neither BMS sends a charge-voltage limit, so the charger uses your Charge V setting. The BMS's max
  cell voltage drives the existing cell guard (Max cell V / Cell resume V, HR 12 and 13).
- **EG4 stop mask 0x0593.** Pack over-voltage, cell over-voltage, charge over-current, MOS over-temperature, charge
  over-temperature, charge under-temperature. Never pack or cell under-voltage: a flat battery must stay chargeable.
  Warnings (0x001A) are ignored. A reply with all zeros (pack V 0.00) is rejected.
- **JK alarm mask 0x3736.** The charge-relevant bits only. Low capacity, discharge faults and cell under-voltage do not
  stop charging, and neither does cell delta (bit 7): it would strand an imbalanced flat pack. The top of charge is
  covered by cell over-voltage (bit 10), the charging bit (0x8C bit 0) and the max-cell guard. The JK sends no charge limit; its 0x99 value is the over-current trip, not used.
- **JK source byte.** Requests go out with source byte 0x03. After 3 polls in a row with no valid reply it flips to 0x02,
  and back, because the reference code is inconsistent.
- **Not used.** The battery current (it is the bank's, not this charger's). Battery voltage and SOC are used as for
  Pylontech.
- **Fresh** while a valid reply is under 3 s old. EG4 needs both blocks within 3 s. A reply must arrive within 400 ms of
  its request (JK: 600 ms).
- **Stale rule.** As for Pylontech: E15, Panel limits with the current capped at Fallback current (HR 9). If the last
  fresh state was a stop (charge not allowed, or an alarm), it stays stopped until the BMS answers again or Source mode
  changes. The latch is set by any valid reply that says stop by itself, even before the source is fresh (EG4: a block B
  with a mask bit set or CCL 0, which also stops charging when fresh; Pylontech: a 0x63 with charge enable off or a 0x62 alarm); only a fresh, fine state clears it.
- **Wiring caveat.** The pinouts come from the source repos and are unverified on this charger. EG4: RJ45 RS485 port,
  pins 2 and 7 = A (+), 1 and 8 = B (-), 3 and 6 = GND; swap A and B if silent. JK: check your model's manual for the
  RS485 adapter. Do not connect a real battery until you have checked the voltages and polarity.
- **Simulators.** `tools/fake_eg4.py` and `tools/fake_jk.py` play the BMS, on a serial port or through the bridge's
  Passthrough. JK needs the bridge baud at 115200 first; EG4 uses 9600.

```bash
.venv/bin/python tools/fake_eg4.py --bridge eco-charger-a.local --soc 60 --ccl 5
.venv/bin/python tools/fake_jk.py --bridge eco-charger-a.local --soc 60
```

  Starting values: EG4 `--voltage --soc --max-cell --cells --ccl --protection`; JK `--voltage --soc --max-cell --cells
  --alarms --status`. `-q` prints only changes, `--control FILE` takes commands appended to FILE. Live commands on stdin:
  `set NAME VALUE` (EG4: voltage, soc, max-cell, ccl, cells, protection; JK: voltage, soc, max-cell, cells, alarms,
  status), `silent on|off` (stop answering, to see E15), `show`, `quit`.

## JK-PB BMS source

Source mode 5, added in firmware 3.9. Tested on a real pack (below); charging from it is not yet tested. For the JK-PB inverter series
(PB-series packs with RS485-1 and RS485-2 ports). Like the other polled sources the board is the RS485 master and the jack
belongs to the BMS: no Modbus server, no bridge polling, no `--modbus-reboot`. Menu-only: HR 6 over Modbus refuses 5. IR 5
reads 5. Protocol notes: [`docs/history/notes/jk-pb-protocol.md`](history/notes/jk-pb-protocol.md).
- **Wiring and pack setup.** Wire the jack to the pack's **RS485-1** and set that port's protocol to **001** (JK RS485
  Modbus) at 115200 8N1. RS485 pinouts vary between packs and adapters: check yours with a meter before connecting.
- **Address.** The pack's DIP address (0-15). The charger finds it itself by trying 0, then 1 to 15, one request a second,
  so up to about 16 s from switching mode to the first reply. If the pack stops answering for 3 polls it scans again.
- **Parallel bank.** Connect to the DIP-0 master pack. The charger sees only that pack, not the bank behind it.
- **CVL.** Cell count x the pack's own charge voltage per cell (its "float" voltage per cell while the pack reports the
  float stage), read from the pack's settings every 60 s. **CCL** is the pack's max charge current. Your Charge V and Max
  current still cap both.
- **Stops.** The charger stops when the pack's charge MOSFET or charge switch is off, its max charge current is set to 0,
  or on a charge-related alarm (**E12**). Cell over-voltage (alarm bit 4) also stops it but without E12, which is normal at the top of charge.
  Under-voltage never stops, so a flat pack stays chargeable. The max cell voltage drives the existing cell guard.
- **Silence.** No fresh reply for 3 s (or no settings read since the lock) gives **E15**: Panel limits with the current capped
  at Fallback current, and a stop stays latched until the pack answers again, as in the other modes.
- **Simulator.** `tools/fake_jkpb.py` plays the pack, on a serial port or through the bridge's Passthrough (bridge baud
  115200 first).

```bash
.venv/bin/python tools/fake_jkpb.py --bridge eco-charger-a.local --soc 60 --address 1
```

  Starting values: `--address --voltage --soc --max-cell --cells --alarms --charge-mos --stage --switch --rcv --rfv
  --coc` (`--rcv`/`--rfv` in V per cell, `--coc` in A). `-q` prints only changes, `--control FILE` takes commands appended
  to FILE. Live commands on stdin: `set NAME VALUE` (same names plus `address`), `silent on|off` (stop answering, to see
  E15), `show`, `quit`.
- **Tested** on a real JK-PB2A16S20P (hw 19A, sw 19.34) on RS485-1 with protocol 001: the charger found its DIP address
  and showed its voltage and SOC (2026-10-16; capture in `docs/history/notes/jk-pb-capture-rs485-1.txt`). Other PB versions untested.

## Modbus source (YamBMS through the bridge)

Firmware 3.6, Source mode 2, in daily use since 2026-10-16. The charger takes its limits from a master on the Modbus link, normally the WiFi bridge
relaying YamBMS. The board stays a Modbus server, so Home Assistant, the bridge and remote updates keep working.
- **Switching.** Menu item 7, or HR 6 over Modbus (the bridge's "Source mode" select lists Panel, Pylontech, Modbus, EG4, JK and JK-PB; the board refuses the polled sources over
  Modbus, so those are menu-only). HR 6 accepts 0 and 2 and refuses 1, 3, 4 and 5. Changing mode keeps the received data but clears the stop latch and (3.8) ends the charging session, so
  Panel to Modbus with a live bridge is fresh at once.
- **Registers.** The master writes HR 100-107 in one FC 16:

| Reg | Value | Unknown |
|---|---|---|
| 100 | heartbeat, any value | - |
| 101 | CVL, V x 100 | - |
| 102 | CCL, A x 10, total for this charger | - |
| 103 | flags: b0 charge allowed, b1 alarm, b2 blocking | - |
| 104 | SOC % | > 100 (0xFFFF) |
| 105 | battery V, V x 100 | 0 (use the ADC) |
| 106 | highest cell V, V x 1000 | 0 |
| 107 | battery current, A x 10, signed, + = charging | 0x7FFF |

- **Snapshot on a full block write.** The limits are taken from a write of all eight registers HR 100-107 in one block.
  Writes to 101-107 alone change what reads back but not the limits. A shorter write starting at HR 100 refreshes only the
  Panel SOC display, so an older 3.5 bridge config can't keep a Modbus-mode charger fresh on stale limits.
- **Fresh** while the last heartbeat is under 10 s old. Never fresh since boot counts as stale.
- **Stale rule.** As for Pylontech: E15, Panel limits with the current capped at Fallback current (HR 9). In Modbus mode, if
  the last fresh snapshot had charge allowed off, the alarm bit, the blocking bit or CCL 0, the charger stays stopped until
  a fresh one arrives or Source mode changes (Pylontech's latch covers charge allowed off and the alarm, as its spec says).
- **Caps.** Charge voltage = min(CVL, Charge V, 58.0 V); current = min(Max current, CCL), less what other chargers push in
  when HR 107 is known. With a SOC, Start SOC / Stop SOC apply. IR 5 reads 2 and IR 4 bit 2 mirrors stale.
- **Bridge relay.** `esphome/oem-bridge.yaml` subscribes to the YamBMS topics under `yambms_prefix` (default
  `yambms/yambms_1`): `requested_charge_voltage`, `requested_charge_current`, `charging_allowed` (binary sensor),
  `errors_bitmask_alarm`, `bms_blocking_charge`, `battery_soc`, `total_voltage`, `max_cell_voltage`. Every 5 s, while
  Passthrough is off and CVL and CCL have both arrived within 60 s and charging_allowed has a value, it writes HR 100-107.
  Otherwise it writes nothing, and the charger goes stale on its own after 10 s. Missing values are sent as their unknown
  codes. YamBMS publishes charging_allowed only on change (retained), so the last value is used and CVL/CCL carry the
  freshness.
- **CCL share.** YamBMS's CCL is for the whole bank, and each charger would take all of it. The `ccl_share` substitution
  (default `"1.0"`) scales it: HR 102 = CCL x share. `eco-charger-a` and `-b` use `"0.5"`.
- **Battery current is not relayed** (HR 107 = 0x7FFF). YamBMS's current includes the other charger and the inverter. Two
  chargers each subtracting the other, 5 s late, could wander. The firmware supports HR 107; the bridge doesn't send it.
- **Known limitation.** After a bridge reboot or MQTT reconnect, the broker's retained CVL, CCL and charging_allowed are
  relayed as fresh for up to 60 s, even if YamBMS itself is down.
- **Home Assistant.** New diagnostic sensors show the relayed CVL and CCL.
- **Bridge build.** ESPHome 2026.9.1. `register_count` is gone: the bridge uses `reuse_previous_range: true` to keep IR 0-39
  one request.
- **Status.** Live on both chargers (2026-10-16): limits follow YamBMS x 0.5, E15 within 10 s when the relay stops,
  fresh again on the next write; charged at 75 and 100 A per charger (see `docs/history/notes/oem-bringup.md`).
