# OEM firmware: Modbus limit source (bridge feeds YamBMS)

**Status:** design, approved in chat on 2026-10-15 (overnight build; no flashing, mode switching or pushing until the
user is back). Implements the Modbus row of [`2026-10-14-oem-board-firmware-design.md`](2026-10-14-oem-board-firmware-design.md)
§2.1 and the HR 100-107 rows of its §4. Firmware 3.6.

## 1. Decisions

- **Source mode 2 (Modbus):** the board stays the Modbus RTU server (as in Panel). The limits come from HR 101-107,
  written by the master (the WiFi bridge); they are fresh while HR 100 (heartbeat) was written within
  `EXT_FRESH_MS` (10 s, already defined for the 3.5 SOC display).
- **Register layout (unchanged from the original spec §4):**

  | Reg | Value | Unknown / none |
  |---|---|---|
  | 100 | heartbeat, any value; a write block starting at 100 refreshes freshness | - |
  | 101 | CVL, V × 100 | - |
  | 102 | CCL, A × 10, total for this charger | - |
  | 103 | flags: b0 charge allowed, b1 alarm, b2 blocking | - |
  | 104 | SOC % | > 100 (0xFFFF) |
  | 105 | battery V, V × 100 | 0 = use the ADC |
  | 106 | max cell V, V × 1000 | 0 |
  | 107 | battery current, A × 10, signed, + = charging | 0x7FFF |

- **Snapshot on a full block write:** the source takes its values from a write of all eight registers HR 100-107 in one
  block (one FC 16 carries them atomically). Writes to 101-107 alone change what is read back but not the limits. A
  shorter write starting at HR 100 refreshes only the Panel SOC display (3.5), so an older 3.5 bridge config can't keep
  a Modbus-mode charger fresh on stale limits. The bridge writes 100-107 in one FC 16.
- **Stale fallback and stop latch: same as Pylontech** (Pylontech spec §5): not fresh → Panel limits with
  ccl = min(Max current, Fallback current HR 9), `stale` (E15); if the last fresh snapshot had charge allowed off or the
  alarm bit, `allowed` stays false while stale. Modbus's latch also covers the blocking bit and CCL 0. Never fresh since boot = stale.
- **Mode change keeps the data, forgets the latch.** The bridge writes HR 100-107 in every mode (Panel uses HR 104 for
  the display), so switching Panel → Modbus with a live bridge is fresh at once (no E15 flicker). Switching modes clears
  the stop latch, as the Pylontech poller's reset does.
- **Source mode over Modbus:** HR 6 writes may now switch between Panel (0) and Modbus (2), both of which keep the
  Modbus link. Writing 1 (Pylontech) is still refused (it would cut the link the write came over); a stored 1 can only
  be reached from the menu, as today.
- **Caps unchanged:** target V = min(CVL, Charge V, 58.0 V); budget from min(Max current, CCL) and, when HR 107 is known,
  `current_budget`'s "CCL minus what others already push in".
- **SOC known → Start/Stop SOC apply** (existing session gate), as with Pylontech. The 3.5 display-only path stays for
  Panel.
- **IR 5** reports 2 in Modbus mode; IR 4 bit 2 mirrors `stale`. IR 18 (source age) stays 0 (not built).

## 2. Bridge

- Subscribes to YamBMS over MQTT, prefix substitution `yambms_prefix` (default `yambms/yambms_1`), all verified live on
  2026-10-15:

  | Topic (under the prefix) | Example | → |
  |---|---|---|
  | `sensor/requested_charge_voltage/state` | 57.0 | HR 101 |
  | `sensor/requested_charge_current/state` | 275 | HR 102 (× `ccl_share`) |
  | `binary_sensor/charging_allowed/state` | ON | HR 103 b0 |
  | `sensor/errors_bitmask_alarm/state` | 0 | HR 103 b1 (≠ 0) |
  | `sensor/bms_blocking_charge/state` | 0 | HR 103 b2 (> 0) |
  | `sensor/battery_soc/state` | 67 | HR 104 |
  | `sensor/total_voltage/state` | 52.99 | HR 105 |
  | `sensor/max_cell_voltage/state` | 3.320 | HR 106 |

- **CCL share:** YamBMS's CCL is for the whole bank, and two chargers each take it. Substitution `ccl_share`
  (default `"1.0"`); eco-charger-a/-b set `"0.5"`. HR 102 = CCL × share.
- **Battery current is not relayed (HR 107 = 0x7FFF).** YamBMS's `current` includes the other charger and the
  inverter; two chargers each subtracting the other with a 5 s lag can wander. The firmware supports it; the bridge
  doesn't send it.
- **Every 5 s**, while Passthrough is off: if CVL and CCL have both been received within the last 60 s and
  charging_allowed has a value, one FC 16 of HR 100-107 (unknown SOC/V/cell → their "unknown" codes). Otherwise
  nothing is written (the charger goes stale on its own after 10 s). Replaces the 3.5 SOC-only relay.
  charging_allowed is a binary sensor: YamBMS publishes it only on change (retained), measured 2026-10-15 (0 live
  messages in 120 s against 12 for CVL/CCL), so its last value is used; CVL/CCL carry the freshness.
- **HA:** the Source mode select lists Panel, Pylontech and Modbus; the board refuses Pylontech over Modbus (menu-only). New diagnostic sensors for the relayed CVL and CCL.

## 3. Testing

- Host: a `ModbusSource` unit (snapshot decode with each unknown code, freshness edge at 10 s, stale fallback with
  Fallback current and HR 9 = 0, stop latch on allowed-off and alarm, latch cleared on mode change, data kept across a
  mode change); modbus_map (block write at 100 snapshots, writes to 101-107 alone don't, HR 6 0↔2 accepted, 1 refused);
  controller with Modbus limits (E15 when stale, IR 5 = 2).
- Bridge: `esphome compile` of both configs (never `esphome config`, it prints secrets).
- Live (when the user is back): flash one charger and its bridge, Source mode Modbus from HA, check IR 14/15 follow
  YamBMS × share; stop the relay (Passthrough on) → E15 after 10 s with Fallback current; back to Panel.
