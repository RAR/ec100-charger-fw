# ESP32 controller (alternative path)

This is the project's other path: it replaces the charger's stock GD32 control board with an ESP32 running ESPHome. The
main path, [replacement firmware for the stock board](../README.md), keeps the stock panel and is what runs day to day.

- The ESP32 drives the charger's two **NASN RM4850-B** rectifier modules over CAN. They speak a variant of the Huawei
  R4850G2 protocol.
- Charge limits come from a **YamBMS** instance over MQTT.
- Home Assistant can tighten those limits, pause charging, and watch everything.

Status: ran on real hardware (the first real charge reached 100 A across both modules); not in daily use.

> **Safety.** This controls about 5 kW of DC into a lithium battery. The BMS and its contactor are the last line of
> defence. This controller never overrides them.

## Hardware

- **Board:** Waveshare ESP32-S3-RS485-CAN. CAN TX is GPIO15 and RX is GPIO16.
- **Bus:** CAN-H and CAN-L go to the rectifier bus in place of the stock board.
  - The bus runs at **125 kbps** with 29-bit IDs.
  - The stock board was providing termination. Once it's removed, **fit the Waveshare 120 Ω jumper**, or the bus stays silent.
- **Module addresses:** the rectifiers are at addresses 1 and 2. Address 0 is a broadcast.

## Files

| Path | What it is |
|---|---|
| `esphome/eco-charger.yaml` | The device config: the controller, sensors, Home Assistant controls, and MQTT. |
| `esphome/components/nasn_rectifier/` | External component: one instance per module. It handles polling, setpoints, readback checks, and alarms. |
| `esphome/components/charge_controller/` | External component: the state machine, the BMS tracker, current sharing, and faults. The pure logic is in `charge_logic.h`. |
| `tests/test_nasn_protocol.cpp`, `tests/test_charge_logic.cpp` | Host tests for the protocol and the charge logic (`make -C tests test`). |
| `tools/fake_nasn.py` | Emulates two modules on a CAN adapter for bench tests. **Never run it on a bus with real modules.** |

## Building and flashing

`esphome/secrets.yaml` is gitignored. It needs these keys:

```yaml
wifi_ssid: ...
wifi_password: ...
domain: .local
api_key: "..."        # base64 API encryption key
ota_password: "..."
mqtt_host: ...
mqtt_username: ...
mqtt_password: ...
```

Build and flash with:

```bash
cd esphome
esphome run eco-charger.yaml --device <ip>   # use `run`, not `upload`: upload flashes the last build, which may be stale
```

Before an OTA starts, the firmware puts both modules into standby. The same happens on shutdown.

## How it charges

**States:** `BOOT → WAIT_BMS → CHARGING ⇄ HOLD`, plus `FAULT`.

- **WAIT_BMS** until YamBMS is online and its charge voltage, charge current and current topics are fresh (under 60 s old).
- **Voltage target:** the BMS-requested charge voltage, capped by *Max charge voltage* in Home Assistant and a hard limit of 58 V.
- **Current budget:** `min(Max charge current, CCL − max(0, battery current − our output))`.
  - The budget is 0 when CCL ≤ 0.
  - The budget is split across healthy modules, at most 50 A each.
  - Below 180 V AC (low line), the total is capped at 60 A.
- **Ramp:** the current ramps at 2 A/s, and only while at least one module is healthy.
- **Hold reasons** are listed in priority order. The *Hold reason* entity shows the active one.

  | Hold reason | When |
  |---|---|
  | paused | The *Pause charging* switch is on. It is restored across reboots. |
  | BMS not allowing charge | YamBMS reports `charging_allowed` off. |
  | BMS alarm | YamBMS reports an alarm. |
  | BMS blocking charge | YamBMS reports that it is blocking charge. |
  | cell voltage high | The max cell voltage reached *Max cell voltage* (default 3.55 V, highest allowed 3.58 V). Charging resumes only after a **live** reading at or below *Cell resume voltage* (default 3.40 V). |
  | current budget below 2 A | There isn't enough headroom to start. |

- **Faults:**
  - A module alarm. The alarm word is data[4..7] of 0x0183, masked with the stock mask 0x0852501F.
  - A setpoint readback mismatch lasting 10 s.
  - Output overvoltage.

  A fault auto-clears after its condition has been gone for 30 s. *Clear fault* clears it immediately.
- **Cooldown before standby:** standby stops a module's fans. So when a module stops delivering with its heatsink at or
  above 45 °C, it stays on instead, with its setpoint at 48 V (below the battery, so no current flows) and a 1 A
  backstop limit. It goes to standby once the heatsink is at or below 40 °C, or after 10 minutes. *Charger state* shows
  `(cooling)` meanwhile. Cooldown is skipped in FAULT, for an offline or alarming module, and when the battery is under
  49 V. OTA and shutdown still put the modules in standby at once.
- **Charging without current:** a warning, not a fault. It turns on when, for 60 s in CHARGING, at least 2 A is
  commanded and the output is under 1 A, the module output is within 0.5 V of the target, and YamBMS `total_voltage`
  says the battery is more than 1 V below the target. That pattern means the DC output isn't reaching the battery. The
  *Charging without current* problem sensor turns on, and *Charger state* shows `(no current)`.
- **CAN-silence fallback:** if CAN goes quiet, the modules revert to their stored defaults. The firmware writes the stored
  defaults as *Fallback voltage* (≤ 54.4 V) and *Fallback current* (≤ 10 A per module). The factory default current was
  about 63 A; it is now 5 A on both modules, and that setting survives a mains power cycle.

### BMS topics (YamBMS defaults)

`yambms/status` is the availability topic. These topics live under `yambms/yambms_1/`:

| Topic | Used for |
|---|---|
| `sensor/requested_charge_voltage/state` | The charge-voltage target (CVL). |
| `sensor/requested_charge_current/state` | The charge-current limit (CCL). |
| `sensor/current/state` | Battery current (positive = charging). |
| `binary_sensor/charging_allowed/state` | Allows or holds charging. |
| `sensor/bms_in_alarm/state` | Holds charging on an alarm. |
| `sensor/bms_blocking_charge/state` | Holds charging when blocking. |
| `sensor/max_cell_voltage/state` | The cell-voltage cutoff. This is volts. Don't confuse it with `max_voltage_cell`, which is a cell ID. |
| `sensor/total_voltage/state` | Battery voltage, used only for the *Charging without current* warning. |
| `sensor/charging_instruction/state` | Display only: *BMS charge stage*. |

Every topic can be overridden in the `charge_controller:` block.

### MQTT out

The device also publishes all of its entities to the broker under `eco-charger/...`, for use outside Home Assistant.
HA MQTT discovery is on, so HA may list each entity twice: once from the native API and once from MQTT.

## NASN protocol summary

| What | Detail |
|---|---|
| Read | `0x108082FE \| addr<<16` (cmd 0x82). |
| Write | `0x108081FE \| addr<<16` (cmd 0x81). Note that Huawei uses 0x80. |
| Reply | ID bit 7 is cleared. The status is the high nibble of byte 0. The register is `(b0 & 0x0F) << 8 \| b1`. The value is bytes 2–7, big-endian. |
| Setpoint encoding | Voltage is V×1023. Current is A×1023/52, confirmed on hardware: 50.02 A commanded gave 49.96 A out. |
| Telemetry encoding | Raw value /1024. |
| Key registers | VOUT setpoint 0x0100 (readback 0x022F). Live current limit 0x0103 (readback 0x022D). Stored defaults 0x0101 (V) and 0x0104 (A). Vout 0x0175/0x0184. Iout 0x0182. AC V 0x0178. Temperatures 0x0180 (inlet) and 0x0235 (heatsink). PFC bus 0x0231. Alarms 0x0183 (bit 9 = standby). Fan speeds 0x0242/0x0243 (rpm). |
| Not available | AC current, AC power and DC power (0x0170, 0x0172, 0x0173 read 0). Output power is computed as V×I. |

See [`docs/history/notes/register-map.md`](history/notes/register-map.md) for everything else.

## Diagnostics

- **`scan_registers(start, end)`:** an ESPHome API action that does a read-only scan of 0x0000–0x0FFF on both modules
  and logs each reply.
- **`probe_command(addr, cmd)`:** an ESPHome API action that sends cmd 0x40, 0x50, 0x51 or 0xD2 and logs the reply.

If a CANable wedges, reset it over USB:

```python
import usb.core
usb.core.find(idVendor=0x1D50, idProduct=0x606F).reset()
```

