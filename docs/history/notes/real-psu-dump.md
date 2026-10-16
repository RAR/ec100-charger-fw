# Real NASN RM4850-B register dump (2026-10-12)

Taken with the Waveshare ESP32-S3 running `esphome/eco-charger-dump.yaml`: read-only cmd 0x82 scans of 0x0100–0x02FF on
both addresses, with the stock board removed. AC was 248 V / 59.8 Hz and output current was 0 A. Raw log:
`captures/real-psu-dump-20261012.log` (local).

**Bus:** with the stock board removed, the ESP must terminate the bus (fit the Waveshare 120 Ω jumper). Without it the
TWAI saw nothing and went bus-off; recovering needs an ESP reboot.

Modules chatter unsolicited on `0x1101007E` / `0x1102007E` (`89 xx 00 00 02 14 ..`).

## Factory state, no controller

| Reg | Meaning | Value (both modules unless noted) |
|---|---|---|
| 0x0100 / 0x022F | VOUT setpoint / its readback | 0xD5F3 → 53.49 V (/1024) |
| 0x0101 | stored default VOUT | 53.49 V |
| 0x0102 | OVP | 60.5 V |
| 0x0103 / 0x022D | live current limit | 0x04E1 ≈ 63 A (A×1023/52) |
| 0x0104 | stored default current | 0x04E1 ≈ 63 A → **CAN-silence fallback is 53.5 V at full current** |
| 0x0175 / 0x0184 | output voltage | 53.51 / 53.41 V; both are real Vout |
| 0x0178 / 0x0171 | AC V / Hz | 247.5 / 249.6 V, 59.8 Hz |
| 0x0180 / 0x0235 | temperatures | 26–29 °C |
| 0x0177, 0x0206/7 | rated power | 3033 W |
| 0x0183 | alarms | `00 01 00 00 00 00`: data[2..3] = 1, alarm word (data[4..7]) = 0 |
| 0x0132 | standby | **not readable** (non-zero status), so the standby readback check never runs |
| 0x0222–0x0226 | serial (ASCII) | `1817Z530262253705976` / `…5999` |
| 0x0231 | internal bus? | ≈ 410 V |

## Stored default current changed to 5 A

Written by the dump firmware's `write_default_current` action:
- frame `0x108[1|2]81FE  01 04 00 00 00 00 00 62`, raw 98 = 5 A;
- acked with status 0 by both modules;
- 0x0104 then read back as 0x0062.

The live limit (0x0103 and 0x022D) followed it to 5 A right away.

**Not yet verified:** that the value survives a mains power cycle, and how the modules behave on CAN silence.

## Under the ESPHome controller (paused / HOLD)

- Both modules online. They accept I = 0 (0x022D readback 0) and standby, and the VOUT setpoint is written at min(CVL, max) = 57.0 V.
- In standby the output reads about 9.9 V, so the battery is not on the output terminals here (or there is a blocking stage).
- 0x0183 in standby is `00 01 00 00 02 00`: alarm word bit 9 (0x200) is set. That bit is not in the stock fault mask
  0x0852501F, so it reads as a status (output off) bit and causes no FAULT. The setpoint-readback check stayed quiet too.

## First real charge (controller unpaused, Max charge current 100 A)

- Ramped to CHARGING, with a target of 57.0 V / 100 A. Each module was set to 50.02 A and delivered 49.96 / 50.00 A, which
  confirms the 0x022D encoding (A×1023/52) on real hardware.
- With output on, the alarm word went to 0 (bit 9 cleared), which confirms bit 9 = standby/output-off status.
- Vout was 53.9 V into the pack at about 100 A total, with no faults.

## Wide scan (0x0000–0x0FFF, both modules) and the charging diff

Raw logs (local): `captures/wide-scan-20261012.log` (standby) and `captures/charging-scan-20261012.log` (25 A per module).

**Commands**
- 0x40 (bulk data) and 0x50 (info) work.
- 0x51 and 0xD2 (e-label) return status 2. Probed again with the first two payload bytes set to 0–3, 0x80 and 0xFF:
  every reply is status 2 echoing the request, so the e-label isn't implemented.
- Address 0 is a broadcast: both modules answer it.

**Registers outside 0x0100–0x02FF**
- 0x0001 and 0x0005–0x0007 are the info registers, the same data as cmd 0x50. Decoded per the Huawei R48xx notes:
  - 0x0001 `00 00 00 68 08 26`: module characteristics. (0x0068 & 0x3FF) / 2 = **52 A nominal**, which matches the A×1023/52 current encoding.
  - 0x0005 `00 08 00 04 00 04`: **HW version 8, DC-DC SW version 4, PFC SW version 4**. Same on both modules.
  - 0x0006 `01 01` / `01 02`: slot ID (top pin, bottom pin), so the bottom slot pin differs per module.
  - 0x0007 `20 23 08 18`: probably the date 2023-08-18. Huawei doesn't define this one.
- 0x0301–0x0308 hold a test pattern (`11111111` … `88888888`).
- 0x0400–0x0425 look like raw ADC/calibration values. 0x041C/0x041D echo the unsolicited chatter frames.

**What moves under load**

| Register | Meaning | Value |
|---|---|---|
| 0x0182 | Iout (/1024) | |
| 0x0176 = 0x0202 | Active current limit in A (/1024) | 53.0 at the factory default |
| 0x0235 | Heatsink/outlet temperature | about 69 °C at 1.33 kW per module |
| 0x0180 = 0x0232 | Inlet temperature | |
| 0x0231 | PFC bus | about 423 V running, 328 V in standby |

**Not implemented on the NASN:** 0x0170, 0x0172 and 0x0173 (AC power, AC current, DC power) read 0 even at 25 A, and no
other register tracks the AC draw. The firmware therefore computes output power as V×I, drops the AC current and power
sensors, and reads the outlet temperature from 0x0235.

## Mains power cycle

- **On AC loss:** PSU2 reported alarm word 0x2082020C before dropping off the bus. That is stock fault bits 2|3 (E11) and
  17 (E2), plus bit 9 (standby), plus bits 23 and 29. The controller went to FAULT, with both modules offline.
- **After AC returned:** both modules came back online, the FAULT auto-cleared after the 30 s wait, and the controller
  ramped back to CHARGING.
- **0x0104 survived the power cycle:** it still reads 0x0062 (5 A) on both modules, so the stored default current is
  persistent. 0x0101 is still 53.49 V.
