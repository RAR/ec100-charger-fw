# Panel controls — replacement charger firmware

How the front panel works with the replacement firmware (version 3.10) on the 48 V charger's control board. This
covers the buttons, the display, the LEDs and the settings menu only. It is not the vendor's manual; for installation,
wiring and safety, keep the original manual.

> **Unofficial firmware.** This is not the charger maker's firmware and isn't supported by them. Installing it will
> likely void your warranty; use it at your own risk.

<!-- figure: lcd -->

## Buttons

<!-- figure: buttons -->

| Button | Short press | Hold |
|---|---|---|
| **Switch** (AC-DC Switch) | **Charge now**: starts a charging session even if the battery is above Start V / Start SOC | **2 s: pause / resume**. A pause survives a crash-restart but not a power-cycle |
| **Up / Down** | In the menu: change the value by one step | Repeats every 150 ms after 1 s |
| **Enter** | Open the menu; then go to the next item. After the last item the menu closes and saves | **1 s: close the menu and save** |

- When the backlight is off, the first press of any button only wakes it; it does nothing else.
- The menu also closes and saves after **30 s** without a press.
- **Up + Down held together for 5 s** (outside the menu) restarts the board. While you hold them, the SOC field counts
  down 5, 4, 3, 2, 1, 0; let go before 0 and nothing happens. At 0 both modules go to standby first, unsaved settings
  are saved, and about a second later the board restarts. It's for service use: the restart opens the bootloader's
  update window. A pause is kept; a charging session starts again only by the usual start rule.
- Settings changed in the menu take effect when the menu closes and are saved a few seconds later; they survive a
  power-cycle.

## Display

| Area | Shows |
|---|---|
| **Voltage** | Battery voltage (V). In the menu: the voltage being edited |
| **Current** | Charge current (A). In the menu: the current being edited |
| **Power** | Charge power (W) |
| **AC** | AC input voltage. In the menu, for items 7 and 8: the item number |
| **SOC** (centre) | Battery state of charge, %: from the BMS in a BMS mode (Pylontech, EG4, JK, JK-PB), from the bridge in Modbus mode, or from a connected WiFi bridge (Home Assistant / YamBMS) in Panel mode. `---` when unknown. During an error it alternates every second between the SOC and the error code (`E12`, …). Items 3, 4, 7 and 8 of the menu are edited here |
| **SOC bars** | The SOC level; a sweep while charging when the SOC is unknown |
| **ON / OFF** | ON while a charge is running, OFF when idle |
| **Error triangle** | Lit while there is an error or warning |
| **BCC, BCV, SOC START/STOP, BATT START/STOP** | In the menu: which setting is being edited (items 1-6) |

The **BMS** icon is not used by this firmware.

In the menu, the value being edited blinks.

## LEDs

<!-- figure: leds -->

| LED | State | Meaning |
|---|---|---|
| **Charge** (green) | On | Charging, with current flowing |
| | Fast blink | Charging but no current yet (starting up, soft-start) |
| | Slow blink | Idle |
| **Fault** (red) | On | A fault (E1-E14, E17): charging is stopped |
| | Slow blink | A warning (E15, E16): see the table below |
| | Fast blink, first 10 s after start | The board restarted itself after a crash (watchdog) |

## Settings menu

Press **Enter** to open the menu at item 1, **Enter** again for each next item, **Up / Down** to change the value.
After item 8 (or a 1 s hold of Enter, or 30 s idle) the menu closes and saves.

<!-- figure: menu-flow -->

| # | Setting | Shown in | Range | Default |
|---|---|---|---|---|
| 1 | **Max current**: the most the charger delivers | Current, BCC icon | 2-100 A, steps of 1 A | 10 A |
| 2 | **Charge V**: the voltage it charges to | Voltage, BCV icon | 47.0-57.8 V, steps of 0.1 V | 57.6 V |
| 3 | **Start SOC**: start a session when the BMS SOC falls to this | SOC, SOC START icon | 0 % up to Stop SOC − 1 | 20 % |
| 4 | **Stop SOC**: end the session when the BMS SOC reaches this | SOC, SOC STOP icon | Start SOC + 1 up to 100 % | 100 % |
| 5 | **Start V**: start a session when the battery falls to this | Voltage, BATT START icon | 42.0 V up to Stop V − 5.0 V | 42.0 V |
| 6 | **Stop V**: end the session at this voltage (once the current has tapered) | Voltage, BATT STOP icon | 47.0 V up to Charge V | 57.6 V |
| 7 | **Source mode**: where the charging limits come from | SOC field; AC shows `7` | 0 Panel, 1 Pylontech BMS, 2 Modbus (limits from Home Assistant / YamBMS through the WiFi bridge), 3 EG4 BMS, 4 JK BMS, 5 JK-PB BMS | 0 |
| 8 | **Modbus address** of the RS485 jack | SOC field; AC shows `8` | 1-247 | 1 |

Items 7 and 8 have no icon of their own: watch the **AC** field, which shows the item number. After you press Enter
on item 7, the SOC field shows item 8's value (the Modbus address, usually `1`), not Source mode.

### When charging starts and stops

- A **session** starts at power-up if the battery is below **Start V** (or, with a BMS, the SOC is at or below
  **Start SOC**), or when you press **Switch** (Charge now).
- It ends when the battery has reached **Stop V** and the current has tapered to 4 A or less for 10 s, or, with a
  BMS, when the SOC reaches **Stop SOC**. It then waits for the next start condition.
- A **pause** (Switch held 2 s) stops charging until you resume it, whatever the session says.
- The SOC shown from a WiFi bridge in Panel mode is for display only: it never starts or stops charging.

### Source mode

- **0 Panel** (default): the charger uses its own settings above. The RS485 jack answers Modbus (Home Assistant,
  tools, remote updates).
- **1 Pylontech BMS**: the charger polls a BMS on the RS485 jack (Pylontech protocol, 9600 baud) and charges within the
  BMS's charge voltage and current limits, stops when the BMS withdraws charge permission, and shows **E12** on a BMS
  alarm. Your Max current and Charge V still cap everything. The jack then belongs to the BMS: there is no Modbus,
  Home Assistant or remote update in this mode. Switch back to 0 from this menu.
- **2 Modbus**: the limits come from Home Assistant / YamBMS through the WiFi bridge, which writes them every 5 s over
  the RS485 jack. The jack stays Modbus, so Home Assistant and remote updates keep working. Your Max current and Charge V
  still cap everything, and the SOC start/stop thresholds apply. If the bridge goes quiet for 10 s, the charger shows
  **E15** and falls back as below. Over Modbus, Source mode can be switched between 0 and 2 (not to 1, 3, 4 or 5).
- **3 EG4 BMS**: the charger polls an EG4 LL / LifePower battery on the RS485 jack (Modbus RTU, 9600 baud, battery address
  0x10: a single battery, or the master battery with its DIP switches all on). It charges to your Charge V, within the
  BMS's charge current limit, stops on a BMS protection (over-voltage, over-current, over-temperature, charge
  under-temperature) and shows **E12**. A flat battery stays chargeable. Like mode 1, the jack belongs to the BMS: no
  Modbus, Home Assistant or remote update. Menu only.
- **4 JK BMS**: the charger polls a JK-B BMS through the JK RS485 adapter (115200 baud). It charges to your Charge V and
  Max current, stops when the BMS turns charging off or reports a charge alarm (**E12**). The jack belongs to the BMS,
  as in mode 3. Menu only.
- **5 JK-PB BMS**: the charger polls a JK-PB inverter-series pack on its RS485-1 port, set to protocol 001 (JK RS485
  Modbus) at 115200 baud, and finds the pack's DIP address (0-15) itself, up to about 16 s. For a parallel bank connect to
  the DIP-0 master pack: the charger sees only that pack. It charges to the cell count times the pack's charge voltage per
  cell (the float voltage in the float stage) and the pack's max charge current, both still capped by your Charge V and
  Max current. It stops when the pack turns charging off (or its max charge current is 0) or reports a charge alarm (**E12**); cell over-voltage stops it
  without E12, which is normal at full; under-voltage never stops it. The jack belongs to the BMS, as in mode 3. Menu only.
- If the BMS (Pylontech, EG4, JK, JK-PB: 3 s) or the bridge (Modbus: 10 s) stops answering, the charger shows **E15** and falls back to
  its own settings, limited to the *Fallback current* (set over Modbus, default 10 A; 0 means "never charge without the
  BMS"). If the last word was "stop" or an alarm, it stays stopped until the BMS or bridge answers again, or Source mode is changed.
  In Modbus mode "stop" also covers a blocking flag and a current limit of 0.
- A pause is kept when you change the source mode. A charging session ends: the new source has to start a new one
  (Start SOC / Start V, or **Switch** for Charge now).

## Error codes

The SOC field alternates between the SOC and the code.

| Code | Meaning | What happens |
|---|---|---|
| E1 | AC input over-voltage (power module) | Stops charging |
| E2 | AC input under-voltage (power module) | Stops charging |
| E3 | Power module over-temperature | Stops charging |
| E4 | Power module fan fault | Stops charging |
| E5 | Battery over-voltage at a power module | Stops charging |
| E6 | Power module output abnormal | Stops charging |
| E8 | Power module output over-voltage latch | Stops charging |
| E9 | Ambient over-temperature at a power module | Stops charging |
| E10 | Uneven current between power modules | Stops charging |
| E11 | Other power-module fault | Stops charging |
| E12 | BMS alarm (Pylontech, EG4, JK, JK-PB or Modbus mode) | Stops while the alarm is reported |
| E13 | A power module isn't following its settings | Stops charging |
| E14 | Output over-voltage | Stops charging |
| E15 | Warning: the BMS / the bridge isn't answering (Pylontech, EG4, JK, JK-PB or Modbus mode) | Fallback limits (see Source mode) |
| E16 | Warning: charging but no current flows for 60 s (breaker off, loose cable) | Keeps trying |
| E17 | Battery reading implausible (below 20 V, sense line fault) | No charging until the reading is plausible |

A fault (E1-E14) clears by itself 30 s after its cause is gone; **Clear fault** over Modbus / Home Assistant
clears it at once. E17 clears as soon as the battery reading is plausible again; the session it ended then needs the
start rule (or Charge now) again.
