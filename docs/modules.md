# Rectifier modules

How the firmware starts the two NASN RM4850-B modules. Back to the [README](../README.md).

## Module start-up (soft-start)

The NASN modules have two start-up quirks, traced with a DC clamp (docs/history/notes/oem-bringup.md):

- **Wake burst.** A module that leaves standby with its voltage target above the battery pushes ~4 A for ~3 s,
  whatever its current limit (even 0 A). Woken below the battery it doesn't. Since 3.2 the firmware wakes each module
  1 V below the battery, holds it there for 12 s, then raises the target at 0.2 V/s with the current limit in force.
  A start therefore takes ~25 s to reach the set current.
- **Conduction spike (known limitation).** A module still spikes (~4.4 A on a clamp's MAX, well under a second) when it
  starts to conduct, at any ramp rate, and the second module starts conducting with the first even when its own target is
  held below the battery, so their spikes coincide. On the bench battery a 3 A start peaked at 7 A on the clamp (8.65 A
  before the soft-start). Waking PSU2 only once PSU1 conducts might shave this to ~5 A; not done, since it lasts well under a
  second once per start. Worth revisiting for a small battery or a BMS charge limit below ~8 A.
