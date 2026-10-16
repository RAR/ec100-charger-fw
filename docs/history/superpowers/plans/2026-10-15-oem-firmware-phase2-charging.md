# OEM Board Firmware, Phase 2 (Charging, Panel Source) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The OEM board charges from the two NASN modules on its own, using the panel settings. It speaks CAN to the modules, runs
the shared charge logic, and shows the stock main screen and an 8-item menu. Faults, cooldown and E-codes behave as on the ESP.

**Architecture:**
- Phase 1's bare-metal C++17 skeleton is extended with three register drivers: CAN (polled, no IRQ), ADC (polled) and FMC
  (internal flash). Each replays the stock v1 register sequence, recorded with evidence in `docs/history/notes/`.
- Everything else is hardware-free and header-only under `oem/src/app/`, host-tested by `make -C tests`.
  - `Rectifier` ports the ESP `nasn_rectifier` component. It talks through a `CanPort` interface, so the tests drive it against
    a fake bus with fake modules.
  - `Controller` ports the ESP `charge_controller` tick on the shared `charge_logic.h`.
  - Also: `SessionGate`, the relay logic, the settings model and its two-page flash store, and the UI (screen, menu, LEDs).
- `oem/src/main.cpp` is a new `charger` app: a cooperative super-loop with the spec §1 task periods. The `blink` and `bringup`
  apps keep building.

**Tech Stack:**
- Arm GNU Toolchain 15.3.Rel2 (`~/.local/arm-gnu-toolchain-15.3.rel2`, override with `ARM_TOOLCHAIN`), CMake ≥ 3.20, newlib-nano.
- Host tests: the system `c++` with `-std=c++17 -Werror`, via `tests/Makefile`.
- Python 3 in the repo `.venv` (pytest, python-can) for `tools/fake_nasn.py`.

**Spec:** `docs/history/superpowers/specs/2026-10-14-oem-board-firmware-design.md`: phase 2 of §7, plus §1–§3, §5.2, §6 and §8.
- Phase-1 plan (conventions, and bench Task 9, which confirms the relay pin): `docs/history/superpowers/plans/2026-10-14-oem-firmware-phase1-bringup.md`.
- Hardware evidence:
  - `docs/history/notes/oem-can-sequences.md` (CAN)
  - `docs/history/notes/oem-adc-flash-sequences.md` (ADC and FMC, added by Task 1)
  - `docs/history/notes/oem-init-sequences.md` §4 (LCD)
  - `docs/history/notes/oem-hw-map.md` §2, §3.1, §4 and §5
  - `docs/history/notes/findings-v2.md` §1, §2 and §5 (stock charge logic, E-code bit map)

**Dry run:** done while planning, against a fresh `git worktree` of `2374bcb`. Every code block of this plan was extracted and applied
in task order by a script, with these results:
- **Fail steps:** each one produced the error it names.
- **After every task:**
  - `make -C tests` passed: 4 test binaries at the start, 11 at the end;
  - `.venv/bin/python -m pytest tools/ -q` passed: 31, then 34 after Task 12, with 1 skip because the flash dump isn't in the
    worktree;
  - the cross build produced no warnings.
- **Final sizes:**

  | App | Flash | RAM |
  |---|---|---|
  | `blink` | 1372 B | 12 B |
  | `bringup` | 3652 B | 12 B |
  | `charger` | 21788 B (21 %) | 804 B |
- **Disassembly checks:** the Tasks 4–6 literal checks and the Task 11 checks (no `malloc`/`_sbrk`/`abort`/`__cxa_pure_virtual`;
  SP 0x20004000, reset 0x08005105) came out as stated.
- **Task 2 probe:** `basic_string` symbols before the change, none after.
- **Not run:**
  - the ESPHome build, because there's no ESP toolchain here; Task 2 changes one ESP line;
  - the bench tasks.

## Global Constraints

- **App placement:** linked at **0x08005000**, at most **0x19000 bytes (100 KB)** including the trailer. Never touch
  0x08000000–0x08004FFF (bootloader) or the **0x0801E000** page.
- **Flash writes:** only the settings pages **0x0801F800 and 0x0801FC00** (spec §5.2). The stock log at 0x0801F000 is left alone.
- **Watchdog:** FWDGT (about 2 s) is fed only at the end of a main-loop pass (`hw::fwdgt_feed()`), never from an interrupt.
- **Interrupts:** SysTick only. CAN RX is polled from FIFO0, because IRQ20 is shared with USBD.
- **Hard limits (spec §6), whatever a setting says:**
  - 58.0 V target (`charge::HARD_MAX_V`)
  - 50 A per module (`charge::MAX_A_PER_PSU`)
  - the 0x0183 alarm mask `0x0852501F`
  - the overvoltage fault
  - stored defaults ≤ 54.4 V and ≤ 10 A per module
- **Compile flags:** C++17, `-mcpu=cortex-m23 -mthumb -Os`, `-Wall -Wextra -Werror`, `-fno-exceptions -fno-rtti`.
- **No heap anywhere.** The linker script has no `end` symbol, so any `malloc` fails to link. No `std::string`, `std::vector`,
  `std::deque` or `std::optional::value()` on the target.
- **Shared code:**
  - `esphome/components/charge_controller/charge_logic.h` and `esphome/components/nasn_rectifier/nasn_protocol.h` are compiled
    into the GD32 build unchanged, except for the one change in Task 2.
  - The ESP component code keeps compiling: same APIs, apart from the one call site Task 2 updates.
- **Host tests:** `make -C tests` and `.venv/bin/python -m pytest tools/ -q` pass after every task.
- **Cross build:** `cmake -S oem -B oem/build && cmake --build oem/build` succeeds with no warnings after every task.
- **Bench safety:** tasks marked **[BENCH]** need the user and the board. Task 13 uses fake modules only. Task 14 is the first
  with real modules, and starts with conservative settings.
- **Style:** comments match the repo: sparse, explaining why. Lines ≤ 120 columns.

## Decisions (made while planning; the spec is silent or they refine it)

| # | Decision | Why |
|---|---|---|
| D1 | Every `oem/src/app/` module is header-only (spec §1 lists `.h/.cpp`). | Keeps the tests' one-TU-per-test Makefile rule, as phase 1 did. Each is used once, so code size doesn't suffer. |
| D2 | `charge::StateMachine::last_fault()` returns `const char *` instead of `const std::string &`. | Its `std::string` assignment is the only heap use the target build pulls in from the shared headers (checked with a cross-compiled probe in Task 2). The spec names this change. `nasn_protocol.h` needs no change: `std::optional` and `std::array` compile without exceptions as long as `.value()` isn't used. |
| D3 | `Rectifier` keeps one read in flight per module, abandons it after **250 ms**, and marks the module offline after **5 s** without a good read reply. A full poll cycle (12 registers, ≥ 20 ms apart) starts every 1 s. | The spec asks for one read in flight and a 5 s reply timeout. A short per-request timeout means one lost reply (for example during a flash erase) doesn't take the module offline. |
| D4 | A 16-frame software TX queue sits in front of the 3 mailboxes and is pumped every pass. TX priority stays as stock (TFO = 0). | The driver never waits for a mailbox. Writes (cmd 0x81) have lower IDs than reads (0x82), so a module's setpoint writes still go out before the read that checks them. |
| D5 | The CAN filter is bank 0 in 32-bit mask mode: ID `0x84000004`, mask `0xFC000406`. That is ESP ID `0x10800000` / mask `0x1F800080` shifted left by 3, with IDE = 1 and RTR = 0 required. RFD0 is written, not OR'ed. | bxCAN's FDATA layout matches TMI: EFID in bits 31..3, FF in bit 2, FT in bit 1. Only extended data frames that are NASN replies reach FIFO0. |
| D6 | The ADC uses polled single conversions: 10 every 100 ms, stock's sort-and-trim mean (middle 8) and stock scale (0.016923 V per count), times (1 + HR 14 / 1000). | It's simpler than stock's free-running circular DMA. The result is the same number. |
| D7 | The FMC driver follows the stock sequence, with three differences. It really clears the error flags (stock's `fmc_flag_clear(0x307)` writes a no-op bit). It checks every result. It refuses any address outside the two settings pages. Interrupts are off per operation. | Safety: a bug can't erase the app or the bootloader. |
| D8 | A settings record is 48 bytes (magic `'O''S'`, version 1, count 17, seq, 17 × u16, CRC-16 in the last doubleword). Saving never erases the page holding the newest valid record. Loading clamps. | Torn writes lose only the record being written. |
| D9 | When the source gives no battery current (Panel), the budget uses `battery_a = our_a`. There is no moving average. | The ESP's `MovingAverage` uses a heap-backed `std::deque`. With no battery-current reading, nobody else's current needs averaging. Phase 3 adds a fixed-size average for Pylontech/Modbus currents. |
| D10 | ≥ 3 rejected writes is reported as **E13**, the same as a setpoint readback mismatch. | The spec's table has no separate code. Both mean the module isn't following us. |
| D11 | Relay Auto: closed while the session is active, not paused and not faulted. It opens 10 s after any of these stops holding. | Matches stock's 10 s hold after stop. Missing AC is never a fault: modules just go offline. |
| D12 | Menu values stop at their range ends; stock wraps. Edits apply when the menu closes. Items 7 and 8 show their number in the AC field, because the glass has no indicator for them. The edited field and its indicator blink 300/300 ms; other fields are blank. | Clamping can't jump from 100 A to 2 A by accident. |
| D13 | With no SOC (Panel): the SOC field shows `---`, and the bars sweep while charging and are off otherwise. When the backlight times out, only the backlight goes off. | Stock blanks the glass too. Keeping it readable is more useful, and the spec only asks for the backlight to turn off. |
| D14 | *Charge now* starts a session, and the stop condition still applies. On a full battery it therefore ends 10 s later. | Stock semantics: charge now ignores only the start thresholds. |
| D15 | PLL lock and the switch to it time out. On IRC16M the app doesn't start CAN at all, since the bit timing would be wrong. | A wrong-rate node would disturb the modules' bus. |
| D16 | `board::RELAY = OUT_PC1` is marked "confirm at phase-1 bench". It is the only line to change if Task 9 of phase 1 shows the relay is on PB6. | Not blocking. |
| D17 | Phase 2 always uses `panel_limits()`. *Source mode* (menu item 7) is stored but ignored until phase 3. | Spec phase split. |
| D18 | Holding Up + Down for 5 s stops feeding the watchdog. | The bench watchdog test (Task 14). It also gives the bootloader its 1.5 s window without a power-cycle. |
| D19 | *Pause* is not persisted; it is off after every boot. | It isn't a setting (spec §4: HR 110 is runtime). |
| D20 | The phase-2 drivers build as a static library, `oem_drivers`. | Each driver task cross-compiles and can be disassembly-checked before the `charger` app exists. |
| D21 | `tools/fake_nasn.py` changes: a running module reads `max(setpoint, battery)`; it answers 0x0235 (heatsink); `--alarm-from/--alarm-until` and `--heatsink/--heatsink-until` add timed fault and cooldown injection. | Without the first change, a module set to the 48 V cooldown voltage reads 48 V < 49 V and cooldown ends at once. The rest makes bench Task 13 possible. |

**Folded-in phase-1 deferred minors** (ledger "fix before phase 2"):
- `lcd_render` bounds checks and the font-B 'E' note (Task 1)
- `slcd_write` is non-blocking (Task 1)
- the `rmw()` / ISR note (Task 1)
- PLL timeouts (Task 1)

## File structure

| File | Responsibility | Task |
|---|---|---|
| `docs/history/notes/oem-adc-flash-sequences.md` | Stock ADC and FMC register sequences with evidence | 1 |
| `oem/src/hw/regs.h` | + RCU_APB2EN, FMC, CAN and ADC addresses; `rmw()` note | 1 |
| `oem/src/hw/clock.h/.cpp` | PLL timeouts, IRC16M fallback | 1 |
| `oem/src/hw/slcd.h/.cpp` | Non-blocking `slcd_write` | 1 |
| `oem/src/app/lcd_render.h` | Bounds checks, `put_error`, animated `put_soc_bars` | 1 |
| `esphome/components/charge_controller/charge_logic.h` (+ `charge_controller.cpp`, `tests/test_charge_logic.cpp`) | `last_fault()` without heap | 2 |
| `oem/src/app/crc16.h` | Modbus CRC-16 | 3 |
| `oem/src/app/settings.h` | HR 0–16 settings, ranges, defaults, clamps | 3 |
| `oem/src/app/flash_io.h` | `FlashIo` interface | 3 |
| `oem/src/app/settings_store.h` | Record format, two-page log | 3 |
| `oem/src/hw/flash.h/.cpp` | FMC driver (`hw::Flash : FlashIo`) | 4 |
| `oem/src/app/can_port.h` | `CanFrame`, `CanPort`, `CanTxQueue` | 5 |
| `oem/src/hw/can_bits.h` | bxCAN TMI/filter/BT encodings (constexpr, host-tested) | 5 |
| `oem/src/hw/can.h/.cpp` | CAN driver | 5 |
| `oem/src/app/battery_adc.h` | Trimmed mean and scale | 6 |
| `oem/src/hw/adc.h/.cpp` | ADC driver | 6 |
| `oem/src/app/rectifier.h` | One NASN module | 7 |
| `tests/fake_nasn_bus.h` | Fake CAN bus and modules for host tests | 7 |
| `oem/src/app/session_gate.h` | Start/stop rules | 8 |
| `oem/src/app/relay.h` | Generator contact | 8 |
| `oem/src/app/limit_source.h` | `Limits`, `panel_limits()` | 9 |
| `oem/src/app/controller.h` | The 1 s tick, E-codes, `Status` | 9 |
| `oem/src/app/ui.h` | Keys → actions, main screen, menu, LEDs, backlight | 10 |
| `oem/src/main.cpp`, `oem/src/board.h` | The `charger` app; `board::RELAY` | 11 |
| `oem/CMakeLists.txt` | `oem_drivers` library, `charger` app | 4, 5, 6, 11 |
| `tests/Makefile`, `.gitignore` | New tests | 3, 5–10 |
| `tools/fake_nasn.py`, `tools/test_fake_nasn.py` | Heatsink register, timed injection, output clamp | 12 |
| `README.md` | The charger app | 11 |
| `docs/history/notes/oem-bringup.md` | Bench results | 13, 14 |

---

### Task 1: Phase-1 carry-overs, the phase-2 register map, and the ADC/FMC evidence

**Files:**
- Create: `docs/history/notes/oem-adc-flash-sequences.md`
- Modify (whole files given): `oem/src/hw/regs.h`, `oem/src/hw/clock.h`, `oem/src/hw/clock.cpp`, `oem/src/hw/slcd.h`,
  `oem/src/hw/slcd.cpp`, `oem/src/app/lcd_render.h`, `tests/test_oem_lcd_render.cpp`

**Interfaces:**
- Consumes: phase 1 (`hw::reg`, `hw::rmw`, `oem::Frame`, `oem::Field`, `put_*`).
- Produces:
  - Register constants: `hw::RCU_APB2EN`, `FMC_KEY/STAT/CTL/ADDR`, `CAN_*` (incl. `can_tmi(n)`, `can_tmp(n)`, `can_tmdata0/1(n)`)
    and `ADC_*`.
  - `hw::slcd_write(const uint32_t com[8]) -> bool`, now non-blocking: false means the frame was dropped.
  - `hw::clock_init() -> bool`: false on an HXTAL **or PLL** failure.
  - `oem::put_error(Frame &, uint8_t code)` and `oem::put_soc_bars(Frame &, int soc, int step = 8)`.
  - `put_glyph` ignores an out-of-range position or glyph.

- [ ] **Step 1: Write the failing tests.** Replace `tests/test_oem_lcd_render.cpp` with the version below. It adds
  `test_glyph_bounds` and `test_error_code`.

`tests/test_oem_lcd_render.cpp` (whole file):

```cpp
#include <initializer_list>

#include "app/lcd_render.h"
#include "check.h"

using namespace oem;

static void test_matches_stock_zero_frame() {
  Frame f;
  put_fixed_labels(f);
  put_dashes(f, Field::AC_V);
  put_number(f, Field::CURRENT, 0);
  put_number(f, Field::POWER, 0);
  put_number(f, Field::SOC, 0);
  put_number(f, Field::BATTERY_V, 0, 2);
  put_segment(f, 28, 2);
  put_segment(f, 28, 6);
  const uint32_t want[8] = {0xDCC27B18, 0x50801088, 0x50801088, 0x40801088,
                            0x0C020000, 0x40801088, 0x50801088, 0x40801088};
  for (int k = 0; k < 8; k++) CHECK(f.com[k] == want[k]);
}

static void test_number_places_digits_right_aligned() {
  Frame f;
  put_number(f, Field::POWER, 1234);  // SEG 11, 13, 14, 23 = 1, 2, 3, 4 in font A
  CHECK(f.com[2] & (1u << 11));       // '1' = 0x24: COM2 and COM5
  CHECK(f.com[5] & (1u << 11));
  CHECK(!(f.com[1] & (1u << 11)));
  CHECK(f.com[4] & (1u << 23));       // '4' = 0x3C: COM2..COM5
}

static void test_overflow_shows_dashes() {
  Frame f;
  put_number(f, Field::CURRENT, 1000);
  for (int seg : {22, 4, 3}) CHECK(f.com[4] & (1u << seg));
}

static void test_ac_uses_font_b() {
  Frame f;
  put_number(f, Field::AC_V, 1);  // font B '1' = 0x48: COM3 and COM6 on SEG17
  CHECK(f.com[3] & (1u << 17));
  CHECK(f.com[6] & (1u << 17));
  CHECK(!(f.com[2] & (1u << 17)));
}

static void test_soc_bars() {
  Frame f;
  put_soc_bars(f, 50);  // > 0, 13, 27, 39 → bars 0..3 = SEG16 COM7..COM4
  for (int k = 4; k <= 7; k++) CHECK(f.com[k] & (1u << 16));
  for (int k = 0; k <= 3; k++) CHECK(!(f.com[k] & (1u << 16)));
}

static void test_battery_v_leading_zero() {
  Frame f;
  put_number(f, Field::BATTERY_V, 5, 2);  // "0.5": SEG30 = '0' (0xEE), SEG12 = '5', SEG31 blank
  CHECK(f.com[1] & (1u << 30));
  CHECK(f.com[1] & (1u << 12));  // '5' = 0xBA: COM1, 3, 4, 5, 7
  CHECK(f.com[7] & (1u << 12));
  CHECK(!(f.com[2] & (1u << 12)));
  for (int k = 0; k < 8; k++) CHECK(!(f.com[k] & (1u << 31)));
}

static void test_power_limits() {
  Frame f;
  put_number(f, Field::POWER, 9999);  // '9' = 0xBE: COM1..5 and COM7, not COM6
  for (int seg : {11, 13, 14, 23}) {
    CHECK(f.com[1] & (1u << seg));
    CHECK(f.com[7] & (1u << seg));
    CHECK(!(f.com[6] & (1u << seg)));
  }
  Frame g;
  put_number(g, Field::POWER, 10000);  // dashes: '-' = 0x10, COM4 only
  for (int seg : {11, 13, 14, 23}) {
    CHECK(g.com[4] & (1u << seg));
    CHECK(!(g.com[1] & (1u << seg)));
  }
}

static void test_soc_bar_thresholds() {
  auto bars = [](int soc) {
    Frame f;
    put_soc_bars(f, soc);
    int n = 0;
    for (int k = 0; k < 8; k++) n += (f.com[k] >> 16) & 1u;
    return n;
  };
  CHECK(bars(0) == 0);
  Frame f13;
  put_soc_bars(f13, 13);
  CHECK((f13.com[7] & (1u << 16)) && bars(13) == 1);
  CHECK(bars(14) == 2);
  CHECK(bars(100) == 8);
}

static void test_glyph_bounds() {
  Frame f;
  put_glyph(f, Field::SOC, 3, 8);         // SOC has 3 digits
  put_glyph(f, Field::CURRENT, -1, 8);
  put_glyph(f, Field::CURRENT, 0, 13);    // no glyph 13
  for (int k = 0; k < 8; k++) CHECK(f.com[k] == 0);
  put_glyph(f, Field::POWER, 3, 1);       // the power field's 4th digit is real
  CHECK(f.com[2] & (1u << 23));
}

static void test_error_code() {
  Frame f;
  put_fixed_labels(f);
  put_error(f, 13);  // SEG9 'E' (0xDA: COM1, 3, 4, 6, 7), SEG8 '1', SEG7 '3'
  for (int k : {1, 3, 4, 6, 7}) CHECK(f.com[k] & (1u << 9));
  CHECK(!(f.com[2] & (1u << 9)) && !(f.com[5] & (1u << 9)));
  CHECK((f.com[2] & (1u << 8)) && (f.com[5] & (1u << 8)) && !(f.com[1] & (1u << 8)));  // '1' = 0x24
  CHECK((f.com[1] & (1u << 7)) && (f.com[7] & (1u << 7)));                           // '3' = 0xB6
  CHECK(!(f.com[0] & (1u << 8)) && !(f.com[0] & (1u << 9)));
  CHECK(f.com[0] & (1u << 28));  // other labels stay
}

int main() {
  test_glyph_bounds();
  test_error_code();
  test_matches_stock_zero_frame();
  test_number_places_digits_right_aligned();
  test_overflow_shows_dashes();
  test_ac_uses_font_b();
  test_soc_bars();
  test_battery_v_leading_zero();
  test_power_limits();
  test_soc_bar_thresholds();
  return test_result("test_oem_lcd_render");
}
```

- [ ] **Step 2: Run them and check they fail**

Run: `make -C tests`
Expected: compile error, `use of undeclared identifier 'put_error'`.

- [ ] **Step 3: LCD render: bounds, error code, animated bars.** The 'E' glyph exists only in font A (font B's entry 12 is
  blank), and only the SOC field shows it.

`oem/src/app/lcd_render.h` (whole file):

```cpp
// oem/src/app/lcd_render.h: the stock glass (docs/history/notes/oem-init-sequences.md §4). Bit s of com[k] is SEGs x COMk; each
// digit is one SEG column with glyph bit k on COMk, and bit 0 of every glyph is clear because COM0 of a digit column
// is a separate label or decimal point.
#pragma once
#include <cstdint>

namespace oem {

struct Frame {
  uint32_t com[8] = {};
};

enum class Field : uint8_t { AC_V, CURRENT, POWER, SOC, BATTERY_V };

constexpr uint8_t GLYPH_BLANK = 10;
constexpr uint8_t GLYPH_DASH = 11;
constexpr uint8_t GLYPH_E = 12;

// Font B has no 'E' (its entry 12 is blank): only the SOC field, which uses font A, shows error codes.
constexpr uint8_t FONT_A[13] = {0xEE, 0x24, 0xD6, 0xB6, 0x3C, 0xBA, 0xFA, 0x26, 0xFE, 0xBE, 0x00, 0x10, 0xDA};
constexpr uint8_t FONT_B[13] = {0xEE, 0x48, 0xD6, 0xDA, 0x78, 0xBA, 0xBE, 0xC8, 0xFE, 0xFA, 0x00, 0x10, 0x00};

struct FieldDef {
  uint8_t segs[4];  // most significant digit first
  uint8_t digits;
  bool font_b;
};

constexpr FieldDef FIELDS[5] = {
    {{26, 27, 17, 0}, 3, true},    // AC_V
    {{22, 4, 3, 0}, 3, false},     // CURRENT
    {{11, 13, 14, 23}, 4, false},  // POWER
    {{9, 8, 7, 0}, 3, false},      // SOC (or "E" + 2 digits)
    {{31, 30, 12, 0}, 3, false},   // BATTERY_V, 0.1 V; the decimal point is a fixed COM0 label
};

constexpr uint8_t LCD_SEGS[19] = {3, 4, 7, 8, 9, 11, 12, 13, 14, 16, 17, 22, 23, 25, 26, 27, 28, 30, 31};

inline void put_segment(Frame &f, int seg, int com) { f.com[com] |= 1u << seg; }

inline void put_fixed_labels(Frame &f) {
  f.com[0] |= 0xDCC27B18u;
  f.com[1] |= 0x10000000u;
}

// Out-of-range positions and glyphs draw nothing.
inline void put_glyph(Frame &f, Field field, int pos, uint8_t glyph) {
  const FieldDef &d = FIELDS[int(field)];
  if (pos < 0 || pos >= d.digits || glyph > GLYPH_E) return;
  const uint8_t bits = (d.font_b ? FONT_B : FONT_A)[glyph];
  for (int k = 1; k < 8; k++) {
    if (bits & (1u << k)) put_segment(f, d.segs[pos], k);
  }
}

inline void put_dashes(Frame &f, Field field) {
  for (int pos = 0; pos < FIELDS[int(field)].digits; pos++) put_glyph(f, field, pos, GLYPH_DASH);
}

// Right-aligned, leading zeros blanked down to min_digits; dashes if it doesn't fit.
inline void put_number(Frame &f, Field field, uint32_t value, int min_digits = 1) {
  const int n = FIELDS[int(field)].digits;
  uint32_t limit = 1;
  for (int i = 0; i < n; i++) limit *= 10;
  if (value >= limit) {
    put_dashes(f, field);
    return;
  }
  for (int pos = n - 1, i = 0; pos >= 0; pos--, i++) {
    if (i >= min_digits && value == 0) break;
    put_glyph(f, field, pos, uint8_t(value % 10));
    value /= 10;
  }
}

// "E" and two digits in the SOC field, with its COM0 labels (SEG8/SEG9) off, as stock does for error codes.
inline void put_error(Frame &f, uint8_t code) {
  put_glyph(f, Field::SOC, 0, GLYPH_E);
  put_glyph(f, Field::SOC, 1, uint8_t(code / 10 % 10));
  put_glyph(f, Field::SOC, 2, uint8_t(code % 10));
  f.com[0] &= ~((1u << 8) | (1u << 9));
}

// Eight bars on SEG16, COM7 first, lit above the stock thresholds. While charging, stock animates them: only the
// first `step` bars are drawn, step counting up every 300 ms.
inline void put_soc_bars(Frame &f, int soc, int step = 8) {
  static constexpr uint8_t THRESHOLDS[8] = {0, 13, 27, 39, 51, 63, 75, 87};
  for (int i = 0; i < 8; i++) {
    if (soc > THRESHOLDS[i] && step > i) put_segment(f, 16, 7 - i);
  }
}

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_lcd_render: ok`, and every other test `ok`.

- [ ] **Step 5: Record the ADC and FMC evidence.** This file was extracted from the v1 app disassembly while planning. Phase 2's ADC and
  flash drivers cite it.

`docs/history/notes/oem-adc-flash-sequences.md` (new file):

````markdown
# OEM v1 app: literal register-level ADC and FMC (internal flash) usage

Companion to `docs/history/notes/oem-init-sequences.md` and `docs/history/notes/oem-can-sequences.md`. Same conventions:
source is `dumps/gd32_flash_20261012.bin` (base 0x08000000), disassembled with
`arm-none-eabi-objdump -D -b binary -marm -Mforce-thumb --adjust-vma=0x08000000`; literal pools resolved from the binary.
Our drivers: `oem/src/hw/adc.cpp` follows §4 and `oem/src/hw/flash.cpp` follows §5 (phase-2 plan,
`docs/history/superpowers/plans/2026-10-15-oem-firmware-phase2-charging.md`).
Tags: **[V]** = verified in code / literals. **[I]** = inferred (datasheet, the GigaDevice GD32L23x firmware library V1.0.0, which isn't in the repo, or reasoning).

The ADC and DMA code is the GD32L23x standard library compiled -O0; every function body matches `gd32l23x_adc.c` / `gd32l23x_dma.c` /
`gd32l23x_rcu.c` V1.0.0 line for line. **[V]** The FMC code is a **newer** GD lib than the V1.0.0 copy we have: it has a doubleword
program function and register-encoded flag IDs (`(reg_offset << 6) | bit`) that V1.0.0 lacks. Every address below is from a literal in the binary. **[V]**

## 0. Bottom line

- **ADC:** PC0 = ADC_IN10. ADC clock = **CK_APB2 / 12 = 5 MHz** (`rcu_adc_clock_config(5)`). 12-bit (DRES never written, so it stays at reset 00), right-aligned.
  10-rank **scan + continuous**, all ranks = channel 10, sample time **41.5 cycles**, software trigger once at the end of init, and then it free-runs
  forever into a **circular DMA CH2** buffer of 10 × u16 at **0x20000010**. **[V]**
- **No code ever reads ADC_RDATA, STAT or EOC.** The only consumer is the main loop (0800b630 → 08007630), which takes a snapshot of the buffer, sorts it, drops min and max, and averages the middle 8 (`sum >> 3`).
  It then computes `(u16)(avg × 0.16923f)` (truncated) → **ST+0x16 = 0x200005EE** (in v1; v2 uses +0x1C). **[V]**
- **DMA is CH2, not CH0** (DMAMUX request 5 = ADC on L23x). CH2CTL final = **0x000035A1**. **[V]**
- **FMC:** unlock with KEY 0x45670123 / 0xCDEF89AB, **1 KiB pages**, page erase = PER, ADDR, START, wait, clear PER. Doubleword program =
  **PG only (no separate doubleword-enable bit)**, then two 32-bit stores to addr and addr+4, wait, clear PG. Ready-wait timeout = **0x00100000 polls**. Lock = CTL |= LK (0x80). **[V]**
- **Stock's flag clear is broken:** the app (and the bootloader's erase path) call `fmc_flag_clear(0x307)`. With the encoded flag IDs that is
  `FMC_STAT = 1 << 7` = **0x00000080**, which clears **none** of PGERR/PGAERR/WPERR/ENDF. 0x307 is evidently `END|WPERR|PGAERR|PGERR`
  (0x305|0x304|0x303|0x302) OR'ed together F10x-style. The bootloader's *program* path does it right: four separate clears, 0x305, 0x304, 0x303, 0x302. **[V]**
  Bit 7 is undefined in the V1.0 header, so writing it should do nothing. **[I]**
- IRQs: `cpsid i` / `cpsie i` around erase-range (08005ebc) and append (08006018); **not** around the page-erase used by 0800c900. **ISB** after each
  doubleword program and after the anti-clone erase. **No DSB** anywhere in the flash paths. **[V]**

v1 function map **[V]**:

| v1 addr | GD lib equivalent / role | notes |
|---|---|---|
| 080080f8 | (app) ADC DMA setup (buf, n) | called (0x20000010, 10) from init 0800b50c @0800b580 |
| 0800816c | (app) ADC bring-up | called right after, @0800b584 |
| 0800f5ac | rcu_periph_clock_enable(x) | reg = 0x40021000 + (x >> 6), bit = x & 31 |
| 0800ea90 | rcu_adc_clock_config(ck) | |
| 0800e8ee | gpio_mode_set(port, mode, pull, pin) | |
| 0800d5a4 | adc_channel_length_config(group, len) | |
| 0800d830 | adc_regular_channel_config(rank, ch, spt) | |
| 0800d724 | adc_external_trigger_source_config(group, src) | |
| 0800d648 | adc_data_alignment_config(align) | |
| 0800d9bc | adc_special_function_config(fn, en) | |
| 0800d6a8 | adc_external_trigger_config(group, en) | |
| 0800d674 | adc_dma_mode_enable() | |
| 0800d62c | adc_charge_pulse_width_counter(n) → ADC_CCTL | only from adc_enable |
| 0800d684 | adc_enable() | |
| 0800839c | (app) delay_ms(n) | n → 0x20000C40, spins until the SysTick ISR counts it to 0 |
| 0800d56c | adc_calibration_enable() | |
| 0800d97c | adc_software_trigger_enable(group) | |
| 0800d778 / 0800d78c | adc_interrupt_flag_clear / _get | only in the IRQ18 handler 08005AF4, which is never enabled |
| 0800e21c, 0800e3d8, 0800e278, 0800e200, 0800e3b4, 0800e450, 0800e1e4, 0800e1c8 | dma_deinit, dma_struct_para_init, dma_init, dma_circulation_enable, dma_memory_to_memory_disable, dmamux_synchronization_disable, dma_channel_enable, dma_channel_disable | |
| 08007630 | (app) ADC → ST+0x16 | from main loop 0800b630 every pass |
| 08007668 | (app) sort-and-trim mean of 10 u16 | |
| 0800e670 | fmc_unlock | |
| 0800e4c8 | fmc_flag_clear(encoded) | |
| 0800e538 | fmc_page_erase(addr) | |
| 0800e470 | fmc_doubleword_program(addr, lo, hi) | args r0 = addr, **r2 = lo, r3 = hi** (u64 in r2:r3, AAPCS) |
| 0800e52c | fmc_lock | |
| 0800e594 | fmc_ready_wait(timeout) | |
| 0800e5f0 | fmc_state_get | |
| 08005e44 | (app) erase log region (ctx) | |
| 08005ebc | (app) erase range [a, b], IRQs off | |
| 08005f28 | (app) log scan / ctx init | |
| 08005fc6 | (app) log read | |
| 08006018 | (app) log append | |

---

## 1. ADC clock, GPIO, DMA (v1 080080f8 and the first part of 0800816c)

### 1a. RCU + GPIO [V]

| # | v1 addr | register (addr) | operation |
|---|---|---|---|
| 1 | 0800f5ac(0x500) @08008104 | RCU_AHBEN (0x40021014) | \|= bit0 **DMAEN** |
| 2 | 0800f5ac(0x609) @08008172 | RCU_APB2EN (0x40021018) | \|= bit9 **ADCEN** |
| 3 | 0800f5ac(0x513) @08008178 | RCU_AHBEN | \|= bit19 **PCEN** |
| 4 | 0800e8ee(GPIOC 0x48000800, mode 3, pull 0, pin 0x1) | GPIOC_CTL (0x48000800) / PUD (0x4800080C) | CTL bits1:0 = **11 (analog)**; PUD bits1:0 = 00 |
| 5 | 0800ea90(**5**) @08008192 | RCU_CFG0 (0x40021004) | &= ~0x0000C000 (ADCPSC[1:0]) |
| 6 | | RCU_CFG2 (0x40021030) | &= 0x3FFFFEFF (clears ADCPSC2 bits31:30 and ADCSEL bit8) |
| 7 | | RCU_CFG0 | \|= (5 & 3) << 14 = **0x00004000** |
| 8 | | RCU_CFG2 | \|= ((5 & 0xC) >> 2) << 30 = **0x40000000** |
| 9 | | RCU_CFG2 | \|= **0x00000100 (ADCSEL = 1: APB2/AHB-divided source, not IRC16M)** |

- `ck_adc = 5` = `RCU_ADCCK_APB2_DIV12` in the lib header → **CK_ADC = 60 MHz / 12 = 5 MHz**. **[V]** for the code and the constant; that the hardware
  decodes {ADCPSC2, ADCPSC} = 0101 as APB2/12 is from the lib's enum, so **[I]**.
- DS limit: f_ADC 0.14 to **16 MHz** max; t_CONV(12-bit) = 15 cycles (2.5-cycle sample + 12.5), t_ST(ADC) startup 5 µs (DS Table 4-33). **[V vs ds/l235.txt]**
  (So the ~14 MHz F1-style limit does not apply; L235 is 16 MHz.) Stock runs well below that.
- Conversion = (41.5 + 12.5) / 5 MHz = **10.8 µs per sample**, so the whole 10-sample buffer is rewritten about every 108 µs. **[I]** (arithmetic)

### 1b. DMA CH2 (080080f8, buf = 0x20000010, n = 10) [V]

Struct passed to dma_init: periph_addr = **0x4001244C (ADC_RDATA)**, periph_width = 0x100, memory_addr = 0x20000010, memory_width = 0x400,
number = 10, priority = 0x3000, periph_inc = 0, memory_inc = 1, direction = 0 (periph→mem), request = **5 (DMA_REQUEST_ADC)**.

| # | lib call | register (addr) | operation |
|---|---|---|---|
| 1 | dma_deinit(2) | DMA_CH2CTL (0x40020030) | &= ~1 (CHEN), then = 0 |
| 2 | | DMA_CH2CNT (0x40020034), CH2PADDR (0x40020038), CH2MADDR (0x4002003C) | = 0 each |
| 3 | | DMA_INTC (0x40020004) | \|= 0xF << (4·2) = **0x00000F00** (clear all CH2 flags) |
| 4 | dma_init(2, &s) | DMA_CH2CTL | &= ~1 (dma_channel_disable) |
| 5 | | CH2PADDR | = **0x4001244C** |
| 6 | | CH2MADDR | = **0x20000010** |
| 7 | | CH2CNT | = **10** |
| 8 | | CH2CTL | = (CTL & ~0x3F00) \| 0x100 \| 0x400 \| 0x3000 (PWIDTH 16, MWIDTH 16, PRIO ultra-high) |
| 9 | | CH2CTL | &= ~0x40 (PNAGA = 0); \|= 0x80 (**MNAGA = 1**); &= ~0x10 (DIR = 0, periph→mem) |
| 10 | | DMAMUX_RM_CH2CFG (0x40020808) | &= ~0x7F; \|= **5** (MUXID = ADC) |
| 11 | dma_circulation_enable(2) | CH2CTL | \|= 0x20 (**CMEN**) |
| 12 | dma_memory_to_memory_disable(2) | CH2CTL | &= ~0x4000 |
| 13 | dmamux_synchronization_disable(2) | DMAMUX_RM_CH2CFG | &= ~0x10000 (SYNCEN = 0) |
| 14 | dma_channel_enable(2) | CH2CTL | \|= 0x1 (**CHEN**) |

**Final CH2CTL = 0x000035A1** (CHEN | CMEN | MNAGA | PWIDTH = 16 | MWIDTH = 16 | PRIO = 11). No DMA interrupts (FTFIE/HTFIE/ERRIE = 0). The
address formula used by the code is CHxCTL = 0x40020008 + 0x14·x, CNT = 0x4002000C + 0x14·x, PADDR = 0x40020010 + 0x14·x, MADDR = 0x40020014 + 0x14·x,
DMAMUX CHxCFG = 0x40020800 + 4·x. **[V]**

---

## 2. ADC init table (0800816c, after §1a) [V]

ADC base 0x40012400 (literals: STAT 0x40012400, CTL0 0x40012404, CTL1 0x40012408, SAMPT0 0x4001240C, SAMPT1 0x40012410, RSQ0 0x4001242C,
RSQ1 0x40012430, RSQ2 0x40012434, ISQ 0x40012438, RDATA 0x4001244C, CCTL 0x400124C0). **[V]**

| # | v1 call | register (addr) | operation |
|---|---|---|---|
| 1 | 0800d5a4(1 REGULAR, 10) | ADC_RSQ0 (0x4001242C) | &= ~0x00F00000; \|= (10−1) << 20 → **RL = 9 (10 conversions)** |
| 2–11 | 0800d830(rank r = 0..9, ch 10, spt 4) | ranks 0–5 → ADC_RSQ2 (0x40012434) field 5·r; ranks 6–9 → ADC_RSQ1 (0x40012430) field 5·(r−6) | each RMW: &= ~(0x1F << f); \|= 10 << f |
| | same calls | ADC_SAMPT0 (0x4001240C) (ch ≥ 10: field 3·(ch−10)) | &= ~0x7; \|= **4** (41.5 cycles), rewritten 10× with the same value |
| 12 | 0800d724(1, 0x000E0000) | ADC_CTL1 (0x40012408) | &= ~0x000E0000; \|= 0x000E0000 → **ETSRC = 111 (software trigger SWRCST)** |
| 13 | 0800d648(0) | ADC_CTL1 | &= ~0x800 (**DAL = 0, right-aligned**) |
| 14 | 0800d9bc(0x100 SCAN, 1) | ADC_CTL0 (0x40012404) | \|= 0x100 (**SM**) |
| 15 | 0800d9bc(0x002 CONTINUOUS, 1) | ADC_CTL1 | \|= 0x2 (**CTN**) |
| 16 | 0800d6a8(1 REGULAR, 1) | ADC_CTL1 | \|= 0x00100000 (**ETERC**) |
| 17 | 0800d674() | ADC_CTL1 | \|= 0x100 (**DMA**) |
| 18 | 0800d684() → 0800d62c(320) | ADC_CCTL (0x400124C0) | = 320 & 0xFFF = **0x140** (charge pulse width counter) |
| 19 | 0800d684() | ADC_CTL1 | if (CTL1 & 1) == 0: \|= 0x1 (**ADCON**) |
| 20 | 0800839c(1) | — | 1 ms SysTick delay (≥ t_ST = 5 µs) |
| 21 | 0800d56c() | ADC_CTL1 | \|= 0x8 (**RSTCLB**); **while (CTL1 & 0x8)**; no timeout |
| 22 | | ADC_CTL1 | \|= 0x4 (**CLB**); **while (CTL1 & 0x4)**; no timeout |
| 23 | 0800d97c(1 REGULAR) | ADC_CTL1 | \|= 0x00400000 (**SWRCST**) → conversions start and never stop (CTN) |

Final values **[V]** (assuming reset = 0 for untouched bits, **[I]**):
- **RSQ0 = 0x00900000**, **RSQ1 = 0x0005294A**, **RSQ2 = 0x14A5294A** (channel 10 in all 10 ranks).
- **SAMPT0 = 0x00000004** (ch10 = 41.5 cycles), SAMPT1 untouched (0).
- **CTL0 = 0x00000100** (SM only). DRES (bits 25:24) is never written, so the ADC is **12-bit [V that it isn't written; I that the reset value is 00 = 12-bit]**. No EOCIE, no watchdog, no discontinuous mode.
- **CTL1 = 0x001E0103** after init (ADCON | CTN | DMA | ETSRC = 7 | ETERC), plus SWRCST (bit22) as written; on GD parts hardware clears SWRCST when the conversion starts. **[I]**
- OVSAMPCTL, IOFFx, WDHT/WDLT, ISQ, TSVEN/INREFEN are never written. **[V]** (no literal for 0x40012480 anywhere)
- **Order note:** stock sets ADCON *after* all the configuration, then waits 1 ms, calibrates, and only then sets SWRCST. **[V]**
- IRQ18 (ADC) has a handler at 08005AF4 that uses adc_interrupt_flag_get/clear (STAT 0x40012400, CTL0 0x40012404), but it is never enabled in the NVIC and EOCIE is never set. **[V]**

---

## 3. ADC read / average path [V]

Main loop 0800b630 calls **08007630 every pass** (no rate limit):

```
u16 avg = f_08007668((u16*)0x20000010);        // the live circular-DMA buffer, no DMA pause, no copy lock
ST.u16[+0x16] = (u16)__aeabi_f2uiz(__aeabi_ui2f(avg) * 0.16923f);   // 0x200005D8 + 0x16 = 0x200005EE
```
- Scale literal @08007664 = **0x3E2D4AA1 = 0.16923000f**. 0800540a = ui2f, 080052d2 = fmul, 08005418 = f2uiz (it **truncates**). **[V]**
- Full scale: 4095 × 0.16923 = 693.0 → 0.1 V units → 69.3 V (agrees with oem-hw-map.md). **[V]** arithmetic.
- In v2 the same value goes to ST+0x1C (0800c964); in v1 it is **+0x16**. **[V]**

`08007668(buf)`:
1. copy buf[0..9] (u16) to a local array;
2. bubble sort ascending (outer i = 0..8, inner j < 9 − i, swap if a[j] > a[j+1]);
3. `sum` (u16) = a[1] + … + a[8] (drops the min and the max);
4. return **sum >> 3**.

Snapshot coherence: the DMA keeps writing while the copy loop runs, so the 10 samples can be from two adjacent sweeps. That doesn't matter for a DC level. **[I]**

---

## 4. Proposed polled single-conversion sequence (no DMA)

Constants: ADC = 0x40012400, RCU = 0x40021000. "[V]" = the same register/value stock writes. "[I]" = our change or from the manual.

**One-time init**

| # | register (addr) | operation | tag |
|---|---|---|---|
| 1 | RCU_APB2EN (0x40021018) | \|= 0x200 (ADCEN) | [V] |
| 2 | RCU_AHBEN (0x40021014) | \|= 0x80000 (PCEN) | [V] |
| 3 | GPIOC_CTL (0x48000800) | \|= 0x3 (PC0 analog); GPIOC_PUD (0x4800080C) &= ~0x3 | [V] |
| 4 | RCU_CFG0 (0x40021004) | = (CFG0 & ~0xC000) \| 0x4000 | [V] |
| 5 | RCU_CFG2 (0x40021030) | = (CFG2 & 0x3FFFFEFF) \| 0x40000100 → CK_ADC = APB2/12 = 5 MHz | [V] |
| 6 | ADC_RSQ0 (0x4001242C) | = 0x00000000 (RL = 0, one conversion) | [I] (stock: 0x00900000) |
| 7 | ADC_RSQ2 (0x40012434) | = 0x0000000A (rank 0 = ch10) | [V] field / [I] single rank |
| 8 | ADC_SAMPT0 (0x4001240C) | = (SAMPT0 & ~0x7) \| 0x4 (41.5 cycles) | [V] |
| 9 | ADC_CTL0 (0x40012404) | = 0 (no SM; 12-bit DRES = 0) | [I] (stock: 0x100) |
| 10 | ADC_CTL1 (0x40012408) | = 0x001E0000 (ETSRC = 7 SW, ETERC = 1, CTN = 0, DMA = 0, DAL = 0) | [V] ETSRC/ETERC; [I] CTN/DMA off |
| 11 | ADC_CCTL (0x400124C0) | = 0x140 | [V] |
| 12 | ADC_CTL1 | \|= 0x1 (ADCON) | [V] |
| 13 | — | wait ≥ 5 µs (stock: 1 ms) | [V] stock / DS t_ST |
| 14 | ADC_CTL1 | \|= 0x8; while (CTL1 & 0x8) {} (RSTCLB), with a timeout we add | [V] (+[I] timeout) |
| 15 | ADC_CTL1 | \|= 0x4; while (CTL1 & 0x4) {} (CLB), with a timeout; DS t_CAL ≤ 68.75 µs at 16 MHz | [V] (+[I] timeout) |

**Per sample**

| # | register | operation | tag |
|---|---|---|---|
| 1 | ADC_STAT (0x40012400) | &= ~0x2 (clear EOC; the lib's adc_flag_clear does STAT &= ~flag, as in 0800d778) | [V] method / [I] that it's needed |
| 2 | ADC_CTL1 (0x40012408) | \|= 0x00400000 (SWRCST) | [V] |
| 3 | ADC_STAT | while (!(STAT & 0x2)) {} with a timeout (expect about 10.8 µs) | [I] (stock never polls EOC) |
| 4 | ADC_RDATA (0x4001244C) | v = RDATA & 0xFFF (reading it clears EOC) | [V] address (DMA source) / [I] clear-on-read |

Then do 10 samples → sort → mean of the middle 8 (>> 3) → × 0.16923 → truncate, to match stock exactly. **[V]**
Notes: SWRCST needs ETERC = 1 with ETSRC = 111 on this ADC family (stock sets both) **[I/V]**. If CTN were left on, one SWRCST would free-run, and RDATA would be the latest value.
That also works for polling, but then EOC is only meaningful once per conversion. Single mode is simpler. **[I]**

---

## 5. FMC

FMC base 0x40022000. Literals used: WS 0x40022000, KEY **0x40022004**, STAT **0x4002200C**, CTL **0x40022010**, ADDR **0x40022014**. OBKEY/OBSTAT are never referenced. **[V]**
Bits (match the L23x header): STAT BUSY b0, PGERR b2, PGAERR b3, WPERR b4, ENDF b5; CTL PG b0, PER b1, START b6, LK b7. **[V]** from the code's use; names **[I]**.

### 5a. Unlock — fmc_unlock 0800e670 [V]

| # | register | operation |
|---|---|---|
| 1 | FMC_CTL (0x40022010) | read; **only if bit7 LK = 1**: |
| 2 | FMC_KEY (0x40022004) | = **0x45670123** |
| 3 | FMC_KEY | = **0xCDEF89AB** |

### 5b. Flag clear — fmc_flag_clear 0800e4c8(flag) [V]

`reg = 0x40022000 + (flag >> 6); bit = flag & 0x1F`. If `(flag >> 6) == 0x18`: reg = (reg & ~0x7F) | (1 << bit) (not used). Else **reg = 1 << bit** (plain write).
- App (08005e72, 08005ee8, 08006060, 0800b534) and boot erase (080005a0): flag = **0x307** (literal @080060ac / @0800b5f8 / @080005e0) → **FMC_STAT = 0x00000080**. None of the real flags (bits 0, 2–5) is cleared.
- Boot program path 08001c08: flag_clear(**0x305**) → STAT = 0x20 (ENDF); (**0x304**) → 0x10 (WPERR); (**0x303**) → 0x08 (PGAERR); (**0x302**) → 0x04 (PGERR). This is the correct form, and it proves the encoding.
- Write-1-to-clear for STAT flags is consistent with this; no RMW. **[I]** from L23x UM convention + code.

### 5c. Ready wait / state — 0800e594(timeout) + 0800e5f0 [V]

```
state = 1 (BUSY);
do { state = fmc_state_get(); timeout--; } while (state == BUSY && timeout != 0);
if (state == BUSY) state = 5 (TOERR);
return state;
```
fmc_state_get reads FMC_STAT (0x4002200C), checked in this priority order: bit0 BUSY → 1; else bit4 WPERR → **4**; else bit2 PGERR → **2**; else bit3 PGAERR → **3**;
else if STAT bits 15:14 ≠ 0 → **7** (meaning unknown; not in the V1.0 header) **[V]** code / **[I]** meaning; else 0 READY.
**Timeout argument is always 0x00100000** (1,048,576 polls; roughly 0.5–1 s at 60 MHz -O0 **[I]**). ENDF is never examined. The boot copy (08001c9c / 08001d78) is byte-identical. **[V]**
Because stock never clears error flags (5b), a leftover PGERR/WPERR/PGAERR would make every later ready-wait return an error and skip the operation. **[I]**

### 5d. Page erase — fmc_page_erase 0800e538(addr) [V]

| # | register | operation | wait |
|---|---|---|---|
| 1 | | | ready_wait(0x100000); if ≠ 0 → return it (nothing written) |
| 2 | FMC_CTL (0x40022010) | \|= 0x2 (**PER**) | |
| 3 | FMC_ADDR (0x40022014) | = addr (page base) | |
| 4 | FMC_CTL | \|= 0x40 (**START**) | |
| 5 | | | ready_wait(0x100000) |
| 6 | FMC_CTL | &= ~0x2 (clear PER), even after an error/timeout | return state |

Page size is **1 KiB**: callers step the address by 0x400 and compute page = (addr − 0x08000000) >> 10 (`addr + 0xF8000000 >> 10`). **[V]**

### 5e. Doubleword program — 0800e470(addr, u64 data in r2:r3) [V]

| # | register | operation | wait |
|---|---|---|---|
| 1 | | | ready_wait(0x100000); if ≠ 0 → return it |
| 2 | FMC_CTL | \|= 0x1 (**PG**) (no other enable bit) | |
| 3 | *(u32*)addr | = lo (r2) | |
| 4 | *(u32*)(addr + 4) | = hi (r3) | |
| 5 | | | ready_wait(0x100000) |
| 6 | FMC_CTL | &= ~0x1 (clear PG) | return state |

There is no DWPGE/FSTPG bit: CTL bit8 FSTPG (fast program) is never set. "Doubleword" is just PG with two consecutive aligned word stores. Whether the L235 needs both words (64-bit ECC granule) or would accept single words is not shown by stock. **[I]**
Boot's 08001b4c is the same code. **[V]**

### 5f. Lock — fmc_lock 0800e52c [V]

FMC_CTL (0x40022010) \|= **0x80 (LK)**.

### 5g. App flash routines [V]

Log context (struct @**0x20000084**): +0 cur (address of the last record), +4 start, +8 end, +12 n (u8; record = n doublewords). Init (0800b5cc):
`08005f28(0x20000084, start 0x0801F000, end 0x0801F3FF, n 2)`, i.e. a **single 1 KiB page at 0x0801F000 with 16-byte records**.

**08005f28 scan:** p = start; while (p + 8n ≤ end) { if the first word of each of the n doublewords at p, p+8, … is 0xFFFFFFFF → break; p += 8n }; cur = p − 8n.
(It only checks the low word of each doubleword.)

**08005e44 erase region(ctx):** pages (start−0x08000000)>>10 … (end−0x08000000)>>10 (here just page 0x7C): unlock; flag_clear(0x307); for each page fmc_page_erase(addr), addr += 0x400;
lock; cur = start − 8n. **No cpsid inside.** Callers: 08006018 (IRQs already off) and **0800c900 (IRQs on)**. Return codes are ignored.

**08005ebc erase range(a, b):** **cpsid i**; unlock; flag_clear(0x307); for page (a−0x08000000)>>10 … (b−0x08000000)>>10: fmc_page_erase(addr), addr += 0x400; lock; **cpsie i**.
Its only caller is the anti-clone path in init (0800b52e…0800b550): if the UID check 0800742c fails → unlock; flag_clear(0x307); **08005ebc(0x08005000, 0x08010F10) twice**
(both calls get the same r0/r1; Ghidra drops the args); **ISB**; lock. That erases the running app itself.

**08006018 append(ctx, src):**
```
cur += 8n;
cpsid i;
if (cur + 8n > end) { 08005e44(ctx); cur += 8n; }          // wrap: erase page, restart at start
fmc_unlock(); fmc_flag_clear(0x307);
for i in 0..n-1: { 0800e470(cur + 8i, src[2i], src[2i+1]); ISB; }   // results ignored, no read-back verify
fmc_lock();
cpsie i;
```
Its only caller is 08005df4 (it packs ST+3, +4, +0x24, +0x28, +0x12, +0x14, +1 into 16 bytes), triggered from the main loop 0800b630 when byte ST+0 (0x200005D8, literal @0800b684) ≠ 0, which it then clears.

---

## 6. Bootloader cross-check [V]

- **08000574 erase(a, b):** cpsid i; unlock (08001d78, same KEY values 0x45670123/0xCDEF89AB at 0x40022004); flag_clear(**0x307**) (08001ba4, same code);
  page loop with fmc_page_erase (08001c40: PER 0x2, ADDR 0x40022014, START 0x40, ready_wait(0x100000), clear PER); lock (08001c34, \|= 0x80); cpsie i. Same as app 08005ebc.
- **08000a34 program(addr, buf, len):** pads to doublewords (len/8, rounded up) via a staging buffer @0x20000140; cpsid i; unlock; **08001c08 = clear 0x305, 0x304, 0x303, 0x302**;
  loop 08001b4c(addr + 8i, lo, hi) + **ISB**; lock; cpsie i; zero the staging buffer (32 doublewords). Return value [sp+4] is always 0.
- fmc_ready_wait/state_get in boot (08001c9c/08001d78 region) are byte-identical to the app's. **[V]**

---

## 7. Literal list [V]

| literal @ | value | meaning |
|---|---|---|
| 08008168 | 0x4001244C | ADC_RDATA (DMA source) |
| 08008250 | 0x00000609 | RCU APB2EN bit9 ADCEN |
| 08008540 | 0x00000513 | RCU AHBEN bit19 PCEN |
| 08008544 | 0x48000800 | GPIOC |
| (imm) 08008100 | 0x500 | RCU AHBEN bit0 DMAEN |
| 0800eaf4 / 0800eaf8 / 0800eafc | 0x40021004 / 0x40021030 / 0x3FFFFEFF | RCU_CFG0, RCU_CFG2, ADCSEL+ADCPSC2 clear mask |
| 0800d628 | 0xFFF00000 | (len−1)<<20 helper |
| 0800d644 | 0x400124C0 | ADC_CCTL |
| 0800d640 | 0x00000FFF | CCNT mask |
| 0800d774, 0800d9b8, 0800da6c | 0x40012408 | ADC_CTL1 |
| 0800da68 | 0x40012404 | ADC_CTL0 |
| 0800d82c | 0x40012400 | ADC_STAT |
| 0800d974 / 0800d978 | 0x4001240C / 0x40012410 | SAMPT0 / SAMPT1 |
| 0800d968 / 0800d96c / 0800d970 | 0x4001242C / 0x40012430 / 0x40012434 | RSQ0 / RSQ1 / RSQ2 |
| 0800d624 | 0x40012438 | ISQ (not reached) |
| 0800e3d4 / 0800e44c / 0800e3ac / 0800e3b0 | 0x40020008 / 0x4002000C / 0x40020010 / 0x40020014 | DMA CH0 CTL/CNT/PADDR/MADDR (+0x14·ch) |
| 0800e274 | 0x40020004 | DMA_INTC |
| 0800e46c | 0x40020800 | DMAMUX RM_CH0CFG (+4·ch) |
| 0800b608 / 08007660 | 0x20000010 | ADC DMA buffer (10 × u16) |
| 08007664 | 0x3E2D4AA1 | 0.16923f |
| 080077c4 | 0x200005D8 | ST (result at +0x16) |
| 080083b8 | 0x20000C40 | delay counter |
| 0800e68c / 0800e690 / 0800e66c / 0800e590 / 0800e528 | 0x40022010 / 0x40022004 / 0x4002200C / 0x40022014 / 0x40022000 | FMC CTL / KEY / STAT / ADDR / base |
| 0800e694 / 0800e698 | 0x45670123 / 0xCDEF89AB | unlock keys |
| (imm) 0800e47c etc. | 0x00100000 | ready-wait timeout |
| 080060ac, 0800b5f8 (boot 080005e0) | 0x00000307 | bogus combined flag → STAT = 0x80 |
| boot 08001c28 / imm / 08001c2c / 08001c30 | 0x305 / 0x304 / 0x303 / 0x302 | proper ENDF / WPERR / PGAERR / PGERR clears |
| 0800b5fc / 0800b600 | 0x08005000 / 0x08010F10 | anti-clone erase range |
| 0800b618 / 0800b61c / 0800b620 | 0x20000084 / 0x0801F000 / 0x0801F3FF | log ctx, log page start/end |

## 8. Not verified / open

- The CK_ADC divider mapping (ck_adc = 5 → APB2/12) comes from the lib enum; the L235 user manual's RCU tables were not checked. **[I]**
- The meaning of STAT bits 15:14 → state 7, and what (flag>>6) == 0x18 (FMC+0x18) refers to in the newer lib. **[I]/unknown**
- Whether the L235 flash requires 64-bit (doubleword) programming, or would also accept 32-bit PG. Stock only ever writes doublewords. **[I]**
- That SWRCST self-clears and that reading RDATA clears EOC on L23x. This is standard GD behaviour, not exercised by stock. **[I]**
````

- [ ] **Step 6: Register map, with the `rmw()` note.** New addresses only. Every one has a literal in the stock app
  (`docs/history/notes/oem-can-sequences.md` §7, `docs/history/notes/oem-adc-flash-sequences.md` §7).

`oem/src/hw/regs.h` (whole file):

```cpp
// oem/src/hw/regs.h: register addresses used by the app. Values and sequences follow the stock v1 firmware;
// see docs/history/notes/oem-init-sequences.md for the evidence behind each one.
#pragma once
#include <cstdint>

namespace hw {

inline volatile uint32_t &reg(uint32_t addr) { return *reinterpret_cast<volatile uint32_t *>(addr); }
// Read-modify-write: not atomic. Only the main loop may use it on a register; the one ISR (SysTick) touches none.
inline void rmw(uint32_t addr, uint32_t mask, uint32_t value) { reg(addr) = (reg(addr) & ~mask) | (value & mask); }

constexpr uint32_t RCU_CTL = 0x40021000;
constexpr uint32_t RCU_CFG0 = 0x40021004;
constexpr uint32_t RCU_INT = 0x40021008;
constexpr uint32_t RCU_APB1RST = 0x40021010;
constexpr uint32_t RCU_AHBEN = 0x40021014;
constexpr uint32_t RCU_APB2EN = 0x40021018;
constexpr uint32_t RCU_APB1EN = 0x4002101C;
constexpr uint32_t RCU_BDCTL = 0x40021020;
constexpr uint32_t RCU_RSTSCK = 0x40021024;
constexpr uint32_t RCU_CFG1 = 0x4002102C;
constexpr uint32_t RCU_CFG2 = 0x40021030;
constexpr uint32_t DMA_CH3CTL = 0x40020044;  // DMA base 0x40020000 + 0x08 + 3 * 0x14
constexpr uint32_t FMC_WS = 0x40022000;
constexpr uint32_t FMC_KEY = 0x40022004;
constexpr uint32_t FMC_STAT = 0x4002200C;
constexpr uint32_t FMC_CTL = 0x40022010;
constexpr uint32_t FMC_ADDR = 0x40022014;

// docs/history/notes/oem-can-sequences.md §7
constexpr uint32_t CAN_CTL = 0x40006400;
constexpr uint32_t CAN_STAT = 0x40006404;
constexpr uint32_t CAN_TSTAT = 0x40006408;
constexpr uint32_t CAN_RFIFO0 = 0x4000640C;
constexpr uint32_t CAN_INTEN = 0x40006414;
constexpr uint32_t CAN_ERR = 0x40006418;
constexpr uint32_t CAN_BT = 0x4000641C;
constexpr uint32_t can_tmi(int n) { return 0x40006580u + 16u * uint32_t(n); }
constexpr uint32_t can_tmp(int n) { return 0x40006584u + 16u * uint32_t(n); }
constexpr uint32_t can_tmdata0(int n) { return 0x40006588u + 16u * uint32_t(n); }
constexpr uint32_t can_tmdata1(int n) { return 0x4000658Cu + 16u * uint32_t(n); }
constexpr uint32_t CAN_RFIFOMI0 = 0x400065B0;
constexpr uint32_t CAN_RFIFOMP0 = 0x400065B4;
constexpr uint32_t CAN_RFIFOMDATA00 = 0x400065B8;
constexpr uint32_t CAN_RFIFOMDATA10 = 0x400065BC;
constexpr uint32_t CAN_FCTL = 0x40006600;
constexpr uint32_t CAN_FMCFG = 0x40006604;
constexpr uint32_t CAN_FSCFG = 0x4000660C;
constexpr uint32_t CAN_FAFIFO = 0x40006614;
constexpr uint32_t CAN_FW = 0x4000661C;
constexpr uint32_t CAN_F0DATA0 = 0x40006640;
constexpr uint32_t CAN_F0DATA1 = 0x40006644;

// docs/history/notes/oem-adc-flash-sequences.md §2, §4
constexpr uint32_t ADC_STAT = 0x40012400;
constexpr uint32_t ADC_CTL0 = 0x40012404;
constexpr uint32_t ADC_CTL1 = 0x40012408;
constexpr uint32_t ADC_SAMPT0 = 0x4001240C;
constexpr uint32_t ADC_RSQ0 = 0x4001242C;
constexpr uint32_t ADC_RSQ2 = 0x40012434;
constexpr uint32_t ADC_RDATA = 0x4001244C;
constexpr uint32_t ADC_CCTL = 0x400124C0;
constexpr uint32_t PMU_CTL0 = 0x40007000;

constexpr uint32_t FWDGT_CTL = 0x40003000;
constexpr uint32_t FWDGT_PSC = 0x40003004;
constexpr uint32_t FWDGT_RLD = 0x40003008;
constexpr uint32_t FWDGT_STAT = 0x4000300C;

constexpr uint32_t SLCD_CTL = 0x40002400;
constexpr uint32_t SLCD_CFG = 0x40002404;
constexpr uint32_t SLCD_STAT = 0x40002408;
constexpr uint32_t slcd_data(int com) { return 0x40002414u + 8u * uint32_t(com); }

constexpr uint32_t GPIOA = 0x48000000;
constexpr uint32_t GPIOB = 0x48000400;
constexpr uint32_t GPIOC = 0x48000800;
constexpr uint32_t GPIOD = 0x48000C00;
constexpr uint32_t GPIO_CTL = 0x00;
constexpr uint32_t GPIO_OMODE = 0x04;
constexpr uint32_t GPIO_OSPD = 0x08;
constexpr uint32_t GPIO_PUD = 0x0C;
constexpr uint32_t GPIO_ISTAT = 0x10;
constexpr uint32_t GPIO_BOP = 0x18;
constexpr uint32_t GPIO_AFSEL0 = 0x20;
constexpr uint32_t GPIO_AFSEL1 = 0x24;
constexpr uint32_t GPIO_BC = 0x28;

constexpr uint32_t SYST_CSR = 0xE000E010;
constexpr uint32_t SYST_RVR = 0xE000E014;
constexpr uint32_t SYST_CVR = 0xE000E018;
constexpr uint32_t SCB_AIRCR = 0xE000ED0C;

}  // namespace hw
```

- [ ] **Step 7: Non-blocking `slcd_write`.** The 200 ms refresh never finds UPRF still set unless the controller is stuck. In
  that case dropping the frame beats spinning in the main loop.

`oem/src/hw/slcd.h` (whole file):

```cpp
// oem/src/hw/slcd.h
#pragma once
#include <cstdint>

namespace hw {

bool slcd_init();                         // false if the controller never synced or turned on
// Never waits: false, and the frame is dropped, while the previous update is still pending (the next call retries).
bool slcd_write(const uint32_t com[8]);

}  // namespace hw
```

`oem/src/hw/slcd.cpp` (whole file):

```cpp
// oem/src/hw/slcd.cpp: the stock v1 sequence (08008588 / 0800cafc): COM and SEG pins on AF3 with pull-downs, VSLCD
// on PD6, the SLCD clocked from IRC32K through the backup domain, 1/8 duty, 1/4 bias, external bias voltage.
#include "hw/slcd.h"
#include "hw/gpio.h"

namespace hw {
namespace {

constexpr Pin LCD_PINS[] = {
    // COM0..COM7
    {GPIOA, 8}, {GPIOA, 9}, {GPIOA, 10}, {GPIOB, 9}, {GPIOC, 10}, {GPIOC, 11}, {GPIOC, 12}, {GPIOD, 2},
    // SEG3, 4, 17 | 7, 8, 9, 16, 11, 12, 13, 14 | 22, 23, 25, 26, 27 | 28, 30, 31
    {GPIOA, 6}, {GPIOA, 7}, {GPIOA, 15}, {GPIOB, 3}, {GPIOB, 4}, {GPIOB, 5}, {GPIOB, 8}, {GPIOB, 11},
    {GPIOB, 12}, {GPIOB, 13}, {GPIOB, 14}, {GPIOC, 4}, {GPIOC, 5}, {GPIOC, 7}, {GPIOC, 8}, {GPIOC, 9},
    {GPIOD, 4}, {GPIOD, 8}, {GPIOD, 9},
};

bool wait_set(uint32_t addr, uint32_t bit) {
  for (uint32_t t = 0; t < 0x100000; t++) {
    if (reg(addr) & bit) return true;
  }
  return false;
}

}  // namespace

bool slcd_init() {
  reg(RCU_AHBEN) |= 0xFu << 17;
  for (const Pin &p : LCD_PINS) pin_config(p, Mode::AF, Pull::DOWN, 3);
  pin_config({GPIOD, 6}, Mode::ANALOG, Pull::NONE);  // VSLCD

  reg(RCU_APB1EN) |= 1u << 28;     // PMU
  reg(PMU_CTL0) |= 0x100u;         // backup-domain write enable
  reg(RCU_RSTSCK) |= 1u;           // IRC32K
  if (!wait_set(RCU_RSTSCK, 2u)) return false;
  rmw(RCU_BDCTL, 0x300u, 0x200u);  // RTC/SLCD clock = IRC32K
  reg(RCU_APB1EN) |= 1u << 10;     // SLCD
  for (volatile uint32_t i = 0; i < 1000; i = i + 1) {
  }
  reg(RCU_APB1RST) |= 1u << 10;
  reg(RCU_APB1RST) &= ~(1u << 10);

  reg(SLCD_CFG) = 0x0042E081u;  // PSC /2, DIV 16, dead time 1, blink /1024 on SEG0 (unwired), high drive
  reg(SLCD_CTL) = 0x00000012u;  // DUTY 1/8, BIAS 1/4, VSRC external
  if (!wait_set(SLCD_STAT, 1u << 5)) return false;  // SYNF
  reg(SLCD_CTL) |= 1u;                              // SLCDON
  return wait_set(SLCD_STAT, 1u);                   // ONF
}

bool slcd_write(const uint32_t com[8]) {
  if (reg(SLCD_STAT) & 4u) return false;  // UPRF: the last update hasn't been taken yet
  for (int k = 0; k < 8; k++) reg(slcd_data(k)) = com[k];
  reg(SLCD_STAT) |= 4u;  // request an update
  return true;
}

}  // namespace hw
```

- [ ] **Step 8: PLL timeouts.** The HXTAL wait already had one; the PLL lock and the SCS switch now do too, falling back to IRC16M.

`oem/src/hw/clock.h` (whole file):

```cpp
#pragma once
#include <cstdint>
#include "hw/regs.h"

namespace hw {

void takeover_from_bootloader();  // stop and reset what the bootloader leaves running (called by clock_init)
bool clock_init();        // 60 MHz from the 12 MHz HXTAL; false if HXTAL or the PLL failed (then 16 MHz IRC16M)
uint32_t core_hz();
void fwdgt_init();        // same ~2 s timeout as the bootloader; the FWDGT keeps running
inline void fwdgt_feed() { reg(FWDGT_CTL) = 0xAAAA; }
void systick_init();      // 1 ms tick
uint32_t millis();
uint32_t reset_flags();   // RCU_RSTSCK bits 23..31 as latched by clock_init()
[[noreturn]] void system_reset();

}  // namespace hw
```

`oem/src/hw/clock.cpp` (whole file):

```cpp
#include "hw/clock.h"

namespace hw {
namespace {

volatile uint32_t g_ms = 0;
uint32_t g_hz = 16000000;
uint32_t g_reset_flags = 0;

void spin(uint32_t n) {
  for (volatile uint32_t i = 0; i < n; i = i + 1) {
  }
}

// The PLL never locked or never became the system clock: stay on IRC16M with HXTAL and the PLL off.
bool pll_failed() {
  rmw(RCU_CFG0, 0x3u, 0x0u);
  while ((reg(RCU_CFG0) & 0xCu) != 0) {  // IRC16M is running: this switch completes
  }
  reg(RCU_CTL) &= ~((1u << 16) | (1u << 24));
  g_hz = 16000000;
  return false;
}

}  // namespace

extern "C" void SysTick_Handler() { g_ms = g_ms + 1; }

// The bootloader jumps to us with TIMER1, UART4 and its UART4 receive DMA still running (docs/history/notes/oem-init-sequences.md
// §1c). The DMA would keep writing into our RAM, and PB0 must be high-Z so it can't fight USART1 TX on the transceiver.
void takeover_from_bootloader() {
  reg(RCU_APB1RST) |= 1u;  // TIMER1
  reg(RCU_APB1RST) &= ~1u;
  reg(RCU_APB1RST) |= 1u << 20;  // UART4
  reg(RCU_APB1RST) &= ~(1u << 20);
  reg(DMA_CH3CTL) &= ~1u;  // CHEN: the bootloader's UART4 RX DMA (buffer 0x20000464)
  reg(RCU_CFG2) = 0;       // as stock SystemInit does
  reg(RCU_INT) = 0;
  reg(RCU_AHBEN) |= 1u << 18;  // GPIOB clock, needed to release PB0/PB1
  rmw(GPIOB + GPIO_CTL, 0xFu, 0x0u);  // PB0, PB1 inputs
  rmw(GPIOB + GPIO_PUD, 0xFu, 0x0u);  // no pull
  reg(0xE000E280) = 0xFFFFFFFFu;  // ICPR0/1 last: a peripheral still asserting would re-pend at once
  reg(0xE000E284) = 0xFFFFFFFFu;
}

bool clock_init() {
  g_reset_flags = reg(RCU_RSTSCK) & 0xFF800000u;
  reg(RCU_RSTSCK) |= 1u << 24;  // RSTFC: clear the reset flags for the next boot

  // Step off the bootloader's PLL onto IRC16M, as stock SystemInit (0800cdf4) does, stepping AHB down first.
  reg(RCU_CTL) |= 1u;
  while (!(reg(RCU_CTL) & 2u)) {
  }
  if ((reg(RCU_CFG0) & 0xCu) == 0x8u) {
    rmw(RCU_CFG0, 0xF0u, 0x80u);
    spin(80);
    rmw(RCU_CFG0, 0xF0u, 0x90u);
    spin(80);
  }
  rmw(RCU_CFG0, 0x3u, 0x0u);
  while ((reg(RCU_CFG0) & 0xCu) != 0) {
  }
  reg(RCU_CTL) &= ~((1u << 16) | (1u << 18) | (1u << 19) | (1u << 24));  // HXTAL, bypass, monitor, PLL off
  reg(RCU_CFG0) = 0;
  reg(RCU_CFG1) &= ~0xFu;
  takeover_from_bootloader();
  g_hz = 16000000;

  // Stock clock config (0800f7dc): HXTAL x5 = 60 MHz, APB1 /2, 2 flash wait states.
  reg(RCU_CTL) |= 1u << 16;
  for (uint32_t t = 0; !(reg(RCU_CTL) & (1u << 17)); t++) {
    if (t > 0x40000) return false;
  }
  rmw(FMC_WS, 0x7u, 0x2u);
  rmw(RCU_CFG0, 0x700u, 0x400u);
  rmw(RCU_CFG0, 0x80FF0000u, 0x000D0000u);  // PLLSEL = HXTAL, PLLMF = x5
  reg(RCU_CTL) |= 1u << 24;
  for (uint32_t t = 0; !(reg(RCU_CTL) & (1u << 25)); t++) {
    if (t > 0x40000) return pll_failed();
  }
  rmw(RCU_CFG0, 0x3u, 0x2u);
  for (uint32_t t = 0; (reg(RCU_CFG0) & 0xCu) != 0x8u; t++) {
    if (t > 0x40000) return pll_failed();
  }
  g_hz = 60000000;
  return true;
}

uint32_t core_hz() { return g_hz; }

void fwdgt_init() {
  reg(RCU_RSTSCK) |= 1u;  // IRC32K clocks the FWDGT
  while (!(reg(RCU_RSTSCK) & 2u)) {
  }
  reg(FWDGT_CTL) = 0x5555;
  while (reg(FWDGT_STAT) & 1u) {
  }
  reg(FWDGT_PSC) = 3;  // /32
  reg(FWDGT_CTL) = 0x5555;
  while (reg(FWDGT_STAT) & 2u) {
  }
  reg(FWDGT_RLD) = 0x800;  // ~2.05 s
  reg(FWDGT_CTL) = 0xAAAA;
  reg(FWDGT_CTL) = 0xCCCC;
}

void systick_init() {
  reg(SYST_RVR) = g_hz / 1000u - 1u;
  reg(SYST_CVR) = 0;
  reg(SYST_CSR) = 7;  // core clock, interrupt, enable
}

uint32_t millis() { return g_ms; }
uint32_t reset_flags() { return g_reset_flags; }

void system_reset() {
  __asm volatile("dsb" ::: "memory");
  reg(SCB_AIRCR) = 0x05FA0004u;  // SYSRESETREQ
  for (;;) {
  }
}

}  // namespace hw
```

- [ ] **Step 9: Cross-build**

Run: `cmake -S oem -B oem/build && cmake --build oem/build 2>&1 | grep -E -i "warning|error|FLASH:|RAM:"`
Expected: no `warning` or `error` lines. `blink` and `bringup` print their memory use (about 1.4 KB and 3.7 KB of flash).

- [ ] **Step 10: Commit**

```bash
git add docs/history/notes/oem-adc-flash-sequences.md oem/src/hw/regs.h oem/src/hw/clock.h oem/src/hw/clock.cpp oem/src/hw/slcd.h \
  oem/src/hw/slcd.cpp oem/src/app/lcd_render.h tests/test_oem_lcd_render.cpp
git commit -m "OEM phase 2 prep: ADC/FMC evidence, register map, non-blocking slcd_write, PLL timeouts, LCD bounds and error code"
```

---

### Task 2: Make the shared state machine heap-free

`charge::StateMachine` stores the last fault reason in a `std::string`. Assigning a fault reason longer than the small-string
buffer allocates, and the target build has no heap.

**Files:**
- Modify: `esphome/components/charge_controller/charge_logic.h` (StateMachine only)
- Modify: `esphome/components/charge_controller/charge_controller.cpp` (the one `last_fault()` call site)
- Modify: `tests/test_charge_logic.cpp` (two comparisons)

**Interfaces:**
- Produces: `const char *charge::StateMachine::last_fault() const`, which returns `""` before the first fault. The reason
  pointer passed in `Inputs::fault` is kept, so it must be a literal or a buffer that is only rewritten when the next fault is
  reported. The ESP's `fault_text_` member and the OEM controller's literals both qualify.

- [ ] **Step 1: Show the problem with a cross-compiled probe** (it lives under the ignored `oem/build/`):

```bash
mkdir -p oem/build/probe && cat > oem/build/probe/probe.cpp <<'EOF'
#include "charge_controller/charge_logic.h"
#include "nasn_rectifier/nasn_protocol.h"
volatile uint32_t sink;
volatile float fs;
int main() {
  charge::StateMachine sm;
  charge::Inputs in;
  in.fault = sink ? "a fault reason longer than fifteen chars" : nullptr;
  sink = uint32_t(sm.update(in, sink));
  uint8_t d[8] = {};
  auto rep = nasn::parse_reply(sink, true, d, 8);
  if (rep) sink = rep->reg + nasn::alarm_word(rep->value);
  sink = nasn::write_voltage(nasn::reg::VOUT_SET, fs)[7];
  charge::RampLimiter r;
  charge::Cooldown c;
  fs = r.step(fs, sink) + float(c.update(sink & 1, sink & 2, fs, sink));
  return 0;
}
EOF
T=${ARM_TOOLCHAIN:-$HOME/.local/arm-gnu-toolchain-15.3.rel2}/bin
$T/arm-none-eabi-g++ -std=c++17 -mcpu=cortex-m23 -mthumb -Os -Wall -Wextra -Werror -fno-exceptions -fno-rtti \
  -fno-threadsafe-statics -fno-use-cxa-atexit -Iesphome/components -c oem/build/probe/probe.cpp -o oem/build/probe/probe.o
$T/arm-none-eabi-nm -C -u oem/build/probe/probe.o
```

Expected: the undefined symbols include `std::__cxx11::basic_string<...>::_M_replace(...)` and `..._M_dispose()`. Linked into
the app, these would pull in `operator new` → `malloc`, which fails to link. Everything else is `__aeabi_*` soft-float, `lroundf`
or `memset`: `nasn_protocol.h` is already fine.

- [ ] **Step 2: Change `StateMachine`.** In `esphome/components/charge_controller/charge_logic.h`, replace:

```cpp
  const std::string &last_fault() const { return last_fault_; }

 private:
  State state_ = State::BOOT;
  std::string last_fault_;
```

with:

```cpp
  // The last fault's reason, "" before the first one. in.fault must outlive the next fault (a literal or a
  // buffer that is only rewritten when a new fault is reported), since only the pointer is kept: no heap.
  const char *last_fault() const { return last_fault_; }

 private:
  State state_ = State::BOOT;
  const char *last_fault_ = "";
```

(`last_fault_ = in.fault;` in `update()` stays as it is.)

- [ ] **Step 3: Run the host tests and check they fail**

Run: `make -C tests`
Expected: `test_charge_logic.cpp:295: error: result of comparison against a string literal is unspecified` (`-Wstring-compare`).

- [ ] **Step 4: Fix the test and the ESP call site.** In `tests/test_charge_logic.cpp`, replace every occurrence of:

```cpp
CHECK(sm.last_fault() == "PSU 1 fault");
```

with:

```cpp
CHECK(std::string(sm.last_fault()) == "PSU 1 fault");
```

In `esphome/components/charge_controller/charge_controller.cpp`, replace:

```cpp
  const std::string fault_text = this->sm_.last_fault().empty() ? "none" : this->sm_.last_fault();
```

with:

```cpp
  const char *last_fault = this->sm_.last_fault();
  const std::string fault_text = *last_fault != '\0' ? last_fault : "none";
```

(`fault_text_` is a member buffer that `check_faults_` rewrites only when it reports a fault, so the stored pointer stays valid.)

- [ ] **Step 5: Check both builds**

Run: `make -C tests` and the probe commands from Step 1 again.
Expected:
- every test is `ok`;
- `nm -u` now lists only `__aeabi_*` helpers, `lroundf` and `memset`, with no `basic_string`.

- [ ] **Step 6: Commit**

```bash
git add esphome/components/charge_controller/charge_logic.h esphome/components/charge_controller/charge_controller.cpp \
  tests/test_charge_logic.cpp
git commit -m "charge_logic: StateMachine keeps the last fault as const char * (no heap, for the GD32 build)"
```

---

### Task 3: Settings model and the two-page store

**Files:**
- Create: `oem/src/app/crc16.h`, `oem/src/app/settings.h`, `oem/src/app/flash_io.h`, `oem/src/app/settings_store.h`,
  `tests/test_oem_settings.cpp`
- Modify: `tests/Makefile`, `.gitignore`

**Interfaces:**
- Produces:
  - `oem::crc16_modbus(const uint8_t *, size_t, uint16_t crc = 0xFFFF) -> uint16_t`.
  - `enum oem::Hr { HR_MAX_CURRENT = 0, …, HR_MODULE_FALLBACK_A = 16, HR_COUNT = 17 }`, `oem::HR_RANGES[17]` (`{min, max, def}`)
    and `START_STOP_GAP_CV = 500`.
  - `struct oem::Settings { uint16_t hr[17]; int32_t get(Hr) const; void set(Hr, int32_t); ==, != }`. `get(HR_ADC_TRIM)` is
    signed.
  - `oem::default_settings()`, `oem::apply_clamps(Settings &)` and `oem::clamp_i32(v, lo, hi)`.
  - `class oem::FlashIo { virtual const uint8_t *read(uint32_t); virtual bool erase_page(uint32_t); virtual bool program(uint32_t, const uint8_t *, uint32_t); }`.
  - `class oem::SettingsStore(FlashIo &)`: `bool load(Settings &)` (false = no record), `bool save(const Settings &)`,
    `uint32_t seq()` and `int active_page()`.
  - `SETTINGS_PAGE[2] = {0x0801F800, 0x0801FC00}`, `RECORD_BYTES = 48`, `RECORD_SLOTS = 21`, `encode_record` and `decode_record`.

- [ ] **Step 1: Write the failing test.** The fake flash is RAM. Programming can only clear bits, and it can be told to stop after
  N doublewords (a torn write) or to fail erases.

`tests/test_oem_settings.cpp` (new file):

```cpp
#include <cstring>

#include "app/settings_store.h"
#include "check.h"

using namespace oem;

// 2 KB of RAM standing in for the two settings pages. fail_after_dw > 0 makes a program call stop after that many
// doublewords (a torn write); erase_fails makes erase_page fail.
class FakeFlash : public FlashIo {
 public:
  FakeFlash() { std::memset(mem, 0xFF, sizeof(mem)); }
  const uint8_t *read(uint32_t addr) override { return mem + (addr - SETTINGS_PAGE[0]); }
  bool erase_page(uint32_t addr) override {
    erases++;
    if (erase_fails) return false;
    std::memset(mem + (addr - SETTINGS_PAGE[0]), 0xFF, SETTINGS_PAGE_BYTES);
    return true;
  }
  bool program(uint32_t addr, const uint8_t *data, uint32_t len) override {
    for (uint32_t i = 0; i < len; i += 8) {
      if (fail_after_dw > 0 && dw_written >= fail_after_dw) return false;
      uint8_t *p = mem + (addr - SETTINGS_PAGE[0]) + i;
      for (int b = 0; b < 8; b++) p[b] &= data[i + b];  // flash can only clear bits
      dw_written++;
    }
    return true;
  }
  uint8_t mem[2 * SETTINGS_PAGE_BYTES];
  int erases = 0;
  int dw_written = 0;
  int fail_after_dw = 0;
  bool erase_fails = false;
};

static void test_crc_check_value() {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_modbus(s, 9) == 0x4B37);  // same check value as tools/oem_image.py
}

static void test_defaults_and_ranges() {
  const Settings s = default_settings();
  CHECK(s.get(HR_MAX_CURRENT) == 10);
  CHECK(s.get(HR_CHARGE_V) == 5760);
  CHECK(s.get(HR_START_SOC) == 20 && s.get(HR_STOP_SOC) == 100);
  CHECK(s.get(HR_START_V) == 4200 && s.get(HR_STOP_V) == 5760);
  CHECK(s.get(HR_FALLBACK_A) == 10 && s.get(HR_BACKLIGHT_S) == 120);
  CHECK(s.get(HR_MODULE_FALLBACK_V) == 5400 && s.get(HR_MODULE_FALLBACK_A) == 5);
  Settings c = s;
  apply_clamps(c);
  CHECK(c == s);  // the defaults already satisfy every clamp
}

static void test_clamps() {
  Settings s = default_settings();
  s.set(HR_CHARGE_V, 5000);  // Stop V follows Charge V down, Start V stays 5.0 V below Stop V
  s.set(HR_START_V, 5280);
  apply_clamps(s);
  CHECK(s.get(HR_STOP_V) == 5000);
  CHECK(s.get(HR_START_V) == 4500);
  s.set(HR_START_SOC, 80);
  s.set(HR_STOP_SOC, 80);
  apply_clamps(s);
  CHECK(s.get(HR_START_SOC) == 79);
  s.set(HR_MAX_CURRENT, 500);
  s.set(HR_ADC_TRIM, -80);
  s.set(HR_CELL_RESUME_MV, 3550);
  s.set(HR_MAX_CELL_MV, 3500);
  apply_clamps(s);
  CHECK(s.get(HR_MAX_CURRENT) == 100);
  CHECK(s.get(HR_ADC_TRIM) == -50);
  CHECK(s.get(HR_CELL_RESUME_MV) == 3499);
}

static void test_record_round_trip() {
  Settings s = default_settings();
  s.set(HR_ADC_TRIM, -7);
  uint8_t rec[RECORD_BYTES];
  encode_record(s, 0x01020304, rec);
  CHECK(rec[0] == 'O' && rec[1] == 'S' && rec[2] == 1 && rec[3] == 17);
  CHECK(rec[4] == 0x04 && rec[7] == 0x01);
  Settings d{};
  uint32_t seq = 0;
  CHECK(decode_record(rec, d, seq));
  CHECK(d == s && seq == 0x01020304 && d.get(HR_ADC_TRIM) == -7);
  rec[20] ^= 1;
  CHECK(!decode_record(rec, d, seq));
}

static void test_empty_flash_loads_nothing_then_saves() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s;
  CHECK(!st.load(s));
  Settings want = default_settings();
  want.set(HR_MAX_CURRENT, 42);
  CHECK(st.save(want));
  CHECK(f.erases == 1 && st.active_page() == 0 && st.seq() == 1);
  SettingsStore again(f);
  Settings got{};
  CHECK(again.load(got) && got == want && again.seq() == 1);
}

static void test_appends_then_rotates() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  for (uint32_t i = 0; i < RECORD_SLOTS; i++) {
    s.set(HR_MAX_CURRENT, int32_t(2 + i));
    CHECK(st.save(s));
  }
  CHECK(f.erases == 1 && st.active_page() == 0);  // 21 records fill page 0 after one erase
  s.set(HR_MAX_CURRENT, 77);
  CHECK(st.save(s));
  CHECK(f.erases == 2 && st.active_page() == 1);  // page 1 erased, record 22 in its first slot
  SettingsStore again(f);
  Settings got{};
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 77 && again.seq() == 22);
  for (uint32_t i = 0; i < RECORD_SLOTS; i++) CHECK(st.save(s));
  CHECK(st.active_page() == 0 && f.erases == 3);  // back to page 0, never erasing the page with the newest record
}

static void test_torn_write_keeps_previous() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  s.set(HR_MAX_CURRENT, 30);
  CHECK(st.save(s));
  f.fail_after_dw = f.dw_written + 3;  // half of the next record, then power is lost
  s.set(HR_MAX_CURRENT, 31);
  st.save(s);
  SettingsStore after(f);  // reboot
  Settings got{};
  CHECK(after.load(got) && got.get(HR_MAX_CURRENT) == 30);
  f.fail_after_dw = 0;
  s.set(HR_MAX_CURRENT, 32);
  CHECK(after.save(s));
  SettingsStore again(f);
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 32 && again.active_page() == 0);
  // the new record went after the torn one, not into it
  Settings probe{};
  uint32_t seq;
  CHECK(!decode_record(f.mem + RECORD_BYTES, probe, seq));
  CHECK(decode_record(f.mem + 2 * RECORD_BYTES, probe, seq));
}

static void test_failed_write_moves_to_other_page() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  CHECK(st.save(s));
  f.fail_after_dw = f.dw_written + 1;
  CHECK(!st.save(s));  // the active-page write fails, and so does the other page (the fake stays broken)
  f.fail_after_dw = 0;
  s.set(HR_MAX_CURRENT, 55);
  CHECK(st.save(s));
  Settings got{};
  SettingsStore again(f);
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 55);
  f.erase_fails = true;
  for (uint32_t i = 0; i < RECORD_SLOTS; i++) st.save(s);
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 55);  // nothing lost when the erase fails
}

static void test_load_clamps_out_of_range() {
  FakeFlash f;
  Settings s = default_settings();
  s.set(HR_MAX_CURRENT, 300);
  uint8_t rec[RECORD_BYTES];
  encode_record(s, 9, rec);
  f.program(SETTINGS_PAGE[1], rec, RECORD_BYTES);
  SettingsStore st(f);
  Settings got{};
  CHECK(st.load(got) && got.get(HR_MAX_CURRENT) == 100 && st.active_page() == 1 && st.seq() == 9);
}

int main() {
  test_crc_check_value();
  test_defaults_and_ranges();
  test_clamps();
  test_record_round_trip();
  test_empty_flash_loads_nothing_then_saves();
  test_appends_then_rotates();
  test_torn_write_keeps_previous();
  test_failed_write_moves_to_other_page();
  test_load_clamps_out_of_range();
  return test_result("test_oem_settings");
}
```

Then:
- Append the phase-2 test binaries to the repo-root `.gitignore`. In `.gitignore`, replace the line `tests/test_oem_lcd_render`
  with:

```
tests/test_oem_lcd_render
tests/test_oem_settings
tests/test_oem_can
tests/test_oem_adc
tests/test_oem_rectifier
tests/test_oem_session
tests/test_oem_controller
tests/test_oem_ui
```

- Replace `tests/Makefile`. `HEADERS` now also covers `tests/*.h`, for Task 7's fake bus.

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/settings_store.h' file not found`.

- [ ] **Step 3: Implement**

`oem/src/app/crc16.h` (new file):

```cpp
// oem/src/app/crc16.h: Modbus CRC-16 (poly 0xA001 reflected, init 0xFFFF), the same CRC as tools/oem_image.py and the
// stock bootloader. Used by the settings store now and by Modbus RTU in phase 3.
#pragma once
#include <cstddef>
#include <cstdint>

namespace oem {

inline uint16_t crc16_modbus(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1u) ? uint16_t((crc >> 1) ^ 0xA001u) : uint16_t(crc >> 1);
  }
  return crc;
}

}  // namespace oem
```

`oem/src/app/settings.h` (new file):

```cpp
// oem/src/app/settings.h: the persisted settings, one uint16 per Modbus holding register 0-16 (spec §4), with their
// ranges, defaults and the cross-field clamps of spec §2.3. Voltages are V x 100, cell voltages V x 1000.
#pragma once
#include <cstdint>

namespace oem {

enum Hr : uint8_t {
  HR_MAX_CURRENT = 0,     // A
  HR_CHARGE_V = 1,        // V x 100
  HR_START_SOC = 2,       // %
  HR_STOP_SOC = 3,        // %
  HR_START_V = 4,         // V x 100
  HR_STOP_V = 5,          // V x 100
  HR_SOURCE_MODE = 6,     // 0 Panel, 1 Pylontech, 2 Modbus (only Panel works in phase 2)
  HR_MODBUS_ADDR = 7,
  HR_MODBUS_BAUD = 8,     // 0 = 9600, 1 = 19200, 2 = 38400, 3 = 115200
  HR_FALLBACK_A = 9,      // total A while a non-panel source is stale
  HR_RELAY_MODE = 10,     // 0 Auto, 1 Off, 2 Modbus
  HR_BACKLIGHT_S = 11,    // 0 = always on
  HR_MAX_CELL_MV = 12,
  HR_CELL_RESUME_MV = 13,
  HR_ADC_TRIM = 14,       // per mille, signed (stored as int16 in the uint16)
  HR_MODULE_FALLBACK_V = 15,  // stored default 0x0101, V x 100
  HR_MODULE_FALLBACK_A = 16,  // stored default 0x0104, A per module
  HR_COUNT = 17,
};

struct HrRange {
  int32_t min, max, def;
};

constexpr HrRange HR_RANGES[HR_COUNT] = {
    {2, 100, 10},  {4700, 5780, 5760}, {0, 99, 20},    {1, 100, 100},  {4200, 5280, 4200}, {4700, 5780, 5760},
    {0, 2, 0},     {1, 247, 1},        {0, 3, 0},      {0, 20, 10},    {0, 2, 0},          {0, 3600, 120},
    {3400, 3580, 3550}, {3200, 3550, 3400}, {-50, 50, 0}, {4800, 5440, 5400}, {0, 10, 5},
};

constexpr int32_t START_STOP_GAP_CV = 500;  // Start V <= Stop V - 5.0 V

struct Settings {
  uint16_t hr[HR_COUNT];

  int32_t get(Hr r) const { return r == HR_ADC_TRIM ? int32_t(int16_t(hr[r])) : int32_t(hr[r]); }
  void set(Hr r, int32_t v) { hr[r] = uint16_t(v); }
  bool operator==(const Settings &o) const {
    for (int i = 0; i < HR_COUNT; i++) {
      if (hr[i] != o.hr[i]) return false;
    }
    return true;
  }
  bool operator!=(const Settings &o) const { return !(*this == o); }
};

inline Settings default_settings() {
  Settings s{};
  for (int i = 0; i < HR_COUNT; i++) s.set(Hr(i), HR_RANGES[i].def);
  return s;
}

inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Every value into its range, then the cross-field rules: Stop V <= Charge V, Start V <= Stop V - 5.0 V,
// Start SOC < Stop SOC, cell resume < max cell. Run after every change (menu, store load, phase-3 Modbus).
inline void apply_clamps(Settings &s) {
  for (int i = 0; i < HR_COUNT; i++) s.set(Hr(i), clamp_i32(s.get(Hr(i)), HR_RANGES[i].min, HR_RANGES[i].max));
  if (s.get(HR_STOP_V) > s.get(HR_CHARGE_V)) s.set(HR_STOP_V, s.get(HR_CHARGE_V));
  if (s.get(HR_START_V) > s.get(HR_STOP_V) - START_STOP_GAP_CV) s.set(HR_START_V, s.get(HR_STOP_V) - START_STOP_GAP_CV);
  if (s.get(HR_START_SOC) >= s.get(HR_STOP_SOC)) s.set(HR_START_SOC, s.get(HR_STOP_SOC) - 1);
  if (s.get(HR_CELL_RESUME_MV) >= s.get(HR_MAX_CELL_MV)) s.set(HR_CELL_RESUME_MV, s.get(HR_MAX_CELL_MV) - 1);
}

}  // namespace oem
```

`oem/src/app/flash_io.h` (new file):

```cpp
// oem/src/app/flash_io.h: what the settings store needs from internal flash. hw::Flash implements it on the chip;
// the host tests use a RAM-backed fake.
#pragma once
#include <cstdint>

namespace oem {

class FlashIo {
 public:
  virtual const uint8_t *read(uint32_t addr) = 0;  // flash is memory-mapped: a pointer to addr
  virtual bool erase_page(uint32_t addr) = 0;      // 1 KB page at addr
  // len a multiple of 8 and addr 8-aligned, into erased flash; one doubleword at a time, in order.
  virtual bool program(uint32_t addr, const uint8_t *data, uint32_t len) = 0;
};

}  // namespace oem
```

`oem/src/app/settings_store.h` (new file):

```cpp
// oem/src/app/settings_store.h: settings log in two 1 KB flash pages (spec §5.2). Records are appended to the
// active page; when it is full, the other page is erased and the next record goes there. Load picks the valid
// record with the highest sequence number, so a torn write (power lost mid-record) only loses that record.
//
// Record, 48 bytes = 6 doublewords, little-endian:
//   0 magic 'O','S' | 2 version | 3 register count | 4 seq u32 | 8 hr[17] u16 | 42 zero x4 | 46 CRC-16 of bytes 0..45
// The CRC sits in the last doubleword, which is programmed last.
#pragma once
#include <cstdint>

#include "app/crc16.h"
#include "app/flash_io.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t SETTINGS_PAGE[2] = {0x0801F800, 0x0801FC00};
constexpr uint32_t SETTINGS_PAGE_BYTES = 1024;
constexpr uint32_t RECORD_BYTES = 48;
constexpr uint32_t RECORD_SLOTS = SETTINGS_PAGE_BYTES / RECORD_BYTES;  // 21
constexpr uint8_t RECORD_VERSION = 1;

inline void encode_record(const Settings &s, uint32_t seq, uint8_t out[RECORD_BYTES]) {
  for (uint32_t i = 0; i < RECORD_BYTES; i++) out[i] = 0;
  out[0] = 'O';
  out[1] = 'S';
  out[2] = RECORD_VERSION;
  out[3] = HR_COUNT;
  for (int i = 0; i < 4; i++) out[4 + i] = uint8_t(seq >> (8 * i));
  for (int i = 0; i < HR_COUNT; i++) {
    out[8 + 2 * i] = uint8_t(s.hr[i]);
    out[9 + 2 * i] = uint8_t(s.hr[i] >> 8);
  }
  const uint16_t crc = crc16_modbus(out, RECORD_BYTES - 2);
  out[RECORD_BYTES - 2] = uint8_t(crc);
  out[RECORD_BYTES - 1] = uint8_t(crc >> 8);
}

inline bool decode_record(const uint8_t *in, Settings &s, uint32_t &seq) {
  if (in[0] != 'O' || in[1] != 'S' || in[2] != RECORD_VERSION || in[3] != HR_COUNT) return false;
  if (crc16_modbus(in, RECORD_BYTES - 2) != uint16_t(in[RECORD_BYTES - 2] | in[RECORD_BYTES - 1] << 8)) return false;
  seq = uint32_t(in[4]) | uint32_t(in[5]) << 8 | uint32_t(in[6]) << 16 | uint32_t(in[7]) << 24;
  for (int i = 0; i < HR_COUNT; i++) s.hr[i] = uint16_t(in[8 + 2 * i] | in[9 + 2 * i] << 8);
  return true;
}

inline bool erased(const uint8_t *p, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    if (p[i] != 0xFF) return false;
  }
  return true;
}

class SettingsStore {
 public:
  explicit SettingsStore(FlashIo &io) : io_(io) {}

  // The newest valid record, clamped (spec §2.3). False if neither page holds one: the caller then uses
  // default_settings() (phase 3 first tries the stock v1 record at 0x0801F000).
  bool load(Settings &out) {
    page_ = -1;
    seq_ = 0;
    for (int p = 0; p < 2; p++) {
      for (uint32_t k = 0; k < RECORD_SLOTS; k++) {
        Settings s;
        uint32_t seq;
        if (!decode_record(io_.read(slot_addr(p, k)), s, seq)) continue;
        if (page_ < 0 || seq > seq_) {
          page_ = p;
          seq_ = seq;
          out = s;
        }
      }
    }
    if (page_ < 0) return false;
    apply_clamps(out);
    return true;
  }

  // Appends a record after the last used slot of the active page. When that page is full or the write fails,
  // the other page is erased and the record goes into its first slot; the page holding the newest record is never
  // the one erased. load() must have run first.
  bool save(const Settings &s) {
    uint8_t rec[RECORD_BYTES];
    encode_record(s, seq_ + 1, rec);
    if (page_ >= 0) {
      const uint32_t slot = next_free(page_);
      if (slot < RECORD_SLOTS && write(page_, slot, rec)) return true;
    }
    const int other = page_ < 0 ? 0 : 1 - page_;
    return io_.erase_page(SETTINGS_PAGE[other]) && write(other, 0, rec);
  }

  uint32_t seq() const { return seq_; }
  int active_page() const { return page_; }

 private:
  static uint32_t slot_addr(int page, uint32_t slot) { return SETTINGS_PAGE[page] + slot * RECORD_BYTES; }

  static bool same(const uint8_t *a, const uint8_t *b) {
    for (uint32_t i = 0; i < RECORD_BYTES; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }

  bool write(int page, uint32_t slot, const uint8_t *rec) {
    const uint32_t addr = slot_addr(page, slot);
    if (!io_.program(addr, rec, RECORD_BYTES) || !same(io_.read(addr), rec)) return false;
    page_ = page;
    seq_++;
    return true;
  }

  // The slot after the last one that isn't fully erased: never write into a hole left before a torn record.
  uint32_t next_free(int page) {
    uint32_t next = 0;
    for (uint32_t k = 0; k < RECORD_SLOTS; k++) {
      if (!erased(io_.read(slot_addr(page, k)), RECORD_BYTES)) next = k + 1;
    }
    return next;
  }

  FlashIo &io_;
  int page_ = -1;  // page holding the newest valid record, -1 if none
  uint32_t seq_ = 0;
};

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_settings: ok`, and every other test `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/crc16.h oem/src/app/settings.h oem/src/app/flash_io.h oem/src/app/settings_store.h \
  tests/test_oem_settings.cpp tests/Makefile .gitignore
git commit -m "OEM settings: HR 0-16 model with spec clamps, and the two-page flash log (torn-write safe)"
```

---

### Task 4: Internal flash (FMC) driver

**Files:**
- Create: `oem/src/hw/flash.h`, `oem/src/hw/flash.cpp`
- Modify: `oem/CMakeLists.txt` (new `oem_drivers` static library)

**Interfaces:**
- Consumes: `oem::FlashIo` (Task 3), `hw::FMC_*` (Task 1).
- Produces: `class hw::Flash : public oem::FlashIo`.
  - `read(addr)` returns `(const uint8_t *)addr`.
  - `erase_page(addr)` takes a 1 KB page.
  - `program(addr, data, len)` writes doublewords.
  - Both return false on any FMC error or timeout, and for any address outside 0x0801F800–0x0801FFFF.

Evidence: `docs/history/notes/oem-adc-flash-sequences.md` §5:
- unlock: KEY = 0x45670123, then 0xCDEF89AB, only while CTL.LK is set;
- erase: PER, ADDR, START, wait, clear PER;
- program: PG, then two word stores, wait, clear PG;
- wait: up to 0x100000 polls of BUSY, then WPERR/PGERR/PGAERR;
- lock: LK.

- [ ] **Step 1: Write the driver**

`oem/src/hw/flash.h` (new file):

```cpp
// oem/src/hw/flash.h: internal flash (FMC) for the settings store: 1 KB page erase and doubleword programming,
// the stock sequence (docs/history/notes/oem-adc-flash-sequences.md §5). Refuses any address outside the two settings pages.
#pragma once
#include <cstdint>

#include "app/flash_io.h"

namespace hw {

class Flash : public oem::FlashIo {
 public:
  const uint8_t *read(uint32_t addr) override { return reinterpret_cast<const uint8_t *>(addr); }
  bool erase_page(uint32_t addr) override;
  bool program(uint32_t addr, const uint8_t *data, uint32_t len) override;
};

}  // namespace hw
```

`oem/src/hw/flash.cpp` (new file):

```cpp
// oem/src/hw/flash.cpp: FMC unlock / page erase / doubleword program / lock as stock (0800e670, 0800e538, 0800e470,
// 0800e52c), except that the error flags really are cleared (stock's flag_clear(0x307) writes a no-op bit) and every
// operation's result is checked. Interrupts are off while the FMC is unlocked, as in stock's append (08006018).
#include "hw/flash.h"

#include "hw/regs.h"

namespace hw {
namespace {

constexpr uint32_t BUSY = 1u << 0, PGERR = 1u << 2, PGAERR = 1u << 3, WPERR = 1u << 4, ENDF = 1u << 5;
constexpr uint32_t PG = 1u << 0, PER = 1u << 1, START = 1u << 6, LK = 1u << 7;
constexpr uint32_t STORE_FIRST = 0x0801F800, STORE_END = 0x08020000;  // spec §5.2: the only pages we may touch

bool in_store(uint32_t addr, uint32_t len) {
  return addr >= STORE_FIRST && addr + len <= STORE_END && addr + len > addr;
}

void unlock() {
  __asm volatile("cpsid i" ::: "memory");
  if (reg(FMC_CTL) & LK) {
    reg(FMC_KEY) = 0x45670123u;
    reg(FMC_KEY) = 0xCDEF89ABu;
  }
  reg(FMC_STAT) = ENDF | WPERR | PGAERR | PGERR;  // write 1 to clear
}

void lock() {
  reg(FMC_CTL) |= LK;
  __asm volatile("cpsie i" ::: "memory");
}

// Stock's ready wait: 0x100000 polls, then the error flags.
bool ready() {
  for (uint32_t t = 0; t < 0x100000; t++) {
    const uint32_t s = reg(FMC_STAT);
    if (!(s & BUSY)) return !(s & (WPERR | PGERR | PGAERR));
  }
  return false;
}

}  // namespace

bool Flash::erase_page(uint32_t addr) {
  if (!in_store(addr, 1024) || (addr & 0x3FFu)) return false;
  unlock();
  bool ok = ready();
  if (ok) {
    reg(FMC_CTL) |= PER;
    reg(FMC_ADDR) = addr;
    reg(FMC_CTL) |= START;
    ok = ready();
    reg(FMC_CTL) &= ~PER;
  }
  lock();
  return ok;
}

bool Flash::program(uint32_t addr, const uint8_t *data, uint32_t len) {
  if (!in_store(addr, len) || (addr & 7u) || (len & 7u)) return false;
  bool ok = true;
  for (uint32_t i = 0; ok && i < len; i += 8) {
    const uint8_t *d = data + i;
    const uint32_t lo = uint32_t(d[0]) | uint32_t(d[1]) << 8 | uint32_t(d[2]) << 16 | uint32_t(d[3]) << 24;
    const uint32_t hi = uint32_t(d[4]) | uint32_t(d[5]) << 8 | uint32_t(d[6]) << 16 | uint32_t(d[7]) << 24;
    unlock();
    ok = ready();
    if (ok) {
      reg(FMC_CTL) |= PG;
      reg(addr + i) = lo;
      reg(addr + i + 4) = hi;
      ok = ready();
      reg(FMC_CTL) &= ~PG;
      __asm volatile("isb" ::: "memory");
    }
    lock();
  }
  return ok;
}

}  // namespace hw
```

`oem/CMakeLists.txt` (whole file):

```cmake
# oem/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
project(oem C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CPU_FLAGS -mcpu=cortex-m23 -mthumb)
set(OEM_VERSION 0.1)

add_compile_options(${CPU_FLAGS} -Os -g -ffunction-sections -fdata-sections -Wall -Wextra -Werror
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>")

set(HW_SOURCES src/startup.c src/hw/clock.cpp src/hw/slcd.cpp)

# Phase-2 drivers, built on their own so each one cross-compiles before the charger app links it.
add_library(oem_drivers STATIC src/hw/flash.cpp)
target_include_directories(oem_drivers PRIVATE src)

function(oem_app name)
  add_executable(${name}.elf ${ARGN})
  target_include_directories(${name}.elf PRIVATE src)
  target_link_options(${name}.elf PRIVATE ${CPU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/ld/app.ld -nostartfiles
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${name}.map)
  add_custom_command(TARGET ${name}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name}.elf ${name}.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../tools/oem_image.py ${name}.bin ${name}.dat --version ${OEM_VERSION})
endfunction()

oem_app(blink ${HW_SOURCES} src/blink.cpp)
oem_app(bringup ${HW_SOURCES} src/bringup.cpp)
```

- [ ] **Step 2: Build and check the disassembly**

Run:
```bash
cmake -S oem -B oem/build && cmake --build oem/build 2>&1 | grep -E -i "warning|error"
T=${ARM_TOOLCHAIN:-$HOME/.local/arm-gnu-toolchain-15.3.rel2}/bin
$T/arm-none-eabi-objdump -d oem/build/liboem_drivers.a | grep -o -E "\.word\s+0x(45670123|cdef89ab|40022010|40022014|0801f7ff|08020000)" | sort -u
$T/arm-none-eabi-objdump -d oem/build/liboem_drivers.a | grep -c -E "cpsid|cpsie"
```
Expected:
- no warnings or errors;
- the six literals, each once: both keys, FMC_CTL, FMC_ADDR, and the store bounds `0x0801f7ff` / `0x08020000`;
- at least 2 `cpsid`/`cpsie` lines.

- [ ] **Step 3: Commit**

```bash
git add oem/src/hw/flash.h oem/src/hw/flash.cpp oem/CMakeLists.txt
git commit -m "OEM FMC driver: stock unlock/erase/doubleword-program/lock, real flag clears, settings pages only"
```

---

### Task 5: CAN: encodings, TX queue and the driver

**Files:**
- Create: `oem/src/app/can_port.h`, `oem/src/hw/can_bits.h`, `oem/src/hw/can.h`, `oem/src/hw/can.cpp`, `tests/test_oem_can.cpp`
- Modify: `tests/Makefile`, `oem/CMakeLists.txt`

**Interfaces:**
- Consumes: `hw::CAN_*` (Task 1), `hw::pin_config`.
- Produces:
  - `struct oem::CanFrame { uint32_t id; uint8_t len; uint8_t data[8]; }`.
  - `class oem::CanPort { virtual bool send(uint32_t id, const uint8_t *data8) = 0; }`.
  - `class oem::CanTxQueue : CanPort`:
    - `send()` queues an extended 8-byte frame, and returns false when the 16-frame queue is full;
    - `template <class F> void pump(F &&hw_send)`;
    - `int size() const`.
  - `hw::can_tmi_ext(id)`, `can_filter_id_ext(id)`, `can_filter_mask_ext(mask)`, `can_rx_is_ext_data(mi)`, `can_rx_ext_id(mi)`,
    `can_bt(psc, bs1, bs2, sjw)` and `le32(p)`. Also `NASN_REPLY_ID = 0x10800000`, `NASN_REPLY_MASK = 0x1F800080`,
    `CAN_TEN/FT/FF`.
  - `hw::can_init() -> bool`, `hw::can_send(const oem::CanFrame &) -> bool` (false = no free mailbox, never waits),
    `hw::can_receive(oem::CanFrame &) -> bool` (false = FIFO0 empty) and `hw::can_error() -> uint32_t`.

- [ ] **Step 1: Write the failing test.** The mailbox word is pinned to the stock pending mailbox recorded in
  `docs/history/notes/oem-can-sequences.md` §4.1, and BT to stock's 0x0014001D.

`tests/test_oem_can.cpp` (new file):

```cpp
#include "app/can_port.h"
#include "check.h"
#include "hw/can_bits.h"

using namespace oem;

static void test_mailbox_words() {
  // The stock firmware's pending mailbox for "read 0x023B from PSU 1" (docs/history/notes/oem-can-sequences.md §4.1).
  CHECK((hw::can_tmi_ext(0x108182FE) | hw::CAN_TEN) == 0x840C17F5);
  const uint8_t d[8] = {0x02, 0x3B, 0, 0, 0, 0, 0, 0};
  CHECK(hw::le32(d) == 0x00003B02 && hw::le32(d + 4) == 0);
  CHECK(hw::can_bt(30, 5, 2, 1) == 0x0014001D);  // stock BT: 30 MHz / 30, 1 + 5 + 2 tq = 125 kbit/s
}

static bool passes(uint32_t mi) {
  const uint32_t id = hw::can_filter_id_ext(hw::NASN_REPLY_ID), mask = hw::can_filter_mask_ext(hw::NASN_REPLY_MASK);
  return ((mi ^ id) & mask) == 0;
}

static void test_filter_words() {
  CHECK(hw::can_filter_id_ext(hw::NASN_REPLY_ID) == 0x84000004);
  CHECK(hw::can_filter_mask_ext(hw::NASN_REPLY_MASK) == 0xFC000406);
  CHECK(passes(hw::can_tmi_ext(0x1081827E)));            // read reply from PSU 1
  CHECK(passes(hw::can_tmi_ext(0x1082817E)));            // write reply from PSU 2
  CHECK(!passes(hw::can_tmi_ext(0x108182FE)));           // a controller request (bit 7 set)
  CHECK(!passes(hw::can_tmi_ext(0x1101007E)));           // proto 0x22 module chatter, dropped in software by stock
  CHECK(!passes(hw::can_tmi_ext(0x1081827E) | hw::CAN_FT));  // remote frame
  CHECK(!passes((0x400u << 21)));                        // a standard-ID frame
}

static void test_rx_decode() {
  const uint32_t mi = hw::can_tmi_ext(0x1081827E);
  CHECK(hw::can_rx_is_ext_data(mi) && hw::can_rx_ext_id(mi) == 0x1081827E);
  CHECK(!hw::can_rx_is_ext_data(mi | hw::CAN_FT));
}

static void test_tx_queue_order_and_backpressure() {
  CanTxQueue q;
  uint8_t d[8] = {};
  for (int i = 0; i < CAN_TX_QUEUE_LEN; i++) {
    d[0] = uint8_t(i);
    CHECK(q.send(0x100u + uint32_t(i), d));
  }
  CHECK(!q.send(0x999, d));  // full
  int mailboxes = 3, next = 0;
  bool in_order = true;
  q.pump([&](const CanFrame &f) {
    if (mailboxes == 0) return false;
    mailboxes--;
    in_order = in_order && f.id == 0x100u + uint32_t(next) && f.data[0] == next && f.len == 8;
    next++;
    return true;
  });
  CHECK(in_order && next == 3 && q.size() == CAN_TX_QUEUE_LEN - 3);
  CHECK(q.send(0x200, d));
  q.pump([&](const CanFrame &) { return true; });
  CHECK(q.size() == 0);
}

int main() {
  test_mailbox_words();
  test_filter_words();
  test_rx_decode();
  test_tx_queue_order_and_backpressure();
  return test_result("test_oem_can");
}
```

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/can_port.h' file not found`.

- [ ] **Step 3: Implement the encodings and the queue**

`oem/src/app/can_port.h` (new file):

```cpp
// oem/src/app/can_port.h: the CAN interface the rectifiers talk through, and the software TX queue that feeds the
// three hardware mailboxes from the main loop (the driver never waits for a mailbox).
#pragma once
#include <cstdint>

namespace oem {

struct CanFrame {
  uint32_t id;  // 29-bit extended ID
  uint8_t len;
  uint8_t data[8];
};

class CanPort {
 public:
  // Queue an 8-byte extended data frame. False if it can't be queued; nothing waits.
  virtual bool send(uint32_t id, const uint8_t *data8) = 0;
};

constexpr int CAN_TX_QUEUE_LEN = 16;  // as the ESP's tx_queue_len

class CanTxQueue : public CanPort {
 public:
  bool send(uint32_t id, const uint8_t *data8) override {
    if (count_ == CAN_TX_QUEUE_LEN) return false;
    CanFrame &f = ring_[(head_ + count_) % CAN_TX_QUEUE_LEN];
    f.id = id;
    f.len = 8;
    for (int i = 0; i < 8; i++) f.data[i] = data8[i];
    count_++;
    return true;
  }

  // Hands queued frames, oldest first, to hw_send(const CanFrame &) -> bool until it reports no free mailbox.
  template <class F>
  void pump(F &&hw_send) {
    while (count_ > 0 && hw_send(ring_[head_])) {
      head_ = (head_ + 1) % CAN_TX_QUEUE_LEN;
      count_--;
    }
  }

  int size() const { return count_; }

 private:
  CanFrame ring_[CAN_TX_QUEUE_LEN] = {};
  int head_ = 0;
  int count_ = 0;
};

}  // namespace oem
```

`oem/src/hw/can_bits.h` (new file):

```cpp
// oem/src/hw/can_bits.h: bxCAN mailbox and filter word encodings (docs/history/notes/oem-can-sequences.md §3, §4, §5). Pure
// constexpr, so the host tests pin them.
#pragma once
#include <cstdint>

namespace hw {

constexpr uint32_t CAN_TEN = 1u << 0;  // TMIx: transmit request
constexpr uint32_t CAN_FT = 1u << 1;   // remote frame
constexpr uint32_t CAN_FF = 1u << 2;   // extended frame

// TMIx for an extended data frame, without TEN: EFID in bits 31..3, FF set.
constexpr uint32_t can_tmi_ext(uint32_t id) { return ((id & 0x1FFFFFFFu) << 3) | CAN_FF; }

// One 32-bit mask-mode filter bank uses the same layout as TMI/RFIFOMI. The mask also compares FF and FT, so only
// extended data frames pass.
constexpr uint32_t can_filter_id_ext(uint32_t id) { return ((id & 0x1FFFFFFFu) << 3) | CAN_FF; }
constexpr uint32_t can_filter_mask_ext(uint32_t mask) { return ((mask & 0x1FFFFFFFu) << 3) | CAN_FF | CAN_FT; }

// NASN replies: protocol 0x21 with the direction bit (7) clear, the same match as the ESP's on_frame.
constexpr uint32_t NASN_REPLY_ID = 0x10800000;
constexpr uint32_t NASN_REPLY_MASK = 0x1F800080;

constexpr bool can_rx_is_ext_data(uint32_t mi) { return (mi & (CAN_FF | CAN_FT)) == CAN_FF; }
constexpr uint32_t can_rx_ext_id(uint32_t mi) { return mi >> 3; }

// BT: (psc - 1) | bs1 - 1 at 16 | bs2 - 1 at 20 | sjw - 1 at 24, normal mode.
constexpr uint32_t can_bt(uint32_t psc, uint32_t bs1_tq, uint32_t bs2_tq, uint32_t sjw_tq) {
  return ((psc - 1) & 0x3FFu) | ((bs1_tq - 1) & 0xFu) << 16 | ((bs2_tq - 1) & 0x7u) << 20 | ((sjw_tq - 1) & 0x3u) << 24;
}

constexpr uint32_t le32(const uint8_t *d) {
  return uint32_t(d[0]) | uint32_t(d[1]) << 8 | uint32_t(d[2]) << 16 | uint32_t(d[3]) << 24;
}

}  // namespace hw
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_can: ok`, and every other test `ok`.

- [ ] **Step 5: The driver** (stock sequence from `docs/history/notes/oem-can-sequences.md` §9, with the decided filter and no interrupt)

`oem/src/hw/can.h` (new file):

```cpp
// oem/src/hw/can.h: the bxCAN at 0x40006400 on PA4 (RX) / PA5 (TX), 125 kbit/s, polled (IRQ20 is shared with USBD).
#pragma once
#include <cstdint>

#include "app/can_port.h"

namespace hw {

// Stock init (docs/history/notes/oem-can-sequences.md §1-§3) with one filter bank passing only NASN replies to FIFO0, and no
// interrupts. False if the controller never entered or left init mode.
bool can_init();
// Into a free TX mailbox. False if all three are busy; never waits.
bool can_send(const oem::CanFrame &f);
// One frame from FIFO0. False if it is empty. A frame that isn't an extended data frame comes back with len 0.
bool can_receive(oem::CanFrame &f);
// CAN_ERR (TEC, REC, error state) for diagnostics.
uint32_t can_error();

}  // namespace hw
```

`oem/src/hw/can.cpp` (new file):

```cpp
// oem/src/hw/can.cpp: register sequence from the stock v1 driver (docs/history/notes/oem-can-sequences.md §9), with a NASN-reply
// filter instead of accept-all, RFD0 written rather than OR'ed, and no RX interrupt.
#include "hw/can.h"

#include "hw/can_bits.h"
#include "hw/gpio.h"

namespace hw {
namespace {

constexpr uint32_t IWMOD = 1u << 0, SLPWMOD = 1u << 1, RFOD = 1u << 3, ABOR = 1u << 6;
constexpr uint32_t CTL_CONFIG_MASK = 0xFCu;  // TFO, RFOD, ARD, AWU, ABOR, TTC
constexpr uint32_t IWS = 1u << 0;            // STAT: in init mode
constexpr uint32_t RFD0 = 1u << 5;           // RFIFO0: release the output mailbox
constexpr uint32_t TME0 = 1u << 26;          // TSTAT: mailbox 0 empty (1 and 2 follow)

bool wait_stat(bool want_init) {
  for (uint32_t t = 0; t < 0xFFFF; t++) {
    if (bool(reg(CAN_STAT) & IWS) == want_init) return true;
  }
  return false;
}

}  // namespace

bool can_init() {
  reg(RCU_APB1EN) |= 1u << 16;  // CANEN
  reg(RCU_AHBEN) |= 1u << 17;   // GPIOA
  pin_config({GPIOA, 4}, Mode::AF, Pull::NONE, 11);  // CAN_RX
  pin_config({GPIOA, 5}, Mode::AF, Pull::NONE, 11);  // CAN_TX
  reg(RCU_APB1RST) |= 1u << 16;
  reg(RCU_APB1RST) &= ~(1u << 16);

  reg(CAN_CTL) &= ~SLPWMOD;
  reg(CAN_CTL) |= IWMOD;
  if (!wait_stat(true)) return false;
  reg(CAN_BT) = can_bt(30, 5, 2, 1);  // 0x0014001D: 30 MHz / 30, 8 tq, 125 kbit/s, sample point 75 %
  // As stock: automatic bus-off recovery, automatic retransmission, FIFO0 keeps the oldest frames when full.
  rmw(CAN_CTL, CTL_CONFIG_MASK, ABOR | RFOD);
  reg(CAN_INTEN) = 0;

  reg(CAN_FCTL) |= 1u;  // filter init mode
  reg(CAN_FW) &= ~1u;
  reg(CAN_FSCFG) |= 1u;  // bank 0: 32-bit
  reg(CAN_F0DATA0) = can_filter_id_ext(NASN_REPLY_ID);
  reg(CAN_F0DATA1) = can_filter_mask_ext(NASN_REPLY_MASK);
  reg(CAN_FMCFG) &= ~1u;   // mask mode
  reg(CAN_FAFIFO) &= ~1u;  // to FIFO0
  reg(CAN_FW) |= 1u;
  reg(CAN_FCTL) &= ~1u;

  reg(CAN_CTL) &= ~IWMOD;
  return wait_stat(false);  // needs 11 recessive bits on the bus
}

bool can_send(const oem::CanFrame &f) {
  const uint32_t tstat = reg(CAN_TSTAT);
  int n = 0;
  while (n < 3 && !(tstat & (TME0 << n))) n++;
  if (n == 3) return false;
  reg(can_tmi(n)) = can_tmi_ext(f.id);
  rmw(can_tmp(n), 0xFu, 8u);
  reg(can_tmdata0(n)) = le32(f.data);
  reg(can_tmdata1(n)) = le32(f.data + 4);
  reg(can_tmi(n)) |= CAN_TEN;
  return true;
}

bool can_receive(oem::CanFrame &f) {
  const uint32_t fifo = reg(CAN_RFIFO0);
  if ((fifo & 3u) == 0 || (fifo & RFD0)) return false;  // empty, or the last release is still in progress
  const uint32_t mi = reg(CAN_RFIFOMI0);
  const uint32_t len = reg(CAN_RFIFOMP0) & 0xFu;
  const uint32_t d0 = reg(CAN_RFIFOMDATA00), d1 = reg(CAN_RFIFOMDATA10);
  reg(CAN_RFIFO0) = RFD0;
  f.id = can_rx_ext_id(mi);
  f.len = can_rx_is_ext_data(mi) && len <= 8 ? uint8_t(len) : 0;
  for (int i = 0; i < 4; i++) {
    f.data[i] = uint8_t(d0 >> (8 * i));
    f.data[4 + i] = uint8_t(d1 >> (8 * i));
  }
  return true;
}

uint32_t can_error() { return reg(CAN_ERR); }

}  // namespace hw
```

`oem/CMakeLists.txt` (whole file):

```cmake
# oem/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
project(oem C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CPU_FLAGS -mcpu=cortex-m23 -mthumb)
set(OEM_VERSION 0.1)

add_compile_options(${CPU_FLAGS} -Os -g -ffunction-sections -fdata-sections -Wall -Wextra -Werror
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>")

set(HW_SOURCES src/startup.c src/hw/clock.cpp src/hw/slcd.cpp)

# Phase-2 drivers, built on their own so each one cross-compiles before the charger app links it.
add_library(oem_drivers STATIC src/hw/flash.cpp src/hw/can.cpp)
target_include_directories(oem_drivers PRIVATE src)

function(oem_app name)
  add_executable(${name}.elf ${ARGN})
  target_include_directories(${name}.elf PRIVATE src)
  target_link_options(${name}.elf PRIVATE ${CPU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/ld/app.ld -nostartfiles
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${name}.map)
  add_custom_command(TARGET ${name}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name}.elf ${name}.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../tools/oem_image.py ${name}.bin ${name}.dat --version ${OEM_VERSION})
endfunction()

oem_app(blink ${HW_SOURCES} src/blink.cpp)
oem_app(bringup ${HW_SOURCES} src/bringup.cpp)
```

- [ ] **Step 6: Build and check the disassembly**

Run:
```bash
cmake --build oem/build 2>&1 | grep -E -i "warning|error"
T=${ARM_TOOLCHAIN:-$HOME/.local/arm-gnu-toolchain-15.3.rel2}/bin
$T/arm-none-eabi-objdump -d oem/build/liboem_drivers.a | grep -o -E "\.word\s+0x(0014001d|84000004|fc000406|40006640|40006644|400065b0)" | sort -u
$T/arm-none-eabi-objdump -d oem/build/liboem_drivers.a | grep -c "40006414"
```
Expected:
- no warnings or errors;
- all six literals: BT, filter ID, filter mask, F0DATA0, F0DATA1, RFIFOMI0;
- CAN_INTEN (`40006414`) appears (written 0).

- [ ] **Step 7: Commit**

```bash
git add oem/src/app/can_port.h oem/src/hw/can_bits.h oem/src/hw/can.h oem/src/hw/can.cpp tests/test_oem_can.cpp \
  tests/Makefile oem/CMakeLists.txt
git commit -m "OEM CAN: stock bxCAN init at 125k, NASN-reply filter, polled FIFO0, non-blocking TX behind a 16-frame queue"
```

---

### Task 6: ADC: battery-voltage sense on PC0

**Files:**
- Create: `oem/src/app/battery_adc.h`, `oem/src/hw/adc.h`, `oem/src/hw/adc.cpp`, `tests/test_oem_adc.cpp`
- Modify: `tests/Makefile`, `oem/CMakeLists.txt`

**Interfaces:**
- Consumes: `hw::ADC_*`, `RCU_APB2EN`, `RCU_CFG0`, `RCU_CFG2` (Task 1).
- Produces:
  - `oem::ADC_SAMPLES = 10`, `oem::adc_trimmed_mean(const uint16_t (&)[10]) -> uint16_t` and
    `oem::adc_volts(uint16_t counts, int32_t trim_permille) -> float`.
  - `hw::adc_init() -> bool` and `hw::adc_read(uint16_t &) -> bool` (one conversion, about 11 µs; false on a timeout).

Evidence: `docs/history/notes/oem-adc-flash-sequences.md` §1a, §2 and §4.
- Clock: CFG0 bits 15:14 = 01 and CFG2 bits 31:30 = 01 with ADCSEL (bit 8), giving APB2 / 12 = 5 MHz.
- Channel 10 sample time: SAMPT0 = 4 (41.5 cycles). CCTL = 0x140.
- Software trigger: CTL1 ETSRC = 7, ETERC.
- Calibration: RSTCLB then CLB.
- Per sample: SWRCST, then poll EOC and read RDATA.

- [ ] **Step 1: Write the failing test**

`tests/test_oem_adc.cpp` (new file):

```cpp
#include "app/battery_adc.h"
#include "check.h"

using namespace oem;

static void test_trimmed_mean_drops_min_and_max() {
  const uint16_t s[ADC_SAMPLES] = {3000, 3100, 3100, 3100, 3100, 3100, 3100, 3100, 3100, 4095};
  CHECK(adc_trimmed_mean(s) == 3100);
  const uint16_t r[ADC_SAMPLES] = {9, 1, 8, 2, 7, 3, 6, 4, 5, 0};
  CHECK(adc_trimmed_mean(r) == (1 + 2 + 3 + 4 + 5 + 6 + 7 + 8) / 8);
}

static void test_scale_and_trim() {
  CHECK_NEAR(adc_volts(4095, 0), 69.30, 0.01);  // stock full scale: 4095 x 0.16923 = 693.0 (0.1 V)
  CHECK_NEAR(adc_volts(3073, 0), 52.00, 0.01);
  CHECK_NEAR(adc_volts(3073, 10), 52.52, 0.01);   // +1.0 %
  CHECK_NEAR(adc_volts(3073, -50), 49.40, 0.01);  // -5.0 %
}

int main() {
  test_trimmed_mean_drops_min_and_max();
  test_scale_and_trim();
  return test_result("test_oem_adc");
}
```

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can \
  test_oem_adc
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/battery_adc.h' file not found`.

- [ ] **Step 3: Implement the conversion helpers**

`oem/src/app/battery_adc.h` (new file):

```cpp
// oem/src/app/battery_adc.h: PC0 samples to volts. Stock (08007630 / 08007668): sort 10 samples, average the middle
// 8, scale by 0.16923 to 0.1 V units. We keep volts as a float and apply the HR 14 trim (per mille).
#pragma once
#include <cstdint>

namespace oem {

constexpr int ADC_SAMPLES = 10;
constexpr float ADC_VOLTS_PER_COUNT = 0.016923f;  // 4095 counts = 69.3 V

inline uint16_t adc_trimmed_mean(const uint16_t (&in)[ADC_SAMPLES]) {
  uint16_t a[ADC_SAMPLES];
  for (int i = 0; i < ADC_SAMPLES; i++) a[i] = in[i];
  for (int i = 0; i < ADC_SAMPLES - 1; i++) {
    for (int j = 0; j < ADC_SAMPLES - 1 - i; j++) {
      if (a[j] > a[j + 1]) {
        const uint16_t t = a[j];
        a[j] = a[j + 1];
        a[j + 1] = t;
      }
    }
  }
  uint32_t sum = 0;
  for (int i = 1; i < ADC_SAMPLES - 1; i++) sum += a[i];
  return uint16_t(sum >> 3);
}

inline float adc_volts(uint16_t counts, int32_t trim_permille) {
  return float(counts) * ADC_VOLTS_PER_COUNT * (1.0f + float(trim_permille) / 1000.0f);
}

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_adc: ok`, and every other test `ok`.

- [ ] **Step 5: The driver**

`oem/src/hw/adc.h` (new file):

```cpp
// oem/src/hw/adc.h: PC0 = ADC_IN10, the 48 V sense. Polled single conversions; stock free-runs a 10-sample DMA scan
// instead (docs/history/notes/oem-adc-flash-sequences.md §2-§4).
#pragma once
#include <cstdint>

namespace hw {

bool adc_init();  // stock clock (APB2 / 12 = 5 MHz), 41.5-cycle sampling, calibration; false if calibration hangs
// One 12-bit conversion of channel 10 (about 11 us). False on a timeout (out is left alone).
bool adc_read(uint16_t &out);

}  // namespace hw
```

`oem/src/hw/adc.cpp` (new file):

```cpp
// oem/src/hw/adc.cpp: docs/history/notes/oem-adc-flash-sequences.md §4, the polled form of the stock init (0800816c): same clock,
// sample time, charge pulse width and calibration, one regular rank, single conversions by software trigger.
#include "hw/adc.h"

#include "hw/gpio.h"

namespace hw {
namespace {

constexpr uint32_t ADCON = 1u << 0, CLB = 1u << 2, RSTCLB = 1u << 3;
constexpr uint32_t ETSRC_SW = 7u << 17, ETERC = 1u << 20, SWRCST = 1u << 22;
constexpr uint32_t EOC = 1u << 1;

bool wait_clear(uint32_t addr, uint32_t bit) {
  for (uint32_t t = 0; t < 0x10000; t++) {
    if (!(reg(addr) & bit)) return true;
  }
  return false;
}

}  // namespace

bool adc_init() {
  reg(RCU_APB2EN) |= 1u << 9;   // ADCEN
  reg(RCU_AHBEN) |= 1u << 19;   // GPIOC
  pin_config({GPIOC, 0}, Mode::ANALOG, Pull::NONE);
  rmw(RCU_CFG0, 0xC000u, 0x4000u);           // ADC prescaler bits, as stock rcu_adc_clock_config(5)
  rmw(RCU_CFG2, 0xC0000100u, 0x40000100u);   // ADCSEL + ADCPSC2: CK_ADC = APB2 / 12 = 5 MHz
  reg(ADC_RSQ0) = 0;                         // one conversion
  reg(ADC_RSQ2) = 10;                        // rank 0 = channel 10
  rmw(ADC_SAMPT0, 0x7u, 0x4u);               // channel 10: 41.5 cycles
  reg(ADC_CTL0) = 0;                         // no scan; 12-bit
  reg(ADC_CTL1) = ETSRC_SW | ETERC;          // software trigger, single, right-aligned, no DMA
  reg(ADC_CCTL) = 0x140;
  reg(ADC_CTL1) |= ADCON;
  for (volatile uint32_t i = 0; i < 600; i = i + 1) {  // >= 5 us start-up (stock waits 1 ms)
  }
  reg(ADC_CTL1) |= RSTCLB;
  if (!wait_clear(ADC_CTL1, RSTCLB)) return false;
  reg(ADC_CTL1) |= CLB;
  return wait_clear(ADC_CTL1, CLB);
}

bool adc_read(uint16_t &out) {
  reg(ADC_STAT) &= ~EOC;
  reg(ADC_CTL1) |= SWRCST;
  for (uint32_t t = 0; t < 0x4000; t++) {
    if (reg(ADC_STAT) & EOC) {
      out = uint16_t(reg(ADC_RDATA) & 0xFFFu);
      return true;
    }
  }
  return false;
}

}  // namespace hw
```

`oem/CMakeLists.txt` (whole file):

```cmake
# oem/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
project(oem C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CPU_FLAGS -mcpu=cortex-m23 -mthumb)
set(OEM_VERSION 0.1)

add_compile_options(${CPU_FLAGS} -Os -g -ffunction-sections -fdata-sections -Wall -Wextra -Werror
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>")

set(HW_SOURCES src/startup.c src/hw/clock.cpp src/hw/slcd.cpp)

# Phase-2 drivers, built on their own so each one cross-compiles before the charger app links it.
add_library(oem_drivers STATIC src/hw/flash.cpp src/hw/can.cpp src/hw/adc.cpp)
target_include_directories(oem_drivers PRIVATE src)

function(oem_app name)
  add_executable(${name}.elf ${ARGN})
  target_include_directories(${name}.elf PRIVATE src)
  target_link_options(${name}.elf PRIVATE ${CPU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/ld/app.ld -nostartfiles
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${name}.map)
  add_custom_command(TARGET ${name}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name}.elf ${name}.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../tools/oem_image.py ${name}.bin ${name}.dat --version ${OEM_VERSION})
endfunction()

oem_app(blink ${HW_SOURCES} src/blink.cpp)
oem_app(bringup ${HW_SOURCES} src/bringup.cpp)
```

- [ ] **Step 6: Build and check the disassembly**

Run:
```bash
cmake --build oem/build 2>&1 | grep -E -i "warning|error"
T=${ARM_TOOLCHAIN:-$HOME/.local/arm-gnu-toolchain-15.3.rel2}/bin
$T/arm-none-eabi-objdump -d oem/build/liboem_drivers.a | grep -o -E "\.word\s+0x(40000100|c0000100|400124c0|4001244c|40021018|40021030)" | sort -u
```
Expected: no warnings or errors, and all six literals: the CFG2 value and mask, CCTL, RDATA, APB2EN and CFG2.

- [ ] **Step 7: Commit**

```bash
git add oem/src/app/battery_adc.h oem/src/hw/adc.h oem/src/hw/adc.cpp tests/test_oem_adc.cpp tests/Makefile \
  oem/CMakeLists.txt
git commit -m "OEM ADC: polled PC0 conversions with the stock clock, sampling and calibration; stock trimmed mean and scale"
```

---

### Task 7: `Rectifier`: one NASN module

A port of `esphome/components/nasn_rectifier/nasn_rectifier.{h,cpp}`. Behaviour kept from the ESP:
- setpoints are written on change and every 5 s;
- live readback checks on 0x022F / 0x022D / 0x0132 against what was queued before the read, faulting after 10 s;
- stored defaults (0x0101 / 0x0104) are rewritten only when their readback differs, at most once per 5 s, clamped to
  54.4 V / 10 A;
- the alarm mask is `STOCK_ALARM_MASK`;
- 3 consecutive rejected writes count as faulted;
- going offline clears all readings.

Changes: decision D3 (in-flight read, 250 ms / 5 s / 1 s), the fixed 12-register poll list, and no sensors.

**Files:**
- Create: `oem/src/app/rectifier.h`, `tests/fake_nasn_bus.h`, `tests/test_oem_rectifier.cpp`
- Modify: `tests/Makefile`

**Interfaces:**
- Consumes: `oem::CanPort`, `oem::CanFrame` (Task 5), and `nasn_protocol.h`.
- Produces: `class oem::Rectifier(CanPort &, uint8_t address, bool write_stored_defaults)`.
  - Driving it:
    - `loop(uint32_t now)` every pass;
    - `on_frame(const CanFrame &, uint32_t now)` for every received frame;
    - `set_target(float v, float a)`, `set_standby(bool)` and `set_fallback(float v, float a)`.
  - Status: `is_online()`, `faulted()`, `setpoint_mismatch()`, `boot_sent()`, `alarms()`, `consecutive_write_errors()`,
    `address()`, `standby()`, `target_current()`.
  - Readings: `output_voltage()`, `output_current()`, `input_voltage()`, `heatsink_temperature()`, all NaN when unknown.
  - Constants: `FALLBACK_MAX_V`, `FALLBACK_MAX_A`, `POLL_SPACING_MS`, `POLL_CYCLE_MS`, `REQUEST_TIMEOUT_MS`, `OFFLINE_MS` and
    `KEEPALIVE_MS`.
- Test helpers (`tests/fake_nasn_bus.h`, used again by Task 9):
  - `FakeModule(addr)`, with fields `battery_v`, `vin`, `heatsink_c`, `vout_set`, `ilimit`, `default_v`, `default_a`, `standby`,
    `alarm`, `muted`, `reject_writes`, `frozen` and `load_connected`.
  - `FakeBus : CanPort`, with `deliver(mods, rects, now)`, `writes(reg)`, `sent`, `pending` and `full`.

- [ ] **Step 1: Write the fake bus and the failing test**

`tests/fake_nasn_bus.h` (new file):

```cpp
#pragma once
// Host-test stand-ins for the CAN bus and NASN modules, modelled on tools/fake_nasn.py: a module answers cmd 0x82
// reads and cmd 0x81 writes; a running module whose setpoint is above the battery delivers its current limit.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "app/can_port.h"
#include "app/rectifier.h"

struct FakeModule {
  explicit FakeModule(uint8_t a) : addr(a) {}

  uint8_t addr;
  float battery_v = 52.0f, vin = 230.0f, heatsink_c = 35.0f;
  float vout_set = 54.0f, ilimit = 5.0f, default_v = 54.0f, default_a = 5.0f;
  int standby = 0;
  uint32_t alarm = 0;           // low 32 bits of the 0x0183 value
  bool muted = false;
  bool reject_writes = false;   // answer writes with status 2
  bool frozen = false;          // ignore setpoint writes (but ack them): a module not following setpoints
  bool load_connected = true;   // false: DC output open, no current flows

  float iout() const { return !standby && load_connected && vout_set > battery_v ? ilimit : 0.0f; }
  // A running module set below the battery can't pull it down: it reads the battery voltage.
  float vout() const { return standby ? battery_v : std::max(vout_set, battery_v); }

  uint64_t read(uint16_t reg, bool &ok) const {
    ok = true;
    auto t = [](float x) { return uint64_t(std::lround(double(x) * 1024.0)); };
    switch (reg) {
      case 0x0100: return uint64_t(std::lround(vout_set * 1023.0f));
      case 0x0101: return uint64_t(std::lround(default_v * 1023.0f));
      case 0x0104: return uint64_t(std::lround(default_a * 1023.0f / 52.0f));
      case 0x0132: return uint64_t(standby);
      case 0x0175: return t(vout());
      case 0x0178: return t(vin);
      case 0x0182: return t(iout());
      case 0x0183: return alarm;
      case 0x0184: return t(vout());
      case 0x0235: return t(heatsink_c);
      case 0x022D: return uint64_t(std::lround(ilimit * 1023.0f / 52.0f));
      case 0x022F: return uint64_t(std::lround(vout_set * 1023.0f));
      default: ok = false; return 0;
    }
  }

  bool handle(const oem::CanFrame &req, oem::CanFrame &rep) {
    if (muted || req.len != 8 || ((req.id >> 23) & 0x3F) != 0x21 || !(req.id & 0x80)) return false;
    if (((req.id >> 16) & 0x7F) != addr) return false;
    const uint8_t cmd = (req.id >> 8) & 0xFF;
    const uint16_t reg = uint16_t((req.data[0] & 0x0F) << 8 | req.data[1]);
    rep.id = req.id & ~0x80u;
    rep.len = 8;
    for (int i = 0; i < 8; i++) rep.data[i] = req.data[i];
    if (cmd == 0x82) {
      bool ok;
      const uint64_t v = read(reg, ok);
      for (int i = 0; i < 6; i++) rep.data[2 + i] = ok ? uint8_t(v >> (8 * (5 - i))) : 0;
      if (!ok) rep.data[0] = uint8_t(0x20 | (req.data[0] & 0x0F));
      return true;
    }
    if (cmd != 0x81) return false;
    if (reject_writes) {
      rep.data[0] = uint8_t(0x20 | (req.data[0] & 0x0F));
      return true;
    }
    const uint16_t raw = uint16_t(req.data[6] << 8 | req.data[7]);
    if (frozen) return true;
    if (reg == 0x0100) vout_set = raw / 1023.0f;
    if (reg == 0x022D) ilimit = raw * 52.0f / 1023.0f;
    if (reg == 0x0132) standby = req.data[3];
    if (reg == 0x0101) default_v = raw / 1023.0f;
    if (reg == 0x0104) default_a = raw * 52.0f / 1023.0f;
    return true;
  }
};

class FakeBus : public oem::CanPort {
 public:
  bool send(uint32_t id, const uint8_t *d) override {
    if (full) return false;
    oem::CanFrame f{id, 8, {}};
    for (int i = 0; i < 8; i++) f.data[i] = d[i];
    sent.push_back(f);
    pending.push_back(f);
    return true;
  }

  // Every pending request reaches the modules; every reply reaches every rectifier.
  void deliver(std::vector<FakeModule *> mods, std::vector<oem::Rectifier *> rects, uint32_t now) {
    std::vector<oem::CanFrame> reqs;
    reqs.swap(pending);
    for (const auto &q : reqs) {
      for (auto *m : mods) {
        oem::CanFrame rep{};
        if (!m->handle(q, rep)) continue;
        for (auto *r : rects) r->on_frame(rep, now);
      }
    }
  }

  // Writes (cmd 0x81) to reg sent so far.
  int writes(uint16_t reg) const {
    int n = 0;
    for (const auto &f : sent) n += ((f.id >> 8) & 0xFF) == 0x81 && (f.data[0] << 8 | f.data[1]) == reg;
    return n;
  }

  std::vector<oem::CanFrame> sent, pending;
  bool full = false;
};
```

`tests/test_oem_rectifier.cpp` (new file):

```cpp
#include <cmath>

#include "check.h"
#include "fake_nasn_bus.h"

using oem::Rectifier;

struct Rig {
  explicit Rig(bool write_defaults = true) : r(bus, 1, write_defaults) {}
  FakeBus bus;
  FakeModule mod{1};
  Rectifier r;
  uint32_t now = 1000;

  void run(uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++, now++) {
      r.loop(now);
      bus.deliver({&mod}, {&r}, now);
    }
  }
};

static void test_comes_online_and_reads_telemetry() {
  Rig g;
  g.mod.vin = 231.0f;
  g.mod.heatsink_c = 41.5f;
  CHECK(!g.r.is_online());
  g.run(400);
  CHECK(g.r.is_online());
  CHECK(g.r.boot_sent());
  CHECK_NEAR(g.r.input_voltage(), 231.0, 0.01);
  CHECK_NEAR(g.r.heatsink_temperature(), 41.5, 0.01);
  CHECK_NEAR(g.r.output_current(), 0.0, 0.01);  // boots in standby: the first setpoints say so
  CHECK(g.mod.standby == 1);
  CHECK(!g.r.faulted());
}

static void test_one_read_in_flight_with_spacing() {
  Rig g;
  g.mod.muted = true;  // nothing answers: each read waits out its 250 ms timeout
  g.run(1000);
  int reads = 0;
  for (const auto &f : g.bus.sent) reads += ((f.id >> 8) & 0xFF) == 0x82;
  CHECK(reads == 4);  // at 0, 250, 500 and 750 ms
  Rig h;             // answered at once: one read per 20 ms until the 12-register cycle is done
  h.run(1000);
  reads = 0;
  for (const auto &f : h.bus.sent) reads += ((f.id >> 8) & 0xFF) == 0x82;
  CHECK(reads == 12);
  h.run(1000);  // the next cycle starts a second after the first
  reads = 0;
  for (const auto &f : h.bus.sent) reads += ((f.id >> 8) & 0xFF) == 0x82;
  CHECK(reads == 24);
}

static void test_offline_after_5s_silence() {
  Rig g;
  g.run(500);
  CHECK(g.r.is_online());
  g.mod.muted = true;  // the last reply came at about 1220 ms
  g.run(4500);
  CHECK(g.r.is_online());
  g.run(1000);
  CHECK(!g.r.is_online());
  CHECK(std::isnan(g.r.output_voltage()) && std::isnan(g.r.input_voltage()));
  g.mod.muted = false;
  const int before = g.bus.writes(0x0100);
  g.run(300);
  CHECK(g.r.is_online());
  CHECK(g.bus.writes(0x0100) == before + 1);  // setpoints again as soon as it reappears
}

static void test_setpoints_on_change_and_keepalive() {
  Rig g;
  g.run(500);
  g.r.set_target(56.0f, 20.0f);
  g.r.set_standby(false);
  g.run(50);
  CHECK_NEAR(g.mod.vout_set, 56.0, 0.01);
  CHECK_NEAR(g.mod.ilimit, 20.0, 0.1);
  CHECK(g.mod.standby == 0);
  const int n = g.bus.writes(0x0100);
  g.r.set_target(56.005f, 20.05f);  // below the 0.01 V / 0.1 A change thresholds
  g.run(4000);
  CHECK(g.bus.writes(0x0100) == n);
  g.run(1100);
  CHECK(g.bus.writes(0x0100) == n + 1);  // 5 s keepalive
  CHECK(g.bus.writes(0x022D) == n + 1 && g.bus.writes(0x0132) == n + 1);
}

static void test_queue_full_retries_at_keepalive() {
  Rig g;
  g.run(500);
  g.bus.full = true;
  g.r.set_target(55.0f, 10.0f);
  g.run(100);
  g.bus.full = false;
  g.run(1000);
  CHECK_NEAR(g.mod.vout_set, 54.0, 0.01);  // as the ESP: a failed send waits for the keepalive
  g.run(4000);
  CHECK_NEAR(g.mod.vout_set, 55.0, 0.01);
}

static void test_alarm_mask() {
  Rig g;
  g.run(500);
  g.mod.alarm = 0x00000020;  // bit 5: not in the stock mask
  g.run(1000);
  CHECK(g.r.alarms() == 0x20 && !g.r.faulted());
  g.mod.alarm = 0x00400000;  // bit 22 (E1)
  g.run(1000);
  CHECK(g.r.faulted());
  g.mod.alarm = 0;
  g.run(1000);
  CHECK(!g.r.faulted());
}

static void test_setpoint_mismatch_after_10s() {
  Rig g;
  g.run(500);
  g.mod.frozen = true;
  g.r.set_target(56.0f, 20.0f);
  g.r.set_standby(false);
  g.run(9000);
  CHECK(!g.r.setpoint_mismatch());
  g.run(3000);
  CHECK(g.r.setpoint_mismatch() && g.r.faulted());
  g.mod.frozen = false;
  g.run(6000);  // the keepalive rewrites the setpoints and the readback agrees again
  CHECK(!g.r.setpoint_mismatch() && !g.r.faulted());
}

static void test_write_errors() {
  Rig g;
  g.run(500);
  g.mod.reject_writes = true;
  g.r.set_target(55.0f, 5.0f);  // 3 rejected writes (voltage, current, standby)
  g.run(100);
  CHECK(g.r.consecutive_write_errors() == 3 && g.r.faulted());
  g.mod.reject_writes = false;
  g.run(5000);
  CHECK(g.r.consecutive_write_errors() == 0 && !g.r.faulted());
}

static void test_stored_defaults() {
  Rig g;
  g.r.set_fallback(57.0f, 30.0f);  // clamped to 54.4 V / 10 A
  g.run(7000);  // 0x0101 is fixed on its first readback, 0x0104 (polled later) one keepalive after
  CHECK_NEAR(g.mod.default_v, 54.4, 0.01);
  CHECK_NEAR(g.mod.default_a, 10.0, 0.1);
  const int n = g.bus.writes(0x0101);
  CHECK(n == 1);
  g.run(10000);
  CHECK(g.bus.writes(0x0101) == n);  // the readback now matches: no more writes
  Rig off(false);
  off.run(1500);
  CHECK(off.bus.writes(0x0101) == 0 && off.bus.writes(0x0104) == 0);
}

static void test_ignores_other_address() {
  FakeBus bus;
  FakeModule two(2);
  Rectifier r(bus, 1, false);
  uint32_t now = 1000;
  for (int i = 0; i < 1000; i++, now++) {
    r.loop(now);
    bus.deliver({&two}, {&r}, now);
  }
  CHECK(!r.is_online());
}

int main() {
  test_comes_online_and_reads_telemetry();
  test_one_read_in_flight_with_spacing();
  test_offline_after_5s_silence();
  test_setpoints_on_change_and_keepalive();
  test_queue_full_retries_at_keepalive();
  test_alarm_mask();
  test_setpoint_mismatch_after_10s();
  test_write_errors();
  test_stored_defaults();
  test_ignores_other_address();
  return test_result("test_oem_rectifier");
}
```

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can \
  test_oem_adc test_oem_rectifier
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/rectifier.h' file not found`.

- [ ] **Step 3: Implement**

`oem/src/app/rectifier.h` (new file):

```cpp
// oem/src/app/rectifier.h: one NASN RM4850-B module. A port of the ESP component (esphome/components/nasn_rectifier)
// without ESPHome: same registers, setpoint keepalive, readback checks, stored defaults and alarm mask, from
// nasn_protocol.h. Differences: one read in flight at a time (spec §1), a poll cycle every second, and offline after
// 5 s without a good read reply (the ESP used 3 x its update interval).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "app/can_port.h"
#include "nasn_rectifier/nasn_protocol.h"

namespace oem {

constexpr float FALLBACK_MAX_V = 54.4f;  // stored defaults: what a module runs on alone if we go quiet
constexpr float FALLBACK_MAX_A = 10.0f;
constexpr uint32_t POLL_SPACING_MS = 20;
constexpr uint32_t POLL_CYCLE_MS = 1000;
constexpr uint32_t REQUEST_TIMEOUT_MS = 250;  // give up on one read and poll the next register
constexpr uint32_t OFFLINE_MS = 5000;         // no good read reply this long: offline
constexpr uint32_t KEEPALIVE_MS = 5000;

class Rectifier {
 public:
  Rectifier(CanPort &can, uint8_t address, bool write_stored_defaults)
      : can_(can), address_(address), write_stored_defaults_(write_stored_defaults) {}

  // Call every main-loop pass.
  void loop(uint32_t now) {
    if (write_stored_defaults_ && online_ && (default_v_wrong_ || default_a_wrong_) &&
        (!defaults_written_ || now - last_defaults_write_ >= KEEPALIVE_MS)) {
      write_stored_defaults_now_(now);
    }
    if (now - last_setpoint_send_ >= KEEPALIVE_MS || (dirty_ && !setpoints_failed_)) {
      const bool ok = send_setpoints_();
      setpoints_failed_ = !ok;
      if (ok) {
        dirty_ = false;
        boot_sent_ = true;
      }
      last_setpoint_send_ = now;
    }
    if (in_flight_ && now - in_flight_since_ >= REQUEST_TIMEOUT_MS) in_flight_ = false;
    if (poll_pos_ >= POLL_COUNT && now - cycle_start_ >= POLL_CYCLE_MS) poll_pos_ = 0;
    if (!in_flight_ && poll_pos_ < POLL_COUNT && now - last_poll_send_ >= POLL_SPACING_MS) {
      const uint16_t reg = POLL_REGS[poll_pos_];
      const int i = live_index_(reg);
      if (i >= 0) polled_[i] = sent_[i];  // writes are queued ahead of this read
      if (send_(nasn::read_id(address_), nasn::read_request(reg))) {
        if (poll_pos_ == 0) cycle_start_ = now;
        poll_pos_++;
        in_flight_ = true;
        in_flight_reg_ = reg;
        in_flight_since_ = now;
      }
      last_poll_send_ = now;
    }
    if (online_ && now - last_reply_ > OFFLINE_MS) set_online_(false);

    const bool wrong = readback_wrong_[0] || readback_wrong_[1] || readback_wrong_[2];
    setpoint_mismatch_ = online_ && mismatch_timer_.update(wrong, now);
  }

  // Every received frame; anything not a reply from this module is ignored.
  void on_frame(const CanFrame &f, uint32_t now) {
    auto reply = nasn::parse_reply(f.id, true, f.data, f.len);
    if (!reply || reply->addr != address_) return;
    if (reply->cmd == nasn::CMD_WRITE) {
      consecutive_write_errors_ = reply->status != 0 ? consecutive_write_errors_ + 1 : 0;
      return;
    }
    if (in_flight_ && reply->reg == in_flight_reg_) in_flight_ = false;
    if (reply->status != 0) return;  // failed reads are not liveness
    last_reply_ = now;
    if (!online_) set_online_(true);
    switch (reply->reg) {
      case nasn::reg::VOUT:
        vout_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::VOUT_ALT:
        vout_alt_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::IOUT:
        iout_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::INPUT_VOLTAGE:
        vin_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::TEMP_HEATSINK:
        heatsink_c_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::ALARMS:
        alarms_ = nasn::alarm_word(reply->value);
        break;
      case nasn::reg::VOUT_DEFAULT:
        if (write_stored_defaults_)
          default_v_wrong_ = nasn::stored_default_mismatch(reply->reg, reply->value, nasn::voltage_raw(fallback_v_));
        break;
      case nasn::reg::ILIMIT_DEFAULT:
        if (write_stored_defaults_)
          default_a_wrong_ = nasn::stored_default_mismatch(reply->reg, reply->value, nasn::current_raw(fallback_a_));
        break;
      default:
        break;
    }
    const int i = live_index_(reply->reg);
    if (i >= 0 && polled_[i] >= 0)
      readback_wrong_[i] = nasn::readback_mismatch(reply->reg, reply->value, uint64_t(polled_[i]));
  }

  void set_target(float volts, float amps) {
    if (!std::isfinite(volts) || !std::isfinite(amps)) return;
    if (std::fabs(volts - target_v_) >= 0.01f || std::fabs(amps - target_a_) >= 0.1f) {
      target_v_ = volts;
      target_a_ = amps;
      dirty_ = true;
    }
  }

  void set_standby(bool standby) {
    if (standby != standby_) {
      standby_ = standby;
      dirty_ = true;
    }
  }

  // Stored defaults (0x0101 / 0x0104), clamped to <= 54.4 V and <= 10 A; rewritten when their readback differs.
  void set_fallback(float volts, float amps) {
    if (!std::isfinite(volts) || !std::isfinite(amps)) return;
    fallback_v_ = std::clamp(volts, nasn::MIN_V, FALLBACK_MAX_V);
    fallback_a_ = std::clamp(amps, 0.0f, FALLBACK_MAX_A);
  }

  bool is_online() const { return online_; }
  bool faulted() const {
    return online_ && ((alarms_ & nasn::STOCK_ALARM_MASK) != 0 || consecutive_write_errors_ >= 3 || setpoint_mismatch_);
  }
  bool setpoint_mismatch() const { return setpoint_mismatch_; }
  bool boot_sent() const { return boot_sent_; }
  float output_voltage() const { return nasn::output_voltage(vout_, vout_alt_); }
  float output_current() const { return iout_; }
  float input_voltage() const { return vin_; }
  float heatsink_temperature() const { return heatsink_c_; }
  uint32_t alarms() const { return alarms_; }
  uint32_t consecutive_write_errors() const { return consecutive_write_errors_; }
  uint8_t address() const { return address_; }
  bool standby() const { return standby_; }
  float target_current() const { return target_a_; }

 private:
  static constexpr uint16_t POLL_REGS[] = {
      nasn::reg::VOUT,         nasn::reg::VOUT_ALT,  nasn::reg::IOUT,          nasn::reg::INPUT_VOLTAGE,
      nasn::reg::TEMP_HEATSINK, nasn::reg::ALARMS,   nasn::reg::VOUT_SET,      nasn::reg::VOUT_READBACK,
      nasn::reg::ILIMIT,       nasn::reg::STANDBY,   nasn::reg::VOUT_DEFAULT,  nasn::reg::ILIMIT_DEFAULT,
  };
  static constexpr int POLL_COUNT = sizeof(POLL_REGS) / sizeof(POLL_REGS[0]);

  // Index into sent_/polled_/readback_wrong_: 0x022F (the voltage written to 0x0100), 0x022D, 0x0132.
  static int live_index_(uint16_t reg) {
    switch (reg) {
      case nasn::reg::VOUT_READBACK:
        return 0;
      case nasn::reg::ILIMIT:
        return 1;
      case nasn::reg::STANDBY:
        return 2;
      default:
        return -1;
    }
  }

  bool send_(uint32_t id, const nasn::Frame &frame) { return can_.send(id, frame.data()); }

  bool send_setpoints_() {
    const uint32_t id = nasn::write_id(address_);
    const uint16_t v = nasn::voltage_raw(target_v_), a = nasn::current_raw(target_a_);
    bool ok = send_(id, nasn::write_u16(nasn::reg::VOUT_SET, v));
    if (ok) sent_[0] = v;
    const bool ok_a = send_(id, nasn::write_u16(nasn::reg::ILIMIT, a));
    if (ok_a) sent_[1] = a;
    const bool ok_s = send_(id, nasn::write_standby(standby_));
    if (ok_s) sent_[2] = standby_ ? 1 : 0;
    return ok && ok_a && ok_s;
  }

  void write_stored_defaults_now_(uint32_t now) {
    const uint32_t id = nasn::write_id(address_);
    if (default_v_wrong_) send_(id, nasn::write_voltage(nasn::reg::VOUT_DEFAULT, fallback_v_));
    if (default_a_wrong_) send_(id, nasn::write_current(nasn::reg::ILIMIT_DEFAULT, fallback_a_));
    default_v_wrong_ = false;  // the next readback decides whether another write is needed
    default_a_wrong_ = false;
    defaults_written_ = true;
    last_defaults_write_ = now;
  }

  void set_online_(bool online) {
    online_ = online;
    if (online) {
      dirty_ = true;  // a module that just appeared needs our setpoints now
      return;
    }
    vout_ = vout_alt_ = iout_ = vin_ = heatsink_c_ = NAN;
    alarms_ = 0;
    consecutive_write_errors_ = 0;
    for (int i = 0; i < 3; i++) {
      readback_wrong_[i] = false;
      polled_[i] = -1;
    }
    mismatch_timer_.reset();
    setpoint_mismatch_ = false;
    default_v_wrong_ = default_a_wrong_ = false;
  }

  CanPort &can_;
  uint8_t address_;
  bool write_stored_defaults_;
  float fallback_v_ = 54.0f;
  float fallback_a_ = 5.0f;

  bool default_v_wrong_ = false, default_a_wrong_ = false, defaults_written_ = false;
  uint32_t last_defaults_write_ = 0;

  int32_t sent_[3] = {-1, -1, -1};
  int32_t polled_[3] = {-1, -1, -1};
  bool readback_wrong_[3] = {false, false, false};
  nasn::PersistTimer mismatch_timer_{nasn::MISMATCH_MS};
  bool setpoint_mismatch_ = false;

  int poll_pos_ = 0;
  uint32_t cycle_start_ = 0, last_poll_send_ = 0;
  bool in_flight_ = false;
  uint16_t in_flight_reg_ = 0;
  uint32_t in_flight_since_ = 0;

  float target_v_ = 54.0f, target_a_ = 0.0f;
  bool standby_ = true, dirty_ = true, setpoints_failed_ = false, boot_sent_ = false;
  uint32_t last_setpoint_send_ = 0;

  bool online_ = false;
  uint32_t last_reply_ = 0;
  float vout_ = NAN, vout_alt_ = NAN, iout_ = NAN, vin_ = NAN, heatsink_c_ = NAN;
  uint32_t alarms_ = 0, consecutive_write_errors_ = 0;
};

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_rectifier: ok`, and every other test `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/rectifier.h tests/fake_nasn_bus.h tests/test_oem_rectifier.cpp tests/Makefile
git commit -m "OEM Rectifier: port of the ESP NASN module logic behind a CanPort, one read in flight, host-tested on a fake bus"
```

---

### Task 8: `SessionGate` and the relay logic

**Files:**
- Create: `oem/src/app/session_gate.h`, `oem/src/app/relay.h`, `tests/test_oem_session.cpp`
- Modify: `tests/Makefile`

**Interfaces:**
- Produces:
  - `struct oem::GateThresholds { int start_soc, stop_soc; float start_v, stop_v; }`.
  - `struct oem::GateInputs { int soc; float battery_v; float output_a; bool charge_now; }` (soc -1 and battery_v NaN mean
    unknown).
  - `class oem::SessionGate { bool update(const GateInputs &, const GateThresholds &, uint32_t now); bool active() const; }`.
  - `enum class oem::RelayMode : uint8_t { AUTO, OFF, MODBUS }` and `RELAY_OFF_DELAY_MS = 10000`.
  - `class oem::RelayLogic { bool update(RelayMode, bool session_active, bool paused, bool faulted, bool modbus_cmd, uint32_t now); bool closed() const; }`.

The rules (spec §2.3 / §2.4):
- **Start:** (SOC known and SOC ≤ Start SOC) or V < Start V, held 5 s; or *charge now*. At the first update, the start
  condition alone starts the session, without the 5 s hold.
- **Stop:** (SOC known and SOC ≥ Stop SOC) or (V ≥ Stop V and output ≤ 4 A), held 10 s.
- **Relay:** decision D11.

- [ ] **Step 1: Write the failing test**

`tests/test_oem_session.cpp` (new file):

```cpp
#include <cmath>

#include "app/relay.h"
#include "app/session_gate.h"
#include "check.h"

using namespace oem;

static const GateThresholds T{20, 100, 42.0f, 57.6f};

static GateInputs volts(float v, float a = 0.0f) { return {-1, v, a, false}; }

static void test_boot_active_when_start_holds() {
  SessionGate g;
  CHECK(g.update(volts(41.0f), T, 0));  // at once, no 5 s wait
  SessionGate h;
  CHECK(!h.update(volts(50.0f), T, 0));
  SessionGate u;
  CHECK(!u.update(volts(NAN), T, 0));  // unknown voltage and SOC: no session
}

static void test_start_by_voltage_after_5s() {
  SessionGate g;
  g.update(volts(50.0f), T, 0);
  CHECK(!g.update(volts(41.9f), T, 1000));
  CHECK(!g.update(volts(41.9f), T, 5999));
  CHECK(!g.update(volts(42.5f), T, 6000));  // a blip above restarts the hold
  CHECK(!g.update(volts(41.9f), T, 7000));
  CHECK(g.update(volts(41.9f), T, 12000));
}

static void test_start_by_soc() {
  SessionGate g;
  g.update({50, 50.0f, 0.0f, false}, T, 0);
  CHECK(!g.update({20, 50.0f, 0.0f, false}, T, 1000));
  CHECK(g.update({20, 50.0f, 0.0f, false}, T, 6000));  // SOC <= Start SOC, held 5 s
}

static void test_stop_needs_voltage_and_tail_current() {
  SessionGate g;
  g.update(volts(41.0f), T, 0);
  CHECK(g.update(volts(57.6f, 10.0f), T, 1000));    // at Stop V but still taking 10 A
  CHECK(g.update(volts(57.6f, 10.0f), T, 20000));
  CHECK(g.update(volts(57.6f, 4.0f), T, 21000));    // tail current reached: start the 10 s hold
  CHECK(g.update(volts(57.6f, 4.0f), T, 30999));
  CHECK(!g.update(volts(57.6f, 3.0f), T, 31000));
}

static void test_stop_by_soc() {
  SessionGate g;
  g.update({10, NAN, 0.0f, false}, T, 0);
  CHECK(g.active());
  CHECK(g.update({100, NAN, 30.0f, false}, T, 1000));
  CHECK(!g.update({100, NAN, 30.0f, false}, T, 11000));  // SOC >= Stop SOC stops whatever the current
}

static void test_charge_now() {
  SessionGate g;
  g.update(volts(50.0f), T, 0);
  CHECK(g.update({-1, 50.0f, 0.0f, true}, T, 1000));  // ignores the start thresholds
  CHECK(g.update(volts(50.0f), T, 60000));            // and stays until a stop condition
  g.update(volts(57.7f, 0.0f), T, 61000);
  CHECK(!g.update(volts(57.7f, 0.0f), T, 71000));
}

static void test_relay_auto() {
  RelayLogic r;
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, 0));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 1000));
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 2000));   // session over: held 10 s
  CHECK(r.update(RelayMode::AUTO, false, false, false, false, 10999));
  CHECK(!r.update(RelayMode::AUTO, false, false, false, false, 11000));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 12000));
  CHECK(r.update(RelayMode::AUTO, true, true, false, false, 13000));    // paused: same 10 s delay
  CHECK(!r.update(RelayMode::AUTO, true, true, false, false, 22000));
  CHECK(r.update(RelayMode::AUTO, true, false, false, false, 23000));
  CHECK(!r.update(RelayMode::AUTO, true, false, true, false, 33000));   // fault
}

static void test_relay_off_and_modbus() {
  RelayLogic r;
  CHECK(!r.update(RelayMode::OFF, true, false, false, true, 0));
  CHECK(r.update(RelayMode::MODBUS, false, false, false, true, 1000));
  CHECK(!r.update(RelayMode::MODBUS, true, false, false, false, 2000));
}

int main() {
  test_boot_active_when_start_holds();
  test_start_by_voltage_after_5s();
  test_start_by_soc();
  test_stop_needs_voltage_and_tail_current();
  test_stop_by_soc();
  test_charge_now();
  test_relay_auto();
  test_relay_off_and_modbus();
  return test_result("test_oem_session");
}
```

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can \
  test_oem_adc test_oem_rectifier test_oem_session
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/relay.h' file not found`.

- [ ] **Step 3: Implement**

`oem/src/app/session_gate.h` (new file):

```cpp
// oem/src/app/session_gate.h: when charging is wanted (spec §2.3, stock semantics). The controller sees
// allowed = source.allowed && active().
#pragma once
#include <cmath>
#include <cstdint>

namespace oem {

constexpr uint32_t SESSION_START_HOLD_MS = 5000;
constexpr uint32_t SESSION_STOP_HOLD_MS = 10000;
constexpr float SESSION_STOP_MAX_A = 4.0f;  // stop by voltage only once the tail current is this low

struct GateThresholds {
  int start_soc, stop_soc;  // %
  float start_v, stop_v;    // V
};

struct GateInputs {
  int soc;           // %, -1 when unknown
  float battery_v;   // NaN when unknown
  float output_a;    // our total output
  bool charge_now;   // Switch click (or HR 113 in phase 3) since the last update
};

class SessionGate {
 public:
  bool update(const GateInputs &in, const GateThresholds &t, uint32_t now_ms) {
    const bool start = (in.soc >= 0 && in.soc <= t.start_soc) || in.battery_v < t.start_v;
    const bool stop =
        (in.soc >= 0 && in.soc >= t.stop_soc) || (in.battery_v >= t.stop_v && in.output_a <= SESSION_STOP_MAX_A);
    if (first_) {
      first_ = false;
      active_ = start;  // a battery that needs charging at power-up starts at once
    }
    if (in.charge_now) {
      active_ = true;
      stop_.reset();
    }
    if (active_) {
      start_.reset();
      if (stop_.held(stop, now_ms, SESSION_STOP_HOLD_MS)) {
        active_ = false;
        stop_.reset();
      }
    } else if (start_.held(start, now_ms, SESSION_START_HOLD_MS)) {
      active_ = true;
      start_.reset();
    }
    return active_;
  }

  bool active() const { return active_; }

 private:
  struct Hold {
    bool timing = false;
    uint32_t since = 0;
    bool held(bool cond, uint32_t now, uint32_t ms) {
      if (!cond) {
        timing = false;
        return false;
      }
      if (!timing) {
        timing = true;
        since = now;
      }
      return now - since >= ms;
    }
    void reset() { timing = false; }
  };

  bool first_ = true;
  bool active_ = false;
  Hold start_, stop_;
};

}  // namespace oem
```

`oem/src/app/relay.h` (new file):

```cpp
// oem/src/app/relay.h: the generator contact (spec §2.4). Auto: closed while a session is wanted, and it stays closed
// 10 s after that ends (session over, paused, or a fault). Missing AC doesn't open it: with no AC the modules go
// offline, which is no fault, and the generator is what brings AC back.
#pragma once
#include <cstdint>

namespace oem {

enum class RelayMode : uint8_t { AUTO = 0, OFF = 1, MODBUS = 2 };

constexpr uint32_t RELAY_OFF_DELAY_MS = 10000;

class RelayLogic {
 public:
  // modbus_cmd is HR 112 (phase 3; always false before then).
  bool update(RelayMode mode, bool session_active, bool paused, bool faulted, bool modbus_cmd, uint32_t now_ms) {
    if (mode == RelayMode::OFF) {
      closed_ = false;
    } else if (mode == RelayMode::MODBUS) {
      closed_ = modbus_cmd;
    } else if (session_active && !paused && !faulted) {
      closed_ = true;
      last_wanted_ms_ = now_ms;
    } else if (closed_ && now_ms - last_wanted_ms_ >= RELAY_OFF_DELAY_MS) {
      closed_ = false;
    }
    return closed_;
  }

  bool closed() const { return closed_; }

 private:
  bool closed_ = false;
  uint32_t last_wanted_ms_ = 0;
};

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_session: ok`, and every other test `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/session_gate.h oem/src/app/relay.h tests/test_oem_session.cpp tests/Makefile
git commit -m "OEM SessionGate (stock start/stop rules, charge now) and generator relay logic"
```

---

### Task 9: `Controller`: the 1 s charge tick

A port of `ChargeController::tick_` (`esphome/components/charge_controller/charge_controller.cpp`) on `charge_logic.h`.
- Unchanged from the ESP:
  - the totals over online modules;
  - `target_voltage`, `line_limited_max` (60 A below 180 V AC) and `current_budget`;
  - the `StateMachine` (30 s fault auto-clear, *clear fault*);
  - the 2 A/s `RampLimiter`;
  - `per_psu_current` (≤ 50 A each over the healthy modules);
  - the per-module `Cooldown` (48 V / 1 A while the heatsink cools from ≥ 45 °C to ≤ 40 °C);
  - `NoCurrentDetector`;
  - the stored defaults pushed to every module each tick.
- New:
  - `Limits` replaces the MQTT tracker;
  - `allowed = limits.allowed && session.active`;
  - `battery_v` is the source's value if known, else the ADC;
  - `battery_a` is unknown in Panel mode (D9);
  - spec §3 error codes replace fault strings;
  - Modbus-style hold-reason codes.

**Files:**
- Create: `oem/src/app/limit_source.h`, `oem/src/app/controller.h`, `tests/test_oem_controller.cpp`
- Modify: `tests/Makefile`

**Interfaces:**
- Consumes: `Rectifier` (Task 7), `SessionGate` (Task 8), `Settings`/`Hr` (Task 3), and `charge_logic.h` (Task 2).
- Produces:
  - `struct oem::Limits { float cvl, ccl; bool allowed, alarm, blocking; int soc; float battery_v, battery_a, max_cell_v; bool fresh; }`.
  - `oem::panel_limits(const Settings &) -> Limits` and `enum class oem::SourceMode { PANEL, PYLONTECH, MODBUS }`.
  - `enum class oem::HoldReason { NONE = 0, PAUSED, NOT_ALLOWED, BMS_ALARM, BMS_BLOCKING, CELL_HIGH, BUDGET, WAITING = 7 }`.
  - `oem::alarm_code(uint32_t alarm_word) -> uint8_t` (E1–E11, stock order) and `oem::is_fault_code(uint8_t)` (1–14).
  - `E_BMS_ALARM = 12`, `E_MISMATCH = 13`, `E_OVERVOLTAGE = 14`, `E_SOURCE_STALE = 15`, `E_NO_CURRENT = 16`, and
    `CONTROLLER_TICK_MS = 1000`.
  - `struct oem::Status`: `state`, `hold`, `error`, `session_active`, `paused`, `cooling`, `no_current`, `online`, `soc`,
    `target_v`, `commanded_a`, `budget`, `output_a`, `power_w`, `battery_v` and `ac_v`.
  - `class oem::Controller(Rectifier &, Rectifier &)`:
    - `tick(const Settings &, const Limits &, float adc_v, uint32_t now) -> const Status &`;
    - `charge_now()`, `toggle_pause()`, `set_paused(bool)`, `clear_fault()`;
    - `status()`.

- [ ] **Step 1: Write the failing test.** It runs the real `Rectifier`s against `FakeModule`s, ticking the controller every second.

`tests/test_oem_controller.cpp` (new file):

```cpp
#include <cmath>

#include "app/controller.h"
#include "check.h"
#include "fake_nasn_bus.h"

using namespace oem;
using charge::State;

struct Rig {
  FakeBus bus;
  FakeModule m1{1}, m2{2};
  Rectifier r1{bus, 1, true}, r2{bus, 2, true};
  Controller ctl{r1, r2};
  Settings s = default_settings();
  float adc_v = 50.0f;  // between Start V (42.0) and Stop V (57.6): no session until something starts one
  uint32_t now = 1000;
  Status st;

  void run_s(int seconds) {
    for (int t = 0; t < seconds * 1000; t++, now++) {
      r1.loop(now);
      r2.loop(now);
      bus.deliver({&m1, &m2}, {&r1, &r2}, now);
      if (now % CONTROLLER_TICK_MS == 0) st = ctl.tick(s, panel_limits(s), adc_v, now);
    }
  }
};

static void test_alarm_codes() {
  CHECK(alarm_code(0) == 0);
  CHECK(alarm_code(1u << 22) == 1);
  CHECK(alarm_code(1u << 17) == 2);
  CHECK(alarm_code(1u << 0) == 8);
  CHECK(alarm_code(1u << 3) == 11 && alarm_code(1u << 2) == 11);
  CHECK(alarm_code((1u << 22) | 1u) == 1);  // first match wins
  CHECK(is_fault_code(14) && !is_fault_code(15) && !is_fault_code(0));
}

static void test_waits_for_start_then_charge_now() {
  Rig g;
  g.run_s(5);
  CHECK(g.st.state == State::HOLD && g.st.hold == HoldReason::WAITING && !g.st.session_active);
  CHECK(g.st.online == 2 && g.m1.standby == 1 && g.m2.standby == 1);
  CHECK_NEAR(g.st.ac_v, 230.0, 0.01);
  g.ctl.charge_now();
  g.run_s(1);
  CHECK(g.st.state == State::CHARGING && g.st.session_active);
  g.run_s(1);
  CHECK(g.m1.standby == 0 && g.m2.standby == 0);
  CHECK_NEAR(g.m1.vout_set, 57.6, 0.01);
  CHECK(g.st.commanded_a <= 4.01f);  // ramped at 2 A/s
  g.run_s(10);
  CHECK_NEAR(g.st.commanded_a, 10.0, 0.01);  // Max current 10 A, split 5 + 5
  CHECK_NEAR(g.m1.ilimit, 5.0, 0.1);
  CHECK_NEAR(g.m2.ilimit, 5.0, 0.1);
  CHECK_NEAR(g.st.output_a, 10.0, 0.1);
  CHECK(g.st.error == 0);
}

static void test_low_battery_starts_session_at_boot() {
  Rig g;
  g.adc_v = 41.0f;
  g.run_s(3);
  CHECK(g.st.session_active && g.st.state == State::CHARGING);
}

static void test_split_cap_and_low_line() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 100);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK_NEAR(g.m1.ilimit, 50.0, 0.1);  // 100 A over two modules, <= 50 A each
  CHECK_NEAR(g.m2.ilimit, 50.0, 0.1);
  g.m1.vin = 175.0f;  // low line: 60 A total
  g.run_s(3);
  CHECK_NEAR(g.st.commanded_a, 60.0, 0.01);
  CHECK_NEAR(g.m1.ilimit, 30.0, 0.1);
}

static void test_one_module_takes_the_budget() {
  Rig g;
  g.m2.muted = true;
  g.s.set(HR_MAX_CURRENT, 80);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.online == 1);
  CHECK_NEAR(g.m1.ilimit, 50.0, 0.1);  // capped per module
  CHECK_NEAR(g.st.commanded_a, 50.0, 0.01);
}

static void test_alarm_fault_and_auto_clear() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(10);
  g.m1.alarm = 1u << 22;
  g.run_s(3);
  CHECK(g.st.state == State::FAULT && g.st.error == 1);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
  g.m1.alarm = 0;
  g.run_s(20);
  CHECK(g.st.state == State::FAULT && g.st.error == 1);  // 30 s after the condition is gone
  g.run_s(15);
  CHECK(g.st.state == State::CHARGING && g.st.error == 0);
}

static void test_clear_fault_at_once() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(5);
  g.m2.alarm = 1u;  // E8
  g.run_s(3);
  CHECK(g.st.error == 8);
  g.m2.alarm = 0;
  g.run_s(3);
  g.ctl.clear_fault();
  g.run_s(1);
  CHECK(g.st.state == State::CHARGING && g.st.error == 0);
}

static void test_overvoltage_e14() {
  Rig g;
  g.m1.battery_v = 59.0f;  // a module reading above HARD_MAX_V + 0.5 V faults whatever the state
  g.run_s(4);
  CHECK(g.st.state == State::FAULT && g.st.error == E_OVERVOLTAGE);
}

static void test_mismatch_e13() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(3);
  g.m2.frozen = true;
  g.s.set(HR_CHARGE_V, 5600);  // module 2 ignores the new voltage
  g.run_s(15);
  CHECK(g.st.state == State::FAULT && g.st.error == E_MISMATCH);
}

static void test_cooldown_before_standby() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(10);
  g.m1.heatsink_c = 50.0f;
  g.run_s(2);
  g.ctl.toggle_pause();
  g.run_s(3);
  CHECK(g.st.state == State::HOLD && g.st.hold == HoldReason::PAUSED);
  CHECK(g.st.cooling);
  CHECK(g.m1.standby == 0 && g.m2.standby == 1);  // module 1 keeps its fans running
  CHECK_NEAR(g.m1.vout_set, 48.0, 0.01);
  CHECK_NEAR(g.m1.ilimit, 1.0, 0.1);
  g.m1.heatsink_c = 39.0f;
  g.run_s(3);
  CHECK(!g.st.cooling && g.m1.standby == 1);
}

static void test_no_current_warning_e16() {
  Rig g;
  g.ctl.charge_now();
  g.run_s(10);
  g.m1.load_connected = g.m2.load_connected = false;  // DC breaker open
  g.run_s(55);
  CHECK(g.st.error == 0);
  g.run_s(10);
  CHECK(g.st.no_current && g.st.error == E_NO_CURRENT && g.st.state == State::CHARGING);
  g.m1.load_connected = g.m2.load_connected = true;
  g.run_s(3);
  CHECK(!g.st.no_current && g.st.error == 0);
}

static void test_session_stops_at_stop_v_with_tail_current() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 4);
  g.ctl.charge_now();
  g.run_s(5);
  g.adc_v = 57.6f;
  g.run_s(12);
  CHECK(!g.st.session_active && g.st.state == State::HOLD && g.st.hold == HoldReason::WAITING);
}

static void test_stored_defaults_follow_settings() {
  Rig g;
  g.s.set(HR_MODULE_FALLBACK_V, 5200);
  g.s.set(HR_MODULE_FALLBACK_A, 3);
  g.run_s(8);
  CHECK_NEAR(g.m1.default_v, 52.0, 0.01);
  CHECK_NEAR(g.m2.default_a, 3.0, 0.1);
}

int main() {
  test_alarm_codes();
  test_waits_for_start_then_charge_now();
  test_low_battery_starts_session_at_boot();
  test_split_cap_and_low_line();
  test_one_module_takes_the_budget();
  test_alarm_fault_and_auto_clear();
  test_clear_fault_at_once();
  test_overvoltage_e14();
  test_mismatch_e13();
  test_cooldown_before_standby();
  test_no_current_warning_e16();
  test_session_stops_at_stop_v_with_tail_current();
  test_stored_defaults_follow_settings();
  return test_result("test_oem_controller");
}
```

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can \
  test_oem_adc test_oem_rectifier test_oem_session test_oem_controller
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/controller.h' file not found`.

- [ ] **Step 3: Implement**

`oem/src/app/limit_source.h` (new file):

```cpp
// oem/src/app/limit_source.h: the common Limits struct every source produces (spec §2.1). Phase 2 has the Panel
// source only; Pylontech and Modbus (phase 3) fill the optional fields and freshness.
#pragma once
#include <cmath>

#include "app/settings.h"

namespace oem {

enum class SourceMode : uint8_t { PANEL = 0, PYLONTECH = 1, MODBUS = 2 };

struct Limits {
  float cvl;         // V
  float ccl;         // A, total
  bool allowed;      // the source allows charging
  bool alarm;
  bool blocking;
  int soc;           // %, -1 when unknown
  float battery_v;   // NaN when unknown
  float battery_a;   // NaN when unknown; positive = charging
  float max_cell_v;  // NaN when unknown
  bool fresh;
};

// Panel: CVL = Charge V, CCL = Max current, allowed, always fresh, nothing else known.
inline Limits panel_limits(const Settings &s) {
  return {float(s.get(HR_CHARGE_V)) / 100.0f, float(s.get(HR_MAX_CURRENT)), true, false, false, -1,
          NAN, NAN, NAN, true};
}

}  // namespace oem
```

`oem/src/app/controller.h` (new file):

```cpp
// oem/src/app/controller.h: the 1 s charge-control tick. A port of the ESP ChargeController::tick_
// (esphome/components/charge_controller) on charge_logic.h, with the source's Limits instead of YamBMS over MQTT, the
// session gate (spec §2.3) folded into "allowed", and spec §3 error codes instead of fault strings.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "app/limit_source.h"
#include "app/rectifier.h"
#include "app/session_gate.h"
#include "app/settings.h"
#include "charge_controller/charge_logic.h"

namespace oem {

enum class HoldReason : uint8_t {  // Modbus IR 2
  NONE = 0, PAUSED = 1, NOT_ALLOWED = 2, BMS_ALARM = 3, BMS_BLOCKING = 4, CELL_HIGH = 5, BUDGET = 6, WAITING = 7,
};

constexpr uint8_t E_BMS_ALARM = 12, E_MISMATCH = 13, E_OVERVOLTAGE = 14, E_SOURCE_STALE = 15, E_NO_CURRENT = 16;
constexpr uint32_t CONTROLLER_TICK_MS = 1000;

// E1-E11 from the 0x0183 alarm word, first match wins, as stock 08006d68 (docs/history/notes/findings-v2.md §5). E7 is unused.
inline uint8_t alarm_code(uint32_t a) {
  static constexpr struct {
    uint32_t bits;
    uint8_t code;
  } MAP[] = {{1u << 22, 1}, {1u << 17, 2}, {1u << 12, 3}, {1u << 4, 4},  {1u << 14, 5},
             {1u << 27, 6}, {1u << 0, 8},  {1u << 1, 9},  {1u << 20, 10}, {3u << 2, 11}};
  for (const auto &m : MAP) {
    if (a & m.bits) return m.code;
  }
  return 0;
}

inline bool is_fault_code(uint8_t e) { return e >= 1 && e <= 14; }

struct Status {
  charge::State state = charge::State::BOOT;
  HoldReason hold = HoldReason::NONE;
  uint8_t error = 0;          // 0 or E1-E16; faults take precedence over warnings
  bool session_active = false;
  bool paused = false;
  bool cooling = false;
  bool no_current = false;
  int online = 0;             // modules online
  int soc = -1;
  float target_v = NAN;       // while charging
  float commanded_a = 0.0f;   // total
  float budget = 0.0f;
  float output_a = 0.0f;      // measured, total
  float power_w = 0.0f;
  float battery_v = NAN;      // in use: the source's when known, else the ADC
  float ac_v = NAN;           // highest AC input reading of the online modules
};

class Controller {
 public:
  Controller(Rectifier &a, Rectifier &b) : rects_{&a, &b} {}

  void charge_now() { charge_now_ = true; }
  void toggle_pause() { paused_ = !paused_; }
  void set_paused(bool p) { paused_ = p; }
  void clear_fault() { clear_fault_ = true; }
  const Status &status() const { return st_; }

  // Every CONTROLLER_TICK_MS.
  const Status &tick(const Settings &s, const Limits &lim, float adc_v, uint32_t now) {
    const float fb_v = float(s.get(HR_MODULE_FALLBACK_V)) / 100.0f, fb_a = float(s.get(HR_MODULE_FALLBACK_A));
    float our_a = 0.0f, power = 0.0f, max_vout = NAN, min_vin = NAN, max_vin = NAN;
    int healthy = 0, online = 0;
    bool boot_done = true;
    for (Rectifier *r : rects_) {
      r->set_fallback(fb_v, fb_a);
      boot_done = boot_done && r->boot_sent();
      if (!r->is_online()) continue;
      online++;
      const float v = r->output_voltage(), i = r->output_current(), vin = r->input_voltage();
      if (std::isfinite(i)) our_a += i;
      if (std::isfinite(v) && std::isfinite(i)) power += v * i;
      if (std::isfinite(v)) max_vout = std::isfinite(max_vout) ? std::max(max_vout, v) : v;
      if (std::isfinite(vin)) {
        min_vin = std::isfinite(min_vin) ? std::min(min_vin, vin) : vin;
        max_vin = std::isfinite(max_vin) ? std::max(max_vin, vin) : vin;
      }
      if (!r->faulted()) healthy++;
    }

    const float battery_v = std::isfinite(lim.battery_v) ? lim.battery_v : adc_v;
    const GateThresholds gt{int(s.get(HR_START_SOC)), int(s.get(HR_STOP_SOC)), float(s.get(HR_START_V)) / 100.0f,
                            float(s.get(HR_STOP_V)) / 100.0f};
    const bool session = gate_.update({lim.soc, battery_v, our_a, charge_now_}, gt, now);
    charge_now_ = false;

    const float target_v = charge::target_voltage(lim.cvl, float(s.get(HR_CHARGE_V)) / 100.0f);
    const float max_a = charge::line_limited_max(float(s.get(HR_MAX_CURRENT)), min_vin);
    // Unknown battery current: assume nothing but us charges the battery.
    const float battery_a = std::isfinite(lim.battery_a) ? lim.battery_a : our_a;
    const float budget = charge::current_budget(lim.ccl, battery_a, our_a, max_a);

    charge::Inputs in;
    in.boot_done = boot_done;
    in.bms_fresh = lim.fresh;
    in.allowed = lim.allowed && session;
    in.bms_alarm = lim.alarm;
    in.paused = paused_;
    in.bms_blocking = lim.blocking;
    const charge::CellThresholds cell =
        charge::cell_thresholds(float(s.get(HR_MAX_CELL_MV)) / 1000.0f, float(s.get(HR_CELL_RESUME_MV)) / 1000.0f);
    in.cell_high = cell_.update(lim.max_cell_v, cell.limit, cell.resume);
    in.budget = budget;
    uint8_t fault_code = 0;
    in.fault = check_faults_(target_v, max_vout, our_a, fault_code);
    if (in.fault != nullptr) last_fault_code_ = fault_code;
    in.clear_fault = clear_fault_;
    clear_fault_ = false;
    const charge::State state = sm_.update(in, now);

    const bool charging = state == charge::State::CHARGING;
    float total = 0.0f;
    if (charging && healthy > 0) {
      total = ramp_.step(budget, now);
    } else {
      ramp_.reset();
    }
    const float per_psu = charge::per_psu_current(total, healthy);
    const float commanded = charging ? per_psu * float(healthy) : 0.0f;
    int cooling = 0;
    for (int k = 0; k < 2; k++) {
      Rectifier *r = rects_[k];
      const bool run = charging && r->is_online() && !r->faulted();
      const bool cool_ok = r->is_online() && !r->faulted() && state != charge::State::FAULT &&
                           r->output_voltage() >= charge::COOLDOWN_MIN_BATTERY_V;
      if (cooldown_[k].update(run, cool_ok, r->heatsink_temperature(), now)) {
        cooling++;
        r->set_target(charge::COOLDOWN_V, charge::COOLDOWN_A);
        r->set_standby(false);
      } else {
        r->set_target(std::isfinite(target_v) ? target_v : fb_v, run ? per_psu : 0.0f);
        r->set_standby(!run);
      }
    }

    const bool no_current = no_current_.update(
        charge::no_current_condition(charging, commanded, our_a, max_vout, target_v, battery_v), now);

    st_.state = state;
    st_.hold = state == charge::State::HOLD ? hold_reason_(in, lim, session) : HoldReason::NONE;
    if (state == charge::State::FAULT) {
      st_.error = last_fault_code_;
    } else if (lim.alarm && lim.fresh) {
      st_.error = E_BMS_ALARM;
    } else if (!lim.fresh) {
      st_.error = E_SOURCE_STALE;
    } else {
      st_.error = no_current ? E_NO_CURRENT : 0;
    }
    st_.session_active = session;
    st_.paused = paused_;
    st_.cooling = cooling > 0;
    st_.no_current = no_current;
    st_.online = online;
    st_.soc = lim.soc;
    st_.target_v = charging ? target_v : NAN;
    st_.commanded_a = commanded;
    st_.budget = budget;
    st_.output_a = our_a;
    st_.power_w = power;
    st_.battery_v = battery_v;
    st_.ac_v = max_vin;
    return st_;
  }

 private:
  // A faulted module (alarm bits: E1-E11; setpoints not followed or writes rejected: E13), then overvoltage (E14).
  const char *check_faults_(float target_v, float max_vout, float our_a, uint8_t &code) {
    for (Rectifier *r : rects_) {
      if (!r->faulted()) continue;
      code = alarm_code(r->alarms() & nasn::STOCK_ALARM_MASK);
      if (code != 0) return "PSU alarm";
      code = E_MISMATCH;
      return "PSU not following setpoints";
    }
    if (charge::overvoltage(sm_.state() == charge::State::CHARGING, max_vout, target_v, our_a)) {
      code = E_OVERVOLTAGE;
      return "Overvoltage";
    }
    return nullptr;
  }

  // charge::hold_reason's order, with "waiting for start" when it's the session, not the source, that says no.
  static HoldReason hold_reason_(const charge::Inputs &in, const Limits &lim, bool session) {
    if (in.paused) return HoldReason::PAUSED;
    if (!session) return HoldReason::WAITING;
    if (!lim.allowed) return HoldReason::NOT_ALLOWED;
    if (in.bms_alarm) return HoldReason::BMS_ALARM;
    if (in.bms_blocking) return HoldReason::BMS_BLOCKING;
    if (in.cell_high) return HoldReason::CELL_HIGH;
    if (!(in.budget >= charge::START_BUDGET_A)) return HoldReason::BUDGET;
    return HoldReason::NONE;
  }

  Rectifier *rects_[2];
  SessionGate gate_;
  charge::StateMachine sm_;
  charge::RampLimiter ramp_;
  charge::CellLimiter cell_;
  charge::Cooldown cooldown_[2];
  charge::NoCurrentDetector no_current_;
  Status st_;
  uint8_t last_fault_code_ = 0;
  bool charge_now_ = false, paused_ = false, clear_fault_ = false;
};

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_controller: ok`, and every other test `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/limit_source.h oem/src/app/controller.h tests/test_oem_controller.cpp tests/Makefile
git commit -m "OEM Controller: port of the ESP charge tick with Panel limits, session gate, E1-E16 codes, cooldown, no-current"
```

---

### Task 10: UI: keys, main screen, menu, LEDs, backlight

**Files:**
- Create: `oem/src/app/ui.h`, `tests/test_oem_ui.cpp`
- Modify: `tests/Makefile`

**Interfaces:**
- Consumes: `oem::Key` (phase 1), the `put_*` functions (Task 1), `Settings` (Task 3), and `Status`, `is_fault_code` (Task 9).
- Produces:
  - `enum oem::KeyId { KEY_SWITCH, KEY_UP, KEY_DOWN, KEY_ENTER }`.
  - `struct oem::UiActions { bool charge_now, toggle_pause, settings_changed; }`.
  - `oem::MENU[8]` and `oem::menu_range(const Settings &, int item, int32_t &lo, int32_t &hi)`.
  - `struct oem::Leds { bool fault, charge; }` and `oem::leds(const Status &, uint32_t now) -> Leds`.
  - `class oem::Ui`:
    - `on_keys(const bool raw[4], Settings &live, uint32_t now) -> UiActions`, every 10 ms;
    - `render(const Status &, uint32_t now) -> Frame`, every 200 ms;
    - `backlight(const Settings &, uint32_t now) -> bool`;
    - `held_ticks(KeyId)`, `in_menu()`, `menu_item()` and `editing()`.

The behaviour (spec §3 and `docs/history/notes/oem-init-sequences.md` §4c):
- **Main screen:**
  - AC V is the highest module reading, or `---`. Current and W are rounded. Battery V is in 0.1 V with a leading zero
    (`0.5`).
  - The SOC field shows SOC or `---`, alternating 1 s / 1 s with `Exx` while there is an error.
  - Bars follow D13. Icons on SEG28: COM7 error, COM6 idle, COM5 charging.
- **Keys:**
  - Switch: click = charge now; held 2 s = pause toggle.
  - Enter: click enters the menu and steps through it; after item 8 the menu exits and saves. Long Enter exits and saves.
  - Up/Down: click or repeat changes the value, within `menu_range`, with `apply_clamps` after each step.
  - The menu also exits after 30 s idle.
  - A key pressed while the backlight is off only wakes it.
- **Menu indicators:**
  - SEG28 COM4 for item 1 and SEG28 COM3 for item 2.
  - SEG25 COM7, COM6, COM5 and COM4 for items 3 to 6.
  - Items 7 and 8 show their number in the AC field.
- **LEDs (active-low pins, handled in main):**
  - Fault: solid for E1–E14, blinking at 500 ms for E15 and E16.
  - Charge, as stock: needs a module online and no error. Solid while charging with current, a 100 ms blink while charging
    without current, a 500 ms blink when idle.

- [ ] **Step 1: Write the failing test**

`tests/test_oem_ui.cpp` (new file):

```cpp
#include <initializer_list>

#include "app/ui.h"
#include "check.h"

using namespace oem;
using charge::State;

struct Rig {
  Ui ui;
  Settings live = default_settings();
  uint32_t now = 10000;
  UiActions last;
  int clicks_now = 0, pauses = 0, saves = 0;

  // Hold `key` (or none, -1) for ms, ticking every 10 ms.
  void hold(int key, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10, now += 10) {
      bool raw[4] = {false, false, false, false};
      if (key >= 0) raw[key] = true;
      last = ui.on_keys(raw, live, now);
      clicks_now += last.charge_now;
      pauses += last.toggle_pause;
      saves += last.settings_changed;
    }
  }
  void click(int key) {
    hold(key, 100);
    hold(-1, 50);
  }
};

static bool seg(const Frame &f, int s, int com) { return f.com[com] & (1u << s); }

static bool same_field(const Frame &a, const Frame &b, Field field) {
  for (int p = 0; p < FIELDS[int(field)].digits; p++) {
    const int s = FIELDS[int(field)].segs[p];
    for (int k = 1; k < 8; k++) {
      if (seg(a, s, k) != seg(b, s, k)) return false;
    }
  }
  return true;
}

static Status charging_status() {
  Status st;
  st.state = State::CHARGING;
  st.online = 2;
  st.ac_v = 231.4f;
  st.output_a = 10.4f;
  st.power_w = 560.2f;
  st.battery_v = 52.34f;
  return st;
}

static void test_main_screen_fields() {
  Ui ui;
  const Frame f = ui.render(charging_status(), 2000);
  Frame want;
  put_number(want, Field::AC_V, 231);
  put_number(want, Field::CURRENT, 10);
  put_number(want, Field::POWER, 560);
  put_number(want, Field::BATTERY_V, 523, 2);
  put_dashes(want, Field::SOC);  // no SOC in Panel mode
  for (Field fl : {Field::AC_V, Field::CURRENT, Field::POWER, Field::BATTERY_V, Field::SOC})
    CHECK(same_field(f, want, fl));
  CHECK(seg(f, 28, 5) && !seg(f, 28, 6) && !seg(f, 28, 7));  // charging icon, no idle or error icon
  CHECK(seg(f, 28, 0) && seg(f, 28, 1));                     // fixed labels
  Status idle;
  idle.battery_v = 0.5f;
  const Frame g = ui.render(idle, 2000);
  Frame want_g;
  put_dashes(want_g, Field::AC_V);  // no module online: no AC reading
  put_number(want_g, Field::BATTERY_V, 5, 2);
  CHECK(same_field(g, want_g, Field::AC_V) && same_field(g, want_g, Field::BATTERY_V));
  CHECK(seg(g, 28, 6) && !seg(g, 28, 5));
}

static void test_error_alternates_with_soc() {
  Ui ui;
  Status st = charging_status();
  st.state = State::FAULT;
  st.error = 13;
  const Frame even = ui.render(st, 4000), odd = ui.render(st, 5000);
  Frame dashes, err;
  put_dashes(dashes, Field::SOC);
  put_error(err, 13);
  CHECK(same_field(even, dashes, Field::SOC) && same_field(odd, err, Field::SOC));
  CHECK(seg(even, 28, 7) && seg(odd, 28, 7));
}

static void test_bars_sweep_while_charging() {
  Ui ui;
  auto bars = [](const Frame &f) {
    int n = 0;
    for (int k = 0; k < 8; k++) n += seg(f, 16, k);
    return n;
  };
  int max_bars = 0, min_bars = 8;
  for (uint32_t t = 0; t < 6000; t += 200) {  // one step per 300 ms: 0..11, then 0 again
    const int n = bars(ui.render(charging_status(), 10000 + t));
    if (n > max_bars) max_bars = n;
    if (n < min_bars) min_bars = n;
  }
  CHECK(max_bars == 8 && min_bars == 0);
  Status idle;
  CHECK(bars(ui.render(idle, 20000)) == 0);
}

static void test_switch_click_and_pause_hold() {
  Rig g;
  g.click(KEY_SWITCH);
  CHECK(g.clicks_now == 1 && g.pauses == 0);
  g.hold(KEY_SWITCH, 2500);
  CHECK(g.ui.held_ticks(KEY_SWITCH) >= 240);
  g.hold(-1, 50);
  CHECK(g.ui.held_ticks(KEY_SWITCH) == 0);
  CHECK(g.pauses == 1 && g.clicks_now == 1);  // a hold is not a click
}

static void test_menu_edit_and_save() {
  Rig g;
  g.click(KEY_ENTER);
  CHECK(g.ui.in_menu() && g.ui.menu_item() == 0);
  for (int i = 0; i < 3; i++) g.click(KEY_UP);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 13 && g.live.get(HR_MAX_CURRENT) == 10);  // live only on exit
  g.click(KEY_ENTER);  // item 2: Charge V
  g.click(KEY_DOWN);
  CHECK(g.ui.editing().get(HR_CHARGE_V) == 5750);
  CHECK(g.ui.editing().get(HR_STOP_V) == 5750);  // Stop V follows it down
  for (int i = 0; i < 6; i++) g.click(KEY_ENTER);
  CHECK(g.ui.in_menu() && g.ui.menu_item() == 7 && g.saves == 0);
  g.click(KEY_ENTER);  // past item 8: exit and save
  CHECK(!g.ui.in_menu() && g.saves == 1);
  CHECK(g.live.get(HR_MAX_CURRENT) == 13 && g.live.get(HR_CHARGE_V) == 5750 && g.live.get(HR_STOP_V) == 5750);
  g.click(KEY_ENTER);
  g.click(KEY_ENTER);
  g.hold(KEY_ENTER, 1100);  // long Enter: exit; nothing changed, so nothing to save
  g.hold(-1, 50);
  CHECK(!g.ui.in_menu() && g.saves == 1);
}

static void test_menu_ranges_and_repeat() {
  Rig g;
  g.click(KEY_ENTER);
  g.hold(KEY_UP, 30000);  // auto-repeat up to the top of the range
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 100);
  for (int i = 0; i < 5; i++) g.click(KEY_ENTER);  // item 6: Stop V, at most Charge V
  CHECK(g.ui.menu_item() == 5);
  g.click(KEY_UP);
  CHECK(g.ui.editing().get(HR_STOP_V) == 5760);
  g.click(KEY_ENTER);  // item 7: source mode 0-2
  g.click(KEY_DOWN);
  CHECK(g.ui.editing().get(HR_SOURCE_MODE) == 0);
}

static void test_menu_idle_exit() {
  Rig g;
  g.click(KEY_ENTER);
  g.click(KEY_UP);
  g.hold(-1, 29000);
  CHECK(g.ui.in_menu());
  g.hold(-1, 1500);
  CHECK(!g.ui.in_menu() && g.saves == 1 && g.live.get(HR_MAX_CURRENT) == 11);
}

static void test_menu_render() {
  Rig g;
  g.click(KEY_ENTER);
  const Status st;
  const Frame off = g.ui.render(st, 600), on = g.ui.render(st, 900);
  Frame want;
  put_number(want, Field::CURRENT, 10);
  CHECK(same_field(on, want, Field::CURRENT) && seg(on, 28, 4));  // value and the item-1 indicator
  CHECK(!seg(off, 28, 4) && !seg(off, 22, 2));                   // both blank in the "off" half
  for (int i = 0; i < 6; i++) g.click(KEY_ENTER);                 // item 7: number in the AC field
  Frame seven;
  put_number(seven, Field::AC_V, 7);
  CHECK(same_field(g.ui.render(st, 600), seven, Field::AC_V));
}

static void test_wake_press_does_nothing_else() {
  Rig g;
  CHECK(g.ui.backlight(g.live, g.now));  // on after power-up
  g.hold(-1, 121000);
  CHECK(!g.ui.backlight(g.live, g.now));
  g.click(KEY_SWITCH);
  CHECK(g.clicks_now == 0 && g.ui.backlight(g.live, g.now));
  g.click(KEY_SWITCH);
  CHECK(g.clicks_now == 1);
  g.live.set(HR_BACKLIGHT_S, 0);
  g.hold(-1, 500000);
  CHECK(g.ui.backlight(g.live, g.now));
}

static void test_leds() {
  Status st = charging_status();
  CHECK(leds(st, 0).charge && leds(st, 100).charge && !leds(st, 0).fault);  // solid while delivering
  st.output_a = 0.0f;
  CHECK(leds(st, 0).charge != leds(st, 100).charge);                        // 100 ms blink without current
  st.state = State::HOLD;
  CHECK(leds(st, 0).charge != leds(st, 500).charge && leds(st, 0).charge == leds(st, 100).charge);
  st.error = 4;
  CHECK(leds(st, 0).fault && leds(st, 500).fault && !leds(st, 0).charge && !leds(st, 500).charge);
  st.error = 16;
  CHECK(leds(st, 0).fault != leds(st, 500).fault);  // warnings blink
  st.error = 0;
  st.online = 0;
  CHECK(!leds(st, 0).charge && !leds(st, 500).charge);
}

int main() {
  test_main_screen_fields();
  test_error_alternates_with_soc();
  test_bars_sweep_while_charging();
  test_switch_click_and_pause_hold();
  test_menu_edit_and_save();
  test_menu_ranges_and_repeat();
  test_menu_idle_exit();
  test_menu_render();
  test_wake_press_does_nothing_else();
  test_leds();
  return test_result("test_oem_ui");
}
```

`tests/Makefile` (whole file):

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys test_oem_lcd_render test_oem_settings test_oem_can \
  test_oem_adc test_oem_rectifier test_oem_session test_oem_controller test_oem_ui
HEADERS = $(wildcard *.h ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h \
  ../oem/src/app/*.h ../oem/src/hw/*_bits.h)

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

test_%: test_%.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -o $@ $<

clean:
	rm -f $(TESTS)

.PHONY: test clean
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `'app/ui.h' file not found`.

- [ ] **Step 3: Implement**

`oem/src/app/ui.h` (new file):

```cpp
// oem/src/app/ui.h: keys, the main screen, the 8-item menu, LEDs and backlight (spec §3). The layout, menu
// indicators, blink and bar animation follow the stock render (docs/history/notes/oem-init-sequences.md §4c).
#pragma once
#include <cmath>
#include <cstdint>

#include "app/controller.h"
#include "app/keys.h"
#include "app/lcd_render.h"
#include "app/settings.h"

namespace oem {

enum KeyId : uint8_t { KEY_SWITCH = 0, KEY_UP = 1, KEY_DOWN = 2, KEY_ENTER = 3 };

constexpr uint16_t PAUSE_HOLD_TICKS = 200;  // Switch held 2 s
constexpr uint32_t MENU_IDLE_MS = 30000;
constexpr uint32_t BLINK_MS = 300;          // edited value: 300 ms off, 300 ms on
constexpr uint32_t BAR_STEP_MS = 300;
constexpr int MENU_ITEMS = 8;

struct UiActions {
  bool charge_now = false;
  bool toggle_pause = false;
  bool settings_changed = false;  // the menu closed with new values (already applied to the live settings)
};

struct MenuItem {
  Hr reg;
  Field field;
  int32_t step;
  int32_t div;     // shown value = setting / div
  int8_t ind_seg;  // menu indicator, -1 for none
  int8_t ind_com;
};

// Items 7 and 8 have no indicator on the stock glass; the AC field shows their number instead.
constexpr MenuItem MENU[MENU_ITEMS] = {
    {HR_MAX_CURRENT, Field::CURRENT, 1, 1, 28, 4},  {HR_CHARGE_V, Field::BATTERY_V, 10, 10, 28, 3},
    {HR_START_SOC, Field::SOC, 1, 1, 25, 7},        {HR_STOP_SOC, Field::SOC, 1, 1, 25, 6},
    {HR_START_V, Field::BATTERY_V, 10, 10, 25, 5},  {HR_STOP_V, Field::BATTERY_V, 10, 10, 25, 4},
    {HR_SOURCE_MODE, Field::SOC, 1, 1, -1, 0},      {HR_MODBUS_ADDR, Field::SOC, 1, 1, -1, 0},
};

// The editable range of menu item i, given the other settings (spec §3).
inline void menu_range(const Settings &s, int i, int32_t &lo, int32_t &hi) {
  const Hr r = MENU[i].reg;
  lo = HR_RANGES[r].min;
  hi = HR_RANGES[r].max;
  if (r == HR_START_SOC) hi = s.get(HR_STOP_SOC) - 1;
  if (r == HR_STOP_SOC) lo = s.get(HR_START_SOC) + 1;
  if (r == HR_START_V) hi = s.get(HR_STOP_V) - START_STOP_GAP_CV;
  if (r == HR_STOP_V) hi = s.get(HR_CHARGE_V);
}

struct Leds {
  bool fault;
  bool charge;
};

// Fault LED: on for E1-E14, blinking (500 ms) for the E15/E16 warnings. Charge LED as stock 0800b444: with a module
// online and no error, solid while charging with current, 100 ms blink while charging without, 500 ms blink idle.
inline Leds leds(const Status &st, uint32_t now) {
  const bool slow = (now / 500) % 2, fast = (now / 100) % 2;
  Leds l{false, false};
  if (is_fault_code(st.error)) l.fault = true;
  else if (st.error != 0) l.fault = slow;
  if (st.online > 0 && st.error == 0) {
    if (st.state == charge::State::CHARGING) l.charge = st.output_a >= 0.5f ? true : fast;
    else l.charge = slow;
  }
  return l;
}

class Ui {
 public:
  // Every 10 ms with the raw key levels (Switch, Up, Down, Enter). Menu edits go to a copy, which replaces `live`
  // when the menu closes.
  UiActions on_keys(const bool raw[4], Settings &live, uint32_t now) {
    UiActions a;
    KeyEvent ev[4];
    const bool lit = backlight(live, now);
    for (int i = 0; i < 4; i++) {
      ev[i] = keys_[i].update(raw[i]);
      if (keys_[i].pressed()) {
        if (!was_pressed_[i] && !lit) swallow_[i] = true;  // a wake press does nothing else
        last_key_ms_ = now;
        any_key_ = true;
      }
      was_pressed_[i] = keys_[i].pressed();
      if (swallow_[i]) {
        if (!keys_[i].pressed()) swallow_[i] = false;
        ev[i] = KeyEvent::NONE;
        continue;
      }
    }
    if (ev[KEY_SWITCH] == KeyEvent::CLICK) a.charge_now = true;
    if (!swallow_[KEY_SWITCH] && keys_[KEY_SWITCH].held_ticks() == PAUSE_HOLD_TICKS) a.toggle_pause = true;

    if (item_ < 0) {
      if (ev[KEY_ENTER] == KeyEvent::CLICK) {
        item_ = 0;
        edit_ = live;
      }
      return a;
    }
    const bool up = ev[KEY_UP] == KeyEvent::CLICK || ev[KEY_UP] == KeyEvent::REPEAT;
    const bool down = ev[KEY_DOWN] == KeyEvent::CLICK || ev[KEY_DOWN] == KeyEvent::REPEAT;
    if (up || down) {
      int32_t lo, hi;
      menu_range(edit_, item_, lo, hi);
      const MenuItem &m = MENU[item_];
      edit_.set(m.reg, clamp_i32(edit_.get(m.reg) + (up ? m.step : -m.step), lo, hi));
      apply_clamps(edit_);
    }
    if (ev[KEY_ENTER] == KeyEvent::CLICK && ++item_ == MENU_ITEMS) close_(live, a);
    else if (ev[KEY_ENTER] == KeyEvent::LONG || now - last_key_ms_ >= MENU_IDLE_MS) close_(live, a);
    return a;
  }

  uint16_t held_ticks(KeyId k) const { return keys_[k].held_ticks(); }
  bool in_menu() const { return item_ >= 0; }
  int menu_item() const { return item_; }  // 0-based, -1 outside the menu
  const Settings &editing() const { return edit_; }

  // On until Backlight timeout (HR 11, 0 = always) after the last key press; on at power-up.
  bool backlight(const Settings &s, uint32_t now) const {
    const uint32_t timeout = uint32_t(s.get(HR_BACKLIGHT_S)) * 1000u;
    return timeout == 0 || now - (any_key_ ? last_key_ms_ : 0u) < timeout;
  }

  // Every 200 ms.
  Frame render(const Status &st, uint32_t now) {
    Frame f;
    put_fixed_labels(f);
    if (item_ >= 0) {
      render_menu_(f, now);
      return f;
    }
    if (std::isfinite(st.ac_v) && st.ac_v >= 1.0f) put_number(f, Field::AC_V, uint32_t(std::lround(st.ac_v)));
    else put_dashes(f, Field::AC_V);
    put_number(f, Field::CURRENT, uint32_t(std::lround(std::fmax(st.output_a, 0.0f))));
    put_number(f, Field::POWER, uint32_t(std::lround(std::fmax(st.power_w, 0.0f))));
    if (std::isfinite(st.battery_v) && st.battery_v >= 0.0f)
      put_number(f, Field::BATTERY_V, uint32_t(std::lround(st.battery_v * 10.0f)), 2);
    else
      put_dashes(f, Field::BATTERY_V);
    if (st.error != 0 && (now / 1000) % 2) put_error(f, st.error);  // 1 s SOC, 1 s error, as stock
    else if (st.soc >= 0) put_number(f, Field::SOC, uint32_t(st.soc));
    else put_dashes(f, Field::SOC);

    const bool charging = st.state == charge::State::CHARGING;
    const int bar_soc = st.soc >= 0 ? st.soc : (charging ? 100 : 0);  // no SOC: a sweep while charging
    if (charging && st.error == 0) {
      if (now - bar_ms_ >= BAR_STEP_MS) {
        bar_ms_ = now;
        bar_step_ = bar_step_ * 10 <= bar_soc ? bar_step_ + 1 : 0;
      }
      put_soc_bars(f, bar_soc, bar_step_);
    } else {
      put_soc_bars(f, bar_soc);
    }
    if (st.error != 0) put_segment(f, 28, 7);
    put_segment(f, 28, charging ? 5 : 6);  // charging / idle icon
    return f;
  }

 private:
  void close_(Settings &live, UiActions &a) {
    apply_clamps(edit_);
    if (edit_ != live) {
      live = edit_;
      a.settings_changed = true;
    }
    item_ = -1;
  }

  void render_menu_(Frame &f, uint32_t now) const {
    const MenuItem &m = MENU[item_];
    if (m.ind_seg < 0) put_number(f, Field::AC_V, uint32_t(item_ + 1));
    if ((now / BLINK_MS) % 2 == 0) return;  // the "off" half of the blink
    put_number(f, m.field, uint32_t(edit_.get(m.reg) / m.div), m.field == Field::BATTERY_V ? 2 : 1);
    if (m.ind_seg >= 0) put_segment(f, m.ind_seg, m.ind_com);
  }

  Key keys_[4];
  bool was_pressed_[4] = {false, false, false, false};
  bool swallow_[4] = {false, false, false, false};
  bool any_key_ = false;
  uint32_t last_key_ms_ = 0;
  int item_ = -1;
  Settings edit_ = default_settings();
  uint32_t bar_ms_ = 0;
  int bar_step_ = 0;
};

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_ui: ok`, and every other test `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/ui.h tests/test_oem_ui.cpp tests/Makefile
git commit -m "OEM UI: stock main screen with E-codes and bars, 8-item menu with clamps, wake press, LEDs, backlight"
```

---

### Task 11: The `charger` app

**Files:**
- Create: `oem/src/main.cpp`
- Modify: `oem/src/board.h` (add `RELAY`), `oem/CMakeLists.txt` (the `charger` app), `README.md`

**Interfaces:**
- Consumes: everything above.
- Produces: `oem/build/charger.elf`, `.bin` and `.dat`.

Main loop (spec §1):
- every pass:
  - drain CAN FIFO0 into both rectifiers;
  - run both rectifiers' `loop`;
  - pump the TX queue;
- keys, LEDs and backlight every 10 ms;
- ADC every 100 ms;
- controller and relay every 1 s;
- LCD every 200 ms;
- save settings 5 s after a menu change, retrying every 5 s on failure;
- feed the watchdog at the end of the pass.

- [ ] **Step 1: The relay pin.** Replace `oem/src/board.h` with:

`oem/src/board.h` (whole file):

```cpp
// oem/src/board.h: v1 board (silkscreen "100A充电器-主控板_V1.1-250911"); see docs/history/notes/oem-hw-map.md §2 and §10.
#pragma once
#include "hw/gpio.h"

namespace board {

using hw::Pin;
constexpr Pin KEY_SWITCH{hw::GPIOC, 6};  // keys: active high, pull-down
constexpr Pin KEY_UP{hw::GPIOB, 15};
constexpr Pin KEY_DOWN{hw::GPIOA, 12};
constexpr Pin KEY_ENTER{hw::GPIOA, 11};
constexpr Pin RS485_DE{hw::GPIOB, 10};   // high = transmit; shared by USART1 and UART4
constexpr Pin USART1_RX{hw::GPIOA, 3};
constexpr Pin UART4_RX{hw::GPIOB, 1};
constexpr Pin LED_FAULT{hw::GPIOD, 0};   // active low in stock v1
constexpr Pin LED_CHARGE{hw::GPIOD, 1};  // active low in stock v1
constexpr Pin OUT_PC1{hw::GPIOC, 1};     // stock: high while charging in voltage mode; relay candidate
constexpr Pin BACKLIGHT{hw::GPIOC, 2};   // stock: high = on
constexpr Pin PROBE_PB6{hw::GPIOB, 6};   // unused by stock v1; silkscreen test point by the relay driver
// Generator contact, high = closed. CONFIRM AT THE PHASE-1 BENCH (plan Task 9, step 4): stock v1 drives PC1 high only
// while charging in voltage mode, but the relay driver sits by the PB6 test point. Change only this line.
constexpr Pin RELAY = OUT_PC1;

inline void enable_gpio_clocks() { hw::reg(hw::RCU_AHBEN) |= 0xFu << 17; }  // PA..PD

}  // namespace board
```

- [ ] **Step 2: The app**

`oem/src/main.cpp` (new file):

```cpp
// oem/src/main.cpp: the charger app, phase 2 (Panel source). A cooperative super-loop (spec §1): CAN RX drain and the
// rectifier polls every pass, keys 10 ms, ADC 100 ms, LCD 200 ms, controller 1 s, settings saved 5 s after the last
// change, and the watchdog fed once per pass and nowhere else. Holding Up + Down for 5 s starves it (bench test).
#include <initializer_list>

#include "app/battery_adc.h"
#include "app/can_port.h"
#include "app/controller.h"
#include "app/limit_source.h"
#include "app/rectifier.h"
#include "app/relay.h"
#include "app/settings_store.h"
#include "app/ui.h"
#include "board.h"
#include "hw/adc.h"
#include "hw/can.h"
#include "hw/clock.h"
#include "hw/flash.h"
#include "hw/slcd.h"

using namespace oem;

namespace {

constexpr uint32_t KEYS_MS = 10, ADC_MS = 100, LCD_MS = 200, SAVE_DELAY_MS = 5000;
constexpr uint16_t STARVE_HOLD_TICKS = 500;      // Up + Down held 5 s: stop feeding the watchdog (bench test)
constexpr bool WRITE_STORED_DEFAULTS = true;     // spec §2.2; false leaves 0x0101 / 0x0104 alone

CanTxQueue g_txq;
Rectifier g_psu1(g_txq, 1, WRITE_STORED_DEFAULTS);
Rectifier g_psu2(g_txq, 2, WRITE_STORED_DEFAULTS);
Controller g_ctl(g_psu1, g_psu2);
Ui g_ui;
RelayLogic g_relay;
hw::Flash g_flash;
SettingsStore g_store(g_flash);

bool due(uint32_t &last, uint32_t period, uint32_t now) {
  if (now - last < period) return false;
  last = now;
  return true;
}

// Ten polled conversions, trimmed mean, scaled; NaN if the ADC doesn't answer.
float read_adc_volts(const Settings &s) {
  uint16_t samples[ADC_SAMPLES];
  for (uint16_t &v : samples) {
    if (!hw::adc_read(v)) return NAN;
  }
  return adc_volts(adc_trimmed_mean(samples), s.get(HR_ADC_TRIM));
}

void init_pins() {
  using board::Pin;
  for (Pin p : {board::KEY_SWITCH, board::KEY_UP, board::KEY_DOWN, board::KEY_ENTER})
    hw::pin_config(p, hw::Mode::INPUT, hw::Pull::DOWN);
  hw::pin_write(board::RS485_DE, false);  // receive: phase 2 never transmits on RS485
  hw::pin_write(board::LED_FAULT, true);  // LEDs are active low: off
  hw::pin_write(board::LED_CHARGE, true);
  hw::pin_write(board::RELAY, false);
  hw::pin_write(board::BACKLIGHT, true);
  for (Pin p : {board::RS485_DE, board::LED_FAULT, board::LED_CHARGE, board::RELAY, board::BACKLIGHT})
    hw::pin_config(p, hw::Mode::OUTPUT, hw::Pull::NONE);
}

}  // namespace

int main() {
  // CAN timing assumes APB1 = 30 MHz; on the IRC16M fallback the bus is left alone rather than run at a wrong rate.
  const bool hxtal = hw::clock_init();
  hw::fwdgt_init();
  hw::systick_init();
  board::enable_gpio_clocks();
  init_pins();
  const bool lcd = hw::slcd_init();
  hw::adc_init();
  const bool can = hxtal && hw::can_init();

  Settings settings;
  if (!g_store.load(settings)) settings = default_settings();  // phase 3: import the stock v1 record first
  apply_clamps(settings);

  Status st;
  float adc_v = NAN;
  bool dirty = false, starve = false;
  const uint32_t t0 = hw::millis();
  uint32_t last_keys = t0, last_adc = t0, last_lcd = t0, last_ctl = t0, changed_ms = t0;

  for (;;) {
    const uint32_t now = hw::millis();

    CanFrame rx;
    while (can && hw::can_receive(rx)) {
      g_psu1.on_frame(rx, now);
      g_psu2.on_frame(rx, now);
    }
    g_psu1.loop(now);
    g_psu2.loop(now);
    if (can) g_txq.pump(hw::can_send);

    if (due(last_keys, KEYS_MS, now)) {
      const bool raw[4] = {hw::pin_read(board::KEY_SWITCH), hw::pin_read(board::KEY_UP), hw::pin_read(board::KEY_DOWN),
                           hw::pin_read(board::KEY_ENTER)};
      const UiActions a = g_ui.on_keys(raw, settings, now);
      if (a.charge_now) g_ctl.charge_now();
      if (a.toggle_pause) g_ctl.toggle_pause();
      if (a.settings_changed) {
        dirty = true;
        changed_ms = now;
      }
      if (g_ui.held_ticks(KEY_UP) >= STARVE_HOLD_TICKS && g_ui.held_ticks(KEY_DOWN) >= STARVE_HOLD_TICKS) starve = true;
      const Leds l = leds(st, now);
      hw::pin_write(board::LED_FAULT, !l.fault);
      hw::pin_write(board::LED_CHARGE, !l.charge);
      hw::pin_write(board::BACKLIGHT, g_ui.backlight(settings, now));
    }

    if (due(last_adc, ADC_MS, now)) adc_v = read_adc_volts(settings);

    if (due(last_ctl, CONTROLLER_TICK_MS, now)) {
      // Phase 2: the Panel source whatever Source mode says (Pylontech and Modbus arrive in phase 3).
      st = g_ctl.tick(settings, panel_limits(settings), adc_v, now);
      const bool relay = g_relay.update(RelayMode(settings.get(HR_RELAY_MODE)), st.session_active, st.paused,
                                        st.state == charge::State::FAULT, false, now);
      hw::pin_write(board::RELAY, relay);
    }

    if (due(last_lcd, LCD_MS, now)) {
      const Frame f = g_ui.render(st, now);
      if (lcd) hw::slcd_write(f.com);
    }

    if (dirty && now - changed_ms >= SAVE_DELAY_MS) {
      if (g_store.save(settings)) {
        dirty = false;
      } else {
        changed_ms = now;  // try again in 5 s
      }
    }

    if (!starve) hw::fwdgt_feed();
  }
}
```

`oem/CMakeLists.txt` (whole file):

```cmake
# oem/CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
project(oem C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CPU_FLAGS -mcpu=cortex-m23 -mthumb)
set(OEM_VERSION 0.1)

add_compile_options(${CPU_FLAGS} -Os -g -ffunction-sections -fdata-sections -Wall -Wextra -Werror
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>")

set(HW_SOURCES src/startup.c src/hw/clock.cpp src/hw/slcd.cpp)

# Phase-2 drivers, built on their own so each one cross-compiles before the charger app links it.
add_library(oem_drivers STATIC src/hw/flash.cpp src/hw/can.cpp src/hw/adc.cpp)
target_include_directories(oem_drivers PRIVATE src)

function(oem_app name)
  add_executable(${name}.elf ${ARGN})
  target_include_directories(${name}.elf PRIVATE src ${CMAKE_CURRENT_SOURCE_DIR}/../esphome/components)
  target_link_options(${name}.elf PRIVATE ${CPU_FLAGS} -T${CMAKE_CURRENT_SOURCE_DIR}/ld/app.ld -nostartfiles
    --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-Map=${name}.map)
  add_custom_command(TARGET ${name}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${name}.elf ${name}.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/../tools/oem_image.py ${name}.bin ${name}.dat --version ${OEM_VERSION})
endfunction()

oem_app(blink ${HW_SOURCES} src/blink.cpp)
oem_app(bringup ${HW_SOURCES} src/bringup.cpp)
oem_app(charger ${HW_SOURCES} src/main.cpp)
target_link_libraries(charger.elf PRIVATE oem_drivers)
```

- [ ] **Step 3: Build and check size, heap and vector table**

Run:
```bash
cmake -S oem -B oem/build && cmake --build oem/build 2>&1 | grep -E -i "warning|error|FLASH:|RAM:"
T=${ARM_TOOLCHAIN:-$HOME/.local/arm-gnu-toolchain-15.3.rel2}/bin
$T/arm-none-eabi-nm oem/build/charger.elf | grep -c -E " (malloc|_sbrk|__cxa_pure_virtual|abort)$"
python3 -c "import struct;d=open('oem/build/charger.bin','rb').read();sp,rv=struct.unpack_from('<II',d);print(hex(sp),hex(rv))"
ls oem/build/blink.dat oem/build/bringup.dat oem/build/charger.dat
```
Expected:
- no warnings or errors;
- three memory reports, the charger app's at about 22 KB FLASH and 0.8 KB RAM;
- `0` from the `nm` count;
- SP `0x20004000`, and an odd reset vector inside 0x08005000–0x0801E000;
- all three `.dat` files listed.

- [ ] **Step 4: README.** In `README.md`, replace:

```
This writes `oem/build/<app>.elf`, `oem/build/<app>.bin` and `oem/build/<app>.dat`. The apps are `blink` and `bringup`.
```

with:

```
This writes `oem/build/<app>.elf`, `oem/build/<app>.bin` and `oem/build/<app>.dat`. The apps are `blink`, `bringup` and
`charger` (phase 2: charges from the panel settings; plan in
[`docs/history/superpowers/plans/2026-10-15-oem-firmware-phase2-charging.md`](docs/history/superpowers/plans/2026-10-15-oem-firmware-phase2-charging.md)).
With `charger` running, holding Up + Down for 5 s lets the watchdog reset the board, which opens the bootloader's 1.5 s window
without a power-cycle.
```

- [ ] **Step 5: Run everything**

Run: `make -C tests && .venv/bin/python -m pytest tools/ -q && cmake --build oem/build`
Expected: all tests pass, and the build succeeds.

- [ ] **Step 6: Commit**

```bash
git add oem/src/main.cpp oem/src/board.h oem/CMakeLists.txt README.md
git commit -m "OEM charger app: super-loop with CAN, rectifiers, controller, UI, ADC, relay and settings store (Panel source)"
```

---

### Task 12: `fake_nasn.py`: heatsink register, timed faults, output clamp

**Files:**
- Modify (whole files given): `tools/fake_nasn.py`, `tools/test_fake_nasn.py`

**Interfaces:**
- Produces: `FakeNasn.heatsink_c` (register 0x0235), and `in_window(t, start, until) -> bool`.
- New CLI options: `--alarm-from S`, `--alarm-until S`, `--heatsink C` and `--heatsink-until S`.
- A running module now reads `max(vout_set, battery_v)` on 0x0175/0x0184 (D21).

- [ ] **Step 1: Write the failing tests.** Replace `tools/test_fake_nasn.py` with:

`tools/test_fake_nasn.py` (whole file):

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from fake_nasn import FakeNasn, in_window  # noqa: E402


def frame(*b):
    return bytes(b)


def test_read_voltage_setpoint():
    m = FakeNasn(1)
    rid, data = m.handle(0x108182FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0))
    assert rid == 0x1081827E
    assert data[:2] == bytes([0x01, 0x00])
    assert int.from_bytes(data[2:], "big") == round(54.0 * 1023)


def test_write_voltage_current_standby():
    m = FakeNasn(2)
    rid, data = m.handle(0x108281FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))
    assert rid == 0x1082817E and data == frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7)
    assert abs(m.vout_set - 57.0) < 0.001
    m.handle(0x108281FE, frame(0x02, 0x2D, 0, 0, 0, 0, 0x02, 0x00))
    assert abs(m.ilimit - 26.03) < 0.01
    m.handle(0x108281FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))
    assert m.standby == 1


def test_telemetry_follows_state():
    m = FakeNasn(1, battery_v=52.0)
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))  # standby
    _, d = m.handle(0x108182FE, frame(0x01, 0x75, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 52 * 1024
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 0
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 0, 0, 0, 0, 0))  # on, 54 V > 52 V battery, 5 A default
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 5 * 1024


def test_ignores_other_traffic():
    m = FakeNasn(1)
    assert m.handle(0x108282FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None  # addr 2
    assert m.handle(0x1081827E, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None  # a reply, not a request
    assert m.handle(0x108182FE, frame(0x01, 0x00)) is None                      # short
    m.muted = True
    assert m.handle(0x108182FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None


def test_unknown_register_status_2():
    m = FakeNasn(1)
    _, d = m.handle(0x108182FE, frame(0x07, 0x77, 0, 0, 0, 0, 0, 0))
    assert d[0] >> 4 == 2 and (d[0] & 0x0F) == 0x07 and d[1] == 0x77


def test_fallback_resets_setpoints():
    m = FakeNasn(1)
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))
    m.reset_to_defaults()
    assert abs(m.vout_set - 54.0) < 0.001 and m.ilimit == 5.0 and m.standby == 0


def test_only_controller_requests_hold_off_fallback():
    from fake_nasn import is_controller_request
    assert is_controller_request(0x108182FE, bytes(8))      # read request to addr 1
    assert is_controller_request(0x108281FE, bytes(8))      # write request to addr 2
    assert not is_controller_request(0x1081827E, bytes(8))  # a module's reply
    assert not is_controller_request(0x1001117E, bytes(8))  # proto 0x20 module chatter
    assert not is_controller_request(0x108182FE, bytes(2))  # short frame


def test_vout_readback_022f_follows_write():
    m = FakeNasn(1)
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))  # 57.0 V
    _, d = m.handle(0x108182FE, frame(0x02, 0x2F, 0, 0, 0, 0, 0, 0))
    assert d[0] >> 4 == 0 and d[:2] == bytes([0x02, 0x2F])
    assert int.from_bytes(d[2:], "big") == 0xE3C7


def test_output_voltage_0184_matches_0175():
    m = FakeNasn(1, battery_v=52.0)
    _, a = m.handle(0x108182FE, frame(0x01, 0x75, 0, 0, 0, 0, 0, 0))
    _, b = m.handle(0x108182FE, frame(0x01, 0x84, 0, 0, 0, 0, 0, 0))
    assert b[0] >> 4 == 0 and b[2:] == a[2:]


def test_heatsink_register():
    m = FakeNasn(1)
    m.heatsink_c = 50.0
    _, d = m.handle(0x108182FE, frame(0x02, 0x35, 0, 0, 0, 0, 0, 0))
    assert d[0] == 0x02 and int.from_bytes(d[2:], "big") == 50 * 1024


def test_running_below_battery_reads_battery_voltage():
    m = FakeNasn(1, battery_v=52.0)
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 0, 0, 0, 0, 0))  # on
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xBF, 0xD0))  # 48.0 V (cooldown setpoint)
    _, d = m.handle(0x108182FE, frame(0x01, 0x84, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 52 * 1024
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 0


def test_in_window():
    assert in_window(5, 0, None) and in_window(60, 60, 90) and not in_window(90, 60, 90) and not in_window(59, 60, None)
```

- [ ] **Step 2: Run them and check they fail**

Run: `.venv/bin/python -m pytest tools/test_fake_nasn.py -q`
Expected: `ImportError: cannot import name 'in_window'`.

- [ ] **Step 3: Implement.** Replace `tools/fake_nasn.py` with:

`tools/fake_nasn.py` (whole file):

```python
#!/usr/bin/env python3
"""Emulate NASN RM4850-B rectifiers (the dialect the stock Eco-Worthy firmware speaks) on the CAN bus.

Answers cmd 0x82 register reads and cmd 0x81 register writes for each emulated address. If the
controller goes quiet for --fallback-after seconds, the modules revert to their stored defaults,
as real modules are expected to. Bench use only: never run this on a bus with real PSUs attached.

  fake_nasn.py [--seconds 600] [--addrs 1,2] [--mute 2] [--alarm 1] [--alarm-from 60 --alarm-until 90]
               [--heatsink 50 --heatsink-until 120] [--fallback-after 60] [--battery-v 52]
"""
import argparse
import time

import can

import sniff  # noqa: F401  (macOS gs_usb detach patch)

SETPOINT_SCALE = 1023
TELEMETRY_SCALE = 1024
NOMINAL_A = 52.0
WRITE_CMD, READ_CMD = 0x81, 0x82


def is_controller_request(arb, data):
    """True for a frame a controller sends to a module (proto 0x21, bit 7 set): only these hold off the fallback."""
    return len(data) == 8 and (arb >> 23) & 0x3F == 0x21 and bool(arb & 0x80)


class FakeNasn:
    """One emulated module. handle() takes a controller request and returns the reply, or None."""

    def __init__(self, addr, battery_v=52.0, default_v=54.0, default_a=5.0):
        self.addr = addr
        self.battery_v = battery_v
        self.default_v = default_v
        self.default_a = default_a
        self.alarm = 0
        self.heatsink_c = 35.0
        self.muted = False
        self.reset_to_defaults()

    def reset_to_defaults(self):
        self.vout_set = self.default_v
        self.ilimit = self.default_a
        self.standby = 0  # modules come up running at their stored defaults

    def registers(self):
        on = not self.standby
        iout = self.ilimit if on and self.vout_set > self.battery_v else 0.0
        # A running module set below the battery can't pull it down: it reads the battery voltage.
        vout = max(self.vout_set, self.battery_v) if on else self.battery_v
        p_out = vout * iout
        p_in = p_out / 0.96 + 15 if on else 5

        def t(x):
            return round(x * TELEMETRY_SCALE)

        return {
            0x0100: round(self.vout_set * SETPOINT_SCALE),
            0x0101: round(self.default_v * SETPOINT_SCALE),
            0x0104: round(self.default_a * SETPOINT_SCALE / NOMINAL_A),
            0x0132: self.standby,
            0x0170: t(p_in),
            0x0171: t(50.0),
            0x0172: t(p_in / 230.0),
            0x0173: t(p_out),
            0x0175: t(vout),
            0x0178: t(230.0),
            0x0184: t(vout),  # the stock firmware's output-voltage register
            0x017F: t(35.0),
            0x0180: t(30.0),
            0x0182: t(iout),
            0x0183: self.alarm,
            0x0235: t(self.heatsink_c),  # heatsink, as the cooldown logic reads it
            0x022D: round(self.ilimit * SETPOINT_SCALE / NOMINAL_A),
            0x022F: round(self.vout_set * SETPOINT_SCALE),  # VOUT readback, as checked by the stock firmware
        }

    def handle(self, arb, data):
        if self.muted or len(data) != 8 or (arb >> 23) & 0x3F != 0x21 or not arb & 0x80:
            return None
        if (arb >> 16) & 0x7F != self.addr:
            return None
        cmd = (arb >> 8) & 0xFF
        reg = (data[0] & 0x0F) << 8 | data[1]
        reply_id = arb & ~0x80
        error = bytes([0x20 | (data[0] & 0x0F), data[1]])  # status 2 in the high nibble
        if cmd == READ_CMD:
            regs = self.registers()
            if reg not in regs:
                return reply_id, error + bytes(6)
            return reply_id, bytes(data[:2]) + regs[reg].to_bytes(6, "big")
        if cmd == WRITE_CMD:
            raw = int.from_bytes(data[6:8], "big")
            if reg == 0x0100:
                self.vout_set = raw / SETPOINT_SCALE
            elif reg == 0x022D:
                self.ilimit = raw * NOMINAL_A / SETPOINT_SCALE
            elif reg == 0x0132:
                self.standby = data[3]
            elif reg == 0x0101:
                self.default_v = raw / SETPOINT_SCALE
            elif reg == 0x0104:
                self.default_a = raw * NOMINAL_A / SETPOINT_SCALE
            else:
                return reply_id, error + bytes(data[2:])
            return reply_id, bytes(data)
        return None

    def status(self):
        return (f"addr {self.addr}: {'STANDBY' if self.standby else 'ON     '} vset={self.vout_set:6.2f} V "
                f"ilim={self.ilimit:5.1f} A defaults={self.default_v:.2f} V/{self.default_a:.1f} A"
                f"{' MUTED' if self.muted else ''}{f' ALARM={self.alarm:#06x}' if self.alarm else ''}")


def in_window(t, start, until):
    """True while start <= t < until; until None means for the rest of the run."""
    return t >= start and (until is None or t < until)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=600)
    ap.add_argument("--addrs", default="1,2")
    ap.add_argument("--mute", type=int, action="append", default=[], help="address that never answers")
    ap.add_argument("--alarm", type=int, action="append", default=[], help="address reporting alarm bit 0x0001 (E8)")
    ap.add_argument("--alarm-from", type=float, default=0, help="seconds into the run the alarm starts")
    ap.add_argument("--alarm-until", type=float, default=None, help="seconds into the run the alarm clears")
    ap.add_argument("--heatsink", type=float, default=35.0, help="heatsink reading in C (cooldown starts at >= 45)")
    ap.add_argument("--heatsink-until", type=float, default=None, help="seconds into the run it drops back to 35 C")
    ap.add_argument("--fallback-after", type=float, default=60)
    ap.add_argument("--battery-v", type=float, default=52.0)
    a = ap.parse_args()

    mods = {int(x): FakeNasn(int(x), battery_v=a.battery_v) for x in a.addrs.split(",")}
    for addr in a.mute:
        mods[addr].muted = True

    bus = can.Bus(interface="gs_usb", channel=0, index=0, bitrate=125000)
    t0 = last_rx = next_status = time.time()
    fell_back = False
    print(f"fake NASN at {sorted(mods)} for {a.seconds:.0f}s", flush=True)
    try:
        while time.time() - t0 < a.seconds:
            msg = bus.recv(timeout=0.2)
            now = time.time()
            alarm_on = in_window(now - t0, a.alarm_from, a.alarm_until)
            for mod in mods.values():
                mod.alarm = 0x0001 if mod.addr in a.alarm and alarm_on else 0
                mod.heatsink_c = a.heatsink if in_window(now - t0, 0, a.heatsink_until) else 35.0
            if msg is not None and msg.is_rx and msg.is_extended_id and not msg.is_error_frame:
                data = bytes(msg.data)
                if is_controller_request(msg.arbitration_id, data):
                    last_rx, fell_back = now, False
                for mod in mods.values():
                    reply = mod.handle(msg.arbitration_id, data)
                    if reply is None:
                        continue
                    bus.send(can.Message(arbitration_id=reply[0], data=reply[1], is_extended_id=True))
                    if (msg.arbitration_id >> 8) & 0xFF == WRITE_CMD:
                        print(f"({now - t0:8.3f}) write {msg.arbitration_id:08X}#{data.hex(' ')}  -> {mod.status()}", flush=True)
            if not fell_back and now - last_rx > a.fallback_after:
                for mod in mods.values():
                    mod.reset_to_defaults()
                fell_back = True
                print(f"({now - t0:8.3f}) CAN silent {a.fallback_after:.0f}s: modules revert to stored defaults", flush=True)
            if now >= next_status:
                for mod in mods.values():
                    print(f"({now - t0:8.3f}) {mod.status()}", flush=True)
                next_status = now + 5
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `.venv/bin/python -m pytest tools/ -q`
Expected: all pass (one skip if the flash dump is absent).

- [ ] **Step 5: Commit**

```bash
git add tools/fake_nasn.py tools/test_fake_nasn.py
git commit -m "fake_nasn: heatsink register, timed alarm and heatsink injection, running output can't drop below the battery"
```

---

### Task 13: [BENCH] The charger app against fake modules

This task needs the user. No real modules are connected: only the board, a bench supply and a CANable running `fake_nasn.py`.

**Files:**
- Modify: `docs/history/notes/oem-bringup.md` (new section "Phase 2: fake modules")

- [ ] **Step 1: Bench setup (user).**
  - Bench supply: 48 V, limited to 0.3 A, on the board's +48 V / GND.
  - USB-RS485 adapter on J5. ST-Link on J3, in case recovery is needed.
  - CANable on CAN_H / CAN_L, with its 120 Ω termination on. It is the only other node.
  - The phase-1 bench (Task 9) must have confirmed the relay pin. If it showed PB6, change `board::RELAY` first and rebuild.

- [ ] **Step 2: Install.** Run `.venv/bin/python tools/oem_update.py --port /dev/cu.usbserial-XXXX oem/build/charger.dat`,
  power-cycling when prompted.
  - Expected: the stock-style main screen, the idle icon, AC `---`, battery V ≈ 48.0 (ADC), and both LEDs off.

- [ ] **Step 3: Modules appear.** Run `.venv/bin/python tools/fake_nasn.py --seconds 1800`.
  - Expected, within 2 s:
    - AC shows 230;
    - the fake logs writes of standby (0x0132 = 1) for both addresses;
    - there are no 0x0101 / 0x0104 writes, because the fake's stored defaults already equal the settings (54.00 V, 5 A);
    - the Charge LED blinks at 500 ms.
  - Record the time from start to "online".

- [ ] **Step 4: Charge now and the ramp.** Click Switch.
  - Expected:
    - the fake logs 0x0132 = 0 and 0x0100 = 57.60 V for both modules;
    - 0x022D then rises by about 1 A per module per second, up to 5 A each (Max current 10 A);
    - the display shows 10 A and about 576 W;
    - the charging icon is on, and the bars sweep;
    - the Charge LED is solid;
    - the relay closes (click, or measure PC1 / PB6).

- [ ] **Step 5: Pause and the relay delay.** Hold Switch for 2 s.
  - Expected: both modules go to standby within 1 s, and the relay opens about 10 s later.
  - Hold Switch for 2 s again: charging resumes, ramping from 0.

- [ ] **Step 6: Fault injection.** Stop the fake, then run
  `.venv/bin/python tools/fake_nasn.py --seconds 300 --alarm 1 --alarm-from 60 --alarm-until 90`. Click Switch to charge.
  - Expected:
    - at about 60 s: E08 alternates with `---`, the Fault LED is on, and both modules go to standby;
    - at about 120 s (30 s after the alarm clears): charging resumes and E08 goes.
  - Record the times.

- [ ] **Step 7: Cooldown.** Run `.venv/bin/python tools/fake_nasn.py --seconds 300 --heatsink 50 --heatsink-until 120`, charge
  for 20 s, then pause (hold Switch 2 s).
  - Expected:
    - both modules stay on at 48.00 V / 1.0 A (writes logged) instead of going to standby;
    - at about 120 s they go to standby;
    - the relay opens 10 s after the pause, independently.

- [ ] **Step 8: One module missing.** Run with `--mute 2`, set Max current to 40 in the menu, and charge.
  - Expected: module 1 alone ramps to 40 A (the fake's 0x022D readback); module 2 is never written after boot.
  - Then set Max current back to 10.

- [ ] **Step 9: Modules vanish.** While charging, stop `fake_nasn.py`.
  - Expected: after about 5 s, AC shows `---`, the current shows 0, and the Charge LED is off.
  - Start the fake again: it reappears within about 1 s and gets setpoints at once.

- [ ] **Step 10: Menu and persistence.**
  - Enter the menu. Step through all 8 items and check each one:
    - the value blinks;
    - the indicator is right for items 1–6;
    - items 7 and 8 show their number in the AC field;
    - Stop V can't go above Charge V, and Start V can't go above Stop V − 5.0.
  - Set Max current to 12 and exit.
  - Wait 6 s, then power-cycle. Expected: Max current is still 12.
  - Over SWD (read-only), `mdw 0x0801F800 12` shows a record starting `'O' 'S' 01 11`.

- [ ] **Step 11: Watchdog.** Hold Up + Down for 5 s.
  - Expected: the display freezes, then within about 2 s the board resets and comes back charging-idle.
  - The fake logs the CAN silence. It shouldn't revert to its stored defaults: its default `--fallback-after` is 60 s.

- [ ] **Step 12: Record and commit.** Write the observations and times into `docs/history/notes/oem-bringup.md` under "Phase 2: fake modules",
  then:

```bash
git add docs/history/notes/oem-bringup.md
git commit -m "OEM phase 2 bench: charger app against fake_nasn (online, ramp, pause, faults, cooldown, menu, watchdog)"
```

---

### Task 14: [BENCH] The charger app with the real modules

This is the first time the board drives real modules. Start conservative. Stop at the first surprise, and capture the CAN traffic
with `tools/sniff.py`.

**Files:**
- Modify: `docs/history/notes/oem-bringup.md` (new section "Phase 2: real modules")

- [ ] **Step 1: Setup (user).**
  - Both RM4850-B modules on AC, on the CAN bus with the board.
  - The DC output goes to the battery through its fuse. Alternatively, use a resistive load the modules can drive at
    ≤ 5 A.
  - The CANable stays on the bus as a sniffer only: `.venv/bin/python tools/sniff.py 1800`.
  - Before charging, set these in the menu:
    - Max current 4;
    - Charge V a safe value for the battery (for example 54.0);
    - Stop V ≤ Charge V.

- [ ] **Step 2: Online and idle.**
  - Expected:
    - AC shows the mains voltage;
    - no E-code;
    - the sniffer shows reads to both addresses about 20 ms apart, one at a time per module;
    - the modules stay in standby.
  - Record the 0x0101 / 0x0104 writes. There should be at most one each, then none, once the readback matches 54.00 V /
    5.0 A.

- [ ] **Step 3: Charge at 4 A.** Click Switch.
  - Expected:
    - the current ramps to 4 A total (2 A per module);
    - battery V on the display tracks the ADC;
    - no E13 after 15 s: the setpoint readback agrees on the real modules;
    - W ≈ V × A.
  - Compare the display's battery V with a multimeter. If they differ by more than 1 %, note the per-mille error: it is the
    HR 14 trim to apply in phase 3.

- [ ] **Step 4: Raise the current.** Set Max current to 10, then 20.
  - Expected: it ramps at about 2 A/s, splits evenly, and there are no faults.
  - Watch the module temperatures in the sniffer replies (0x0235).

- [ ] **Step 5: Pause, and cooldown if hot.** Pause.
  - Expected: if a module's heatsink is ≥ 45 °C, it stays on at 48 V / 1 A until it is ≤ 40 °C (at most 10 min), then standby.
    Otherwise it goes straight to standby.

- [ ] **Step 6: Watchdog with the modules running.** While charging at 10 A, hold Up + Down for 5 s.
  - Expected: the board resets within about 2 s and comes back. While it is gone, the modules keep their last setpoints
    (or their stored defaults). After boot they get standby, until the session restarts. Record what the output current did.

- [ ] **Step 7: CAN-silence fallback.** While charging at 10 A, switch off the board's own 48 V supply. The modules keep AC.
  - Watch the output current with a clamp meter, or the battery's BMS, until the modules revert to their stored defaults.
    Record how long that takes.
  - Expected: at most 54.0 V and 5 A per module.
  - Restore the board's power. It takes over again: standby, until *charge now* or the start condition.
  - With the board off, `.venv/bin/python tools/psu_ctl.py read --addr 1` can confirm the 0x0100 / 0x022D readbacks.

- [ ] **Step 8: Session stop.** Set Stop V just above the battery's present voltage and charge.
  - Expected: once V ≥ Stop V and the current is ≤ 4 A for 10 s, the session ends: HOLD, modules in standby, relay open 10 s
    later.

- [ ] **Step 9: Record and commit.** Write everything, including the sniffer log names, into `docs/history/notes/oem-bringup.md` under
  "Phase 2: real modules". Then:

```bash
git add docs/history/notes/oem-bringup.md
git commit -m "OEM phase 2 bench: charger app on the real modules (ramp, split, cooldown, watchdog, CAN-silence fallback)"
```

---

## Self-review

**Spec coverage (phase 2 of §7):**

| Spec item | Task |
|---|---|
| CAN driver (§1, `hw/can`) | 5 |
| `Rectifier` (§1, §2.2: poll schedule, setpoints, readback, stored defaults, alarm mask) | 7 |
| `Controller` (§2.2: budget, low line, split, ramp, state machine, faults with 30 s auto-clear, cooldown, no-current) | 9 |
| `SessionGate` (§2.3), with its clamps in `apply_clamps` (§2.3, Task 3) | 8, 3 |
| Relay Auto/Off, with the Modbus input stubbed (§2.4) | 8, 11 |
| Panel limits and `Limits` (§2.1) | 9 |
| Main screen, E1–E16, LEDs, keys, menu items 1–8, backlight (§3) | 1, 9, 10 |
| ADC with trim (§2.2, §1 ADC task) | 6, 11 |
| Settings store (§5.2), with the stock-import hook left in `main.cpp` for phase 3 | 3, 4, 11 |
| Super-loop periods and the watchdog (§1, §6) | 11 |
| Hard limits (§6) | 7 (`FALLBACK_MAX_*`), 9 (`HARD_MAX_V`, `MAX_A_PER_PSU`, mask, overvoltage) |
| Host tests (§8): SessionGate, settings incl. the torn write, `lcd_render`, `Rectifier`/`Controller` on a fake bus, relay | 1, 3, 7–10 |
| Size limits (§8): the linker script fails above 100 KB of flash or below 4 KB of stack | 11 (reported) |
| Bench: `fake_nasn.py` first, then the real modules (§7) | 13, 14 |

**Left for phase 3, by design:**
- Modbus, the Pylontech master, and the source-mode switch;
- the stale-source fallback current (E15 is wired but never raised in Panel mode);
- relay mode Modbus (HR 112);
- the stock settings import;
- telemetry of each module's 0x0184 reading next to the ADC.

**Placeholders:** none. Every code step gives the whole file or an exact replace pair.

**Type consistency:** the names in each task's **Interfaces** block were checked against the code. The dry run compiled and ran
every task in order.
