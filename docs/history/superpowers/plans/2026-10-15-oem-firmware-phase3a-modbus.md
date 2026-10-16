# OEM Board Firmware, Phase 3a (Modbus RTU server) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The charger app answers Modbus RTU on the rear RS485 jack. Through it, a USB-RS485 adapter can:
- read every input register;
- read and write the settings and the runtime commands (pause, charge now, clear fault, relay command, reboot into the
  bootloader).

That lets the board be observed and driven once the unit is assembled and SWD can't be reached.

**Architecture:**
- **USART1 driver.** USART1 on PA2/PA3 (DE on PB10) gets a small driver:
  - RX by circular DMA, polled from the main loop, as stock does;
  - TX from a 256-byte buffer by the USART1 interrupt, with DE dropped on transmission complete.
- **Host-tested Modbus code**, header-only:
  - `oem/src/app/modbus_rtu.h`: framing, CRC, function codes and exceptions;
  - `oem/src/app/modbus_map.h`: the spec §4 register map over the live `Status`, rectifiers and settings.
- **`main.cpp`** wires these together and applies the commands.
- **Mac tool.** `tools/oem_modbus.py` reads and writes the registers from the Mac, and `tools/oem_update.py` gains
  `--modbus-reboot`, so an installed board can be updated without a power-cycle.

**Tech Stack:** as phase 2: Arm GNU Toolchain 15.3.Rel2, CMake, C++17 bare metal, host tests via `tests/Makefile`, Python 3 in
`.venv` (pyserial, pytest).

**Spec:** `docs/history/superpowers/specs/2026-10-14-oem-board-firmware-design.md`: §4 (Modbus RTU), the RS485 parts of §5.3, and
phase 3 of §7. This plan covers the Modbus slave only; the Pylontech master and the non-panel limit sources are phase 3b.
- Phase-2 plan (conventions, decisions D1–D21): `docs/history/superpowers/plans/2026-10-15-oem-firmware-phase2-charging.md`.
- Hardware evidence:
  - `docs/history/notes/oem-hw-map.md` §2 and §3 (USART1 on PA2/PA3 AF7, DE on PB10, one transceiver);
  - `docs/history/notes/oem-init-sequences.md` (08008824 USART init; IRQ28 = USART1);
  - `docs/history/notes/oem-bringup.md` (bench: PA3 sees RO).
  - Task 1 adds `docs/history/notes/oem-usart-sequences.md`.

## Global Constraints

- **App placement:** linked at **0x08005000**, at most **0x19000 bytes (100 KB)** including the trailer. Never touch
  0x08000000–0x08004FFF (bootloader) or the **0x0801E000** page.
- **Flash writes:** only the settings pages **0x0801F800 and 0x0801FC00** (spec §5.2).
- **Watchdog:** FWDGT (about 2 s) is fed only at the end of a main-loop pass (`hw::fwdgt_feed()`), never from an interrupt.
- **Interrupts:** SysTick and **USART1 (IRQ28) only**. The USART1 handler moves TX bytes and drops DE. It never touches
  anything the main loop changes with `rmw()` except through the documented handoff in Task 4. CAN RX stays polled.
- **Hard limits (spec §6):** unchanged. No Modbus write can get round them: 58.0 V target, 50 A per module, the alarm mask, the
  overvoltage fault, stored defaults ≤ 54.4 V / 10 A. Settings writes go through the same ranges and clamps as the menu.
- **Compile flags:** C++17, `-mcpu=cortex-m23 -mthumb -Os`, `-Wall -Wextra -Werror`, `-fno-exceptions -fno-rtti`.
- **No heap anywhere** (the linker script has no `end`). No `std::string`, `std::vector`, `std::deque`.
- **Shared code:** `charge_logic.h` and `nasn_protocol.h` are not changed by this plan.
- **Host tests:** `make -C tests` and `.venv/bin/python -m pytest tools/ -q` pass after every task.
- **Cross build:** `cmake -S oem -B oem/build && cmake --build oem/build` succeeds with no warnings after every task, and the
  `charger` ELF has no `malloc`/`_sbrk`.
- **Style:** comments match the repo: sparse, explaining why. Lines ≤ 120 columns. App modules stay header-only (phase-2 D1).
- **Never commit** `dumps/`, `captures/`, `re/`, `ghidra/`, `stock_flasher/`, `build/`, `oem/build/` or `*.dat`.

## Decisions (made while planning; the spec is silent or they refine it)

| # | Decision | Why |
|---|---|---|
| M1 | **Scope:** the Modbus server only. HR 6 (Source mode) accepts only 0 (Panel); 1 and 2 get exception 03 until phase 3b. HR 100–107 are stored and read back but nothing uses them yet. | The goal is observability and remote control for the first real-module run. Accepting a mode that doesn't work would be misleading. |
| M2 | **TX by interrupt**, RX by circular DMA polled from the main loop. | RTU forbids gaps over 1.5 characters inside a frame (≈130 µs at 115200). A flash erase stalls the main loop for milliseconds, so polled TX would break frames. RX DMA tolerates stalls up to the buffer size. |
| M3 | **End of frame** = no new byte for `gap_ms(baud)`: `ceil(38500 / baud) + 1` ms for baud ≤ 19200 (9600 → 6 ms), else 2 ms. Frames over 256 bytes are dropped. | 3.5 characters of 11 bits at 1 ms resolution, with one tick of margin. Above 19200 the RTU spec fixes t3.5 at 1.75 ms. |
| M4 | **Own echo:** bytes received from the start of a reply until `gap_ms` after its last bit are discarded. | Whether the transceiver's receiver is off while DE is high isn't known. Either way, the slave must not parse its own reply. |
| M5 | **Addressing:** HR 7 (1–247). Address 0 (broadcast) executes writes and never replies. Frames for other addresses are ignored. | Modbus RTU. |
| M6 | **Settings writes** (HR 0–16, FC 06/16): the values go into a copy of the live settings. Each value must be inside its `HR_RANGES` entry, and the copy must not violate a cross-field clamp (`violates_clamps`). Otherwise exception 03 and nothing is stored. FC 16 is all-or-nothing. On success the live settings change, and the existing 5 s save applies. | The spec says a write that breaks a clamp gets exception 03. Silent clamping would store something the master didn't write. |
| M7 | **While the menu is open**, writes to HR 0–16 get exception 06 (slave device busy). Runtime registers still work. | Phase-2 deferred finding: the menu edits a copy and replaces the live settings when it closes, which would silently undo a Modbus write. |
| M8 | **Line changes** (HR 7 address, HR 8 baud) take effect after the reply has been sent completely. | Spec §4. |
| M9 | **Runtime commands:**<br>- HR 110: 0/1 sets pause, persisted in BKP4 like a key pause.<br>- HR 111: 1 clears the fault.<br>- HR 113: 1 = charge now.<br>- HR 112: 0/1 is the relay command, used in relay mode 2.<br>- HR 120: 0xB007 reboots into the bootloader once the reply has been sent.<br>Any other value is exception 03. Reads return the current pause and relay command, and 0 for 111/113/120. | Spec §4. Pause must behave the same whichever way it was set. |
| M10 | **Input registers** are computed on demand from live references; no snapshot copy. NaN reads as 0x7FFF (signed) or 0xFFFF (unsigned). Values are saturated to the int16 / uint16 range. Registers the board doesn't measure read as not-available: PSU inlet temperature and fans → 0x7FFF / 0xFFFF; IR 19 → 0xFFFF. | Spec §4 values convention. |
| M11 | **IR 0** firmware version = `0x0300` (major 3, minor 0) as `OEM_FW_VERSION` in `modbus_map.h`. **IR 3** error code 0–17 (E17 added after the spec's 1–16). **IR 5** active source = 0. **IR 18** source age = 0 (Panel is always fresh). | The spec is silent on the version; the rest follows from M1. |
| M12 | `Status` gains `cvl` (V, in use) and `ccl` (A, total, in use). `Controller` gains `module_cooling(int k)`. `RelayLogic` re-arms `last_wanted_ms_` when leaving Modbus or Off mode (phase-2 deferred: switching back to Auto used a stale time). | IR 14/15 and the PSU status bit need them. The relay fix is in the code this plan touches. |
| M13 | The UART4 vendor-handshake listener (spec §5.3, last bullet) is **not** in this slice. `--modbus-reboot` covers updates of our own firmware. | Keep the slice small. |

## File structure

| File | Responsibility | Task |
|---|---|---|
| `docs/history/notes/oem-usart-sequences.md` | Stock USART1 + DMA + DE register sequences with evidence | 1 |
| `oem/src/app/modbus_rtu.h` | RTU framing, CRC check, FC 03/04/06/16, exceptions, `FrameAssembler`, `gap_ms` | 2 |
| `tests/test_oem_modbus.cpp` | Tests for the above | 2 |
| `oem/src/app/settings.h` | + `violates_clamps()` | 3 |
| `oem/src/app/controller.h` | + `Status::cvl/ccl`, `module_cooling()` | 3 |
| `oem/src/app/relay.h` | Re-arm on leaving Modbus/Off | 3 |
| `oem/src/app/modbus_map.h` | `OemRegisters : RegisterBank`: IR 0–39, HR 0–16, 100–113, 120; `Commands` out | 3 |
| `tests/test_oem_modbus_map.cpp` | Tests for the above | 3 |
| `oem/src/hw/rs485.h/.cpp`, `oem/src/hw/regs.h`, `oem/src/startup.c`, `oem/CMakeLists.txt` | USART1 driver, IRQ28 vector | 4 |
| `oem/src/main.cpp` | Modbus task, commands, line changes, reboot | 5 |
| `tools/oem_modbus.py`, `tools/test_oem_modbus.py` | Mac-side client | 6 |
| `tools/oem_update.py`, `tools/test_oem_update.py` | `--modbus-reboot` | 6 |
| `README.md`, spec §4 note | Docs | 6 |
| `docs/history/notes/oem-bringup.md` | Bench results | 7 |

---

### Task 1: Stock USART1 sequences (research, no code)

**Files:** Create `docs/history/notes/oem-usart-sequences.md`.

Sources:
- the v1 decompile in `re/` (`re/decomp_app.c`, `re/v1/`) and Ghidra projects in `ghidra/`;
- `dumps/gd32_flash_20261012.bin`;
- the existing notes: `docs/history/notes/oem-init-sequences.md` (08008824 initialises USART1, UART3 and UART4; IRQ28 = USART1 at
  0x0800D1F9), and `docs/history/notes/oem-hw-map.md` §2–3.

Bench evidence from 2026-10-15, under the stock app:
- DMA channel 1: CTL 0x000030A1, CNT 0x200, PADDR 0x40004424 (USART1 RDATA);
- UART4 BAUD 0x41 at 460800, so APB1 = 30 MHz.

Record, with the stock address that proves each one (tag `[V]` verified in code, `[I]` inferred):

- [ ] **Step 1: USART1 init.**
  - The RCU enable and reset bits.
  - Every register write with its value: CTL0/CTL1/CTL2, BAUD for 9600 at 30 MHz, the order, and when UEN is set.
  - The GPIO setup for PA2 (TX: AF7, push-pull, pull-up) and PA3 (RX: AF7, pull-up).
- [ ] **Step 2: RX DMA.**
  - Which DMA channel, and how the request is routed. Does the GD32L23x use a DMAMUX? If so, its register, the channel
    multiplexer value and the request ID for USART1_RX.
  - CTL/CNT/PADDR/MADDR values, and the USART CTL2 DENR bit.
  - How stock reads the received bytes (CNT arithmetic).
- [ ] **Step 3: TX and DE.**
  - How stock transmits on USART1 (polled TBE, interrupt or DMA).
  - When PB10 (DE) goes high and when it goes low (TC?).
  - The STAT bit positions for TBE, TC, RBNE, IDLEF, ORERR, FERR, NERR.
  - Which INTC/clear register clears TC and the error flags.
  - The CTL0 bit positions for UEN, REN, TEN, TBEIE, TCIE, RBNEIE, IDLEIE.
  - The NVIC enable for IRQ28, and its priority if stock sets one.
- [ ] **Step 4: Baud values.** Compute BAUD for 9600, 19200, 38400 and 115200 at 30 MHz with the formula stock uses (OVSMOD 16
  unless stock differs), and check 9600 against stock.
- [ ] **Step 5: Commit** `docs/history/notes/oem-usart-sequences.md` ("notes: stock USART1/DMA/DE sequences for the Modbus driver").

Report anything that contradicts this plan's assumptions (for example, no DMAMUX, or DE being active low).

---

### Task 2: RTU framing (`oem/src/app/modbus_rtu.h`)

**Files:** Create `oem/src/app/modbus_rtu.h` and `tests/test_oem_modbus.cpp`; add `test_oem_modbus` to `tests/Makefile`.

**Interfaces (produces):**

```cpp
namespace oem {
constexpr size_t MB_MAX_FRAME = 256;
constexpr uint8_t MB_EX_ILLEGAL_FUNCTION = 1, MB_EX_ILLEGAL_ADDRESS = 2, MB_EX_ILLEGAL_VALUE = 3,
                  MB_EX_BUSY = 6;

// The register map behind the slave. Implemented by OemRegisters (Task 3) and by fakes in tests.
struct RegisterBank {
  virtual bool read_input(uint16_t reg, uint16_t &value) = 0;    // false: no such register
  virtual bool read_holding(uint16_t reg, uint16_t &value) = 0;  // false: no such register
  // Validate the whole block, then apply all of it or none. 0 = done, else a Modbus exception code (2, 3 or 6).
  virtual uint8_t write_holding(uint16_t start, const uint16_t *values, uint16_t count) = 0;
 protected:
  ~RegisterBank() = default;
};

// One request in, one response out. Returns the response length, 0 for no reply (bad CRC, short frame, another
// address, or broadcast).
size_t modbus_handle(uint8_t address, const uint8_t *req, size_t len, uint8_t *resp, RegisterBank &bank);

uint32_t modbus_gap_ms(uint32_t baud);  // M3

// Assembles frames from bytes and the silence after them (M3, M4).
class FrameAssembler {
 public:
  void set_gap_ms(uint32_t gap);
  void push(const uint8_t *data, size_t n, uint32_t now);  // bytes just received
  // A complete frame once gap_ms has passed since the last byte; returns its length (0: none yet). The frame stays valid
  // until the next push(). Frames longer than MB_MAX_FRAME are dropped whole.
  size_t poll(uint32_t now, const uint8_t *&frame);
  void discard_until(uint32_t t);  // M4: drop everything received before t, and any frame in progress
};
}
```

Rules for `modbus_handle`:
- **Discard silently:** frames shorter than 4 bytes, a bad CRC (CRC-16/Modbus over all but the last 2 bytes, low byte first,
  `crc16_modbus` from `app/crc16.h`), and an address that is neither ours nor 0.
- **FC 03 / 04:** `start`, `count` big-endian; `count` must be 1–125, else exception 03. Any register in the range unreadable
  → exception 02. Reply: addr, fc, byte count, the values big-endian, CRC.
- **FC 06:** the reply echoes the request (8 bytes) after `write_holding(reg, &v, 1)` returns 0. Otherwise an exception with
  that code.
- **FC 16:** `count` 1–123 and byte count = 2 × count and the frame length matching, else exception 03. Then
  `write_holding`. Reply: addr, 16, start, count, CRC.
- **Other function codes** → exception 01.
- **Exception reply:** addr, fc | 0x80, code, CRC.
- **Broadcast (address 0):** FC 06/16 are executed, but nothing is returned. Reads are ignored.

- [ ] **Step 1: Write the failing tests** in `tests/test_oem_modbus.cpp`, with a `FakeBank` (100 holding registers,
  50 input registers, a write log, an injectable exception code). Exact cases:
  - **CRC:** the reference frame `01 03 00 00 00 0A` has CRC `C5 CD`. Build it, then corrupt one byte → no reply.
  - **FC 03:** read HR 0–2 (values 10, 5760, 20) → `01 03 06 00 0A 16 80 00 14` + CRC.
  - **FC 04:** read IR 3 → value. Read 126 registers → exception 03.
  - **FC 03 range:** one register outside the bank → exception 02 (`01 83 02` + CRC).
  - **FC 06:** echo. Bank returns 3 → `01 86 03` + CRC, and nothing logged.
  - **FC 16:** 2 registers → reply `01 10 start count`. Byte-count mismatch → exception 03. Bank returns 6 → exception 06.
  - **Other function code:** FC 05 → exception 01.
  - **Address:** address 2 → no reply. Broadcast FC 06 → written, no reply. Broadcast FC 03 → no reply, no read.
  - **Short frame:** a 3-byte frame → no reply.
  - **`modbus_gap_ms`:** 9600 → 6, 19200 → 4, 38400 → 2, 115200 → 2.
  - **`FrameAssembler`:**
    - bytes at t = 0 and t = 3, polled at t = 8 with gap 6 → nothing; at t = 9 → the whole frame;
    - two frames separated by a gap → two polls;
    - 300 bytes without a gap → dropped, and the next frame after a gap is clean;
    - `discard_until(10)` drops bytes pushed at t < 10 and keeps later ones.
- [ ] **Step 2:** Run `make -C tests`: it fails to compile (no header).
- [ ] **Step 3:** Implement `modbus_rtu.h` (no heap; `FrameAssembler` holds a `uint8_t buf_[MB_MAX_FRAME]`).
- [ ] **Step 4:** Run `make -C tests`: all pass.
- [ ] **Step 5: Commit** ("OEM: Modbus RTU framing and function codes (host-tested)").

---

### Task 3: Register map (`oem/src/app/modbus_map.h`)

**Files:**
- Create `oem/src/app/modbus_map.h` and `tests/test_oem_modbus_map.cpp` (add to the Makefile).
- Modify `oem/src/app/settings.h`, `oem/src/app/controller.h` and `oem/src/app/relay.h`.

**Interfaces:**

```cpp
// settings.h
// True when s has a value outside its range or apply_clamps() would change it (M6).
inline bool violates_clamps(const Settings &s);

// controller.h
struct Status { ...; float cvl = NAN; float ccl = NAN; };  // V and A in use (set every tick from the limits)
bool Controller::module_cooling(int k) const;              // cooldown_[k].active()

// modbus_map.h
constexpr uint16_t OEM_FW_VERSION = 0x0300;
constexpr uint16_t HR_HEARTBEAT = 100, HR_EXT_FIRST = 101, HR_EXT_LAST = 107, HR_PAUSE = 110, HR_CLEAR_FAULT = 111,
                   HR_RELAY_CMD = 112, HR_CHARGE_NOW = 113, HR_REBOOT = 120;
constexpr uint16_t REBOOT_MAGIC = 0xB007;

struct Commands {         // what main.cpp applies after the reply has gone out (M8, M9)
  bool settings_changed;  // live settings were changed: mark dirty for the 5 s save
  bool pause_set; bool pause;
  bool clear_fault, charge_now, reboot;
  bool line_changed;      // HR 7 or 8 changed
};

struct Telemetry {        // live references main.cpp fills once; read on demand (M10)
  const Status *st; const Rectifier *psu[2]; const Controller *ctl;
  bool relay_closed; float adc_v; uint32_t uptime_s; uint32_t now_ms;
  bool menu_open;
};

class OemRegisters : public RegisterBank {
 public:
  OemRegisters(Settings &live, const Telemetry &t);
  bool read_input(uint16_t reg, uint16_t &v) override;
  bool read_holding(uint16_t reg, uint16_t &v) override;
  uint8_t write_holding(uint16_t start, const uint16_t *v, uint16_t n) override;
  Commands take_commands();   // returns and clears what accumulated
  bool relay_command() const;
  bool paused() const;        // last pause state set over Modbus or by sync_pause()
  void sync_pause(bool p);    // main keeps HR 110 in step with key pauses
};
```

**IR map** (spec §4, plus M10/M11): 0–18 and 20–39 exist, and 19 reads 0xFFFF. Scales are exactly as in the spec table:
- **Flags (IR 4):** b0 `st.cooling`, b1 `st.no_current`, b2 0 (Panel is never stale), b3 `st.session_active`,
  b4 `relay_closed`, b5 `st.online > 0`, b6/b7 PSU 1/2 `is_online()`.
- **IR 6:** `st.target_v` ×100 (0xFFFF when NaN).
- **IR 7:** `st.commanded_a` ×10.
- **IR 8:** `st.budget` ×10.
- **IR 9:** `st.output_a` ×10.
- **IR 10:** `st.power_w`.
- **IR 11:** `st.battery_v` ×100.
- **IR 12:** `adc_v` ×100.
- **IR 13:** `st.soc`, or 0xFFFF when < 0.
- **IR 14/15:** `st.cvl` ×100 and `st.ccl` ×10.
- **IR 16/17:** `uptime_s`, high word first.
- **PSU block:**
  - +0: b0 online, b1 faulted, b2 `module_cooling(k)`, b3 standby;
  - +1: Vout ×100; +2: Iout ×10; +3: AC V ×10;
  - +4: 0x7FFF; +5: heatsink °C ×10, signed; +6/+7: 0xFFFF;
  - +8/+9: alarms high / low 16 bits.
- **Rounding:** to nearest, then saturate (M10).

**HR map:**
- 0–16 = settings (M6, M7).
- 100–107 store and return the raw value.
- 110–113 and 120 per M9.
- Any other register → read false, write exception 02.
- A write block that spans a gap (for example 16–100) → exception 02, nothing written.

- [ ] **Step 1: Failing tests.** Use the phase-2 `Rig` pattern: `FakeBus`, two `FakeModule`s, two `Rectifier`s and a
  `Controller` ticked for a few seconds with `charge_now`, so `Status` holds real values. Cases:
  - **Status registers:**
    - IR 0 = 0x0300;
    - IR 1 = state 2 while charging;
    - IR 3 = 17 with `adc_v` = 0;
    - IR 4 flag bits as above;
    - IR 6 = 5760 while charging;
    - IR 9 = 100 at 10 A;
    - IR 11 = round(battery_v × 100);
    - IR 13 = 0xFFFF;
    - IR 16/17 split of uptime 70000 s → 1, 4464.
  - **PSU block:**
    - IR 20 bits for an online module in standby;
    - IR 21 = Vout ×100;
    - IR 25 = 350 for 35.0 °C;
    - IR 24 = 0x7FFF;
    - IR 26 = 0xFFFF;
    - IR 28/29 for alarm 0x00010001;
    - IR 19 = 0xFFFF;
    - IR 40 → false.
  - **NaN handling:** a NaN target reads 0xFFFF; a negative signed value reads as int16 (e.g. −5.0 °C → 0xFFCE).
  - **Settings writes:**
    - HR 0 = 20 → settings changed, `take_commands().settings_changed`;
    - HR 0 = 101 → exception 03, unchanged;
    - HR 5 = 5790 (> Charge V) → exception 03;
    - FC 16 HR 1–5 with one bad value → exception 03 and nothing changed;
    - HR 6 = 1 → exception 03 (M1);
    - HR 7 = 5 → `line_changed`;
    - HR 0 with `menu_open` → exception 06.
  - **Runtime registers:**
    - HR 110 = 1 → `pause_set && pause`, and reading HR 110 → 1;
    - HR 110 = 2 → exception 03;
    - HR 111 = 1 → `clear_fault`;
    - HR 113 = 1 → `charge_now`;
    - HR 120 = 0xB007 → `reboot`, and = 1 → exception 03;
    - HR 112 = 1 → `relay_command()`;
    - HR 101 = 5500 then read → 5500;
    - HR 99 → exception 02;
    - `take_commands()` clears.
  - **Clamps:**
    - `violates_clamps` false for `default_settings()`;
    - true for Stop V > Charge V;
    - true for a value out of range;
    - true for Start SOC ≥ Stop SOC.
  - **Relay:** Modbus mode with command on → closed; switch to Auto with session active at t → closed, then session ends →
    open 10 s after the end, not at once (M12).
  - **Wire-level round trip:** `modbus_handle` with `OemRegisters` reads IR 0–39 in one request (80 data bytes).
- [ ] **Step 2:** Run `make -C tests`: it fails.
- [ ] **Step 3:** Implement. In the controller tick, fill `st_.cvl` from `charge::target_voltage(lim.cvl, Charge V)`, which is
  the same value as `target_v` but also set when not charging, and `st_.ccl` from `lim.ccl`.
- [ ] **Step 4:** Run `make -C tests`: all pass, and phase-2 tests unchanged.
- [ ] **Step 5: Commit** ("OEM: Modbus register map over the live status and settings").

---

### Task 4: USART1 driver (`oem/src/hw/rs485.h/.cpp`)

**Files:**
- Create `oem/src/hw/rs485.h` and `oem/src/hw/rs485.cpp`.
- Modify `oem/src/hw/regs.h` (USART1, DMA ch / DMAMUX per Task 1), `oem/src/startup.c` (IRQ28 → `USART1_IRQHandler`) and
  `oem/CMakeLists.txt` (`rs485.cpp` into `oem_drivers`).

**Interfaces:**

```cpp
namespace hw {
bool rs485_init(uint32_t baud);         // USART1 8N1 on PA2/PA3, DE PB10 low, RX circular DMA, IRQ28 enabled
void rs485_set_baud(uint32_t baud);     // only while !rs485_tx_busy()
size_t rs485_read(uint8_t *buf, size_t max);  // bytes received since the last call (from the DMA ring)
bool rs485_send(const uint8_t *data, size_t n);  // copies up to 256 bytes, DE high, starts TX; false if busy
bool rs485_tx_busy();                   // true until TC after the last byte, when the ISR drops DE
uint32_t rs485_tx_done_ms();            // millis() when the last TX completed (for FrameAssembler::discard_until)
uint32_t rs485_rx_overruns();           // count of DMA ring overruns or ORERR
}
```

Design:
- **RX ring:** 256 bytes, circular DMA as Task 1 found. `rs485_read` compares the DMA CNT with the last position.
  - If more than 256 − 16 bytes arrived since the last call, count an overrun and resynchronise to the current position.
- **TX:** a static 256-byte buffer. `rs485_send` sets DE (PB10) high, waits ≥ 1 µs, writes the first byte and enables TBEIE.
  - The ISR writes the next byte on TBE. After the last byte it disables TBEIE and enables TCIE.
  - On TC it disables TCIE, clears TC, drops DE, records `tx_done_ms` and clears busy.
  - The busy flag is `volatile`, and the main loop only reads it.
- **Errors:** clear ORERR / FERR / NERR on every `rs485_read` per Task 1.
- **Pins:** PB0 stays a high-Z input (phase-1 takeover); `rs485_init` must not touch it.

- [ ] **Step 1:** Implement from `docs/history/notes/oem-usart-sequences.md`.
- [ ] **Step 2:** Add a `rs485_loopback` bench hook in the `bringup` app (Enter on the RX screen sends `"U"` × 16 at 9600).
  Optional, if it keeps `bringup` under 8 KB.
- [ ] **Step 3:** Cross build with no warnings. Check the disassembly: `USART1_IRQHandler` is at vector 16 + 28, and the
  handler doesn't call `fwdgt_feed`.
- [ ] **Step 4: Commit** ("OEM: USART1 RS485 driver (DMA RX, interrupt TX, DE on TC)").

---

### Task 5: The Modbus task in `main.cpp`

**Files:** Modify `oem/src/main.cpp`.

**Behaviour, every pass:**
1. `n = hw::rs485_read(...)`, then `assembler.push(..., now)`.
2. If not `rs485_tx_busy()` and `assembler.poll(now, frame)` gives a frame: `len = modbus_handle(settings address, ...)`.
   If `len > 0`, `hw::rs485_send(resp, len)`.
3. When a TX completes (busy → idle), `assembler.discard_until(rs485_tx_done_ms() + gap)` (M4). Then apply deferred line
   changes (M8):
   - `rs485_set_baud(...)`;
   - `assembler.set_gap_ms(...)`;
   - the new address applies to the next frame.

   Then a pending reboot: `hw::system_reset()`.
4. `take_commands()` after each handled frame:
   - `settings_changed` → `dirty = true; changed_ms = now`;
   - `pause_set` → `g_ctl.set_paused(pause)` and `hw::set_backup_word(pause_word(...))`;
   - `clear_fault` → `g_ctl.clear_fault()`;
   - `charge_now` → `g_ctl.charge_now()`;
   - `reboot` and `line_changed` → remembered until TX completes.

   With broadcast there is no TX, so apply at once.
5. **Key pauses:** call `g_regs.sync_pause(g_ctl.paused())` after a key toggle, so HR 110 reads true.
6. **Relay:** `g_relay.update(..., modbus_cmd = g_regs.relay_command(), ...)`.
7. **Telemetry:** filled once before the loop with pointers. `relay_closed`, `adc_v`, `uptime_s`, `now_ms` and `menu_open` are
   updated each pass before step 2.
8. **Startup:** `rs485_init(baud from HR 8)` after the settings load. A false return leaves Modbus off, and phase-2 behaviour
   continues.

- [ ] **Step 1:** Implement.
- [ ] **Step 2:** Cross build with no warnings and no heap. Report the size (budget: < 32 KB).
- [ ] **Step 3: Commit** ("OEM charger: Modbus RTU on the RS485 jack").

---

### Task 6: Mac tools and docs

**Files:**
- Create `tools/oem_modbus.py` and `tools/test_oem_modbus.py`.
- Modify `tools/oem_update.py`, `tools/test_oem_update.py` and `README.md`.

**`tools/oem_modbus.py`:**
- Built on pyserial, using `crc16_modbus` from `oem_image`.
- The port opens with RTS/DTR off.
- Options: `--port`, `--baud 9600`, `--addr 1`, `--timeout 0.5`.
- Subcommands:
  - `status [--watch SECONDS]`: one FC 04 read of IR 0–39, printed decoded and scaled, with state, hold reason and E-code
    names;
  - `get REG...`: holding registers;
  - `set REG=VALUE...`: FC 06 for one register, FC 16 for consecutive registers;
  - `pause on|off`, `charge-now`, `clear-fault`, `relay on|off`;
  - `reboot-bootloader`.
- An exception reply prints the code's name and exits 1. No reply exits 2.

**`oem_update.py --modbus-reboot`** (with `--modbus-baud` and `--modbus-addr`): before the handshake, send FC 06 HR 120 =
0xB007 at that baud and address, wait for the echo reply (warn if none, since the board may be running stock), then reopen
at 460800 for the usual handshake.

- [ ] **Step 1: Failing tests:**
  - request bytes for FC 03/04/06/16 (the CRC reference `01 03 00 00 00 0A C5 CD`);
  - response parsing, including an exception reply and a bad CRC;
  - the `status` decoder on a canned 80-byte IR block, with NaN markers shown as `n/a`;
  - `--modbus-reboot` sends `01 06 00 78 B0 07` + CRC first (fake serial).
- [ ] **Step 2:** Implement.
- [ ] **Step 3:** README "Modbus" subsection in the OEM section: wiring (rear RJ45 RS485 jack, 9600 8N1, address 1), the tool,
  and the register summary pointing to spec §4.
- [ ] **Step 4:** `.venv/bin/python -m pytest tools/ -q` passes.
- [ ] **Step 5: Commit** ("tools: oem_modbus.py client; oem_update --modbus-reboot").

---

### Task 7: [BENCH] Modbus on the board

Needs the user: the board powered and the FTDI adapter on the RS485 jack.

- [ ] **Step 1:** Install `charger.dat`, by power-cycle or an Up+Down watchdog reset.
- [ ] **Step 2:** `oem_modbus.py --port … status`.
  - Expected: version 3.0, the state, E17 on 5 V, ADC battery V, flags.
- [ ] **Step 3:** Settings:
  - `get 0 1 5`;
  - `set 0=12` → saved; still 12 after a power-cycle;
  - `set 5=5790` → exception 03 (illegal value);
  - `set 6=1` → exception 03.
- [ ] **Step 4:** `pause on` → status shows paused, BKP4 "PAUS" (SWD if reachable); `pause off`.
- [ ] **Step 5:** `oem_update.py --modbus-reboot oem/build/charger.dat` → installs without a power-cycle.
- [ ] **Step 6:** With fake modules (CANable), if available: per-PSU telemetry matches the fake's log.
- [ ] **Step 7:** Record in `docs/history/notes/oem-bringup.md` ("Modbus") and commit.
