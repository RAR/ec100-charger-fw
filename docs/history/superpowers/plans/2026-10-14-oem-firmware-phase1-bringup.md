# OEM Board Firmware, Phase 1 (Bring-up) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Get our own image built, installed through the stock RS485 bootloader, and booting on the bench board. It must survive the
watchdog, drive the LCD and keys, and identify the unknown GPIOs. Reverting to the stock firmware must be proven to work.

**Architecture:**
- A C startup file and a C++17 bare-metal app linked at 0x08005000, behind the stock bootloader.
- Hardware access goes through a small `regs.h` of literal addresses taken from the stock firmware (`docs/history/notes/oem-init-sequences.md`).
- Hardware-free logic (keys, LCD rendering) is header-only and host-tested by `make -C tests`.
- Python tools build the encrypted `.dat` image and talk to the bootloader. They are tested against a byte-accurate fake bootloader.

**Tech Stack:**
- Arm GNU Toolchain 15.3.Rel2 (installed at `~/.local/arm-gnu-toolchain-15.3.rel2`), CMake ≥ 3.20, newlib-nano.
- Python 3 in the repo `.venv` (pyserial 3.5, pytest), and the existing `tools/tea_decrypt.py`.

**Spec:** `docs/history/superpowers/specs/2026-10-14-oem-board-firmware-design.md` (phase 1 of §7). Hardware references: `docs/history/notes/oem-hw-map.md` and
`docs/history/notes/oem-init-sequences.md`.

**Spec deviation, decided:** Phase 1 uses our own `oem/src/hw/regs.h` instead of GigaDevice's CMSIS device header.
- The public v1.0 header predates the GD32L235, and v2.4.0 sits behind a licence click that only the user can accept.
- Every register phase 1 needs is listed with its stock-firmware evidence in `docs/history/notes/oem-init-sequences.md`.
- Phase 2 (CAN) revisits this once the user has downloaded v2.4.0.

## Global Constraints

- **App placement:** linked at **0x08005000**. Image ≤ **0x19000 bytes (100 KB)** including the 8-byte trailer. Never touch
  0x08000000–0x08004FFF (bootloader) or the **0x0801E000** page.
- **Startup:** `Reset_Handler` sets **VTOR = 0x08005000 first**. The bootloader's SysTick is still running and would otherwise vector into
  the bootloader.
- **Watchdog:** FWDGT is already running (PSC /32, RLD 0x800, about 2 s) and can't be stopped. Feed it (`FWDGT_CTL = 0xAAAA`) only from
  the main loop, never from an interrupt.
- **Image trailer:** the last 8 bytes must not be all 0xFF, otherwise the bootloader won't jump. `oem_image.py` appends `'O' 'E' 'M'`,
  major, minor, 0, CRC16-LE.
- **Bootloader frames are never resent.** Any update error aborts, and the install restarts from a board reset.
- **Clocks:** 12 MHz HXTAL × 5 = 60 MHz SYSCLK/AHB, APB1 30 MHz, FMC_WS = 2. If HXTAL fails, run on IRC16M and report it.
- **Compile flags:** C++17, `-mcpu=cortex-m23 -mthumb -Os`, `-Wall -Wextra -Werror`, `-fno-exceptions -fno-rtti`. No heap use.
- **Host tests:** `make -C tests` (with `-Werror`) and `.venv/bin/python -m pytest tools/` must pass after every task.
- **Bench safety:** steps marked **[BENCH]** need the user and the physical board, with **no rectifier modules connected** during
  phase 1. Phase 1 never sends anything on CAN.
- Comments match the repo's existing style: sparse, and they explain why.

## File structure

| File | Responsibility |
|---|---|
| `tools/oem_image.py` | CRC16 (Modbus); build pad + trailer + TEA encrypt → `.dat`; stock-image extraction from a flash dump |
| `tools/test_oem_image.py` | pytest for the above |
| `tools/oem_update.py` | Bootloader client: handshake, header, data frames, abort-on-error |
| `tools/test_oem_update.py` | pytest with `FakeBootloader` (byte-accurate per `docs/history/notes/oem-hw-map.md` §6.2) |
| `oem/CMakeLists.txt`, `oem/cmake/arm-none-eabi.cmake` | Cross build; post-build `.bin` + `.dat` |
| `oem/ld/app.ld` | Memory map, sections, RAM/stack assert |
| `oem/src/startup.c` | Vector table, `Reset_Handler` |
| `oem/src/hw/regs.h` | Register addresses + `reg()`/`rmw()` |
| `oem/src/hw/gpio.h` | Pin type, `pin_config/pin_write/pin_read` |
| `oem/src/hw/clock.h/.cpp` | Clock tree, FWDGT, SysTick, `millis()`, reset flags, `system_reset()` |
| `oem/src/hw/slcd.h/.cpp` | Segment-LCD GPIO, backup-domain and controller bring-up; `slcd_write()` |
| `oem/src/board.h` | Pin assignments for the v1 board |
| `oem/src/app/keys.h` | Key debounce / click / long / repeat state machine (header-only, host-tested) |
| `oem/src/app/lcd_render.h` | Fonts, field map, frame composition (header-only, host-tested) |
| `oem/src/blink.cpp` | Task 4's minimal app (LED blink) |
| `oem/src/bringup.cpp` | Task 8's bring-up app (screens, probes, watchdog/bootloader tests) |
| `tests/test_oem_keys.cpp`, `tests/test_oem_lcd_render.cpp` | Host tests |
| `docs/history/notes/oem-bringup.md` | Bench results (Tasks 3, 5 and 9) |

---

### Task 1: Image builder (`tools/oem_image.py`)

**Files:**
- Create: `tools/oem_image.py`, `tools/test_oem_image.py`

**Interfaces:**
- Consumes: `tools/tea_decrypt.py`: `crypt(buf, fn)`, `enc_block`, `dec_block`.
- Produces:
  - `crc16_modbus(data: bytes) -> int`
  - `build_image(raw: bytes, major: int, minor: int) -> bytes` (plaintext, padded, with trailer)
  - `stock_from_dump(dump: bytes) -> bytes` (plaintext stock app)
  - `encrypt(img: bytes) -> bytes`
  - Constants `APP_BASE = 0x08005000`, `LEN_ADDR = 0x0801E000`, `MAX_LEN = 0x19000`.

- [ ] **Step 1: Write the failing tests**

```python
# tools/test_oem_image.py
import os
import struct

import pytest

from oem_image import MAX_LEN, build_image, crc16_modbus, encrypt, stock_from_dump
from tea_decrypt import crypt, dec_block

DUMP = os.path.join(os.path.dirname(__file__), "..", "dumps", "gd32_flash_20261012.bin")


def test_crc16_modbus_check_value():
    assert crc16_modbus(b"123456789") == 0x4B37


def test_build_pads_and_appends_trailer():
    img = build_image(b"\x01" * 13, 0, 1)
    assert len(img) % 8 == 0
    assert img[:13] == b"\x01" * 13 and img[13:16] == b"\x00" * 3
    assert img[-8:-5] == b"OEM" and img[-5] == 0 and img[-4] == 1 and img[-3] == 0
    assert struct.unpack("<H", img[-2:])[0] == crc16_modbus(img[:-8])
    assert img[-8:] != b"\xff" * 8


def test_build_rejects_oversize():
    with pytest.raises(ValueError):
        build_image(b"\x00" * MAX_LEN, 0, 1)


def test_encrypt_round_trips():
    img = build_image(bytes(range(256)) * 3, 1, 2)
    assert crypt(encrypt(img), dec_block) == img


def test_stock_from_synthetic_dump():
    dump = bytearray(b"\xff" * 0x20000)
    app = bytes(range(64))
    dump[0x5000:0x5000 + len(app)] = app
    dump[0x1E000:0x1E008] = struct.pack(">I", len(app)) + b"\x00" * 4
    assert stock_from_dump(bytes(dump)) == app


@pytest.mark.skipif(not os.path.exists(DUMP), reason="flash dump not present")
def test_stock_from_real_dump():
    img = stock_from_dump(open(DUMP, "rb").read())
    assert len(img) == 48984                                   # 0x0801E000 holds 00 00 BF 58
    assert struct.unpack_from("<I", img, 0)[0] == 0x20001448   # v1 app's initial SP
```

- [ ] **Step 2: Run the tests and check they fail**

Run: `.venv/bin/python -m pytest tools/test_oem_image.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'oem_image'`.

- [ ] **Step 3: Implement**

```python
#!/usr/bin/env python3
"""Build an install image (.dat) for the stock Eco-Worthy bootloader.

The bootloader installs at 0x08005000, refuses to jump if the image's last 8 bytes are all 0xFF, and decrypts with
8-round TEA (see tea_decrypt.py). We pad to a multiple of 8 and append an 8-byte trailer
('OEM', major, minor, 0, CRC16-LE of the padded body) so the last word is never erased-looking.

Usage:
  oem_image.py app.bin out.dat --version 0.1
  oem_image.py --stock-from-dump dumps/gd32_flash_20261012.bin out.dat   # re-encrypt the stock v1 app
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tea_decrypt import crypt, enc_block  # noqa: E402

FLASH_BASE = 0x08000000
APP_BASE = 0x08005000
LEN_ADDR = 0x0801E000
MAX_LEN = LEN_ADDR - APP_BASE  # 0x19000


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_image(raw, major, minor):
    body = raw + b"\x00" * (-len(raw) % 8)
    img = body + b"OEM" + bytes([major, minor, 0]) + struct.pack("<H", crc16_modbus(body))
    if len(img) > MAX_LEN:
        raise ValueError(f"image is {len(img)} bytes; the bootloader takes at most {MAX_LEN}")
    return img


def stock_from_dump(dump):
    length = struct.unpack_from(">I", dump, LEN_ADDR - FLASH_BASE)[0]
    if not 8 < length <= MAX_LEN or length % 8:
        raise ValueError(f"implausible stock image length {length:#x}")
    img = dump[APP_BASE - FLASH_BASE:APP_BASE - FLASH_BASE + length]
    if img[-8:] == b"\xff" * 8:
        raise ValueError("stock image ends in erased flash")
    return img


def encrypt(img):
    return crypt(img, enc_block)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile", nargs="?")
    ap.add_argument("outfile")
    ap.add_argument("--version", default="0.0", help="major.minor, stored in the trailer")
    ap.add_argument("--stock-from-dump", metavar="DUMP", help="extract the stock app from a full flash dump instead")
    a = ap.parse_args()
    if a.stock_from_dump:
        img = stock_from_dump(open(a.stock_from_dump, "rb").read())
    else:
        if not a.infile:
            ap.error("infile is required unless --stock-from-dump is given")
        major, minor = (int(x) for x in a.version.split("."))
        img = build_image(open(a.infile, "rb").read(), major, minor)
    open(a.outfile, "wb").write(encrypt(img))
    print(f"wrote {a.outfile}: {len(img)} bytes ({len(img) * 100 // MAX_LEN}% of the app area)")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `.venv/bin/python -m pytest tools/test_oem_image.py -q`
Expected: `6 passed` (5 if the dump is absent, with 1 skipped).

- [ ] **Step 5: Ignore build output.** Append `/build/` to the repo-root `.gitignore`. Tasks 3 and 9 write restore images and SWD
  read-backs there.

- [ ] **Step 6: Commit**

```bash
git add tools/oem_image.py tools/test_oem_image.py .gitignore
git commit -m "oem_image.py: pad, trailer and TEA-encrypt images for the stock bootloader; extract the stock app from a dump"
```

---

### Task 2: Bootloader client (`tools/oem_update.py`)

**Files:**
- Create: `tools/oem_update.py`, `tools/test_oem_update.py`

**Interfaces:**
- Consumes: `oem_image.crc16_modbus`, and `tea_decrypt.crypt` / `dec_block` (in the fake only).
- Produces:
  - `header_frame(length: int) -> bytes`
  - `data_frame(seq: int, chunk: bytes) -> bytes`
  - `install(port, dat: bytes, wait_s=30.0, clock=time.monotonic, sleep=time.sleep, log=print) -> None`, which raises
    `UpdateError` on any failure.
  - `port` is anything with `write(bytes)`, `read(n) -> bytes` (may return fewer bytes) and `reset_input_buffer()`.

- [ ] **Step 1: Write the failing tests**

```python
# tools/test_oem_update.py
import itertools
import struct

import pytest

from oem_image import crc16_modbus, encrypt
from oem_update import HANDSHAKE, HANDSHAKE_REPLY, UpdateError, data_frame, header_frame, install
from tea_decrypt import crypt, dec_block


class FakeBootloader:
    """Byte-level model of the stock bootloader (docs/history/notes/oem-hw-map.md §6.2): it acks the handshake, takes the first
    4-byte frame as the header, decrypts and programs every CRC-good data frame at the running offset (no sequence
    check), and acks with the sequence number echoed."""

    def __init__(self, silent=False, drop_ack_seq=None):
        self.flash = bytearray(b"\xff" * 0x19000)
        self.length = None
        self.offset = 0
        self.silent = silent
        self.drop_ack_seq = drop_ack_seq
        self.out = bytearray()

    def reset_input_buffer(self):
        self.out.clear()

    def read(self, n):
        r = bytes(self.out[:n])
        del self.out[:n]
        return r

    def write(self, b):
        b = bytes(b)
        if self.silent:
            return
        if b == HANDSHAKE:
            self.out += HANDSHAKE_REPLY
            return
        n = struct.unpack(">H", b[1:3])[0]
        assert b[0] == 0xFB and b[3:5] == b"\x03\x00" and len(b) == n + 9
        if crc16_modbus(b[:n + 7]) != struct.unpack("<H", b[n + 7:])[0]:
            return  # bad CRC: no reply
        seq = b[5:7]
        if self.length is None:
            assert n == 4
            self.length = struct.unpack(">I", b[7:11])[0]
            self.flash[:] = b"\xff" * len(self.flash)  # erase on the first header
        else:
            self.flash[self.offset:self.offset + n] = crypt(b[7:7 + n], dec_block)
            self.offset += n
            if struct.unpack(">H", seq)[0] == self.drop_ack_seq:
                return
        self.out += b"\xfb\x03\x00" + seq + b"\x00\x00"


def fake_time():
    ticks = itertools.count()
    return (lambda: next(ticks) * 0.01), (lambda s: None)


def test_header_frame_bytes():
    f = header_frame(0xBF58)
    assert f[:11] == bytes.fromhex("FB0004030000000000BF58")
    assert struct.unpack("<H", f[11:])[0] == crc16_modbus(f[:11])
    assert len(f) == 13


def test_data_frame_layout():
    f = data_frame(7, b"\xaa" * 16)
    assert f[:7] == bytes.fromhex("FB001003000007")
    assert f[7:23] == b"\xaa" * 16 and len(f) == 25


def test_install_programs_the_plaintext():
    plain = bytes((i * 7) & 0xFF for i in range(1000)) + b"\x00" * 8  # 1008 bytes, a multiple of 8
    fake = FakeBootloader()
    clock, sleep = fake_time()
    install(fake, encrypt(plain), clock=clock, sleep=sleep, log=lambda *_: None)
    assert fake.length == len(plain)
    assert bytes(fake.flash[:len(plain)]) == plain
    assert fake.offset == len(plain)


def test_lost_ack_aborts_without_resending():
    fake = FakeBootloader(drop_ack_seq=2)
    clock, sleep = fake_time()
    with pytest.raises(UpdateError):
        install(fake, encrypt(b"\x11" * 1024), clock=clock, sleep=sleep, log=lambda *_: None)
    assert fake.offset == 512  # frames 1 and 2 programmed; frame 2 was not sent again


def test_no_bootloader_times_out():
    clock, sleep = fake_time()
    with pytest.raises(UpdateError):
        install(FakeBootloader(silent=True), encrypt(b"\x00" * 64), wait_s=1.0, clock=clock, sleep=sleep,
                log=lambda *_: None)


def test_rejects_unaligned_image():
    clock, sleep = fake_time()
    with pytest.raises(UpdateError):
        install(FakeBootloader(), b"\x00" * 13, clock=clock, sleep=sleep, log=lambda *_: None)
```

- [ ] **Step 2: Run the tests and check they fail**

Run: `.venv/bin/python -m pytest tools/test_oem_update.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'oem_update'`.

- [ ] **Step 3: Implement**

```python
#!/usr/bin/env python3
"""Install a .dat image through the stock Eco-Worthy bootloader (RS485, 460800 8N1).

Protocol (docs/history/notes/oem-hw-map.md §6.2):
- Handshake: F9 45 F9 53 23 -> F9 45 03 00 21 9D.
- Header: a frame carrying the 4-byte big-endian image length. The first one after a reset erases the app area.
- Data: frames of <= 256 encrypted bytes, each acked with its sequence number echoed.

The bootloader programs every CRC-good frame at a running offset without checking the sequence number, so a frame
is never resent. Any error aborts, and the install must restart from a board reset.

Usage: oem_update.py --port /dev/cu.usbserial-XXXX image.dat [--wait 30]
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oem_image import crc16_modbus  # noqa: E402

BAUD = 460800
HANDSHAKE = bytes.fromhex("F945F95323")
HANDSHAKE_REPLY = bytes.fromhex("F9450300219D")
FRAME_BYTES = 256
HEADER_ACK_S = 5.0  # erasing 100 pages
DATA_ACK_S = 1.0
QUIET_S = 2.0  # > 1.5 s of silence makes the bootloader jump to the app


class UpdateError(Exception):
    pass


def data_frame(seq, chunk):
    body = b"\xfb" + struct.pack(">H", len(chunk)) + b"\x03\x00" + struct.pack(">H", seq) + chunk
    return body + struct.pack("<H", crc16_modbus(body))


def header_frame(length):
    return data_frame(0, struct.pack(">I", length))


def read_exact(port, n, timeout, clock):
    buf = b""
    end = clock() + timeout
    while len(buf) < n and clock() < end:
        buf += port.read(n - len(buf))
    return buf


def handshake(port, wait_s, clock, sleep, log):
    # The stock app answers once, then resets into the bootloader, so a reply only counts once it has come twice.
    log(f"waiting up to {wait_s:.0f} s for the bootloader: reset or power-cycle the board now")
    end = clock() + wait_s
    replies = 0
    while clock() < end:
        port.reset_input_buffer()
        port.write(HANDSHAKE)
        if read_exact(port, len(HANDSHAKE_REPLY), 0.1, clock) == HANDSHAKE_REPLY:
            replies += 1
            if replies == 2:
                return
            sleep(0.2)
    raise UpdateError("no bootloader handshake")


def install(port, dat, wait_s=30.0, clock=time.monotonic, sleep=time.sleep, log=print):
    if not dat or len(dat) % 8:
        raise UpdateError(f"image length {len(dat)} is not a positive multiple of 8")
    handshake(port, wait_s, clock, sleep, log)
    port.reset_input_buffer()
    port.write(header_frame(len(dat)))
    ack = read_exact(port, 7, HEADER_ACK_S, clock)
    if ack[:5] != b"\xfb\x03\x00\x00\x00":
        raise UpdateError(f"no header ack (got {ack.hex() or 'nothing'})")
    frames = (len(dat) + FRAME_BYTES - 1) // FRAME_BYTES
    for seq, off in enumerate(range(0, len(dat), FRAME_BYTES), start=1):
        port.write(data_frame(seq, dat[off:off + FRAME_BYTES]))
        ack = read_exact(port, 7, DATA_ACK_S, clock)
        if ack[:5] != b"\xfb\x03\x00" + struct.pack(">H", seq):
            raise UpdateError(f"frame {seq}/{frames}: bad or missing ack (got {ack.hex() or 'nothing'}); "
                              "reset the board and start again")
        if seq % 20 == 0 or seq == frames:
            log(f"  {seq}/{frames} frames")
    sleep(QUIET_S)
    log("done: the bootloader should now start the new image")


def main():
    import serial  # pyserial; imported here so the tests don't need a port

    ap = argparse.ArgumentParser()
    ap.add_argument("image", help=".dat file (already encrypted, e.g. from oem_image.py)")
    ap.add_argument("--port", required=True)
    ap.add_argument("--wait", type=float, default=30.0, help="seconds to keep trying the handshake")
    a = ap.parse_args()
    dat = open(a.image, "rb").read()
    with serial.Serial(a.port, BAUD, timeout=0.02) as port:
        try:
            install(port, dat, wait_s=a.wait)
        except UpdateError as e:
            sys.exit(f"update failed: {e}")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `.venv/bin/python -m pytest tools/test_oem_update.py tools/test_oem_image.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add tools/oem_update.py tools/test_oem_update.py
git commit -m "oem_update.py: install a .dat through the stock RS485 bootloader; no resends, abort on any error"
```

---

### Task 3: [BENCH] Prove the tools on the real bootloader by reinstalling stock v1

This task needs the user. The executor prepares the commands, and the controller walks the user through them. The bench board
must have no rectifier modules connected.

**Files:**
- Create: `docs/history/notes/oem-bringup.md`

- [ ] **Step 1: Bench setup (user).**
  - Power the board from a current-limited bench supply on the +48 V / GND pins of the power connector: 48 V, limit 0.3 A.
  - Connect a USB-RS485 adapter to J5: A↔A, B↔B, GND↔GND.
  - Connect the ST-Link to J3 (`+ DIO CLK −`).
  - Note the adapter's port, e.g. `ls /dev/cu.usbserial*`.

- [ ] **Step 2: Take a fresh SWD backup (read-only).**

```bash
openocd -f interface/stlink.cfg -c "transport select hla_swd" -c "reset_config none" -c "set CPUTAPID 0" \
  -f target/stm32f1x.cfg -c "\$_TARGETNAME configure -event examine-end {}" \
  -c "init; halt; dump_image dumps/gd32_flash_$(date +%Y%m%d)-pre-oem.bin 0x08000000 0x20000; resume; exit"
cmp dumps/gd32_flash_20261012.bin dumps/gd32_flash_$(date +%Y%m%d)-pre-oem.bin && echo identical
```

  Expected: the dump is written. `cmp` may report differences only in 0x0801F000–0x0801F3FF (the settings log). Never use this
  openocd config to *write* flash.

- [ ] **Step 3: Build the stock restore image.**

```bash
mkdir -p build && .venv/bin/python tools/oem_image.py --stock-from-dump dumps/gd32_flash_20261012.bin build/stock-v1.dat
```

  Expected: `wrote build/stock-v1.dat: 48984 bytes`.

- [ ] **Step 4: Install it (user power-cycles when prompted).**

```bash
.venv/bin/python tools/oem_update.py --port /dev/cu.usbserial-XXXX build/stock-v1.dat
```

  Expected: the handshake succeeds, 192/192 frames are acked, and "done". The board then boots the stock UI as before.

  If it fails: power-cycle and run the same command again. The board stays in its bootloader until an install completes. The SWD
  backup from Step 2 is the last resort.

- [ ] **Step 5: Verify over SWD that the app area matches the dump.**

```bash
openocd -f interface/stlink.cfg -c "transport select hla_swd" -c "reset_config none" -c "set CPUTAPID 0" \
  -f target/stm32f1x.cfg -c "\$_TARGETNAME configure -event examine-end {}" \
  -c "init; halt; dump_image build/after-restore.bin 0x08005000 0xBF58; resume; exit"
cmp build/after-restore.bin <(dd if=dumps/gd32_flash_20261012.bin bs=1 skip=$((0x5000)) count=$((0xBF58)) 2>/dev/null) && echo match
```

  Expected: `match`.

- [ ] **Step 6: Record the result.** Create `docs/history/notes/oem-bringup.md` with a "Bootloader round trip" section covering the date, the adapter
  used, frame count, time taken, whether the stock UI came back, and the SWD compare result.

- [ ] **Step 7: Commit**

```bash
git add docs/history/notes/oem-bringup.md
git commit -m "OEM bring-up: stock v1 reinstalled through the bootloader with oem_update.py"
```

---

### Task 4: Firmware skeleton: build, startup, clock, watchdog, blink

**Files:**
- Create: `oem/CMakeLists.txt`, `oem/cmake/arm-none-eabi.cmake`, `oem/ld/app.ld`, `oem/src/startup.c`, `oem/src/hw/regs.h`,
  `oem/src/hw/gpio.h`, `oem/src/hw/clock.h`, `oem/src/hw/clock.cpp`, `oem/src/board.h`, `oem/src/blink.cpp`, `oem/.gitignore`

**Interfaces:**
- Produces, for Tasks 7 and 8:
  - `hw::reg(addr)` and `hw::rmw(addr, mask, value)`
  - `hw::Pin{port, n}`, `hw::pin_config(Pin, Mode, Pull, af=0)`, `hw::pin_write(Pin, bool)`, `hw::pin_read(Pin)`
  - `hw::clock_init() -> bool`, `hw::core_hz()`, `hw::fwdgt_init()`, `hw::fwdgt_feed()`, `hw::systick_init()`, `hw::millis()`,
    `hw::reset_flags()`, `hw::system_reset()`
  - `board::*` pins, and `board::enable_gpio_clocks()`
- Build output: `oem/build/<app>.elf/.bin/.dat` for each app target.

- [ ] **Step 1: Toolchain file and CMake project**

```cmake
# oem/cmake/arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
if(DEFINED ENV{ARM_TOOLCHAIN})
  set(ARM_TOOLCHAIN "$ENV{ARM_TOOLCHAIN}")
else()
  set(ARM_TOOLCHAIN "$ENV{HOME}/.local/arm-gnu-toolchain-15.3.rel2")
endif()
set(CMAKE_C_COMPILER "${ARM_TOOLCHAIN}/bin/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${ARM_TOOLCHAIN}/bin/arm-none-eabi-g++")
set(CMAKE_OBJCOPY "${ARM_TOOLCHAIN}/bin/arm-none-eabi-objcopy")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
```

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

set(HW_SOURCES src/startup.c src/hw/clock.cpp)

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
```

```gitignore
# oem/.gitignore
/build/
```

- [ ] **Step 2: Linker script**

```ld
/* oem/ld/app.ld: our app behind the stock bootloader (0x08000000-0x08004FFF). The bootloader stores the image length
   at 0x0801E000, so the app must end before it; oem_image.py appends an 8-byte trailer after the binary. */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08005000, LENGTH = 0x18FF8   /* 0x19000 minus the trailer */
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 16K       /* conservative: the stock app's stack top is 0x20001448 */
}

_stack_top = ORIGIN(RAM) + LENGTH(RAM);
_min_stack = 0x1000;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

  .text :
  {
    *(.text*)
    *(.rodata*)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    __preinit_array_start = .;
    KEEP(*(.preinit_array))
    __preinit_array_end = .;
    __init_array_start = .;
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array))
    __init_array_end = .;
    . = ALIGN(4);
  } > FLASH

  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT > FLASH

  .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
  PROVIDE(end = _ebss);

  ASSERT(_ebss + _min_stack <= _stack_top, "RAM: less than 4 KB left for the stack")
}
```

- [ ] **Step 3: Startup (C, so the vector table is a plain link-time constant)**

```c
/* oem/src/startup.c */
#include <stdint.h>

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _stack_top;
extern void (*__preinit_array_start[])(void);
extern void (*__preinit_array_end[])(void);
extern void (*__init_array_start[])(void);
extern void (*__init_array_end[])(void);
int main(void);
void SysTick_Handler(void);
void Reset_Handler(void);

#define REG(a) (*(volatile uint32_t *)(a))

/* Unexpected exception: stop feeding the watchdog, which resets the board within ~2 s. */
void Default_Handler(void) {
  for (;;) {
  }
}

typedef void (*vector_t)(void);

__attribute__((section(".isr_vector"), used)) const vector_t g_vectors[16 + 32] = {
    (vector_t)&_stack_top, Reset_Handler, Default_Handler, Default_Handler,  /* SP, reset, NMI, HardFault */
    0, 0, 0, 0, 0, 0, 0, Default_Handler, 0, 0, Default_Handler, SysTick_Handler,  /* SVC 11, PendSV 14, SysTick 15 */
    [16 ... 47] = Default_Handler,
};

void Reset_Handler(void) {
  REG(0xE000ED08) = (uint32_t)g_vectors; /* VTOR first: the bootloader's SysTick is still running */
  REG(0xE000E010) = 0;                   /* SysTick off until systick_init() */
  REG(0xE000E180) = 0xFFFFFFFFu;         /* NVIC: disable and un-pend whatever the bootloader left */
  REG(0xE000E280) = 0xFFFFFFFFu;
  REG(0x40003000) = 0xAAAAu;             /* feed the FWDGT the bootloader started */
  for (uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) *dst++ = *src++;
  for (uint32_t *dst = &_sbss; dst < &_ebss;) *dst++ = 0;
  for (void (**f)(void) = __preinit_array_start; f < __preinit_array_end; f++) (*f)();
  for (void (**f)(void) = __init_array_start; f < __init_array_end; f++) (*f)();
  __asm volatile("cpsie i" ::: "memory");
  main();
  for (;;) {
  }
}
```

- [ ] **Step 4: Registers and GPIO**

```cpp
// oem/src/hw/regs.h: register addresses used by the app. Values and sequences follow the stock v1 firmware;
// see docs/history/notes/oem-init-sequences.md for the evidence behind each one.
#pragma once
#include <cstdint>

namespace hw {

inline volatile uint32_t &reg(uint32_t addr) { return *reinterpret_cast<volatile uint32_t *>(addr); }
inline void rmw(uint32_t addr, uint32_t mask, uint32_t value) { reg(addr) = (reg(addr) & ~mask) | (value & mask); }

constexpr uint32_t RCU_CTL = 0x40021000;
constexpr uint32_t RCU_CFG0 = 0x40021004;
constexpr uint32_t RCU_APB1RST = 0x40021010;
constexpr uint32_t RCU_AHBEN = 0x40021014;
constexpr uint32_t RCU_APB1EN = 0x4002101C;
constexpr uint32_t RCU_BDCTL = 0x40021020;
constexpr uint32_t RCU_RSTSCK = 0x40021024;
constexpr uint32_t RCU_CFG1 = 0x4002102C;
constexpr uint32_t FMC_WS = 0x40022000;
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

```cpp
// oem/src/hw/gpio.h
#pragma once
#include "hw/regs.h"

namespace hw {

enum class Mode : uint32_t { INPUT = 0, OUTPUT = 1, AF = 2, ANALOG = 3 };
enum class Pull : uint32_t { NONE = 0, UP = 1, DOWN = 2 };

struct Pin {
  uint32_t port;
  uint8_t n;
};

// Push-pull, high speed (OSPD 3), as the stock firmware configures every output and AF pin.
inline void pin_config(Pin p, Mode mode, Pull pull, uint8_t af = 0) {
  const uint32_t s2 = 2u * p.n, s4 = 4u * (p.n & 7u);
  rmw(p.port + GPIO_PUD, 3u << s2, uint32_t(pull) << s2);
  rmw(p.port + GPIO_OSPD, 3u << s2, 3u << s2);
  rmw(p.port + GPIO_OMODE, 1u << p.n, 0);
  rmw(p.port + (p.n < 8 ? GPIO_AFSEL0 : GPIO_AFSEL1), 0xFu << s4, uint32_t(af) << s4);
  rmw(p.port + GPIO_CTL, 3u << s2, uint32_t(mode) << s2);
}

inline void pin_write(Pin p, bool high) { reg(p.port + (high ? GPIO_BOP : GPIO_BC)) = 1u << p.n; }
inline bool pin_read(Pin p) { return (reg(p.port + GPIO_ISTAT) >> p.n) & 1u; }

}  // namespace hw
```

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

inline void enable_gpio_clocks() { hw::reg(hw::RCU_AHBEN) |= 0xFu << 17; }  // PA..PD

}  // namespace board
```

- [ ] **Step 5: Clock, watchdog, tick**

```cpp
// oem/src/hw/clock.h
#pragma once
#include <cstdint>
#include "hw/regs.h"

namespace hw {

bool clock_init();        // 60 MHz from the 12 MHz HXTAL; false if HXTAL never started (then 16 MHz IRC16M)
uint32_t core_hz();
void fwdgt_init();        // same ~2 s timeout as the bootloader; the FWDGT keeps running
inline void fwdgt_feed() { reg(FWDGT_CTL) = 0xAAAA; }
void systick_init();      // 1 ms tick
uint32_t millis();
uint32_t reset_flags();   // RCU_RSTSCK bits 23..31 as latched by clock_init()
[[noreturn]] void system_reset();

}  // namespace hw
```

```cpp
// oem/src/hw/clock.cpp
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

}  // namespace

extern "C" void SysTick_Handler() { g_ms = g_ms + 1; }

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
  reg(RCU_APB1RST) |= 1u;  // TIMER1: the bootloader leaves it running
  reg(RCU_APB1RST) &= ~1u;
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
  while (!(reg(RCU_CTL) & (1u << 25))) {
  }
  rmw(RCU_CFG0, 0x3u, 0x2u);
  while ((reg(RCU_CFG0) & 0xCu) != 0x8u) {
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
  reg(SCB_AIRCR) = 0x05FA0004u;  // SYSRESETREQ
  for (;;) {
  }
}

}  // namespace hw
```

- [ ] **Step 6: Blink app**

```cpp
// oem/src/blink.cpp: first image on the board. The Charge LED blinks at 1 Hz on a 60 MHz clock, or at 5 Hz if
// HXTAL failed and we're on IRC16M.
#include "board.h"
#include "hw/clock.h"

int main() {
  const bool hxtal = hw::clock_init();
  hw::fwdgt_init();
  hw::systick_init();
  board::enable_gpio_clocks();
  hw::pin_write(board::LED_CHARGE, true);  // off (active low) before it becomes an output
  hw::pin_config(board::LED_CHARGE, hw::Mode::OUTPUT, hw::Pull::NONE);
  const uint32_t half_period = hxtal ? 500 : 100;
  uint32_t last = hw::millis();
  bool on = false;
  for (;;) {
    if (hw::millis() - last >= half_period) {
      last += half_period;
      on = !on;
      hw::pin_write(board::LED_CHARGE, !on);
    }
    hw::fwdgt_feed();
  }
}
```

- [ ] **Step 7: Build it and check the image**

Run:
```bash
cmake -S oem -B oem/build && cmake --build oem/build
~/.local/arm-gnu-toolchain-15.3.rel2/bin/arm-none-eabi-objdump -h oem/build/blink.elf | grep -E "isr_vector|\.text|\.data"
python3 -c "import struct;d=open('oem/build/blink.bin','rb').read();sp,rv=struct.unpack_from('<II',d);print(hex(sp),hex(rv))"
```

  Expected:
  - The build succeeds, with memory usage printed and FLASH a few KB.
  - `.isr_vector` VMA = 08005000.
  - SP prints `0x20004000`, and the reset vector is odd and between 0x08005000 and 0x0801E000.
  - `oem/build/blink.dat` exists.

- [ ] **Step 8: Commit**

```bash
git add oem/
git commit -m "OEM firmware skeleton: CMake/linker/startup behind the stock bootloader, stock clock tree, FWDGT, blink app"
```

---

### Task 5: [BENCH] First own image on the board

- [ ] **Step 1: Install the blink app** (user power-cycles when prompted):
  `.venv/bin/python tools/oem_update.py --port /dev/cu.usbserial-XXXX oem/build/blink.dat`

- [ ] **Step 2: Observe.** The user reports what the LEDs do for 60 s.
  - Expected: one LED blinks steadily at 1 Hz, with no pause or stutter.
  - A 1–2 s gap every ~2 s would mean watchdog resets.
  - Fast blinking (5 Hz) means HXTAL didn't start.
  - Record which physical LED ("Charge" green or "Fault" red) is on PD1.

- [ ] **Step 3: If nothing happens:**
  - Power-cycle and reinstall `build/stock-v1.dat` to confirm the path still works.
  - Then attach openocd: `init; halt; reg pc; mdw 0xE000ED08; mdw 0x40021004` (read-only) and capture the output for debugging.

- [ ] **Step 4: Record** the result in `docs/history/notes/oem-bringup.md` ("First image"), then commit:

```bash
git add docs/history/notes/oem-bringup.md
git commit -m "OEM bring-up: blink image boots behind the stock bootloader"
```

---

### Task 6: Key state machine (`oem/src/app/keys.h`)

**Files:**
- Create: `oem/src/app/keys.h`, `tests/test_oem_keys.cpp`
- Modify: `tests/Makefile` (add the test and the include path)

**Interfaces:**
- Produces: `oem::KeyEvent {NONE, CLICK, LONG, REPEAT}`, `oem::Key::update(bool raw) -> KeyEvent` (called every 10 ms),
  `Key::pressed()`, `Key::held_ticks()`, and constants `KEY_LONG_TICKS = 100`, `KEY_REPEAT_TICKS = 15`.

- [ ] **Step 1: Write the failing test**

```cpp
// tests/test_oem_keys.cpp
#include "app/keys.h"
#include "check.h"

using oem::Key;
using oem::KeyEvent;

static int count(Key &k, bool raw, int ticks, KeyEvent want) {
  int n = 0;
  for (int i = 0; i < ticks; i++) n += k.update(raw) == want;
  return n;
}

static void test_glitch_ignored() {
  Key k;
  CHECK(k.update(true) == KeyEvent::NONE);  // one high sample, then low again
  CHECK(k.update(false) == KeyEvent::NONE);
  CHECK(count(k, false, 10, KeyEvent::CLICK) == 0);
  CHECK(!k.pressed());
}

static void test_click_on_release() {
  Key k;
  CHECK(count(k, true, 5, KeyEvent::CLICK) == 0);
  CHECK(k.pressed());
  CHECK(count(k, false, 5, KeyEvent::CLICK) == 1);
  CHECK(!k.pressed());
}

static void test_long_then_repeat_no_click() {
  Key k;
  int longs = 0, repeats = 0;
  for (int i = 0; i < 2 + 100 + 45; i++) {
    const KeyEvent e = k.update(true);
    longs += e == KeyEvent::LONG;
    repeats += e == KeyEvent::REPEAT;
  }
  CHECK(longs == 1);
  CHECK(repeats == 3);  // 15, 30 and 45 ticks after the long press
  CHECK(k.held_ticks() == 145);
  CHECK(count(k, false, 5, KeyEvent::CLICK) == 0);
}

int main() {
  test_glitch_ignored();
  test_click_on_release();
  test_long_then_repeat_no_click();
  return test_result("test_oem_keys");
}
```

Append `tests/test_oem_keys` and `tests/test_oem_lcd_render` (Task 7's binary) to the repo-root `.gitignore`, next to the
existing test binaries. Then in `tests/Makefile`, change the first three lines to:

```make
CXX ?= c++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -I../esphome/components -I../oem/src
TESTS = test_nasn_protocol test_charge_logic test_oem_keys
HEADERS = check.h $(wildcard ../esphome/components/*/*_protocol.h ../esphome/components/*/*_logic.h ../oem/src/app/*.h)
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `app/keys.h: No such file or directory`.

- [ ] **Step 3: Implement**

Superseded by commit c6796d0: a raw edge returns NONE without stepping the hold counter (see the ledger ruling).

```cpp
// oem/src/app/keys.h: one front-panel key. Debounce over 2 samples (20 ms), click on release, long press at 1 s,
// then repeat every 150 ms (the stock FlexibleButton timing, docs/history/notes/oem-hw-map.md §4).
#pragma once
#include <cstdint>

namespace oem {

enum class KeyEvent : uint8_t { NONE, CLICK, LONG, REPEAT };

constexpr uint16_t KEY_LONG_TICKS = 100;
constexpr uint16_t KEY_REPEAT_TICKS = 15;

class Key {
 public:
  // Call every 10 ms with the raw, active-high pin level.
  KeyEvent update(bool raw) {
    if (raw != last_raw_) {
      last_raw_ = raw;
      return step();
    }
    if (raw != stable_) {
      stable_ = raw;
      if (stable_) {
        held_ = 0;
        long_sent_ = false;
        return KeyEvent::NONE;
      }
      return long_sent_ ? KeyEvent::NONE : KeyEvent::CLICK;
    }
    return step();
  }

  bool pressed() const { return stable_; }
  uint16_t held_ticks() const { return stable_ ? held_ : 0; }

 private:
  KeyEvent step() {
    if (!stable_) return KeyEvent::NONE;
    if (held_ < 0xFFFF) held_++;
    if (held_ == KEY_LONG_TICKS) {
      long_sent_ = true;
      return KeyEvent::LONG;
    }
    if (long_sent_ && held_ > KEY_LONG_TICKS && (held_ - KEY_LONG_TICKS) % KEY_REPEAT_TICKS == 0) return KeyEvent::REPEAT;
    return KeyEvent::NONE;
  }

  bool last_raw_ = false;
  bool stable_ = false;
  bool long_sent_ = false;
  uint16_t held_ = 0;
};

}  // namespace oem
```

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_keys: ok`, and every other test still `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/keys.h tests/test_oem_keys.cpp tests/Makefile .gitignore
git commit -m "OEM keys: debounce/click/long/repeat state machine with host tests"
```

---

### Task 7: LCD rendering (`oem/src/app/lcd_render.h`)

**Files:**
- Create: `oem/src/app/lcd_render.h`, `tests/test_oem_lcd_render.cpp`
- Modify: `tests/Makefile` (append `test_oem_lcd_render` to `TESTS`)

**Interfaces:**
- Produces:
  - `oem::Frame{uint32_t com[8]}` and `oem::Field {AC_V, CURRENT, POWER, SOC, BATTERY_V}`
  - glyph constants `GLYPH_BLANK = 10`, `GLYPH_DASH = 11`, `GLYPH_E = 12`
  - `put_fixed_labels`, `put_glyph(f, field, pos, glyph)`, `put_number(f, field, value, min_digits = 1)`, `put_dashes`,
    `put_soc_bars(f, soc)`, `put_segment(f, seg, com)`
  - `LCD_SEGS[19]`, the SEG lines that are wired.
- **Bit model:** bit *s* of `com[k]` is SEG*s* × COM*k*. A glyph's bit *k* lights COM*k* on the digit's SEG column. Source:
  `docs/history/notes/oem-init-sequences.md` §4.

- [ ] **Step 1: Write the failing test.** The golden frame is the stock v1 render of an all-zero state, taken from emulating
  0800a0dc. That frame is AC `---`, current `0`, power `0`, SOC `0`, battery `0.0`, and the SEG28 icons COM0/1/2/6.

```cpp
// tests/test_oem_lcd_render.cpp
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

int main() {
  test_matches_stock_zero_frame();
  test_number_places_digits_right_aligned();
  test_overflow_shows_dashes();
  test_ac_uses_font_b();
  test_soc_bars();
  return test_result("test_oem_lcd_render");
}
```

- [ ] **Step 2: Run it and check it fails**

Run: `make -C tests`
Expected: compile error, `app/lcd_render.h: No such file or directory`.

- [ ] **Step 3: Implement**

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

inline void put_glyph(Frame &f, Field field, int pos, uint8_t glyph) {
  const FieldDef &d = FIELDS[int(field)];
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

// Eight bars on SEG16, COM7 first, lit above the stock thresholds.
inline void put_soc_bars(Frame &f, int soc) {
  static constexpr uint8_t THRESHOLDS[8] = {0, 13, 27, 39, 51, 63, 75, 87};
  for (int i = 0; i < 8; i++) {
    if (soc > THRESHOLDS[i]) put_segment(f, 16, 7 - i);
  }
}

}  // namespace oem
```

Then append `test_oem_lcd_render` to the `TESTS` line in `tests/Makefile`.

- [ ] **Step 4: Run the tests and check they pass**

Run: `make -C tests`
Expected: `test_oem_lcd_render: ok`, and every other test still `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/app/lcd_render.h tests/test_oem_lcd_render.cpp tests/Makefile
git commit -m "OEM LCD render: stock fonts, field map and fixed labels; golden test against the stock zero frame"
```

---

### Task 8: SLCD driver and the bring-up app

**Files:**
- Create: `oem/src/hw/slcd.h`, `oem/src/hw/slcd.cpp`, `oem/src/bringup.cpp`
- Modify: `oem/CMakeLists.txt` (add the `bringup` app and `src/hw/slcd.cpp` to `HW_SOURCES`)

**Interfaces:**
- Consumes: Task 4 (`hw::*`, `board::*`), Task 6 (`oem::Key`), Task 7 (`oem::Frame` and the `put_*` functions).
- Produces: `hw::slcd_init() -> bool` and `hw::slcd_write(const uint32_t com[8]) -> bool`, used by later phases.

- [ ] **Step 1: SLCD driver**, a literal replay of stock v1 (08008588 and 0800cafc, `docs/history/notes/oem-init-sequences.md` §3).

```cpp
// oem/src/hw/slcd.h
#pragma once
#include <cstdint>

namespace hw {

bool slcd_init();                         // false if the controller never synced or turned on
bool slcd_write(const uint32_t com[8]);   // false if the previous update never completed

}  // namespace hw
```

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
  bool ok = false;
  for (uint32_t t = 0; t < 0x100000; t++) {
    if (!(reg(SLCD_STAT) & 4u)) {  // UPRF clear: the last update is done
      ok = true;
      break;
    }
  }
  for (int k = 0; k < 8; k++) reg(slcd_data(k)) = com[k];
  reg(SLCD_STAT) |= 4u;  // request an update
  return ok;
}

}  // namespace hw
```

- [ ] **Step 2: Bring-up app**

```cpp
// oem/src/bringup.cpp: phase-1 bench app (docs/history/superpowers/plans/2026-10-14-oem-firmware-phase1-bringup.md, Task 9).
//
// Screens (Switch click cycles them):
//   STATUS: AC field = clock MHz (60 or 16), battery V = reset cause (1 power-on, 2 pin, 3 software, 4 watchdog,
//           0 other), current = uptime s % 1000, power = keys held (Switch 1, Up 2, Down 4, Enter 8), SOC = probe step.
//           Enter click advances the probe step: 0 idle, 1 PC1 high, 2 PB6 high, 3 PD0 low, 4 PD1 low, 5 PC2 low.
//   WALK:   one segment lit. Up/Down step through the 19 SEG x 8 COM bits; Enter toggles an overlay with SEG in the
//           AC field and COM in the SOC field.
//   RX:     falling edges per second on PA3 (USART1 RX) in the current field and PB1 (UART4 RX) in the power field.
// Hold Up+Down 3 s: stop feeding the watchdog (expect a reset, then cause 4).
// Hold Switch+Enter 3 s: software reset, giving the bootloader its 1.5 s window for oem_update.py.
#include <initializer_list>

#include "app/keys.h"
#include "app/lcd_render.h"
#include "board.h"
#include "hw/clock.h"
#include "hw/slcd.h"

using namespace oem;

namespace {

enum class Screen : uint8_t { STATUS, WALK, RX };

int reset_cause(uint32_t flags) {
  if (flags & (1u << 29)) return 4;
  if (flags & (1u << 28)) return 3;
  if (flags & (1u << 27)) return 1;
  if (flags & (1u << 26)) return 2;
  return 0;
}

void apply_probe(int step) {
  using namespace board;
  hw::pin_write(OUT_PC1, step == 1);
  if (step == 2) {
    hw::pin_write(PROBE_PB6, true);
    hw::pin_config(PROBE_PB6, hw::Mode::OUTPUT, hw::Pull::NONE);
  } else {
    hw::pin_config(PROBE_PB6, hw::Mode::INPUT, hw::Pull::NONE);
  }
  hw::pin_write(LED_FAULT, step != 3);
  hw::pin_write(LED_CHARGE, step != 4);
  hw::pin_write(BACKLIGHT, step != 5);
}

}  // namespace

int main() {
  using namespace board;
  const bool hxtal = hw::clock_init();
  hw::fwdgt_init();
  hw::systick_init();
  enable_gpio_clocks();

  for (Pin p : {KEY_SWITCH, KEY_UP, KEY_DOWN, KEY_ENTER}) hw::pin_config(p, hw::Mode::INPUT, hw::Pull::DOWN);
  for (Pin p : {USART1_RX, UART4_RX}) hw::pin_config(p, hw::Mode::INPUT, hw::Pull::UP);
  hw::pin_write(RS485_DE, false);  // receive
  hw::pin_config(RS485_DE, hw::Mode::OUTPUT, hw::Pull::NONE);
  apply_probe(0);  // stock idle levels: LEDs off (high), PC1 low, backlight on
  for (Pin p : {OUT_PC1, LED_FAULT, LED_CHARGE, BACKLIGHT}) hw::pin_config(p, hw::Mode::OUTPUT, hw::Pull::NONE);

  const bool lcd = hw::slcd_init();
  const int cause = reset_cause(hw::reset_flags());

  Key keys[4];  // Switch, Up, Down, Enter
  const Pin key_pins[4] = {KEY_SWITCH, KEY_UP, KEY_DOWN, KEY_ENTER};
  Screen screen = Screen::STATUS;
  int probe = 0, walk = 0;
  bool overlay = false, starve = false;
  uint32_t edges[2] = {0, 0}, shown[2] = {0, 0};
  bool level[2] = {true, true};
  uint32_t last_key = hw::millis(), last_draw = 0, last_rx = hw::millis(), last_fault_blink = 0;

  for (;;) {
    const uint32_t now = hw::millis();

    const bool rx[2] = {hw::pin_read(USART1_RX), hw::pin_read(UART4_RX)};
    for (int i = 0; i < 2; i++) {
      if (level[i] && !rx[i]) edges[i]++;
      level[i] = rx[i];
    }
    if (now - last_rx >= 1000) {
      last_rx += 1000;
      shown[0] = edges[0];
      shown[1] = edges[1];
      edges[0] = edges[1] = 0;
    }

    if (now - last_key >= 10) {
      last_key += 10;
      KeyEvent ev[4];
      for (int i = 0; i < 4; i++) ev[i] = keys[i].update(hw::pin_read(key_pins[i]));
      if (keys[1].held_ticks() >= 300 && keys[2].held_ticks() >= 300) starve = true;
      if (keys[0].held_ticks() >= 300 && keys[3].held_ticks() >= 300) hw::system_reset();
      if (ev[0] == KeyEvent::CLICK) screen = Screen((uint8_t(screen) + 1) % 3);
      if (screen == Screen::STATUS && ev[3] == KeyEvent::CLICK) {
        probe = (probe + 1) % 6;
        apply_probe(probe);
      }
      if (screen == Screen::WALK) {
        const bool up = ev[1] == KeyEvent::CLICK || ev[1] == KeyEvent::REPEAT;
        const bool down = ev[2] == KeyEvent::CLICK || ev[2] == KeyEvent::REPEAT;
        if (up) walk = (walk + 1) % (19 * 8);
        if (down) walk = (walk + 19 * 8 - 1) % (19 * 8);
        if (ev[3] == KeyEvent::CLICK) overlay = !overlay;
      }
    }

    if (now - last_draw >= 100) {
      last_draw = now;
      Frame f;
      if (now < 2000) {
        for (uint32_t &w : f.com) w = 0xFFFFFFFFu;  // lamp test
      } else if (screen == Screen::STATUS) {
        put_fixed_labels(f);
        put_number(f, Field::AC_V, hxtal ? 60 : 16);
        put_number(f, Field::BATTERY_V, uint32_t(cause));
        put_number(f, Field::CURRENT, (now / 1000) % 1000);
        uint32_t held = 0;
        for (int i = 0; i < 4; i++) held |= uint32_t(keys[i].pressed()) << i;
        put_number(f, Field::POWER, held);
        put_number(f, Field::SOC, uint32_t(probe));
      } else if (screen == Screen::WALK) {
        const int seg = LCD_SEGS[walk / 8], com = walk % 8;
        put_segment(f, seg, com);
        if (overlay) {
          put_number(f, Field::AC_V, uint32_t(seg));
          put_number(f, Field::SOC, uint32_t(com));
        }
      } else {
        put_number(f, Field::CURRENT, shown[0]);
        put_number(f, Field::POWER, shown[1]);
      }
      if (lcd) hw::slcd_write(f.com);
    }

    if (!lcd && now - last_fault_blink >= 100) {  // no LCD: blink the Fault LED fast instead
      last_fault_blink = now;
      hw::pin_write(LED_FAULT, (now / 100) % 2);
    }

    if (!starve) hw::fwdgt_feed();
  }
}
```

- [ ] **Step 3: Add the app to CMake.** In `oem/CMakeLists.txt`, set `HW_SOURCES` to
  `src/startup.c src/hw/clock.cpp src/hw/slcd.cpp` and add `oem_app(bringup ${HW_SOURCES} src/bringup.cpp)` after the blink line.

- [ ] **Step 4: Build**

Run: `cmake --build oem/build && make -C tests`
Expected: both apps build with no warnings, `oem/build/bringup.dat` exists, and the host tests are all `ok`.

- [ ] **Step 5: Commit**

```bash
git add oem/src/hw/slcd.h oem/src/hw/slcd.cpp oem/src/bringup.cpp oem/CMakeLists.txt
git commit -m "OEM bring-up app: stock SLCD init, status/segment-walk/RS485-RX screens, GPIO probe, watchdog and bootloader tests"
```

---

### Task 9: [BENCH] Bring-up session

The user runs each step, and the controller records the results in `docs/history/notes/oem-bringup.md`.

- [ ] **Step 1: Install** `oem/build/bringup.dat` with `oem_update.py`, power-cycling when prompted.

- [ ] **Step 2: Lamp test.**
  - Expected: every segment is on for 2 s, then the STATUS screen appears with 60 in the AC field.
  - Photograph the lamp test so the glass legend can be recorded.

- [ ] **Step 3: Keys.** Press each key and record which physical key shows 1, 2, 4 or 8 in the power field. Confirm Switch = PC6,
  Up = PB15, Down = PA12 and Enter = PA11, or correct `board.h`.

- [ ] **Step 4: GPIO probe.** On the STATUS screen, click Enter through steps 1–5 and record what each step does:
  - 1: PC1 high. Relay click?
  - 2: PB6 high.
  - 3: PD0 low. Which LED?
  - 4: PD1 low.
  - 5: PC2 low. Backlight off?

  Update the comments in `board.h` to match.

- [ ] **Step 5: Segment walk.** On the WALK screen, step through all 152 bits. For each lit element, note what it is: a digit
  segment, a unit label, the decimal point, an icon, a menu indicator, or nothing. A photo of every eighth step is enough. This
  produces the glass legend table for `docs/history/notes/oem-hw-map.md`.

- [ ] **Step 6: RS485 RX wiring.**
  - On the RX screen, send continuous 0x55 from the host at 9600 8N1:
    `python3 -c "import serial,time;s=serial.Serial('/dev/cu.usbserial-XXXX',9600);t=time.time()
while time.time()-t<20: s.write(b'U'*96)"`
  - Expected: the current field (PA3) shows roughly 4800. The power field (PB1) shows a similar count if UART4 RX is also wired to
    RO, or 0 if not. Record both.

- [ ] **Step 7: Watchdog.**
  - Hold Up+Down for 3 s.
  - Expected: the display freezes, then within about 2 s the board reboots: lamp test, then STATUS with battery field = 4.

- [ ] **Step 8: Bootloader re-entry and revert.**
  - Start `oem_update.py ... build/stock-v1.dat`, then hold Switch+Enter for 3 s.
  - Expected: the tool catches the bootloader window without a power cycle, and the stock UI returns.
  - Then reinstall `bringup.dat` the same way, power-cycling this time, because the stock app has no Switch+Enter reset.

- [ ] **Step 9: Record and commit.**
  - Write all results into `docs/history/notes/oem-bringup.md`.
  - Add the glass legend and the confirmed pin roles to `docs/history/notes/oem-hw-map.md` §10.
  - Fix any `board.h` comments.
  - Commit:

```bash
git add docs/history/notes/oem-bringup.md docs/history/notes/oem-hw-map.md oem/src/board.h
git commit -m "OEM bring-up results: key map, GPIO roles, LCD glass legend, RS485 RX wiring, watchdog and revert verified"
```

---

### Task 10: Docs

**Files:**
- Modify: `README.md` (add an "OEM board firmware" section and the new files to the Layout table)
- Create: `docs/history/notes/oem-init-sequences.md` (already copied in; commit it if it isn't yet)

- [ ] **Step 1: README section.** Add it after "Bench tools", covering:
  - the purpose (link the spec);
  - toolchain location and override (`ARM_TOOLCHAIN`);
  - build: `cmake -S oem -B oem/build && cmake --build oem/build`;
  - install: `.venv/bin/python tools/oem_update.py --port … oem/build/<app>.dat`, power-cycling when prompted;
  - revert: `oem_image.py --stock-from-dump …` then `oem_update.py`;
  - never resend a frame: on failure, reset and rerun.

  Add `oem/`, `docs/history/notes/oem-hw-map.md`, `docs/history/notes/oem-init-sequences.md` and `docs/history/notes/oem-bringup.md` to the Layout table.

- [ ] **Step 2: Run everything**

Run: `make -C tests && .venv/bin/python -m pytest tools/ -q && cmake --build oem/build`
Expected: all pass and build.

- [ ] **Step 3: Commit**

```bash
git add README.md docs/history/notes/oem-init-sequences.md
git commit -m "README: OEM board firmware build, install and revert"
```
