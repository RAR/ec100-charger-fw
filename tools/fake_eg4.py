#!/usr/bin/env python3
"""EG4 LL BMS simulator: a Modbus RTU slave at address 0x10 (9600 8N1), the way the board's EG4 source reads it.

Usage: fake_eg4.py --port /dev/cu.usbserial-XXXX [--voltage 53.2] [--soc 75] [--max-cell 3.345] [--cells 16] [--ccl 30]
                   [--protection 0x0000] [--control FILE] [-q]
       fake_eg4.py --bridge eco-charger-a.local --soc 60       (through the WiFi bridge's Passthrough, set to 9600)

Answers FC 03 reads of 0x0000 x 0x12 (pack V, 16 cells) and 0x0012 x 0x15 (CCL at 0x16, SOC at 0x18, protection bits
at 0x1B) with a correct CRC; everything else (other addresses, functions, ranges) is ignored. Commands on stdin change
the values live: set voltage|soc|max-cell|ccl|cells|protection VALUE, silent on|off (no replies at all), show, quit.
protection is the 0x001B register (0x0593 is the alarm mask the board checks). --control FILE takes the same commands as
lines appended to FILE (created if missing), so separate shell calls can drive it. -q prints only when a reply
differs from the previous one.
"""
import argparse
import dataclasses
import math
import os
import queue
import struct
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_pylon import CONTROL_POLL_S, ControlFile, say, stdin_reader  # noqa: E402
from oem_image import crc16_modbus  # noqa: E402
from oem_modbus import (PASSTHROUGH_SETTLE_S, bridge_url, open_port, resolve_bridge,  # noqa: E402
                        set_bridge_baud, set_bridge_passthrough)

ADDR = 0x10
BAUD = 9600
BLOCKS = {(0x0000, 0x12): "A", (0x0012, 0x15): "B"}
REQUEST_LEN = 8


@dataclasses.dataclass
class State:
    voltage: float = 53.2
    soc: int = 75
    max_cell: float = 3.345
    cells: int = 16
    ccl: int = 30
    protection: int = 0
    silent: bool = False


def with_crc(body):
    return body + struct.pack("<H", crc16_modbus(body))


def build_request(first, count):
    return with_crc(struct.pack(">BBHH", ADDR, 3, first, count))


def block_a(s):
    """Registers 0x00-0x11: pack V (0.01 V), one reserved word, 16 cell mV (the first is the highest, absent ones 0)."""
    cells = [0] * 16
    top = round(s.max_cell * 1000)
    for i in range(min(s.cells, 16)):
        cells[i] = max(top - (i % 4) * 4, 0) if i else top
    return [round(s.voltage * 100), 0] + cells


def block_b(s):
    """Registers 0x12-0x26 (zero except CCL at 0x16, SOC at 0x18 and protection at 0x1B)."""
    regs = [0] * 0x15
    regs[0x16 - 0x12], regs[0x18 - 0x12], regs[0x1B - 0x12] = s.ccl, s.soc, s.protection
    return regs


def reply_for(first, count, s):
    kind = BLOCKS.get((first, count))
    if s.silent or kind is None:
        return None
    regs = block_a(s) if kind == "A" else block_b(s)
    return with_crc(struct.pack(f">BBB{count}H", ADDR, 3, 2 * count, *regs))


def parse_request(frame):
    """(first, count) of a valid 8 byte FC 03 request to 0x10, else None."""
    if len(frame) != REQUEST_LEN or frame[0] != ADDR or frame[1] != 3:
        return None
    if struct.pack("<H", crc16_modbus(frame[:6])) != frame[6:]:
        return None
    return struct.unpack(">HH", frame[2:6])


def extract_eg4(buf):
    """(consumed, request or None) for the front of buf; (0, None) when more bytes are needed."""
    if buf[0] != ADDR:
        return 1, None
    if len(buf) < REQUEST_LEN:
        return 0, None
    req = parse_request(bytes(buf[:REQUEST_LEN]))
    return (REQUEST_LEN, req) if req else (1, None)


def describe(req, s):
    if BLOCKS[req] == "A":
        return f"A -> V={s.voltage:.2f} maxcell={s.max_cell:.3f} cells={s.cells}"
    return f"B -> SOC={s.soc} CCL={s.ccl} protection=0x{s.protection:04X}"


# name: (State field, converter, lowest, highest)
SETTABLE = {"voltage": ("voltage", float, 0, 655.35), "soc": ("soc", int, 0, 100),
            "max-cell": ("max_cell", float, 0, 65.535), "cells": ("cells", int, 0, 16),
            "ccl": ("ccl", int, 0, 65535), "protection": ("protection", lambda t: int(t, 0), 0, 0xFFFF)}
SWITCHES = {"silent": "silent"}
USAGE = "error: set NAME VALUE | silent on|off | show | quit"


def apply_command(s, line):
    """Applies one command (stdin or --control) to the state; returns the text to print, or None for quit."""
    w = line.split()
    try:
        if w[:1] == ["quit"]:
            return None
        if w[:1] == ["show"]:
            return ", ".join(f"{k}={v}" for k, v in dataclasses.asdict(s).items())
        if len(w) == 3 and w[0] == "set" and w[1] in SETTABLE:
            field, conv, lo, hi = SETTABLE[w[1]]
            value = conv(w[2])
            if not math.isfinite(value) or not lo <= value <= hi:
                return f"error: {w[1]} must be {lo} to {hi}"
            setattr(s, field, value)
            return f"{w[1]} = {getattr(s, field)}"
        if len(w) == 2 and w[0] in SWITCHES and w[1] in ("on", "off"):
            setattr(s, SWITCHES[w[0]], w[1] == "on")
            return f"{w[0]} {w[1]}"
    except ValueError:
        pass
    return USAGE


def check_args(parser, a, settable):
    """Range-checks the command-line values like `set` does (a bad one exits with the usage error, status 2)."""
    for name, (_, _, lo, hi) in settable.items():
        value = getattr(a, name.replace("-", "_"))
        if not math.isfinite(value) or not lo <= value <= hi:
            parser.error(f"--{name} must be {lo} to {hi}")


def serve_frames(port, s, extract, answer, apply, quiet=False, commands=None, out=say, clock=time.monotonic,
                 control=None):
    """Shared serve loop. extract(buf) -> (consumed, request); answer(request, s) -> (reply bytes or None, label);
    apply(s, line) -> text or None to quit."""
    buf, last, control_at = bytearray(), {}, None
    while True:
        lines = []
        while commands is not None and not commands.empty():
            lines.append(commands.get())
        if control is not None and (control_at is None or clock() - control_at >= CONTROL_POLL_S):
            control_at = clock()
            lines += control.lines()
        for line in filter(str.strip, lines):
            msg = apply(s, line)
            if msg is None:
                return
            out(msg)
        data = port.read(256)
        if not data:
            continue
        buf += data
        while buf:
            used, req = extract(buf)
            if not used:
                break
            del buf[:used]
            if req is None:
                continue
            reply, label = answer(req, s)
            if reply is None:
                continue
            port.write(reply)
            kind = label.split(" ")[0]
            if not quiet or last.get(kind) != label:
                out(label)
            last[kind] = label


def answer_eg4(req, s):
    reply = reply_for(*req, s)
    return reply, describe(req, s) if reply else ""


def serve(port, s, quiet=False, commands=None, out=say, clock=time.monotonic, control=None):
    serve_frames(port, s, extract_eg4, answer_eg4, apply_command, quiet, commands, out, clock, control)


def build_parser():
    p = argparse.ArgumentParser(description="EG4 LL BMS simulator (Modbus slave 0x10)")
    where = p.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial device on the charger's RS485 line")
    where.add_argument("--bridge", metavar="HOST", help="WiFi bridge host; uses its Passthrough at 9600")
    p.add_argument("--voltage", type=float, default=State.voltage, help="pack V")
    p.add_argument("--soc", type=int, default=State.soc, help="SOC %%")
    p.add_argument("--max-cell", type=float, default=State.max_cell, help="highest cell V")
    p.add_argument("--cells", type=int, default=State.cells, help="cells reporting, 0-16")
    p.add_argument("--ccl", type=int, default=State.ccl, help="charge current limit (register 0x16), A")
    p.add_argument("--protection", type=lambda t: int(t, 0), default=0, help="protection bits (register 0x1B)")
    p.add_argument("--control", metavar="FILE", help="also read commands appended to FILE (created if missing)")
    p.add_argument("-q", "--quiet", action="store_true", help="print only changes")
    return p


def run_main(a, s, serve_fn, baud, sleep=time.sleep):
    """Opens the port (or the bridge's Passthrough at `baud`) and serves until quit; shared by fake_jk."""
    import serial  # pyserial; imported here so the tests don't need a port

    if a.bridge:
        a.bridge = resolve_bridge(a.bridge)
    try:
        if a.bridge:
            # inside the try, so an interrupt during the settle still turns Passthrough off
            set_bridge_passthrough(a.bridge, True)
            sleep(PASSTHROUGH_SETTLE_S)
        name = bridge_url(a.bridge) if a.bridge else a.port
        if a.bridge:
            set_bridge_baud(a.bridge, baud)
        try:
            port = open_port(serial, name, baud)
        except serial.SerialException as e:
            print(f"error: cannot open {name}: {e}", file=sys.stderr, flush=True)
            sys.exit(3)
        commands = queue.Queue()
        control = ControlFile(a.control) if a.control else None
        threading.Thread(target=stdin_reader, args=(commands,), daemon=True).start()
        with port:
            try:
                serve_fn(port, s, a.quiet, commands, control=control)
            except KeyboardInterrupt:
                pass
    finally:
        if a.bridge:
            set_bridge_passthrough(a.bridge, False)


def main(argv=None, sleep=time.sleep):
    parser = build_parser()
    a = parser.parse_args(argv)
    check_args(parser, a, SETTABLE)
    s = State(voltage=a.voltage, soc=a.soc, max_cell=a.max_cell, cells=a.cells, ccl=a.ccl, protection=a.protection)
    run_main(a, s, serve, BAUD, sleep)


if __name__ == "__main__":
    main()
