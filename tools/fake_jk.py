#!/usr/bin/env python3
"""JK-B BMS simulator: answers the board's read-all request (4E 57 ... 06 SS ...) at 115200 8N1.

Usage: fake_jk.py --port /dev/cu.usbserial-XXXX [--voltage 53.6] [--soc 75] [--max-cell 3.345] [--cells 14|16]
                  [--alarms 0] [--status 7] [--control FILE] [-q]
       fake_jk.py --bridge eco-charger-a.local --soc 60       (through the WiFi bridge's Passthrough, set to 115200)

The reply carries the record set of a real 14S pack (docs/history/notes/jk-bms-protocol.md section 5) with the pack
voltage (0x83), SOC (0x85), alarms (0x8B), status (0x8C) and the cell list (0x79, with the cell count in 0x8A) replaced,
a correct length field and a 16-bit checksum. Either source byte (0x02, 0x03) is answered; other frames are ignored.
Commands on stdin change the values live: set voltage|soc|max-cell|cells|alarms|status VALUE, silent on|off (no replies
at all), show, quit. alarms is 0x8B (the board blocks on 0x3736), status is 0x8C (bit 0 = charging allowed, default
0x0007).
--control FILE takes the same commands as lines appended to FILE (created if missing). -q prints only changes.
"""
import argparse
import dataclasses
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fake_eg4 import check_args, run_main, serve_frames  # noqa: E402
from fake_pylon import say  # noqa: E402

BAUD = 115200
REQUEST_LEN = 21
SOURCE_BYTES = (0x02, 0x03)

# Records of the real 14S reply (TV1) around the ones the board reads; those are filled in per reply.
PRE = bytes.fromhex("80001D81001E82001C")  # between the cell list and 0x83
MID = bytes.fromhex("8480D0")  # between 0x83 and 0x85
POST85 = bytes.fromhex("860287000489" "00000000")  # between 0x85 and 0x8A
TAIL = bytes.fromhex(
    "8E16268F10AE900FD2910FA0920005930BEA940C1C95000596012C9700079800039900059A00059B0CE49C00089D019E005A9F0046A00064"
    "A10064A20014A30046A40046A5FFECA6FFF6A7FFECA8FFF6A90EAA0000000EAB01AC01AD0411AE01AF01B0000AB114B231323334353600000"
    "000B300B4496E707574205573B53231303"
    "1B60000E200B748362E585F5F53362E312E33535F5FB800B900000000BA425433303732303230313230303030323030353231303031C001")
RECORD_NUMBER = bytes(4)


@dataclasses.dataclass
class State:
    voltage: float = 53.6
    soc: int = 75
    max_cell: float = 3.345
    cells: int = 14
    alarms: int = 0
    status: int = 0x0007
    silent: bool = False
    cell_mv: tuple = ()  # explicit cell voltages (overrides cells/max_cell), for exact reproductions


def cell_list(s):
    """Millivolts of each cell: the first is the highest."""
    if s.cell_mv:
        return list(s.cell_mv)
    top = round(s.max_cell * 1000)
    return [top - (i % 4) * 4 if i else top for i in range(s.cells)]


def frame(s):
    """A whole reply: header, records from byte 11, record number, 68 00 00, 16-bit big-endian sum."""
    cells = cell_list(s)
    rec79 = bytes([0x79, 3 * len(cells)]) + b"".join(struct.pack(">BH", i + 1, mv) for i, mv in enumerate(cells))
    body = (rec79 + PRE + b"\x83" + struct.pack(">H", round(s.voltage * 100)) + MID + b"\x85" + bytes([s.soc])
            + POST85 + b"\x8a" + struct.pack(">H", len(cells)) + b"\x8b" + struct.pack(">H", s.alarms)
            + b"\x8c" + struct.pack(">H", s.status) + TAIL + RECORD_NUMBER + b"\x68\x00\x00")
    total = 11 + len(body) + 2
    out = bytes([0x4E, 0x57]) + struct.pack(">H", total - 2) + bytes([0, 0, 0, 0, 0x06, 0, 1]) + body
    return out + struct.pack(">H", sum(out) & 0xFFFF)


def request_body(source_byte):
    return bytes([0x4E, 0x57, 0, 0x13, 0, 0, 0, 0, 0x06, source_byte, 0, 0, 0, 0, 0, 0, 0x68, 0, 0])


def build_request(source_byte):
    body = request_body(source_byte)
    return body + struct.pack(">H", sum(body) & 0xFFFF)


def parse_request(f):
    """The source byte of a valid read-all request (21 bytes, either source byte), else None."""
    f = bytes(f)
    if len(f) == REQUEST_LEN and f[9] in SOURCE_BYTES and f == build_request(f[9]):
        return f[9]
    return None


def extract_jk(buf):
    """(consumed, source byte or None) for the front of buf; (0, None) when more bytes are needed."""
    if buf[0] != 0x4E or (len(buf) > 1 and buf[1] != 0x57):
        return 1, None
    if len(buf) < REQUEST_LEN:
        return 0, None
    src = parse_request(buf[:REQUEST_LEN])
    return (REQUEST_LEN, src) if src is not None else (1, None)


def reply_for(source_byte, s):
    return None if s.silent else frame(s)


def answer_jk(source_byte, s):
    reply = reply_for(source_byte, s)
    cells = cell_list(s)
    return reply, (f"SS=0x{source_byte:02X} -> V={s.voltage:.2f} SOC={s.soc} maxcell={max(cells, default=0) / 1000:.3f} "
                   f"cells={len(cells)} alarms=0x{s.alarms:04X} status=0x{s.status:04X}")


# name: (State field, converter, lowest, highest)
SETTABLE = {"voltage": ("voltage", float, 0, 655.35), "soc": ("soc", int, 0, 255),
            "max-cell": ("max_cell", float, 0, 65.535), "cells": ("cells", int, 1, 16),
            "alarms": ("alarms", lambda t: int(t, 0), 0, 0xFFFF), "status": ("status", lambda t: int(t, 0), 0, 0xFFFF)}
SWITCHES = {"silent": "silent"}
USAGE = "error: set NAME VALUE | silent on|off | show | quit"


def apply_command(s, line):
    """Applies one command (stdin or --control) to the state; returns the text to print, or None for quit."""
    w = line.split()
    try:
        if w[:1] == ["quit"]:
            return None
        if w[:1] == ["show"]:
            return ", ".join(f"{k}={v}" for k, v in dataclasses.asdict(s).items() if k != "cell_mv")
        if len(w) == 3 and w[0] == "set" and w[1] in SETTABLE:
            field, conv, lo, hi = SETTABLE[w[1]]
            value = conv(w[2])
            if not math.isfinite(value) or not lo <= value <= hi:
                return f"error: {w[1]} must be {lo} to {hi}"
            setattr(s, field, value)
            if field in ("cells", "max_cell"):
                s.cell_mv = ()
            return f"{w[1]} = {getattr(s, field)}"
        if len(w) == 2 and w[0] in SWITCHES and w[1] in ("on", "off"):
            setattr(s, SWITCHES[w[0]], w[1] == "on")
            return f"{w[0]} {w[1]}"
    except ValueError:
        pass
    return USAGE


def serve(port, s, quiet=False, commands=None, out=say, clock=time.monotonic, control=None):
    serve_frames(port, s, extract_jk, answer_jk, apply_command, quiet, commands, out, clock, control)


def build_parser():
    p = argparse.ArgumentParser(description="JK-B BMS simulator (4E 57 read-all)")
    where = p.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial device on the charger's RS485 line")
    where.add_argument("--bridge", metavar="HOST", help="WiFi bridge host; uses its Passthrough at 115200")
    p.add_argument("--voltage", type=float, default=State.voltage, help="pack V")
    p.add_argument("--soc", type=int, default=State.soc, help="SOC %%")
    p.add_argument("--max-cell", type=float, default=State.max_cell, help="highest cell V")
    p.add_argument("--cells", type=int, choices=(14, 16), default=State.cells, help="cells in the record")
    p.add_argument("--alarms", type=lambda t: int(t, 0), default=0, help="0x8B alarm bits")
    p.add_argument("--status", type=lambda t: int(t, 0), default=State.status, help="0x8C status bits")
    p.add_argument("--control", metavar="FILE", help="also read commands appended to FILE (created if missing)")
    p.add_argument("-q", "--quiet", action="store_true", help="print only changes")
    return p


def main(argv=None, sleep=time.sleep):
    parser = build_parser()
    a = parser.parse_args(argv)
    check_args(parser, a, SETTABLE)
    s = State(voltage=a.voltage, soc=a.soc, max_cell=a.max_cell, cells=a.cells, alarms=a.alarms, status=a.status)
    run_main(a, s, serve, BAUD, sleep)


if __name__ == "__main__":
    main()
