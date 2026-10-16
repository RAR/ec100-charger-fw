#!/usr/bin/env python3
"""JK-PB BMS simulator: answers the board's FC 0x10 trigger requests (JK RS485 Modbus V1.0) at 115200 8N1.

Usage: fake_jkpb.py --port /dev/cu.usbserial-XXXX [--address 1] [--voltage 52.3] [--soc 64] [--max-cell 3.268]
                    [--cells 16] [--alarms 0] [--charge-mos 1] [--stage 0] [--rcv 3.6] [--rfv 3.5] [--coc 40]
                    [--switch 1] [--control FILE] [-q]
       fake_jkpb.py --bridge eco-charger-a.local --soc 60     (through the WiFi bridge's Passthrough, set to 115200)

A request is `addr 10 RH RL 00 01 02 00 00 CRClo CRChi` (a write of 0 to register 0x1620 = cell info, 0x161E =
settings). Only a well-formed request (good CRC) to --address (the DIP, default 1) and one of those registers is
answered, with the 300-byte record (55 AA EB 90, type 02 or 01) and then the 8-byte reply `addr 10 RH RL 00 01 CRC`.
The records are the real slave 01 ones from docs/history/notes/jk-pb-capture.txt with the fields below patched and the
sum fixed: cell mV (cell n at 6+2n), pack mV (150), alarms (166), SOC (173), charge MOS (198), stage (280); settings RCV
(38), RFV (42), COC mA (50), cell count (114), charge switch (118). Cell voltages in V, --rcv/--rfv per cell, --coc in
A. Commands on stdin change the values live: set voltage|soc|max-cell|cells|alarms|charge-mos|stage|rcv|rfv|coc|switch|
address VALUE, silent on|off (no replies at all), show, quit. --control FILE takes the same commands as lines appended
to FILE (created if missing). -q prints only changes.
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
from oem_image import crc16_modbus  # noqa: E402

BAUD = 115200
REQUEST_LEN = 11
REG_CELL_INFO = 0x1620
REG_SETTINGS = 0x161E

# The real slave 01 records (docs/history/notes/jk-pb-capture.txt): cell info (type 02) and settings (type 01).
CELL_INFO_HEX = (
    "55AAEB900205C40CC40CC40CC40CC40CC40CC40CC30CC40CC40CC40CC40CC40CC40CC40CC40C00000000000000000000"
    "00000000000000000000000000000000000000000000FFFF0000C40C010000053A0037003A0037003A0038003A003700"
    "3B0039003B0038003B003A003C003A000000000000000000000000000000000000000000000000000000000000000000"
    "AE00000000003FCC00000000000000000000A700AC000000080000000040CA14030090CA040000000000160200006400"
    "0000A8FF030001010000000000000000000000000000FF0001000000A803000000009CD73E40000000006C1400000001"
    "010100060000F46B260000000000AE00AB00A900A40380B81F09030000008051010000000100496D00000000000000FE"
    "FF7FDC2F0101B00F0000002D"
)
SETTINGS_HEX = (
    "55AAEB900105AC0D0000140A00003C0A0000420E0000FC0D00000A000000060E0000280A0000100E0000AC0D0000C409"
    "0000409C0000030000003C000000F04902002C0100003C00000005000000E8030000BC02000058020000BC0200005802"
    "00001400000032000000E8030000200300001000000001000000010000000100000090CA0400DC050000B80B00000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000010000000000000060E3160000323C3218FE"
    "FFFFFF9FE91D0200000000E6"
)
CELL_INFO = bytes.fromhex(CELL_INFO_HEX)
SETTINGS = bytes.fromhex(SETTINGS_HEX)
assert len(CELL_INFO) == len(SETTINGS) == 300


@dataclasses.dataclass
class State:
    address: int = 1
    voltage: float = 52.287
    soc: int = 64
    max_cell: float = 3.268
    cells: int = 16
    alarms: int = 0
    charge_mos: int = 1
    stage: int = 0
    rcv: float = 3.6  # CellRCV, V per cell
    rfv: float = 3.5  # CellRFV, V per cell
    coc: float = 40.0  # CurBatCOC, A
    switch: int = 1  # charge switch
    silent: bool = False
    cell_mv: tuple = ()  # explicit cell voltages (overrides cells/max_cell), for exact reproductions


def cell_list(s):
    """Millivolts of each cell: the first is the highest."""
    if s.cell_mv:
        return list(s.cell_mv)
    top = round(s.max_cell * 1000)
    return [max(0, top - (i % 3) * 2) for i in range(s.cells)]


def with_sum(r):
    r[299] = sum(r[:299]) & 0xFF
    return bytes(r)


def cell_info_record(s):
    """The 300-byte type 02 record: the captured one with the cells, pack, alarms, SOC, charge MOS and stage patched."""
    r = bytearray(CELL_INFO)
    cells = cell_list(s)
    for n in range(32):
        struct.pack_into("<H", r, 6 + 2 * n, cells[n] if n < len(cells) else 0)
    struct.pack_into("<I", r, 150, round(s.voltage * 1000))
    struct.pack_into("<I", r, 166, s.alarms)
    r[173], r[198], r[280] = s.soc, s.charge_mos, s.stage
    return with_sum(r)


def settings_record(s):
    """The 300-byte type 01 record: the captured one with RCV, RFV, COC, cell count and charge switch patched."""
    r = bytearray(SETTINGS)
    struct.pack_into("<I", r, 38, round(s.rcv * 1000))
    struct.pack_into("<I", r, 42, round(s.rfv * 1000))
    struct.pack_into("<I", r, 50, round(s.coc * 1000))
    struct.pack_into("<I", r, 114, len(cell_list(s)))
    struct.pack_into("<I", r, 118, s.switch)
    return with_sum(r)


def trailer(addr, reg):
    body = bytes([addr, 0x10, reg >> 8, reg & 0xFF, 0x00, 0x01])
    return body + struct.pack("<H", crc16_modbus(body))


def build_request(addr, reg):
    body = bytes([addr, 0x10, reg >> 8, reg & 0xFF, 0x00, 0x01, 0x02, 0x00, 0x00])
    return body + struct.pack("<H", crc16_modbus(body))


def extract_jkpb(buf):
    """(consumed, (addr, reg) or None) for the front of buf; (0, None) when more bytes are needed."""
    fixed = {1: 0x10, 2: 0x16, 4: 0x00, 5: 0x01, 6: 0x02, 7: 0x00, 8: 0x00}
    if any(i < len(buf) and buf[i] != v for i, v in fixed.items()) or (len(buf) > 3 and buf[3] not in (0x20, 0x1E)):
        return 1, None
    if len(buf) < REQUEST_LEN:
        return 0, None
    f = bytes(buf[:REQUEST_LEN])
    if struct.pack("<H", crc16_modbus(f[:9])) != f[9:]:
        return 1, None
    return REQUEST_LEN, (f[0], f[2] << 8 | f[3])


def reply_for(addr, reg, s):
    """Record plus trailer for a request to this simulator's address, else None."""
    if s.silent or addr != s.address:
        return None
    if reg == REG_CELL_INFO:
        return cell_info_record(s) + trailer(addr, reg)
    if reg == REG_SETTINGS:
        return settings_record(s) + trailer(addr, reg)
    return None


def answer_jkpb(req, s):
    reply = reply_for(*req, s)
    if reply is None:
        return None, ""
    if req[1] == REG_SETTINGS:
        return reply, (f"settings addr={s.address} cells={len(cell_list(s))} rcv={s.rcv:.3f} rfv={s.rfv:.3f} "
                       f"coc={s.coc:g}A switch={s.switch}")
    top = max(cell_list(s), default=0) / 1000
    return reply, (f"cell-info addr={s.address} V={s.voltage:.3f} SOC={s.soc} maxcell={top:.3f} "
                   f"alarms=0x{s.alarms:08X} mos={s.charge_mos} stage={s.stage}")


def hexint(t):
    return int(t, 0)


# name: (State field, converter, lowest, highest)
SETTABLE = {"address": ("address", int, 0, 15), "voltage": ("voltage", float, 0, 4294967.295),
            "soc": ("soc", int, 0, 255), "max-cell": ("max_cell", float, 0, 4.999), "cells": ("cells", int, 0, 32),
            "alarms": ("alarms", hexint, 0, 0xFFFFFFFF), "charge-mos": ("charge_mos", int, 0, 255),
            "stage": ("stage", int, 0, 255), "rcv": ("rcv", float, 0, 4294967.295),
            "rfv": ("rfv", float, 0, 4294967.295), "coc": ("coc", float, 0, 4294967.295),
            "switch": ("switch", int, 0, 0xFFFFFFFF)}
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
    serve_frames(port, s, extract_jkpb, answer_jkpb, apply_command, quiet, commands, out, clock, control)


def build_parser():
    p = argparse.ArgumentParser(description="JK-PB BMS simulator (FC 0x10 trigger, 300-byte records)")
    where = p.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial device on the charger's RS485 line")
    where.add_argument("--bridge", metavar="HOST", help="WiFi bridge host; uses its Passthrough at 115200")
    p.add_argument("--address", type=int, default=State.address, help="Modbus address (the BMS DIP), 0-15")
    p.add_argument("--voltage", type=float, default=State.voltage, help="pack V")
    p.add_argument("--soc", type=int, default=State.soc, help="SOC %%")
    p.add_argument("--max-cell", type=float, default=State.max_cell, help="highest cell V")
    p.add_argument("--cells", type=int, default=State.cells, help="cells in the record and the settings, 0-32")
    p.add_argument("--alarms", type=hexint, default=0, help="alarm bits (u32 at 166; the board stops on 0x00E107EF)")
    p.add_argument("--charge-mos", type=int, default=State.charge_mos, help="charge MOSFET byte (1 = on)")
    p.add_argument("--stage", type=int, default=State.stage, help="charge stage (0 bulk, 1 absorption, 2 float)")
    p.add_argument("--rcv", type=float, default=State.rcv, help="CellRCV, V per cell")
    p.add_argument("--rfv", type=float, default=State.rfv, help="CellRFV, V per cell")
    p.add_argument("--coc", type=float, default=State.coc, help="CurBatCOC, max charge A")
    p.add_argument("--switch", type=int, default=State.switch, help="charge switch (1 = on)")
    p.add_argument("--control", metavar="FILE", help="also read commands appended to FILE (created if missing)")
    p.add_argument("-q", "--quiet", action="store_true", help="print only changes")
    return p


def main(argv=None, sleep=time.sleep):
    parser = build_parser()
    a = parser.parse_args(argv)
    check_args(parser, a, SETTABLE)
    s = State(address=a.address, voltage=a.voltage, soc=a.soc, max_cell=a.max_cell, cells=a.cells, alarms=a.alarms,
              charge_mos=a.charge_mos, stage=a.stage, rcv=a.rcv, rfv=a.rfv, coc=a.coc, switch=a.switch)
    run_main(a, s, serve, BAUD, sleep)


if __name__ == "__main__":
    main()
