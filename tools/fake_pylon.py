#!/usr/bin/env python3
"""Pylontech BMS simulator in the stock charger's dialect (CID2 0x61/0x62/0x63, ASCII frames, 9600 8N1).

Usage: fake_pylon.py --port /dev/cu.usbserial-XXXX [--voltage 53.4] [--soc 75] [--max-cell 3.345] [--cvl 54.6] [--ccl 30]
                     [--no-charge-enable] [--alarm] [--control FILE] [-q]
       fake_pylon.py --bridge eco-charger-a.local --soc 60       (through the WiFi bridge's Passthrough)

Answers the charger's polls (~2012466xxxxxFDxx) with the encoding of the YamBMS charger proxy. Commands on stdin
change the values live: set ccl|cvl|dvl|dcl|voltage|current|soc|max-cell VALUE, enable on|off, alarm on|off,
silent on|off (no replies at all), show, quit. At the end of stdin it keeps serving (until quit or Ctrl-C).
--control FILE takes the same commands as lines appended to FILE (created if missing; what it already holds is
ignored), so separate shell calls can drive it: echo "set ccl 1.5" >> FILE. -q prints only when a reply differs from
the previous one.
"""
import argparse
import dataclasses
import functools
import math
import os
import queue
import re
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oem_modbus import (PASSTHROUGH_SETTLE_S, bridge_url, open_port, resolve_bridge,  # noqa: E402
                        set_bridge_baud, set_bridge_passthrough)

HEADER = "20024600"  # VER 0x20, ADR 0x02, CID1 0x46, RTN 0x00 (as the YamBMS proxy)
REQUEST = re.compile(r"~20[0-9A-F]{2}46([0-9A-F]{2})0000([0-9A-F]{4})\r")


@dataclasses.dataclass
class State:
    voltage: float = 53.4
    current: float = 0.0
    soc: int = 75
    max_cell: float = 3.345
    cvl: float = 54.6
    dvl: float = 44.0
    ccl: float = 30.0
    dcl: float = 100.0
    charge_enable: bool = True
    alarm: bool = False
    silent: bool = False


def length_field(lenid):
    """LENID with its checksum nibble on top: ~(sum of the three nibbles) + 1."""
    s = (lenid & 0xF) + ((lenid >> 4) & 0xF) + ((lenid >> 8) & 0xF)
    return f"{((~s + 1) & 0xF) << 12 | lenid:04X}"


def checksum(text):
    """CHKSUM of the characters after the '~': two's complement of their ASCII sum, 16 bit."""
    return f"{(~sum(text.encode()) + 1) & 0xFFFF:04X}"


def frame(info_hex):
    body = HEADER + length_field(len(info_hex)) + info_hex
    return "~" + body + checksum(body) + "\r"


def parse_request(text):
    """CID2 of a valid request (~20xx46xx0000cccc CR), else None."""
    m = REQUEST.fullmatch(text)
    if not m or checksum(text[1:-5]) != m.group(2):
        return None
    return int(m.group(1), 16)


def kelvin_x10(celsius):
    return int((celsius + 273.15) * 10)


def info_61(s):
    cell = round(s.max_cell * 1000)
    temps = "".join(f"{kelvin_x10(avg):04X}{kelvin_x10(hi):04X}{a}{kelvin_x10(lo):04X}{b}"
                    for avg, hi, lo, a, b in ((27.5, 30.0, 25.0, "0103", "0109"), (27.5, 30.0, 25.0, "0101", "0101"),
                                              (27.5, 30.0, 25.0, "0101", "0101")))
    return (f"{round(s.voltage * 1000) & 0xFFFF:04X}{round(s.current * 100) & 0xFFFF:04X}{s.soc & 0xFF:02X}"
            f"{10:04X}{10:04X}{100:02X}{100:02X}{cell:04X}0101{cell - 15:04X}0108{temps}")


def info_62(s):
    return "0000" + ("0200" if s.alarm else "0000")


def info_63(s):
    status = (0x80 if s.charge_enable else 0) | 0x40
    return (f"{round(s.cvl * 1000):04X}{round(s.dvl * 1000):04X}{round(s.ccl * 10):04X}{round(s.dcl * 10):04X}"
            f"{status:02X}")


INFOS = {0x61: info_61, 0x62: info_62, 0x63: info_63}


def reply_for(cid2, s):
    if s.silent or cid2 not in INFOS:
        return None
    return frame(INFOS[cid2](s))


def summary(cid2, s):
    return {0x61: f"V={s.voltage:.2f} SOC={s.soc} maxcell={s.max_cell:.3f}",
            0x62: f"alarm={'on' if s.alarm else 'off'}",
            0x63: f"CVL={s.cvl:.2f} CCL={s.ccl:.1f} enable={'on' if s.charge_enable else 'off'}"}[cid2]


# name: (State field, type, lowest, highest) -- what the reply's 16-bit (SOC 8-bit) fields can carry
SETTABLE = {"voltage": ("voltage", float, 0, 65.535), "current": ("current", float, -327.68, 327.67),
            "soc": ("soc", int, 0, 100), "max-cell": ("max_cell", float, 0, 65.535),
            "cvl": ("cvl", float, 0, 65.535), "dvl": ("dvl", float, 0, 65.535),
            "ccl": ("ccl", float, 0, 6553.5), "dcl": ("dcl", float, 0, 6553.5)}
SWITCHES = {"enable": "charge_enable", "alarm": "alarm", "silent": "silent"}


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
    return "error: set NAME VALUE | enable|alarm|silent on|off | show | quit"


def build_parser():
    p = argparse.ArgumentParser(description="Pylontech BMS simulator (stock charger dialect)")
    where = p.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial device on the charger's RS485 line")
    where.add_argument("--bridge", metavar="HOST", help="WiFi bridge host; uses its Passthrough at 9600")
    p.add_argument("--voltage", type=float, default=State.voltage, help="battery V")
    p.add_argument("--soc", type=int, default=State.soc, help="SOC %%")
    p.add_argument("--max-cell", type=float, default=State.max_cell, help="highest cell V")
    p.add_argument("--cvl", type=float, default=State.cvl, help="charge voltage limit, V")
    p.add_argument("--ccl", type=float, default=State.ccl, help="charge current limit, A")
    p.add_argument("--no-charge-enable", action="store_true", help="start with charging not allowed")
    p.add_argument("--alarm", action="store_true", help="start with the alarm bit set")
    p.add_argument("--control", metavar="FILE", help="also read commands appended to FILE (created if missing)")
    p.add_argument("-q", "--quiet", action="store_true", help="print only changes")
    return p


def stdin_reader(commands, stream=None):
    """Queues each stdin line. At its end it stops reading; serving goes on (the simulator may run detached)."""
    for line in stream if stream is not None else sys.stdin:
        commands.put(line)


class ControlFile:
    """Tails FILE for commands: lines appended after it was opened (it's created if missing)."""

    def __init__(self, path):
        self.path, self.partial = path, ""
        with open(path, "a"):
            pass
        self.pos = os.path.getsize(path)

    def lines(self):
        try:
            with open(self.path) as f:
                f.seek(0, os.SEEK_END)
                if f.tell() < self.pos:  # truncated: follow from its new end
                    self.pos, self.partial = f.tell(), ""
                    return []
                f.seek(self.pos)
                text = f.read()
                self.pos = f.tell()
        except OSError:
            return []
        *done, self.partial = (self.partial + text).split("\n")
        return done


CONTROL_POLL_S = 0.1
say = functools.partial(print, flush=True)


def serve(port, s, quiet=False, commands=None, out=say, clock=time.monotonic, control=None):
    buf, last, control_at = "", {}, None
    while True:
        lines = []
        while commands is not None and not commands.empty():
            lines.append(commands.get())
        if control is not None and (control_at is None or clock() - control_at >= CONTROL_POLL_S):
            control_at = clock()
            lines += control.lines()
        for line in filter(str.strip, lines):
            msg = apply_command(s, line)
            if msg is None:
                return
            out(msg)
        data = port.read(256)
        if not data:
            continue
        buf += data.decode("ascii", "replace")
        while "\r" in buf:
            text, buf = buf.split("\r", 1)
            text = text[text.rfind("~"):] + "\r" if "~" in text else ""
            cid2 = parse_request(text)
            reply = reply_for(cid2, s) if cid2 is not None else None
            if reply is None:
                continue
            port.write(reply.encode("ascii"))
            line = f"0x{cid2:02X} -> {summary(cid2, s)}"
            if not quiet or last.get(cid2) != line:
                out(line)
            last[cid2] = line


def main(argv=None, sleep=time.sleep):
    import serial  # pyserial; imported here so the tests don't need a port

    a = build_parser().parse_args(argv)
    s = State(voltage=a.voltage, soc=a.soc, max_cell=a.max_cell, cvl=a.cvl, ccl=a.ccl,
              charge_enable=not a.no_charge_enable, alarm=a.alarm)
    if a.bridge:
        a.bridge = resolve_bridge(a.bridge)
    try:
        if a.bridge:
            # inside the try, so an interrupt during the settle still turns Passthrough off
            set_bridge_passthrough(a.bridge, True)
            sleep(PASSTHROUGH_SETTLE_S)
        name = bridge_url(a.bridge) if a.bridge else a.port
        if a.bridge:
            set_bridge_baud(a.bridge, 9600)
        try:
            port = open_port(serial, name, 9600)
        except serial.SerialException as e:
            print(f"error: cannot open {name}: {e}", file=sys.stderr, flush=True)
            sys.exit(3)
        commands = queue.Queue()
        control = ControlFile(a.control) if a.control else None
        threading.Thread(target=stdin_reader, args=(commands,), daemon=True).start()
        with port:
            try:
                serve(port, s, a.quiet, commands, control=control)
            except KeyboardInterrupt:
                pass
    finally:
        if a.bridge:
            set_bridge_passthrough(a.bridge, False)


if __name__ == "__main__":
    main()
