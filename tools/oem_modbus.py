#!/usr/bin/env python3
"""Modbus RTU client for the OEM board firmware (spec §4): 9600 8N1, address 1 by default, on the RS485 jack.

Usage: oem_modbus.py --port /dev/cu.usbserial-XXXX status [--watch SECONDS]
       oem_modbus.py --port ... get 0 1 2
       oem_modbus.py --port ... set 1=5700 5=5700      (settings: read HR 0-16, then one FC 16, checked as a whole)
       oem_modbus.py --port ... pause on|off | charge-now | clear-fault | relay on|off | reboot-bootloader
       oem_modbus.py --port ... diag VOLTS AMPS_PER_MODULE psu1|psu2|both SECONDS | diag off

A set of settings only (HR 0-16) reads HR 0-16, applies the values and writes the span from the lowest to the highest
given register as one FC 16, so the firmware validates the result as a whole and the order doesn't matter. Other
registers go out as one write per run of consecutive registers; if a later one fails, those already applied are
printed.

Exit status: 0 ok, 1 exception reply (its name is printed), 2 no or unusable reply.
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oem_image import crc16_modbus  # noqa: E402

HR_SETTINGS = 17  # HR 0-16
HR_PAUSE, HR_CLEAR_FAULT, HR_RELAY_CMD, HR_CHARGE_NOW, HR_REBOOT = 110, 111, 112, 113, 120
HR_DIAG_SECONDS = 130  # 130 seconds, 131 V x100, 132 A x10 per module, 133 module mask (firmware 3.1)
REBOOT_MAGIC = 0xB007
IR_COUNT = 40
NA_U, NA_S = 0xFFFF, 0x7FFF

EXCEPTIONS = {1: "illegal function", 2: "illegal address", 3: "illegal value",
              6: "busy (the menu is open on the panel)"}
STATES = ["BOOT", "WAIT_BMS", "CHARGING", "HOLD", "FAULT"]
HOLD_REASONS = ["none", "paused", "not allowed", "BMS alarm", "BMS blocking", "cell high", "budget < 2 A",
                "waiting for start"]
ERRORS = {0: "none", 12: "BMS alarm", 13: "setpoint mismatch", 14: "output overvoltage", 15: "source stale (warning)",
          16: "charging without current (warning)", 17: "battery reading implausible"}
ERRORS.update({n: "module alarm" for n in range(1, 12)})
FLAGS = ["cooling", "no current", "source stale", "session active", "relay closed", "AC present", "PSU1 online",
         "PSU2 online", "diagnostic override"]
SOURCES = ["Panel", "Pylontech", "Modbus", "EG4", "JK", "JK-PB"]
PSU_STATUS = ["online", "faulted", "cooling", "standby"]
HR_ADC_TRIM = 14  # signed (int16)
HR_NAMES = {0: "Max current, A", 1: "Charge V, V x100", 2: "Start SOC, %", 3: "Stop SOC, %", 4: "Start V, V x100",
            5: "Stop V, V x100", 6: "Source mode", 7: "Modbus address", 8: "Modbus baud", 9: "Fallback current, A",
            10: "Relay mode", 11: "Backlight timeout, s", 12: "Max cell V, V x1000", 13: "Cell resume V, V x1000",
            14: "ADC trim, per mille", 15: "Module fallback V, V x100", 16: "Module fallback A per module"}


class ModbusError(Exception):
    """No usable reply: silence, a short or bad-CRC frame, or a reply that doesn't match the request."""


class ModbusException(Exception):
    def __init__(self, code):
        self.code = code
        super().__init__(f"exception {code:02d}: {EXCEPTIONS.get(code, 'unknown')}")


def with_crc(body):
    return body + struct.pack("<H", crc16_modbus(body))


def read_request(addr, fc, reg, count):
    return with_crc(struct.pack(">BBHH", addr, fc, reg, count))


def write_single(addr, reg, value):
    return with_crc(struct.pack(">BBHH", addr, 6, reg, value & 0xFFFF))


def write_multiple(addr, reg, values):
    return with_crc(struct.pack(">BBHHB", addr, 16, reg, len(values), 2 * len(values))
                    + b"".join(struct.pack(">H", v & 0xFFFF) for v in values))


def parse_response(req, resp):
    """Returns the register words of an FC 03/04 reply, or [] for a write ack. Raises ModbusException / ModbusError."""
    if len(resp) < 5:
        raise ModbusError(f"short reply ({resp.hex() or 'nothing'})")
    if struct.pack("<H", crc16_modbus(resp[:-2])) != resp[-2:]:
        raise ModbusError(f"bad CRC in reply {resp.hex()}")
    if resp[0] != req[0]:
        raise ModbusError(f"reply from address {resp[0]}, expected {req[0]}")
    if resp[1] == req[1] | 0x80:
        raise ModbusException(resp[2])
    if resp[1] != req[1]:
        raise ModbusError(f"reply function {resp[1]:#04x} does not match the request")
    if req[1] in (3, 4):
        count = struct.unpack(">H", req[4:6])[0]
        if resp[2] != 2 * count or len(resp) != 5 + 2 * count:
            raise ModbusError(f"expected {count} registers, got {resp.hex()}")
        return list(struct.unpack(f">{count}H", resp[3:-2]))
    if resp[:6] != req[:6] or len(resp) != 8:
        raise ModbusError(f"write not acknowledged as sent ({resp.hex()})")
    return []


def read_exact(port, n, timeout, clock=time.monotonic):
    buf = b""
    end = clock() + timeout
    while len(buf) < n and clock() < end:
        buf += port.read(n - len(buf))
    return buf


def reply_length(req, head):
    """Reply length from its first three bytes, or None while undecidable."""
    if head[1] & 0x80:
        return 5
    return 5 + head[2] if req[1] in (3, 4) else 8


PASSTHROUGH_SETTLE_S = 0.5  # the web server answers before the switch action runs, and a poll may be in flight
BRIDGE_PORT = 8899  # esphome/oem-bridge.yaml stream_server


def resolve_bridge(host, lookup=None):
    """The bridge's IPv4 address. macOS spends ~5 s on every new connection to a .local name (an IPv6 mDNS lookup
    first), which is longer than the bootloader's 1.5 s window, so the tools look the name up once."""
    if lookup is None:
        import socket
        lookup = socket.gethostbyname
    try:
        return lookup(host)
    except OSError:
        return host  # let the connection report it


def bridge_url(host):
    return f"socket://{host}:{BRIDGE_PORT}"


def auth_header(env=None):
    """HTTP basic auth for the bridge's web server, from OEM_BRIDGE_USER / OEM_BRIDGE_PASSWORD (empty if unset)."""
    import base64
    env = os.environ if env is None else env
    user, password = env.get("OEM_BRIDGE_USER"), env.get("OEM_BRIDGE_PASSWORD")
    if not user or password is None:
        return {}
    return {"Authorization": "Basic " + base64.b64encode(f"{user}:{password}".encode()).decode()}


def bridge_post(url, env=None):
    import urllib.request
    req = urllib.request.Request(url, data=b"", method="POST", headers=auth_header(env))
    urllib.request.urlopen(req, timeout=5).read()


def set_bridge_baud(host, baud, post=None):
    """Switches the WiFi bridge's RS485 line rate (its "RS485 baud" select, through the ESPHome web server)."""
    (post or bridge_post)(f"http://{host}/select/RS485%20baud/set?option={baud}")  # ESPHome addresses entities by name


def set_bridge_passthrough(host, on, post=None):
    """Hands the bridge's RS485 line to the TCP stream (on) or back to its Modbus polling (off)."""
    (post or bridge_post)(f"http://{host}/switch/Passthrough/turn_{'on' if on else 'off'}")


def open_port(serial, name, baud):
    # pyserial asserts RTS and DTR on open and some adapters use RTS for TX-enable; the vendor tool keeps both off.
    # A URL (socket://host:port for the WiFi bridge) needs serial_for_url; its baud is set on the bridge instead.
    port = serial.serial_for_url(name, do_not_open=True) if "://" in name else serial.Serial()
    port.port, port.baudrate, port.timeout = name, baud, 0.02
    port.rts = port.dtr = False
    port.open()
    return port


def read_frame(port, req, timeout, clock, head=b""):
    head += read_exact(port, 3 - len(head), timeout, clock)
    if len(head) < 3:
        raise ModbusError("no reply" if not head else f"short reply ({head.hex()})")
    if req[1] in (3, 4) and head == req[:3]:  # a read reply never starts like its request: an adapter echo
        read_exact(port, len(req) - 3, timeout, clock)
        return read_frame(port, req, timeout, clock)
    if req[1] == 16 and head == req[:3]:
        # The ack and the echo both start like the request, but the echo's bytes 6-7 are the byte count and data.
        frame = head + read_exact(port, 5, timeout, clock)
        if frame == req[:8]:
            read_exact(port, len(req) - 8, timeout, clock)
            return read_frame(port, req, timeout, clock)
        return frame
    return head + read_exact(port, reply_length(req, head) - 3, timeout, clock)


def transact(port, req, timeout=0.5, clock=time.monotonic):
    # An FC 06 ack is identical to its request, so an echoing adapter can't be told apart from a board that
    # answered; callers that need to know the board is alive read a register first (see oem_update.modbus_reboot).
    port.reset_input_buffer()
    port.write(req)
    return parse_response(req, read_frame(port, req, timeout, clock))


def read_registers(port, addr, fc, reg, count, timeout=0.5):
    return transact(port, read_request(addr, fc, reg, count), timeout)


def runs(items):
    """Splits [(reg, value)] into lists of consecutive registers."""
    out = []
    for reg, v in items:
        if out and out[-1][-1][0] + 1 == reg:
            out[-1].append((reg, v))
        else:
            out.append([(reg, v)])
    return out


def write_registers(port, addr, items, timeout=0.5, applied=None):
    """One write per run of consecutive registers; each acked register is appended to `applied`."""
    for run in runs(items):
        if len(run) == 1:
            req = write_single(addr, run[0][0], run[0][1])
        else:
            req = write_multiple(addr, run[0][0], [v for _, v in run])
        transact(port, req, timeout)
        if applied is not None:
            applied.extend(r for r, _ in run)


def write_settings(port, addr, items, timeout=0.5):
    """Settings (HR 0-16) as one FC 16 over the current values, so the clamps see the final result, not each step."""
    hr = read_registers(port, addr, 3, 0, HR_SETTINGS, timeout)
    for reg, v in items:
        hr[reg] = v
    lo, hi = min(r for r, _ in items), max(r for r, _ in items)
    transact(port, write_multiple(addr, lo, hr[lo:hi + 1]), timeout)


def signed(v):
    return v - 0x10000 if v & 0x8000 else v


def scaled(v, scale, unit, na=NA_U, sign=False):
    if v == (NA_S if sign else na):
        return "n/a"
    x = signed(v) if sign else v
    return f"{x / scale:.{len(str(scale)) - 1}f}{unit}"


def name_of(table, v):
    if isinstance(table, dict):
        return table.get(v, "?")
    return table[v] if v < len(table) else "?"


def decode_status(ir):
    """Lines describing a 40-register input block."""
    if len(ir) < IR_COUNT:
        raise ValueError(f"need {IR_COUNT} registers, got {len(ir)}")
    uptime = (ir[16] << 16) | ir[17]
    flags = [FLAGS[b] for b in range(len(FLAGS)) if ir[4] >> b & 1]
    lines = [
        f"version      {ir[0] >> 8}.{ir[0] & 0xFF}",
        f"state        {ir[1]} {name_of(STATES, ir[1])}",
        f"hold reason  {ir[2]} {name_of(HOLD_REASONS, ir[2])}",
        f"error        E{ir[3]} {name_of(ERRORS, ir[3])}" if ir[3] else "error        none",
        f"flags        {', '.join(flags) or 'none'}",
        f"source       {ir[5]} {name_of(SOURCES, ir[5])}",
        f"target       {scaled(ir[6], 100, ' V')}  {scaled(ir[7], 10, ' A')}",
        f"budget       {scaled(ir[8], 10, ' A')}",
        f"output       {scaled(ir[9], 10, ' A')}  {'n/a' if ir[10] == NA_U else f'{ir[10]} W'}",
        f"battery V    {scaled(ir[11], 100, ' V')} in use, {scaled(ir[12], 100, ' V')} ADC",
        f"SOC          {'n/a' if ir[13] == NA_U else f'{ir[13]} %'}",
        f"CVL / CCL    {scaled(ir[14], 100, ' V')}  {scaled(ir[15], 10, ' A')}",
        f"uptime       {uptime} s",
    ]
    for k, base in enumerate((20, 30), start=1):
        p = ir[base:base + 10]
        st = [PSU_STATUS[b] for b in range(4) if p[0] >> b & 1]
        alarm = "n/a" if p[8] == NA_U and p[9] == NA_U else f"{p[8]:04X}{p[9]:04X}"
        lines.append(
            f"PSU{k}         {', '.join(st) or 'offline'}; {scaled(p[1], 100, ' V')} {scaled(p[2], 10, ' A')}"
            f" AC {scaled(p[3], 10, ' V')} inlet {scaled(p[4], 10, ' C', sign=True)}"
            f" heatsink {scaled(p[5], 10, ' C', sign=True)} fans {'n/a' if p[6] == NA_U else p[6]}/"
            f"{'n/a' if p[7] == NA_U else p[7]} alarm {alarm}")
    return lines


def parse_assignment(text):
    reg, sep, val = text.partition("=")
    if not sep:
        raise argparse.ArgumentTypeError(f"{text!r}: expected REG=VALUE")
    try:
        r, v = int(reg, 0), int(val, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(f"{text!r}: expected integers") from None
    if not 0 <= r <= 0xFFFF or not -0x8000 <= v <= 0xFFFF:
        raise argparse.ArgumentTypeError(f"{text!r}: out of range")
    return r, v & 0xFFFF


def run(port, a, out=print, sleep=time.sleep):
    cmd = a.cmd
    if cmd == "status":
        while True:
            for line in decode_status(read_registers(port, a.addr, 4, 0, IR_COUNT, a.timeout)):
                out(line)
            if not a.watch:
                return
            out("")
            sleep(a.watch)
    elif cmd == "get":
        for run_ in runs([(r, None) for r in sorted(set(a.regs))]):
            vals = read_registers(port, a.addr, 3, run_[0][0], len(run_), a.timeout)
            for (reg, _), v in zip(run_, vals):
                shown = v - 0x10000 if reg == HR_ADC_TRIM and v >= 0x8000 else v  # HR 14 is an int16
                out(f"HR {reg:3d} = {shown} (0x{v:04X}){'  ' + HR_NAMES[reg] if reg in HR_NAMES else ''}")
    elif cmd == "set":
        if all(r < HR_SETTINGS for r, _ in a.assignments):
            write_settings(port, a.addr, a.assignments, a.timeout)
        else:
            applied = []
            try:
                write_registers(port, a.addr, a.assignments, a.timeout, applied)
            except (ModbusException, ModbusError):
                if applied:
                    out("already applied: " + ", ".join(f"HR {r}" for r in applied))
                raise
        out("ok")
    elif cmd == "diag":
        if a.args == ["off"]:
            items = [(HR_DIAG_SECONDS, 0)]
        else:
            if len(a.args) != 4 or a.args[2] not in DIAG_MASKS:
                raise SystemExit("diag VOLTS AMPS_PER_MODULE psu1|psu2|both SECONDS, or diag off")
            volts, amps, seconds = float(a.args[0]), float(a.args[1]), int(a.args[3])
            items = [(HR_DIAG_SECONDS, seconds), (HR_DIAG_SECONDS + 1, round(volts * 100)),
                     (HR_DIAG_SECONDS + 2, round(amps * 10)), (HR_DIAG_SECONDS + 3, DIAG_MASKS[a.args[2]])]
        write_registers(port, a.addr, items, a.timeout)
        out("ok")
    else:
        items = {"pause": [(HR_PAUSE, int(a.value == "on") if cmd == "pause" else 0)],
                 "charge-now": [(HR_CHARGE_NOW, 1)], "clear-fault": [(HR_CLEAR_FAULT, 1)],
                 "relay": [(HR_RELAY_CMD, int(getattr(a, "value", "") == "on"))],
                 "reboot-bootloader": [(HR_REBOOT, REBOOT_MAGIC)]}[cmd]
        write_registers(port, a.addr, items, a.timeout)
        out("ok")


DIAG_MASKS = {"psu1": 1, "psu2": 2, "both": 3}


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial device, or a pyserial URL such as socket://host:8899")
    where.add_argument("--bridge", metavar="HOST", help="the WiFi bridge (esphome/oem-bridge.yaml) at HOST")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--addr", type=int, default=1)
    ap.add_argument("--timeout", type=float, default=0.5)
    sub = ap.add_subparsers(dest="cmd", required=True)
    st = sub.add_parser("status", help="read and decode IR 0-39")
    st.add_argument("--watch", type=float, metavar="SECONDS", help="repeat at this interval")
    g = sub.add_parser("get", help="read holding registers")
    g.add_argument("regs", nargs="+", type=lambda s: int(s, 0))
    s = sub.add_parser("set", help="write holding registers")
    s.add_argument("assignments", nargs="+", type=parse_assignment, metavar="REG=VALUE")
    for name in ("pause", "relay"):
        sub.add_parser(name).add_argument("value", choices=["on", "off"])
    for name in ("charge-now", "clear-fault", "reboot-bootloader"):
        sub.add_parser(name)
    d = sub.add_parser("diag", help="hold modules out of standby at V / A for a while (firmware 3.1), or 'diag off'")
    d.add_argument("args", nargs="+", metavar="VOLTS AMPS psu1|psu2|both SECONDS | off")
    return ap


def port_name(a):
    return bridge_url(a.bridge) if a.bridge else a.port


def main(argv=None, sleep=time.sleep):
    import serial  # pyserial; imported here so the tests don't need a port

    a = build_parser().parse_args(argv)
    if a.bridge:
        a.bridge = resolve_bridge(a.bridge)
    try:
        if a.bridge:
            # inside the try, so an interrupt during the settle still turns Passthrough off
            set_bridge_passthrough(a.bridge, True)
            sleep(PASSTHROUGH_SETTLE_S)
        try:
            port = open_port(serial, port_name(a), a.baud)
        except serial.SerialException as e:
            print(f"error: cannot open {port_name(a)}: {e}", file=sys.stderr)
            sys.exit(3)
        with port:
            try:
                run(port, a)
            except ModbusException as e:
                sys.exit(f"{e}")
            except ModbusError as e:
                print(f"error: {e}", file=sys.stderr)
                sys.exit(2)
            except KeyboardInterrupt:
                sys.exit(130)
    finally:
        if a.bridge:
            set_bridge_passthrough(a.bridge, False)


if __name__ == "__main__":
    main()
