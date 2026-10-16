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

import gs_usb_macos  # noqa: F401  (macOS gs_usb workarounds)

SETPOINT_SCALE = 1023
TELEMETRY_SCALE = 1024
NOMINAL_A = 52.0
WRITE_CMD, READ_CMD = 0x81, 0x82
IDLE_HEATSINK_C = 35.0


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
        self.heatsink_c = IDLE_HEATSINK_C
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
                f"ilim={self.ilimit:5.1f} A defaults={self.default_v:.2f} V/{self.default_a:.1f} A "
                f"heatsink={self.heatsink_c:.1f} C"
                f"{' MUTED' if self.muted else ''}{f' ALARM={self.alarm:#06x}' if self.alarm else ''}")


def in_window(t, start, until):
    """True while start <= t < until; until None means for the rest of the run."""
    return t >= start and (until is None or t < until)


def injection_events(prev, cur):
    """Messages for the injection windows ({"alarm": on, "heatsink": on}) that started or ended between two samples."""
    return [f"{k} injection {'starts' if cur[k] else 'ends'}" for k in ("alarm", "heatsink") if cur[k] != prev[k]]


def parse_args(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=600)
    ap.add_argument("--addrs", default="1,2")
    ap.add_argument("--mute", type=int, action="append", default=[], help="address that never answers")
    ap.add_argument("--alarm", type=int, action="append", default=[], help="address reporting alarm bit 0x0001 (E8)")
    ap.add_argument("--alarm-from", type=float, default=0, help="seconds into the run the alarm starts")
    ap.add_argument("--alarm-until", type=float, default=None, help="seconds into the run the alarm clears")
    ap.add_argument("--heatsink", type=float, default=IDLE_HEATSINK_C, help="heatsink in C (cooldown starts at >= 45)")
    ap.add_argument("--heatsink-until", type=float, default=None, help="seconds into the run it drops back to 35 C")
    ap.add_argument("--fallback-after", type=float, default=60)
    ap.add_argument("--battery-v", type=float, default=52.0)
    a = ap.parse_args(argv)
    a.addr_list = [int(x) for x in a.addrs.split(",")]
    for opt in ("alarm", "mute"):
        unknown = sorted(set(getattr(a, opt)) - set(a.addr_list))
        if unknown:
            ap.error(f"--{opt} {unknown[0]} is not one of --addrs {a.addrs}")
    return a


def main():
    a = parse_args()
    mods = {addr: FakeNasn(addr, battery_v=a.battery_v) for addr in a.addr_list}
    for addr in a.mute:
        mods[addr].muted = True

    bus = can.Bus(interface="gs_usb", channel=0, index=0, bitrate=125000)
    t0 = last_rx = next_status = time.time()
    fell_back = False
    injecting = {"alarm": False, "heatsink": False}
    print(f"fake NASN at {sorted(mods)} for {a.seconds:.0f}s", flush=True)
    try:
        while time.time() - t0 < a.seconds:
            msg = bus.recv(timeout=0.2)
            now = time.time()
            alarm_on = in_window(now - t0, a.alarm_from, a.alarm_until)
            hot = in_window(now - t0, 0, a.heatsink_until)
            for mod in mods.values():
                mod.alarm = 0x0001 if mod.addr in a.alarm and alarm_on else 0
                mod.heatsink_c = a.heatsink if hot else IDLE_HEATSINK_C
            cur = {"alarm": bool(a.alarm) and alarm_on, "heatsink": a.heatsink != IDLE_HEATSINK_C and hot}
            for event in injection_events(injecting, cur):
                print(f"({now - t0:8.3f}) {event}", flush=True)
            injecting = cur
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
                        print(f"({now - t0:8.3f}) write {msg.arbitration_id:08X}#{data.hex(' ')}  -> {mod.status()}",
                              flush=True)
            if not fell_back and now - last_rx > a.fallback_after:
                for mod in mods.values():
                    mod.reset_to_defaults()
                fell_back = True
                print(f"({now - t0:8.3f}) CAN silent {a.fallback_after:.0f}s: modules revert to stored defaults",
                      flush=True)
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
