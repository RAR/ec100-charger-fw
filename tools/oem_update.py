#!/usr/bin/env python3
"""Install a .dat image through the stock Eco-Worthy bootloader (RS485, 460800 8N1).

Protocol (docs/history/notes/oem-hw-map.md §6.2):
- Handshake: F9 45 F9 53 23 -> F9 45 03 00 21 9D.
- Header: a frame carrying the 4-byte big-endian image length. The first one after a reset erases the app area.
- Data: frames of <= 256 encrypted bytes, each acked with its sequence number echoed and a Modbus CRC (checked).

The bootloader programs every CRC-good frame at a running offset without checking the sequence number, so a frame
is never resent. Any error after the header aborts, and the install must restart from a power-cycle: the bootloader erases the app
area only once per boot.

With --modbus-reboot the running app is first asked over Modbus (HR 120 = 0xB007) to reset into the bootloader, so no
power-cycle is needed. The bootloader's window is only 1.5 s, so the handshake starts right after the reply.

Usage: oem_update.py --port /dev/cu.usbserial-XXXX image.dat [--wait 30] [--frame-bytes 256]
                     [--modbus-reboot [--modbus-baud 9600] [--modbus-addr 1]]
"""
import argparse
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oem_image import MAX_LEN, crc16_modbus  # noqa: E402
from oem_modbus import (HR_REBOOT, PASSTHROUGH_SETTLE_S, REBOOT_MAGIC, ModbusError, ModbusException,  # noqa: E402
                        bridge_url, open_port, read_exact, read_request, resolve_bridge, set_bridge_baud, set_bridge_passthrough,
                        transact, write_single)

BAUD = 460800
HANDSHAKE = bytes.fromhex("F945F95323")
HANDSHAKE_REPLY = bytes.fromhex("F9450300219D")
FRAME_BYTES = 256
# Through the WiFi bridge: a 73-byte frame fits the ESP32's 128-byte UART TX FIFO, so no refill gap splits it (the
# bootloader ends a frame at an idle line, and a split frame is never acked).
BRIDGE_FRAME_BYTES = 64
HEADER_ACK_S = 5.0  # erasing 100 pages
DATA_ACK_S = 3.0  # a WiFi bridge has delayed an ack past 1 s (frame 187 of 192)
QUIET_S = 2.0  # > 1.5 s of silence makes the bootloader jump to the app


POWER_CYCLE = "power-cycle the board before retrying (the bootloader only erases once per boot)"
REPLY_GAP_S = 1.0  # the two handshake replies must come this close together
# Past 3.12's standby second and the 1.5 s window: a board that acked HR 120 but never answered missed it.
REBOOT_HINT_S = 5.0


class UpdateError(Exception):
    pass


def data_frame(seq, chunk):
    body = b"\xfb" + struct.pack(">H", len(chunk)) + b"\x03\x00" + struct.pack(">H", seq) + chunk
    return body + struct.pack("<H", crc16_modbus(body))


def header_frame(length):
    return data_frame(0, struct.pack(">I", length))


def handshake(port, wait_s, clock, sleep, log, rebooted=False):
    """Returns True if the adapter echoes what it transmits."""
    # The stock app answers once, then resets into the bootloader, so a reply only counts once it has come twice.
    if rebooted:
        log(f"the board was asked to reboot into the bootloader; waiting up to {wait_s:.0f} s for it")
    else:
        log(f"waiting up to {wait_s:.0f} s for the bootloader: power-cycle the board now")
    start = clock()
    end = start + wait_s
    hint = rebooted
    replies = 0
    last = 0.0
    echo = False
    while clock() < end:
        if hint and clock() - start >= REBOOT_HINT_S:
            log("no bootloader reply; power-cycle the board now")
            hint = False
        port.reset_input_buffer()
        port.write(HANDSHAKE)
        got = read_exact(port, len(HANDSHAKE) + len(HANDSHAKE_REPLY), 0.1, clock)
        if got.startswith(HANDSHAKE):
            echo = True
            got = got[len(HANDSHAKE):]
        if got == HANDSHAKE_REPLY:
            hint = False
            now = clock()
            replies = replies + 1 if replies and now - last <= REPLY_GAP_S else 1
            last = now
            if replies == 2:
                if echo:
                    log("adapter echoes its own transmission; compensating")
                return echo
            sleep(0.2)
    raise UpdateError("no bootloader handshake")


def send(port, frame, echo, clock):
    port.reset_input_buffer()  # like the vendor tool; the echo, if any, arrives after the write
    port.write(frame)
    if echo and read_exact(port, len(frame), DATA_ACK_S, clock) != frame:
        raise UpdateError("the adapter's echo of a frame was corrupted, so the line is bad; " + POWER_CYCLE)


def ack_ok(ack, seq):
    """Acks are FB 03 00 sH sL cL cH: the sequence number echoed, then the Modbus CRC of the first 5 bytes."""
    return (len(ack) == 7 and ack[:5] == b"\xfb\x03\x00" + struct.pack(">H", seq)
            and ack[5:] == struct.pack("<H", crc16_modbus(ack[:5])))


def check_image(dat):
    if not dat or len(dat) % 8:
        raise UpdateError(f"image length {len(dat)} is not a positive multiple of 8")
    if len(dat) > MAX_LEN:
        raise UpdateError(f"image is {len(dat)} bytes; the bootloader takes at most {MAX_LEN}")


def install(port, dat, wait_s=30.0, clock=time.monotonic, sleep=time.sleep, log=print, frame_bytes=FRAME_BYTES,
            rebooted=False):
    check_image(dat)
    echo = handshake(port, wait_s, clock, sleep, log, rebooted)
    send(port, header_frame(len(dat)), echo, clock)
    ack = read_exact(port, 7, HEADER_ACK_S, clock)
    if not ack_ok(ack, 0):
        raise UpdateError(f"no header ack (got {ack.hex() or 'nothing'}); " + POWER_CYCLE)
    frames = (len(dat) + frame_bytes - 1) // frame_bytes
    for seq, off in enumerate(range(0, len(dat), frame_bytes), start=1):
        send(port, data_frame(seq, dat[off:off + frame_bytes]), echo, clock)
        ack = read_exact(port, 7, DATA_ACK_S, clock)
        if not ack_ok(ack, seq):
            raise UpdateError(f"frame {seq}/{frames}: bad or missing ack, or bad ack CRC (got {ack.hex() or 'nothing'}); " + POWER_CYCLE)
        if seq % 20 == 0 or seq == frames:
            log(f"  {seq}/{frames} frames")
    sleep(QUIET_S)
    log("done: the bootloader should now start the new image")


MODBUS_REPLY_S = 0.5


def modbus_reboot(port, addr, log=print, clock=time.monotonic):
    """Asks the running app to reset into the bootloader. Returns True if it did not refuse and the board answered.

    An IR 0 read goes first: a half-duplex adapter echoes what it sends, and an echo can't pass for a register
    reply, whereas the FC 06 ack is identical to its request."""
    try:
        transact(port, read_request(addr, 4, 0, 1), MODBUS_REPLY_S, clock)
    except ModbusException as e:
        if e.code == 6:
            raise UpdateError("the menu is open on the panel (Modbus busy); close it and retry") from e
        log(f"warning: the board answered with {e} to a read; skipping the reboot request, power-cycle it")
        return False
    except ModbusError as e:
        log(f"warning: board not running our firmware (no Modbus reply: {e}); power-cycle it into the bootloader")
        return False
    try:
        transact(port, write_single(addr, HR_REBOOT, REBOOT_MAGIC), MODBUS_REPLY_S, clock)
    except ModbusException as e:
        if e.code == 6:  # 3.12: a reboot would lose the menu's unsaved edits
            raise UpdateError("the menu is open on the panel (Modbus busy); close it and retry") from e
        log(f"warning: the board refused the reboot request ({e}); power-cycle it")
        return False
    except ModbusError as e:
        log(f"warning: no ack to the reboot request ({e}); the board may or may not have reset")
        return False
    log("the board acked the reboot request; it resets into the bootloader now")
    return True


def frame_size(text):
    n = int(text)
    if n <= 0 or n % 8 or n > FRAME_BYTES:
        raise argparse.ArgumentTypeError(f"must be a positive multiple of 8 and at most {FRAME_BYTES}")
    return n


def main(argv=None, sleep=time.sleep):
    import serial  # pyserial; imported here so the tests don't need a port

    ap = argparse.ArgumentParser()
    ap.add_argument("image", help=".dat file (already encrypted, e.g. from oem_image.py)")
    where = ap.add_mutually_exclusive_group(required=True)
    where.add_argument("--port", help="serial device, or a pyserial URL")
    where.add_argument("--bridge", metavar="HOST",
                       help="install through the WiFi bridge (esphome/oem-bridge.yaml), switching its baud as needed")
    ap.add_argument("--wait", type=float, default=30.0, help="seconds to keep trying the handshake")
    ap.add_argument("--frame-bytes", type=frame_size, default=None,
                    help=f"data bytes per frame (default {FRAME_BYTES}, or {BRIDGE_FRAME_BYTES} with --bridge); "
                         "try 64 if the first frame fails with a small-FIFO adapter")
    ap.add_argument("--modbus-reboot", action="store_true",
                    help="first ask the running app over Modbus to reset into the bootloader")
    ap.add_argument("--modbus-baud", type=int, default=9600)
    ap.add_argument("--modbus-addr", type=int, default=1)
    a = ap.parse_args(argv)
    if a.bridge:
        a.bridge = resolve_bridge(a.bridge)
    name = bridge_url(a.bridge) if a.bridge else a.port
    if a.frame_bytes is None:
        a.frame_bytes = BRIDGE_FRAME_BYTES if a.bridge else FRAME_BYTES
    dat = open(a.image, "rb").read()
    try:
        if a.bridge:
            # inside the try, so an interrupt during the settle still turns Passthrough off
            set_bridge_passthrough(a.bridge, True)  # the bridge's Modbus poller stays off the line throughout
            sleep(PASSTHROUGH_SETTLE_S)
        rebooted = False
        try:
            check_image(dat)  # before anything resets the board
            if a.modbus_reboot:
                if a.bridge:
                    set_bridge_baud(a.bridge, a.modbus_baud)
                with open_port(serial, name, a.modbus_baud) as mb:
                    rebooted = modbus_reboot(mb, a.modbus_addr)
        except UpdateError as e:
            sys.exit(f"update failed: {e}")
        try:
            if a.bridge:
                set_bridge_baud(a.bridge, BAUD)  # quickly: the bootloader listens for 1.5 s after the reset
            with open_port(serial, name, BAUD) as port:
                try:
                    install(port, dat, wait_s=a.wait, frame_bytes=a.frame_bytes, rebooted=rebooted)
                except UpdateError as e:
                    sys.exit(f"update failed: {e}")
                except KeyboardInterrupt:
                    sys.exit("interrupted; " + POWER_CYCLE)
        finally:
            if a.bridge:
                set_bridge_baud(a.bridge, a.modbus_baud)  # leave the bridge on the Modbus line rate
    finally:
        if a.bridge:
            set_bridge_passthrough(a.bridge, False)


if __name__ == "__main__":
    main()
