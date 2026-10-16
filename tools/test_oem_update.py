# tools/test_oem_update.py
import itertools
import struct

import pytest

from oem_image import MAX_LEN, crc16_modbus, encrypt
from oem_update import HANDSHAKE, HANDSHAKE_REPLY, UpdateError, data_frame, header_frame, install
from tea_decrypt import crypt, dec_block


class FakeBootloader:
    """Byte-level model of the stock bootloader (docs/history/notes/oem-hw-map.md §6.2): it acks the handshake, takes
    the first 4-byte frame as the header, decrypts and programs every CRC-good data frame at the running offset (no
    sequence check), and acks with the sequence number echoed and a Modbus CRC."""

    def __init__(self, silent=False, drop_ack_seq=None, echo=False, handshake_replies=None, bad_header_ack=False,
                 bad_seq_ack=None, bad_ack_crc=False):
        self.echo = echo
        self.handshake_replies = handshake_replies  # None = always answer; N = answer only the first N
        self.bad_header_ack = bad_header_ack
        self.bad_seq_ack = bad_seq_ack
        self.bad_ack_crc = bad_ack_crc
        self.erased = False  # the real bootloader erases only on the first header after a boot
        self.writes = []
        self.frame_lens = []
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
        self.writes.append(b)
        if self.silent:
            return
        if self.echo:
            self.out += b
        if b == HANDSHAKE:
            if self.handshake_replies is None or self.handshake_replies > 0:
                self.out += HANDSHAKE_REPLY
                if self.handshake_replies is not None:
                    self.handshake_replies -= 1
            return
        self.frame_lens.append(len(b))
        n = struct.unpack(">H", b[1:3])[0]
        assert b[0] == 0xFB and b[3:5] == b"\x03\x00" and len(b) == n + 9
        if crc16_modbus(b[:n + 7]) != struct.unpack("<H", b[n + 7:])[0]:
            return  # bad CRC: no reply
        seq = b[5:7]
        if self.length is None:
            assert n == 4
            self.length = struct.unpack(">I", b[7:11])[0]
            if not self.erased:
                self.flash[:] = b"\xff" * len(self.flash)  # erase on the first header since power-up
                self.erased = True
            if self.bad_header_ack:
                return
        else:
            self.flash[self.offset:self.offset + n] = crypt(b[7:7 + n], dec_block)
            self.offset += n
            if struct.unpack(">H", seq)[0] == self.drop_ack_seq:
                return
        if self.bad_seq_ack is not None and struct.unpack(">H", seq)[0] == self.bad_seq_ack:
            seq = b"\x7f\x7f"
        ack = b"\xfb\x03\x00" + seq
        ack += struct.pack("<H", crc16_modbus(ack))
        if self.bad_ack_crc:
            ack = ack[:5] + bytes([ack[5] ^ 0xFF, ack[6]])
        self.out += ack


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


QUIET = dict(log=lambda *_: None)


def test_echoing_adapter_is_compensated():
    plain = bytes((i * 5) & 0xFF for i in range(600)) + b"\x00" * 8
    fake = FakeBootloader(echo=True)
    clock, sleep = fake_time()
    install(fake, encrypt(plain), clock=clock, sleep=sleep, **QUIET)
    assert bytes(fake.flash[:len(plain)]) == plain


def test_abort_message_says_power_cycle_and_a_retry_does_not_erase():
    # The real bootloader erases only on the first header after a boot, so a retry without a power cycle would program
    # over a half-written app. That is why every post-header error tells the user to power-cycle.
    fake = FakeBootloader(drop_ack_seq=1)
    clock, sleep = fake_time()
    with pytest.raises(UpdateError, match="power-cycle"):
        install(fake, encrypt(b"\x11" * 512), clock=clock, sleep=sleep, **QUIET)
    assert fake.erased
    fake.flash[0x1000:0x1004] = b"\x00" * 4  # stale data from the aborted run
    fake.length = None
    fake.offset = 0
    fake.drop_ack_seq = None
    install(fake, encrypt(b"\x22" * 512), clock=clock, sleep=sleep, **QUIET)
    assert bytes(fake.flash[0x1000:0x1004]) == b"\x00" * 4  # still not erased


def test_header_ack_failure_says_power_cycle():
    fake = FakeBootloader(bad_header_ack=True)
    clock, sleep = fake_time()
    with pytest.raises(UpdateError, match="power-cycle"):
        install(fake, encrypt(b"\x11" * 64), clock=clock, sleep=sleep, **QUIET)


def test_wrong_sequence_ack_raises():
    fake = FakeBootloader(bad_seq_ack=1)
    clock, sleep = fake_time()
    with pytest.raises(UpdateError, match="power-cycle"):
        install(fake, encrypt(b"\x11" * 64), clock=clock, sleep=sleep, **QUIET)


def test_single_handshake_reply_is_not_a_bootloader():
    fake = FakeBootloader(handshake_replies=1)
    clock, sleep = fake_time()
    with pytest.raises(UpdateError, match="no bootloader handshake"):
        install(fake, encrypt(b"\x11" * 64), wait_s=2.0, clock=clock, sleep=sleep, **QUIET)


def test_oversize_image_is_rejected_before_any_write():
    fake = FakeBootloader()
    clock, sleep = fake_time()
    with pytest.raises(UpdateError):
        install(fake, b"\x00" * (MAX_LEN + 8), clock=clock, sleep=sleep, **QUIET)
    assert fake.writes == []


def test_small_frames():
    plain = bytes(range(256)) * 2
    fake = FakeBootloader()
    clock, sleep = fake_time()
    install(fake, encrypt(plain), frame_bytes=64, clock=clock, sleep=sleep, **QUIET)
    assert bytes(fake.flash[:len(plain)]) == plain
    assert fake.frame_lens == [13] + [73] * 8


def test_header_ack_bytes():
    # The bootloader's ack for the header (seq 0) is fixed; the CRC covers the first 5 bytes, low byte first.
    ack = b"\xfb\x03\x00\x00\x00"
    assert ack + struct.pack("<H", crc16_modbus(ack)) == bytes.fromhex("FB03000000C190")


def test_corrupted_header_ack_crc_aborts():
    fake = FakeBootloader(bad_ack_crc=True)
    clock, sleep = fake_time()
    with pytest.raises(UpdateError, match="power-cycle"):
        install(fake, encrypt(b"\x11" * 64), clock=clock, sleep=sleep, **QUIET)
    assert fake.offset == 0  # no data frame was sent


def test_corrupted_data_ack_crc_aborts_without_resending():
    fake = FakeBootloader()
    clock, sleep = fake_time()
    real_write = fake.write

    def write(b):
        if len(b) > 13:
            fake.bad_ack_crc = True  # corrupt the acks of data frames only
        real_write(b)

    fake.write = write
    with pytest.raises(UpdateError, match="power-cycle"):
        install(fake, encrypt(b"\x11" * 1024), clock=clock, sleep=sleep, **QUIET)
    assert fake.offset == 256  # frame 1 programmed once, never resent


def test_input_is_flushed_before_every_frame():
    fake = FakeBootloader()
    clock, sleep = fake_time()
    events = []
    real_reset, real_write = fake.reset_input_buffer, fake.write
    fake.reset_input_buffer = lambda: (events.append("flush"), real_reset())
    fake.write = lambda b: (events.append("write" if len(bytes(b)) > 5 else "hs"), real_write(b))
    install(fake, encrypt(b"\x11" * 600), clock=clock, sleep=sleep, **QUIET)
    frames = [i for i, e in enumerate(events) if e == "write"]
    assert len(frames) == 1 + 3
    assert all(events[i - 1] == "flush" for i in frames)


def test_rebooted_board_that_never_answers_gets_a_power_cycle_hint():
    from oem_update import handshake
    for rebooted in (True, False):
        logs = []
        ticks = iter(range(0, 1000))
        with pytest.raises(UpdateError):
            handshake(FakeBootloader(silent=True), 10, lambda: next(ticks) * 0.1, lambda s: None, logs.append,
                      rebooted)
        hints = [ln for ln in logs if "no bootloader reply; power-cycle the board now" in ln]
        assert len(hints) == (1 if rebooted else 0)


def test_bridge_switches_baud_around_the_install(monkeypatch, tmp_path):
    """--bridge: 9600 for the Modbus reboot, 460800 for the bootloader, and back to 9600 whatever happens."""
    import oem_update as ou
    events = []
    monkeypatch.setattr(ou, "set_bridge_baud", lambda host, baud: events.append(("baud", host, baud)))
    monkeypatch.setattr(ou, "resolve_bridge", lambda h: h)
    monkeypatch.setattr(ou, "set_bridge_passthrough", lambda host, on: events.append(("pass", host, on)))

    class Port:
        def __init__(self, name):
            self.name = name

        def __enter__(self):
            return self

        def __exit__(self, *a):
            events.append(("close", self.name))

    def fake_open(serial, name, baud):
        events.append(("open", name, baud))
        return Port(name)
    monkeypatch.setattr(ou, "open_port", fake_open)
    monkeypatch.setattr(ou, "modbus_reboot", lambda port, addr: events.append(("reboot",)) or True)

    def fail_install(port, dat, **kw):
        events.append(("install", kw["rebooted"]))
        assert kw["frame_bytes"] == ou.BRIDGE_FRAME_BYTES  # 73 bytes on the line: one ESP32 UART FIFO
        raise ou.UpdateError("boom")
    monkeypatch.setattr(ou, "install", fail_install)
    img = tmp_path / "a.dat"
    img.write_bytes(b"\x11" * 64)
    with pytest.raises(SystemExit):
        ou.main([str(img), "--bridge", "br", "--modbus-reboot"], sleep=lambda s: events.append(("sleep", s)))
    url = "socket://br:8899"
    assert events == [("pass", "br", True), ("sleep", 0.5), ("baud", "br", 9600), ("open", url, 9600), ("reboot",), ("close", url),
                      ("baud", "br", 460800), ("open", url, 460800), ("install", True), ("close", url),
                      ("baud", "br", 9600), ("pass", "br", False)]

def test_interrupt_during_the_settle_still_turns_passthrough_off(monkeypatch, tmp_path):
    import oem_update as ou
    events = []
    monkeypatch.setattr(ou, "resolve_bridge", lambda h: h)
    monkeypatch.setattr(ou, "set_bridge_passthrough", lambda host, on: events.append(("pass", on)))

    def interrupted(s):
        raise KeyboardInterrupt
    img = tmp_path / "a.dat"
    img.write_bytes(b"\x11" * 64)
    with pytest.raises(KeyboardInterrupt):
        ou.main([str(img), "--bridge", "br"], sleep=interrupted)
    assert events == [("pass", True), ("pass", False)]
