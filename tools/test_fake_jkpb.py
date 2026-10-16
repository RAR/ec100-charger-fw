# tools/test_fake_jkpb.py
import math
import queue
import struct
from pathlib import Path

import pytest

from fake_jkpb import (State, apply_command, build_request, cell_info_record, extract_jkpb, reply_for, serve,
                       settings_record, trailer)
from oem_image import crc16_modbus

CAPTURE_FILE = Path(__file__).resolve().parent.parent / "docs" / "history" / "notes" / "jk-pb-capture.txt"
CAPTURE = [line.split("<<<")[1].split() for line in CAPTURE_FILE.read_text().splitlines() if line.startswith("slave")]
CAP_CELL, CAP_CELL_TRAILER, CAP_SET, CAP_SET_TRAILER = (bytes.fromhex("".join(x)) for x in CAPTURE)
# the state that reproduces the captured slave 01 records
CAP_STATE = State(voltage=52.287, soc=64, cells=16, alarms=0x80000, stage=0,
                  cell_mv=(3268,) * 7 + (3267,) + (3268,) * 8)


def u16(b, o):
    return struct.unpack_from("<H", b, o)[0]


def u32(b, o):
    return struct.unpack_from("<I", b, o)[0]


def record_ok(r, type_):
    return r[:4] == bytes.fromhex("55AAEB90") and r[4] == type_ and len(r) == 300 and r[299] == sum(r[:299]) % 256


def decode_cell_info(r):
    """Python mirror of oem::jkpb::decode_cell_info."""
    cells = [u16(r, 6 + 2 * n) for n in range(32)]
    best = max([c for c in cells if 0 < c < 5000], default=0)
    return {"pack": u32(r, 150) / 1000, "soc": r[173] if r[173] <= 100 else -1,
            "max": best / 1000 if best else math.nan, "alarms": u32(r, 166), "mos": r[198] == 1, "stage": r[280]}


def decode_settings(r):
    """Python mirror of oem::jkpb::decode_settings (None when invalid)."""
    cells, rcv, rfv = u32(r, 114), u32(r, 38), u32(r, 42)
    if not 1 <= cells <= 32 or not (2500 <= rcv <= 4500 and 2500 <= rfv <= 4500):
        return None
    return {"cells": cells, "rcv": rcv, "rfv": rfv, "coc": u32(r, 50), "switch": u32(r, 118) == 1}


def test_unpatched_templates_reproduce_the_capture():
    assert cell_info_record(CAP_STATE) == CAP_CELL
    assert trailer(1, 0x1620) == CAP_CELL_TRAILER
    assert settings_record(CAP_STATE) == CAP_SET
    assert trailer(1, 0x161E) == CAP_SET_TRAILER


def test_requests_match_the_firmware_vectors():
    assert build_request(1, 0x1620) == bytes.fromhex("01101620000102" "0000D6F1")
    assert build_request(0, 0x1620) == bytes.fromhex("00101620000102" "0000DB61")
    assert build_request(1, 0x161E) == bytes.fromhex("0110161E000102" "0000D22F")
    assert build_request(0, 0x161E) == bytes.fromhex("00101" "61E000102" "0000DFBF")


def test_default_state_records_are_valid_and_decode():
    s = State()
    c, g = cell_info_record(s), settings_record(s)
    assert record_ok(c, 2) and record_ok(g, 1)
    d = decode_cell_info(c)
    assert d["mos"] and d["alarms"] == 0 and d["stage"] == 0 and d["max"] == pytest.approx(s.max_cell)
    assert decode_settings(g) == {"cells": 16, "rcv": 3600, "rfv": 3500, "coc": 40000, "switch": True}


def test_patched_fields_decode_like_the_firmware():
    s = State(voltage=57.6, soc=88, max_cell=3.612, cells=14, alarms=0x00000010, charge_mos=0, stage=2, rcv=3.55,
              rfv=3.45, coc=25.5, switch=0)
    c, g = cell_info_record(s), settings_record(s)
    assert record_ok(c, 2) and record_ok(g, 1)
    d = decode_cell_info(c)
    assert (d["pack"], d["soc"], d["max"], d["alarms"], d["mos"], d["stage"]) == (57.6, 88, 3.612, 0x10, False, 2)
    assert sum(1 for n in range(32) if u16(c, 6 + 2 * n)) == 14
    assert decode_settings(g) == {"cells": 14, "rcv": 3550, "rfv": 3450, "coc": 25500, "switch": False}


def test_soc_out_of_range_and_no_cells():
    d = decode_cell_info(cell_info_record(State(soc=101, cells=0)))
    assert d["soc"] == -1 and math.isnan(d["max"])
    assert decode_settings(settings_record(State(cells=0))) is None
    assert decode_settings(settings_record(State(rcv=2.4))) is None


def test_trailer_has_polled_address_register_and_good_crc():
    for addr, reg in ((1, 0x1620), (7, 0x161E), (15, 0x1620)):
        t = trailer(addr, reg)
        assert t[:6] == bytes([addr, 0x10, reg >> 8, reg & 255, 0, 1]) and len(t) == 8
        assert struct.pack("<H", crc16_modbus(t[:6])) == t[6:]


def test_reply_is_record_plus_matching_trailer():
    s = State(address=3)
    assert reply_for(3, 0x1620, s) == cell_info_record(s) + trailer(3, 0x1620)
    assert reply_for(3, 0x161E, s) == settings_record(s) + trailer(3, 0x161E)


def test_reply_ignores_other_address_register_and_silent():
    s = State()
    assert reply_for(2, 0x1620, s) is None and reply_for(0, 0x1620, s) is None
    assert reply_for(1, 0x1621, s) is None
    assert reply_for(1, 0x1620, State(silent=True)) is None


def test_extract_resyncs_on_noise_and_bad_crc():
    r = bytearray(build_request(1, 0x1620))
    assert extract_jkpb(bytearray(b"\x01")) == (0, None)  # could be a request start
    assert extract_jkpb(bytearray(b"\x01\x03")) == (1, None)
    assert extract_jkpb(r[:10]) == (0, None)
    assert extract_jkpb(r) == (11, (1, 0x1620))
    r[10] ^= 1
    assert extract_jkpb(r) == (1, None)
    assert extract_jkpb(bytearray(build_request(0, 0x161E))) == (11, (0, 0x161E))


def test_apply_command():
    s = State()
    assert apply_command(s, "set alarms 0x10") == "alarms = 16"
    apply_command(s, "set charge-mos 0")
    apply_command(s, "set stage 2")
    apply_command(s, "set cells 14")
    apply_command(s, "set coc 20")
    apply_command(s, "set switch 0")
    apply_command(s, "silent on")
    assert (s.alarms, s.charge_mos, s.stage, s.cells, s.coc, s.switch, s.silent) == (16, 0, 2, 14, 20, 0, True)
    assert s.cell_mv == ()
    assert apply_command(s, "set cells 33").startswith("error")
    assert apply_command(s, "set soc 256").startswith("error")
    assert apply_command(s, "set voltage nan").startswith("error")
    assert apply_command(s, "nope").startswith("error")
    assert apply_command(s, "quit") is None
    assert "alarms" in apply_command(s, "show")


class ScriptPort:
    def __init__(self, chunks, commands):
        self.chunks, self.commands, self.written = list(chunks), commands, []

    def read(self, n):
        if self.chunks:
            return self.chunks.pop(0)
        self.commands.put("quit")
        return b""

    def write(self, data):
        self.written.append(bytes(data))


def test_serve_answers_only_good_requests_to_its_address():
    q, lines = queue.Queue(), []
    good, other = build_request(1, 0x1620), build_request(2, 0x1620)
    bad = bytearray(build_request(1, 0x161E))
    bad[9] ^= 1
    settings = build_request(1, 0x161E)
    port = ScriptPort([b"\xff" + good[:5], good[5:] + b"junk", other, bytes(bad), settings], q)
    serve(port, State(), commands=q, out=lines.append)
    assert port.written == [cell_info_record(State()) + trailer(1, 0x1620),
                            settings_record(State()) + trailer(1, 0x161E)]
    assert lines[0].startswith("cell-info") and lines[1].startswith("settings")


def test_serve_quiet_and_command():
    q, lines = queue.Queue(), []
    q.put("set soc 20")
    port = ScriptPort([build_request(1, 0x1620) * 2], q)
    serve(port, State(), quiet=True, commands=q, out=lines.append)
    assert len(port.written) == 2 and port.written[0][173] == 20 and len(lines) == 2


def test_main_bridge_passthrough_at_115200_and_off_in_finally(monkeypatch):
    import fake_eg4
    import fake_jkpb

    events = []
    monkeypatch.setattr(fake_eg4, "resolve_bridge", lambda h: "10.0.0.5")
    monkeypatch.setattr(fake_eg4, "set_bridge_passthrough", lambda host, on: events.append(("pass", host, on)))
    monkeypatch.setattr(fake_eg4, "set_bridge_baud", lambda host, baud: events.append(("baud", host, baud)))
    monkeypatch.setattr(fake_eg4, "stdin_reader", lambda q: None)

    class Port:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            events.append(("close",))

    monkeypatch.setattr(fake_eg4, "open_port", lambda serial, name, baud: events.append(("open", name, baud)) or Port())

    def boom(port, s, quiet, commands, control=None):
        events.append(("serve", s.soc, s.address))
        raise RuntimeError("stop")

    monkeypatch.setattr(fake_jkpb, "serve", boom)
    with pytest.raises(RuntimeError):
        fake_jkpb.main(["--bridge", "jkpb.local", "--soc", "60", "--address", "2"],
                       sleep=lambda s: events.append(("sleep", s)))
    assert events == [("pass", "10.0.0.5", True), ("sleep", fake_eg4.PASSTHROUGH_SETTLE_S),
                      ("baud", "10.0.0.5", 115200), ("open", "socket://10.0.0.5:8899", 115200), ("serve", 60, 2),
                      ("close",), ("pass", "10.0.0.5", False)]


@pytest.mark.parametrize("args", [["--soc", "256"], ["--soc", "-1"], ["--voltage", "inf"], ["--max-cell", "6"],
                                  ["--cells", "33"], ["--alarms", "0x100000000"], ["--address", "16"],
                                  ["--stage", "256"], ["--rcv", "-1"], ["--coc", "nan"]])
def test_command_line_values_are_range_checked(args, capsys):
    from fake_jkpb import main
    with pytest.raises(SystemExit) as e:
        main(["--port", "x"] + args)
    assert e.value.code == 2
    assert capsys.readouterr().err


@pytest.mark.parametrize("max_cell", [0, 0.001])
def test_tiny_max_cell_builds_a_valid_record(max_cell):
    c = cell_info_record(State(max_cell=max_cell))
    assert record_ok(c, 2)
    assert math.isnan(decode_cell_info(c)["max"]) or decode_cell_info(c)["max"] <= 0.001
