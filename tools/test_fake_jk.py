# tools/test_fake_jk.py
import queue
import re
from pathlib import Path

import pytest

from fake_jk import (State, apply_command, build_request, extract_jk, frame, parse_request, reply_for, serve)

CPP = (Path(__file__).resolve().parent.parent / "tests" / "test_oem_jk.cpp").read_text()
TV1 = bytes.fromhex("".join(re.findall(r'"([0-9A-F]+)"', re.search(r"TV1_HEX =(.*?);", CPP, re.S).group(1))))
TV1_CELLS = (3821, 3834, 3831, 3820, 3832, 3834, 3825, 3832, 3811, 3834, 3825, 3835, 3835, 3826)
RECORD_LEN = {0x85: 1, 0x86: 1, 0x9D: 1, 0xA9: 1, 0xAB: 1, 0xAC: 1, 0xAE: 1, 0xAF: 1, 0xB1: 1, 0xB3: 1, 0xB8: 1, 0xC0: 1,
              0x89: 4, 0xAA: 4, 0xB5: 4, 0xB6: 4, 0xB9: 4, 0xB4: 8, 0xB2: 10, 0xB7: 15, 0xBA: 24}


def parse(f):
    """Python mirror of oem::jk::parse_reply (jk_protocol.h): returns the values the board uses."""
    n = len(f)
    assert f[:2] == bytes([0x4E, 0x57])
    assert (f[2] << 8 | f[3]) + 2 == n  # length = total - 2
    assert sum(f[:-2]) & 0xFFFF == (f[-2] << 8 | f[-1])  # 16-bit sum
    assert f[8] == 0x06
    i, end, seen, d, cells8a = 11, n - 9, 0, {}, None
    while i < end:
        rid = f[i]
        if rid == 0x79:
            ln = 1 + f[i + 1]
            assert f[i + 1] % 3 == 0
            cells = [f[i + 2 + 3 * c + 1] << 8 | f[i + 2 + 3 * c + 2] for c in range(f[i + 1] // 3)]
            d["cells"], seen = cells, seen | 1
        else:
            ln = RECORD_LEN.get(rid, 2 if 0x80 <= rid <= 0xB0 else 0)
            assert ln, hex(rid)
            v = f[i + 1:i + 1 + ln]
            if rid == 0x83:
                d["v"], seen = (v[0] << 8 | v[1]) / 100, seen | 2
            elif rid == 0x85:
                d["soc"], seen = v[0], seen | 4
            elif rid == 0x8A:
                cells8a = v[0] << 8 | v[1]
            elif rid == 0x8B:
                d["alarms"], seen = v[0] << 8 | v[1], seen | 8
            elif rid == 0x8C:
                d["status"], seen = v[0] << 8 | v[1], seen | 16
        i += 1 + ln
    assert i == end  # every record consumed
    assert seen == 31 and cells8a == len(d["cells"])
    return d


def test_frame_from_tv1_values_is_tv1():
    s = State(voltage=53.59, soc=15, alarms=0, status=7, cell_mv=TV1_CELLS)
    assert frame(s) == TV1


def test_tv1_parses_with_the_mirror():
    d = parse(TV1)
    assert (d["v"], d["soc"], d["alarms"], d["status"]) == (53.59, 15, 0, 7) and len(d["cells"]) == 14


@pytest.mark.parametrize("cells", [14, 16])
def test_default_frame_round_trips(cells):
    f = frame(State(voltage=52.5, soc=80, max_cell=3.4, cells=cells, alarms=0x0102, status=5))
    d = parse(f)
    assert (d["v"], d["soc"], d["alarms"], d["status"]) == (52.5, 80, 0x0102, 5)
    assert len(d["cells"]) == cells and max(d["cells"]) == 3400
    assert len(f) == (285 if cells == 14 else 285 + 6)


def test_requests_match_the_firmware_vectors():
    r3, r2 = build_request(0x03), build_request(0x02)
    assert len(r3) == 21 and r3[19:] == bytes([0x01, 0x29]) and r2[19:] == bytes([0x01, 0x28])
    assert r3[:19] == bytes.fromhex("4E5700130000000006" "03" "000000000000" "6800" "00")


def test_parse_request_accepts_either_source_byte_only():
    assert parse_request(build_request(0x03)) == 3 and parse_request(build_request(0x02)) == 2
    assert parse_request(build_request(0x04)) is None
    bad = bytearray(build_request(0x03))
    bad[20] ^= 1
    assert parse_request(bytes(bad)) is None


def test_reply_for_and_silent():
    assert reply_for(3, State()) == frame(State())
    assert reply_for(2, State(silent=True)) is None


def test_extract_resyncs_on_noise():
    r = bytearray(build_request(3))
    assert extract_jk(bytearray(b"\x00")) == (1, None)
    assert extract_jk(bytearray(b"\x4e\x00")) == (1, None)
    assert extract_jk(r[:10]) == (0, None)
    assert extract_jk(r) == (21, 3)
    r[20] ^= 1
    assert extract_jk(r) == (1, None)


def test_apply_command():
    s = State(cell_mv=TV1_CELLS)
    assert apply_command(s, "set alarms 0x3736") == "alarms = 14134"
    apply_command(s, "set status 0")
    apply_command(s, "set cells 16")
    apply_command(s, "silent on")
    assert (s.alarms, s.status, s.cells, s.silent, s.cell_mv) == (0x3736, 0, 16, True, ())
    assert apply_command(s, "set cells 17").startswith("error")
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


def test_serve_answers_both_source_bytes_and_ignores_noise():
    q, lines = queue.Queue(), []
    r3, r2 = build_request(3), build_request(2)
    port = ScriptPort([b"\xff\x4e" + r3[:5], r3[5:] + b"junk", r2, build_request(9)], q)
    serve(port, State(), commands=q, out=lines.append)
    assert port.written == [frame(State())] * 2
    assert lines[0].startswith("SS=0x03") and lines[1].startswith("SS=0x02")


def test_serve_quiet_and_command():
    q, lines = queue.Queue(), []
    q.put("set soc 20")
    port = ScriptPort([build_request(3) * 2], q)
    serve(port, State(), quiet=True, commands=q, out=lines.append)
    assert len(port.written) == 2 and parse(port.written[0])["soc"] == 20 and len(lines) == 2  # command echo + one reply


def test_main_bridge_passthrough_at_115200_and_off_in_finally(monkeypatch):
    import fake_eg4
    import fake_jk

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
        events.append(("serve", s.soc))
        raise RuntimeError("stop")

    monkeypatch.setattr(fake_jk, "serve", boom)
    with pytest.raises(RuntimeError):
        fake_jk.main(["--bridge", "jk.local", "--soc", "60"], sleep=lambda s: events.append(("sleep", s)))
    assert events == [("pass", "10.0.0.5", True), ("sleep", fake_eg4.PASSTHROUGH_SETTLE_S),
                      ("baud", "10.0.0.5", 115200), ("open", "socket://10.0.0.5:8899", 115200), ("serve", 60),
                      ("close",), ("pass", "10.0.0.5", False)]


@pytest.mark.parametrize("args", [["--soc", "256"], ["--soc", "-1"], ["--voltage", "inf"], ["--max-cell", "70"],
                                  ["--cells", "15"], ["--alarms", "0x10000"], ["--status", "-1"]])
def test_command_line_values_are_range_checked(args, capsys):
    from fake_jk import main
    with pytest.raises(SystemExit) as e:
        main(["--port", "x"] + args)
    assert e.value.code == 2
    assert capsys.readouterr().err
