# tools/test_fake_eg4.py
import queue
import struct

import pytest

from fake_eg4 import (ControlFile, State, apply_command, build_request, extract_eg4, parse_request, reply_for, serve)
from oem_image import crc16_modbus

REQ_A = bytes([0x10, 0x03, 0x00, 0x00, 0x00, 0x12, 0xC6, 0x86])  # tests/test_oem_eg4.cpp
REQ_B = bytes([0x10, 0x03, 0x00, 0x12, 0x00, 0x15, 0x27, 0x41])


def words(reply):
    return struct.unpack(f">{(len(reply) - 5) // 2}H", reply[3:-2])


def test_requests_match_the_firmware_vectors():
    assert build_request(0x0000, 0x12) == REQ_A
    assert build_request(0x0012, 0x15) == REQ_B


def test_parse_request():
    assert parse_request(REQ_A) == (0, 0x12) and parse_request(REQ_B) == (0x12, 0x15)
    assert parse_request(REQ_A[:7] + b"\x87") is None  # bad CRC
    other = bytearray(REQ_A)
    other[0] = 0x11  # another slave
    assert parse_request(bytes(other)) is None


def test_block_a_reply_is_valid_and_carries_the_values():
    r = reply_for(0, 0x12, State(voltage=53.2, max_cell=3.345, cells=16))
    assert len(r) == 41 and r[:3] == bytes([0x10, 3, 36])  # 5 + 2 * 18, as parse_reply wants
    assert struct.pack("<H", crc16_modbus(r[:-2])) == r[-2:]
    w = words(r)
    assert w[0] == 5320 and max(w[2:]) == 3345 and all(w[2:]) and w[1] == 0


def test_block_a_cell_count():
    w = words(reply_for(0, 0x12, State(cells=4)))
    assert sum(1 for c in w[2:] if c) == 4
    assert not any(words(reply_for(0, 0x12, State(cells=0)))[2:])


def test_block_b_reply_is_valid_and_carries_the_values():
    r = reply_for(0x12, 0x15, State(soc=60, ccl=25, protection=0x0593))
    assert len(r) == 47 and r[2] == 42
    assert struct.pack("<H", crc16_modbus(r[:-2])) == r[-2:]
    w = words(r)
    assert (w[0x16 - 0x12], w[0x18 - 0x12], w[0x1B - 0x12]) == (25, 60, 0x0593)


@pytest.mark.parametrize("first,count", [(0, 0x11), (0x12, 0x14), (1, 0x12), (0x1B, 1)])
def test_other_reads_get_no_reply(first, count):
    assert reply_for(first, count, State()) is None


def test_silent():
    assert reply_for(0, 0x12, State(silent=True)) is None


def test_extract_resyncs_on_noise():
    assert extract_eg4(bytearray(b"\x00\x01")) == (1, None)
    assert extract_eg4(bytearray(REQ_A[:5])) == (0, None)
    assert extract_eg4(bytearray(REQ_A)) == (8, (0, 0x12))
    assert extract_eg4(bytearray(b"\x10\x03\x00\x00\x00\x12\x00\x00")) == (1, None)


def test_apply_command():
    s = State()
    assert apply_command(s, "set soc 40") == "soc = 40"
    apply_command(s, "set protection 0x0593")
    apply_command(s, "set ccl 12")
    apply_command(s, "set cells 15")
    apply_command(s, "silent on")
    assert (s.soc, s.protection, s.ccl, s.cells, s.silent) == (40, 0x0593, 12, 15, True)
    assert apply_command(s, "bogus").startswith("error")
    assert apply_command(s, "set soc x").startswith("error")
    assert apply_command(s, "quit") is None
    assert "voltage" in apply_command(s, "show")


@pytest.mark.parametrize("cmd", ["set voltage nan", "set soc 101", "set cells 17", "set ccl -1", "set protection 0x10000"])
def test_apply_command_rejects_out_of_range(cmd):
    s = State()
    assert apply_command(s, cmd).startswith("error")
    assert s == State()


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


def test_serve_answers_both_blocks_and_ignores_noise():
    q, lines = queue.Queue(), []
    port = ScriptPort([b"\x00\xff" + REQ_A[:3], REQ_A[3:] + b"\x10\x03\x00\x01\x00\x01\x00\x00", REQ_B], q)
    serve(port, State(), commands=q, out=lines.append)
    assert [len(w) for w in port.written] == [41, 47]
    assert lines[0].startswith("A -> ") and lines[1].startswith("B -> ")


def test_serve_silent_and_quiet():
    q, lines = queue.Queue(), []
    port = ScriptPort([REQ_A * 2], q)
    serve(port, State(silent=True), commands=q, out=lines.append)
    assert port.written == []
    q, lines = queue.Queue(), []
    port = ScriptPort([REQ_A * 2], q)
    serve(port, State(), quiet=True, commands=q, out=lines.append)
    assert len(port.written) == 2 and len(lines) == 1


def test_command_changes_the_next_reply():
    q = queue.Queue()
    q.put("set soc 33")
    port = ScriptPort([REQ_B], q)
    serve(port, State(), commands=q, out=lambda m: None)
    assert words(port.written[0])[0x18 - 0x12] == 33


def test_control_file(tmp_path):
    path = tmp_path / "ctl"
    path.write_text("set soc 1\n")
    c = ControlFile(str(path))
    assert c.lines() == []
    with open(path, "a") as f:
        f.write("set soc 2\n")
    assert c.lines() == ["set soc 2"]


def test_main_bridge_passthrough_at_9600_and_off_in_finally(monkeypatch):
    import fake_eg4

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

    monkeypatch.setattr(fake_eg4, "serve", boom)
    with pytest.raises(RuntimeError):
        fake_eg4.main(["--bridge", "eg4.local", "--soc", "60"], sleep=lambda s: events.append(("sleep", s)))
    assert events == [("pass", "10.0.0.5", True), ("sleep", fake_eg4.PASSTHROUGH_SETTLE_S),
                      ("baud", "10.0.0.5", 9600), ("open", "socket://10.0.0.5:8899", 9600), ("serve", 60),
                      ("close",), ("pass", "10.0.0.5", False)]


@pytest.mark.parametrize("args", [["--soc", "101"], ["--soc", "-1"], ["--cells", "17"], ["--cells", "-1"],
                                  ["--voltage", "nan"], ["--max-cell", "70"], ["--ccl", "70000"],
                                  ["--protection", "0x10000"]])
def test_command_line_values_are_range_checked(args, capsys):
    from fake_eg4 import main
    with pytest.raises(SystemExit) as e:
        main(["--port", "x"] + args)
    assert e.value.code == 2
    assert "must be" in capsys.readouterr().err
