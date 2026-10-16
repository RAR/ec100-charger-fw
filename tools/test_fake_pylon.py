# tools/test_fake_pylon.py
import dataclasses

import pytest

from fake_pylon import (ControlFile, State, apply_command, checksum, frame, length_field, parse_request, reply_for,
                        serve, stdin_reader)

V61 = ("~200246008062D098012C4B000A000A64640D1101010D0201080BBE0BD701030BA501090BBE0BD701010BA501010BBE0BD701010BA50101"
       "E8C4\r")
V62_CLEAR = "~20024600800800000000FC22\r"
V62_ALARM = "~20024600800800000200FC20\r"
V63_ON = "~20024600D012D548ABE0012C03E8C0F995\r"
V63_OFF = "~20024600D012D098ABE0000003E840F9BA\r"
REQUESTS = {0x61: "~201246610000FDAA\r", 0x62: "~201246620000FDA9\r", 0x63: "~201246630000FDA8\r"}


def test_length_field():
    assert length_field(0) == "0000"
    assert length_field(8) == "8008"
    assert length_field(0x62) == "8062"


def test_checksum_of_a_stock_request():
    assert checksum("2012466100" "00") == "FDAA"


def test_frame_matches_the_62_vectors():
    assert frame("00" "00" "0000") == V62_CLEAR
    assert frame("00" "00" "0200") == V62_ALARM
    assert reply_for(0x62, State()) == V62_CLEAR
    assert reply_for(0x62, State(alarm=True)) == V62_ALARM


def test_reply_61_vector():
    assert reply_for(0x61, State(current=3.0)) == V61


def test_reply_63_vectors():
    assert reply_for(0x63, State()) == V63_ON
    assert reply_for(0x63, State(cvl=53.4, ccl=0.0, charge_enable=False)) == V63_OFF


def test_negative_current_is_twos_complement():
    assert reply_for(0x61, State(current=-3.0)).startswith("~200246008062D098FED4")


@pytest.mark.parametrize("cid2", [0x60, 0x61, 0x62, 0x63])
def test_parse_request_accepts_stock_requests_and_0x60(cid2):
    body = f"201246{cid2:02X}0000"
    assert parse_request("~" + body + checksum(body) + "\r") == cid2


def test_parse_request_stock_strings():
    for cid2, text in REQUESTS.items():
        assert parse_request(text) == cid2


def test_parse_request_rejects_bad_input():
    assert parse_request("~201246610000FDAB\r") is None  # bad checksum
    assert parse_request(V62_CLEAR) is None  # a reply, not a request
    assert parse_request("201246610000FDAA\r") is None  # no start byte
    assert parse_request("~201246610000FDAA") is None  # no terminator
    assert parse_request("~2012476100" "00FDAA\r") is None  # wrong CID1


def test_silent_gives_no_reply():
    assert reply_for(0x61, State(silent=True)) is None
    assert reply_for(0x63, State(silent=True)) is None


def test_unknown_cid2_gives_no_reply():
    assert reply_for(0x60, State()) is None


def test_apply_command():
    s = State()
    assert apply_command(s, "set ccl 2") == "ccl = 2.0"
    assert s.ccl == 2.0
    apply_command(s, "set cvl 53.8")
    apply_command(s, "set soc 90")
    apply_command(s, "enable off")
    apply_command(s, "alarm on")
    apply_command(s, "silent on")
    assert (s.cvl, s.soc, s.charge_enable, s.alarm, s.silent) == (53.8, 90, False, True, True)
    apply_command(s, "enable on")
    assert s.charge_enable
    assert apply_command(s, "set bogus 1").startswith("error")
    assert apply_command(s, "set soc abc").startswith("error")
    assert apply_command(s, "frobnicate").startswith("error")
    assert apply_command(s, "quit") is None
    assert "ccl" in apply_command(s, "show")


@pytest.mark.parametrize("cmd", ["set voltage nan", "set ccl inf", "set ccl -1", "set ccl 6553.6", "set soc 101",
                                 "set soc -1", "set current 327.68", "set current -327.69", "set voltage 65.536",
                                 "set max-cell -0.1", "set cvl 70", "set dvl 66", "set dcl 7000"])
def test_apply_command_rejects_out_of_range(cmd):
    s = State()
    before = dataclasses.asdict(s)
    assert apply_command(s, cmd).startswith("error")
    assert dataclasses.asdict(s) == before


def test_apply_command_accepts_the_range_ends():
    s = State()
    for cmd in ["set voltage 65.535", "set current -327.68", "set current 327.67", "set soc 0", "set soc 100",
                "set ccl 6553.5", "set ccl 0"]:
        assert not apply_command(s, cmd).startswith("error"), cmd
    assert reply_for(0x61, State(voltage=65.535, current=-327.68)).startswith("~200246008062FFFF8000")


def test_rejected_value_does_not_stop_serve():
    import queue
    q, lines = queue.Queue(), []
    q.put("set ccl nan")
    port = ScriptPort([REQUESTS[0x63].encode()], q)
    serve(port, State(), commands=q, out=lines.append)
    assert lines[0].startswith("error") and port.written == [V63_ON]


class ScriptPort:
    """Serial stand-in: read() hands out the scripted chunks, then asks for quit through the command queue."""

    def __init__(self, chunks, commands):
        self.chunks, self.commands, self.written = list(chunks), commands, []

    def read(self, n):
        if self.chunks:
            return self.chunks.pop(0)
        self.commands.put("quit")
        return b""

    def write(self, data):
        self.written.append(data.decode())


def test_serve_answers_requests_and_ignores_noise():
    import queue
    q, lines = queue.Queue(), []
    port = ScriptPort([b"\x00junk" + REQUESTS[0x62].encode()[:6], REQUESTS[0x62].encode()[6:] + b"~bad\r",
                       REQUESTS[0x63].encode()], q)
    serve(port, State(), commands=q, out=lines.append)
    assert port.written == [V62_CLEAR, V63_ON]
    assert lines[0].startswith("0x62 -> ") and lines[1].startswith("0x63 -> ")


def test_serve_silent_writes_nothing_and_quiet_suppresses_repeats():
    import queue
    q, lines = queue.Queue(), []
    port = ScriptPort([REQUESTS[0x62].encode() * 2], q)
    serve(port, State(silent=True), commands=q, out=lines.append)
    assert port.written == []
    q, lines = queue.Queue(), []
    port = ScriptPort([REQUESTS[0x62].encode() * 2], q)
    serve(port, State(), quiet=True, commands=q, out=lines.append)
    assert len(port.written) == 2 and len(lines) == 1


def test_stdin_eof_keeps_serving():
    import io
    import queue
    q, lines = queue.Queue(), []
    stdin_reader(q, io.StringIO("set ccl 2\n"))  # stdin ends here
    assert list(q.queue) == ["set ccl 2\n"]  # no quit queued
    port = ScriptPort([REQUESTS[0x63].encode(), b"", b"", REQUESTS[0x63].encode()], q)  # idle reads in between
    serve(port, State(), commands=q, out=lines.append)
    assert lines[0] == "ccl = 2.0"
    assert len(port.written) == 2 and port.written[0] == port.written[1] and "0014" in port.written[0]


class ControlPort(ScriptPort):
    """A port whose reads, one per tick of the fake clock, append to the control file between requests."""

    def __init__(self, chunks, commands, path, appends, clock):
        super().__init__(chunks, commands)
        self.path, self.appends, self.clock = path, dict(appends), clock

    def read(self, n):
        self.clock.t += 0.05
        self.clock.reads += 1
        if self.clock.reads in self.appends:
            with open(self.path, "a") as f:
                f.write(self.appends[self.clock.reads])
        return super().read(n)


class Clock:
    def __init__(self):
        self.t, self.reads = 0.0, 0

    def __call__(self):
        return self.t


def test_control_file_commands_are_applied(tmp_path):
    import queue
    path = tmp_path / "ctl"
    path.write_text("enable off\nset ccl 9\n")  # already there: ignored
    control = ControlFile(str(path))
    clock, q, lines = Clock(), queue.Queue(), []
    idle = [b""] * 4
    # read 2 appends a command and half of another; read 6 finishes it. Requests at reads 1, 6 and 11.
    port = ControlPort([REQUESTS[0x63].encode()] + idle + [REQUESTS[0x63].encode()] + idle + [REQUESTS[0x63].encode()],
                       q, str(path), {2: "set ccl 1.5\nenab", 6: "le off\n\n"}, clock)
    serve(port, State(), commands=q, out=lines.append, clock=clock, control=control)
    assert port.written[0] == V63_ON
    assert port.written[1] == reply_for(0x63, State(ccl=1.5))
    assert port.written[2] == reply_for(0x63, State(ccl=1.5, charge_enable=False))
    assert [x for x in lines if not x.startswith("0x")] == ["ccl = 1.5", "enable off"]


def test_control_file_is_created_and_quit_works(tmp_path):
    import queue
    path = tmp_path / "new"
    control = ControlFile(str(path))
    assert path.exists()
    path.write_text("quit\n")
    lines = []

    class Idle:
        def read(self, n):
            return b""

    serve(Idle(), State(), commands=queue.Queue(), out=lines.append, clock=Clock(), control=control)
    assert lines == []
