# tools/test_oem_modbus.py
import argparse

import oem_modbus as om
import struct

import pytest

from oem_modbus import (ModbusError, ModbusException, build_parser, decode_status, parse_assignment, parse_response,
                        read_request, run, transact, with_crc, write_multiple, write_single)
from oem_image import MAX_LEN
from oem_update import UpdateError, main as update_main, modbus_reboot


class FakePort:
    """Serial stand-in: each write is answered by `replies(request)`; echo=True first echoes the request back."""

    def __init__(self, replies=lambda req: b"", echo=False):
        self.replies, self.echo = replies, echo
        self.writes = []
        self.out = bytearray()

    def reset_input_buffer(self):
        self.out.clear()

    def write(self, b):
        b = bytes(b)
        self.writes.append(b)
        if self.echo:
            self.out += b
        self.out += self.replies(b)

    def read(self, n):
        r = bytes(self.out[:n])
        del self.out[:n]
        return r


def ir_block(**over):
    ir = [0] * 40
    ir[0:16] = [0x0300, 2, 0, 0, 0b00111001, 0, 5760, 100, 100, 95, 5300, 5700, 5690, 80, 5760, 100]
    ir[16:19] = [1, 0x0002, 0xFFFF]
    ir[19] = 0xFFFF
    ir[20:30] = [0b1, 5760, 48, 2300, 250, 0x7FFF, 3000, 0xFFFF, 0, 0]
    ir[30:40] = [0b11, 0, 0, 0, 0xFFF6, 0x7FFF, 0, 0, 0, 4]
    for k, v in over.items():
        ir[int(k[1:])] = v
    return ir


def read_reply(req, words):
    return with_crc(bytes([req[0], req[1], 2 * len(words)]) + struct.pack(f">{len(words)}H", *words))


def test_request_bytes():
    assert read_request(1, 3, 0, 10) == bytes.fromhex("01 03 00 00 00 0A C5 CD")
    assert read_request(1, 4, 0, 40)[:6] == bytes.fromhex("01 04 00 00 00 28")
    assert write_single(1, 120, 0xB007) == with_crc(bytes.fromhex("01 06 00 78 B0 07"))
    assert write_multiple(1, 1, [5700, 5700])[:-2] == bytes.fromhex("01 10 00 01 00 02 04 16 44 16 44")
    assert write_single(1, 107, -1)[4:6] == b"\xff\xff"


def test_parse_read_response():
    req = read_request(1, 3, 0, 2)
    assert parse_response(req, read_reply(req, [10, 5760])) == [10, 5760]


def test_parse_write_acks():
    req = write_single(1, 0, 20)
    assert parse_response(req, req) == []
    req = write_multiple(1, 1, [1, 2])
    assert parse_response(req, with_crc(req[:6])) == []


def test_parse_exception_reply():
    req = write_single(1, 1, 9999)
    with pytest.raises(ModbusException, match="exception 03: illegal value") as e:
        parse_response(req, with_crc(bytes([1, 0x86, 3])))
    assert e.value.code == 3
    assert "busy" in str(ModbusException(6))


def test_parse_bad_crc():
    req = read_request(1, 3, 0, 1)
    bad = bytearray(read_reply(req, [7]))
    bad[-1] ^= 0xFF
    with pytest.raises(ModbusError, match="bad CRC"):
        parse_response(req, bytes(bad))


def test_parse_wrong_length_or_address():
    req = read_request(1, 3, 0, 2)
    with pytest.raises(ModbusError):
        parse_response(req, read_reply(req, [1]))
    with pytest.raises(ModbusError, match="address"):
        parse_response(req, with_crc(bytes([2, 3, 4, 0, 1, 0, 2])))


def test_transact_no_reply():
    ticks = iter(range(0, 1000))
    with pytest.raises(ModbusError, match="no reply"):
        transact(FakePort(), read_request(1, 3, 0, 1), 0.5, clock=lambda: next(ticks) * 0.1)


def test_transact_strips_adapter_echo():
    port = FakePort(lambda req: read_reply(req, [5]), echo=True)
    assert transact(port, read_request(1, 3, 6, 1)) == [5]
    port = FakePort(lambda req: req, echo=True)  # write acked, with the request echoed too
    assert transact(port, write_single(1, 110, 1)) == []
    port = FakePort(lambda req: req)
    assert transact(port, write_single(1, 110, 1)) == []


def test_status_decoder():
    text = "\n".join(decode_status(ir_block()))
    assert "version      3.0" in text
    assert "2 CHARGING" in text
    assert "none" in text.split("hold reason")[1].splitlines()[0]
    assert "cooling, session active, relay closed, AC present" in text
    assert "57.60 V" in text and "10.0 A" in text
    assert "5300 W" in text
    assert "uptime       65538 s" in text
    assert "source age" not in text  # IR 18 is reserved (always 0)
    psu1 = [ln for ln in text.splitlines() if ln.startswith("PSU1")][0]
    assert "online" in psu1 and "57.60 V" in psu1 and "4.8 A" in psu1 and "AC 230.0 V" in psu1
    assert "inlet 25.0 C" in psu1 and "heatsink n/a" in psu1 and "fans 3000/n/a" in psu1
    psu2 = [ln for ln in text.splitlines() if ln.startswith("PSU2")][0]
    assert "inlet -1.0 C" in psu2 and "online, faulted" in psu2 and "alarm 00000004" in psu2


def test_status_names_and_na():
    text = "\n".join(decode_status(ir_block(i1=4, i2=7, i3=13, i6=0xFFFF, i13=0xFFFF, i12=0xFFFF)))
    assert "4 FAULT" in text and "7 waiting for start" in text and "E13 setpoint mismatch" in text
    assert "target       n/a" in text
    assert "SOC          n/a" in text and "5700" not in text.split("battery V")[1].splitlines()[0]
    assert "n/a ADC" in text
    assert "E11 module alarm" in "\n".join(decode_status(ir_block(i3=11)))
    assert "E17 battery reading implausible" in "\n".join(decode_status(ir_block(i3=17)))


def test_status_command_reads_ir_0_to_39():
    port = FakePort(lambda req: read_reply(req, ir_block()))
    out = []
    run(port, build_parser().parse_args(["--port", "x", "status"]), out=out.append)
    assert port.writes == [read_request(1, 4, 0, 40)]
    assert any(ln.startswith("state") for ln in out)


def test_status_watch_repeats():
    port = FakePort(lambda req: read_reply(req, ir_block()))
    sleeps = []

    def sleep(s):
        sleeps.append(s)
        if len(sleeps) == 2:
            raise KeyboardInterrupt

    with pytest.raises(KeyboardInterrupt):
        run(port, build_parser().parse_args(["--port", "x", "status", "--watch", "2"]), out=lambda s: None, sleep=sleep)
    assert sleeps == [2.0, 2.0] and len(port.writes) == 2


def test_set_groups_consecutive_runtime_registers():
    port = FakePort(lambda req: req if req[1] == 6 else with_crc(req[:6]))
    a = build_parser().parse_args(["--port", "x", "set", "0x6B=5", "110=1", "111=1", "113=1"])
    run(port, a, out=lambda s: None)
    assert port.writes == [write_single(1, 107, 5), write_multiple(1, 110, [1, 1]), write_single(1, 113, 1)]


DEFAULT_HR = [10, 5760, 20, 100, 4200, 5760, 0, 1, 0, 10, 0, 120, 3550, 3400, 0, 5400, 5]


def settings_port(hr=None):
    hr = list(DEFAULT_HR if hr is None else hr)
    return FakePort(lambda req: read_reply(req, hr[req[3]:req[3] + req[5]]) if req[1] == 3 else with_crc(req[:6]))


def test_set_settings_reads_then_writes_the_span_as_one_block():
    port = settings_port()
    run(port, build_parser().parse_args(["--port", "x", "set", "5=5700", "1=5700"]), out=lambda s: None)
    # Charge V alone below Stop V would be refused; as one block the firmware checks the result as a whole.
    assert port.writes == [read_request(1, 3, 0, 17), write_multiple(1, 1, [5700, 20, 100, 4200, 5700])]


def test_set_single_setting_is_one_fc16():
    port = settings_port()
    run(port, build_parser().parse_args(["--port", "x", "set", "0=20"]), out=lambda s: None)
    assert port.writes == [read_request(1, 3, 0, 17), write_multiple(1, 0, [20])]


def test_set_mixed_settings_and_runtime_keeps_per_run_writes():
    port = FakePort(lambda req: req if req[1] == 6 else with_crc(req[:6]))
    run(port, build_parser().parse_args(["--port", "x", "set", "0=20", "110=1"]), out=lambda s: None)
    assert port.writes == [write_single(1, 0, 20), write_single(1, 110, 1)]


def test_set_settings_refused_writes_nothing_else():
    def reply(req):
        return read_reply(req, DEFAULT_HR) if req[1] == 3 else with_crc(bytes([1, 0x90, 3]))

    port = FakePort(reply)
    out = []
    with pytest.raises(ModbusException):
        run(port, build_parser().parse_args(["--port", "x", "set", "1=5700"]), out=out.append)
    assert len(port.writes) == 2 and not out


def test_partial_set_reports_what_was_applied():
    def reply(req):
        return with_crc(bytes([1, 0x86, 3])) if req[3] == 113 else req if req[1] == 6 else with_crc(req[:6])

    port = FakePort(reply)
    out = []
    with pytest.raises(ModbusException):
        run(port, build_parser().parse_args(["--port", "x", "set", "110=1", "111=1", "113=1"]), out=out.append)
    assert out == ["already applied: HR 110, HR 111"]


def test_set_parse():
    assert parse_assignment("107=-1") == (107, 0xFFFF)
    with pytest.raises(argparse.ArgumentTypeError):
        parse_assignment("5")


def test_get_reads_holding():
    port = FakePort(lambda req: read_reply(req, [10, 5760]))
    out = []
    run(port, build_parser().parse_args(["--port", "x", "get", "1", "0"]), out=out.append)
    assert port.writes == [read_request(1, 3, 0, 2)]
    assert out[0].startswith("HR   0 = 10") and "Max current" in out[0]


@pytest.mark.parametrize("argv, reg, value", [
    (["pause", "on"], 110, 1), (["pause", "off"], 110, 0), (["charge-now"], 113, 1), (["clear-fault"], 111, 1),
    (["relay", "on"], 112, 1), (["relay", "off"], 112, 0), (["reboot-bootloader"], 120, 0xB007)])
def test_commands(argv, reg, value):
    port = FakePort(lambda req: req)
    run(port, build_parser().parse_args(["--port", "x", *argv]), out=lambda s: None)
    assert port.writes == [write_single(1, reg, value)]



def test_diag_writes_one_block_and_off():
    port = FakePort(lambda req: with_crc(req[:6]))
    run(port, build_parser().parse_args(["--port", "x", "diag", "53.2", "0", "psu1", "60"]), out=lambda s: None)
    assert port.writes == [write_multiple(1, 130, [60, 5320, 0, 1])]
    port = FakePort(lambda req: req)
    run(port, build_parser().parse_args(["--port", "x", "diag", "off"]), out=lambda s: None)
    assert port.writes == [write_single(1, 130, 0)]
    with pytest.raises(SystemExit):
        run(FakePort(lambda req: req), build_parser().parse_args(["--port", "x", "diag", "53", "1", "all", "5"]))


def test_status_shows_diag_flag():
    ir = [0] * 40
    ir[4] = 1 << 8
    assert any("diagnostic override" in ln for ln in decode_status(ir))

def alive_then_ack(req):
    return read_reply(req, [0x0300]) if req[1] == 4 else req


def test_transact_strips_echo_of_fc16():
    port = FakePort(lambda req: with_crc(req[:6]), echo=True)
    req = write_multiple(1, 1, [5700, 30])
    assert transact(port, req) == []


def test_modbus_reboot_probes_ir0_then_sends_hr120():
    port = FakePort(alive_then_ack)
    logs = []
    assert modbus_reboot(port, 1, log=logs.append)
    assert port.writes == [read_request(1, 4, 0, 1), write_single(1, 120, 0xB007)]
    assert port.writes[1][:6] == bytes.fromhex("01 06 00 78 B0 07")
    assert not any("warning" in ln for ln in logs)


def test_modbus_reboot_on_echoing_adapter_with_silent_board_sends_nothing_more():
    port = FakePort(echo=True)
    ticks = iter(range(0, 1000))
    logs = []
    assert not modbus_reboot(port, 1, log=logs.append, clock=lambda: next(ticks) * 0.1)
    assert port.writes == [read_request(1, 4, 0, 1)]
    assert "not running our firmware" in logs[0]


def test_modbus_reboot_silent_board_skips_hr120():
    ticks = iter(range(0, 1000))
    logs = []
    assert not modbus_reboot(FakePort(), 1, log=logs.append, clock=lambda: next(ticks) * 0.1)
    assert "power-cycle it into the bootloader" in logs[0]


def test_modbus_reboot_busy_menu_stops():
    port = FakePort(lambda req: with_crc(bytes([1, 0x84, 6])))
    with pytest.raises(UpdateError, match="menu is open"):
        modbus_reboot(port, 1, log=lambda s: None)
    assert len(port.writes) == 1


def test_modbus_reboot_honours_the_address_and_refusal():
    def reply(req):
        return read_reply(req, [1]) if req[1] == 4 else with_crc(bytes([3, 0x86, 1]))

    port = FakePort(reply)
    logs = []
    assert not modbus_reboot(port, 3, log=logs.append)
    assert port.writes[0][0] == 3 and port.writes[1][0] == 3 and "illegal function" in logs[0]


class FakeSerial:
    instances = []

    def __init__(self):
        self.writes = []
        FakeSerial.instances.append(self)

    def open(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *a):
        pass

    def reset_input_buffer(self):
        pass

    def read(self, n):
        return b""

    def write(self, b):
        self.writes.append(bytes(b))


@pytest.fixture
def fake_serial(monkeypatch):
    import serial
    FakeSerial.instances = []
    monkeypatch.setattr(serial, "Serial", FakeSerial)
    return FakeSerial


def test_oversize_image_with_modbus_reboot_sends_nothing(fake_serial, tmp_path):
    img = tmp_path / "big.dat"
    img.write_bytes(b"\x00" * (MAX_LEN + 8))
    with pytest.raises(SystemExit, match="update failed"):
        update_main([str(img), "--port", "x", "--modbus-reboot"])
    assert all(not p.writes for p in fake_serial.instances)


def test_handshake_message_depends_on_reboot():
    from oem_update import handshake
    for rebooted, text in ((True, "asked to reboot"), (False, "power-cycle the board now")):
        logs = []
        ticks = iter(range(0, 1000))
        with pytest.raises(UpdateError):
            handshake(FakePort(), 0.5, lambda: next(ticks) * 0.1, lambda s: None, logs.append, rebooted)
        assert text in logs[0]


def test_exit_codes(monkeypatch, capsys):
    import oem_modbus
    import serial
    monkeypatch.setattr(serial, "Serial", FakeSerial)
    FakeSerial.read = lambda self, n: b""
    with pytest.raises(SystemExit) as e:
        oem_modbus.main(["--port", "x", "--timeout", "0.05", "get", "0"])
    assert e.value.code == 2

    class Exc(FakeSerial):
        def write(self, b):
            self.out = bytearray(with_crc(bytes([1, 0x83, 2])))

        def read(self, n):
            r, self.out = bytes(self.out[:n]), self.out[n:]
            return r

    monkeypatch.setattr(serial, "Serial", Exc)
    with pytest.raises(SystemExit) as e:
        oem_modbus.main(["--port", "x", "get", "0"])
    assert "illegal address" in str(e.value.code)  # exit 1: sys.exit(message)

    class Bad(FakeSerial):
        def open(self):
            raise serial.SerialException("no such port")

    monkeypatch.setattr(serial, "Serial", Bad)
    with pytest.raises(SystemExit) as e:
        oem_modbus.main(["--port", "x", "get", "0"])
    assert e.value.code == 3
    assert "cannot open" in capsys.readouterr().err


# --- WiFi bridge (esphome/oem-bridge.yaml) ---

class _FakeSerialModule:
    """Records how open_port opened the port."""
    class SerialException(Exception):
        pass

    def __init__(self):
        self.opened = []

    def _port(self, kind, url=None):
        mod = self

        class P:
            def __init__(self):
                self.port = url
                self.baudrate = self.timeout = None
                self.rts = self.dtr = True

            def open(self):
                mod.opened.append((kind, self.port, self.baudrate, self.rts, self.dtr))
        return P()

    def Serial(self):
        return self._port("serial")

    def serial_for_url(self, url, do_not_open=False):
        assert do_not_open
        return self._port("url", url)


def test_open_port_device_and_url():
    fake = _FakeSerialModule()
    om.open_port(fake, "/dev/cu.usbserial-X", 9600)
    om.open_port(fake, "socket://bridge.local:8899", 9600)
    assert fake.opened == [("serial", "/dev/cu.usbserial-X", 9600, False, False),
                           ("url", "socket://bridge.local:8899", 9600, False, False)]


def test_bridge_port_url():
    assert om.bridge_url("oem-bridge.local") == "socket://oem-bridge.local:8899"
    a = om.build_parser().parse_args(["--bridge", "oem-bridge.local", "status"])
    assert om.port_name(a) == "socket://oem-bridge.local:8899"
    a = om.build_parser().parse_args(["--port", "/dev/x", "status"])
    assert om.port_name(a) == "/dev/x"
    with pytest.raises(SystemExit):
        om.build_parser().parse_args(["status"])  # one of --port / --bridge is required


def test_resolve_bridge_once_or_pass_through():
    assert om.resolve_bridge("oem-bridge.local", lookup=lambda h: "10.0.0.5") == "10.0.0.5"

    def fail(h):
        raise OSError("no such host")
    assert om.resolve_bridge("nowhere.local", lookup=fail) == "nowhere.local"


def test_set_bridge_baud_posts_to_select():
    calls = []
    om.set_bridge_baud("oem-bridge.local", 460800, post=lambda url: calls.append(url))
    # ESPHome 2026 addresses entities by name: "RS485 baud"
    assert calls == ["http://oem-bridge.local/select/RS485%20baud/set?option=460800"]


def test_set_bridge_passthrough_posts_to_switch():
    calls = []
    om.set_bridge_passthrough("10.0.0.5", True, post=calls.append)
    om.set_bridge_passthrough("10.0.0.5", False, post=calls.append)
    assert calls == ["http://10.0.0.5/switch/Passthrough/turn_on", "http://10.0.0.5/switch/Passthrough/turn_off"]


def test_auth_header_from_env():
    assert om.auth_header({}) == {}
    h = om.auth_header({"OEM_BRIDGE_USER": "u", "OEM_BRIDGE_PASSWORD": "p"})
    assert h == {"Authorization": "Basic dTpw"}  # base64("u:p")


def test_main_bridge_wraps_the_run_in_passthrough(monkeypatch):
    events = []
    monkeypatch.setattr(om, "resolve_bridge", lambda h: "10.0.0.5")
    monkeypatch.setattr(om, "set_bridge_passthrough", lambda host, on: events.append(("pass", host, on)))

    class Port:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            events.append(("close",))

    monkeypatch.setattr(om, "open_port", lambda serial, name, baud: events.append(("open", name)) or Port())
    monkeypatch.setattr(om, "run", lambda port, a: events.append(("run", a.cmd)))
    om.main(["--bridge", "oem-bridge.local", "status"], sleep=lambda s: events.append(("sleep", s)))
    assert events == [("pass", "10.0.0.5", True), ("sleep", om.PASSTHROUGH_SETTLE_S),
                      ("open", "socket://10.0.0.5:8899"), ("run", "status"), ("close",), ("pass", "10.0.0.5", False)]
    assert om.PASSTHROUGH_SETTLE_S == 0.5


def test_main_bridge_turns_passthrough_off_when_the_run_fails(monkeypatch):
    events = []
    monkeypatch.setattr(om, "resolve_bridge", lambda h: "10.0.0.5")
    monkeypatch.setattr(om, "set_bridge_passthrough", lambda host, on: events.append(("pass", on)))

    class Port:
        def __enter__(self):
            return self

        def __exit__(self, *a):
            events.append(("close",))

    def failing_run(port, a):
        raise om.ModbusError("no reply")
    monkeypatch.setattr(om, "open_port", lambda serial, name, baud: Port())
    monkeypatch.setattr(om, "run", failing_run)
    with pytest.raises(SystemExit) as e:
        om.main(["--bridge", "oem-bridge.local", "status"], sleep=lambda s: None)
    assert e.value.code == 2
    assert events == [("pass", True), ("close",), ("pass", False)]


def test_sources_know_the_polled_modes():
    assert om.SOURCES == ["Panel", "Pylontech", "Modbus", "EG4", "JK", "JK-PB"]


def test_modbus_reboot_refused_busy_by_312_stops():
    # 3.12 answers the IR read but refuses HR 120 with exception 06 while the menu is open.
    port = FakePort(lambda req: read_reply(req, [0x030C]) if req[1] == 4 else with_crc(bytes([1, 0x86, 6])))
    with pytest.raises(UpdateError, match="menu is open"):
        modbus_reboot(port, 1, log=lambda s: None)
    assert len(port.writes) == 2
