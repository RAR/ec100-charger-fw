import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from fake_nasn import FakeNasn, in_window  # noqa: E402


def frame(*b):
    return bytes(b)


def test_read_voltage_setpoint():
    m = FakeNasn(1)
    rid, data = m.handle(0x108182FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0))
    assert rid == 0x1081827E
    assert data[:2] == bytes([0x01, 0x00])
    assert int.from_bytes(data[2:], "big") == round(54.0 * 1023)


def test_write_voltage_current_standby():
    m = FakeNasn(2)
    rid, data = m.handle(0x108281FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))
    assert rid == 0x1082817E and data == frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7)
    assert abs(m.vout_set - 57.0) < 0.001
    m.handle(0x108281FE, frame(0x02, 0x2D, 0, 0, 0, 0, 0x02, 0x00))
    assert abs(m.ilimit - 26.03) < 0.01
    m.handle(0x108281FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))
    assert m.standby == 1


def test_telemetry_follows_state():
    m = FakeNasn(1, battery_v=52.0)
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))  # standby
    _, d = m.handle(0x108182FE, frame(0x01, 0x75, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 52 * 1024
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 0
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 0, 0, 0, 0, 0))  # on, 54 V > 52 V battery, 5 A default
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 5 * 1024


def test_ignores_other_traffic():
    m = FakeNasn(1)
    assert m.handle(0x108282FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None  # addr 2
    assert m.handle(0x1081827E, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None  # a reply, not a request
    assert m.handle(0x108182FE, frame(0x01, 0x00)) is None                      # short
    m.muted = True
    assert m.handle(0x108182FE, frame(0x01, 0x00, 0, 0, 0, 0, 0, 0)) is None


def test_unknown_register_status_2():
    m = FakeNasn(1)
    _, d = m.handle(0x108182FE, frame(0x07, 0x77, 0, 0, 0, 0, 0, 0))
    assert d[0] >> 4 == 2 and (d[0] & 0x0F) == 0x07 and d[1] == 0x77


def test_fallback_resets_setpoints():
    m = FakeNasn(1)
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 1, 0, 0, 0, 0))
    m.reset_to_defaults()
    assert abs(m.vout_set - 54.0) < 0.001 and m.ilimit == 5.0 and m.standby == 0


def test_only_controller_requests_hold_off_fallback():
    from fake_nasn import is_controller_request
    assert is_controller_request(0x108182FE, bytes(8))      # read request to addr 1
    assert is_controller_request(0x108281FE, bytes(8))      # write request to addr 2
    assert not is_controller_request(0x1081827E, bytes(8))  # a module's reply
    assert not is_controller_request(0x1001117E, bytes(8))  # proto 0x20 module chatter
    assert not is_controller_request(0x108182FE, bytes(2))  # short frame


def test_vout_readback_022f_follows_write():
    m = FakeNasn(1)
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xE3, 0xC7))  # 57.0 V
    _, d = m.handle(0x108182FE, frame(0x02, 0x2F, 0, 0, 0, 0, 0, 0))
    assert d[0] >> 4 == 0 and d[:2] == bytes([0x02, 0x2F])
    assert int.from_bytes(d[2:], "big") == 0xE3C7


def test_output_voltage_0184_matches_0175():
    m = FakeNasn(1, battery_v=52.0)
    _, a = m.handle(0x108182FE, frame(0x01, 0x75, 0, 0, 0, 0, 0, 0))
    _, b = m.handle(0x108182FE, frame(0x01, 0x84, 0, 0, 0, 0, 0, 0))
    assert b[0] >> 4 == 0 and b[2:] == a[2:]


def test_heatsink_register():
    m = FakeNasn(1)
    m.heatsink_c = 50.0
    _, d = m.handle(0x108182FE, frame(0x02, 0x35, 0, 0, 0, 0, 0, 0))
    assert d[0] == 0x02 and int.from_bytes(d[2:], "big") == 50 * 1024


def test_running_below_battery_reads_battery_voltage():
    m = FakeNasn(1, battery_v=52.0)
    m.handle(0x108181FE, frame(0x01, 0x32, 0, 0, 0, 0, 0, 0))  # on
    m.handle(0x108181FE, frame(0x01, 0x00, 0, 0, 0, 0, 0xBF, 0xD0))  # 48.0 V (cooldown setpoint)
    _, d = m.handle(0x108182FE, frame(0x01, 0x84, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 52 * 1024
    _, d = m.handle(0x108182FE, frame(0x01, 0x82, 0, 0, 0, 0, 0, 0))
    assert int.from_bytes(d[2:], "big") == 0


def test_in_window():
    assert in_window(5, 0, None) and in_window(60, 60, 90) and not in_window(90, 60, 90) and not in_window(59, 60, None)


def test_alarm_address_must_be_emulated():
    import pytest
    from fake_nasn import parse_args
    assert parse_args(["--addrs", "1,2", "--alarm", "2"]).alarm == [2]
    with pytest.raises(SystemExit):
        parse_args(["--addrs", "1,2", "--alarm", "3"])  # a typo would make the injection a silent no-op
    with pytest.raises(SystemExit):
        parse_args(["--addrs", "1", "--mute", "2"])


def test_status_shows_heatsink():
    m = FakeNasn(1)
    m.heatsink_c = 50.0
    assert "heatsink=50.0 C" in m.status()


def test_injection_events():
    from fake_nasn import injection_events
    assert injection_events({"alarm": False, "heatsink": False}, {"alarm": False, "heatsink": False}) == []
    assert injection_events({"alarm": False, "heatsink": True}, {"alarm": True, "heatsink": False}) == [
        "alarm injection starts", "heatsink injection ends"]
