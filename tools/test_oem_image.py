# tools/test_oem_image.py
import os
import struct

import pytest

from oem_image import MAX_LEN, build_image, crc16_modbus, encrypt, stock_from_dump
from tea_decrypt import crypt, dec_block

DUMP = os.path.join(os.path.dirname(__file__), "..", "dumps", "gd32_flash_20261012.bin")


def test_crc16_modbus_check_value():
    assert crc16_modbus(b"123456789") == 0x4B37


def test_build_pads_and_appends_trailer():
    img = build_image(b"\x01" * 13, 0, 1)
    assert len(img) % 8 == 0
    assert img[:13] == b"\x01" * 13 and img[13:16] == b"\x00" * 3
    assert img[-8:-5] == b"OEM" and img[-5] == 0 and img[-4] == 1 and img[-3] == 0
    assert struct.unpack("<H", img[-2:])[0] == crc16_modbus(img[:-8])
    assert img[-8:] != b"\xff" * 8


def test_build_rejects_oversize():
    with pytest.raises(ValueError):
        build_image(b"\x00" * MAX_LEN, 0, 1)


def test_encrypt_round_trips():
    img = build_image(bytes(range(256)) * 3, 1, 2)
    assert crypt(encrypt(img), dec_block) == img


def test_stock_from_synthetic_dump():
    dump = bytearray(b"\xff" * 0x20000)
    app = bytes(range(64))
    dump[0x5000:0x5000 + len(app)] = app
    dump[0x1E000:0x1E008] = struct.pack(">I", len(app)) + b"\x00" * 4
    assert stock_from_dump(bytes(dump)) == app


@pytest.mark.skipif(not os.path.exists(DUMP), reason="flash dump not present")
def test_stock_from_real_dump():
    img = stock_from_dump(open(DUMP, "rb").read())
    assert len(img) == 48984                                   # 0x0801E000 holds 00 00 BF 58
    assert struct.unpack_from("<I", img, 0)[0] == 0x20001448   # v1 app's initial SP
