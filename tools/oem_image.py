#!/usr/bin/env python3
"""Build an install image (.dat) for the stock Eco-Worthy bootloader.

The bootloader installs at 0x08005000, refuses to jump if the image's last 8 bytes are all 0xFF, and decrypts with
8-round TEA (see tea_decrypt.py). We pad to a multiple of 8 and append an 8-byte trailer
('OEM', major, minor, 0, CRC16-LE of the padded body) so the last word is never erased-looking.

Usage:
  oem_image.py app.bin out.dat --version 0.1
  oem_image.py --stock-from-dump dumps/gd32_flash_20261012.bin out.dat   # re-encrypt the stock v1 app
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from tea_decrypt import crypt, enc_block  # noqa: E402

FLASH_BASE = 0x08000000
APP_BASE = 0x08005000
LEN_ADDR = 0x0801E000
MAX_LEN = LEN_ADDR - APP_BASE  # 0x19000


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def build_image(raw, major, minor):
    body = raw + b"\x00" * (-len(raw) % 8)
    img = body + b"OEM" + bytes([major, minor, 0]) + struct.pack("<H", crc16_modbus(body))
    if len(img) > MAX_LEN:
        raise ValueError(f"image is {len(img)} bytes; the bootloader takes at most {MAX_LEN}")
    return img


def stock_from_dump(dump):
    length = struct.unpack_from(">I", dump, LEN_ADDR - FLASH_BASE)[0]
    if not 8 < length <= MAX_LEN or length % 8:
        raise ValueError(f"implausible stock image length {length:#x}")
    img = dump[APP_BASE - FLASH_BASE:APP_BASE - FLASH_BASE + length]
    if img[-8:] == b"\xff" * 8:
        raise ValueError("stock image ends in erased flash")
    return img


def encrypt(img):
    return crypt(img, enc_block)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile", nargs="?")
    ap.add_argument("outfile")
    ap.add_argument("--version", default="0.0", help="major.minor, stored in the trailer")
    ap.add_argument("--stock-from-dump", metavar="DUMP", help="extract the stock app from a full flash dump instead")
    a = ap.parse_args()
    if a.stock_from_dump:
        img = stock_from_dump(open(a.stock_from_dump, "rb").read())
    else:
        if not a.infile:
            ap.error("infile is required unless --stock-from-dump is given")
        major, minor = (int(x) for x in a.version.split("."))
        img = build_image(open(a.infile, "rb").read(), major, minor)
    open(a.outfile, "wb").write(encrypt(img))
    print(f"wrote {a.outfile}: {len(img)} bytes ({len(img) * 100 // MAX_LEN}% of the app area)")


if __name__ == "__main__":
    main()
