#!/usr/bin/env python3
"""Decrypt (or encrypt) Eco-Worthy charger firmware update data.

The bootloader (FUN_080002cc / FUN_08000280) decrypts the update in 8-byte blocks with
TEA reduced to 8 rounds. It uses the standard delta 0x9E3779B9 and the key ASCII "1234567123456789"
(copied to RAM 0x20000004 from flash 0x0800368C at startup). Trailing bytes that don't
fill a whole 8-byte block are left untouched, just as the bootloader leaves them.

Usage:
  tea_decrypt.py in.bin out.bin [--offset N] [--encrypt]
  tea_decrypt.py in.bin out.bin --find dumps/gd32_flash_20261012.bin   # find where the plaintext lands
"""
import argparse
import struct

KEY = struct.unpack("<4I", b"1234567123456789")
DELTA = 0x9E3779B9
ROUNDS = 8
M = 0xFFFFFFFF


def dec_block(v0, v1, k=KEY):
    s = (DELTA * ROUNDS) & M
    for _ in range(ROUNDS):
        v1 = (v1 - ((((v0 << 4) + k[2]) & M) ^ ((v0 + s) & M) ^ (((v0 >> 5) + k[3]) & M))) & M
        v0 = (v0 - ((((v1 << 4) + k[0]) & M) ^ ((v1 + s) & M) ^ (((v1 >> 5) + k[1]) & M))) & M
        s = (s - DELTA) & M
    return v0, v1


def enc_block(v0, v1, k=KEY):
    s = 0
    for _ in range(ROUNDS):
        s = (s + DELTA) & M
        v0 = (v0 + ((((v1 << 4) + k[0]) & M) ^ ((v1 + s) & M) ^ (((v1 >> 5) + k[1]) & M))) & M
        v1 = (v1 + ((((v0 << 4) + k[2]) & M) ^ ((v0 + s) & M) ^ (((v0 >> 5) + k[3]) & M))) & M
    return v0, v1


def crypt(buf, fn):
    out = bytearray(buf)
    for i in range(0, len(buf) - len(buf) % 8, 8):
        v0, v1 = struct.unpack_from("<2I", buf, i)
        struct.pack_into("<2I", out, i, *fn(v0, v1))
    return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("infile")
    ap.add_argument("outfile")
    ap.add_argument("--offset", type=lambda x: int(x, 0), default=0, help="skip N header bytes before decrypting")
    ap.add_argument("--encrypt", action="store_true")
    ap.add_argument("--find", help="flash dump to search for decrypted blocks (sanity check)")
    a = ap.parse_args()

    raw = open(a.infile, "rb").read()
    body = crypt(raw[a.offset:], enc_block if a.encrypt else dec_block)
    open(a.outfile, "wb").write(raw[:a.offset] + body)
    print(f"wrote {a.outfile} ({len(raw)} bytes, {'encrypted' if a.encrypt else 'decrypted'} from offset {a.offset:#x})")

    if a.find:
        dump = open(a.find, "rb").read()
        # Try every header offset 0..255 and report where 8-byte decrypted blocks match the dump.
        for off in range(0, 256):
            pt = crypt(raw[off:off + 4096], dec_block)
            hits = [dump.find(pt[i:i + 32]) for i in range(0, len(pt) - 32, 512) if pt[i:i + 32] != b"\xff" * 32]
            good = [h for h in hits if h >= 0]
            if good:
                print(f"  header offset {off:#x}: {len(good)}/{len(hits)} probe blocks found in dump, first at flash {0x08000000 + good[0]:#010x}")


if __name__ == "__main__":
    main()
