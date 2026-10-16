# Stock Pylontech RS485 master (EC100 v1.01.1105, cross-checked against v2.0.1.1)

This note covers how the stock app builds the BMS requests on USART1 (9600 8N1) and how it validates and parses the replies.
The goal is a compatible reimplementation, both as a master and as a fake BMS that stock would accept.

Tags: **[V]** = verified in code at the address given. **[I]** = inferred. Line numbers are `re/decomp_app.c` (v1)
unless marked v2 (`re/v2/decomp_app.c`). The v1 addresses were also checked in an objdump of
`dumps/gd32_flash_20261012.bin`. RAM addresses are v1 unless stated. `re/v1/` holds only `ram_init.bin` (v1 `.data`,
base 0x20000000). `re/v2/` holds the v2 decompile, the function list and `ram_init.bin`.

Builds on `findings-v2.md` §3 (addresses, the 0x60→0x63 cycle, the 0xAA68 mask, link loss) and `oem-usart-sequences.md`
§2–3 (DMA RX on IDLE, polled TX, DE timing). Nothing here contradicts them. Two points are made more precise: what "byte 10"
and "INFO bytes 8–9" refer to (§3), and the 0x61 "cycles" field (§3.2).

**Offset convention.** "Decoded byte n" (`d[n]`) is an index into the hex-decoded frame buffer. It starts at VER, so
`d[0]` = VER, `d[1]` = ADR, `d[2]` = CID1, `d[3]` = CID2/RTN, `d[4..5]` = LENGTH, and `d[6]` = INFO byte 0.
"INFO n" = `d[6+n]`. "ASCII position p" is a character index in the raw received string, where p = 0 is SOI `~`.
`findings-v2.md` uses decoded-byte offsets: its "SOC byte 10" is INFO 4, and its "INFO bytes 8–9" for 0x62 are INFO 2–3.

---

## 1. Requests

### 1.1 Header and build **[V]** (0800982c, l.3839; disasm 0800982c–080098ca)

- **Header storage.** Six bytes at 0x20000009, initialised from `.data`: `20 12 46 00 00 00`, meaning VER 0x20, ADR 0x12,
  CID1 0x46, CID2 slot, then two LENGTH bytes = 0x0000. **[V]** (`re/v1/ram_init.bin` +0x09; v2 has the same bytes at
  0x20000362)
- **Current command.** The current CID2 is at 0x20000007 (`.data` = 0x60). At each send it is copied into the CID2 slot
  (`DAT_080098d0[3] = *DAT_080098cc`). **[V]**
- **Build steps** **[V]**:
  1. `n = sprintf(hex, "%02X%02X%02X%02X%02X%02X", VER, ADR, CID1, CID2, 0x00, 0x00)`. The format string is at 0x08010EFF.
     This gives 12 upper-case chars. LENGTH is printed as two raw zero bytes, so **LCHKSUM is never computed on TX.** It
     is 0 only because LENID is 0.
  2. `cs = FUN_0800613c(hex, n)`. This returns `(0x10000 − Σ ASCII) & 0xFFFF`, or 0 if n < 3 (l.1255). The result is
     stored with `strh` and reloaded with `ldrh`, so it is zero-extended.
  3. `sprintf(out, "~%s%04X\r", hex, cs)`. The format string is at 0x08010EF6.
  4. DE (PB10) goes high, `0800839c(1)` waits up to 1 ms, the frame is sent with `printf` through polled USART1, then DE goes
     low. See `oem-usart-sequences.md` §3.2. In v2 the DE pin is PB2 (`FUN_08015b00(GPIOB, 4)` in 080117c8). **[V]**
- **No INFO field.** Requests never carry INFO (no "command value" byte as in the LV protocol). **[V]**

### 1.2 The four request strings **[V]**

These were computed with the same algorithm. The 0x60 string matches the one captured on the bench
(`oem-bringup.md` l.9).

| CID2 | frame (EOI = CR 0x0D) |
|---|---|
| 0x60 | `~201246600000FDAB\r` |
| 0x61 | `~201246610000FDAA\r` |
| 0x62 | `~201246620000FDA9\r` |
| 0x63 | `~201246630000FDA8\r` |

### 1.3 Order and timing **[V]** (08006bb0 l.1728; dispatch 080097cc disasm 080097e8–08009822)

- **Period.** The TX fires when `state+0x36 > 500`. The counter is cleared at each send. The 1 ms ISR's 10 ms block adds
  10 to `state+0x36` (l.5953), so the period is about **510 ms**. The poll runs from main-loop task 0800b630 (l.4847), and
  only while byte 0x2000007C == 0. **[V]** What that flag means is not established. **[I]**
- **How the sequence advances.** It moves on **only on a valid reply.** The dispatch table, keyed on the CID2 that was
  *sent*:
  - 0x60: no parse, next = 0x61.
  - 0x61: run 08007a54, next = 0x62.
  - 0x62: run 080077d0, next = 0x63.
  - 0x63: run 08007bc8, next = 0x60.

  With no valid reply, the same CID2 is resent every ~510 ms. That explains why an idle bus shows only 0x60. A fake BMS
  must answer 0x60 validly before stock will ever ask 0x61. **[V]** The same holds in v2 (08011768, table 0x08011784, disasm
  identical in shape). **[V]**
- **Reply window.** There is no timeout of its own. Whatever frame is received before the next main-loop pass is
  attributed to the current CID2. **[V]** A late reply would be parsed as the answer to whatever is current then. **[I]**

## 2. Reply handling

### 2.1 Reception **[V]** (see `oem-usart-sequences.md` §2)

- DMA writes into a 512-byte buffer. An IDLE interrupt copies the whole buffer to 0x20000630, zeroes the DMA buffer, sets
  0x20000930 = 1 and restarts the DMA.
- The main loop (08006bb0) clears the flag and parses 0x20000630 **from index 0**. So the reply must:
  - be the first thing received after an idle gap;
  - arrive with no idle gap inside it (an IDLE in mid-frame splits it);
  - be at most 512 chars long. **[V]**/**[I]**: the split behaviour is a consequence of the code.

### 2.2 Decode and validate: v1 0800cc8c (l.5777) + 08006bb0 (l.1728)

1. **LENGTH chars must be hex.** ASCII positions 9–12 must each be `0-9A-Fa-f` (080098e4 returns −1 otherwise, and the
   frame is rejected). **[V]**
2. **LCHKSUM.** With `w` = the 16-bit LENGTH, the check is `((~((w>>8)+(w>>4)+w)) & 0xF) + 1 == w>>12`
   (disasm 0800cd22–0800cd34: `bics r0 = 0xF & ~sum; adds #1; cmp`). **[V]**
   - This equals the standard `(−(n2+n1+n0)) mod 16` except when the nibble sum ≡ 0 mod 16. In that case the left side is
     16 and **can never match.** **[V]**
   - So stock rejects any reply whose LENID nibbles sum to a multiple of 16, including **LENID = 0** (LENGTH `0000`), and
     also 0x01F, 0x02E, 0x03D, 0x04C, 0x05B, 0x06A, 0x079, 0x088, … **[V]** (by the arithmetic)
   - **A fake BMS must never answer with an empty INFO.** It should pad or choose a LENID that avoids this.
3. **Hex-decode the whole frame.** `LENID = w & 0xFFF`. `(LENID+16)>>1` byte pairs are decoded, starting at ASCII
   position 1, into `d[]` at 0x200004B3. That covers 12 header chars + LENID INFO chars + 4 CHKSUM chars. **[V]**
   - A non-hex pair decodes silently to 0xFF and is **not** rejected (0800d0ac). **[V]**
   - Both upper- and lower-case hex are accepted. **[V]**
4. **CHKSUM.** The firmware compares `FUN_0800613c(&raw[1], LENID+12)` with `d[6+LENID/2]<<8 | d[7+LENID/2]`. In other
   words, the negated 16-bit sum of the ASCII chars from VER to the end of INFO must equal the decoded 4-char CHKSUM.
   **[V]** (disasm 08006bec–08006c0c, both sides zero-extended)
   - The checksum is over the characters actually received. A lower-case sender that sums its own lower-case chars
     passes. **[V]**
5. **No other checks in v1** **[V]**:
   - SOI is not checked: `raw[0]` is never read.
   - EOI is not checked.
   - VER, ADR, CID1 and **RTN are never read.** A reply with RTN ≠ 0 is parsed as data.
   - The INFO length is not compared with what the parser needs.

**v2 is stricter** (08013d6c, v2 l.11966) **[V]**:
- `raw[0] == '~'` and `raw[1] != '~'`;
- LENGTH ≠ `"0000"`, and LENID ≠ 0;
- LENID < 0x1EF;
- `raw[LENID+17] == '\r'`: EOI must sit exactly after the CHKSUM.

The LCHKSUM quirk and the CHKSUM check are unchanged (08008fd8). v2 still ignores VER, ADR, CID1 and RTN. **A reply that
satisfies v2 also satisfies v1.** Target v2's rules.

### 2.3 Link-loss counter **[V]**

The counter is the byte at 0x2000007D (v2: 0x20000A51). The "BMS lost" flag is `state+8` (v1, state = 0x200005D8) or
`state+9` (v2, state = 0x20001148). In v2 that flag forces voltage mode (`findings-v2.md` §3).

| event | v1 | v2 |
|---|---|---|
| every TX | `cnt++`; if `cnt > 1` → cnt = 2, lost = 1 | `cnt++`; if `cnt > 3` → cnt = 4, lost = 1 |
| frame received, decode/LCHKSUM fail | `cnt++`; if `cnt > 1` → lost | same, threshold `> 1` |
| frame received, CHKSUM mismatch | `cnt++`; if `cnt > 1` → lost | same, threshold `> 1` |
| valid frame | dispatch, `cnt = 0`, lost = 0 | same |

Consequences **[V]** (by the arithmetic):
- **v1:** two TXs with no valid reply between them set lost (~1 s).
- **v2:** four TXs set lost (~2 s).
- **A single bad frame** received after a TX (cnt is already 1) sets lost **at once, in both versions.** One good frame
  clears it.
- RX noise or an RS485 echo that triggers an IDLE also counts as a bad frame. **[I]**

## 3. INFO layouts as parsed

All multi-byte fields are read big-endian, `d[i]<<8 | d[i+1]`. The parsers never check LENID. If a reply is shorter than
the parser's reach, the missing fields come **silently from the previous frame**, left over in 0x200004B3, which is never
cleared. **[V]**

### 3.1 0x60: not parsed **[V]**

Any reply that passes §2.2 advances to 0x61. INFO must be non-empty (LCHKSUM quirk). Something like 2 bytes `0000` is
enough; see §5.

### 3.2 0x61: analog data. Parser 08007a54 (l.2395); v2 0800f7ac (v2 l.8793) is identical **[V]**

The values are stored at 0x2000002C (v2: 0x20000540) as raw u16 or u8. **No scaling or sign handling in the parser.**

**How the fields are named.** The "Pylon name" column comes from matching the offsets against the Pylontech high-voltage
0x61 layout:

| INFO | Pylon field | size |
|---|---|---|
| 0–1 | total V | 2 |
| 2–3 | I | 2 |
| 4 | SOC | 1 |
| 5–6 | avg cycles | 2 |
| 7–8 | max cycles | 2 |
| 9 | avg SOH | 1 |
| 10 | min SOH | 1 |
| 11–18 | max cell V, its module, min cell V, its module | 4 × 2 |
| 19–28 | cell temperature: avg, max, max-module, min, min-module | 5 × 2 |
| 29–38 | MOSFET temperature: same 5 fields | 5 × 2 |
| 39–48 | BMS temperature: same 5 fields | 5 × 2 |

The parser reads exactly the value fields of this layout and skips every module-number field and both cycle counts. That
makes the mapping a strong **[I]**, but it is still **[I]**: the units are not used by the code.

- **Cycles.** `findings-v2.md` §3 lists "cycles" among the 0x61 fields. By this mapping the cycle counts (INFO 5–8) are
  *not* stored.

| INFO | d[] | width | stored at (v1) | Pylon name **[I]** | use **[V]** |
|---|---|---|---|---|---|
| 0–1 | 6–7 | u16 | +0x00 | total voltage | v1: none. v2: app status frame (080061f4 mode 2) |
| 2–3 | 8–9 | 16, stored as `short` | +0x02 | current. Signed by type only; no code tests the sign | v1: none. v2: app frame |
| 4 | 10 | u8 | +0x04 | **SOC %** | v1 08006d68 (l.1823): `state+6 = SOC` if link OK, else 0. This is the BMS-SOC mode input. v2: `state+7` (0800bbf0, v2 l.5388), app frame |
| 9 | 15 | u8 | +0x05 | avg SOH | none found |
| 10 | 16 | u8 | +0x06 | min SOH | none found |
| 11–12 | 17–18 | u16 | +0x08 | max cell V | none found |
| 15–16 | 21–22 | u16 | +0x0A | min cell V | none found |
| 19–20 | 25–26 | u16 | +0x0C | avg cell T | none found |
| 21–22 | 27–28 | u16 | +0x0E | max cell T | none found |
| 25–26 | 31–32 | u16 | +0x10 | min cell T | none found |
| 29–30 | 35–36 | u16 | +0x12 | MOSFET avg T | none found |
| 31–32 | 37–38 | u16 | +0x14 | MOSFET max T | none found |
| 35–36 | 41–42 | u16 | +0x16 | MOSFET min T | none found |
| 39–40 | 45–46 | u16 | +0x18 | BMS avg T | none found |
| 41–42 | 47–48 | u16 | +0x1A | BMS max T | none found |
| 45–46 | 51–52 | u16 | +0x1C | BMS min T | none found |

- **Minimum INFO.** The parser reads up to INFO 46, so INFO needs at least 47 bytes. A full HV layout has 49 bytes
  (LENID = 98 = 0x062).
- **"None found."** In v1 the only literal pointing into this block outside the parser is the SOC read (`DAT_08007158`).
  **[V]** (all v1 literals resolved with `tools/dat.py`)
- **Units.** None of these fields is scaled anywhere, so the units cannot be read from the code. The test vector in §5
  assumes mV, 10 mA and 0.1 K. **[I]**

### 3.3 0x62: alarms

- **v1, 080077d0 (l.2361).** Clears flag bytes 0x20000028[0], [1] (all bits) and [2] bits 0–5. **It reads nothing from
  the reply.** **[V]**
  - 0800c620, called after every TX, folds those flags into summary bits [2].7 and [2].6. They are therefore always clear
    in v1. **[V]**
- **v2, 0800f4f8 (v2 l.8738)** **[V]**:
  - **Flags at 0x2000052C.** Bit-copies are *assigned*, not latched. "fl[k].b" means bit b of flag byte k.

    | source | destination |
    |---|---|
    | INFO 0 bits 0–7 | fl[0].0–7 |
    | INFO 1 bits 4–7 | fl[1].0–3 |
    | INFO 2 bits 1–4 | fl[1].4–7 |
    | INFO 2 bits 5–7 | fl[2].0–2 |
    | INFO 3 bit 3 | fl[2].3 |
    | INFO 3 bit 5 | fl[2].4 |
    | INFO 3 bit 6 | fl[2].5 |

  - **Alarm word at 0x20000530.** The 32-bit word is updated with
    `|= INFO0<<24 | INFO1<<16 | INFO2<<8 | INFO3`. This is an **OR**, so it is sticky.
  - **Error 12.** In 0800b9b4, if the link is OK and `state+0x50 == 0`, then `(u16 @0x20000530) & 0xAA68` sets
    `state+0x50 = 12`. Because the word is little-endian, that halfword is `INFO2<<8 | INFO3`. The test is therefore
    **INFO 2 & 0xAA** (bits 1, 3, 5, 7) or **INFO 3 & 0x68** (bits 3, 5, 6).
  - **Clearing.** The word is cleared only while lost = 1. Error 12 is cleared only on that same lost path, if it is
    currently 12. So with the link up, **an alarm bit seen once stays latched, and so does error 12, until the BMS link
    drops.** **[V]** for 0800b9b4. Whether other code clears `+0x50` was not fully traced. **[I]**
  - **Bit meanings.** Pylon HV puts warning bytes first, then protection bytes, so INFO 2–3 would be "protect" flags.
    **[I]**, as in `findings-v2.md`.
  - **Minimum INFO.** 4 bytes.

### 3.4 0x63: charge management. Parser 08007bc8 (l.2425); v2 0800f920 (v2 l.8823) is identical **[V]**

Stored at 0x200005B4 (v2: 0x2000101E).

| INFO | d[] | width | stored | Pylon name **[I]** | use **[V]** |
|---|---|---|---|---|---|
| 0–1 | 6–7 | u16 | +0x00 | CVL (charge voltage upper limit) | none in v1. v2: app frame only (never VOUT, `findings-v2.md` l.114) |
| 2–3 | 8–9 | u16 | +0x02 | DVL (discharge voltage lower limit) | v2 app frame only |
| 4–5 | 10–11 | u16 | +0x04 | **CCL** | v2 follow mode (08010e0c, v2 l.9886): if CCL < 5000, the limit is CCL × 0.005 → 0.01 A units **[V]** by the arithmetic (see `findings-v2.md` §2). v2 app frame also shows `CCL/100` capped at 100 (v2 l.1658) |
| 6–7 | 12–13 | u16 | +0x06 | DCL | v2 app frame only |
| 8 bit 4 | 14 | flag u8 | +0x08 | "charge immediately 2" **[I]** | display/app only |
| 8 bit 5 | 14 | flag u8 | +0x09 | "charge immediately 1" **[I]** | display/app only |
| 8 bit 6 | 14 | flag u8 | +0x0A | discharge enable **[I]** | display/app only |
| 8 bit 7 | 14 | flag u8 | +0x0B | charge enable **[I]** | display/app only (`findings-v2.md` §3) |

- The v2 app frame repacks the four flags as `b7 | b6<<1 | b5<<2 | b4<<3` (v2 l.1838). **[V]**
- Bits 0–3 of INFO 8 are ignored. **[V]**
- **Minimum INFO.** 9 bytes.
- **v1.** No literal other than the parser's points at 0x200005B4, so the stored 0x63 data is unused in v1. **[V]**

## 4. ADR, VER and variable length

- **VER, ADR, CID1 and RTN in replies are never checked** in either version. **[V]** The reply header can echo `20 12 46 00`.
- **The request ADR is fixed** at 0x12 in `.data`. No code was found that changes 0x20000009..0x2000000B. **[I]**: only the
  TX reads them.
- **No variable-length handling.** There is no cell count, no module count and no loop over INFO. Every parser uses fixed
  offsets. **[V]**
- **LENID bounds** **[V]**:
  - The only use of LENID is to bound the decode and locate the CHKSUM.
  - In v1 a corrupt LENGTH with a valid LCHKSUM could decode up to 2055 bytes into 0x200004B3. The block between that
    buffer and the 0x63 store at 0x200005B4 is 257 bytes. **[V]** for the arithmetic; the overrun is **[I]**.
  - v2 caps LENID below 0x1EF.

## 5. Test vectors (**synthetic**: made up here, not captured)

Every frame below was checked with a Python port of 0800cc8c, 08006bb0 and the parsers, plus the extra v2 rules.

**0x61 reply.** 49-byte INFO, LENID 0x062, LCHKSUM 8. It is 116 chars plus CR:
```
~201246008062CFD003E84B0032003462610D0C00010CF800020BA50BAF00010B9B00020BB90BC300010BAF00020BB90BC300010BAF0002E8A5\r
```

Decoded:

| field | raw | meaning **[I]**: assumed units, not used by the code |
|---|---|---|
| total V | 0xCFD0 = 53200 | 53.200 V if in mV |
| current | 0x03E8 = 1000 | 10.00 A if in 10 mA |
| **SOC** | 0x4B = **75** | 75 %, this is what stock uses |
| cycles (not stored) | 50 avg, 52 max | |
| SOH | 98 avg, 97 min | |
| max cell | 3340 | 3.340 V, module 1 |
| min cell | 3320 | 3.320 V, module 2 |
| cell T | avg 2981, max 2991, min 2971 | 25.0 / 26.0 / 24.0 °C if in 0.1 K |
| MOSFET T and BMS T | avg 3001, max 3011, min 2991 each | |

**0x63 reply.** 9-byte INFO, LENID 0x012, LCHKSUM D:
```
~20124600D012163011300FA02710C0F9E7\r
```

Decoded: CVL = 0x1630 = 5680, DVL = 0x1130 = 4400, **CCL = 0x0FA0 = 4000** (40.00 A in v2's 0.01 A reading), and
DCL = 0x2710 = 10000. The status byte is 0xC0, giving bit7 = 1 and bit6 = 1, with bits 5 and 4 = 0.

**0x60 reply (minimal).** It passes, so the sequence moves on to 0x61:
```
~20124600C0040000FCDA\r
```

**Negative checks** **[V]** (on the emulator):
- `~201246000000FDB1\r` (empty INFO, otherwise correct) is **rejected** by the LCHKSUM quirk.
- The 0x61 vector with SOI replaced passes v1 but fails v2.
- The 0x61 vector lower-cased without recomputing CHKSUM fails CHKSUM.

## 6. Open questions

1. **Units.** The units of 0x61 total V, current and temperatures, and of 0x63 CVL, DVL and DCL, are not visible in the
   code. Only CCL (0.01 A, v2) and SOC (%) have code-implied units. A capture from a real Eco-Worthy BMS would settle them.
2. **Byte 0x2000007C.** It gates the whole main-loop task block, including the BMS poll. Its meaning is not traced.
3. **Error-12 latch in v2.** Whether anything other than the link-lost path clears the 0x20000530 word or `+0x50 == 12`
   is not fully traced.
4. **Echo and back-to-back frames.** It is not established whether a real transceiver echoes, or whether two frames can
   arrive within one main-loop pass. The second would overwrite the first at 0x20000630. Not tested on hardware.
