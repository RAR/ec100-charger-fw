# OEM v1 app: literal register-level CAN usage (for our Phase-2 driver)

Companion to `docs/history/notes/oem-init-sequences.md` (§1–§7). Same conventions:
source is `dumps/gd32_flash_20261012.bin` (base 0x08000000), v1 app disassembly (`scratchpad/v1.dis`), literal pools resolved by hand.
Tags: **[V]** = verified in code / literals. **[I]** = inferred (datasheet, other GD32 family header, or reasoning).

Register names come from the GD32E50x/GD32F30x-style `*_can.h` (zephyr `hal_gigadevice`), because the GD32L23x v1.0
library has no CAN. Every address below was taken from a literal in the v1 app, so the **addresses are [V]**; the
**names/bit meanings are [I]** (from the other-family header), except where the code's behaviour pins them down.

## 0. Bottom line

- **The L235 CAN is a classic GD32 "bxCAN" (CAN0 of the F10x/F30x/E50x family).** Every access matches that map exactly:
  CTL/STAT/TSTAT/RFIFO0/RFIFO1/INTEN/BT at +0x00…+0x1C; mailboxes TMIx/TMPx/TMDATA0x/TMDATA1x at **+0x180 + 16·n**;
  RX FIFO mailboxes at **+0x1B0 + 16·fifo**; filter block FCTL/FMCFG/FSCFG/FAFIFO/FW at +0x200…+0x21C; FxDATA0/1 at **+0x240 + 8·n**. **[V]**
  The datasheet agrees: "three mailboxes for transmission, two FIFOs of three message deep, 28 filter banks" (DS §3.24). **[V vs ds/l235.txt]**
- **Base 0x40006400** (DS memory map: CAN 0x4000_6400–0x4000_67FF). **[V]**
- **Shared USBD/CAN RAM (0x4000_6000–0x4000_63FF, 512 B): the stock code never touches it.** There is no literal anywhere
  in the app (or the bootloader) in 0x40005C00–0x400063FF, and no extra RCU bit is set for it. This is the STM32F103 arrangement
  (CAN and USB share that SRAM; the CAN needs no software setup for it, and the two can't be used at once). **[V]** that stock does nothing; **[I]** the F103 analogy.
- The whole driver is the GD standard library CAN code (can_deinit / can_init / can_filter_init / can_interrupt_enable /
  can_message_transmit / can_transmit_states / can_message_receive / can_struct_para_init), compiled -O0. **[V]** from bodies.
- **Stock does no error handling at all:** no ERR reads, no error/bus-off interrupts, no abort, no overrun checks, no
  re-init. Recovery relies entirely on hardware **ABOR = 1**. **[V]**

v1 function map **[V]**:

| v1 addr | GD lib equivalent | notes |
|---|---|---|
| 08008254 | (app) CAN bring-up | called once from `init` 0800b50c @0800b5b0 (step 16) |
| 0800dfa4 | can_struct_para_init(type, p) | types 0 (init) and 1 (filter) are used |
| 0800da70 | can_deinit | APB1RST bit16 pulse |
| 0800dba0 | can_init(p) | returns SUCCESS/ERROR; **result ignored by caller** |
| 0800da88 | can_filter_init(p) | |
| 0800dd60 | can_interrupt_enable(x) | INTEN \|= x |
| 0800de88 | can_message_transmit(p) → mailbox 0/1/2 or 3 (none) | |
| 0800e0e8 | can_transmit_states(mb) → 0 FAILED / 1 OK / 2 PENDING | |
| 08008328 | (app) send-and-wait | spins ≤ 0x0FFF |
| 0800dd78 | can_message_receive(fifo, p) | |
| 0800d274 | IRQ20 handler (vector at 0x08005000 + 4·(16+20) = 0x0800D275) | |
| 08008bc8 | (app) factory-test CAN echo | §4.4 |

---

## 1. RCU and GPIO (v1 08008254) [V]

| # | v1 addr | register (addr) | operation | wait |
|---|---|---|---|---|
| 1 | 0800f5ac(0x710) | RCU_APB1EN (0x4002101C) | \|= **bit16 (CANEN)** | — |
| 2 | 0800f5ac(0x511) | RCU_AHBEN (0x40021014) | \|= bit17 PAEN | — |
| 3 | 0800e97c(GPIOA, PP=0, speed=3, 0x10) | GPIOA_OMODE (0x48000004) / OSPD (0x48000008) | OMODE &= ~bit4; OSPD = (OSPD & ~0x300) \| 0x300 (50 MHz) | — |
| 4 | 0800e8ee(GPIOA, AF=2, NONE=0, 0x10) | GPIOA_CTL (0x48000000) / PUD (0x4800000C) | CTL = (CTL & ~0x300) \| 0x200; PUD &= ~0x300 | — |
| 5 | 0800e7d4(GPIOA, 11, 0x10) | GPIOA_AFSEL0 (0x48000020) | = (AFSEL0 & ~0x000F0000) \| 0x000B0000 (**PA4 = AF11 CAN_RX**) | — |
| 6 | 0800e97c(GPIOA, 0, 3, 0x20) | OMODE / OSPD | OMODE &= ~bit5; OSPD \|= 0xC00 | — |
| 7 | 0800e8ee(GPIOA, 2, 0, 0x20) | CTL / PUD | CTL = (CTL & ~0xC00) \| 0x800; PUD &= ~0xC00 | — |
| 8 | 0800e7d4(GPIOA, 11, 0x20) | AFSEL0 | = (AFSEL0 & ~0x00F00000) \| 0x00B00000 (**PA5 = AF11 CAN_TX**) | — |
| 9 | 0800da70 → 0800f5f4/0800f5d0(0x410) | RCU_APB1RST (0x40021010) | \|= **bit16 (CANRST)**, then &= ~bit16 | — |

Net PA4/PA5 fields: CTL mask 0x00000F00 → 0x00000A00 (AF), PUD mask 0x00000F00 → 0 (no pull, **both pins incl. RX**),
OMODE bits 4,5 = 0 (push-pull), OSPD mask 0x00000F00 → 0x00000F00 (50 MHz), AFSEL0 mask 0x00FF0000 → 0x00BB0000. **[V]**
Order per pin is options → mode → AF (so each pin is briefly AF mode with its old AFSEL; harmless). AF11 = CAN_RX/CAN_TX on PA4/PA5 matches the DS AF table. **[V vs DS]**
APB1EN/APB1RST bit16 is undefined in the L23x v1.0 header (it has no CAN), consistent with it being CANEN/CANRST on L235. **[V]** the bit; name **[I]**.
CAN sits on APB1 = 30 MHz (DS block diagram; APB1 max 32 MHz). **[V]** clock from §1 of init notes.

---

## 2. CAN init — can_init 0800dba0, called with the struct built at 080082d4 [V]

Init struct the caller passes (byte offsets, GD `can_parameter_struct`): working_mode=0 (normal), sjw=0 (1 tq), bs1=4 (5 tq),
bs2=1 (2 tq), time_triggered=0, auto_bus_off_recovery=1, auto_wake_up=0, auto_retrans=1, rec_fifo_overwrite=0, trans_fifo_order=0, prescaler=30.

| # | register (addr) | operation | wait / result |
|---|---|---|---|
| 1 | CAN_CTL (0x40006400) | &= ~0x2 (clear **SLPWMOD**) | — |
| 2 | CAN_CTL | \|= 0x1 (**IWMOD**, request init mode) | — |
| 3 | CAN_STAT (0x40006404) | | **while ((STAT & 1) != 1 && t--)**, t starts at **0x0000FFFF**. If STAT.IWS still 0 → return ERROR immediately (BT/CTL **not** written). Caller ignores the return. |
| 4 | CAN_BT (0x4000641C) | **= 0x0014001D** (plain write, not RMW) | built as `mode<<30 \| (sjw&3)<<24 \| (bs1&0xF)<<16 \| (bs2&7)<<20 \| ((psc−1)&0x3FF)` |
| 5 | CAN_CTL | &= ~0x80 (**TTC = 0**, no time-triggered) | |
| 6 | CAN_CTL | \|= 0x40 (**ABOR = 1**, automatic bus-off recovery) | |
| 7 | CAN_CTL | &= ~0x20 (**AWU = 0**) | |
| 8 | CAN_CTL | &= ~0x10 (**ARD = 0**, automatic retransmission ON) | (field==1 → clear ARD) |
| 9 | CAN_CTL | \|= 0x08 (**RFOD = 1**, RX FIFO overwrite *disabled*: when full, new frames are discarded) | (field==0 → set RFOD) |
| 10 | CAN_CTL | &= ~0x04 (**TFO = 0**, TX order by identifier priority) | |
| 11 | CAN_CTL | &= ~0x01 (leave init mode) | |
| 12 | CAN_STAT | | **while ((STAT & 1) && t--)**, t reloaded with **0xFFFF**; returns SUCCESS if t ≠ 0. |

- BT = 0x0014001D: BAUDPSC = 29 (/30), BS1 = 4 (5 tq), BS2 = 1 (2 tq), SJW = 0 (1 tq), **LCMOD (bit30) = 0, SCMOD (bit31) = 0**.
  30 MHz / 30 = 1 MHz tq; 1 + 5 + 2 = 8 tq → **125 kbit/s, sample point 75 %**. **[V]**
- **No silent/loopback mode is ever used** (BT is written once, with bits 30/31 = 0; there is no other BT write and no other caller of can_init). **[V]**
- **Final CTL = 0x00010048 [I]**: ABOR | RFOD, plus DFZ (bit16, debug freeze), which stock never touches and is 1 after reset on GD32F30x
  (CTL reset value 0x00010002). Our driver should write the bits explicitly rather than rely on that. SWRST (bit15) is never used. **[V]** for bits 0–7.
- Time quanta values in the gd32 lib encoding: struct bs1=4 → BT[19:16]=4; bs2=1 → BT[22:20]=1 (hardware adds 1). **[V]**

---

## 3. Filter — can_filter_init 0800da88, struct built at 080082fc [V]

Struct: list_high=0, list_low=0, mask_high=0, mask_low=0, fifo=0, filter_number=0, mode=0 (MASK), bits=1 (32-bit), enable=1.

| # | register (addr) | operation |
|---|---|---|
| 1 | CAN_FCTL (0x40006600) | \|= 0x1 (**FLD**, enter filter-init) |
| 2 | CAN_FW (0x4000661C) | &= ~bit0 (deactivate bank 0) |
| 3 | CAN_FSCFG (0x4000660C) | \|= bit0 (**32-bit scale**) |
| 4 | CAN_F0DATA0 (0x40006640) | = (list_high<<16) \| list_low = **0x00000000** (ID) |
| 5 | CAN_F0DATA1 (0x40006644) | = (mask_high<<16) \| mask_low = **0x00000000** (mask) |
| 6 | CAN_FMCFG (0x40006604) | &= ~bit0 (**mask mode**) |
| 7 | CAN_FAFIFO (0x40006614) | &= ~bit0 (**→ FIFO0**) |
| 8 | CAN_FW | \|= bit0 (**activate bank 0**) |
| 9 | CAN_FCTL | &= ~0x1 (leave filter-init) |

- **Stock accepts every frame** (std and ext, data and remote): one 32-bit mask filter, ID 0 / mask 0, to FIFO0. Banks 1–27 untouched. **[V]**
- The 0x1101007E / 0x1102007E rejection is done **in software** in the ISR, not by the filter (§5). **[V]**
- FCTL is RMW'd only on bit0; HBC1F (bits 8–13) is never written. (On single-CAN L235 it presumably doesn't matter.) **[I]**
- Filter bank register formula used by the code: FxDATA0 = 0x40006640 + 8·n, FxDATA1 = 0x40006644 + 8·n. **[V]**

### 3a. Interrupt + NVIC (08008314)

| # | v1 addr | register | operation |
|---|---|---|---|
| 1 | 0800dd60(2) | CAN_INTEN (0x40006414) | \|= 0x2 (**RFNEIE0**, FIFO0 not empty) — the only INTEN bit ever set |
| 2 | 0800ea28(20, 0) | NVIC IPR/ISER | IRQ **20**, priority byte 0, enable |

INTEN reset value is 0, so **INTEN = 0x00000002** finally. **[V]** No TMEIE, no FIFO full/overrun, no error (WERRIE/PERRIE/BOIE/ERRNIE/ERRIE), no FIFO1. **[V]**
IRQ20 is USBD_LP in the L23x header; on L235 it is evidently shared with CAN RX0 (cf. F103 `USB_LP_CAN1_RX0` = 20). Vectors for TX/RX1/SCE on L235 are unknown **[I]** — stock doesn't need them.

---

## 4. TX

### 4.1 can_message_transmit 0800de88 [V]

TX message struct (same 20-byte layout as the RX struct): +0 sfid (u32), +4 efid (u32), +8 ff (u8, **raw TMI bit value: 0 or 4**),
+9 ft (u8, raw: 0 data / 2 remote), +10 dlen (u8), +12..+19 data[8] (read as two u32 words).

| # | register (addr) | operation |
|---|---|---|
| 1 | CAN_TSTAT (0x40006408) | read; mailbox = 0 if bit26 TME0, else 1 if bit27 TME1, else 2 if bit28 TME2, else **3 → return 3 (no mailbox), nothing written** |
| 2 | CAN_TMIn (0x40006580 + 16·n) | &= 0x1 (keep only TEN; clears old ID/FF/FT) |
| 3 | CAN_TMIn | ext (ff ≠ 0): \|= (efid << 3) \| ff \| ft. std (ff == 0): \|= (sfid << 21) \| ft |
| 4 | CAN_TMPn (0x40006584 + 16·n) | &= ~0xF; \|= dlen (DLENC; upper bits incl. TSEN preserved) |
| 5 | CAN_TMDATA0n (0x40006588 + 16·n) | = data[0] \| data[1]<<8 \| data[2]<<16 \| data[3]<<24 |
| 6 | CAN_TMDATA1n (0x4000658C + 16·n) | = data[4] \| … \| data[7]<<24 |
| 7 | CAN_TMIn | \|= 0x1 (**TEN**, request transmission) |
| — | return n | |

All stock requests are built by 0800c428: **ff = 4 (FF, extended), ft = 0 (data), dlen = 8**, efid = 0x108081FE \| addr<<16 (write, cmd 0x81)
or 0x108082FE \| addr<<16 (read, cmd 0x82), data[0..1] = register (big-endian), rest per register. **[V]**
TMI layout for an extended data frame: bits 31..3 = EFID[28:0], bit2 FF = 1, bit1 FT = 0, bit0 TEN.
Example: ID 0x108182FE → TMI = **0x840C17F5** after TEN, TMP DLENC = 8, `02 3B 00 00 …` → TMDATA0 = **0x00003B02**, TMDATA1 = 0. **[V]** arithmetic.
This also explains the "pending TX mailboxes" seen on the bench: TEN stays set while the frame can't get an ACK.

### 4.2 Completion — can_transmit_states 0800e0e8 [V]

`v = TSTAT & mask(n)`, mask0 = **0x04000003** (TME0|MTFNERR0|MTF0), mask1 = **0x08000300**, mask2 = **0x10030000**; mailbox 3 → v = 0.
v == 0 → **2 PENDING**; v == mask → **1 OK**; anything else → **0 FAILED**.
TSTAT is **never written** (MTF/MTFNERR/MAL/MTE are not cleared by software; hardware clears them on the next TEN of that mailbox). **[V]** / **[I]** for the hardware-clear.

### 4.3 send-and-wait 08008328 [V]

```
mb = can_message_transmit(msg);
t  = 0x0FFF;
while (can_transmit_states(mb) != 1 /*OK*/ && t != 0) t--;
```
- No result is returned/used, no abort (CAN_TMIn / TSTAT.MSTn never written), no retry. With no free mailbox (mb = 3) or
  a frame that never gets ACKed, it simply burns the 4095 iterations and returns; hardware keeps retrying (ARD = 0) with TEN set. **[V]**
- Wall time: 4095 × (call + ~20 instructions at -O0) ≈ 2–3 ms at 60 MHz **[I]**; one 8-byte ext frame at 125 k ≈ 1 ms.
- Callers: 08007d70 (write path, after 0800c428), 0800c57a (poll cycle, after 0800bf4c), 08008c18 (factory echo). **[V]**

### 4.4 Factory-test CAN echo 08008bc8 [V]

Only on the factory-test path (08005c38, main loop when 0x2000007C ≠ 0). If the RX flag **0x2000004F == 1**: clear it, copy
the RX slot at **0x20000050** (sfid, efid, ff, ft, dlen, data[8]) into a TX struct and send-and-wait it. So it re-transmits
**every received frame with the same ID/format/data**. It does **not** use CAN silent/loopback hardware modes. **[V]**

---

## 5. RX — IRQ20 handler 0800d274 → can_message_receive(0, &m) 0800dd78 [V]

Handler: no INTF/STAT/RFIFO0 check; reads exactly one message per entry (the level-sensitive RFNE0 re-fires if more are queued **[I]**).

| # | register (addr) | operation |
|---|---|---|
| 1 | CAN_RFIFOMI0 (0x400065B0) | ff = MI & 0x4. If ff == 0: sfid = MI >> 21; else **efid = MI >> 3** (no mask needed; bits 31..3) |
| 2 | CAN_RFIFOMI0 | ft = MI & 0x2 |
| 3 | CAN_RFIFOMP0 (0x400065B4) | fi = (MP >> 8) & 0xFF (filter index); dlen = MP & 0xF |
| 4 | CAN_RFIFOMDATA00 (0x400065B8) | data[0..3] = bytes 0..3 (LSB first) |
| 5 | CAN_RFIFOMDATA10 (0x400065BC) | data[4..7] |
| 6 | CAN_RFIFO0 (0x4000640C) | \|= 0x20 (**RFD0**, release the FIFO0 output mailbox). (fifo 1 would use 0x40006410.) |

Then in the handler:
- if `m.efid == 0x1101007E` or `== 0x1102007E` → drop (compares efid only, without checking ff; for a std frame efid is stack garbage). **[V]**
- else copy the 20-byte struct to **0x20000050** (single slot, latest wins) and set **0x2000004F = 1**. **[V]**
- RFL0, RFF0 (bit3) and RFO0 (bit4) are never read or cleared; with RFOD = 1 an overflow just drops the newest frame. **[V]**

---

## 6. Error handling [V]

- **None in software.** CAN_ERR (0x40006418) is never read; no literal for it exists. No WERR/PERR/BOFF/ERRN interrupt, no TEC/REC
  inspection, no re-init, no mailbox abort, no sleep/wakeup handling.
- Bus-off recovery is purely hardware (**CTL.ABOR = 1**: leaves bus-off after 128 × 11 recessive bits). **[V]** bit; **[I]** timing (bxCAN behaviour).
- The ERR = 0x00B00053 seen on the bench (TEC 176, warning + passive, ERRN = 5 bit-dominant) is consistent: with ARD = 0
  the pending mailboxes keep retrying and the counters stay error-passive, which stock never notices.

---

## 7. Register map used (CAN base 0x40006400)

| offset | addr | name (GD) | used by v1 | source of the name |
|---|---|---|---|---|
| +0x000 | 0x40006400 | CAN_CTL | can_init | addr **[V]**, bits from gd32e50x_can.h, behaviour consistent **[I]** |
| +0x004 | 0x40006404 | CAN_STAT (bit0 IWS) | can_init | **[V]** addr / **[I]** name |
| +0x008 | 0x40006408 | CAN_TSTAT | transmit, transmit_states | **[V]**/**[I]** |
| +0x00C | 0x4000640C | CAN_RFIFO0 (bit5 RFD0) | receive | **[V]**/**[I]** |
| +0x010 | 0x40006410 | CAN_RFIFO1 | receive (fifo 1 branch, unused) | **[V]**/**[I]** |
| +0x014 | 0x40006414 | CAN_INTEN | interrupt_enable | **[V]**/**[I]** |
| +0x018 | 0x40006418 | CAN_ERR | **not used** | header only **[I]** |
| +0x01C | 0x4000641C | CAN_BT | can_init | **[V]**/**[I]** |
| +0x180 + 16n | 0x40006580/90/A0 | CAN_TMIn | transmit | **[V]** |
| +0x184 + 16n | | CAN_TMPn | transmit | **[V]** |
| +0x188 / +0x18C + 16n | | CAN_TMDATA0n / TMDATA1n | transmit | **[V]** |
| +0x1B0 / +0x1B4 / +0x1B8 / +0x1BC (+16 for FIFO1) | | RFIFOMI / RFIFOMP / RFIFOMDATA0 / RFIFOMDATA1 | receive | **[V]** |
| +0x200 | 0x40006600 | CAN_FCTL | filter | **[V]** |
| +0x204 | 0x40006604 | CAN_FMCFG | filter | **[V]** |
| +0x20C | 0x4000660C | CAN_FSCFG | filter | **[V]** |
| +0x214 | 0x40006614 | CAN_FAFIFO | filter | **[V]** |
| +0x21C | 0x4000661C | CAN_FW | filter | **[V]** |
| +0x240 + 8n / +0x244 + 8n | 0x40006640/44 (n=0) | CAN_FnDATA0 / FnDATA1 | filter | **[V]** |

- **Exhaustive check [V]:** scanning the app (0x08005000…) for 32-bit literals in 0x40005C00–0x400067FF finds exactly the 22 literals above
  (0x40006400, 404, 408, 40C, 410, 414, 41C, 580, 584, 588, 58C, 5B0, 5B4, 5B8, 5BC, 600, 604, 60C, 614, 61C, 640, 644), all inside
  0800da88…0800e19c. None for USBD (0x40005C00) or the shared RAM (0x40006000). The bootloader has none at all.
  Every CAN function has exactly one call site chain from 08008254 / 08008328 / IRQ20, so there is no second (test-mode) configuration.
- Where L235 might differ from the F30x/E50x header **[I]**: no CAN-FD registers (+0x20…+0x2C are E50x FD extensions; DS says CAN2.0B only);
  BT.SJW is written as 2 bits (24–25) like F30x, not 5 like E50x; the RX1/TX/SCE IRQ numbers; the filter bank count (DS: 28) and HBC1F semantics
  with a single CAN; DFZ reset value. The register *offsets* actually exercised are all identical to F10x/F30x bxCAN.

---

## 8. Shared USBD/CAN SRAM (AN184 / DS)

- DS memory map: `0x4000 6000 – 0x4000 63FF  Shared USBD/CAN RAM (512 bytes)`, CAN registers at 0x4000 6400. **[V vs DS]**
- Stock v1 **does not touch** 0x40006000–0x400063FF, USBD registers, RCU_APB1EN.USBDEN (bit23) or RCU_CFG2.USBDSEL. CAN mailboxes are
  accessed only through the bxCAN register windows at +0x180/+0x1B0. So the CAN IP is **classic bxCAN**, not a message-RAM IP
  (M_CAN-style or USBD-PMA-style); the shared SRAM is internal backing for the CAN (as on F103) and needs no software setup. **[V]** stock usage; **[I]** "needs no setup" (consistent with it working in the field).
- Implication for us: never enable USBD while CAN is in use. **[I]**

---

## 9. Literal replication (pseudo-C, what our driver should do to match stock)

```c
RCU_APB1EN |= 1u<<16;  RCU_AHBEN |= 1u<<17;                 // CANEN, PAEN
// PA4/PA5: PP, 50 MHz, AF, no pull, AF11
GPIOA_OMODE &= ~0x30;  GPIOA_OSPD |= 0xF00;
GPIOA_CTL = (GPIOA_CTL & ~0xF00) | 0xA00;  GPIOA_PUD &= ~0xF00;
GPIOA_AFSEL0 = (GPIOA_AFSEL0 & ~0x00FF0000) | 0x00BB0000;
RCU_APB1RST |= 1u<<16;  RCU_APB1RST &= ~(1u<<16);            // can_deinit
CAN_CTL &= ~2u;  CAN_CTL |= 1u;                              // wake, request init
for (t=0xFFFF; !(CAN_STAT & 1) && t; t--);                   // stock: give up silently if IWS never sets
CAN_BT  = 0x0014001D;                                        // 125k @ 30 MHz, normal mode
CAN_CTL = (CAN_CTL & ~0xBC) | 0x48;                          // TTC0 ABOR1 AWU0 ARD0 RFOD1 TFO0 (stock does 6 separate RMWs)
CAN_CTL &= ~1u;  for (t=0xFFFF; (CAN_STAT & 1) && t; t--);
CAN_FCTL |= 1;  CAN_FW &= ~1;  CAN_FSCFG |= 1;
CAN_F0DATA0 = 0;  CAN_F0DATA1 = 0;  CAN_FMCFG &= ~1;  CAN_FAFIFO &= ~1;  CAN_FW |= 1;  CAN_FCTL &= ~1;
CAN_INTEN |= 0x2;  nvic_irq_enable(20, prio 0);
```
