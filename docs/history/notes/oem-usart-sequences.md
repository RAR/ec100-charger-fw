# OEM v1 app: literal register-level USART1 / DMA RX / RS485 DE usage (for our Phase-3a Modbus driver)

Companion to `docs/history/notes/oem-init-sequences.md`, `docs/history/notes/oem-adc-flash-sequences.md` and `docs/history/notes/oem-can-sequences.md`. Same conventions:
source is `dumps/gd32_flash_20261012.bin` (base 0x08000000), disassembled with
`arm-none-eabi-objdump -D -b binary -marm -Mforce-thumb --adjust-vma=0x08000000`; literal pools resolved from the binary.
Tags: **[V]** = verified in code / literals (or bench 2026-10-15). **[I]** = inferred (GD32L23x USART layout seen on UART4, the GD standard library, reasoning).

The USART code is the GD32L23x standard library compiled -O0 (usart_deinit, usart_baudrate_set, usart_word_length_set, usart_stop_bit_set,
usart_parity_config, usart_hardware_flow_rts/cts_config, usart_receive/transmit_config, usart_interrupt_enable, usart_enable,
usart_dma_receive_config, usart_flag_get, usart_data_transmit, usart_interrupt_flag_get/clear). **[V]** from bodies.
Register-encoded flag IDs are `(reg_offset << 6) | bit`, as in the FMC library. **[V]**

## 0. Bottom line

- **Stock USART1 = 9600 8N1, no flow control, TX polled, RX by DMA ch1 (circular bit set) + IDLE interrupt on IRQ28.** **[V]**
- **The part has a DMAMUX.** USART1_RX = **request ID 0x34 (52)** written to `DMAMUX_RM_CH1CFG` (0x40020804), MUXID field bits 6:0. **[V]**
  (UART3_RX = 0x36 on DMA ch4, UART4_RX = 0x38 on DMA ch3, ADC = 5 on ch2.) **[V]**
  The stock USART1 TX request ID is **not** in the app, because TX never uses DMA; by the pattern it is 0x35 (**[I]**).
- **DE (PB10) is active HIGH** (set = transmit). **[V]**
- **DE is controlled by software around a polled, per-byte TBE then TC transmit.** DE goes high, a 1-tick delay, bytes out, DE low immediately after the last TC. No TX DMA, no TX interrupt. **[V]**
- **Stock RX handling is IDLE-IRQ only.** The handler does not look at RBNE, ORERR, FERR or NERR; it does not use the DMA half/transfer-complete interrupts. **[V]**
- **No parity bit handling** (PCEN cleared) and no error recovery code for USART1. **[V]**

## 1. USART1 init (v1 08008824, param 1, baud 9600) [V]

Called once from `init` as `08008824(1, 9600)` (see `oem-init-sequences.md` step 14). Order of operations as executed:

| # | v1 call | register (addr) | operation |
|---|---|---|---|
| 1 | 0800f5ac(0x511) | RCU_AHBEN (0x40021014) | \|= bit17 (**PAEN**) |
| 2 | 0800f5ac(0x711) | RCU_APB1EN (0x4002101C) | \|= bit17 (**USART1EN**) |
| 3 | af_set 0800e7d4(GPIOA, AF7, PA2) and again for PA3 | GPIOA_AFSEL0 (0x48000020) | PA2 field (bits 11:8) = 7, PA3 field (bits 15:12) = 7 |
| 4 | mode_set 0800e8ee(GPIOA, AF=2, PULLUP=1, PA2) | GPIOA_CTL / PUD | PA2 CTL field = 2 (AF), PUD field = 1 (pull-up) |
| 5 | output_options 0800e97c(GPIOA, PP=0, 50 MHz=3, PA2) | GPIOA_OMODE / OSPD | OMODE bit2 = 0 (push-pull); OSPD PA2 = 3 (50 MHz) |
| 6 | mode_set 0800e8ee(GPIOA, AF=2, PULLUP=1, PA3) | GPIOA_CTL / PUD | PA3 CTL = 2 (AF), PUD = 1 (pull-up). **No** output options call for PA3 (OSPD stays at reset) |
| 7 | usart_deinit 0800fcb8(0x40004400) | RCU_APB1RST (0x40021010) | bit17 (`0x411`) set, then cleared (0800f5f4 then 0800f5d0) |
| 8 | usart_baudrate_set 0800fbc4(USART1, 9600) | USART1_BAUD (0x4000440C) | = **0x0C35** (see section 4) |
| 9 | word_length 0800ff66(…, 0) | CTL0 (+0x00) | UEN cleared (&= ~1), WL cleared (&= ~0x1000) then \|= 0: **8 data bits** |
| 10 | stop_bit 0800ff1e(…, 0) | CTL0 UEN cleared; CTL1 (+0x04) | &= ~0x3000 then \|= 0: **1 stop bit** |
| 11 | parity 0800fed6(…, 0) | CTL0 | UEN cleared; &= ~0x600 (PM bit9, PCEN bit10) then \|= 0: **no parity** |
| 12 | rts_config 0800fde6(…, 0) | CTL2 (+0x08) | UEN cleared; &= ~0x100 (RTSEN) then \|= 0 |
| 13 | cts_config 0800fdbc(…, 0) | CTL2 | UEN cleared; &= ~0x200 (CTSEN) then \|= 0 |
| 14 | receive_config 0800ff00(…, 4) | CTL0 | &= ~4; \|= 4: **REN (bit2) set** |
| 15 | transmit_config 0800ff48(…, 8) | CTL0 | &= ~8; \|= 8: **TEN (bit3) set** |
| 16 | interrupt_enable 0800fe10(…, 0x0004) | CTL0 | \|= bit4: **IDLEIE** (flag ID 0x004 = reg 0, bit 4) |
| 17 | nvic_irq_enable 0800ea28(28, 0) | NVIC IPR + ISER (0xE000E100) | IRQ28 priority **0** (`prio << 6` in the byte field, 0800d428), then ISER bit 28 (0800d3f4) |
| 18 | usart_enable 0800fd6e(USART1) | CTL0 | \|= 1: **UEN set, near the end** (after REN/TEN/IDLEIE/NVIC, before the DMA setup) |
| 19 | RX DMA setup 080086d4(1, 0x20000931, 0x200) | see section 2 | DENR is set after UEN |

Net CTL0 after step 18: UEN (bit0) \| REN (bit2) \| TEN (bit3) \| IDLEIE (bit4) = **0x0000001D**. CTL1 = 0 (STB 00, no LIN/clock). CTL2 = DENR (0x40) after step 19; RTSEN/CTSEN clear. **[V]** from the sequence (nothing else touches CTL0). The OVSMOD bit (CTL0 bit15) is never set; the baud code reads it (0800fc2c-0800fc32) and takes the 16x branch. **[V]**

GPIO net for USART1: PA2 = AF7, push-pull, 50 MHz, pull-up; PA3 = AF7, pull-up; AFSEL0 bits 15:8 = 0x77. **[V]** Matches `oem-hw-map.md` §2. DE pin PB10 is a separate output, push-pull, no pull, init low at 08008548 (init step 10). **[V]**

Note: there is **no** wait for stability or flag after UEN, and no flag clear before enabling. **[V]**

## 2. RX DMA (v1 080086d4, channel 1) [V]

Library calls, in order (ch = 1; the DMA channel number is 0-based, CH_x registers at 0x40020008 + 0x14·x):

| # | v1 call | register | operation |
|---|---|---|---|
| 1 | 0800f5ac(0x500) | RCU_AHBEN (0x40021014) | \|= bit0 (**DMAEN**) |
| 2 | dma_deinit 0800e21c(1) | CH1CTL (0x4002001C) = 0; CH1CNT (0x40020020) = 0; CH1PADDR (0x40020024) = 0; CH1MADDR (0x40020028) = 0; INTC (0x40020004) \|= 0xF << 4 | clears CHEN, resets, clears the channel's flags |
| 3 | dma_struct_para_init 0800e3d8 | (stack) | defaults |
| 4 | struct fields | (stack) | periph_addr = **0x40004424** (USART1 base + 0x24 = RDATA), periph_width = 0 (8 bit), memory_addr = **0x20000931**, memory_width = 0 (8 bit), number = **0x200 (512)**, priority = 0x3000 (ultra-high), periph_inc = 0, memory_inc = 1, direction = 0 (periph to mem), request = **0x34** |
| 5 | dma_init 0800e278(1, &struct) | CH1PADDR, CH1MADDR, CH1CNT; CH1CTL (mask 0x3F00 cleared, then \|= widths \| priority); CH1CTL bit7 (MNAGA) set, bit6 (PNAGA) cleared, bit4 (DIR) cleared; **DMAMUX_RM_CH1CFG (0x40020804)**: &= ~0x7F then \|= 0x34 | |
| 6 | dma_circulation_enable 0800e200(1) | CH1CTL | \|= 0x20 (**CMEN**) |
| 7 | dma_memory_to_memory_disable 0800e3b4(1) | CH1CTL | &= ~0x4000 (M2M) |
| 8 | dmamux_synchronization_disable 0800e450(1) | DMAMUX_RM_CH1CFG | &= ~0x10000 (SYNCEN) |
| 9 | usart_dma_receive_config 0800fd50(USART1, 0x40) | USART1_CTL2 (+0x08) | &= ~0x40 then \|= **0x40 (DENR, bit6)** |
| 10 | dma_channel_enable 0800e1e4(1) | CH1CTL | \|= 1 (**CHEN**) |

Final CH1CTL = 0x3000 (PRIO) \| 0x80 (MNAGA) \| 0x20 (CMEN) \| 0x01 (CHEN) = **0x000030A1**, matching the bench read (CTL 0x30A1, CNT 0x200, PADDR 0x40004424, MADDR 0x20000931). **[V]**
No DMA interrupt enables (TFTFIE/HTFIE/ERRIE all 0). **[V]**

DMAMUX: base 0x40020800, `DMAMUX_RM_CHxCFG` at 0x40020800 + 4·x, MUXID = bits 6:0, SYNCEN = bit16. **[V]** for the address, MUXID mask 0x7F and SYNCEN (consistent with the ADC note, request 5 on ch2). No other DMAMUX register (request generator, interrupt flags) is written anywhere. **[V]** There is no extra RCU bit for the DMAMUX (only AHBEN bit0 DMAEN). **[V]**

Request ID table seen in stock: ADC = 5 (ch2); USART1_RX = 0x34 (ch1); UART3_RX = 0x36 (ch4); UART4_RX = 0x38 (ch3). **[V]**

### How stock reads the received bytes [V]

Stock does **not** read RBNE and does **not** compute a read pointer from CNT. It treats the buffer as a one-frame buffer, restarted on every IDLE:

1. IRQ28 vector = 0x0800D1F9 → thunk 0800d1f8 → handler **0800d184**.
2. `usart_interrupt_flag_get(USART1, 0x07040004)` (0800fe74): tests CTL0.IDLEIE (bit4) and STAT.IDLEF (bit4). If clear, return.
3. `usart_interrupt_flag_clear(USART1, 0x07040004)` (0800fe34): writes **INTC (+0x20) \|= 1 << 4 (IDLEC)**. The code is `ldr [usart+0x20]; orr; str` for any flag other than the `0x340f3409` special case. **[V]** This confirms INTC at +0x20.
4. `len = 0x200 - DMA_CH1CNT` (0800e434 reads 0x40020020), stored to the halfword at 0x20000B32. (Nothing else ever reads 0x20000B32; the parser works on a zero-terminated buffer instead.) **[V]**
5. Byte at 0x20000930 = 1 (frame-ready flag).
6. memcpy(0x20000630, 0x20000931, 0x200) (080051d8) then memset(0x20000931, 0, 0x200) (0800520a).
7. DMA restart: dma_channel_disable(1) (0800e1c8), `CH1CNT = 0x200` (0800e414), dma_channel_enable(1). MADDR is not rewritten (only CNT is). Whether the hardware reloads MADDR from its shadow on the next enable is not shown by the code. See section 5 item 2.

Consumer: 0x08006ca8 region (main-loop Pylon task, decompile line ~1736) clears 0x20000930, then parses the 0x20000630 copy. **[V]**

## 3. TX, DE, flags and interrupts

### 3.1 Transmit (polled) [V]

`08008ab8(1, byte)` (also `fputc` 0800e69c → this, so `printf` goes out USART1):

```
wait until STAT.TBE (flag 0x707, STAT +0x1C bit 7) is set
USART1_TDATA (+0x28) = byte & 0x1FF
wait until STAT.TC  (flag 0x706, STAT +0x1C bit 6) is set
```

So stock waits for **TC after every byte**, not just the last. No timeout on either wait. No TX interrupt, no TX DMA (CTL2.DENT never set). **[V]**

### 3.2 DE (PB10) timing, Pylon frame TX at 0800982c [V]

```
GPIOB_BOP (0x48000418) = 0x400        ; 0800e8a4  DE high  (PB10 set)
0800839c(1)                            ; spin until the 1 ms tick counter (0x20000C40) reaches 0
printf(...) -> fputc -> 08008ab8(1, c) for each byte   ; TBE, TDATA, TC per byte
GPIOB_BC (0x48000428) = 0x400         ; 0800e894  DE low   (right after the last TC)
```

- **DE active high.** DE is reset low at init and raised only for TX. **[V]**
- High to first byte: a delay of one tick of the 1 ms counter (`0800839c` loads 1, spins until the TIMER1 ISR decrements it to 0; so between 0 and 1 ms, effectively up to 1 ms). **[V]**
- Low after the last byte's TC; no turnaround delay after TC. **[V]**
- Receiver: stock does nothing to the RX path while DE is high (REN stays on, and the half-duplex RS485 chip echo is not handled). Because /RE is presumably tied to DE the receiver is deaf during TX. **[I]**
- Same sequence (DE high, delay, polled TX, DE low) is used for UART4 replies (080069ac). The one PB10 serves both ports. **[V]** per `oem-init-sequences.md` §5.

### 3.3 STAT bit positions (offset +0x1C) [V]/[I]

| bit | name | tag | evidence |
|---|---|---|---|
| 7 | TBE | **[V]** | flag ID 0x707 in 08008ab8 |
| 6 | TC | **[V]** | flag ID 0x706 in 08008ab8 |
| 5 | RBNE | [I] | GD32L23x layout (same as F3x0/E23); stock never tests it |
| 4 | IDLEF | **[V]** | int flag 0x07040004 in 0800d184 |
| 3 | ORERR | [I] | stock never tests it |
| 2 | NERR | [I] | stock never tests it |
| 1 | FERR | [I] | stock never tests it |
| 0 | PERR | [I] | stock never tests it |

Bench: UART4 STAT idle = 0x002000C0 = bit7 TBE + bit6 TC + bit21 (REA, receive enable acknowledge). **[V]** bench. So on L23x STAT also has REA (bit21) and TEA (bit21/bit20 on the newer parts); do not treat STAT == 0 as "idle".

### 3.4 INTC (flag clear, +0x20) [V]/[I]

INTC is at **+0x20** [V] (the stock idle clear is a write of `1<<4` there, ORed into the register).
Bit meaning, by the GD32L23x layout: PERRC bit0, FERRC bit1, NERRC bit2, ORERRC bit3, **IDLEC bit4 [V]**, TCC bit6, LBDC bit8, CTSC bit9, RTC bit11, EBC bit12, CMC bit17, WUC bit20. All except IDLEC are **[I]**. TC also clears by a TDATA write following a STAT read; the ORERR flag is only cleared via INTC (ORERRC). Stock clears none of them.

### 3.5 CTL0 bit positions [V]/[I]

| bit | name | tag |
|---|---|---|
| 0 | UEN | **[V]** (cleared first and set last by the library functions) |
| 2 | REN | **[V]** (flag value 4 in 0800ff00) |
| 3 | TEN | **[V]** (0800ff48) |
| 4 | IDLEIE | **[V]** (0800fe10 value 0x004) |
| 5 | RBNEIE | [I] |
| 6 | TCIE | [I] |
| 7 | TBEIE | [I] |
| 8 | PERRIE | [I] |
| 9 / 10 | PM / PCEN | **[V]** (mask 0x600 in 0800fed6) |
| 12 | WL | **[V]** (mask 0x1000 in 0800ff66) |
| 15 | OVSMOD | **[V]** (tested in 0800fc2c) |

CTL1: STB = bits 13:12 **[V]** (mask 0x3000 in 0800ff1e). CTL2: RTSEN bit8, CTSEN bit9 **[V]** (masks in 0800fde6/0800fdbc), **DENR bit6 [V]** (0800fd50 with 0x40), DENT bit7 [I].
Other registers on this block: BAUD +0x0C **[V]**, STAT +0x1C **[V]**, INTC +0x20 **[V]**, RDATA +0x24 **[V]** (RX DMA PADDR = base + 0x24 = 0x40004424), TDATA +0x28 **[V]**.

### 3.6 NVIC [V]

`nvic_irq_enable(28, 0)` (0800ea28): priority value 0 stored in the NVIC IPR byte (`value << 6`, so the top 2 bits are 00 = highest), then ISER (0xE000E100) bit 28. Vector 28 (0x08005000 + 4·(16+28) → 0x0800D1F9) = USART1. The sub-priority argument is ignored on this core. **[V]**
Stock priorities are not changed elsewhere for IRQ28. (The priority group register is not touched by the call.) **[I]** for the global grouping.

## 4. Baud values (30 MHz APB1) [V]/[I]

Formula from `usart_baudrate_set` (0800fbc4), `clk` from `rcu_clock_freq_get(CK_APB1)` (0800eb00(2); bench: APB1 = 30 MHz because UART4 BAUD = 0x41 at 460800 **[V]**):

- OVSMOD = 0 (16x): `udiv = (clk + baud/2) / baud`; `BAUD = (udiv & 0xFFF0) | (udiv & 0xF)` which is just `udiv` (INTDIV = udiv[15:4], FRADIV = udiv[3:0]). **[V]**
- OVSMOD = 1 (8x, not used by stock): `udiv = (2·clk + baud/2) / baud`; `BAUD = (udiv & 0xFFF0) | ((udiv & 0xF) >> 1)`. **[V]** code; unused.

| baud | BAUD register | actual rate | error |
|---|---|---|---|
| 9600 | **0x0C35** (3125) | 9600.0 | 0.00 % |
| 19200 | 0x061B (1563) | 19193.9 | -0.03 % |
| 38400 | 0x030D (781) | 38412.3 | +0.03 % |
| 115200 | 0x0104 (260) | 115384.6 | +0.16 % |
| 460800 | 0x0041 (65) | 461538 | +0.16 % (matches the bench UART4 read of 0x41) |

9600 check against stock: `(30000000 + 4800) / 9600 = 3125.5`, integer division gives 3125 = 0x0C35. Stock USART1 is initialised with `param_2 = 9600` and the same function produces UART4's 0x41 at 460800, so the formula and the 30 MHz clock are **[V]**. The USART1 BAUD register itself was not read on the bench (only UART4's was), so 0x0C35 for USART1 is computed, not measured. **[I]** for the register value.

## 5. Things that contradict or differ from the plan's assumptions

1. **RX DMA is circular-bit-set but stock restarts it on every IDLE** (disable, CNT = 0x200, enable) and copies and clears the buffer. Stock never uses CNT to chase a write pointer. A Modbus driver that wants a free-running circular buffer with CNT arithmetic will not be copying stock's code, only its register values. Both approaches work with CH1CTL = 0x30A1.
2. **MADDR after a disable/reload cycle.** Stock rewrites only CNT before re-enabling, never MADDR. Whether the next frame lands at 0x20000931 again or continues past where DMA stopped depends on DMA hardware behaviour the code does not show. **[I]** Check on the bench by reading CH1MADDR after a received frame. A driver of ours should rewrite MADDR and CNT together while the channel is disabled.
3. **DE is active high.** No contradiction.
4. **Stock TX is polled with TC after every byte**, not TX by DMA, and DE is never driven by hardware (the USART DEM bit is never set). Pure software GPIO DE, so the Modbus driver needs its own turnaround handling.
5. **No STAT error handling at all** in stock USART1; ORERR, FERR and NERR flags and their INTC clears are [I] only.
6. **PA3 has no OSPD/OMODE write** and PA2 also gets a pull-up although it is a TX output; harmless, mirrored in the plan.
7. **The DMA is 0-based**: "DMA ch1" here is CH1 at 0x4002001C, DMAMUX_RM_CH1CFG at 0x40020804.

## 6. Open questions

1. USART1 TX request ID (0x35 by pattern) is unconfirmed, because stock never uses TX DMA.
2. STAT RBNE/ORERR/NERR/FERR bit positions and the INTC bit map (other than IDLEC) are from the layout, not from stock code. Verify against the GD32L23x user manual before relying on them.
3. Whether an idle 1-ms-class delay between DE high and the first byte is needed by the transceiver or is only a stock habit.
4. Whether the DMAMUX request-line 0x34 stays assigned after a USART reset (stock writes it each init, so we do too).

## 7. Checked against the vendor header (2026-10-15)

GigaDevice's GD32L23x standard peripheral library (`gd32l23x_usart.h`, `gd32l23x_dma.h`, `gd32l23x_rcu.h`, `gd32l23x.h`) confirms
every bit above that stock never exercised, so the **[I]** tags in §3 for these now read **[V, vendor header]**:

- **CTL0:** UEN 0, REN 2, TEN 3, IDLEIE 4, RBNEIE 5 (also the overrun interrupt), TCIE 6, TBEIE 7, OVSMOD 15.
- **CTL2:** DENR 6, DENT 7, OVRD 12.
- **STAT:** PERR 0, FERR 1, NERR 2, ORERR 3, IDLEF 4, RBNE 5, TC 6, TBE 7, BSY 16, TEA 21, REA 22. The bench's idle
  0x002000C0 is TEA | TBE | TC.
- **INTC (+0x20):** PEC 0, FEC 1, NEC 2, OREC 3, IDLEC 4, TCC 6.
- **USART1:** base 0x40004400; APB1EN / APB1RST bit 17; IRQ 28.
- **DMAMUX:** base 0x40020800, `RM_CHxCFG` at +4·x, MUXID bits 5:0. USART1_RX request 52 (0x34), USART1_TX request 53 (0x35).
- **DMA CHxCTL:** CHEN 0, CMEN 5 (circular), MNAGA 7, PRIO 13:12.

Notes for the driver (Task 4):
- Stock drops DE straight after TC with no turnaround delay. Since TC means the stop bit has been shifted out, that is safe at any
  baud.
- Whether the receiver is deaf while DE is high is unknown. The Modbus task discards RX from the start of a reply until one frame
  gap after it (plan M4), whichever way the transceiver is wired.
