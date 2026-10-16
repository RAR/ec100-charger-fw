# OEM v1 app: literal register-level ADC and FMC (internal flash) usage

Companion to `docs/history/notes/oem-init-sequences.md` and `docs/history/notes/oem-can-sequences.md`. Same conventions:
source is `dumps/gd32_flash_20261012.bin` (base 0x08000000), disassembled with
`arm-none-eabi-objdump -D -b binary -marm -Mforce-thumb --adjust-vma=0x08000000`; literal pools resolved from the binary.
Our drivers: `oem/src/hw/adc.cpp` follows §4 and `oem/src/hw/flash.cpp` follows §5 (phase-2 plan,
`docs/history/superpowers/plans/2026-10-15-oem-firmware-phase2-charging.md`).
Tags: **[V]** = verified in code / literals. **[I]** = inferred (datasheet, the GigaDevice GD32L23x firmware library V1.0.0, which isn't in the repo, or reasoning).

The ADC and DMA code is the GD32L23x standard library compiled -O0; every function body matches `gd32l23x_adc.c` / `gd32l23x_dma.c` /
`gd32l23x_rcu.c` V1.0.0 line for line. **[V]** The FMC code is a **newer** GD lib than the V1.0.0 copy we have: it has a doubleword
program function and register-encoded flag IDs (`(reg_offset << 6) | bit`) that V1.0.0 lacks. Every address below is from a literal in the binary. **[V]**

## 0. Bottom line

- **ADC:** PC0 = ADC_IN10. ADC clock = **CK_APB2 / 12 = 5 MHz** (`rcu_adc_clock_config(5)`). 12-bit (DRES never written, so it stays at reset 00), right-aligned.
  10-rank **scan + continuous**, all ranks = channel 10, sample time **41.5 cycles**, software trigger once at the end of init, and then it free-runs
  forever into a **circular DMA CH2** buffer of 10 × u16 at **0x20000010**. **[V]**
- **No code ever reads ADC_RDATA, STAT or EOC.** The only consumer is the main loop (0800b630 → 08007630), which takes a snapshot of the buffer, sorts it, drops min and max, and averages the middle 8 (`sum >> 3`).
  It then computes `(u16)(avg × 0.16923f)` (truncated) → **ST+0x16 = 0x200005EE** (in v1; v2 uses +0x1C). **[V]**
- **DMA is CH2, not CH0** (DMAMUX request 5 = ADC on L23x). CH2CTL final = **0x000035A1**. **[V]**
- **FMC:** unlock with KEY 0x45670123 / 0xCDEF89AB, **1 KiB pages**, page erase = PER, ADDR, START, wait, clear PER. Doubleword program =
  **PG only (no separate doubleword-enable bit)**, then two 32-bit stores to addr and addr+4, wait, clear PG. Ready-wait timeout = **0x00100000 polls**. Lock = CTL |= LK (0x80). **[V]**
- **Stock's flag clear is broken:** the app (and the bootloader's erase path) call `fmc_flag_clear(0x307)`. With the encoded flag IDs that is
  `FMC_STAT = 1 << 7` = **0x00000080**, which clears **none** of PGERR/PGAERR/WPERR/ENDF. 0x307 is evidently `END|WPERR|PGAERR|PGERR`
  (0x305|0x304|0x303|0x302) OR'ed together F10x-style. The bootloader's *program* path does it right: four separate clears, 0x305, 0x304, 0x303, 0x302. **[V]**
  Bit 7 is undefined in the V1.0 header, so writing it should do nothing. **[I]**
- IRQs: `cpsid i` / `cpsie i` around erase-range (08005ebc) and append (08006018); **not** around the page-erase used by 0800c900. **ISB** after each
  doubleword program and after the anti-clone erase. **No DSB** anywhere in the flash paths. **[V]**

v1 function map **[V]**:

| v1 addr | GD lib equivalent / role | notes |
|---|---|---|
| 080080f8 | (app) ADC DMA setup (buf, n) | called (0x20000010, 10) from init 0800b50c @0800b580 |
| 0800816c | (app) ADC bring-up | called right after, @0800b584 |
| 0800f5ac | rcu_periph_clock_enable(x) | reg = 0x40021000 + (x >> 6), bit = x & 31 |
| 0800ea90 | rcu_adc_clock_config(ck) | |
| 0800e8ee | gpio_mode_set(port, mode, pull, pin) | |
| 0800d5a4 | adc_channel_length_config(group, len) | |
| 0800d830 | adc_regular_channel_config(rank, ch, spt) | |
| 0800d724 | adc_external_trigger_source_config(group, src) | |
| 0800d648 | adc_data_alignment_config(align) | |
| 0800d9bc | adc_special_function_config(fn, en) | |
| 0800d6a8 | adc_external_trigger_config(group, en) | |
| 0800d674 | adc_dma_mode_enable() | |
| 0800d62c | adc_charge_pulse_width_counter(n) → ADC_CCTL | only from adc_enable |
| 0800d684 | adc_enable() | |
| 0800839c | (app) delay_ms(n) | n → 0x20000C40, spins until the SysTick ISR counts it to 0 |
| 0800d56c | adc_calibration_enable() | |
| 0800d97c | adc_software_trigger_enable(group) | |
| 0800d778 / 0800d78c | adc_interrupt_flag_clear / _get | only in the IRQ18 handler 08005AF4, which is never enabled |
| 0800e21c, 0800e3d8, 0800e278, 0800e200, 0800e3b4, 0800e450, 0800e1e4, 0800e1c8 | dma_deinit, dma_struct_para_init, dma_init, dma_circulation_enable, dma_memory_to_memory_disable, dmamux_synchronization_disable, dma_channel_enable, dma_channel_disable | |
| 08007630 | (app) ADC → ST+0x16 | from main loop 0800b630 every pass |
| 08007668 | (app) sort-and-trim mean of 10 u16 | |
| 0800e670 | fmc_unlock | |
| 0800e4c8 | fmc_flag_clear(encoded) | |
| 0800e538 | fmc_page_erase(addr) | |
| 0800e470 | fmc_doubleword_program(addr, lo, hi) | args r0 = addr, **r2 = lo, r3 = hi** (u64 in r2:r3, AAPCS) |
| 0800e52c | fmc_lock | |
| 0800e594 | fmc_ready_wait(timeout) | |
| 0800e5f0 | fmc_state_get | |
| 08005e44 | (app) erase log region (ctx) | |
| 08005ebc | (app) erase range [a, b], IRQs off | |
| 08005f28 | (app) log scan / ctx init | |
| 08005fc6 | (app) log read | |
| 08006018 | (app) log append | |

---

## 1. ADC clock, GPIO, DMA (v1 080080f8 and the first part of 0800816c)

### 1a. RCU + GPIO [V]

| # | v1 addr | register (addr) | operation |
|---|---|---|---|
| 1 | 0800f5ac(0x500) @08008104 | RCU_AHBEN (0x40021014) | \|= bit0 **DMAEN** |
| 2 | 0800f5ac(0x609) @08008172 | RCU_APB2EN (0x40021018) | \|= bit9 **ADCEN** |
| 3 | 0800f5ac(0x513) @08008178 | RCU_AHBEN | \|= bit19 **PCEN** |
| 4 | 0800e8ee(GPIOC 0x48000800, mode 3, pull 0, pin 0x1) | GPIOC_CTL (0x48000800) / PUD (0x4800080C) | CTL bits1:0 = **11 (analog)**; PUD bits1:0 = 00 |
| 5 | 0800ea90(**5**) @08008192 | RCU_CFG0 (0x40021004) | &= ~0x0000C000 (ADCPSC[1:0]) |
| 6 | | RCU_CFG2 (0x40021030) | &= 0x3FFFFEFF (clears ADCPSC2 bits31:30 and ADCSEL bit8) |
| 7 | | RCU_CFG0 | \|= (5 & 3) << 14 = **0x00004000** |
| 8 | | RCU_CFG2 | \|= ((5 & 0xC) >> 2) << 30 = **0x40000000** |
| 9 | | RCU_CFG2 | \|= **0x00000100 (ADCSEL = 1: APB2/AHB-divided source, not IRC16M)** |

- `ck_adc = 5` = `RCU_ADCCK_APB2_DIV12` in the lib header → **CK_ADC = 60 MHz / 12 = 5 MHz**. **[V]** for the code and the constant; that the hardware
  decodes {ADCPSC2, ADCPSC} = 0101 as APB2/12 is from the lib's enum, so **[I]**.
- DS limit: f_ADC 0.14 to **16 MHz** max; t_CONV(12-bit) = 15 cycles (2.5-cycle sample + 12.5), t_ST(ADC) startup 5 µs (DS Table 4-33). **[V vs ds/l235.txt]**
  (So the ~14 MHz F1-style limit does not apply; L235 is 16 MHz.) Stock runs well below that.
- Conversion = (41.5 + 12.5) / 5 MHz = **10.8 µs per sample**, so the whole 10-sample buffer is rewritten about every 108 µs. **[I]** (arithmetic)

### 1b. DMA CH2 (080080f8, buf = 0x20000010, n = 10) [V]

Struct passed to dma_init: periph_addr = **0x4001244C (ADC_RDATA)**, periph_width = 0x100, memory_addr = 0x20000010, memory_width = 0x400,
number = 10, priority = 0x3000, periph_inc = 0, memory_inc = 1, direction = 0 (periph→mem), request = **5 (DMA_REQUEST_ADC)**.

| # | lib call | register (addr) | operation |
|---|---|---|---|
| 1 | dma_deinit(2) | DMA_CH2CTL (0x40020030) | &= ~1 (CHEN), then = 0 |
| 2 | | DMA_CH2CNT (0x40020034), CH2PADDR (0x40020038), CH2MADDR (0x4002003C) | = 0 each |
| 3 | | DMA_INTC (0x40020004) | \|= 0xF << (4·2) = **0x00000F00** (clear all CH2 flags) |
| 4 | dma_init(2, &s) | DMA_CH2CTL | &= ~1 (dma_channel_disable) |
| 5 | | CH2PADDR | = **0x4001244C** |
| 6 | | CH2MADDR | = **0x20000010** |
| 7 | | CH2CNT | = **10** |
| 8 | | CH2CTL | = (CTL & ~0x3F00) \| 0x100 \| 0x400 \| 0x3000 (PWIDTH 16, MWIDTH 16, PRIO ultra-high) |
| 9 | | CH2CTL | &= ~0x40 (PNAGA = 0); \|= 0x80 (**MNAGA = 1**); &= ~0x10 (DIR = 0, periph→mem) |
| 10 | | DMAMUX_RM_CH2CFG (0x40020808) | &= ~0x7F; \|= **5** (MUXID = ADC) |
| 11 | dma_circulation_enable(2) | CH2CTL | \|= 0x20 (**CMEN**) |
| 12 | dma_memory_to_memory_disable(2) | CH2CTL | &= ~0x4000 |
| 13 | dmamux_synchronization_disable(2) | DMAMUX_RM_CH2CFG | &= ~0x10000 (SYNCEN = 0) |
| 14 | dma_channel_enable(2) | CH2CTL | \|= 0x1 (**CHEN**) |

**Final CH2CTL = 0x000035A1** (CHEN | CMEN | MNAGA | PWIDTH = 16 | MWIDTH = 16 | PRIO = 11). No DMA interrupts (FTFIE/HTFIE/ERRIE = 0). The
address formula used by the code is CHxCTL = 0x40020008 + 0x14·x, CNT = 0x4002000C + 0x14·x, PADDR = 0x40020010 + 0x14·x, MADDR = 0x40020014 + 0x14·x,
DMAMUX CHxCFG = 0x40020800 + 4·x. **[V]**

---

## 2. ADC init table (0800816c, after §1a) [V]

ADC base 0x40012400 (literals: STAT 0x40012400, CTL0 0x40012404, CTL1 0x40012408, SAMPT0 0x4001240C, SAMPT1 0x40012410, RSQ0 0x4001242C,
RSQ1 0x40012430, RSQ2 0x40012434, ISQ 0x40012438, RDATA 0x4001244C, CCTL 0x400124C0). **[V]**

| # | v1 call | register (addr) | operation |
|---|---|---|---|
| 1 | 0800d5a4(1 REGULAR, 10) | ADC_RSQ0 (0x4001242C) | &= ~0x00F00000; \|= (10−1) << 20 → **RL = 9 (10 conversions)** |
| 2–11 | 0800d830(rank r = 0..9, ch 10, spt 4) | ranks 0–5 → ADC_RSQ2 (0x40012434) field 5·r; ranks 6–9 → ADC_RSQ1 (0x40012430) field 5·(r−6) | each RMW: &= ~(0x1F << f); \|= 10 << f |
| | same calls | ADC_SAMPT0 (0x4001240C) (ch ≥ 10: field 3·(ch−10)) | &= ~0x7; \|= **4** (41.5 cycles), rewritten 10× with the same value |
| 12 | 0800d724(1, 0x000E0000) | ADC_CTL1 (0x40012408) | &= ~0x000E0000; \|= 0x000E0000 → **ETSRC = 111 (software trigger SWRCST)** |
| 13 | 0800d648(0) | ADC_CTL1 | &= ~0x800 (**DAL = 0, right-aligned**) |
| 14 | 0800d9bc(0x100 SCAN, 1) | ADC_CTL0 (0x40012404) | \|= 0x100 (**SM**) |
| 15 | 0800d9bc(0x002 CONTINUOUS, 1) | ADC_CTL1 | \|= 0x2 (**CTN**) |
| 16 | 0800d6a8(1 REGULAR, 1) | ADC_CTL1 | \|= 0x00100000 (**ETERC**) |
| 17 | 0800d674() | ADC_CTL1 | \|= 0x100 (**DMA**) |
| 18 | 0800d684() → 0800d62c(320) | ADC_CCTL (0x400124C0) | = 320 & 0xFFF = **0x140** (charge pulse width counter) |
| 19 | 0800d684() | ADC_CTL1 | if (CTL1 & 1) == 0: \|= 0x1 (**ADCON**) |
| 20 | 0800839c(1) | — | 1 ms SysTick delay (≥ t_ST = 5 µs) |
| 21 | 0800d56c() | ADC_CTL1 | \|= 0x8 (**RSTCLB**); **while (CTL1 & 0x8)**; no timeout |
| 22 | | ADC_CTL1 | \|= 0x4 (**CLB**); **while (CTL1 & 0x4)**; no timeout |
| 23 | 0800d97c(1 REGULAR) | ADC_CTL1 | \|= 0x00400000 (**SWRCST**) → conversions start and never stop (CTN) |

Final values **[V]** (assuming reset = 0 for untouched bits, **[I]**):
- **RSQ0 = 0x00900000**, **RSQ1 = 0x0005294A**, **RSQ2 = 0x14A5294A** (channel 10 in all 10 ranks).
- **SAMPT0 = 0x00000004** (ch10 = 41.5 cycles), SAMPT1 untouched (0).
- **CTL0 = 0x00000100** (SM only). DRES (bits 25:24) is never written, so the ADC is **12-bit [V that it isn't written; I that the reset value is 00 = 12-bit]**. No EOCIE, no watchdog, no discontinuous mode.
- **CTL1 = 0x001E0103** after init (ADCON | CTN | DMA | ETSRC = 7 | ETERC), plus SWRCST (bit22) as written; on GD parts hardware clears SWRCST when the conversion starts. **[I]**
- OVSAMPCTL, IOFFx, WDHT/WDLT, ISQ, TSVEN/INREFEN are never written. **[V]** (no literal for 0x40012480 anywhere)
- **Order note:** stock sets ADCON *after* all the configuration, then waits 1 ms, calibrates, and only then sets SWRCST. **[V]**
- IRQ18 (ADC) has a handler at 08005AF4 that uses adc_interrupt_flag_get/clear (STAT 0x40012400, CTL0 0x40012404), but it is never enabled in the NVIC and EOCIE is never set. **[V]**

---

## 3. ADC read / average path [V]

Main loop 0800b630 calls **08007630 every pass** (no rate limit):

```
u16 avg = f_08007668((u16*)0x20000010);        // the live circular-DMA buffer, no DMA pause, no copy lock
ST.u16[+0x16] = (u16)__aeabi_f2uiz(__aeabi_ui2f(avg) * 0.16923f);   // 0x200005D8 + 0x16 = 0x200005EE
```
- Scale literal @08007664 = **0x3E2D4AA1 = 0.16923000f**. 0800540a = ui2f, 080052d2 = fmul, 08005418 = f2uiz (it **truncates**). **[V]**
- Full scale: 4095 × 0.16923 = 693.0 → 0.1 V units → 69.3 V (agrees with oem-hw-map.md). **[V]** arithmetic.
- In v2 the same value goes to ST+0x1C (0800c964); in v1 it is **+0x16**. **[V]**

`08007668(buf)`:
1. copy buf[0..9] (u16) to a local array;
2. bubble sort ascending (outer i = 0..8, inner j < 9 − i, swap if a[j] > a[j+1]);
3. `sum` (u16) = a[1] + … + a[8] (drops the min and the max);
4. return **sum >> 3**.

Snapshot coherence: the DMA keeps writing while the copy loop runs, so the 10 samples can be from two adjacent sweeps. That doesn't matter for a DC level. **[I]**

---

## 4. Proposed polled single-conversion sequence (no DMA)

Constants: ADC = 0x40012400, RCU = 0x40021000. "[V]" = the same register/value stock writes. "[I]" = our change or from the manual.

**One-time init**

| # | register (addr) | operation | tag |
|---|---|---|---|
| 1 | RCU_APB2EN (0x40021018) | \|= 0x200 (ADCEN) | [V] |
| 2 | RCU_AHBEN (0x40021014) | \|= 0x80000 (PCEN) | [V] |
| 3 | GPIOC_CTL (0x48000800) | \|= 0x3 (PC0 analog); GPIOC_PUD (0x4800080C) &= ~0x3 | [V] |
| 4 | RCU_CFG0 (0x40021004) | = (CFG0 & ~0xC000) \| 0x4000 | [V] |
| 5 | RCU_CFG2 (0x40021030) | = (CFG2 & 0x3FFFFEFF) \| 0x40000100 → CK_ADC = APB2/12 = 5 MHz | [V] |
| 6 | ADC_RSQ0 (0x4001242C) | = 0x00000000 (RL = 0, one conversion) | [I] (stock: 0x00900000) |
| 7 | ADC_RSQ2 (0x40012434) | = 0x0000000A (rank 0 = ch10) | [V] field / [I] single rank |
| 8 | ADC_SAMPT0 (0x4001240C) | = (SAMPT0 & ~0x7) \| 0x4 (41.5 cycles) | [V] |
| 9 | ADC_CTL0 (0x40012404) | = 0 (no SM; 12-bit DRES = 0) | [I] (stock: 0x100) |
| 10 | ADC_CTL1 (0x40012408) | = 0x001E0000 (ETSRC = 7 SW, ETERC = 1, CTN = 0, DMA = 0, DAL = 0) | [V] ETSRC/ETERC; [I] CTN/DMA off |
| 11 | ADC_CCTL (0x400124C0) | = 0x140 | [V] |
| 12 | ADC_CTL1 | \|= 0x1 (ADCON) | [V] |
| 13 | — | wait ≥ 5 µs (stock: 1 ms) | [V] stock / DS t_ST |
| 14 | ADC_CTL1 | \|= 0x8; while (CTL1 & 0x8) {} (RSTCLB), with a timeout we add | [V] (+[I] timeout) |
| 15 | ADC_CTL1 | \|= 0x4; while (CTL1 & 0x4) {} (CLB), with a timeout; DS t_CAL ≤ 68.75 µs at 16 MHz | [V] (+[I] timeout) |

**Per sample**

| # | register | operation | tag |
|---|---|---|---|
| 1 | ADC_STAT (0x40012400) | &= ~0x2 (clear EOC; the lib's adc_flag_clear does STAT &= ~flag, as in 0800d778) | [V] method / [I] that it's needed |
| 2 | ADC_CTL1 (0x40012408) | \|= 0x00400000 (SWRCST) | [V] |
| 3 | ADC_STAT | while (!(STAT & 0x2)) {} with a timeout (expect about 10.8 µs) | [I] (stock never polls EOC) |
| 4 | ADC_RDATA (0x4001244C) | v = RDATA & 0xFFF (reading it clears EOC) | [V] address (DMA source) / [I] clear-on-read |

Then do 10 samples → sort → mean of the middle 8 (>> 3) → × 0.16923 → truncate, to match stock exactly. **[V]**
Notes: SWRCST needs ETERC = 1 with ETSRC = 111 on this ADC family (stock sets both) **[I/V]**. If CTN were left on, one SWRCST would free-run, and RDATA would be the latest value.
That also works for polling, but then EOC is only meaningful once per conversion. Single mode is simpler. **[I]**

---

## 5. FMC

FMC base 0x40022000. Literals used: WS 0x40022000, KEY **0x40022004**, STAT **0x4002200C**, CTL **0x40022010**, ADDR **0x40022014**. OBKEY/OBSTAT are never referenced. **[V]**
Bits (match the L23x header): STAT BUSY b0, PGERR b2, PGAERR b3, WPERR b4, ENDF b5; CTL PG b0, PER b1, START b6, LK b7. **[V]** from the code's use; names **[I]**.

### 5a. Unlock — fmc_unlock 0800e670 [V]

| # | register | operation |
|---|---|---|
| 1 | FMC_CTL (0x40022010) | read; **only if bit7 LK = 1**: |
| 2 | FMC_KEY (0x40022004) | = **0x45670123** |
| 3 | FMC_KEY | = **0xCDEF89AB** |

### 5b. Flag clear — fmc_flag_clear 0800e4c8(flag) [V]

`reg = 0x40022000 + (flag >> 6); bit = flag & 0x1F`. If `(flag >> 6) == 0x18`: reg = (reg & ~0x7F) | (1 << bit) (not used). Else **reg = 1 << bit** (plain write).
- App (08005e72, 08005ee8, 08006060, 0800b534) and boot erase (080005a0): flag = **0x307** (literal @080060ac / @0800b5f8 / @080005e0) → **FMC_STAT = 0x00000080**. None of the real flags (bits 0, 2–5) is cleared.
- Boot program path 08001c08: flag_clear(**0x305**) → STAT = 0x20 (ENDF); (**0x304**) → 0x10 (WPERR); (**0x303**) → 0x08 (PGAERR); (**0x302**) → 0x04 (PGERR). This is the correct form, and it proves the encoding.
- Write-1-to-clear for STAT flags is consistent with this; no RMW. **[I]** from L23x UM convention + code.

### 5c. Ready wait / state — 0800e594(timeout) + 0800e5f0 [V]

```
state = 1 (BUSY);
do { state = fmc_state_get(); timeout--; } while (state == BUSY && timeout != 0);
if (state == BUSY) state = 5 (TOERR);
return state;
```
fmc_state_get reads FMC_STAT (0x4002200C), checked in this priority order: bit0 BUSY → 1; else bit4 WPERR → **4**; else bit2 PGERR → **2**; else bit3 PGAERR → **3**;
else if STAT bits 15:14 ≠ 0 → **7** (meaning unknown; not in the V1.0 header) **[V]** code / **[I]** meaning; else 0 READY.
**Timeout argument is always 0x00100000** (1,048,576 polls; roughly 0.5–1 s at 60 MHz -O0 **[I]**). ENDF is never examined. The boot copy (08001c9c / 08001d78) is byte-identical. **[V]**
Because stock never clears error flags (5b), a leftover PGERR/WPERR/PGAERR would make every later ready-wait return an error and skip the operation. **[I]**

### 5d. Page erase — fmc_page_erase 0800e538(addr) [V]

| # | register | operation | wait |
|---|---|---|---|
| 1 | | | ready_wait(0x100000); if ≠ 0 → return it (nothing written) |
| 2 | FMC_CTL (0x40022010) | \|= 0x2 (**PER**) | |
| 3 | FMC_ADDR (0x40022014) | = addr (page base) | |
| 4 | FMC_CTL | \|= 0x40 (**START**) | |
| 5 | | | ready_wait(0x100000) |
| 6 | FMC_CTL | &= ~0x2 (clear PER), even after an error/timeout | return state |

Page size is **1 KiB**: callers step the address by 0x400 and compute page = (addr − 0x08000000) >> 10 (`addr + 0xF8000000 >> 10`). **[V]**

### 5e. Doubleword program — 0800e470(addr, u64 data in r2:r3) [V]

| # | register | operation | wait |
|---|---|---|---|
| 1 | | | ready_wait(0x100000); if ≠ 0 → return it |
| 2 | FMC_CTL | \|= 0x1 (**PG**) (no other enable bit) | |
| 3 | *(u32*)addr | = lo (r2) | |
| 4 | *(u32*)(addr + 4) | = hi (r3) | |
| 5 | | | ready_wait(0x100000) |
| 6 | FMC_CTL | &= ~0x1 (clear PG) | return state |

There is no DWPGE/FSTPG bit: CTL bit8 FSTPG (fast program) is never set. "Doubleword" is just PG with two consecutive aligned word stores. Whether the L235 needs both words (64-bit ECC granule) or would accept single words is not shown by stock. **[I]**
Boot's 08001b4c is the same code. **[V]**

### 5f. Lock — fmc_lock 0800e52c [V]

FMC_CTL (0x40022010) \|= **0x80 (LK)**.

### 5g. App flash routines [V]

Log context (struct @**0x20000084**): +0 cur (address of the last record), +4 start, +8 end, +12 n (u8; record = n doublewords). Init (0800b5cc):
`08005f28(0x20000084, start 0x0801F000, end 0x0801F3FF, n 2)`, i.e. a **single 1 KiB page at 0x0801F000 with 16-byte records**.

**08005f28 scan:** p = start; while (p + 8n ≤ end) { if the first word of each of the n doublewords at p, p+8, … is 0xFFFFFFFF → break; p += 8n }; cur = p − 8n.
(It only checks the low word of each doubleword.)

**08005e44 erase region(ctx):** pages (start−0x08000000)>>10 … (end−0x08000000)>>10 (here just page 0x7C): unlock; flag_clear(0x307); for each page fmc_page_erase(addr), addr += 0x400;
lock; cur = start − 8n. **No cpsid inside.** Callers: 08006018 (IRQs already off) and **0800c900 (IRQs on)**. Return codes are ignored.

**08005ebc erase range(a, b):** **cpsid i**; unlock; flag_clear(0x307); for page (a−0x08000000)>>10 … (b−0x08000000)>>10: fmc_page_erase(addr), addr += 0x400; lock; **cpsie i**.
Its only caller is the anti-clone path in init (0800b52e…0800b550): if the UID check 0800742c fails → unlock; flag_clear(0x307); **08005ebc(0x08005000, 0x08010F10) twice**
(both calls get the same r0/r1; Ghidra drops the args); **ISB**; lock. That erases the running app itself.

**08006018 append(ctx, src):**
```
cur += 8n;
cpsid i;
if (cur + 8n > end) { 08005e44(ctx); cur += 8n; }          // wrap: erase page, restart at start
fmc_unlock(); fmc_flag_clear(0x307);
for i in 0..n-1: { 0800e470(cur + 8i, src[2i], src[2i+1]); ISB; }   // results ignored, no read-back verify
fmc_lock();
cpsie i;
```
Its only caller is 08005df4 (it packs ST+3, +4, +0x24, +0x28, +0x12, +0x14, +1 into 16 bytes), triggered from the main loop 0800b630 when byte ST+0 (0x200005D8, literal @0800b684) ≠ 0, which it then clears.

---

## 6. Bootloader cross-check [V]

- **08000574 erase(a, b):** cpsid i; unlock (08001d78, same KEY values 0x45670123/0xCDEF89AB at 0x40022004); flag_clear(**0x307**) (08001ba4, same code);
  page loop with fmc_page_erase (08001c40: PER 0x2, ADDR 0x40022014, START 0x40, ready_wait(0x100000), clear PER); lock (08001c34, \|= 0x80); cpsie i. Same as app 08005ebc.
- **08000a34 program(addr, buf, len):** pads to doublewords (len/8, rounded up) via a staging buffer @0x20000140; cpsid i; unlock; **08001c08 = clear 0x305, 0x304, 0x303, 0x302**;
  loop 08001b4c(addr + 8i, lo, hi) + **ISB**; lock; cpsie i; zero the staging buffer (32 doublewords). Return value [sp+4] is always 0.
- fmc_ready_wait/state_get in boot (08001c9c/08001d78 region) are byte-identical to the app's. **[V]**

---

## 7. Literal list [V]

| literal @ | value | meaning |
|---|---|---|
| 08008168 | 0x4001244C | ADC_RDATA (DMA source) |
| 08008250 | 0x00000609 | RCU APB2EN bit9 ADCEN |
| 08008540 | 0x00000513 | RCU AHBEN bit19 PCEN |
| 08008544 | 0x48000800 | GPIOC |
| (imm) 08008100 | 0x500 | RCU AHBEN bit0 DMAEN |
| 0800eaf4 / 0800eaf8 / 0800eafc | 0x40021004 / 0x40021030 / 0x3FFFFEFF | RCU_CFG0, RCU_CFG2, ADCSEL+ADCPSC2 clear mask |
| 0800d628 | 0xFFF00000 | (len−1)<<20 helper |
| 0800d644 | 0x400124C0 | ADC_CCTL |
| 0800d640 | 0x00000FFF | CCNT mask |
| 0800d774, 0800d9b8, 0800da6c | 0x40012408 | ADC_CTL1 |
| 0800da68 | 0x40012404 | ADC_CTL0 |
| 0800d82c | 0x40012400 | ADC_STAT |
| 0800d974 / 0800d978 | 0x4001240C / 0x40012410 | SAMPT0 / SAMPT1 |
| 0800d968 / 0800d96c / 0800d970 | 0x4001242C / 0x40012430 / 0x40012434 | RSQ0 / RSQ1 / RSQ2 |
| 0800d624 | 0x40012438 | ISQ (not reached) |
| 0800e3d4 / 0800e44c / 0800e3ac / 0800e3b0 | 0x40020008 / 0x4002000C / 0x40020010 / 0x40020014 | DMA CH0 CTL/CNT/PADDR/MADDR (+0x14·ch) |
| 0800e274 | 0x40020004 | DMA_INTC |
| 0800e46c | 0x40020800 | DMAMUX RM_CH0CFG (+4·ch) |
| 0800b608 / 08007660 | 0x20000010 | ADC DMA buffer (10 × u16) |
| 08007664 | 0x3E2D4AA1 | 0.16923f |
| 080077c4 | 0x200005D8 | ST (result at +0x16) |
| 080083b8 | 0x20000C40 | delay counter |
| 0800e68c / 0800e690 / 0800e66c / 0800e590 / 0800e528 | 0x40022010 / 0x40022004 / 0x4002200C / 0x40022014 / 0x40022000 | FMC CTL / KEY / STAT / ADDR / base |
| 0800e694 / 0800e698 | 0x45670123 / 0xCDEF89AB | unlock keys |
| (imm) 0800e47c etc. | 0x00100000 | ready-wait timeout |
| 080060ac, 0800b5f8 (boot 080005e0) | 0x00000307 | bogus combined flag → STAT = 0x80 |
| boot 08001c28 / imm / 08001c2c / 08001c30 | 0x305 / 0x304 / 0x303 / 0x302 | proper ENDF / WPERR / PGAERR / PGERR clears |
| 0800b5fc / 0800b600 | 0x08005000 / 0x08010F10 | anti-clone erase range |
| 0800b618 / 0800b61c / 0800b620 | 0x20000084 / 0x0801F000 / 0x0801F3FF | log ctx, log page start/end |

## 8. Not verified / open

- The CK_ADC divider mapping (ck_adc = 5 → APB2/12) comes from the lib enum; the L235 user manual's RCU tables were not checked. **[I]**
- The meaning of STAT bits 15:14 → state 7, and what (flag>>6) == 0x18 (FMC+0x18) refers to in the newer lib. **[I]/unknown**
- Whether the L235 flash requires 64-bit (doubleword) programming, or would also accept 32-bit PG. Stock only ever writes doublewords. **[I]**
- That SWRCST self-clears and that reading RDATA clears EOC on L23x. This is standard GD behaviour, not exercised by stock. **[I]**
