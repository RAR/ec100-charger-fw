# OEM v1 app (v1.01.1105) + stock bootloader: literal register-level init sequences

Source: `dumps/gd32_flash_20261012.bin` (base 0x08000000). `re/decomp_app.c` is the **v1** app (its `app_Reset` is at 0x08005174).
Register names are from the GD32L23x v1.0 library headers. Method: objdump disassembly with literal-pool resolution,
`bin/calls.py` constant propagation for library call arguments, and **unicorn emulation of the v1 render function
0800a0dc and of `rcu_clock_freq_get` 0800eb00** for the LCD bitmap and clock numbers (scripts in `scratchpad/init/`).

Tags: **[V]** = verified in code, literals or emulation. **[I]** = inferred (semantics, or reasoning from the datasheet).

Library helper addresses in v1. These are all GD standard-library functions, identified from their bodies **[V]**:

| v1 addr | function | effect |
|---|---|---|
| 0800f5ac | rcu_periph_clock_enable(x) | `REG(0x40021000 + (x>>6)) |= 1<<(x&31)` |
| 0800f5f4 / 0800f5d0 | rcu_periph_reset_enable / _disable | same encoding |
| 0800f358 / 0800f37c | rcu_osci_on / rcu_osci_stab_wait | 0x900 = RSTSCK bit0 IRC32KEN; waits on 0x901 (IRC32KSTB), timeout 0xFFFFF |
| 0800f61c | rcu_rtc_clock_config | BDCTL &= ~0x300; BDCTL \|= arg |
| 0800eb00 | rcu_clock_freq_get(0=SYS,1=AHB,2=APB1,3=APB2) | HXTAL_VALUE literal 0x00B71B00 = 12 MHz @0x0800ef50 |
| 0800e8ee | gpio_mode_set(port, mode, pupd, pins) | RMW CTL (+0x00), then PUD (+0x0C) |
| 0800e97c | gpio_output_options_set(port, otype, speed, pins) | RMW OMODE (+0x04), then OSPD (+0x08) |
| 0800e7d4 | gpio_af_set(port, af, pins) | RMW AFSEL0 (+0x20), then AFSEL1 (+0x24) |
| 0800e8a4 / 0800e894 / 0800e8b4 / 0800e8c4 | gpio_bit_set (BOP +0x18) / gpio_bit_reset (BC +0x28) / toggle (TG +0x2C) / input_bit_get (ISTAT +0x10) | |
| 0800e7c4 / 0800e6b4 / 0800e6c0 / 0800e7bc | FWDGT_CTL = 0x5555 / 0xAAAA / 0xCCCC / 0x0000 | |
| 0800e6cc / 0800e73c | fwdgt_prescaler_value_config / fwdgt_reload_value_config | |
| 0800f63c / 0800f6fc / 0800f780 / 0800f718 | SLCD bias / duty / VSRC / SLCDON (SLCD_CTL) | |
| 0800f684 / 0800f6bc / 0800f658 / 0800f750 | SLCD clock(PSC\|DIV) / dead time / blink / HDEN (SLCD_CFG) | |
| 0800f724 / 0800f6b0 | slcd_flag_get(SLCD_STAT & f) / SLCD_STAT \|= UPRF (update request) | |
| 0800ea28 | nvic_irq_enable(irq, pre) | priority byte = pre<<6, then ISER |
| 0800d4a4 | NVIC_SetPriority(irq, p) | SysTick uses SHPR3 (0xE000ED1C) |
| 0800ea58 | nvic_vector_table_set(base, off) | VTOR = base \| (off & 0x1FFFFF80) |

---

## 0. Order of execution in the v1 app [V]

`app_Reset` 08005174:
1. Zeroes **all SRAM**: size = (`*(u32*)0x1FFFF7E0` >> 16) KB = 24 KB from 0x20000000.
2. Calls `SystemInit` 0800cdf4.
3. `__main` 08005160 sets SP = 0x20001448 and runs scatter-load 08005908 (table 0x08010F28..0x08010F48).
4. `main` 0800ea08 calls:
   - `rcu_clock_freq_get(CK_SYS)`, storing the result at 0x20000C3C
   - `init` 0800b50c
   - then loops forever in 0800b630

`init` 0800b50c, in this order:

| # | call | what |
|---|---|---|
| 1 | 0800ea58(0x08000000, 0x5000) | **VTOR = 0x08005000** |
| 2 | 08007338 | stack-paint (fills the 2 KB below 0x20001448). No hardware access. |
| 3 | 08008644 | SysTick 1 ms (§6) |
| 4 | 0800742c | anti-clone check (on mismatch it erases itself). Not hardware init. |
| 5 | 0800f5ac(0x616) | RCU_APB2EN \|= DBGMCUEN (bit22) |
| 6 | 0800e1a0(0x108) | DBG_CTL0 (0x40015804) \|= FWDGT_HOLD (bit8) |
| 7 | 080083e8 | FWDGT (§2) |
| 8 | 080084a4 | PD0/PD1 outputs, high (§5) |
| 9 | 08008504 | PC1 output, low |
| 10 | 08008548 | PB10 output, low (RS485 DE) |
| 11 | 08008470 | PC2 output, low |
| 12 | 0800840c | keys PC6/PB15/PA12/PA11 |
| 13 | 080080f8(0x20000010, 10) and 0800816c | ADC DMA buffer and ADC; PC0 analog |
| 14 | 08008824(1, 9600), (3, 9600), (4, 460800) | USART1, UART3, UART4 |
| 15 | 0800866a(0x3B, 0x3E7) | TIMER1, 1 ms |
| 16 | 08008254 | CAN on PA4/PA5 |
| 17 | 08008588 | **SLCD** (§3) |
| 18 | | settings load, etc. Sets ST(0x200005D8)+0x4C = 0x1D4C0 (backlight 120 s) and ST+9 = 1. |

Main loop 0800b630 **[V]**:
- Calls 080083e0 first on every pass. This is the **FWDGT reload, FWDGT_CTL = 0xAAAA**.
- In normal mode (0x2000007C == 0) it calls, among others, 08005c54 (PC1 logic 08007754 and LED logic 0800b444) and then the LCD refresh 08006864.
- Factory test mode instead calls 08005c38 (0800c91c and 08009e54).

---

## 1. Clock tree

### 1a. v1 `SystemInit` 0800cdf4 (runs from `app_Reset`, before RAM init) [V]

| step | register (addr) | operation | wait |
|---|---|---|---|
| 1 | RCU_CTL (0x40021000) | \|= 0x1 (IRC16MEN) | until RCU_CTL bit1 IRC16MSTB = 1 (no timeout) |
| 2 | RCU_CFG0 (0x40021004) | if (CFG0 & 0xC) == 0x8 (SCSS = PLL, which is always true after the bootloader): 80-iteration spin; CFG0 = (CFG0 & ~0xF0) \| 0x80 (AHB/2); 80 spin; CFG0 = (CFG0 & ~0xF0) \| 0x90 (AHB/4); 80 spin | software loops |
| 3 | RCU_CFG0 | &= ~0x3 (SCS = IRC16M) | then 0800d544(10): a 100-iteration spin. SCSS is **not** polled. |
| 4 | RCU_CFG1 (0x4002102C) | &= 0xFFFEFFFF (bit16; undefined in the v1.0 header) | |
| 5 | RCU_CTL | &= 0xFEF2FFFF (clears HXTALEN b16, HXTALBPS b18, CKMEN b19, PLLEN b24) | |
| 6 | RCU_CFG0 | &= 0x88FF000C, then &= 0x7700FFFF (clears SCS, AHB/APB1/APB2/ADC PSC, PLLSEL, PLLMF, CKOUT, PLLMF_6, PLLDV) | |
| 7 | RCU_CFG1 | &= ~0xF (PREDV = /1), then &= 0xFFFEFFFF again | |
| 8 | RCU_CFG2 (0x40021030) | = 0 | |
| 9 | RCU_INT (0x40021008) | = 0 | |
| 10 | | `system_clock_config` 0800f8f0 → 0800f7dc (§1b) | |
| 11 | VTOR (0xE000ED08) | = 0x08000000 (nvic_vector_table_set(0x08000000, 0)), then DSB | |

### 1b. v1 clock configuration 0800f7dc. The bootloader's 08002c8c is the same code. [V]

| step | register | operation | wait |
|---|---|---|---|
| 1 | RCU_CTL | \|= 0x00010000 (HXTALEN) | fixed spin: counter 0 → 8192 (until counter>>13 ≠ 0) |
| 2 | | | poll RCU_CTL bit17 HXTALSTB with timeout counter 0xFFFF; if still clear, **hang forever** (b .) |
| 3 | FMC_WS (0x40022000) | = (FMC_WS \| 0x2) & ~0x5, i.e. **WSCNT = 2** | |
| 4 | RCU_CFG0 | \|= 0 (AHB /1), \|= 0 (APB2 /1), \|= 0x400 (**APB1PSC = 100b, /2**) | |
| 5 | RCU_CFG1 | &= ~0xF (PREDV /1) | |
| 6 | RCU_CFG0 | &= 0x7F00FFFF, then \|= 0x000D0000 (**PLLSEL = 01 HXTAL, PLLMF = 3 → ×5**) | |
| 7 | RCU_CTL | \|= 0x01000000 (PLLEN) | until RCU_CTL bit25 PLLSTB |
| 8 | RCU_CFG0 | = (CFG0 & ~0x3) \| 0x2 (SCS = PLL) | until (CFG0 & 0xC) == 0x8 |

**Final result [V]** (emulated `rcu_clock_freq_get`): SYSCLK = 60 MHz, AHB = 60 MHz, APB1 = 30 MHz, APB2 = 60 MHz.
The APB1 timer clock is 60 MHz **[I]** (because APB1PSC ≠ 1). Final RCU_CFG0 = 0x000D040A. That the crystal is 12 MHz is **[I]**: it is the
compiled-in HXTAL_VALUE 0x00B71B00.

**PMU / LDO [V]:** the app never writes PMU_CTL0.LDOVS, RCU_VKEY (0x40021100), RCU_LPLDO or RCU_LPB. No literal for any of them exists in the app.
The only PMU write is PMU_CTL0 \|= BKPWEN (§3).

### 1c. What the bootloader leaves configured at the jump (boot 080005e4) [V unless tagged]

- **Clock:** identical to §1b (60/60/30/60 MHz, FMC WSCNT = 2). The boot `SystemInit` 0800149c is byte-identical in logic. VTOR = 0x08000000.
- **SysTick:** boot 08000cfc → 08001308:
  - LOAD = SYSCLK/1000 − 1 = 59999 (0xEA5F), VAL = 0, **CTRL = 7** (running, TICKINT on)
  - SHPR3 priority set to 3, then reset to 0 by boot 080020e0(−1, 0)
  - Still running at the jump, and it vectors through VTOR = 0x08000000 to the **boot SysTick handler 0x08001355** (which toggles PB8 and uses RAM 0x20000026…)
  - The window lasts until the app's `SystemInit` + `init` reprogram it. Note that the app's `app_Reset` zeroes RAM while this handler can still fire.
- **FWDGT:** running. PSC = 3 (/32), RLD = 0x800. It was last reloaded (0xAAAA, via 08000be4) **immediately before the jump** (080005e4 @0x08000668).
  Timeout ≈ 2048/(32 kHz/32) ≈ 2.05 s **[I]** (nominal IRC32K).
- **NVIC:** ICER0 (0xE000E180) = 0xFFFFFFFF, so all IRQs are disabled. **Pending bits are not cleared.** Priorities from boot remain: IRQ21 and IRQ30 at 0.
  PRIMASK is not touched **[I]**. MSP = *(0x08005000); the jump is `blx` to *(0x08005004).
- **Peripherals left on:**
  - RCU clocks: AHBEN GPIOA/B/C (0x511/512/513) and DMA (0x500); APB1EN TIMER1 (0x700), UART4 (0x714), PMU (0x71C), BKP (0x71F); APB2EN DBGMCU (0x616).
  - IRC32K on (boot 08000fb0: rcu_osci_on(0x900) + wait).
  - DBG_CTL0 FWDGT_HOLD (0x108) + TIMER1_HOLD (0x10C).
  - PMU_CTL0 BKPWEN set (boot 08002134).
  - **TIMER1 running**, 1 ms with UPIE set (only its NVIC line is off).
  - **UART4 enabled** on PB0/PB1 (AF4, pull-up, PB0 PP 50 MHz) at 460800, with RX DMA and IDLE IRQ (NVIC off).
  - PB8/PB9 outputs PP 50 MHz (PB8 heartbeat; both reset low at init); PB10 output PP low.
  - RTC BKP2 (0x40002858) written 0 just before the jump. BKP3 (0x4002285C) is written only on the stay path.
- **The app re-does all of these**:
  - TIMER1 via timer_deinit (APB1RST bit0)
  - UART4 via 08008824
  - PB8/PB9 → SLCD AF3 in 0800cafc
  - SysTick via SysTick_Config
  - It does **not** reset DMA, or clear NVIC pending bits, before enabling IRQs **[V]** (no ICPR literal 0xE000E280 in the app).

---

## 2. FWDGT

**Bootloader 08000bec [V]:**

| step | register | value | wait |
|---|---|---|---|
| 1 | FWDGT_CTL 0x40003000 | 0x5555 | |
| 2 | FWDGT_CTL | 0x5555 (again, inside psc_config 08001dbc) | while FWDGT_STAT (0x4000300C) bit0 PUD = 1, timeout 0xFFFFF |
| 3 | FWDGT_PSC 0x40003004 | 0x3 (**/32**) | |
| 4 | FWDGT_CTL | 0x5555 (inside rld_config) | while STAT bit1 RUD = 1, timeout 0xFFFFF |
| 5 | FWDGT_RLD 0x40003008 | **0x800** (& 0xFFF) | |
| 6 | FWDGT_CTL | **0xCCCC** (start) | |
| 7 | FWDGT_CTL | 0xCCCC (second call to the same function; no 0xAAAA here) | |
| 8 | FWDGT_CTL | 0x0000 (re-lock) | |
| 9 | FWDGT_CTL | 0xAAAA (reload, 08000be4 at the end of boot init and before the jump) | |

**v1 app 080083e8 re-runs it with the same values [V]:** 0x5555 → (0x5555, wait PUD) PSC = 3 → (0x5555, wait RUD) RLD = 0x800 → **0xAAAA** → **0xCCCC** → 0x0000.
- The app does not change the timeout.
- It reloads (CTL = 0xAAAA via 080083e0) once per main-loop pass.
- It also sets DBG_CTL0.FWDGT_HOLD.

FWDGT_WND is never written **[V]**.

**IRC32K [V]:** RCU_RSTSCK (0x40021024) bit0 IRC32KEN is set explicitly:
- by the bootloader (08000fb0, before anything else)
- by the app in its SLCD init (§3 step 4), with a wait on bit1 IRC32KSTB (timeout 0xFFFFF, result ignored)

There is no IRC40K on this part; the FWDGT clock is IRC32K **[I]**.

---

## 3. SLCD bring-up (v1 08008588; GPIO part 0800cafc) [V]

### 3a. GPIO (0800cafc). Call order and arguments:
1. RCU_AHBEN \|= PAEN, PBEN, PCEN, PDEN (0x511, 0x512, 0x513, 0x514).
2. **COM pins:**
   - mode_set(port, AF = 2, PULLDOWN = 2, pins) and output_options(port, PP, 50 MHz = 3, pins), for:
     GPIOA 0x0700 (PA8 COM0, PA9 COM1, PA10 COM2), GPIOB 0x0200 (PB9 COM3), GPIOC 0x1C00 (PC10 COM4, PC11 COM5, PC12 COM6), GPIOD 0x0004 (PD2 COM7)
   - then af_set(AF3) on the same four port/pin groups.
3. **SEG pins**, each port mode → options → AF3, AF + pull-down + PP + 50 MHz:
   - GPIOA 0x80C0 (PA6 SEG3, PA7 SEG4, PA15 SEG17)
   - GPIOB 0x7938 (PB3 SEG7, PB4 SEG8, PB5 SEG9, PB8 SEG16, PB11 SEG11, PB12 SEG12, PB13 SEG13, PB14 SEG14)
   - GPIOC 0x03B0 (PC4 SEG22, PC5 SEG23, PC7 SEG25, PC8 SEG26, PC9 SEG27)
   - GPIOD 0x0310 (PD4 SEG28, PD8 SEG30, PD9 SEG31)

   The SEG numbers per pin come from the datasheet AF table **[V vs ds/l235.txt]**. The 19 SEGs match the set of SEG bits the render code uses exactly.
4. **PD6** (VSLCD): mode_set(GPIOD, ANALOG = 3, NONE, 0x40).

Resulting register fields for the LCD pins (RMW: only these bits change):

| port | CTL mask / value | PUD mask / value | OSPD mask / value | OMODE bits cleared | AFSEL0 mask / value | AFSEL1 mask / value |
|---|---|---|---|---|---|---|
| A (6,7,8,9,10,15) | 0xC03FF000 / 0x802AA000 | same / 0x802AA000 | same / 0xC03FF000 | 0x87C0 | 0xFF000000 / 0x33000000 | 0xF0000FFF / 0x30000333 |
| B (3,4,5,8,9,11–14) | 0x3FCF0FC0 / 0x2A8A0A80 | same / 0x2A8A0A80 | same / 0x3FCF0FC0 | 0x7B38 | 0x00FFF000 / 0x00333000 | 0x0FFFF0FF / 0x03333033 |
| C (4,5,7–12) | 0x03FFCF00 / 0x02AA8A00 | same / 0x02AA8A00 | same / 0x03FFCF00 | 0x1FB0 | 0xF0FF0000 / 0x30330000 | 0x000FFFFF / 0x00033333 |
| D (2,4,8,9) + PD6 | 0x000F3330 / 0x000A3220 | 0x000F3330 / 0x000A0220 (PD6 none) | 0x000F0330 / 0x000F0330 | 0x0314 | 0x000F0F00 / 0x00030300 | 0x000000FF / 0x00000033 |

### 3b. Clocks, backup domain, controller

| step | v1 addr | register | operation | wait |
|---|---|---|---|---|
| 1 | 0800cafc | GPIO | §3a | |
| 2 | 0800f5ac(0x71C) | RCU_APB1EN 0x4002101C | \|= bit28 **PMUEN** | |
| 3 | 0800ea7c | PMU_CTL0 0x40007000 | \|= 0x100 **BKPWEN** | none |
| 4 | 0800f358(0x900) | RCU_RSTSCK 0x40021024 | \|= bit0 **IRC32KEN** | |
| 5 | 0800f37c(0x900) | RCU_RSTSCK | | bit1 IRC32KSTB, timeout 0xFFFFF |
| 6 | 0800f61c(0x200) | **RCU_BDCTL 0x40021020** | &= ~0x300; \|= 0x200: **RTCSRC = 10b = IRC32K**. LXTAL is never enabled and RTCEN is never set. | |
| 7 | 0800f5ac(0x70A) | RCU_APB1EN | \|= bit10 **SLCDEN** | |
| 8 | | | software spin i = 0…999 | |
| 9 | 0800f6e0 | RCU_APB1RST 0x40021010 | \|= bit10, then &= ~bit10 (slcd_deinit) | |
| 10 | 0800f684(0x400000, 0) | SLCD_CFG 0x40002404 | &= ~0x03FC0000; \|= 0x00400000: **PSC = 1 (/2), DIV = 0 (16)** | |
| 11 | 0800f63c(0) | SLCD_CTL 0x40002400 | &= ~0x60; \|= 0: **BIAS = 1/4** | |
| 12 | 0800f6fc(0x10) | SLCD_CTL | &= ~0x1C; \|= 0x10: **DUTY = 100b = 1/8** | |
| 13 | 0800f6bc(0x80) | SLCD_CFG | &= ~0x380; \|= 0x80: **DTD = 1** | |
| 14 | 0800f780(2) | SLCD_CTL | \|= 0x2: **VSRC = external (VSLCD/PD6)** | |
| 15 | 0800f658(0x20000, 0xE000) | SLCD_CFG | &= ~0x3E000; \|= 0x2E000: **BLKDIV = 7 (/1024), BLKMOD = 2 (SEG0 all COM)**. SEG0 is not wired, so nothing visibly blinks **[I]**. | |
| 16 | 0800f750(1) | SLCD_CFG | \|= 0x1: **HDEN** | |
| 17 | 0800f724(0x20) | SLCD_STAT 0x40002408 | | **until SYNF (bit5) = 1** |
| 18 | 0800f718 | SLCD_CTL | \|= 0x1: **SLCDON** | |
| 19 | 0800f724(0x01) | SLCD_STAT | | **until ONF (bit0) = 1** |

Not done [V]:
- no VRDYF wait
- PULSE and CONR stay 0
- **COMS (CTL bit7) is never written (stays 0)** and VODEN is never set
- no SOFIE/UPDIE
- SLCD_STATC is never written

COMS = 0 together with the datasheet pinout means COM4–7 sit on PC10/PC11/PC12/PD2 and SEG28/30/31 on PD4/PD8/PD9 **[I]** (this pairing works on the bench).

**Final values: SLCD_CTL = 0x00000013, SLCD_CFG = 0x0042E081.** **[V]**: the computed RMW result after the APB1 reset.
Frame rate ≈ 32 kHz / (2·16) / 8 = 125 Hz **[I]**.

### 3c. Data registers and update procedure
- SLCD_DATA0…7 = **0x40002414, 0x4000241C, 0x40002424, 0x4000242C, 0x40002434, 0x4000243C, 0x40002444, 0x4000244C** (COM0…COM7). **[V]**
- v1 RAM framebuffer: **0x20000154 … 0x20000170** (COM0…COM7). **[V]**
- Refresh 08006864 runs on every main-loop pass **[V]**:
  - If ST+0x4C (backlight countdown, ms, −10 per 10 ms in the TIMER1 ISR) ≠ 0:
    1. If ST+0x40 == 0: set ST+0x40 = 200 and call the render 0800a0dc. So **render every 200 ms**.
    2. `while (SLCD_STAT & 0x4 /*UPRF*/);`
    3. DATAn = fb[n] for n = 0..7.
    4. **SLCD_STAT \|= 0x4** (update request, 0800f6b0).
    5. **PC2 = 1**.
  - Else:
    - fb[0..7] = 0, SLCD_STAT \|= 0x4 **without writing DATAn** (the glass keeps its last frame), **PC2 = 0**.
- Factory test display 08009e54 writes DATAn directly after waiting for UPRF to clear, then sets UPRF.

---

## 4. LCD bitmap model (v1 render 0800a0dc, verified by emulation) [V]

The render clears fb[0..7] and rebuilds it from the state struct `ST` = 0x200005D8. There are no calls inside, so it is a pure function.
Bit *s* of word COMk is SEG*s* × COM*k*.

### 4a. Fonts (v1 copies; same bytes as v2) [V]
- **Font A** @0x08010EDC: `EE 24 D6 B6 3C BA FA 26 FE BE | 00 (blank) | 10 ('-') | DA ('E')`
  - Bit k → COMk on the digit's SEG column.
  - Segment map **[I]**, derived from the glyphs: COM1 = a, COM2 = b, COM3 = f, COM4 = g, COM5 = c, COM6 = e, COM7 = d.
- **Font B** @0x08010EE9: `EE 48 D6 DA 78 BA BE C8 FE FA | 00 | 10`
  - These are not mirrored digits. They are the same digits for a differently wired glass position **[I]**: COM1 = d, COM2 = e, COM3 = c, COM4 = g, COM5 = f, COM6 = b, COM7 = a.
- **Bit0 (COM0) is 0 in every glyph.** COM0 of every digit column is therefore a separate fixed label/unit/DP segment.

### 4b. Fixed bits (set unconditionally) [V]
- **COM0 = 0xDCC27B18**: SEG3, 4, 8, 9, 11, 12, 13, 14, 17, 22, 23, 26, 27, 28, 30, 31.
  - SEG8/SEG9 COM0 are cleared (`&= ~0x200`, `&= ~0x100`) only in the error-display phase.
  - SEG7 COM0 is never set by the render (the factory test does light it).
- **COM1 = 0x10000000** (SEG28 COM1).
- **"All fixed labels on" constant:** `{0xDCC27B18, 0x10000000, 0, 0, 0, 0, 0, 0}`.
- Which COM0 label is the battery-V decimal point (SEG30 or SEG12 COM0) is **[I]** (glass legend unknown).

### 4c. Fields

| field | SEG (most → least significant digit) | font | source (v1 ST offsets) | blanking / notes |
|---|---|---|---|---|
| Battery V (0.1 V) | 31, 30, 12 | A | +0x44 (u32). Menu 2: +0x28 charge V. Menu 5: +0x12 start V. Menu 6: +0x14 stop V. | Hundreds digit blank if 0. "57.6" = 576. |
| Current (A) | 22, 4, 3 | A | +0x48 (u32). Menu 1: +0x24 max current. | Hundreds blank if 0; tens blank if hundreds and tens are 0. |
| Power (W) | 11, 13, 14, 23 | A | +0x30 (u16) | Leading zeros blanked down to units. |
| SOC % / error | 9, 8, 7 | A | +6 SOC. Menu 3: +3 start SOC. Menu 4: +4 stop SOC. | +8 ≠ 0 → `---` (0x10 ×3). Error (+0x3E ≠ 0 and ISR counter +0x10 ≥ 100): SEG9 = 'E' (0xDA), SEG8/SEG7 = 2 digits of +0x3E, SEG8/9 COM0 cleared. +0x10 wraps at >200, giving 1 s SOC / 1 s error. |
| AC input V | 26, 27, 17 | **B** | +0x2C (u16) | 0 → `---` |
| SOC bar | SEG16 COM7, COM6 … COM0 | — | +6 | Bars at SOC > 0, 13, 27, 39, 51, 63, 75, 87 % (COM7 first, COM0 last). |
| Icons (SEG28) | COM0, COM1 always on. COM2 if +8 == 0. COM5 if +9 ≠ 0. COM6 if +9 == 0. COM7 if +0x3E ≠ 0 (error). | | | |
| Menu indicators | Menu 1: SEG28 COM4. Menu 2: SEG28 COM3. Menu 3: SEG25 COM7. Menu 4: SEG25 COM6. Menu 5: SEG25 COM5. Menu 6: SEG25 COM4. | | +2 = menu item | Lit only in the "on" half of the blink. |

- **Menu blink:** ST+0x20 (+10 per 10 ms while the menu is active).
  - < 300 ms: the edited field's digits are blank (labels stay) and the indicator is off.
  - 300…600 ms: value and indicator are shown.
  - The counter resets at ≥ 601.
- **SOC bar animation:**
  - It is animated when all of these hold: +0x48 ≠ 0, +10 ≠ 0, (0x200004B0[0] == 1 or [1] == 1), and +0x3E == 0. Otherwise it is static.
  - Animated mode: bar k (k = 0..7 from COM7 up) is lit if SOC > its threshold **and** the step counter @0x20000174 > k.
  - Every 300 ms (+0x22) the counter does `cnt = (cnt*10 <= SOC) ? cnt+1 : 0`. In a menu it is forced to 8.
- **Full-glass mask** (union of the four factory-test groups in 08009e54; test bits in 0x2000007F; 0xF = all on/off flash):
  - COM0–3 = 0xDCC37B98, COM4–7 = 0xDEC37B98.
  - Groups:
    1. 0xD0001000 (SEG12, 28, 30, 31)
    2. 0x04030380 for COM0, 0x0C030380 for COM1–7 (SEG7, 8, 9, 16, 17, 26, 27)
    3. 0x02000000 for COM4–7 (SEG25)
    4. 0x08C06818 for COM0, 0x00C06818 for COM1–7 (SEG3, 4, 11, 13, 14, 22, 23, 27)
- Emulated frame for an all-zero state: `DCC27B18 50801088 50801088 40801088 0C020000 40801088 50801088 40801088`.

---

## 5. GPIO init (non-LCD) and v1 use of each pin [V]

| pin | init (call site) | initial level | v1 use (call sites) |
|---|---|---|---|
| PC6, PB15, PA12, PA11 | 0800840c: mode_set(port, INPUT 0, PULLDOWN 2, pin) | — | Read through ISTAT in 08006cc0 (keys, active-high). |
| PB10 | 08008548: OUTPUT, no pull; PP, 50 MHz | **low** (BC) | RS485 DE. Set before TX, reset after: 0800982c (Pylon), 080069ac (UART4 replies), 08009b24/08005ac0/0800c80e/0800c9d4 (test mode). |
| PA2 / PA3 (USART1) | 08008824: RCU APB1EN USART1 (0x711), GPIOA; af_set(GPIOA, AF7, PA2), af_set(GPIOA, AF7, PA3); mode_set(PA2, AF, PULLUP); options(PA2, PP, 50 MHz); mode_set(PA3, AF, PULLUP) | | BMS 9600 |
| PB0 / PB1 (UART4) | 08008824: APB1EN UART4 (0x714); af_set(GPIOB, AF4, PB0/PB1); mode_set(PB0, AF, PULLUP); options(PB0, PP, 50 MHz); mode_set(PB1, AF, PULLUP) | | service / update, 460800 |
| PA0 / PA1 (UART3) | Same pattern with AF8; APB1EN 0x713 | | 9600, never transmits in v1 |
| PC0 | 0800816c: APB2EN ADCEN (0x609), GPIOC; mode_set(GPIOC, ANALOG 3, NONE, PIN0) | — | ADC |
| PD0, PD1 | 080084a4: OUTPUT, no pull; PP, 50 MHz | **high** (BOP) | **LEDs, active-low [I]**; see the logic below. |
| PC1 | 08008504: OUTPUT, no pull; PP, 50 MHz | **low** | See the logic below. |
| PC2 | 08008470: OUTPUT, no pull; PP, 50 MHz | **low** | **LCD backlight**: set every refresh while ST+0x4C > 0 (080068e0), cleared when it expires (08006932). Also set in test display 08009fcc. |
| **PB6** | **never configured or touched by v1** | reset state | Exhaustive check: every GPIOB literal reference and every gpio_* call in the app; none uses pin 0x40. |
| PD0 / PD1 in test mode | 080069ac @08006b7c/84: both **reset (low)** when test mode is entered | | The bench note "both LEDs turn on" confirms they are **active-low LEDs**. 08009e54 toggles both periodically. |

- **PD0 (0800b444):** high (off) when ST+0x3E == 0; low when an error code is set. This is the **Fault LED** **[I]**.
- **PD1 (0800b444)**, the **charge/status LED** **[I]**. M below means a PSU module is present (0x200004B0[0] == 1 or [1] == 1). In order:
  1. If charging (ST+10 == 1), M, and no error:
     - current ST+0x48 == 0: toggle every 100 ms (ST+0xE = 10 ticks of 10 ms)
     - otherwise: **solid low**
  2. Otherwise, if M and no error: toggle every 500 ms (ST+0xE = 0x32).
  3. Otherwise: high (off).
- **PC1 (08007754)**, the "charging output" (load **[I]**):
  1. If ST+9 == 1 (voltage mode, no BMS) **and** ST+10 == 1 (charging): **set high**, ST+7 = 1.
  2. Else if ST+10 == 0: reset low, ST+7 = 0.
  3. Else (charging in BMS mode): forced low once ST+0x2E reaches ≥ 1001 (10 s; the ISR counts it while charging with PC1 off).

  **In v1, PC1 is never driven high in BMS mode.** It is also driven by the B5 5B factory-test command (0800c91c @0800cad6 set / 0800cae0 reset).

---

## 6. SysTick / TIMER1 in v1 [V]

- **SysTick** (08008644 → 0800cda0):
  - LOAD = 60 000 000/1000 − 1 = **59999**; SHPR3 prio 3 (0xC0); VAL = 0; **CTRL = 7**
  - A failure return makes it hang.
  - Then 0800d4a4(−1, 0) sets the SysTick priority to **0**.
  - Handler (exception 15) 0800cdec → 08008388: `if (*(u32*)0x20000C40) --*(u32*)0x20000C40;` (delay-ms counter only).
- **TIMER1** (0800866a(0x3B, 0x3E7)):
  1. RCU_APB1EN \|= TIMER1EN (0x700).
  2. timer_deinit: APB1RST bit0 set, then cleared.
  3. TIMER1_PSC (0x40000028) = 0x3B.
  4. CTL0 &= ~0x70 (edge-aligned, up), CAR (0x2C) = 0x3E7, CTL0 &= ~0x300 (CKDIV /1).
  5. SWEVG (0x14) \|= UPG.
  6. CTL0 \|= ARSE (0x80).
  7. INTF (0x10) = ~UPIF.
  8. DMAINTEN (0x0C) \|= UPIE.
  9. **nvic_irq_enable(21, 0)**.
  10. CTL0 \|= CEN.

  Result: 60 MHz/60/1000 = **1 kHz**.
- **TIMER1 ISR, IRQ21 (vector 37) 0800cf2c:**
  - Every 1 ms: ST+0x3C++ and a saturating counter at 0x20000152.
  - Every 10th tick (counter 0x20000628), the 10 ms work:
    - key scan 0800978c
    - backlight countdown ST+0x4C −= 10
    - render countdown ST+0x40 −= 10
    - menu blink ST+0x20 += 10
    - bar timer ST+0x22 += 10
    - LED timer ST+0xE −= 1
    - error blink ST+0x10++
    - PC1 timer ST+0x2E
    - etc.
- **Other NVIC enables, all priority 0:**

  | IRQ | peripheral | enabled in |
  |---|---|---|
  | 20 | CAN (shares the USBD_LP slot on L235) **[I]** | 08008254 |
  | 28 | USART1 | 08008824 |
  | 29 | UART3 | 08008824 |
  | 30 | UART4 | 08008824 |

  IRQ18 (ADC) has a handler (08005AF5) but is **never enabled**.

---

## 7. v1 vector table @0x08005000 [V]

| entry | value | handler |
|---|---|---|
| 0 | **0x20001448** | initial SP |
| 1 | 0x08005175 | Reset (app_Reset) |
| 2 | 0x0800C5F9 | NMI |
| 3 | 0x080098E1 | HardFault |
| 11 | 0x0800CC89 | SVCall |
| 14 | 0x0800C61D | PendSV |
| 15 | 0x0800CDED | SysTick |
| 16+18 (IRQ18) | 0x08005AF5 | ADC (never enabled) |
| IRQ20 | 0x0800D275 | CAN RX (USBD_LP slot) |
| IRQ21 | 0x0800CF2D | TIMER1 |
| IRQ28 | 0x0800D1F9 | USART1 |
| IRQ29 | 0x0800D103 | UART3 |
| IRQ30 | 0x0800D17D | UART4 |
| all other IRQ0–47 | 0x080051A1 | default (b .) |

Entries 4–10, 12, 13 and IRQ38 are 0.

For comparison, the bootloader table @0x08000000 has SP 0x20001890, IRQ21 0x080015D5, IRQ27 0x08001701, IRQ30 0x080016F9 and SysTick 0x08001355.

---

## 8. Differences from docs/history/notes/oem-hw-map.md

1. §3.1 says the menu indicators are "SEG25 COM3/4/5, SEG28 COM4". In **v1** they are:
   - SEG28 COM4 = menu 1, SEG28 COM3 = menu 2
   - SEG25 COM7/6/5/4 = menus 3/4/5/6
2. §3.1 icon row: the SEG28 COM2 / COM5 / COM6 conditions are:
   - COM2: ST+8 == 0 (and +8 ≠ 0 also turns SOC into `---`)
   - COM6: ST+9 == 0
   - COM5: ST+9 ≠ 0

   The note pairs the "BMS-online (ST+9 == 0)" label with COM2.
3. §3.1 describes the v2 frame buffer at 0x20000B68, copied every 200 ms. In **v1**:
   - The frame buffer is at **0x20000154**.
   - It is re-rendered every 200 ms but **copied to SLCD_DATA and update-requested on every main-loop pass**.
   - Backlight-off clears only the RAM buffer. The DATA registers are not rewritten.
4. §2.1 describes PC1 as "high while charging, forced on, or for 10 s after stop". That is v2 behaviour. In **v1**, PC1 is high only while charging in voltage mode (ST+9 == 1) and is never high in BMS mode (§5).
5. §1 says the "v2 SLCD init then asks for IRC32K". **v1 does the same**:
   - BDCTL RTCSRC = IRC32K (0x200)
   - it never touches LXTAL or RTCEN
   - the RTC itself is unused in v1 (only BKP writes)
6. Not in the map, and matter for our firmware:
   - v1 `SystemInit` first steps AHB to /2 → /4 and SCS → IRC16M, because the bootloader left the PLL running.
   - It points VTOR at 0x08000000 before `main` sets 0x08005000.
   - The bootloader leaves TIMER1/UART4/DMA running with only the NVIC disabled, and pending bits uncleared.
   - The bootloader FWDGT sequence uses 0xCCCC twice and no 0xAAAA (the AAAA comes later).
7. PB6 is **not used at all by v1**. The board-photo hypothesis "PB6 = relay driver" therefore cannot be checked from v1 code.
   - PC2 is the backlight in v1 code, not an LED.
   - PD0/PD1 behave as active-low Fault/Charge LEDs.
   - So the photo-based guess "PC1/PC2 = Charge/Fault LEDs" conflicts with v1 code: PC2 follows the display backlight timer. Bench-toggle to confirm.
