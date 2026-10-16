# Eco-Worthy charger control board: findings

## MCU (read over SWD with an ST-Link V2 and OpenOCD, 2026-10-12)
- GigaDevice Cortex-M23 r1p0 (CPUID 0x411CD200, SW-DP 0x0BF11477), DBG_ID @0x40015800 = 0x23050410.
- Flash size register 0x1FFFF7E0 = 0x00180080, i.e. 128 KB flash and 24 KB SRAM (assumed meaning).
- **Not read-protected.** Option bytes start `A5 5A` (SPC=0xA5, unprotected); FMC_OBSTAT=0x03FFFFFC.
- OpenOCD: `-f interface/stlink.cfg -c "transport select hla_swd" -c "reset_config none" -c "set CPUTAPID 0" -f target/stm32f1x.cfg -c "\$_TARGETNAME configure -event examine-end {}"`
  (stm32f1x.cfg is used only to get a generic Cortex-M target; its flash driver doesn't fit this chip, so **don't flash with it**).

## Flash layout (dumps/gd32_flash_20261012.bin, SHA1 54c5f34538c958cc6d99ac6e17f48b1b66c5e883)
- 0x08000000–0x08004FFF: bootloader (presumably the RS485 updater). SP=0x20001890, reset=0x08000169.
- 0x08005000–0x0801F3FF: application (VTOR=0x08005000). SP=0x20001448, reset=0x08005175.
- Version string "1.01.1105" @ 0x10F1B.

## CAN
- bxCAN-style peripheral at 0x40006400 (GD32 CAN0).
- BT=0x0014001D: BRP=30, TS1=5, TS2=2, i.e. 8 tq at a 75% sample point. Clock is PLL from HXTAL with x5 and APB1 /2.
  If the crystal is 12 MHz: SYSCLK 60 MHz, APB1 30 MHz, so **125 kbps**, matching the Huawei R48xx. (Check the crystal marking.)
- Pending TX mailboxes seen live (bench, no PSUs connected):
  - `108182FE  02 3B 00 00 00 00 00 00`: Huawei reg-get, PSU addr 1, reg 0x023B (not in the Huawei doc)
  - `108181FE  01 00 00 00 00 00 E3 C7`: **cmd 0x81** (undocumented; 0x80 is the Huawei reg-set), PSU addr 1, reg 0x0100 (VOUT) = 0xE3C7/1024 = **56.94 V**
  - `108282FE  01 00 00 00 00 00 00 00`: reg-get, PSU addr 2, reg 0x0100
  - So the controller addresses PSU soft addresses 1 and 2 using the Huawei ID layout.
- ERR=0x00B00053: TEC=176, error-passive, LEC=5 (bit-dominant error). The MCU drives TX dominant but RX never sees it,
  so the transceiver's bus side is unpowered on the bench (most likely fed from the PSU/48V side).

## Update encryption (bootloader)
- TEA reduced to **8 rounds**, delta 0x9E3779B9, key = ASCII **"1234567123456789"** (little-endian words
  0x34333231 0x31373635 0x35343332 0x39383736). Decrypted in 8-byte blocks by FUN_08000280 → FUN_080002cc.
- The key sits in .data: copied from flash 0x08003688 (+4) to RAM 0x20000004 by the Keil scatter-load table at 0x08003664.
- tools/tea_decrypt.py decrypts or encrypts; `--find` locates the decrypted data inside the flash dump.
- Bootloader CRC: a table-driven CRC-16 (two 256-byte tables at DAT_08000278/27C, init 0xFFFF) in FUN_0800021c; the polynomial looks like Modbus.

## Application CAN layer (from Ghidra; addresses are in the app)
- CAN pins are PA4 (RX) / PA5 (TX), AF11 (corrected 2026-10-14; PA11/PA12 are keys, see `docs/history/notes/oem-hw-map.md`). Driver: init FUN_0800dba0, filter FUN_0800da88, RX FUN_0800dd78 (from IRQ20 at 0x0800d274), TX FUN_0800de88,
  send-and-wait FUN_08008328 (spins up to 0xFFF on TX status).
- The RX ISR drops IDs 0x1101007E / 0x1102007E (proto 0x22, PSU-to-PSU chatter?) and copies every other frame into a single-slot
  buffer at RAM 0x20000050, setting a flag at 0x2000004F.
- **Read:** ID 0x108082FE | addr<<16 (cmd 0x82), data `01 xx 00..`. **Write:** ID 0x108081FE | addr<<16 (**cmd 0x81**, not Huawei's 0x80).
- Poll cycle (FUN_0800bf4c; one register per request; PSU addr toggles 1<->2 after 0x24E):
  0100 0101 0102 0103 0104 010C 0239 0132 0133 0135 013A 0140 0142 0143 0170 0171 0172 0173 0175 0176 0177 0178
  0180 0182 0183 0184 0188 021D 022D 022F 024E | 023A 023B -> repeat
- Per-PSU state structs: PSU1 at 0x20000178 and PSU2 at 0x20000314 (readbacks; +0x17C = reg 0x22D value, +0x188 = VOUT).
  Charger state at 0x200005D8.
- Write request builder FUN_0800c428 → value packer FUN_08008ce0 (the format type comes from a per-register table in the PSU struct):
  - **0x0100 VOUT**: `01 00 00 00 00 00 HI LO`, raw = V * **1023** (not 1024). The setpoint comes from state+0x28 in 0.1 V units
    (FUN_0800d2b8). The live mailbox value 0xE3C7 = 57.0 V. It's sent only when the readback differs by more than 0.1 V.
  - **0x022D current limit**: raw ≈ amps * 1023 / 52 (52 A nominal), halved per PSU when both run (FUN_08008f64). The current
    (state+0x32, in A) soft-starts at 2 A and ramps +1 A every 2000 ticks up to the max at state+0x24; it's capped at 60 A
    (or at 50 A with a single PSU). Like VOUT, it's resent only when the readback differs.
  - **0x0132 on/off**: data[3] = 0 → on (Huawei: 0 = on, 1 = standby). On/off logic is in FUN_08008dc0, with hysteresis on
    state+6 (probably SOC) against +4/+3 and on voltage state+0x44 against +0x14/+0x12; a hard threshold 0x1A7 = 423 (42.3 V?) re-enables output.
- The float helpers FUN_080052d2/534c/5220/53f4/5418/540a/53c8/5472 are the Keil soft-float library (fmul/fdiv/fadd/i2f/f2i/...).
