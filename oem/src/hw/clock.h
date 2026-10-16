#pragma once
#include <cstdint>
#include "hw/regs.h"

namespace hw {

void takeover_from_bootloader();  // stop and reset what the bootloader leaves running (called by clock_init)
bool clock_init();        // 60 MHz from the 12 MHz HXTAL; false if HXTAL or the PLL failed (then 16 MHz IRC16M)
uint32_t core_hz();
void fwdgt_init();        // same ~2 s timeout as the bootloader; the FWDGT keeps running
inline void fwdgt_feed() { reg(FWDGT_CTL) = 0xAAAA; }
void systick_init();      // 1 ms tick
uint32_t millis();
uint32_t reset_flags();   // RCU_RSTSCK bits 23..31 as latched by clock_init()
[[noreturn]] void system_reset();

}  // namespace hw
