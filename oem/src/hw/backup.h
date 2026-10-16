// oem/src/hw/backup.h: one RTC backup register. It keeps its value across watchdog and software resets; a power-down
// loses it with the backup domain.
#pragma once
#include <cstdint>
#include "hw/regs.h"

namespace hw {

// PMU and BKP clocks, then BKPWEN, as stock does (docs/history/notes/oem-init-sequences.md §3b, enable codes 0x71C /
// 0x71F).
inline void backup_init() {
  reg(RCU_APB1EN) |= (1u << 28) | (1u << 31);
  reg(PMU_CTL0) |= 1u << 8;
}
inline uint32_t backup_word() { return reg(RTC_BKP4); }
inline void set_backup_word(uint32_t v) { reg(RTC_BKP4) = v; }

}  // namespace hw
