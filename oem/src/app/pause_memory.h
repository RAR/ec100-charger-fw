// oem/src/app/pause_memory.h: a pause survives a watchdog reset (a hang mustn't resume charging) and a Modbus reboot
// (HR 120), but not a power-cycle or any other reset. The word lives in an RTC backup register, which the bootloader
// leaves alone; it jumps to the app without another reset, so the reset flags still say why the chip restarted.
#pragma once
#include <cstdint>

namespace oem {

constexpr uint32_t PAUSE_MAGIC = 0x50415553;         // "PAUS": paused, kept across a watchdog reset
constexpr uint32_t PAUSE_REBOOT_MAGIC = 0x50415552;  // "PAUR": written just before a Modbus reboot while paused

inline uint32_t pause_word(bool paused) { return paused ? PAUSE_MAGIC : 0; }
inline bool restore_pause(bool watchdog_reset, bool software_reset, uint32_t word) {
  return (watchdog_reset && word == PAUSE_MAGIC) || (software_reset && word == PAUSE_REBOOT_MAGIC);
}

}  // namespace oem
