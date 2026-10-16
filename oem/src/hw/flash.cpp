// oem/src/hw/flash.cpp: FMC unlock / page erase / doubleword program / lock as stock (0800e670, 0800e538, 0800e470,
// 0800e52c), except that the error flags really are cleared (stock's flag_clear(0x307) writes a no-op bit) and every
// operation's result is checked. Interrupts are off while the FMC is unlocked, as in stock's append (08006018).
#include "hw/flash.h"

#include "hw/regs.h"

namespace hw {
namespace {

constexpr uint32_t BUSY = 1u << 0, PGERR = 1u << 2, PGAERR = 1u << 3, WPERR = 1u << 4, ENDF = 1u << 5;
constexpr uint32_t PG = 1u << 0, PER = 1u << 1, START = 1u << 6, LK = 1u << 7;
constexpr uint32_t STORE_FIRST = 0x0801F800, STORE_END = 0x08020000;  // spec §5.2: the only pages we may touch

bool in_store(uint32_t addr, uint32_t len) {
  return addr >= STORE_FIRST && addr + len <= STORE_END && addr + len > addr;
}

void unlock() {
  __asm volatile("cpsid i" ::: "memory");
  if (reg(FMC_CTL) & LK) {
    reg(FMC_KEY) = 0x45670123u;
    reg(FMC_KEY) = 0xCDEF89ABu;
  }
  reg(FMC_STAT) = ENDF | WPERR | PGAERR | PGERR;  // write 1 to clear
}

void lock() {
  reg(FMC_CTL) |= LK;
  __asm volatile("cpsie i" ::: "memory");
}

// Stock's ready wait: 0x100000 polls, then the error flags.
bool ready() {
  for (uint32_t t = 0; t < 0x100000; t++) {
    const uint32_t s = reg(FMC_STAT);
    if (!(s & BUSY)) return !(s & (WPERR | PGERR | PGAERR));
  }
  return false;
}

}  // namespace

bool Flash::erase_page(uint32_t addr) {
  if (!in_store(addr, 1024) || (addr & 0x3FFu)) return false;
  unlock();
  bool ok = ready();
  if (ok) {
    reg(FMC_CTL) |= PER;
    reg(FMC_ADDR) = addr;
    reg(FMC_CTL) |= START;
    ok = ready();
    reg(FMC_CTL) &= ~PER;
  }
  lock();
  return ok;
}

bool Flash::program(uint32_t addr, const uint8_t *data, uint32_t len) {
  if (!in_store(addr, len) || (addr & 7u) || (len & 7u)) return false;
  bool ok = true;
  for (uint32_t i = 0; ok && i < len; i += 8) {
    const uint8_t *d = data + i;
    const uint32_t lo = uint32_t(d[0]) | uint32_t(d[1]) << 8 | uint32_t(d[2]) << 16 | uint32_t(d[3]) << 24;
    const uint32_t hi = uint32_t(d[4]) | uint32_t(d[5]) << 8 | uint32_t(d[6]) << 16 | uint32_t(d[7]) << 24;
    unlock();
    ok = ready();
    if (ok) {
      reg(FMC_CTL) |= PG;
      reg(addr + i) = lo;
      reg(addr + i + 4) = hi;
      ok = ready();
      reg(FMC_CTL) &= ~PG;
      __asm volatile("isb" ::: "memory");
    }
    lock();
  }
  return ok;
}

}  // namespace hw
