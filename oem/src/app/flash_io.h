// oem/src/app/flash_io.h: what the settings store needs from internal flash. hw::Flash implements it on the chip;
// the host tests use a RAM-backed fake.
#pragma once
#include <cstdint>

namespace oem {

class FlashIo {
 public:
  virtual const uint8_t *read(uint32_t addr) = 0;  // flash is memory-mapped: a pointer to addr
  virtual bool erase_page(uint32_t addr) = 0;      // 1 KB page at addr
  // len a multiple of 8 and addr 8-aligned, into erased flash; one doubleword at a time, in order.
  virtual bool program(uint32_t addr, const uint8_t *data, uint32_t len) = 0;
};

}  // namespace oem
