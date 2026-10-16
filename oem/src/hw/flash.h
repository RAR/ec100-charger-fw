// oem/src/hw/flash.h: internal flash (FMC) for the settings store: 1 KB page erase and doubleword programming,
// the stock sequence (docs/history/notes/oem-adc-flash-sequences.md §5). Refuses any address outside the two settings
// pages.
#pragma once
#include <cstdint>

#include "app/flash_io.h"

namespace hw {

class Flash : public oem::FlashIo {
 public:
  const uint8_t *read(uint32_t addr) override { return reinterpret_cast<const uint8_t *>(addr); }
  bool erase_page(uint32_t addr) override;
  bool program(uint32_t addr, const uint8_t *data, uint32_t len) override;
};

}  // namespace hw
