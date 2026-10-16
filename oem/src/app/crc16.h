// oem/src/app/crc16.h: Modbus CRC-16 (poly 0xA001 reflected, init 0xFFFF), the same CRC as tools/oem_image.py and the
// stock bootloader. Used by the settings store now and by Modbus RTU in phase 3.
#pragma once
#include <cstddef>
#include <cstdint>

namespace oem {

inline uint16_t crc16_modbus(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1u) ? uint16_t((crc >> 1) ^ 0xA001u) : uint16_t(crc >> 1);
  }
  return crc;
}

}  // namespace oem
