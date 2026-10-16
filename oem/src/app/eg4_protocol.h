// oem/src/app/eg4_protocol.h: the EG4 LL BMS RS485 protocol, a plain Modbus RTU slave at 0x10 that the board reads as
// master (docs/history/notes/eg4-bms-protocol.md §2-4). Two reads cover everything used: block A (registers 0x00-0x11:
// pack V and the 16 cells) and block B (0x12-0x26: max charging current, SOC, protection bits).
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/crc16.h"

namespace oem::eg4 {

constexpr uint8_t ADDR = 0x10;
constexpr size_t REQUEST_LEN = 8;
constexpr uint16_t A_FIRST = 0x0000, A_COUNT = 0x12;  // block A: 18 registers, 41 byte reply
constexpr uint16_t B_FIRST = 0x0012, B_COUNT = 0x15;  // block B: 21 registers, 47 byte reply
constexpr size_t MAX_REPLY = 5 + 2 * B_COUNT;

constexpr size_t reply_len(uint16_t count) { return 5 + 2 * size_t(count); }

// addr 03 first count crc (low byte first). Returns REQUEST_LEN.
inline size_t build_request(uint16_t first_reg, uint16_t count, uint8_t *out) {
  out[0] = ADDR;
  out[1] = 0x03;
  out[2] = uint8_t(first_reg >> 8);
  out[3] = uint8_t(first_reg);
  out[4] = uint8_t(count >> 8);
  out[5] = uint8_t(count);
  const uint16_t crc = crc16_modbus(out, 6);
  out[6] = uint8_t(crc);
  out[7] = uint8_t(crc >> 8);
  return REQUEST_LEN;
}

enum class Error : uint8_t { OK, SHORT, ADDR, FUNC, COUNT, CRC };

// A reply to a read of `count` registers: exactly 5 + 2 * count bytes. On OK, data points at the first register.
inline Error parse_reply(const uint8_t *f, size_t n, uint16_t count, const uint8_t *&data) {
  data = nullptr;
  if (n < 5) return Error::SHORT;
  if (f[0] != ADDR) return Error::ADDR;
  if (f[1] != 0x03) return Error::FUNC;  // includes an exception reply (0x83)
  if (f[2] != 2 * count) return Error::COUNT;
  if (n != reply_len(count)) return Error::SHORT;
  const uint16_t crc = crc16_modbus(f, n - 2);
  if (f[n - 2] != uint8_t(crc) || f[n - 1] != uint8_t(crc >> 8)) return Error::CRC;
  data = f + 3;
  return Error::OK;
}

inline uint16_t word(const uint8_t *data, size_t reg_off) {
  return uint16_t(data[2 * reg_off] << 8 | data[2 * reg_off + 1]);
}

struct BlockA {
  float pack_v;      // V, 0 = the all-zero reply quirk
  float max_cell_v;  // V, NaN when no cell reads
};

// data = block A's first register (0x0000): pack V at 0 (0.01 V), cells at 2..17 (mV, 0 and >= 5000 ignored).
inline BlockA decode_a(const uint8_t *data) {
  BlockA a{float(word(data, 0)) / 100.0f, NAN};
  uint16_t best = 0;
  for (size_t i = 2; i < 18; i++) {
    const uint16_t mv = word(data, i);
    if (mv != 0 && mv < 5000 && mv > best) best = mv;
  }
  if (best) a.max_cell_v = float(best) / 1000.0f;
  return a;
}

struct BlockB {
  int soc;              // %, -1 when out of range
  float ccl;            // A, register 0x0016
  uint16_t protection;  // register 0x001B
};

// data = block B's first register (0x0012): 0x16 at 4, 0x18 at 6, 0x1B at 9.
inline BlockB decode_b(const uint8_t *data) {
  const uint16_t soc = word(data, 0x18 - 0x12);
  return {soc <= 100 ? int(soc) : -1, float(word(data, 0x16 - 0x12)), word(data, 0x1B - 0x12)};
}

}  // namespace oem::eg4
