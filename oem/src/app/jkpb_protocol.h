// oem/src/app/jkpb_protocol.h: the JK-PB BMS RS485 protocol (JK RS485 Modbus V1.0, port protocol 001) that the board
// reads as master (docs/history/notes/jk-pb-protocol.md §3-6). A request is an FC 0x10 write of 0x0000 to one trigger
// register; the pack answers with a 300-byte record (55 AA EB 90, type, counter, data, sum of bytes 0..298 at 299) and
// then the 8-byte FC10 reply `addr 10 RH RL 00 01 CRClo CRChi`. All record fields are little-endian.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "app/crc16.h"

namespace oem::jkpb {

constexpr size_t RECORD_LEN = 300;
constexpr size_t TRAILER_LEN = 8;
constexpr size_t REQUEST_LEN = 11;
constexpr uint16_t REG_CELL_INFO = 0x1620;  // answered by a type 0x02 record
constexpr uint16_t REG_SETTINGS = 0x161E;   // answered by a type 0x01 record
constexpr uint8_t TYPE_SETTINGS = 0x01;
constexpr uint8_t TYPE_CELL_INFO = 0x02;
constexpr uint8_t HEADER[4] = {0x55, 0xAA, 0xEB, 0x90};

// addr 10 RH RL 00 01 02 00 00 CRClo CRChi: write 0x0000 to one register.
inline size_t build_request(uint8_t addr, uint16_t reg, uint8_t *out) {
  const uint8_t req[9] = {addr, 0x10, uint8_t(reg >> 8), uint8_t(reg), 0x00, 0x01, 0x02, 0x00, 0x00};
  for (size_t i = 0; i < 9; i++) out[i] = req[i];
  const uint16_t crc = crc16_modbus(out, 9);
  out[9] = uint8_t(crc);
  out[10] = uint8_t(crc >> 8);
  return REQUEST_LEN;
}

// A RECORD_LEN record: header, the expected type, byte 299 = sum of bytes 0..298 mod 256.
inline bool record_ok(const uint8_t *r, uint8_t type) {
  for (size_t i = 0; i < 4; i++)
    if (r[i] != HEADER[i]) return false;
  if (r[4] != type) return false;
  uint8_t sum = 0;
  for (size_t i = 0; i < RECORD_LEN - 1; i++) sum = uint8_t(sum + r[i]);
  return r[RECORD_LEN - 1] == sum;
}

// The FC10 reply that follows a record: the polled address and register, quantity 1, a good CRC.
inline bool trailer_ok(const uint8_t *t, uint8_t addr, uint16_t reg) {
  if (t[0] != addr || t[1] != 0x10 || t[2] != uint8_t(reg >> 8) || t[3] != uint8_t(reg) || t[4] != 0x00 ||
      t[5] != 0x01)
    return false;
  const uint16_t crc = crc16_modbus(t, 6);
  return t[6] == uint8_t(crc) && t[7] == uint8_t(crc >> 8);
}

inline uint16_t u16le(const uint8_t *p) { return uint16_t(p[0] | p[1] << 8); }
inline uint32_t u32le(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}

struct CellInfo {
  float pack_v;      // V
  int soc;           // %, -1 when out of range
  float max_cell_v;  // V, NaN when no cell reads
  uint32_t alarms;   // 0x12A0 bitmask
  bool charge_mos;   // the charge MOSFET is on (byte 198 == 1)
  uint8_t stage;     // 0 bulk, 1 absorption, 2 float
};

struct ChargeSettings {
  int cells;
  uint32_t rcv_mv;   // CellRCV, charge voltage per cell
  uint32_t rfv_mv;   // CellRFV, float voltage per cell
  uint32_t coc_ma;   // CurBatCOC, max charge current
  bool charge_switch;
};

// A type 0x02 record (record_ok already checked). Cells u16 mV at 6 + 2n (0 = absent, >= 5000 ignored), pack u32 mV
// at 150, alarms u32 at 166, SOC u8 at 173, charge MOSFET u8 at 198, charge stage u8 at 280.
inline CellInfo decode_cell_info(const uint8_t *r) {
  uint16_t best = 0;
  for (size_t n = 0; n < 32; n++) {
    const uint16_t mv = u16le(r + 6 + 2 * n);
    if (mv != 0 && mv < 5000 && mv > best) best = mv;
  }
  return {float(u32le(r + 150)) / 1000.0f,
          r[173] <= 100 ? int(r[173]) : -1,
          best ? float(best) / 1000.0f : NAN,
          u32le(r + 166),
          r[198] == 1,
          r[280]};
}

// A type 0x01 record (record_ok already checked): CellRCV at 38, CellRFV at 42, CurBatCOC at 50, cell count at 114,
// charge switch at 118 (u32 each). False when the cell count is outside 1..32 or RCV / RFV outside 2500..4500 mV.
inline bool decode_settings(const uint8_t *r, ChargeSettings &s) {
  const uint32_t cells = u32le(r + 114), rcv = u32le(r + 38), rfv = u32le(r + 42);
  if (cells < 1 || cells > 32) return false;
  if (rcv < 2500 || rcv > 4500 || rfv < 2500 || rfv > 4500) return false;
  s = {int(cells), rcv, rfv, u32le(r + 50), u32le(r + 118) == 1};
  return true;
}

}  // namespace oem::jkpb
