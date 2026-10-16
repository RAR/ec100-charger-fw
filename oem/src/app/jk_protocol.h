// oem/src/app/jk_protocol.h: the JK-B BMS RS485 protocol (4E 57 frames) that the board reads as master
// (docs/history/notes/jk-bms-protocol.md §2-4). A reply is a 4E 57 header, a big-endian length N (total = N + 2), 9
// more header bytes, ID-tagged records from byte 11, a 4 byte record number, 68 00 00 and a 16-bit sum of bytes 0..N-1.
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace oem::jk {

constexpr size_t REQUEST_LEN = 21;
constexpr size_t MAX_FRAME = 400;
// Smallest plausible reply: header (11) + 0x79 with one cell (5) + 0x83 (3) + 0x85 (2) + 0x8B (3) + 0x8C (3) +
// record number, end byte, unused, checksum (9).
constexpr size_t MIN_FRAME = 36;
constexpr size_t RECORDS_START = 11;
constexpr size_t TRAILER_LEN = 9;  // record number (4), 68, 00 00, checksum (2)

inline size_t build_request(uint8_t source_byte, uint8_t *out) {
  const uint8_t req[19] = {0x4E, 0x57, 0x00, 0x13, 0, 0, 0, 0, 0x06, source_byte, 0, 0, 0, 0, 0, 0, 0x68, 0, 0};
  uint16_t sum = 0;
  for (size_t i = 0; i < 19; i++) {
    out[i] = req[i];
    sum = uint16_t(sum + req[i]);
  }
  out[19] = uint8_t(sum >> 8);
  out[20] = uint8_t(sum);
  return REQUEST_LEN;
}

enum class Error : uint8_t { OK, SHORT, HEADER, LENGTH, CHECKSUM, COMMAND, MISSING, CELLS };

struct Data {
  float pack_v;      // V
  int soc;           // %, -1 when out of range
  float max_cell_v;  // V, NaN when no cell reads
  uint16_t alarms;   // 0x8B
  uint16_t status;   // 0x8C
  int cells;         // 0x79 LEN / 3
  size_t walk_end;   // byte offset where the record walk stopped (frame length - 9 when every record was consumed)
};

// Total frame length from the length field, or 0 when it is implausible.
inline size_t frame_len(uint8_t hi, uint8_t lo) {
  const size_t total = size_t(hi << 8 | lo) + 2;
  return total >= MIN_FRAME && total <= MAX_FRAME ? total : 0;
}

// Value length of a fixed-size record, 0 for an unknown ID (0x79 is variable and handled by the caller).
inline size_t record_len(uint8_t id) {
  switch (id) {
    case 0x85: case 0x86: case 0x9D: case 0xA9: case 0xAB: case 0xAC: case 0xAE: case 0xAF: case 0xB1: case 0xB3:
    case 0xB8: case 0xC0:
      return 1;
    case 0x80: case 0x81: case 0x82: case 0x83: case 0x84: case 0x87: case 0x8A: case 0x8B: case 0x8C: case 0x8E:
    case 0x8F: case 0x90: case 0x91: case 0x92: case 0x93: case 0x94: case 0x95: case 0x96: case 0x97: case 0x98:
    case 0x99: case 0x9A: case 0x9B: case 0x9C: case 0x9E: case 0x9F: case 0xA0: case 0xA1: case 0xA2: case 0xA3:
    case 0xA4: case 0xA5: case 0xA6: case 0xA7: case 0xA8: case 0xAD: case 0xB0:
      return 2;
    case 0x89: case 0xAA: case 0xB5: case 0xB6: case 0xB9:
      return 4;
    case 0xB4:
      return 8;
    case 0xB2:
      return 10;
    case 0xB7:
      return 15;
    case 0xBA:
      return 24;
    default:
      return 0;
  }
}

// A whole reply of exactly N + 2 bytes. Walks the records from byte 11 up to the record number; an unknown ID or a
// record that doesn't fit stops the walk. 0x79 0x83 0x85 0x8B 0x8C must all have been seen.
inline Error parse_reply(const uint8_t *f, size_t n, Data &d) {
  if (n < 4) return Error::SHORT;
  if (f[0] != 0x4E || f[1] != 0x57) return Error::HEADER;
  const size_t total = frame_len(f[2], f[3]);
  if (total == 0 || total != n) return Error::LENGTH;
  uint16_t sum = 0;
  for (size_t i = 0; i < n - 2; i++) sum = uint16_t(sum + f[i]);
  if (f[n - 2] != uint8_t(sum >> 8) || f[n - 1] != uint8_t(sum)) return Error::CHECKSUM;
  if (f[8] != 0x06) return Error::COMMAND;
  const size_t end = n - TRAILER_LEN;  // records end before the record number
  unsigned seen = 0;                   // 1 = 79, 2 = 83, 4 = 85, 8 = 8B, 16 = 8C
  int cells79 = -1, cells8a = -1;
  d = {NAN, -1, NAN, 0, 0, 0, 0};
  size_t i = RECORDS_START;
  while (i < end) {
    const uint8_t id = f[i];
    size_t len;
    if (id == 0x79) {
      if (i + 1 >= end) break;
      len = 1 + size_t(f[i + 1]);
      if (f[i + 1] % 3 != 0 || i + 1 + len > end) break;
    } else {
      len = record_len(id);
      if (len == 0 || i + 1 + len > end) break;
    }
    const uint8_t *v = f + i + 1;
    switch (id) {
      case 0x79: {
        cells79 = f[i + 1] / 3;
        uint16_t best = 0;
        for (int c = 0; c < cells79; c++) {
          const uint16_t mv = uint16_t(v[1 + 3 * c + 1] << 8 | v[1 + 3 * c + 2]);
          if (mv != 0 && mv < 5000 && mv > best) best = mv;
        }
        if (best) d.max_cell_v = float(best) / 1000.0f;
        seen |= 1;
        break;
      }
      case 0x83: d.pack_v = float(v[0] << 8 | v[1]) / 100.0f; seen |= 2; break;
      case 0x85: d.soc = v[0] <= 100 ? int(v[0]) : -1; seen |= 4; break;
      case 0x8A: cells8a = v[0] << 8 | v[1]; break;
      case 0x8B: d.alarms = uint16_t(v[0] << 8 | v[1]); seen |= 8; break;
      case 0x8C: d.status = uint16_t(v[0] << 8 | v[1]); seen |= 16; break;
      default: break;
    }
    i += 1 + len;
  }
  if (seen != 31) return Error::MISSING;
  if (cells8a >= 0 && cells8a != cells79) return Error::CELLS;
  if (cells79 < 1) return Error::CELLS;
  d.walk_end = i;
  d.cells = cells79;
  return Error::OK;
}

}  // namespace oem::jk
