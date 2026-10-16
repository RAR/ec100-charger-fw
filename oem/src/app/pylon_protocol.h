// oem/src/app/pylon_protocol.h: Pylontech RS485 framing (ASCII) and the stock charger's dialect: the CID2
// 0x61/0x62/0x63 requests and the reply fields we use.
// docs/history/superpowers/specs/2026-10-15-oem-pylontech-source-design.md §2-3; layouts from
// docs/history/notes/pylon-stock-protocol.md and the user's YamBMS charger proxy.
#pragma once
#include <cstddef>
#include <cstdint>

namespace oem {
namespace pylon {

constexpr uint8_t CID1 = 0x46;
constexpr uint8_t CID_ANALOG = 0x61, CID_ALARM = 0x62, CID_MGMT = 0x63;
constexpr size_t REQUEST_LEN = 18;     // "~201246xx0000CCCC\r"
constexpr size_t FRAME_OVERHEAD = 18;  // every character but INFO
constexpr size_t MAX_FRAME = 256;
constexpr size_t MAX_INFO = (MAX_FRAME - FRAME_OVERHEAD) / 2;  // decoded INFO bytes

// The fewest INFO bytes each reply must carry for the fields we read.
inline size_t min_info(uint8_t cid2) { return cid2 == CID_ANALOG ? 13 : cid2 == CID_ALARM ? 4 : 9; }

// LENGTH: LCHKSUM (top nibble) over LENID's three nibbles, then LENID = ASCII characters of INFO.
inline uint16_t length_field(uint16_t lenid) {
  const unsigned s = ((lenid >> 8) & 0xFu) + ((lenid >> 4) & 0xFu) + (lenid & 0xFu);
  return uint16_t((((~s + 1u) & 0xFu) << 12) | (lenid & 0x0FFFu));
}

// CHKSUM: two's complement of the ASCII sum of everything between '~' and the checksum.
inline uint16_t checksum(const char *s, size_t n) {
  uint32_t sum = 0;
  for (size_t i = 0; i < n; i++) sum += uint8_t(s[i]);
  return uint16_t((~sum + 1u) & 0xFFFFu);
}

inline char hex_digit(unsigned v) { return "0123456789ABCDEF"[v & 0xFu]; }
inline int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}
// The byte in two hex characters at p (callers have checked they are hex).
inline int hex_byte(const char *p) { return (hex_value(p[0]) << 4) | hex_value(p[1]); }

// The stock request: VER 0x20, ADR 0x12, CID1 0x46, cid2, empty INFO. Writes REQUEST_LEN characters (no NUL).
inline size_t build_request(uint8_t cid2, char *out) {
  const char head[12] = {'2', '0', '1', '2', '4', '6', hex_digit(cid2 >> 4), hex_digit(cid2), '0', '0', '0', '0'};
  out[0] = '~';
  for (int i = 0; i < 12; i++) out[1 + i] = head[i];
  const uint16_t c = checksum(out + 1, 12);
  for (int i = 0; i < 4; i++) out[13 + i] = hex_digit(unsigned(c) >> (12 - 4 * i));
  out[17] = '\r';
  return REQUEST_LEN;
}

enum class Error : uint8_t { OK, FORMAT, LENGTH, LCHKSUM, CHKSUM, CID1, RTN, SHORT };

// One reply frame, '~' to '\r' (n characters). VER and ADR aren't checked. On OK, the INFO bytes are decoded into
// info[] (info_len of them, at most MAX_INFO); otherwise info_len is 0.
inline Error parse_reply(const char *f, size_t n, size_t need, uint8_t *info, size_t &info_len) {
  info_len = 0;
  if (n < FRAME_OVERHEAD || n > MAX_FRAME || f[0] != '~' || f[n - 1] != '\r') return Error::FORMAT;
  for (size_t i = 1; i < n - 1; i++) {
    if (hex_value(f[i]) < 0) return Error::FORMAT;
  }
  const uint16_t length = uint16_t((hex_byte(f + 9) << 8) | hex_byte(f + 11));
  const uint16_t lenid = length & 0x0FFFu;
  if (n != FRAME_OVERHEAD + lenid || (lenid & 1u)) return Error::LENGTH;
  if (length_field(lenid) != length) return Error::LCHKSUM;
  const uint16_t sent = uint16_t((hex_byte(f + n - 5) << 8) | hex_byte(f + n - 3));
  if (checksum(f + 1, n - 6) != sent) return Error::CHKSUM;
  if (hex_byte(f + 5) != CID1) return Error::CID1;
  if (hex_byte(f + 7) != 0) return Error::RTN;
  if (lenid / 2u < need) return Error::SHORT;
  info_len = lenid / 2u;
  for (size_t i = 0; i < info_len; i++) info[i] = uint8_t(hex_byte(f + 13 + 2 * i));
  return Error::OK;
}

inline uint16_t be16(const uint8_t *p) { return uint16_t((p[0] << 8) | p[1]); }

struct Analog {
  float battery_v;   // V
  int soc;           // %, -1 when out of range
  float max_cell_v;  // V
};
// 0x61. The current at bytes 2-3 isn't used: its unit is unconfirmed (spec §1).
inline Analog decode_analog(const uint8_t *info) {
  return {be16(info) / 1000.0f, info[4] <= 100 ? int(info[4]) : -1, be16(info + 11) / 1000.0f};
}

// 0x62: stock v2's error-12 mask over the protection bytes.
inline bool decode_alarm(const uint8_t *info) { return (info[2] & 0xAAu) != 0 || (info[3] & 0x68u) != 0; }

struct Mgmt {
  float cvl, dvl;  // V
  float ccl, dcl;  // A
  uint8_t status;
  bool charge_enable;  // status bit 7
};
// 0x63.
inline Mgmt decode_mgmt(const uint8_t *info) {
  return {be16(info) / 1000.0f, be16(info + 2) / 1000.0f, be16(info + 4) / 10.0f, be16(info + 6) / 10.0f, info[8],
          (info[8] & 0x80u) != 0};
}

}  // namespace pylon
}  // namespace oem
