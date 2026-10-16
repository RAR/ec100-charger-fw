// oem/src/app/settings_store.h: settings log in two 1 KB flash pages (spec §5.2). Records are appended to the
// active page; when it is full, the other page is erased and the next record goes there. Load picks the valid
// record with the highest sequence number, so a torn write (power lost mid-record) only loses that record.
//
// Record, 48 bytes = 6 doublewords, little-endian:
//   0 magic 'O','S' | 2 version | 3 register count | 4 seq u32 | 8 hr[17] u16 | 42 zero x4 | 46 CRC-16 of bytes 0..45
// The CRC sits in the last doubleword, which is programmed last.
#pragma once
#include <cstdint>

#include "app/crc16.h"
#include "app/flash_io.h"
#include "app/settings.h"

namespace oem {

constexpr uint32_t SETTINGS_PAGE[2] = {0x0801F800, 0x0801FC00};
constexpr uint32_t SETTINGS_PAGE_BYTES = 1024;
constexpr uint32_t RECORD_BYTES = 48;
constexpr uint32_t RECORD_SLOTS = SETTINGS_PAGE_BYTES / RECORD_BYTES;  // 21
constexpr uint8_t RECORD_VERSION = 1;
constexpr int RECORD_MAX_REGS = (RECORD_BYTES - 2 - 8) / 2;  // 19: the registers that fit before the CRC
static_assert(HR_COUNT <= RECORD_MAX_REGS, "the settings no longer fit one record");

inline void encode_record(const Settings &s, uint32_t seq, uint8_t out[RECORD_BYTES]) {
  for (uint32_t i = 0; i < RECORD_BYTES; i++) out[i] = 0;
  out[0] = 'O';
  out[1] = 'S';
  out[2] = RECORD_VERSION;
  out[3] = HR_COUNT;
  for (int i = 0; i < 4; i++) out[4 + i] = uint8_t(seq >> (8 * i));
  for (int i = 0; i < HR_COUNT; i++) {
    out[8 + 2 * i] = uint8_t(s.hr[i]);
    out[9 + 2 * i] = uint8_t(s.hr[i] >> 8);
  }
  const uint16_t crc = crc16_modbus(out, RECORD_BYTES - 2);
  out[RECORD_BYTES - 2] = uint8_t(crc);
  out[RECORD_BYTES - 1] = uint8_t(crc >> 8);
}

// A record from a firmware with fewer settings keeps those and takes the defaults for the rest; one with more keeps
// the first HR_COUNT. So adding a setting never resets the others (the caller clamps the result as a whole).
inline bool decode_record(const uint8_t *in, Settings &s, uint32_t &seq) {
  const int count = in[3];
  if (in[0] != 'O' || in[1] != 'S' || in[2] != RECORD_VERSION || count < 1 || count > RECORD_MAX_REGS) return false;
  if (crc16_modbus(in, RECORD_BYTES - 2) != uint16_t(in[RECORD_BYTES - 2] | in[RECORD_BYTES - 1] << 8)) return false;
  seq = uint32_t(in[4]) | uint32_t(in[5]) << 8 | uint32_t(in[6]) << 16 | uint32_t(in[7]) << 24;
  s = default_settings();
  for (int i = 0; i < HR_COUNT && i < count; i++) s.hr[i] = uint16_t(in[8 + 2 * i] | in[9 + 2 * i] << 8);
  return true;
}

inline bool erased(const uint8_t *p, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    if (p[i] != 0xFF) return false;
  }
  return true;
}

class SettingsStore {
 public:
  explicit SettingsStore(FlashIo &io) : io_(io) {}

  // The newest valid record, clamped (spec §2.3). False if neither page holds one: the caller then uses
  // default_settings() (phase 3 first tries the stock v1 record at 0x0801F000).
  bool load(Settings &out) {
    loaded_ = true;
    page_ = -1;
    seq_ = 0;
    for (int p = 0; p < 2; p++) {
      for (uint32_t k = 0; k < RECORD_SLOTS; k++) {
        Settings s;
        uint32_t seq;
        if (!decode_record(io_.read(slot_addr(p, k)), s, seq)) continue;
        if (page_ < 0 || seq > seq_) {
          page_ = p;
          seq_ = seq;
          out = s;
        }
      }
    }
    if (page_ < 0) return false;
    apply_clamps(out);
    return true;
  }

  // Appends a record after the last used slot of the active page. When that page is full or the write fails,
  // the other page is erased and the record goes into its first slot; the page holding the newest record is never
  // the one erased. If load() hasn't run, it runs first, so the only valid record is never mistaken for free space.
  // seq is a u32 and wraps only after ~4e9 saves, far beyond the flash's endurance.
  bool save(const Settings &s) {
    if (!loaded_) {
      Settings scratch;
      load(scratch);
    }
    uint8_t rec[RECORD_BYTES];
    encode_record(s, seq_ + 1, rec);
    if (page_ >= 0) {
      const uint32_t slot = next_free(page_);
      if (slot < RECORD_SLOTS && write(page_, slot, rec)) return true;
    }
    const int other = page_ < 0 ? 0 : 1 - page_;
    return io_.erase_page(SETTINGS_PAGE[other]) && write(other, 0, rec);
  }

  uint32_t seq() const { return seq_; }
  int active_page() const { return page_; }

 private:
  static uint32_t slot_addr(int page, uint32_t slot) { return SETTINGS_PAGE[page] + slot * RECORD_BYTES; }

  static bool same(const uint8_t *a, const uint8_t *b) {
    for (uint32_t i = 0; i < RECORD_BYTES; i++) {
      if (a[i] != b[i]) return false;
    }
    return true;
  }

  bool write(int page, uint32_t slot, const uint8_t *rec) {
    const uint32_t addr = slot_addr(page, slot);
    if (!io_.program(addr, rec, RECORD_BYTES) || !same(io_.read(addr), rec)) return false;
    page_ = page;
    seq_++;
    return true;
  }

  // The slot after the last one that isn't fully erased: never write into a hole left before a torn record.
  uint32_t next_free(int page) {
    uint32_t next = 0;
    for (uint32_t k = 0; k < RECORD_SLOTS; k++) {
      if (!erased(io_.read(slot_addr(page, k)), RECORD_BYTES)) next = k + 1;
    }
    return next;
  }

  FlashIo &io_;
  bool loaded_ = false;
  int page_ = -1;  // page holding the newest valid record, -1 if none
  uint32_t seq_ = 0;
};

}  // namespace oem
