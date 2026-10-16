// oem/src/app/modbus_rtu.h: Modbus RTU slave framing and function codes 03, 04, 06 and 16. Pure logic: the register map
// sits behind RegisterBank, bytes and time come from the caller, so it is all host-tested.
#pragma once
#include <cstddef>
#include <cstdint>

#include "app/crc16.h"

namespace oem {

constexpr size_t MB_MAX_FRAME = 256;
constexpr uint8_t MB_EX_ILLEGAL_FUNCTION = 1, MB_EX_ILLEGAL_ADDRESS = 2, MB_EX_ILLEGAL_VALUE = 3, MB_EX_BUSY = 6;

// The register map behind the slave. Implemented by OemRegisters (Task 3) and by fakes in tests.
struct RegisterBank {
  virtual bool read_input(uint16_t reg, uint16_t &value) = 0;    // false: no such register
  virtual bool read_holding(uint16_t reg, uint16_t &value) = 0;  // false: no such register
  // Validate the whole block, then apply all of it or none. 0 = done, else a Modbus exception code (2, 3 or 6).
  virtual uint8_t write_holding(uint16_t start, const uint16_t *values, uint16_t count) = 0;

 protected:
  ~RegisterBank() = default;
};

namespace mb_detail {

inline uint16_t be16(const uint8_t *p) { return uint16_t((p[0] << 8) | p[1]); }

// Appends the CRC (low byte first) and returns the total length.
inline size_t finish(uint8_t *resp, size_t n) {
  uint16_t crc = crc16_modbus(resp, n);
  resp[n++] = uint8_t(crc & 0xFF);
  resp[n++] = uint8_t(crc >> 8);
  return n;
}

inline size_t exception(uint8_t addr, uint8_t fc, uint8_t code, uint8_t *resp) {
  resp[0] = addr;
  resp[1] = uint8_t(fc | 0x80);
  resp[2] = code;
  return finish(resp, 3);
}

}  // namespace mb_detail

// One request in, one response out. Returns the response length, 0 for no reply (bad CRC, short frame, another
// address, or broadcast). resp must hold MB_MAX_FRAME bytes.
inline size_t modbus_handle(uint8_t address, const uint8_t *req, size_t len, uint8_t *resp, RegisterBank &bank) {
  using namespace mb_detail;
  if (len < 4) return 0;
  uint16_t crc = crc16_modbus(req, len - 2);
  if (req[len - 2] != (crc & 0xFF) || req[len - 1] != (crc >> 8)) return 0;
  const uint8_t addr = req[0];
  if (addr != address && addr != 0) return 0;
  const bool broadcast = addr == 0;
  const uint8_t fc = req[1];

  switch (fc) {
    case 3:
    case 4: {
      if (broadcast) return 0;
      if (len != 8) return exception(addr, fc, MB_EX_ILLEGAL_VALUE, resp);
      const uint16_t start = be16(req + 2), count = be16(req + 4);
      if (count < 1 || count > 125) return exception(addr, fc, MB_EX_ILLEGAL_VALUE, resp);
      resp[0] = addr;
      resp[1] = fc;
      resp[2] = uint8_t(count * 2);
      for (uint16_t i = 0; i < count; i++) {
        uint16_t v = 0;
        const uint32_t reg = uint32_t(start) + i;
        bool ok = reg <= 0xFFFF && (fc == 3 ? bank.read_holding(uint16_t(reg), v) : bank.read_input(uint16_t(reg), v));
        if (!ok) return exception(addr, fc, MB_EX_ILLEGAL_ADDRESS, resp);
        resp[3 + 2 * i] = uint8_t(v >> 8);
        resp[4 + 2 * i] = uint8_t(v & 0xFF);
      }
      return finish(resp, 3 + size_t(count) * 2);
    }
    case 6: {
      if (len != 8) return broadcast ? 0 : exception(addr, fc, MB_EX_ILLEGAL_VALUE, resp);
      const uint16_t v = be16(req + 4);
      uint8_t ex = bank.write_holding(be16(req + 2), &v, 1);
      if (broadcast) return 0;
      if (ex) return exception(addr, fc, ex, resp);
      for (size_t i = 0; i < 8; i++) resp[i] = req[i];
      return 8;
    }
    case 16: {
      uint16_t values[123];
      const uint16_t count = len >= 9 ? be16(req + 4) : 0;
      if (len < 9 || count < 1 || count > 123 || req[6] != 2 * count || len != size_t(9) + 2 * count) {
        return broadcast ? 0 : exception(addr, fc, MB_EX_ILLEGAL_VALUE, resp);
      }
      for (uint16_t i = 0; i < count; i++) values[i] = be16(req + 7 + 2 * i);
      uint8_t ex = bank.write_holding(be16(req + 2), values, count);
      if (broadcast) return 0;
      if (ex) return exception(addr, fc, ex, resp);
      for (size_t i = 0; i < 6; i++) resp[i] = req[i];
      return finish(resp, 6);
    }
    default:
      break;
  }
  return broadcast ? 0 : exception(addr, fc, MB_EX_ILLEGAL_FUNCTION, resp);
}

// Frame-end silence in ms: 3.5 characters of 11 bits, rounded up plus one tick for the 1 ms clock, but never under
// 5 ms: a main-loop pass takes 1-2 ms, so a shorter gap could split a frame read across two passes. A slave only
// needs the silence to be at least t3.5 (1.75 ms above 19200 baud).
inline uint32_t modbus_gap_ms(uint32_t baud) {
  constexpr uint32_t MIN_GAP_MS = 5;
  const uint32_t t35 = baud == 0 ? 0 : (38500u + baud - 1) / baud + 1;
  return t35 > MIN_GAP_MS ? t35 : MIN_GAP_MS;
}

// Assembles frames from bytes and the silence after them (M3, M4).
class FrameAssembler {
 public:
  void set_gap_ms(uint32_t gap) { gap_ = gap; }

  void push(const uint8_t *data, size_t n, uint32_t now) {
    if (n == 0) return;
    if (has_floor_) {
      if (int32_t(now - floor_) < 0) return;
      has_floor_ = false;  // past the cut: stop comparing, or the 32-bit clock wrap would revive it
    }
    // A new byte after a poll (or after a gap nobody polled, even one that ended in an overflow) starts a fresh frame.
    if (delivered_ || (busy_ && gap_passed(now))) restart();
    if (overflow_ || len_ + n > MB_MAX_FRAME) {
      overflow_ = true;  // dropped whole: ignore the rest until the next gap
      len_ = 0;
    } else {
      for (size_t i = 0; i < n; i++) buf_[len_++] = data[i];
    }
    busy_ = true;
    last_ = now;
  }

  // A complete frame once gap_ms has passed since the last byte; returns its length (0: none yet). The frame stays
  // valid until the next push(). Frames longer than MB_MAX_FRAME are dropped whole.
  size_t poll(uint32_t now, const uint8_t *&frame) {
    if (has_floor_ && int32_t(now - floor_) >= 0) has_floor_ = false;  // reached: drop it before the clock wraps
    if (!busy_ || !gap_passed(now)) return 0;
    busy_ = false;
    if (overflow_ || len_ == 0) {
      restart();
      return 0;
    }
    delivered_ = true;
    frame = buf_;
    return len_;
  }

  void discard_until(uint32_t t) {
    floor_ = t;
    has_floor_ = true;
    restart();
  }

 private:
  bool gap_passed(uint32_t now) const { return uint32_t(now - last_) >= gap_; }
  void restart() {
    len_ = 0;
    overflow_ = false;
    delivered_ = false;
    busy_ = false;
  }

  uint8_t buf_[MB_MAX_FRAME];
  size_t len_ = 0;
  uint32_t last_ = 0, gap_ = 4, floor_ = 0;
  bool busy_ = false, delivered_ = false, overflow_ = false, has_floor_ = false;
};

}  // namespace oem
