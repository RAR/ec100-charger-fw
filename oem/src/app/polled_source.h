// oem/src/app/polled_source.h: the interface of a BMS the board polls on RS485 (the board is the master; no Modbus
// server meanwhile). main.cpp drives whichever one HR 6 selects through this; see pylon_source.h.
#pragma once
#include <cstddef>
#include <cstdint>

#include "app/limit_source.h"
#include "app/settings.h"

namespace oem {

constexpr size_t POLL_REQ_MAX = 32;  // longest request any poller writes

class PolledSource {
 public:
  virtual void reset() = 0;                                           // a mode change: stale, start over
  virtual void push(const uint8_t *data, size_t n, uint32_t now) = 0;  // bytes received since the last call
  virtual size_t poll(uint32_t now, uint8_t *out) = 0;                // a due request into out, else 0
  virtual Limits limits(const Settings &s, uint32_t now) const = 0;
  virtual uint32_t baud() const = 0;

 protected:
  ~PolledSource() = default;  // never deleted through the base (no heap)
};

}  // namespace oem
