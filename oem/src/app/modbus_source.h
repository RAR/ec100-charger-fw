// oem/src/app/modbus_source.h: the Modbus limit source (spec 2026-10-15-oem-modbus-source-design.md). A master writes
// HR 101-107 and the HR 100 heartbeat; a write of all eight starting at HR 100 hands them here, and the limits are
// that snapshot while it is at most EXT_FRESH_MS old. Otherwise Panel limits capped at Fallback current, flagged stale
// (E15), unless the last snapshot was a stop (charge allowed off, alarm, blocking or CCL 0): that holds charging off
// until a fresh snapshot or a mode change (forget_stop).
#pragma once
#include <cmath>
#include <cstdint>

#include "app/limit_source.h"
#include "app/settings.h"

namespace oem {

// HR 104 (external SOC) and the Modbus source count while HR 100 (heartbeat) was written this recently.
constexpr uint32_t EXT_FRESH_MS = 10000;
constexpr uint16_t EXT_UNKNOWN_I16 = 0x7FFF;  // HR 107: current unknown

class ModbusSource {
 public:
  // ext = HR 100..107 as just written.
  void heartbeat(const uint16_t *ext, uint32_t now) {
    const uint16_t flags = ext[3];
    l_ = {float(ext[1]) / 100.0f,
          float(ext[2]) / 10.0f,
          (flags & 1u) != 0,
          (flags & 2u) != 0,
          (flags & 4u) != 0,
          ext[4] <= 100 ? int(ext[4]) : -1,
          ext[5] != 0 ? float(ext[5]) / 100.0f : NAN,
          ext[7] != EXT_UNKNOWN_I16 ? float(int16_t(ext[7])) / 10.0f : NAN,
          ext[6] != 0 ? float(ext[6]) / 1000.0f : NAN,
          true,
          false,
          SourceMode::MODBUS};
    seen_ = true;
    at_ms_ = now;
    stopped_ = !l_.allowed || l_.alarm || l_.blocking || l_.ccl <= 0.0f;
  }

  void forget_stop() { stopped_ = false; }

  bool fresh(uint32_t now) const { return seen_ && now - at_ms_ <= EXT_FRESH_MS; }

  Limits limits(const Settings &s, uint32_t now) const {
    if (fresh(now)) return l_;
    return stale_limits(s, stopped_, SourceMode::MODBUS);
  }

 private:
  Limits l_{};
  bool seen_ = false;
  uint32_t at_ms_ = 0;
  bool stopped_ = false;  // the last snapshot had charge allowed off, alarm, blocking or CCL 0
};

}  // namespace oem
