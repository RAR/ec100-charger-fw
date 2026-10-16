// oem/src/app/limit_source.h: the common Limits struct every source produces (spec §2.1). This file holds the Panel
// source; the Pylontech source is in pylon_source.h; Modbus is in modbus_source.h.
#pragma once
#include <algorithm>
#include <cmath>

#include "app/settings.h"

namespace oem {

// HR 6 / IR 5. PANEL and MODBUS keep RS485 as the Modbus server; PYLONTECH, EG4, JK and JKPB make the board the BMS
// poller.
enum class SourceMode : uint8_t { PANEL = 0, PYLONTECH = 1, MODBUS = 2,
                              EG4 = 3,     // EG4 BMS (eg4_source.h)
                              JK = 4,      // JK-B BMS (jk_source.h)
                              JKPB = 5 };  // JK-PB BMS (jkpb_source.h)

// True for the modes where the board polls a BMS over RS485 (and so isn't a Modbus server).
inline bool is_polled(SourceMode m) {
  return m == SourceMode::PYLONTECH || m == SourceMode::EG4 || m == SourceMode::JK || m == SourceMode::JKPB;
}

struct Limits {
  float cvl;         // V
  float ccl;         // A, total
  bool allowed;      // the source allows charging
  bool alarm;
  bool blocking;
  int soc;           // %, -1 when unknown
  float battery_v;   // NaN when unknown
  float battery_a;   // NaN when unknown; positive = charging
  float max_cell_v;  // NaN when unknown
  bool fresh;
  bool stale;         // a non-panel source is configured but not fresh: these are its Panel fallback limits
  SourceMode source;  // the source these came from
};

// Panel: CVL = Charge V, CCL = Max current, allowed, always fresh, nothing else known.
inline Limits panel_limits(const Settings &s) {
  return {float(s.get(HR_CHARGE_V)) / 100.0f, float(s.get(HR_MAX_CURRENT)), true, false, false, -1,
          NAN, NAN, NAN, true, false, SourceMode::PANEL};
}

// A configured source that isn't fresh: Panel limits with ccl capped at Fallback current, flagged stale; a source
// whose last fresh state was a stop keeps charging off.
inline Limits stale_limits(const Settings &s, bool stopped, SourceMode source) {
  Limits l = panel_limits(s);
  l.ccl = std::min(l.ccl, float(s.get(HR_FALLBACK_A)));
  l.allowed = !stopped;
  l.stale = true;
  l.source = source;
  return l;
}

}  // namespace oem
