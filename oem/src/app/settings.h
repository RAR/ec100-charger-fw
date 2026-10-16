// oem/src/app/settings.h: the persisted settings, one uint16 per Modbus holding register 0-16 (spec §4), with their
// ranges, defaults and the cross-field clamps of spec §2.3. Voltages are V x 100, cell voltages V x 1000.
#pragma once
#include <cstdint>

namespace oem {

enum Hr : uint8_t {
  HR_MAX_CURRENT = 0,     // A
  HR_CHARGE_V = 1,        // V x 100
  HR_START_SOC = 2,       // %
  HR_STOP_SOC = 3,        // %
  HR_START_V = 4,         // V x 100
  HR_STOP_V = 5,          // V x 100
  HR_SOURCE_MODE = 6,     // 0 Panel, 1 Pylontech, 2 Modbus, 3 EG4, 4 JK, 5 JK-PB
  HR_MODBUS_ADDR = 7,
  HR_MODBUS_BAUD = 8,     // 0 = 9600, 1 = 19200, 2 = 38400, 3 = 115200
  HR_FALLBACK_A = 9,      // total A while a non-panel source is stale
  HR_RELAY_MODE = 10,     // 0 Auto, 1 Off, 2 Modbus
  HR_BACKLIGHT_S = 11,    // 0 = always on
  HR_MAX_CELL_MV = 12,
  HR_CELL_RESUME_MV = 13,
  HR_ADC_TRIM = 14,       // per mille, signed (stored as int16 in the uint16)
  HR_MODULE_FALLBACK_V = 15,  // stored default 0x0101, V x 100
  HR_MODULE_FALLBACK_A = 16,  // stored default 0x0104, A per module
  HR_COUNT = 17,
};

struct HrRange {
  int32_t min, max, def;
};

constexpr HrRange HR_RANGES[HR_COUNT] = {
    {2, 100, 10},  {4700, 5780, 5760}, {0, 99, 20},    {1, 100, 100},  {4200, 5280, 4200}, {4700, 5780, 5760},
    {0, 5, 0},     {1, 247, 1},        {0, 3, 0},      {0, 20, 10},    {0, 2, 0},          {0, 3600, 120},
    {3400, 3580, 3550}, {3200, 3550, 3400}, {-50, 50, 0}, {4800, 5440, 5400}, {0, 10, 5},
};

constexpr int32_t START_STOP_GAP_CV = 500;  // Start V <= Stop V - 5.0 V

struct Settings {
  uint16_t hr[HR_COUNT];

  int32_t get(Hr r) const { return r == HR_ADC_TRIM ? int32_t(int16_t(hr[r])) : int32_t(hr[r]); }
  void set(Hr r, int32_t v) { hr[r] = uint16_t(v); }
  bool operator==(const Settings &o) const {
    for (int i = 0; i < HR_COUNT; i++) {
      if (hr[i] != o.hr[i]) return false;
    }
    return true;
  }
  bool operator!=(const Settings &o) const { return !(*this == o); }
};

inline Settings default_settings() {
  Settings s{};
  for (int i = 0; i < HR_COUNT; i++) s.set(Hr(i), HR_RANGES[i].def);
  return s;
}

inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Every value into its range, then the cross-field rules: Stop V <= Charge V, Start V <= Stop V - 5.0 V,
// Start SOC < Stop SOC, cell resume < max cell. Run after every change (menu, store load, phase-3 Modbus).
inline void apply_clamps(Settings &s) {
  for (int i = 0; i < HR_COUNT; i++) s.set(Hr(i), clamp_i32(s.get(Hr(i)), HR_RANGES[i].min, HR_RANGES[i].max));
  if (s.get(HR_STOP_V) > s.get(HR_CHARGE_V)) s.set(HR_STOP_V, s.get(HR_CHARGE_V));
  if (s.get(HR_START_V) > s.get(HR_STOP_V) - START_STOP_GAP_CV) s.set(HR_START_V, s.get(HR_STOP_V) - START_STOP_GAP_CV);
  if (s.get(HR_START_SOC) >= s.get(HR_STOP_SOC)) s.set(HR_START_SOC, s.get(HR_STOP_SOC) - 1);
  if (s.get(HR_CELL_RESUME_MV) >= s.get(HR_MAX_CELL_MV)) s.set(HR_CELL_RESUME_MV, s.get(HR_MAX_CELL_MV) - 1);
}

// True when s has a value outside its range or apply_clamps() would change it. A Modbus write that does is refused
// rather than clamped, so the master never reads back something it didn't write.
inline bool violates_clamps(const Settings &s) {
  Settings c = s;
  apply_clamps(c);
  return c != s;
}

}  // namespace oem
