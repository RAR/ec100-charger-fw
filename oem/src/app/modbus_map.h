// oem/src/app/modbus_map.h: the spec §4 register map over the live status and settings. Input registers are read on
// demand from Telemetry (no snapshot); writes are validated whole, then applied, and anything main.cpp must act on
// after the reply has gone out is collected in Commands.
#pragma once
#include <cmath>
#include <cstdint>

#include "app/controller.h"
#include "app/modbus_rtu.h"
#include "app/modbus_source.h"
#include "app/rectifier.h"
#include "app/settings.h"

namespace oem {

constexpr uint16_t OEM_FW_VERSION = 0x030A;
constexpr uint16_t HR_HEARTBEAT = 100, HR_EXT_FIRST = 101, HR_EXT_LAST = 107, HR_PAUSE = 110, HR_CLEAR_FAULT = 111,
                   HR_RELAY_CMD = 112, HR_CHARGE_NOW = 113, HR_REBOOT = 120;
constexpr uint16_t REBOOT_MAGIC = 0xB007;
// HR 104 (external SOC) is shown on the LCD and in IR 13 while HR 100 (heartbeat) was written this recently, as long
// as the active source has no SOC of its own. Display only: the charge logic never sees it.
constexpr uint16_t HR_EXT_SOC = 104;
// Diagnostic override (Controller::start_diag): 130 seconds left (write 1-600 to start, 0 to stop), 131 V x100
// (42.00 V to Charge V), 132 A x10 per module (0 to Max current), 133 module mask (bit 0 = PSU1, bit 1 = PSU2).
constexpr uint16_t HR_DIAG_SECONDS = 130, HR_DIAG_V = 131, HR_DIAG_A = 132, HR_DIAG_MASK = 133;
constexpr uint16_t DIAG_MIN_V100 = 4200;
constexpr uint16_t IR_PSU_BASE[2] = {20, 30};
constexpr uint16_t IR_COUNT = 40;

struct Commands {         // what main.cpp applies after the reply has gone out (M8, M9)
  bool settings_changed = false;  // live settings were changed: mark dirty for the 5 s save
  bool pause_set = false;
  bool pause = false;
  bool clear_fault = false, charge_now = false, reboot = false;
  bool line_changed = false;      // HR 7 or 8 changed
  bool diag_start = false, diag_stop = false;
  uint16_t diag_seconds = 0, diag_v100 = 0, diag_a10 = 0, diag_mask = 0;
};

struct Telemetry {        // live references main.cpp fills once; read on demand (M10)
  const Status *st;
  const Rectifier *psu[2];
  const Controller *ctl;
  bool relay_closed;
  float adc_v;
  uint32_t uptime_s;
  uint32_t now_ms;
  bool menu_open;
};

// Scaled to the nearest integer, then saturated; NaN is "not available" (M10).
inline uint16_t mb_u16(float v, float scale) {
  if (std::isnan(v)) return 0xFFFF;
  const float x = v * scale;
  if (!(x > 0.0f)) return 0;
  if (x >= 65535.0f) return 0xFFFF;
  return uint16_t(x + 0.5f);
}

inline uint16_t mb_i16(float v, float scale) {
  if (std::isnan(v)) return 0x7FFF;
  const float x = v * scale;
  if (x <= -32768.0f) return 0x8000;
  if (x >= 32767.0f) return 0x7FFF;
  return uint16_t(int16_t(x < 0.0f ? x - 0.5f : x + 0.5f));
}

class OemRegisters : public RegisterBank {
 public:
  OemRegisters(Settings &live, const Telemetry &t) : live_(live), t_(t) {}

  bool read_input(uint16_t reg, uint16_t &v) override {
    if (reg >= IR_COUNT) return false;
    if (reg >= IR_PSU_BASE[0]) {
      const int k = reg >= IR_PSU_BASE[1] ? 1 : 0;
      v = psu_register_(k, reg - IR_PSU_BASE[k]);
      return true;
    }
    const Status &st = *t_.st;
    switch (reg) {
      case 0: v = OEM_FW_VERSION; break;
      case 1: v = uint16_t(st.state); break;
      case 2: v = uint16_t(st.hold); break;
      case 3: v = st.error; break;
      case 4: {
        const bool bits[9] = {st.cooling, st.no_current, st.stale, st.session_active, t_.relay_closed, st.online > 0,
                              t_.psu[0]->is_online(), t_.psu[1]->is_online(), st.diag};
        v = 0;
        for (int b = 0; b < 9; b++) v = uint16_t(v | (bits[b] ? 1u << b : 0u));
        break;
      }
      case 5: v = st.source; break;  // active source: 0 Panel, 1 Pylontech, 2 Modbus, 3 EG4, 4 JK, 5 JK-PB
      case 6: v = mb_u16(st.target_v, 100.0f); break;
      case 7: v = mb_u16(st.commanded_a, 10.0f); break;
      case 8: v = mb_u16(st.budget, 10.0f); break;
      case 9: v = mb_u16(st.output_a, 10.0f); break;
      case 10: v = mb_u16(st.power_w, 1.0f); break;
      case 11: v = mb_u16(st.battery_v, 100.0f); break;
      case 12: v = mb_u16(t_.adc_v, 100.0f); break;
      case 13: {
        const int soc = st.soc >= 0 ? st.soc : external_soc(t_.now_ms);
        v = soc < 0 ? 0xFFFF : uint16_t(soc);
        break;
      }
      case 14: v = mb_u16(st.cvl, 100.0f); break;
      case 15: v = mb_u16(st.ccl, 10.0f); break;
      case 16: v = uint16_t(t_.uptime_s >> 16); break;
      case 17: v = uint16_t(t_.uptime_s & 0xFFFF); break;
      case 18: v = 0; break;  // reserved (was "source age", never implemented); IR 4 bit 2 is the stale flag
      default: v = 0xFFFF; break;  // 19: reserved
    }
    return true;
  }

  bool read_holding(uint16_t reg, uint16_t &v) override {
    if (reg < HR_COUNT) {
      v = live_.hr[reg];
    } else if (reg >= HR_HEARTBEAT && reg <= HR_EXT_LAST) {
      v = ext_[reg - HR_HEARTBEAT];
    } else if (reg == HR_PAUSE) {
      v = paused_;
    } else if (reg == HR_RELAY_CMD) {
      v = relay_cmd_;
    } else if (reg == HR_CLEAR_FAULT || reg == HR_CHARGE_NOW || reg == HR_REBOOT) {
      v = 0;  // commands, not state
    } else if (reg == HR_DIAG_SECONDS) {
      v = uint16_t(t_.ctl->diag_remaining_s(t_.now_ms));
    } else if (reg >= HR_DIAG_V && reg <= HR_DIAG_MASK) {
      v = diag_[reg - HR_DIAG_V];
    } else {
      return false;
    }
    return true;
  }

  uint8_t write_holding(uint16_t start, const uint16_t *v, uint16_t n) override {
    if (n == 0) return MB_EX_ILLEGAL_VALUE;
    const uint32_t last = uint32_t(start) + n - 1;
    if (last < HR_COUNT) return write_settings_(start, v, n);
    if (start >= HR_HEARTBEAT && last <= HR_EXT_LAST) {
      for (uint16_t i = 0; i < n; i++) ext_[start - HR_HEARTBEAT + i] = v[i];
      if (start == HR_HEARTBEAT) {
        // Any write starting at HR 100 keeps the 3.5 Panel SOC display fresh. Only a write of all eight registers is
        // a Modbus snapshot, so an older bridge config (shorter writes) can't keep a Modbus-mode charger fresh on
        // stale limits.
        heartbeat_ms_ = t_.now_ms;
        heartbeat_seen_ = true;
        if (n == HR_EXT_LAST - HR_HEARTBEAT + 1) modbus_.heartbeat(ext_, t_.now_ms);
      }
      return 0;
    }
    const bool runtime = (start >= HR_PAUSE && last <= HR_CHARGE_NOW) || (start == HR_REBOOT && n == 1);
    if (runtime) return write_runtime_(start, v, n);
    if (start >= HR_DIAG_SECONDS && last <= HR_DIAG_MASK) return write_diag_(start, v, n);
    return MB_EX_ILLEGAL_ADDRESS;
  }

  Commands take_commands() {
    const Commands c = cmd_;
    cmd_ = Commands{};
    return c;
  }
  bool relay_command() const { return relay_cmd_; }
  Limits modbus_limits(const Settings &s, uint32_t now) const { return modbus_.limits(s, now); }
  void forget_modbus_stop() { modbus_.forget_stop(); }

  // HR 104 while the HR 100 heartbeat is fresh and the value is 0-100 %, else -1.
  int external_soc(uint32_t now) const {
    const uint16_t soc = ext_[HR_EXT_SOC - HR_HEARTBEAT];
    if (!heartbeat_seen_ || now - heartbeat_ms_ > EXT_FRESH_MS || soc > 100) return -1;
    return soc;
  }
  bool paused() const { return paused_; }
  void sync_pause(bool p) { paused_ = p; }

 private:
  uint16_t psu_register_(int k, int off) const {
    const Rectifier &r = *t_.psu[k];
    switch (off) {
      case 0:
        return uint16_t((r.is_online() ? 1u : 0u) | (r.faulted() ? 2u : 0u) | (t_.ctl->module_cooling(k) ? 4u : 0u) |
                        (r.standby() ? 8u : 0u));
      case 1: return mb_u16(r.output_voltage(), 100.0f);
      case 2: return mb_u16(r.output_current(), 10.0f);
      case 3: return mb_u16(r.input_voltage(), 10.0f);
      case 4: return 0x7FFF;  // inlet temperature: not measured
      case 5: return mb_i16(r.heatsink_temperature(), 10.0f);
      case 8: return uint16_t(r.alarms() >> 16);
      case 9: return uint16_t(r.alarms() & 0xFFFF);
      default: return 0xFFFF;  // fans: not measured
    }
  }

  // M6, M7: into a copy, every value checked and the clamps respected, or nothing is stored.
  uint8_t write_settings_(uint16_t start, const uint16_t *v, uint16_t n) {
    if (t_.menu_open) return MB_EX_BUSY;
    Settings c = live_;
    for (uint16_t i = 0; i < n; i++) c.hr[start + i] = v[i];
    // M1: a polled source (Pylontech, EG4, JK, JK-PB) can't be chosen over Modbus (it would cut this link). A Source
    // mode the menu stored doesn't block other writes, including a block that writes it back unchanged; only
    // changing it to a polled mode is refused.
    const bool changes_source = c.hr[HR_SOURCE_MODE] != live_.hr[HR_SOURCE_MODE];
    if ((changes_source && is_polled(SourceMode(c.get(HR_SOURCE_MODE)))) || violates_clamps(c)) {
      return MB_EX_ILLEGAL_VALUE;
    }
    if (c == live_) return 0;
    const bool line = c.hr[HR_MODBUS_ADDR] != live_.hr[HR_MODBUS_ADDR] ||
                      c.hr[HR_MODBUS_BAUD] != live_.hr[HR_MODBUS_BAUD];
    live_ = c;
    cmd_.settings_changed = true;
    cmd_.line_changed = cmd_.line_changed || line;
    return 0;
  }

  // M9: every value checked first, so a block is all or nothing.
  uint8_t write_runtime_(uint16_t start, const uint16_t *v, uint16_t n) {
    for (uint16_t i = 0; i < n; i++) {
      const uint16_t reg = uint16_t(start + i);
      const bool ok = reg == HR_REBOOT                                   ? v[i] == REBOOT_MAGIC
                      : reg == HR_CLEAR_FAULT || reg == HR_CHARGE_NOW ? v[i] == 1
                                                                       : v[i] <= 1;
      if (!ok) return MB_EX_ILLEGAL_VALUE;
    }
    for (uint16_t i = 0; i < n; i++) {
      switch (start + i) {
        case HR_PAUSE:
          paused_ = v[i] != 0;
          cmd_.pause_set = true;
          cmd_.pause = paused_;
          break;
        case HR_CLEAR_FAULT: cmd_.clear_fault = true; break;
        case HR_RELAY_CMD: relay_cmd_ = v[i] != 0; break;
        case HR_CHARGE_NOW: cmd_.charge_now = true; break;
        case HR_REBOOT: cmd_.reboot = true; break;
        default: break;
      }
    }
    return 0;
  }

  // HR 130-133: every value checked against the live Charge V and Max current before anything is stored.
  uint8_t write_diag_(uint16_t start, const uint16_t *v, uint16_t n) {
    uint16_t d[3] = {diag_[0], diag_[1], diag_[2]};
    bool has_seconds = false;
    uint16_t seconds = 0;
    for (uint16_t i = 0; i < n; i++) {
      const uint16_t reg = uint16_t(start + i);
      if (reg == HR_DIAG_SECONDS) {
        has_seconds = true;
        seconds = v[i];
      } else {
        d[reg - HR_DIAG_V] = v[i];
      }
    }
    if (has_seconds && seconds > DIAG_MAX_S) return MB_EX_ILLEGAL_VALUE;
    const bool starting = has_seconds && seconds > 0;
    const bool v_ok = d[0] >= DIAG_MIN_V100 && d[0] <= live_.hr[HR_CHARGE_V];
    // HR 132 is per module: with both modules (mask 3) the total is twice it, and that total is what Max current caps.
    const int32_t modules = d[2] == 3 ? 2 : 1;
    const bool a_ok = int32_t(d[1]) * modules <= live_.get(HR_MAX_CURRENT) * 10;
    const bool mask_ok = d[2] <= 3;
    // Values written on their own must already be valid; a start also needs a voltage and at least one module.
    if ((start != HR_DIAG_SECONDS || n > 1) && !(v_ok && a_ok && mask_ok)) return MB_EX_ILLEGAL_VALUE;
    if (starting && !(v_ok && a_ok && mask_ok && d[2] != 0)) return MB_EX_ILLEGAL_VALUE;
    for (int i = 0; i < 3; i++) diag_[i] = d[i];
    if (has_seconds) {
      cmd_.diag_start = starting;
      cmd_.diag_stop = !starting;
      cmd_.diag_seconds = seconds;
      cmd_.diag_v100 = d[0];
      cmd_.diag_a10 = d[1];
      cmd_.diag_mask = d[2];
    }
    return 0;
  }

  Settings &live_;
  const Telemetry &t_;
  uint16_t diag_[3] = {};  // 131-133
  // 100-107: the Modbus source's registers; HR 104 is also the Panel SOC display (3.5)
  uint16_t ext_[HR_EXT_LAST - HR_HEARTBEAT + 1] = {};
  ModbusSource modbus_;
  uint32_t heartbeat_ms_ = 0;
  bool heartbeat_seen_ = false;
  Commands cmd_;
  bool paused_ = false, relay_cmd_ = false;
};

}  // namespace oem
