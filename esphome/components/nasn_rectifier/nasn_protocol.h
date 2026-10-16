#pragma once
// NASN RM4850-B CAN framing, as used by the stock Eco-Worthy controller firmware (see docs/history/notes/findings.md).
// Pure C++17 with no ESPHome dependencies, so it is unit-tested on the host (tests/test_nasn_protocol.cpp).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace nasn {

constexpr uint32_t READ_BASE = 0x108082FE;   // cmd 0x82 register get
constexpr uint32_t WRITE_BASE = 0x108081FE;  // cmd 0x81 register set (Huawei docs use 0x80)
constexpr uint8_t PROTO_SMU = 0x21;
constexpr uint8_t CMD_WRITE = 0x81;
constexpr uint8_t CMD_READ = 0x82;
constexpr float SETPOINT_SCALE = 1023.0f;   // stock firmware scales setpoints by 1023
constexpr float TELEMETRY_SCALE = 1024.0f;  // Huawei scale for readings (confirm in Task 9)
constexpr float NOMINAL_A = 52.0f;
constexpr float MIN_V = 42.0f;
constexpr float MAX_V = 58.0f;          // hard cap, never exceeded
constexpr float MAX_A_PER_PSU = 50.0f;  // hard cap per module

namespace reg {
constexpr uint16_t VOUT_SET = 0x0100;
constexpr uint16_t VOUT_DEFAULT = 0x0101;
constexpr uint16_t ILIMIT_DEFAULT = 0x0104;
constexpr uint16_t STANDBY = 0x0132;
constexpr uint16_t INPUT_POWER = 0x0170;
constexpr uint16_t INPUT_FREQ = 0x0171;
constexpr uint16_t INPUT_CURRENT = 0x0172;
constexpr uint16_t OUTPUT_POWER = 0x0173;
constexpr uint16_t VOUT = 0x0175;
constexpr uint16_t VOUT_ALT = 0x0184;  // the stock firmware's output-voltage reading (/1024)
constexpr uint16_t INPUT_VOLTAGE = 0x0178;
constexpr uint16_t TEMP_OUT = 0x017F;
constexpr uint16_t TEMP_IN = 0x0180;
constexpr uint16_t TEMP_HEATSINK = 0x0235;  // NASN heatsink/outlet temperature (/1024 °C)
constexpr uint16_t IOUT = 0x0182;
constexpr uint16_t ALARMS = 0x0183;
constexpr uint16_t ILIMIT = 0x022D;
constexpr uint16_t VOUT_READBACK = 0x022F;  // the stock firmware's VOUT setpoint readback (x1023)
}  // namespace reg

using Frame = std::array<uint8_t, 8>;

inline uint32_t read_id(uint8_t addr) { return READ_BASE | (uint32_t(addr & 0x7F) << 16); }
inline uint32_t write_id(uint8_t addr) { return WRITE_BASE | (uint32_t(addr & 0x7F) << 16); }

inline Frame read_request(uint16_t r) { return {uint8_t(r >> 8), uint8_t(r & 0xFF), 0, 0, 0, 0, 0, 0}; }

inline Frame write_u16(uint16_t r, uint16_t raw) {
  return {uint8_t(r >> 8), uint8_t(r & 0xFF), 0, 0, 0, 0, uint8_t(raw >> 8), uint8_t(raw & 0xFF)};
}

inline uint16_t voltage_raw(float volts) {
  if (!std::isfinite(volts)) volts = MIN_V;
  return uint16_t(std::lround(std::clamp(volts, MIN_V, MAX_V) * SETPOINT_SCALE));
}

inline uint16_t current_raw(float amps) {
  if (!std::isfinite(amps)) amps = 0.0f;
  return uint16_t(std::lround(std::clamp(amps, 0.0f, MAX_A_PER_PSU) * SETPOINT_SCALE / NOMINAL_A));
}

inline Frame write_voltage(uint16_t r, float volts) { return write_u16(r, voltage_raw(volts)); }
inline Frame write_current(uint16_t r, float amps) { return write_u16(r, current_raw(amps)); }

inline Frame write_standby(bool standby) {
  return {uint8_t(reg::STANDBY >> 8), uint8_t(reg::STANDBY & 0xFF), 0, uint8_t(standby ? 1 : 0), 0, 0, 0, 0};
}

struct Reply {
  uint8_t addr;
  uint8_t cmd;     // CMD_READ or CMD_WRITE
  uint8_t status;  // 0 = ok
  uint16_t reg;
  uint64_t value;  // 48-bit big-endian payload
};

inline std::optional<Reply> parse_reply(uint32_t id, bool extended, const uint8_t *data, size_t len) {
  if (!extended || len != 8) return std::nullopt;
  if (((id >> 23) & 0x3F) != PROTO_SMU || (id & 0x80)) return std::nullopt;  // bit 7 set = sent by a controller
  uint8_t cmd = (id >> 8) & 0xFF;
  if (cmd != CMD_READ && cmd != CMD_WRITE) return std::nullopt;
  Reply r{};
  r.addr = (id >> 16) & 0x7F;
  r.cmd = cmd;
  r.status = data[0] >> 4;
  r.reg = uint16_t((data[0] & 0x0F) << 8 | data[1]);
  for (size_t i = 2; i < 8; i++) r.value = (r.value << 8) | data[i];
  return r;
}

// ---- Alarms (0x0183) ----

// The stock firmware reads reply data[4..7] of 0x0183 as a big-endian u32 and forces a stop on these
// bits (E1-E11: bits 0-4, 12, 14, 17, 20, 22, 27; see docs/history/notes/findings-v2.md section 5, decoded from
// FUN_0800bbf0). Those bits OR to 0x0852501F; the "0x0855501F" quoted in the notes has bits 16/18 instead of 17.
constexpr uint32_t STOCK_ALARM_MASK = 0x0852501F;

// data[4..7] of a reply: the low 32 bits of the 48-bit value (data[2..3] are dropped).
inline uint32_t alarm_word(uint64_t value) { return uint32_t(value & 0xFFFFFFFFull); }

inline float telemetry(uint64_t raw) { return float(raw) / TELEMETRY_SCALE; }
inline float setpoint_volts(uint64_t raw) { return float(raw) / SETPOINT_SCALE; }
inline float setpoint_amps(uint64_t raw) { return float(raw) * NOMINAL_A / SETPOINT_SCALE; }

// Output voltage from 0x0175 and 0x0184: it isn't known yet which one the NASN means, so take the
// higher of the finite readings (conservative for the overvoltage check). NaN when neither is known.
inline float output_voltage(float vout_0175, float vout_0184) {
  if (!std::isfinite(vout_0175)) return std::isfinite(vout_0184) ? vout_0184 : NAN;
  if (!std::isfinite(vout_0184)) return vout_0175;
  return std::max(vout_0175, vout_0184);
}

// ---- Setpoint readback ----

constexpr float MISMATCH_V = 0.15f;            // live setpoint readback tolerance
constexpr float MISMATCH_A = 1.0f;
constexpr uint32_t MISMATCH_MS = 10000;        // continuous mismatch before the module counts as faulted
constexpr float DEFAULT_TOLERANCE_V = 0.05f;   // stored defaults: rewrite only beyond this
constexpr float DEFAULT_TOLERANCE_A = 0.5f;

// The 0x0132 readback layout isn't confirmed yet (Task 9), so any nonzero value counts as standby.
inline bool standby_readback(uint64_t raw) { return raw != 0; }

// Does a live setpoint register's readback disagree with what we commanded? `commanded` is the raw
// word we wrote (voltage_raw / current_raw), or 0/1 for standby. Other registers never mismatch.
// The voltage is checked on 0x022F, as the stock firmware does; 0x0100 is only published for comparison.
inline bool readback_mismatch(uint16_t r, uint64_t readback, uint64_t commanded) {
  switch (r) {
    case reg::VOUT_READBACK:
      return std::fabs(setpoint_volts(readback) - setpoint_volts(commanded)) > MISMATCH_V;
    case reg::ILIMIT:
      return std::fabs(setpoint_amps(readback) - setpoint_amps(commanded)) > MISMATCH_A;
    case reg::STANDBY:
      return standby_readback(readback) != (commanded != 0);
    default:
      return false;
  }
}

// Does a stored default (0x0101 / 0x0104) differ enough from the wanted raw word to be rewritten?
inline bool stored_default_mismatch(uint16_t r, uint64_t readback, uint64_t wanted) {
  switch (r) {
    case reg::VOUT_DEFAULT:
      return std::fabs(setpoint_volts(readback) - setpoint_volts(wanted)) > DEFAULT_TOLERANCE_V;
    case reg::ILIMIT_DEFAULT:
      return std::fabs(setpoint_amps(readback) - setpoint_amps(wanted)) > DEFAULT_TOLERANCE_A;
    default:
      return false;
  }
}

// True once a condition has held continuously for duration_ms; any false sample restarts it.
class PersistTimer {
 public:
  explicit PersistTimer(uint32_t duration_ms = MISMATCH_MS) : duration_ms_(duration_ms) {}

  bool update(bool condition, uint32_t now_ms) {
    if (!condition) {
      reset();
      return false;
    }
    if (!timing_) {
      timing_ = true;
      since_ms_ = now_ms;
    }
    active_ = now_ms - since_ms_ >= duration_ms_;
    return active_;
  }

  void reset() {
    timing_ = false;
    active_ = false;
  }

  bool active() const { return active_; }

 private:
  uint32_t duration_ms_;
  uint32_t since_ms_ = 0;
  bool timing_ = false;
  bool active_ = false;
};

}  // namespace nasn
