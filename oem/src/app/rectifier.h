// oem/src/app/rectifier.h: one NASN RM4850-B module. A port of the ESP component (esphome/components/nasn_rectifier)
// without ESPHome: same registers, setpoint keepalive, readback checks, stored defaults and alarm mask, from
// nasn_protocol.h. Differences: one read in flight at a time (spec §1), a poll cycle every second, and offline after
// 5 s without a good read reply (the ESP used 3 x its update interval).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

#include "app/can_port.h"
#include "nasn_rectifier/nasn_protocol.h"

namespace oem {

constexpr float FALLBACK_MAX_V = 54.4f;  // stored defaults: what a module runs on alone if we go quiet
constexpr float FALLBACK_MAX_A = 10.0f;
constexpr uint32_t POLL_SPACING_MS = 20;
constexpr uint32_t POLL_CYCLE_MS = 1000;
constexpr uint32_t REQUEST_TIMEOUT_MS = 250;  // give up on one read and poll the next register
constexpr uint32_t OFFLINE_MS = 5000;         // no good read reply this long: offline
constexpr uint32_t KEEPALIVE_MS = 5000;
constexpr uint32_t SETPOINT_RETRY_MS = 50;    // a setpoint frame the TX queue refused goes again this soon
constexpr int DEFAULT_WRITE_LIMIT = 3;        // stored-default rewrites per value per boot: each one costs module flash
constexpr int DEFAULT_WRITES_PER_BOOT = 10;   // all stored-default writes per module per boot, whatever the values

class Rectifier {
 public:
  Rectifier(CanPort &can, uint8_t address, bool write_stored_defaults)
      : can_(can), address_(address), write_stored_defaults_(write_stored_defaults) {}

  // Call every main-loop pass.
  void loop(uint32_t now) {
    const bool budget = default_writes_ < DEFAULT_WRITES_PER_BOOT;
    const bool v_due = budget && default_v_wrong_ && default_v_writes_ < DEFAULT_WRITE_LIMIT;
    const bool a_due = budget && default_a_wrong_ && default_a_writes_ < DEFAULT_WRITE_LIMIT;
    if (write_stored_defaults_ && online_ && (v_due || a_due) &&
        (!defaults_written_ || now - last_defaults_write_ >= KEEPALIVE_MS)) {
      write_stored_defaults_now_(v_due, a_due, now);
    }
    // All three setpoints go out on a change and as the keepalive; any the queue refused go again every 50 ms, so a
    // full queue delays a stop by that much rather than until the next keepalive.
    const bool retry_ok = pending_ == 0 || now - last_attempt_ >= SETPOINT_RETRY_MS;
    if ((dirty_ || now - last_setpoint_send_ >= KEEPALIVE_MS) && retry_ok) {
      pending_ = SEND_V | SEND_A | SEND_STANDBY;
      dirty_ = false;
      last_setpoint_send_ = now;
    }
    if (pending_ != 0 && retry_ok) {
      send_setpoints_();
      last_attempt_ = now;
      if (pending_ == 0) boot_sent_ = true;
    }
    if (in_flight_ && now - in_flight_since_ >= REQUEST_TIMEOUT_MS) in_flight_ = false;
    if (poll_pos_ >= POLL_COUNT && now - cycle_start_ >= POLL_CYCLE_MS) poll_pos_ = 0;
    if (!in_flight_ && poll_pos_ < POLL_COUNT && now - last_poll_send_ >= POLL_SPACING_MS) {
      const uint16_t reg = POLL_REGS[poll_pos_];
      const int i = live_index_(reg);
      if (i >= 0) polled_[i] = sent_[i];  // writes are queued ahead of this read
      if (send_(nasn::read_id(address_), nasn::read_request(reg))) {
        if (poll_pos_ == 0) cycle_start_ = now;
        poll_pos_++;
        in_flight_ = true;
        in_flight_reg_ = reg;
        in_flight_since_ = now;
      }
      last_poll_send_ = now;
    }
    if (online_ && now - last_reply_ > OFFLINE_MS) set_online_(false);

    const bool wrong = readback_wrong_[0] || readback_wrong_[1] || readback_wrong_[2];
    setpoint_mismatch_ = online_ && mismatch_timer_.update(wrong, now);
  }

  // Every received frame; anything not a reply from this module is ignored.
  void on_frame(const CanFrame &f, uint32_t now) {
    auto reply = nasn::parse_reply(f.id, true, f.data, f.len);
    if (!reply || reply->addr != address_) return;
    if (reply->cmd == nasn::CMD_WRITE) {
      consecutive_write_errors_ = reply->status != 0 ? consecutive_write_errors_ + 1 : 0;
      return;
    }
    if (in_flight_ && reply->reg == in_flight_reg_) in_flight_ = false;
    if (reply->status != 0) return;  // failed reads are not liveness
    last_reply_ = now;
    if (!online_) set_online_(true);
    switch (reply->reg) {
      case nasn::reg::VOUT:
        vout_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::VOUT_ALT:
        vout_alt_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::IOUT:
        iout_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::INPUT_VOLTAGE:
        vin_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::TEMP_HEATSINK:
        heatsink_c_ = nasn::telemetry(reply->value);
        break;
      case nasn::reg::ALARMS:
        alarms_ = nasn::alarm_word(reply->value);
        break;
      case nasn::reg::VOUT_DEFAULT:
        if (write_stored_defaults_)
          default_v_wrong_ = nasn::stored_default_mismatch(reply->reg, reply->value, nasn::voltage_raw(fallback_v_));
        break;
      case nasn::reg::ILIMIT_DEFAULT:
        if (write_stored_defaults_)
          default_a_wrong_ = nasn::stored_default_mismatch(reply->reg, reply->value, nasn::current_raw(fallback_a_));
        break;
      default:
        break;
    }
    const int i = live_index_(reply->reg);
    if (i >= 0 && polled_[i] >= 0)
      readback_wrong_[i] = nasn::readback_mismatch(reply->reg, reply->value, uint64_t(polled_[i]));
  }

  void set_target(float volts, float amps) {
    if (!std::isfinite(volts) || !std::isfinite(amps)) return;
    target_set_ = true;
    if (std::fabs(volts - target_v_) >= 0.01f || std::fabs(amps - target_a_) >= 0.1f) {
      target_v_ = volts;
      target_a_ = amps;
      dirty_ = true;
    }
  }

  void set_standby(bool standby) {
    if (standby != standby_) {
      standby_ = standby;
      dirty_ = true;
    }
  }

  // Stored defaults (0x0101 / 0x0104), clamped to <= 54.4 V and <= 10 A; rewritten when their readback differs, at
  // most DEFAULT_WRITE_LIMIT times per value and DEFAULT_WRITES_PER_BOOT in all, so a master flipping HR 15/16 can't
  // wear the module's flash. Until the first set_target() it is also the live voltage, as on the ESP.
  void set_fallback(float volts, float amps) {
    if (!std::isfinite(volts) || !std::isfinite(amps)) return;
    const float v = std::clamp(volts, nasn::MIN_V, FALLBACK_MAX_V), a = std::clamp(amps, 0.0f, FALLBACK_MAX_A);
    if (v != fallback_v_) default_v_writes_ = 0;
    if (a != fallback_a_) default_a_writes_ = 0;
    fallback_v_ = v;
    fallback_a_ = a;
    if (!target_set_ && std::fabs(v - target_v_) >= 0.01f) {
      target_v_ = v;
      dirty_ = true;
    }
  }

  bool is_online() const { return online_; }
  bool faulted() const {
    return online_ && ((alarms_ & nasn::STOCK_ALARM_MASK) != 0 || consecutive_write_errors_ >= 3 || setpoint_mismatch_);
  }
  bool setpoint_mismatch() const { return setpoint_mismatch_; }
  bool boot_sent() const { return boot_sent_; }
  float output_voltage() const { return nasn::output_voltage(vout_, vout_alt_); }
  float output_current() const { return iout_; }
  float input_voltage() const { return vin_; }
  float heatsink_temperature() const { return heatsink_c_; }
  uint32_t alarms() const { return alarms_; }
  uint32_t consecutive_write_errors() const { return consecutive_write_errors_; }
  uint8_t address() const { return address_; }
  bool standby() const { return standby_; }
  float target_current() const { return target_a_; }

 private:
  static constexpr uint16_t POLL_REGS[] = {
      nasn::reg::VOUT,         nasn::reg::VOUT_ALT,  nasn::reg::IOUT,          nasn::reg::INPUT_VOLTAGE,
      nasn::reg::TEMP_HEATSINK, nasn::reg::ALARMS,   nasn::reg::VOUT_SET,      nasn::reg::VOUT_READBACK,
      nasn::reg::ILIMIT,       nasn::reg::STANDBY,   nasn::reg::VOUT_DEFAULT,  nasn::reg::ILIMIT_DEFAULT,
  };
  static constexpr int POLL_COUNT = sizeof(POLL_REGS) / sizeof(POLL_REGS[0]);

  // Index into sent_/polled_/readback_wrong_: 0x022F (the voltage written to 0x0100), 0x022D, 0x0132.
  static int live_index_(uint16_t reg) {
    switch (reg) {
      case nasn::reg::VOUT_READBACK:
        return 0;
      case nasn::reg::ILIMIT:
        return 1;
      case nasn::reg::STANDBY:
        return 2;
      default:
        return -1;
    }
  }

  bool send_(uint32_t id, const nasn::Frame &frame) { return can_.send(id, frame.data()); }

  static constexpr uint8_t SEND_V = 1, SEND_A = 2, SEND_STANDBY = 4;

  // Sends the pending setpoints, clearing each one the queue takes. Standby goes first when stopping and last when
  // starting, so a partly accepted send never leaves a module running on half of the new setpoints.
  void send_setpoints_() {
    const uint32_t id = nasn::write_id(address_);
    if (standby_) send_standby_(id);
    if (pending_ & SEND_V) {
      const uint16_t v = nasn::voltage_raw(target_v_);
      if (send_(id, nasn::write_u16(nasn::reg::VOUT_SET, v))) {
        sent_[0] = v;
        pending_ &= ~SEND_V;
      }
    }
    if (pending_ & SEND_A) {
      const uint16_t a = nasn::current_raw(target_a_);
      if (send_(id, nasn::write_u16(nasn::reg::ILIMIT, a))) {
        sent_[1] = a;
        pending_ &= ~SEND_A;
      }
    }
    if (!standby_) send_standby_(id);
  }

  void send_standby_(uint32_t id) {
    if ((pending_ & SEND_STANDBY) && send_(id, nasn::write_standby(standby_))) {
      sent_[2] = standby_ ? 1 : 0;
      pending_ &= ~SEND_STANDBY;
    }
  }

  void write_stored_defaults_now_(bool v, bool a, uint32_t now) {
    const uint32_t id = nasn::write_id(address_);
    if (v && send_(id, nasn::write_voltage(nasn::reg::VOUT_DEFAULT, fallback_v_))) {
      default_v_writes_++;
      default_writes_++;
    }
    if (a && default_writes_ < DEFAULT_WRITES_PER_BOOT &&
        send_(id, nasn::write_current(nasn::reg::ILIMIT_DEFAULT, fallback_a_))) {
      default_a_writes_++;
      default_writes_++;
    }
    default_v_wrong_ = false;  // the next readback decides whether another write is needed
    default_a_wrong_ = false;
    defaults_written_ = true;
    last_defaults_write_ = now;
  }

  void set_online_(bool online) {
    online_ = online;
    dirty_ = true;  // a module that just appeared needs our setpoints now
    if (online) return;
    // Nothing that reaches a module we can't hear may run it: what goes out while it's away (and may sit in a
    // mailbox until it's back) says standby, 0 A. The controller says the same from its next tick.
    standby_ = true;
    target_a_ = 0.0f;
    vout_ = vout_alt_ = iout_ = vin_ = heatsink_c_ = NAN;
    alarms_ = 0;
    consecutive_write_errors_ = 0;
    for (int i = 0; i < 3; i++) {
      readback_wrong_[i] = false;
      polled_[i] = -1;
    }
    mismatch_timer_.reset();
    setpoint_mismatch_ = false;
    default_v_wrong_ = default_a_wrong_ = false;
  }

  CanPort &can_;
  uint8_t address_;
  bool write_stored_defaults_;
  float fallback_v_ = 54.0f;
  float fallback_a_ = 5.0f;

  bool default_v_wrong_ = false, default_a_wrong_ = false, defaults_written_ = false;
  int default_v_writes_ = 0, default_a_writes_ = 0, default_writes_ = 0;  // per value (reset on change), per boot
  uint32_t last_defaults_write_ = 0;

  int32_t sent_[3] = {-1, -1, -1};
  int32_t polled_[3] = {-1, -1, -1};
  bool readback_wrong_[3] = {false, false, false};
  nasn::PersistTimer mismatch_timer_{nasn::MISMATCH_MS};
  bool setpoint_mismatch_ = false;

  int poll_pos_ = 0;
  uint32_t cycle_start_ = 0, last_poll_send_ = 0;
  bool in_flight_ = false;
  uint16_t in_flight_reg_ = 0;
  uint32_t in_flight_since_ = 0;

  float target_v_ = 54.0f, target_a_ = 0.0f;
  bool target_set_ = false, standby_ = true, dirty_ = true, boot_sent_ = false;
  uint8_t pending_ = 0;  // SEND_* not yet accepted by the queue
  uint32_t last_setpoint_send_ = 0, last_attempt_ = 0;

  bool online_ = false;
  uint32_t last_reply_ = 0;
  float vout_ = NAN, vout_alt_ = NAN, iout_ = NAN, vin_ = NAN, heatsink_c_ = NAN;
  uint32_t alarms_ = 0, consecutive_write_errors_ = 0;
};

}  // namespace oem
