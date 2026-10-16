#pragma once

#include <vector>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/canbus/canbus.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "nasn_protocol.h"

namespace esphome::nasn_rectifier {

enum ValueKind : uint8_t { KIND_TELEMETRY = 0, KIND_VOLTS = 1, KIND_AMPS = 2, KIND_RAW = 3 };

// The fallback is what the module runs on by itself when CAN goes quiet, possibly straight into the
// battery, so keep it conservative: at most 54.4 V and 10 A per module.
static constexpr float FALLBACK_MAX_V = 54.4f;
static constexpr float FALLBACK_MAX_A = 10.0f;

// One NASN RM4850-B module: polls its registers, and writes the setpoints on change and every
// keepalive interval (the module reverts to its stored defaults when CAN goes quiet).
class NasnRectifier : public PollingComponent {
 public:
  explicit NasnRectifier(canbus::Canbus *canbus) : canbus_(canbus) {}

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void on_shutdown() override { this->stop_now(); }
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_address(uint8_t address) { this->address_ = address; }
  void set_keepalive_interval(uint32_t ms) { this->keepalive_ms_ = ms; }
  void set_write_stored_defaults(bool write) { this->write_stored_defaults_ = write; }
  void set_alarm_mask(uint32_t mask) { this->alarm_mask_ = mask; }
  void add_sensor(uint16_t reg, uint8_t kind, sensor::Sensor *sensor);
  void set_online_binary_sensor(binary_sensor::BinarySensor *sensor) { this->online_sensor_ = sensor; }
  void set_output_power_sensor(sensor::Sensor *sensor) { this->output_power_sensor_ = sensor; }

  // Control interface used by charge_controller.
  void set_target(float volts, float amps);
  void set_standby(bool standby);
  void set_fallback(float volts, float amps);
  // Immediately sends current 0 and standby (OTA start, shutdown), without waiting for loop().
  void stop_now();
  bool is_online() const { return this->online_; }
  bool faulted() const {
    return this->online_ && ((this->alarms_ & this->alarm_mask_) != 0 || this->consecutive_write_errors_ >= 3 ||
                             this->setpoint_mismatch_);
  }
  // The module's setpoint readback (0x022F / 0x022D / 0x0132) has disagreed with what we commanded
  // continuously for nasn::MISMATCH_MS while online.
  bool setpoint_mismatch() const { return this->setpoint_mismatch_; }
  uint64_t standby_readback_raw() const { return this->standby_readback_raw_; }
  bool boot_sent() const { return this->boot_sent_; }
  // Higher of the finite 0x0175 and 0x0184 readings (the stock firmware measures 0x0184); NaN if neither.
  float output_voltage() const { return nasn::output_voltage(this->vout_, this->vout_alt_); }
  float output_current() const { return this->iout_; }
  // AC input voltage (0x0178); NaN when unknown.
  float input_voltage() const { return this->vin_; }
  // Heatsink temperature (0x0235) in °C; NaN when unknown.
  float heatsink_temperature() const { return this->heatsink_c_; }
  // 0x0183 reply data[4..7] as a big-endian u32 (nasn::alarm_word).
  uint32_t alarms() const { return this->alarms_; }
  uint32_t consecutive_write_errors() const { return this->consecutive_write_errors_; }
  uint8_t address() const { return this->address_; }

 protected:
  struct Entry {
    uint16_t reg;
    uint8_t kind;
    sensor::Sensor *sensor;
  };

  void on_frame_(uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data);
  bool send_(uint32_t can_id, const nasn::Frame &frame);
  bool send_setpoints_();
  bool send_voltage_();
  bool send_current_();
  bool send_standby_();
  void write_stored_defaults_now_(uint32_t now);
  static int live_index_(uint16_t reg);
  void set_online_(bool online);
  void add_poll_reg_(uint16_t reg);

  canbus::Canbus *canbus_;
  uint8_t address_{1};
  uint32_t keepalive_ms_{5000};
  bool write_stored_defaults_{false};
  uint32_t alarm_mask_{nasn::STOCK_ALARM_MASK};
  float fallback_v_{54.0f};
  float fallback_a_{5.0f};
  bool setpoints_failed_{false};

  // Stored defaults (0x0101 / 0x0104): written only when their readback differs from the fallback.
  bool default_v_wrong_{false};
  bool default_a_wrong_{false};
  bool defaults_written_{false};
  uint32_t last_defaults_write_{0};

  // Live setpoint readback, indexed by live_index_(): 0x022F (voltage written to 0x0100), 0x022D, 0x0132.
  // sent_ = last raw value queued for the module (-1 = none yet); polled_ = sent_ when the read
  // request went out, which is what the reply must match.
  int32_t sent_[3]{-1, -1, -1};
  int32_t polled_[3]{-1, -1, -1};
  bool readback_wrong_[3]{false, false, false};
  nasn::PersistTimer mismatch_timer_{nasn::MISMATCH_MS};
  bool setpoint_mismatch_{false};
  uint64_t standby_readback_raw_{0};

  std::vector<Entry> entries_;
  std::vector<uint16_t> poll_regs_;
  size_t poll_pos_{0};
  uint32_t last_poll_send_{0};

  float target_v_{54.0f};
  float target_a_{0.0f};
  bool standby_{true};
  bool dirty_{true};
  bool boot_sent_{false};
  uint32_t last_setpoint_send_{0};

  bool online_{false};
  uint32_t last_reply_{0};
  float vout_{NAN};
  float vout_alt_{NAN};
  float iout_{NAN};
  float vin_{NAN};
  float heatsink_c_{NAN};
  uint32_t alarms_{0};
  uint32_t consecutive_write_errors_{0};
  binary_sensor::BinarySensor *online_sensor_{nullptr};
  sensor::Sensor *output_power_sensor_{nullptr};
};

}  // namespace esphome::nasn_rectifier
