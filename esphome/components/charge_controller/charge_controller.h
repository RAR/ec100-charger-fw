#pragma once

#include <cmath>
#include <string>
#include <vector>

#include "charge_logic.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/nasn_rectifier/nasn_rectifier.h"
#include "esphome/components/number/number.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/component.h"

namespace esphome::charge_controller {

// Turns YamBMS limits (MQTT) and HA caps into per-rectifier targets once a second.
class ChargeController : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void add_rectifier(nasn_rectifier::NasnRectifier *rectifier) {
    this->rectifiers_.push_back(rectifier);
    this->cooldown_.emplace_back();
  }
  void set_max_voltage_number(number::Number *n) { this->max_v_ = n; }
  void set_max_current_number(number::Number *n) { this->max_a_ = n; }
  void set_fallback_voltage_number(number::Number *n) { this->fallback_v_ = n; }
  void set_fallback_current_number(number::Number *n) { this->fallback_a_ = n; }
  void set_max_cell_voltage_number(number::Number *n) { this->cell_limit_ = n; }
  void set_cell_resume_voltage_number(number::Number *n) { this->cell_resume_ = n; }
  void set_pause_switch(switch_::Switch *s) { this->pause_ = s; }
  void set_topic(uint8_t topic, const std::string &name) { this->topics_[topic] = name; }
  void set_instruction_topic(const std::string &name) { this->instruction_topic_ = name; }

  void set_target_voltage_sensor(sensor::Sensor *s) { this->target_v_sensor_ = s; }
  void set_target_current_sensor(sensor::Sensor *s) { this->target_a_sensor_ = s; }
  void set_budget_sensor(sensor::Sensor *s) { this->budget_sensor_ = s; }
  void set_output_power_sensor(sensor::Sensor *s) { this->power_sensor_ = s; }
  void set_bms_age_sensor(sensor::Sensor *s) { this->bms_age_sensor_ = s; }
  void set_max_cell_age_sensor(sensor::Sensor *s) { this->max_cell_age_sensor_ = s; }
  void set_state_text_sensor(text_sensor::TextSensor *s) { this->state_sensor_ = s; }
  void set_last_fault_text_sensor(text_sensor::TextSensor *s) { this->fault_sensor_ = s; }
  void set_hold_reason_text_sensor(text_sensor::TextSensor *s) { this->hold_reason_sensor_ = s; }
  void set_bms_charge_stage_text_sensor(text_sensor::TextSensor *s) { this->stage_sensor_ = s; }
  void set_no_current_binary_sensor(binary_sensor::BinarySensor *s) { this->no_current_sensor_ = s; }

  void clear_fault() { this->clear_fault_ = true; }

 protected:
  void tick_(uint32_t now);
  const char *check_faults_(float target_v, float max_vout, float our_a);
  static void publish_throttled_(sensor::Sensor *s, float value, float deadband, float &last, uint32_t &last_ms,
                                 uint32_t now);
  static float number_or_(number::Number *n, float fallback);
  static bool same_(float a, float b) { return a == b || (std::isnan(a) && std::isnan(b)); }

  std::vector<nasn_rectifier::NasnRectifier *> rectifiers_;
  number::Number *max_v_{nullptr};
  number::Number *max_a_{nullptr};
  number::Number *fallback_v_{nullptr};
  number::Number *fallback_a_{nullptr};
  number::Number *cell_limit_{nullptr};
  number::Number *cell_resume_{nullptr};
  switch_::Switch *pause_{nullptr};
  std::string topics_[charge::TOPIC_COUNT];
  std::string instruction_topic_;

  charge::BmsTracker bms_;
  charge::StateMachine sm_;
  charge::MovingAverage battery_avg_;
  charge::MovingAverage ours_avg_;
  charge::RampLimiter ramp_;
  charge::CellLimiter cell_;
  std::vector<charge::Cooldown> cooldown_;  // one per rectifier, same order
  bool cell_fallback_{false};  // for logging invalid thresholds once per change
  float bad_limit_{NAN}, bad_resume_{NAN};
  bool cell_stale_{false};  // for logging a missing/stale max cell reading once per transition
  bool mqtt_was_connected_{false};
  bool clear_fault_{false};
  uint32_t last_tick_{0};
  char fault_text_[96]{};
  std::string published_state_;
  std::string published_fault_;
  std::string published_hold_reason_;
  std::string published_stage_;
  bool hold_reason_published_{false};
  bool stage_published_{false};

  float last_tv_{NAN}, last_ta_{NAN}, last_bu_{NAN}, last_pw_{NAN}, last_ag_{NAN};
  uint32_t last_tv_ms_{0}, last_ta_ms_{0}, last_bu_ms_{0}, last_pw_ms_{0}, last_ag_ms_{0};
  float last_mc_{NAN};
  uint32_t last_mc_ms_{0};

  sensor::Sensor *target_v_sensor_{nullptr};
  sensor::Sensor *target_a_sensor_{nullptr};
  sensor::Sensor *budget_sensor_{nullptr};
  sensor::Sensor *power_sensor_{nullptr};
  sensor::Sensor *bms_age_sensor_{nullptr};
  sensor::Sensor *max_cell_age_sensor_{nullptr};
  text_sensor::TextSensor *state_sensor_{nullptr};
  text_sensor::TextSensor *fault_sensor_{nullptr};
  text_sensor::TextSensor *hold_reason_sensor_{nullptr};
  text_sensor::TextSensor *stage_sensor_{nullptr};
  binary_sensor::BinarySensor *no_current_sensor_{nullptr};
  charge::NoCurrentDetector no_current_;
  bool no_current_published_{false};
  bool published_no_current_{false};
};

}  // namespace esphome::charge_controller
