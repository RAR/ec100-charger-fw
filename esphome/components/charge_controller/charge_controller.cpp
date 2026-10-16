#include "charge_controller.h"

#include <cmath>
#include <cstdio>

#include "esphome/components/mqtt/mqtt_client.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::charge_controller {

static const char *const TAG = "charge_controller";
static const uint32_t TICK_MS = 1000;

void ChargeController::setup() {
  for (int i = 0; i < charge::TOPIC_COUNT; i++) {
    const auto topic = charge::Topic(i);
    mqtt::global_mqtt_client->subscribe(this->topics_[i], [this, topic](const std::string &, const std::string &payload) {
      const uint32_t now = millis();
      // A retained replay may be hours old: only live updates feed the battery-current average.
      const bool live = this->bms_.on_message(topic, payload, now);
      if (live && topic == charge::Topic::CURRENT) this->battery_avg_.add(this->bms_.current(), now);
    });
  }
  if (!this->instruction_topic_.empty()) {
    // Informational only (Bulk / Absorption / Float ...): passed through to HA, never used for control.
    mqtt::global_mqtt_client->subscribe(this->instruction_topic_, [this](const std::string &, const std::string &payload) {
      if (this->stage_sensor_ == nullptr) return;
      if (this->stage_published_ && payload == this->published_stage_) return;
      this->stage_sensor_->publish_state(payload);
      this->published_stage_ = payload;
      this->stage_published_ = true;
    });
  }
}

void ChargeController::loop() {
  const uint32_t now = millis();
  const bool connected = mqtt::global_mqtt_client->is_connected();
  if (connected && !this->mqtt_was_connected_) this->bms_.on_reconnect();
  this->mqtt_was_connected_ = connected;
  if (now - this->last_tick_ >= TICK_MS) {
    this->last_tick_ = now;
    this->tick_(now);
  }
}

float ChargeController::number_or_(number::Number *n, float fallback) {
  return (n != nullptr && std::isfinite(n->state)) ? n->state : fallback;
}

void ChargeController::tick_(uint32_t now) {
  const float fb_v = number_or_(this->fallback_v_, 54.0f);
  const float fb_a = number_or_(this->fallback_a_, 5.0f);
  float our_a = 0.0f;
  float power = 0.0f;
  float max_vout = NAN;
  float min_vin = NAN;
  int healthy = 0;
  bool boot_done = true;
  for (auto *r : this->rectifiers_) {
    r->set_fallback(fb_v, fb_a);
    boot_done = boot_done && r->boot_sent();
    if (!r->is_online()) continue;
    const float v = r->output_voltage(), i = r->output_current();
    if (std::isfinite(i)) our_a += i;
    if (std::isfinite(v) && std::isfinite(i)) power += v * i;
    if (std::isfinite(v)) max_vout = std::isfinite(max_vout) ? std::max(max_vout, v) : v;
    const float vin = r->input_voltage();
    if (std::isfinite(vin)) min_vin = std::isfinite(min_vin) ? std::min(min_vin, vin) : vin;
    if (!r->faulted()) healthy++;
  }
  this->ours_avg_.add(our_a, now);

  const float target_v = charge::target_voltage(this->bms_.cvl(), number_or_(this->max_v_, 57.0f));
  // Stock-firmware parity: at most 60 A total while any online PSU sees AC input below 180 V.
  const float max_a = charge::line_limited_max(number_or_(this->max_a_, 0.0f), min_vin);
  const float budget =
      charge::current_budget(this->bms_.ccl(), this->battery_avg_.value(), this->ours_avg_.value(), max_a);

  charge::Inputs in;
  in.boot_done = boot_done;
  in.bms_fresh = this->bms_.fresh(now);
  in.allowed = this->bms_.allowed();
  in.bms_alarm = this->bms_.alarm();
  in.paused = this->pause_ == nullptr || this->pause_->state;
  in.bms_blocking = this->bms_.blocking();
  // Raw values (NaN included) so one bad threshold makes both fall back to the defaults.
  const float limit_in = this->cell_limit_ != nullptr ? this->cell_limit_->state : charge::CELL_LIMIT_V;
  const float resume_in = this->cell_resume_ != nullptr ? this->cell_resume_->state : charge::CELL_RESUME_V;
  const charge::CellThresholds cell = charge::cell_thresholds(limit_in, resume_in);
  if (cell.fallback && !(this->cell_fallback_ && same_(limit_in, this->bad_limit_) && same_(resume_in, this->bad_resume_))) {
    ESP_LOGW(TAG, "Invalid cell thresholds (max %.3f V, resume %.3f V): using %.2f / %.2f V", limit_in, resume_in,
             charge::CELL_LIMIT_V, charge::CELL_RESUME_V);
  }
  this->cell_fallback_ = cell.fallback;
  this->bad_limit_ = limit_in;
  this->bad_resume_ = resume_in;
  in.cell_high = this->cell_.update(this->bms_.max_cell(), cell.limit, cell.resume, this->bms_.max_cell_live());
  // Not part of freshness, but a cutoff without cell data is blind: make that visible.
  const uint32_t cell_age = this->bms_.topic_age_ms(charge::Topic::MAX_CELL, now);
  const bool cell_stale = cell_age == UINT32_MAX ? now > charge::FRESH_MS : cell_age > charge::FRESH_MS;
  if (cell_stale != this->cell_stale_) {
    if (cell_stale) {
      ESP_LOGW(TAG, "Max cell voltage missing or stale for > %u s: cell-voltage cutoff is blind",
               (unsigned) (charge::FRESH_MS / 1000));
    } else {
      ESP_LOGI(TAG, "Max cell voltage updating again");
    }
    this->cell_stale_ = cell_stale;
  }
  in.budget = budget;
  in.fault = this->check_faults_(target_v, max_vout, our_a);
  in.clear_fault = this->clear_fault_;
  this->clear_fault_ = false;
  const charge::State state = this->sm_.update(in, now);

  const bool charging = state == charge::State::CHARGING;
  float total = 0.0f;
  if (charging && healthy > 0) {
    total = this->ramp_.step(budget, now);
  } else {
    this->ramp_.reset();
  }
  const float per_psu = charge::per_psu_current(total, healthy);
  const float commanded = charging ? per_psu * healthy : 0.0f;
  int cooling = 0;
  for (size_t k = 0; k < this->rectifiers_.size(); k++) {
    auto *r = this->rectifiers_[k];
    const bool run = charging && r->is_online() && !r->faulted();
    const bool cool_ok = r->is_online() && !r->faulted() && state != charge::State::FAULT &&
                         r->output_voltage() >= charge::COOLDOWN_MIN_BATTERY_V;
    const bool was_cooling = this->cooldown_[k].active();
    const bool cool = this->cooldown_[k].update(run, cool_ok, r->heatsink_temperature(), now);
    if (cool != was_cooling) {
      ESP_LOGI(TAG, "PSU %u %s at %.1f °C", r->address(), cool ? "cooling down before standby" : "cooldown over",
               r->heatsink_temperature());
    }
    if (cool) {
      cooling++;
      r->set_target(charge::COOLDOWN_V, charge::COOLDOWN_A);
      r->set_standby(false);
    } else {
      r->set_target(std::isfinite(target_v) ? target_v : fb_v, run ? per_psu : 0.0f);
      r->set_standby(!run);
    }
  }

  this->publish_throttled_(this->target_v_sensor_, charging ? target_v : NAN, 0.01f, this->last_tv_, this->last_tv_ms_, now);
  this->publish_throttled_(this->target_a_sensor_, commanded, 0.5f, this->last_ta_, this->last_ta_ms_, now);
  this->publish_throttled_(this->budget_sensor_, budget, 0.5f, this->last_bu_, this->last_bu_ms_, now);
  this->publish_throttled_(this->power_sensor_, power, 20.0f, this->last_pw_, this->last_pw_ms_, now);
  if (this->bms_age_sensor_ != nullptr) {
    const uint32_t age = this->bms_.age_ms(now);
    this->publish_throttled_(this->bms_age_sensor_, age == UINT32_MAX ? NAN : age / 1000.0f, INFINITY, this->last_ag_,
                             this->last_ag_ms_, now);
  }
  this->publish_throttled_(this->max_cell_age_sensor_, cell_age == UINT32_MAX ? NAN : cell_age / 1000.0f, INFINITY,
                           this->last_mc_, this->last_mc_ms_, now);
  // A CHARGING state with no healthy PSU to drive is worth spelling out in HA.
  std::string state_str = charge::state_name(state);
  const float battery_v = this->bms_.battery_voltage(now);
  const bool no_current = this->no_current_.update(
      charge::no_current_condition(charging, commanded, our_a, max_vout, target_v, battery_v), now);
  if (no_current != this->published_no_current_ || !this->no_current_published_) {
    if (no_current) {
      ESP_LOGW(TAG, "Charging without current: %.1f A commanded, %.1f A out at %.2f V, battery %.2f V - "
                    "is the DC output connected?", commanded, our_a, max_vout, battery_v);
    } else if (this->no_current_published_) {
      ESP_LOGI(TAG, "Charge current flowing again (or no longer charging)");
    }
    if (this->no_current_sensor_ != nullptr) this->no_current_sensor_->publish_state(no_current);
    this->published_no_current_ = no_current;
    this->no_current_published_ = true;
  }
  if (charging && healthy == 0) state_str += " (no PSU)";
  if (no_current) state_str += " (no current)";
  if (cooling > 0) state_str += " (cooling)";
  if (this->state_sensor_ != nullptr && state_str != this->published_state_) {
    ESP_LOGI(TAG, "State %s -> %s", this->published_state_.c_str(), state_str.c_str());
    this->state_sensor_->publish_state(state_str);
    this->published_state_ = state_str;
  }
  const char *reason = "";
  if (state == charge::State::HOLD) {
    reason = charge::hold_reason(in);
  } else if (state == charge::State::WAIT_BMS) {
    reason = "waiting for BMS data";
  }
  if (this->hold_reason_sensor_ != nullptr &&
      (!this->hold_reason_published_ || this->published_hold_reason_ != reason)) {
    this->hold_reason_sensor_->publish_state(reason);
    this->published_hold_reason_ = reason;
    this->hold_reason_published_ = true;
  }
  const char *last_fault = this->sm_.last_fault();
  const std::string fault_text = *last_fault != '\0' ? last_fault : "none";
  if (this->fault_sensor_ != nullptr && fault_text != this->published_fault_) {
    this->fault_sensor_->publish_state(fault_text);
    this->published_fault_ = fault_text;
  }
}

void ChargeController::publish_throttled_(sensor::Sensor *s, float value, float deadband, float &last,
                                          uint32_t &last_ms, uint32_t now) {
  if (s == nullptr) return;
  const bool first = last_ms == 0;
  const bool nan_change = std::isnan(value) != std::isnan(last);
  const bool moved = !std::isnan(value) && !std::isnan(last) && std::fabs(value - last) >= deadband;
  if (first || nan_change || moved || now - last_ms >= 10000) {
    s->publish_state(value);
    last = value;
    last_ms = now == 0 ? 1 : now;
  }
}

const char *ChargeController::check_faults_(float target_v, float max_vout, float our_a) {
  for (auto *r : this->rectifiers_) {
    if (r->faulted()) {
      if (r->setpoint_mismatch()) {
        snprintf(this->fault_text_, sizeof(this->fault_text_),
                 "PSU %u not following setpoints (alarms 0x%08X, write errors %u)", r->address(), (unsigned) r->alarms(),
                 (unsigned) r->consecutive_write_errors());
      } else {
        snprintf(this->fault_text_, sizeof(this->fault_text_), "PSU %u fault: alarms 0x%08X, write errors %u",
                 r->address(), (unsigned) r->alarms(), (unsigned) r->consecutive_write_errors());
      }
      return this->fault_text_;
    }
  }
  if (charge::overvoltage(this->sm_.state() == charge::State::CHARGING, max_vout, target_v, our_a)) {
    snprintf(this->fault_text_, sizeof(this->fault_text_), "Overvoltage: %.2f V measured, target %.2f V", max_vout,
             target_v);
    return this->fault_text_;
  }
  return nullptr;
}

void ChargeController::dump_config() {
  ESP_LOGCONFIG(TAG, "Charge controller:");
  ESP_LOGCONFIG(TAG, "  Rectifiers: %u", (unsigned) this->rectifiers_.size());
  for (int i = 0; i < charge::TOPIC_COUNT; i++) ESP_LOGCONFIG(TAG, "  Topic %d: %s", i, this->topics_[i].c_str());
  ESP_LOGCONFIG(TAG, "  Charging instruction topic: %s", this->instruction_topic_.c_str());
}

}  // namespace esphome::charge_controller
