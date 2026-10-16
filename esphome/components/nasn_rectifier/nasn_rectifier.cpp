#include "nasn_rectifier.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::nasn_rectifier {

static const char *const TAG = "nasn_rectifier";
static const uint32_t POLL_SPACING_MS = 20;

static float convert(uint8_t kind, uint64_t raw) {
  switch (kind) {
    case KIND_VOLTS:
      return nasn::setpoint_volts(raw);
    case KIND_AMPS:
      return nasn::setpoint_amps(raw);
    case KIND_RAW:
      return float(raw);
    default:
      return nasn::telemetry(raw);
  }
}

void NasnRectifier::setup() {
  // Registers the control loop needs, whether or not they have sensors.
  for (uint16_t reg : {nasn::reg::VOUT, nasn::reg::VOUT_ALT, nasn::reg::IOUT, nasn::reg::INPUT_VOLTAGE,
                       nasn::reg::TEMP_HEATSINK, nasn::reg::ALARMS, nasn::reg::VOUT_SET, nasn::reg::VOUT_READBACK, nasn::reg::ILIMIT,
                       nasn::reg::STANDBY})
    this->add_poll_reg_(reg);
  if (this->write_stored_defaults_) {
    this->add_poll_reg_(nasn::reg::VOUT_DEFAULT);
    this->add_poll_reg_(nasn::reg::ILIMIT_DEFAULT);
  }
  this->target_v_ = this->fallback_v_;
  this->canbus_->add_callback([this](uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data) {
    this->on_frame_(can_id, extended, rtr, data);
  });
  if (this->online_sensor_ != nullptr) this->online_sensor_->publish_state(false);
}

void NasnRectifier::add_sensor(uint16_t reg, uint8_t kind, sensor::Sensor *sensor) {
  this->entries_.push_back({reg, kind, sensor});
  this->add_poll_reg_(reg);
}

void NasnRectifier::add_poll_reg_(uint16_t reg) {
  for (uint16_t r : this->poll_regs_) {
    if (r == reg) return;
  }
  this->poll_regs_.push_back(reg);
}

void NasnRectifier::update() { this->poll_pos_ = 0; }

void NasnRectifier::loop() {
  const uint32_t now = millis();
  const bool due = now - this->last_setpoint_send_ >= this->keepalive_ms_;
  // Stored defaults live in the module's flash: rewrite them only when the readback says they differ,
  // and at most once per keepalive interval.
  if (this->write_stored_defaults_ && this->online_ && (this->default_v_wrong_ || this->default_a_wrong_) &&
      (!this->defaults_written_ || now - this->last_defaults_write_ >= this->keepalive_ms_)) {
    this->write_stored_defaults_now_(now);
  }
  if (due || (this->dirty_ && !this->setpoints_failed_)) {
    const bool ok = this->send_setpoints_();
    this->setpoints_failed_ = !ok;
    if (ok) {
      this->dirty_ = false;
      this->boot_sent_ = true;
    }
    this->last_setpoint_send_ = now;
  }
  if (this->poll_pos_ < this->poll_regs_.size() && now - this->last_poll_send_ >= POLL_SPACING_MS) {
    const uint16_t reg = this->poll_regs_[this->poll_pos_++];
    const int i = live_index_(reg);
    // Writes are queued ahead of this read, so the reply should show what we had sent by now.
    if (i >= 0) this->polled_[i] = this->sent_[i];
    this->send_(nasn::read_id(this->address_), nasn::read_request(reg));
    this->last_poll_send_ = now;
  }
  if (this->online_ && now - this->last_reply_ > 3 * this->get_update_interval()) this->set_online_(false);

  const bool wrong = this->readback_wrong_[0] || this->readback_wrong_[1] || this->readback_wrong_[2];
  const bool mismatch = this->online_ && this->mismatch_timer_.update(wrong, now);
  if (mismatch && !this->setpoint_mismatch_) {
    ESP_LOGW(TAG, "PSU %u not following setpoints (voltage %s, current %s, standby %s)", this->address_,
             this->readback_wrong_[0] ? "off" : "ok", this->readback_wrong_[1] ? "off" : "ok",
             this->readback_wrong_[2] ? "off" : "ok");
  }
  this->setpoint_mismatch_ = mismatch;
}

// Index into sent_/polled_/readback_wrong_ for a register whose readback is checked. The voltage
// written to 0x0100 is checked on its 0x022F readback (as the stock firmware does).
int NasnRectifier::live_index_(uint16_t reg) {
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

bool NasnRectifier::send_voltage_() {
  const uint16_t raw = nasn::voltage_raw(this->target_v_);
  if (!this->send_(nasn::write_id(this->address_), nasn::write_u16(nasn::reg::VOUT_SET, raw))) return false;
  this->sent_[0] = raw;
  return true;
}

bool NasnRectifier::send_current_() {
  const uint16_t raw = nasn::current_raw(this->target_a_);
  if (!this->send_(nasn::write_id(this->address_), nasn::write_u16(nasn::reg::ILIMIT, raw))) return false;
  this->sent_[1] = raw;
  return true;
}

bool NasnRectifier::send_standby_() {
  if (!this->send_(nasn::write_id(this->address_), nasn::write_standby(this->standby_))) return false;
  this->sent_[2] = this->standby_ ? 1 : 0;
  return true;
}

bool NasnRectifier::send_setpoints_() {
  bool ok = this->send_voltage_();
  ok = this->send_current_() && ok;
  ok = this->send_standby_() && ok;
  return ok;
}

void NasnRectifier::stop_now() {
  this->target_a_ = 0.0f;
  this->standby_ = true;
  bool ok = this->send_current_();
  ok = this->send_standby_() && ok;
  ESP_LOGI(TAG, "PSU %u: stop (current 0, standby)%s", this->address_, ok ? "" : " - send failed, will retry");
  if (!ok) {
    this->dirty_ = true;  // loop() retries straight away if it gets to run
    this->setpoints_failed_ = false;
  }
}

void NasnRectifier::write_stored_defaults_now_(uint32_t now) {
  const uint32_t id = nasn::write_id(this->address_);
  ESP_LOGI(TAG, "PSU %u: writing stored defaults %.1f V / %.1f A", this->address_, this->fallback_v_,
           this->fallback_a_);
  if (this->default_v_wrong_) this->send_(id, nasn::write_voltage(nasn::reg::VOUT_DEFAULT, this->fallback_v_));
  if (this->default_a_wrong_) this->send_(id, nasn::write_current(nasn::reg::ILIMIT_DEFAULT, this->fallback_a_));
  // The next readback decides whether another write is needed.
  this->default_v_wrong_ = false;
  this->default_a_wrong_ = false;
  this->defaults_written_ = true;
  this->last_defaults_write_ = now;
}

bool NasnRectifier::send_(uint32_t can_id, const nasn::Frame &frame) {
  std::vector<uint8_t> data(frame.begin(), frame.end());
  return this->canbus_->send_data(can_id, true, data) == canbus::ERROR_OK;
}

void NasnRectifier::set_target(float volts, float amps) {
  if (!std::isfinite(volts) || !std::isfinite(amps)) return;
  if (std::fabs(volts - this->target_v_) >= 0.01f || std::fabs(amps - this->target_a_) >= 0.1f) {
    this->target_v_ = volts;
    this->target_a_ = amps;
    this->dirty_ = true;
  }
}

void NasnRectifier::set_standby(bool standby) {
  if (standby != this->standby_) {
    this->standby_ = standby;
    this->dirty_ = true;
  }
}

void NasnRectifier::set_fallback(float volts, float amps) {
  if (!std::isfinite(volts) || !std::isfinite(amps)) return;
  // Stored defaults are rewritten when their next readback differs (see loop()).
  this->fallback_v_ = std::clamp(volts, nasn::MIN_V, FALLBACK_MAX_V);
  this->fallback_a_ = std::clamp(amps, 0.0f, FALLBACK_MAX_A);
}

void NasnRectifier::on_frame_(uint32_t can_id, bool extended, bool rtr, const std::vector<uint8_t> &data) {
  auto reply = nasn::parse_reply(can_id, extended, data.data(), data.size());
  if (!reply || reply->addr != this->address_) return;

  if (reply->cmd == nasn::CMD_WRITE) {
    if (reply->status != 0) {
      this->consecutive_write_errors_++;
      ESP_LOGW(TAG, "PSU %u: write to 0x%04X rejected (status %u)", this->address_, reply->reg, reply->status);
    } else {
      this->consecutive_write_errors_ = 0;
    }
    return;
  }
  if (reply->status != 0) {  // failed reads are not liveness
    ESP_LOGV(TAG, "PSU %u: read of 0x%04X failed (status %u)", this->address_, reply->reg, reply->status);
    return;
  }
  this->last_reply_ = millis();
  if (!this->online_) this->set_online_(true);
  if (reply->reg == nasn::reg::VOUT) {
    this->vout_ = nasn::telemetry(reply->value);
  } else if (reply->reg == nasn::reg::VOUT_ALT) {
    this->vout_alt_ = nasn::telemetry(reply->value);
  } else if (reply->reg == nasn::reg::IOUT) {
    this->iout_ = nasn::telemetry(reply->value);
    if (this->output_power_sensor_ != nullptr && std::isfinite(this->vout_))
      this->output_power_sensor_->publish_state(this->vout_ * this->iout_);
  } else if (reply->reg == nasn::reg::INPUT_VOLTAGE) {
    this->vin_ = nasn::telemetry(reply->value);
  } else if (reply->reg == nasn::reg::TEMP_HEATSINK) {
    this->heatsink_c_ = nasn::telemetry(reply->value);
  } else if (reply->reg == nasn::reg::ALARMS) {
    this->alarms_ = nasn::alarm_word(reply->value);
  } else if (reply->reg == nasn::reg::VOUT_DEFAULT && this->write_stored_defaults_) {
    this->default_v_wrong_ =
        nasn::stored_default_mismatch(reply->reg, reply->value, nasn::voltage_raw(this->fallback_v_));
  } else if (reply->reg == nasn::reg::ILIMIT_DEFAULT && this->write_stored_defaults_) {
    this->default_a_wrong_ =
        nasn::stored_default_mismatch(reply->reg, reply->value, nasn::current_raw(this->fallback_a_));
  }
  if (reply->reg == nasn::reg::STANDBY) {
    this->standby_readback_raw_ = reply->value;
    ESP_LOGV(TAG, "PSU %u: 0x0132 readback 0x%012llX", this->address_, (unsigned long long) reply->value);
  }
  const int i = live_index_(reply->reg);
  if (i >= 0 && this->polled_[i] >= 0)
    this->readback_wrong_[i] = nasn::readback_mismatch(reply->reg, reply->value, uint64_t(this->polled_[i]));
  for (auto &e : this->entries_) {
    if (e.reg == reply->reg) e.sensor->publish_state(convert(e.kind, reply->value));
  }
}

void NasnRectifier::set_online_(bool online) {
  this->online_ = online;
  ESP_LOGI(TAG, "PSU %u %s", this->address_, online ? "online" : "offline");
  if (this->online_sensor_ != nullptr) this->online_sensor_->publish_state(online);
  if (online) {
    this->dirty_ = true;  // a module that just appeared needs our setpoints now
  } else {
    this->vout_ = NAN;
    this->vout_alt_ = NAN;
    this->iout_ = NAN;
    this->vin_ = NAN;
    this->heatsink_c_ = NAN;
    this->alarms_ = 0;
    this->consecutive_write_errors_ = 0;
    for (int i = 0; i < 3; i++) {
      this->readback_wrong_[i] = false;
      this->polled_[i] = -1;
    }
    this->mismatch_timer_.reset();
    this->setpoint_mismatch_ = false;
    this->default_v_wrong_ = false;
    this->default_a_wrong_ = false;
    for (auto &e : this->entries_) e.sensor->publish_state(NAN);
    if (this->output_power_sensor_ != nullptr) this->output_power_sensor_->publish_state(NAN);
  }
}

void NasnRectifier::dump_config() {
  ESP_LOGCONFIG(TAG, "NASN rectifier:");
  ESP_LOGCONFIG(TAG, "  Address: %u", this->address_);
  ESP_LOGCONFIG(TAG, "  Keepalive: %u ms", (unsigned) this->keepalive_ms_);
  ESP_LOGCONFIG(TAG, "  Fallback: %.1f V / %.1f A (write stored defaults: %s)", this->fallback_v_, this->fallback_a_,
                YESNO(this->write_stored_defaults_));
  ESP_LOGCONFIG(TAG, "  Alarm mask: 0x%08X", (unsigned) this->alarm_mask_);
  ESP_LOGCONFIG(TAG, "  Polled registers: %u", (unsigned) this->poll_regs_.size());
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace esphome::nasn_rectifier
