#include "charger_update.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>

#include "esphome/components/json/json_util.h"
#include "esphome/components/network/util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32
#include <esp_heap_caps.h>
#include <mbedtls/sha256.h>
#endif

namespace esphome {
namespace charger_update {

static const char *const TAG = "charger_update";
static const size_t READ_CHUNK = 512;
static const size_t MANIFEST_MAX = 1024;
static const uint32_t SETTLE_AFTER_INSTALL_MS = 60000;  // the charger reboots and is polled again well within this
static const size_t HEAP_MARGIN = 16384;                // left free beside the image for TLS, MQTT and the API

// "M.m" -> M * 256 + m, as IR 0; -1 if it isn't one.
static int parse_version(const std::string &s) {
  unsigned major, minor;
  char extra;
  if (std::sscanf(s.c_str(), "%u.%u%c", &major, &minor, &extra) != 2 || major > 255 || minor > 255) return -1;
  return int(major << 8 | minor);
}

static bool safe_file_name(const std::string &s) {
  if (s.empty() || s.size() > 64) return false;
  for (char c : s) {
    if (!(std::isalnum((unsigned char) c) || c == '.' || c == '-' || c == '_')) return false;
  }
  return true;
}

void ChargerUpdate::setup() {
  this->update_info_.title = "Charger firmware";
  this->update_info_.release_url = "https://github.com/" + this->repo_ + "/releases/latest";
  this->version_sensor_->add_on_state_callback([this](const std::string &) { this->refresh_state_(); });
  // The first check once WiFi is up; then every update_interval.
  this->set_interval("first_check", 10000, [this]() {
    if (!network::is_connected()) return;
    this->cancel_interval("first_check");
    this->update();
  });
}

void ChargerUpdate::dump_config() {
  ESP_LOGCONFIG(TAG, "Charger firmware update:\n  Releases: https://github.com/%s", this->repo_.c_str());
}

// ---- check ----

void ChargerUpdate::update() {
  if (!network::is_connected() || this->installing_) return;
#ifdef USE_ESP32
  if (this->task_ != nullptr) return;
  xTaskCreate(ChargerUpdate::check_task_, "charger_check", 8192, this, 1, &this->task_);
#else
  check_task_(this);
#endif
}

void ChargerUpdate::check_task_(void *param) {
  auto *self = static_cast<ChargerUpdate *>(param);
  auto *m = new Manifest();
  const std::string url = "https://github.com/" + self->repo_ + "/releases/latest/download/manifest.json";
  auto c = self->http_->get(url);
  if (c == nullptr || c->status_code != 200 || c->content_length == 0 || c->content_length > MANIFEST_MAX) {
    m->error = "could not fetch " + url;
  } else {
    std::vector<uint8_t> buf(c->content_length);
    auto r = http_request::http_read_fully(c.get(), buf.data(), buf.size(), READ_CHUNK, self->http_->get_timeout());
    if (r.status != http_request::HttpReadStatus::OK) {
      m->error = "could not read the manifest";
    } else if (!json::parse_json(buf.data(), buf.size(), [m](JsonObject root) -> bool {
                 if (!root["version"].is<const char *>() || !root["file"].is<const char *>() ||
                     !root["sha256"].is<const char *>() || !root["size"].is<unsigned>())
                   return false;
                 m->version = root["version"].as<std::string>();
                 m->file = root["file"].as<std::string>();
                 m->sha256 = root["sha256"].as<std::string>();
                 m->size = root["size"].as<unsigned>();
                 return true;
               })) {
      m->error = "the manifest isn't valid JSON or lacks version, file, size or sha256";
    } else if (parse_version(m->version) < 0 || !safe_file_name(m->file) || m->sha256.size() != 64 ||
               m->size == 0 || m->size % 8 != 0 || m->size > charger_update_logic::MAX_IMAGE) {
      m->error = "the manifest's values are out of range";
    }
  }
  if (c != nullptr) c->end();
  c.reset();
  self->defer([self, m]() {
#ifdef USE_ESP32
    self->task_ = nullptr;
#endif
    self->apply_manifest_(m);
    delete m;
  });
#ifdef USE_ESP32
  vTaskDelete(nullptr);
#endif
}

void ChargerUpdate::apply_manifest_(Manifest *m) {
  if (!m->error.empty()) {
    ESP_LOGW(TAG, "Check failed: %s", m->error.c_str());
    this->status_set_warning(LOG_STR("Could not check GitHub for charger firmware"));
    return;
  }
  this->status_clear_warning();
  ESP_LOGI(TAG, "Latest charger release on GitHub: %s", m->version.c_str());
  this->update_info_.latest_version = m->version;
  this->update_info_.firmware_url =
      "https://github.com/" + this->repo_ + "/releases/download/v" + m->version + "/" + m->file;
  this->update_info_.release_url = "https://github.com/" + this->repo_ + "/releases/tag/v" + m->version;
  this->latest_file_ = m->file;
  this->latest_sha256_ = m->sha256;
  this->latest_size_ = m->size;
  this->refresh_state_();
}

// AVAILABLE when the release is newer than the charger, or the charger's version is unknown (it isn't answering:
// stock firmware, or stuck in the bootloader after a failed install, which this recovers).
void ChargerUpdate::refresh_state_() {
  if (this->installing_) return;
  const bool has = this->version_sensor_->has_state();
  this->update_info_.current_version = has ? this->version_sensor_->get_state() : "";
  const int latest = parse_version(this->update_info_.latest_version);
  const int current = parse_version(this->update_info_.current_version);
  if (this->just_installed_ && current >= latest) this->just_installed_ = false;
  update::UpdateState s = update::UPDATE_STATE_UNKNOWN;
  if (this->just_installed_) {
    s = update::UPDATE_STATE_NO_UPDATE;  // the version sensor hasn't caught up with the reboot yet
  } else if (latest >= 0) {
    s = current < 0 || latest > current ? update::UPDATE_STATE_AVAILABLE : update::UPDATE_STATE_NO_UPDATE;
  }
  const bool newly = s == update::UPDATE_STATE_AVAILABLE && this->state_ != update::UPDATE_STATE_AVAILABLE;
  this->state_ = s;
  this->publish_state();
  if (newly) this->get_update_available_trigger()->trigger(this->update_info_);
}

// ---- install ----

void ChargerUpdate::perform(bool force) {
  if (this->installing_) return;
  if (this->state_ != update::UPDATE_STATE_AVAILABLE && !force) return;
#ifdef USE_ESP32
  if (this->task_ != nullptr) {
    this->refuse_("a check for updates is still running; try again in a few seconds");
    return;
  }
#endif
  if (this->latest_size_ == 0) {
    this->refuse_("no release known yet; check for updates first");
    return;
  }
  if (this->mux_->passthrough()) {
    this->refuse_("Passthrough is on (a tool is using the line); turn it off, then install again");
    return;
  }
  this->status_clear_warning();
  this->status_clear_error();
  this->installing_ = true;
  this->update_info_.summary = "Downloading " + this->latest_file_;
  this->update_info_.has_progress = true;
  this->update_info_.progress = 0.0f;
  this->state_ = update::UPDATE_STATE_INSTALLING;
  this->publish_state();
#ifdef USE_ESP32
  xTaskCreate(ChargerUpdate::download_task_, "charger_dl", 8192, this, 1, &this->task_);
#else
  download_task_(this);
#endif
}

void ChargerUpdate::download_task_(void *param) {
  auto *self = static_cast<ChargerUpdate *>(param);
  auto *d = new Download();
  const size_t size = self->latest_size_;
  auto c = self->http_->get(self->update_info_.firmware_url);
  if (c == nullptr || c->status_code != 200) {
    d->error = "could not download " + self->update_info_.firmware_url;
  } else if (c->content_length != size) {
    d->error = "the download's size doesn't match the manifest";
#ifdef USE_ESP32
  } else if (heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) < size + HEAP_MARGIN) {
    d->error = "not enough free memory on the bridge for the image";
#endif
  } else {
    d->data.resize(size);
    auto r = http_request::http_read_fully(c.get(), d->data.data(), size, READ_CHUNK, self->http_->get_timeout());
    if (r.status != http_request::HttpReadStatus::OK) {
      d->error = "the download was cut short";
    } else {
#ifdef USE_ESP32
      uint8_t sum[32];
      mbedtls_sha256(d->data.data(), size, sum, 0);
      char hex[65];
      for (int i = 0; i < 32; i++) std::snprintf(hex + 2 * i, 3, "%02x", sum[i]);
      if (self->latest_sha256_ != hex) d->error = "the download's SHA-256 doesn't match the manifest";
#endif
    }
  }
  if (c != nullptr) c->end();
  c.reset();
  if (!d->error.empty()) d->data.clear();
  self->defer([self, d]() {
#ifdef USE_ESP32
    self->task_ = nullptr;
#endif
    if (!d->error.empty()) {
      self->installing_ = false;
      self->refuse_(d->error);  // nothing was sent to the charger
    } else {
      self->image_ = std::move(d->data);
      self->begin_install_();
    }
    delete d;
  });
#ifdef USE_ESP32
  vTaskDelete(nullptr);
#endif
}

void ChargerUpdate::begin_install_() {
  if (this->mux_->passthrough()) {  // turned on while downloading
    this->installing_ = false;
    this->image_.clear();
    this->refuse_("Passthrough is on (a tool is using the line); turn it off, then install again");
    return;
  }
  ESP_LOGI(TAG, "Installing %s (%u bytes) on the charger", this->latest_file_.c_str(), (unsigned) this->image_.size());
  this->controller_->stop_poller();
  this->mux_->set_exclusive(true);
  if (!this->line_) this->line_.reset(new UartLine(this->uart_));
  this->installer_.start(this->image_.data(), this->image_.size(), millis());
  this->high_freq_.start();  // a frame per loop pass, not per 16 ms
  this->published_frames_ = 0;
  this->update_info_.summary = "Installing " + this->latest_file_;
  this->publish_state();
}

void ChargerUpdate::loop() {
  if (!this->installing_ || this->image_.empty()) return;
  if (this->installer_.active()) {
    this->installer_.step(*this->line_, millis());
    const size_t done = this->installer_.frames_done();
    const bool last = done == this->installer_.frames_total() && done != this->published_frames_;
    if (done >= this->published_frames_ + 10 || last) {
      this->published_frames_ = done;
      this->update_info_.progress = this->installer_.progress();
      this->publish_state();
    }
  }
  if (!this->installer_.active()) this->end_install_();
}

void ChargerUpdate::end_install_() {
  using charger_update_logic::Outcome;
  this->high_freq_.stop();
  this->line_->set_baud(charger_update_logic::MODBUS_BAUD);
  this->mux_->set_exclusive(false);
  if (!this->mux_->passthrough()) this->controller_->start_poller();  // else passthrough_off resumes it
  this->image_.clear();
  this->image_.shrink_to_fit();
  this->installing_ = false;
  this->update_info_.has_progress = false;
  const Outcome o = this->installer_.outcome();
  if (o == Outcome::INSTALLED) {
    ESP_LOGI(TAG, "Installed %s; the charger is starting it", this->latest_file_.c_str());
    this->update_info_.summary = "Installed " + this->latest_file_;
    this->just_installed_ = true;
    this->refresh_state_();
    this->set_timeout("settle", SETTLE_AFTER_INSTALL_MS, [this]() {
      this->just_installed_ = false;
      this->refresh_state_();
    });
    return;
  }
  const char *e = this->installer_.error() != nullptr ? this->installer_.error() : "install failed";
  if (o == Outcome::FAILED_AFTER_HEADER) {
    if (this->installer_.detail()[0] != '\0') ESP_LOGE(TAG, "Bootloader: %s", this->installer_.detail());
    this->fail_(e);
  } else {
    this->refuse_(e);  // the charger's app is untouched
  }
}

// Nothing was changed on the charger: a warning, not a fault.
void ChargerUpdate::refuse_(const std::string &message) {
  ESP_LOGW(TAG, "Charger firmware: %s", message.c_str());
  this->status_set_warning(LOG_STR("Charger firmware not installed"));
  this->update_info_.summary = message;
  this->update_info_.has_progress = false;
  this->state_ = update::UPDATE_STATE_UNKNOWN;
  this->refresh_state_();
}

void ChargerUpdate::fail_(const std::string &message) {
  ESP_LOGE(TAG, "Charger firmware: %s", message.c_str());
  this->status_set_error(LOG_STR("Charger firmware install failed"));
  this->update_info_.summary = message;
  this->update_info_.has_progress = false;
  this->state_ = update::UPDATE_STATE_UNKNOWN;  // refresh_state_ sets AVAILABLE / NO_UPDATE again
  this->refresh_state_();
}

}  // namespace charger_update
}  // namespace esphome
