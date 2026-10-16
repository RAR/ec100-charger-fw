// charger_update: the charger's firmware as an update entity on its WiFi bridge
// (docs/history/superpowers/specs/2026-10-16-bridge-charger-update-design.md). A check fetches the latest GitHub
// release's manifest.json; Install downloads the .dat it names, checks its size and SHA-256, takes the RS485 line from
// Modbus and runs the installer (installer.h), which refuses while the charger is charging.
#pragma once
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/modbus_controller/modbus_controller.h"
#include "esphome/components/rs485_mux/rs485_mux.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart_component.h"
#include "esphome/components/update/update_entity.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "installer.h"

#ifdef USE_ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

namespace esphome {
namespace charger_update {

// The installer's view of the UART, while the mux has handed it over.
class UartLine : public charger_update_logic::LineIo {
 public:
  explicit UartLine(uart::UARTComponent *uart) : uart_(uart) {}
  void write(const uint8_t *data, size_t len) override { this->uart_->write_array(data, len); }
  size_t read(uint8_t *data, size_t max) override {
    const size_t n = std::min<size_t>(this->uart_->available(), max);
    return n > 0 && this->uart_->read_array(data, n) ? n : 0;
  }
  void set_baud(uint32_t baud) override {
    if (this->uart_->get_baud_rate() == baud) return;
    this->uart_->set_baud_rate(baud);
    this->uart_->load_settings(false);
  }

 protected:
  uart::UARTComponent *uart_;
};

class ChargerUpdate : public update::UpdateEntity, public PollingComponent {
 public:
  void set_http(http_request::HttpRequestComponent *http) { this->http_ = http; }
  void set_mux(rs485_mux::RS485Mux *mux) { this->mux_ = mux; }
  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_controller(modbus_controller::ModbusController *c) { this->controller_ = c; }
  void set_version_sensor(text_sensor::TextSensor *s) { this->version_sensor_ = s; }
  void set_repo(const std::string &repo) { this->repo_ = repo; }

  void setup() override;
  void loop() override;
  void update() override;  // the periodic check
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  void check() override { this->update(); }
  void perform(bool force) override;

 protected:
  struct Manifest {
    std::string version, file, sha256;
    size_t size{0};
    std::string error;  // empty on success
  };
  struct Download {
    std::vector<uint8_t> data;
    std::string error;
  };

  static void check_task_(void *self);
  static void download_task_(void *self);
  void apply_manifest_(Manifest *m);
  void refresh_state_();
  void begin_install_();
  void end_install_();
  void fail_(const std::string &message);    // the install broke off after the header: power-cycle
  void refuse_(const std::string &message);  // nothing was changed on the charger

  http_request::HttpRequestComponent *http_{nullptr};
  rs485_mux::RS485Mux *mux_{nullptr};
  uart::UARTComponent *uart_{nullptr};
  modbus_controller::ModbusController *controller_{nullptr};
  text_sensor::TextSensor *version_sensor_{nullptr};
  std::string repo_;

  std::string latest_file_, latest_sha256_;
  size_t latest_size_{0};
  std::vector<uint8_t> image_;
  charger_update_logic::Installer installer_;
  std::unique_ptr<UartLine> line_;
  bool installing_{false};
  bool just_installed_{false};  // until the charger reports the new version (or a minute passes)
  HighFrequencyLoopRequester high_freq_;
  size_t published_frames_{0};
#ifdef USE_ESP32
  TaskHandle_t task_{nullptr};
#endif
};

}  // namespace charger_update
}  // namespace esphome
