// rs485_mux: one RS485 UART shared by modbus_controller (Home Assistant polling) and stream_server (raw TCP for the
// tools). Both read every byte they're given, so the mux hands the line to one side at a time: only the active port
// receives bytes and may transmit. MODBUS by default; PASSTHROUGH while set_passthrough(true).
// docs/history/superpowers/specs/2026-10-15-oem-bridge-home-assistant-design.md §3.
#pragma once
#include <deque>

#include "esphome/components/uart/uart_component.h"
#include "esphome/core/component.h"
#include "mux_logic.h"

namespace esphome {
namespace rs485_mux {

class RS485Mux;

// One side of the mux. Looks like a UART to its user; settings (baud, bits) mirror the real UART.
class MuxPort : public uart::UARTComponent {
 public:
  void write_array(const uint8_t *data, size_t len) override;
  bool peek_byte(uint8_t *data) override;
  bool read_array(uint8_t *data, size_t len) override;
  size_t available() override { return this->rx_.size(); }
  uart::UARTFlushResult flush() override;
#if defined(USE_ESP8266) || defined(USE_ESP32)
  void load_settings(bool dump_config) override {}  // the real UART owns the line settings
#endif

  void set_mux(RS485Mux *mux) { this->mux_ = mux; }
  void push(const uint8_t *data, size_t len);
  void clear() { this->rx_.clear(); }

 protected:
  void check_logger_conflict() override {}

  static constexpr size_t MAX_RX = 1024;
  RS485Mux *mux_{nullptr};
  std::deque<uint8_t> rx_;
};

class RS485Mux : public Component {
 public:
  void set_uart(uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_ports(MuxPort *modbus, MuxPort *stream);
  void set_idle_timeout(uint32_t ms) { this->idle_.set_timeout(ms); }

  void setup() override;
  void loop() override;
  void dump_config() override;
  // After the UART (BUS) and before modbus (BUS - 1), whose setup() divides by the port's baud rate.
  float get_setup_priority() const override { return setup_priority::BUS - 0.5f; }

  void set_passthrough(bool on);
  bool passthrough() const { return this->passthrough_; }
  void set_client_connected(bool connected) { this->client_ = connected; }
  // True once passthrough has been on for the idle timeout with no TCP client; the YAML then turns it off.
  bool idle_expired() { return this->idle_.update(this->passthrough_, this->client_, millis()); }

  bool is_active(const MuxPort *port) const { return port == (this->passthrough_ ? this->stream_ : this->modbus_); }
  uart::UARTComponent *uart() { return this->uart_; }

 protected:
  void sync_settings_(MuxPort *port);

  uart::UARTComponent *uart_{nullptr};
  MuxPort *modbus_{nullptr};
  MuxPort *stream_{nullptr};
  rs485_mux_logic::IdleTimer idle_{600000};
  bool passthrough_{false};
  bool client_{false};
};

}  // namespace rs485_mux
}  // namespace esphome
