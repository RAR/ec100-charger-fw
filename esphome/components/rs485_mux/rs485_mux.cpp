#include "rs485_mux.h"

#include <algorithm>

#include "esphome/core/log.h"

namespace esphome {
namespace rs485_mux {

static const char *const TAG = "rs485_mux";

void MuxPort::write_array(const uint8_t *data, size_t len) {
  if (this->mux_ != nullptr && this->mux_->is_active(this)) this->mux_->uart()->write_array(data, len);
}

bool MuxPort::peek_byte(uint8_t *data) {
  if (this->rx_.empty()) return false;
  *data = this->rx_.front();
  return true;
}

bool MuxPort::read_array(uint8_t *data, size_t len) {
  if (this->rx_.size() < len) return false;
  for (size_t i = 0; i < len; i++) {
    data[i] = this->rx_.front();
    this->rx_.pop_front();
  }
  return true;
}

uart::UARTFlushResult MuxPort::flush() {
  if (this->mux_ != nullptr && this->mux_->is_active(this)) return this->mux_->uart()->flush();
  return uart::UARTFlushResult::UART_FLUSH_RESULT_ASSUMED_SUCCESS;
}

void MuxPort::push(const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (this->rx_.size() >= MAX_RX) this->rx_.pop_front();  // keep the newest bytes
    this->rx_.push_back(data[i]);
  }
}

void RS485Mux::set_ports(MuxPort *modbus, MuxPort *stream) {
  this->modbus_ = modbus;
  this->stream_ = stream;
  modbus->set_mux(this);
  stream->set_mux(this);
  // Also here, not only in setup(): the ports must never report baud 0 to a user that sets up first.
  if (this->uart_ != nullptr) {
    this->sync_settings_(modbus);
    this->sync_settings_(stream);
  }
}

void RS485Mux::sync_settings_(MuxPort *port) {
  port->set_baud_rate(this->uart_->get_baud_rate());
  port->set_data_bits(this->uart_->get_data_bits());
  port->set_stop_bits(this->uart_->get_stop_bits());
  port->set_parity(this->uart_->get_parity());
  port->set_rx_buffer_size(this->uart_->get_rx_buffer_size());
}

void RS485Mux::setup() {
  this->sync_settings_(this->modbus_);
  this->sync_settings_(this->stream_);
}

void RS485Mux::loop() {
  // The baud select changes the real UART at run time; modbus reads get_baud_rate() for its frame timing.
  const uint32_t baud = this->uart_->get_baud_rate();
  if (this->modbus_->get_baud_rate() != baud) {
    this->sync_settings_(this->modbus_);
    this->sync_settings_(this->stream_);
  }
  MuxPort *active = this->passthrough_ ? this->stream_ : this->modbus_;
  uint8_t buf[64];
  size_t n;
  while ((n = std::min(this->uart_->available(), sizeof(buf))) > 0) {
    if (!this->uart_->read_array(buf, n)) break;
    active->push(buf, n);
  }
}

void RS485Mux::set_passthrough(bool on) {
  if (on == this->passthrough_) return;
  this->passthrough_ = on;
  // A half-received frame must not cross over.
  this->modbus_->clear();
  this->stream_->clear();
  ESP_LOGI(TAG, "RS485 line -> %s", on ? "TCP passthrough" : "Modbus polling");
}

void RS485Mux::dump_config() {
  ESP_LOGCONFIG(TAG, "RS485 mux: line to %s", this->passthrough_ ? "TCP passthrough" : "Modbus polling");
}

}  // namespace rs485_mux
}  // namespace esphome
