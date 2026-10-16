// rs485_mux idle timeout: passthrough turns itself off after timeout_ms without a TCP client, counted from the
// switch-on or from the last client leaving. Plain C++ (no ESPHome) so it builds in the host tests.
#pragma once
#include <cstdint>

namespace rs485_mux_logic {

class IdleTimer {
 public:
  explicit IdleTimer(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}
  void set_timeout(uint32_t timeout_ms) { timeout_ms_ = timeout_ms; }

  // Call every loop. True when passthrough should be turned off; it keeps returning true until it is.
  bool update(bool passthrough, bool client, uint32_t now_ms) {
    if (!passthrough || client) {
      armed_ = false;
      return false;
    }
    if (!armed_) {
      armed_ = true;
      since_ms_ = now_ms;
    }
    return now_ms - since_ms_ >= timeout_ms_;
  }

 private:
  uint32_t timeout_ms_;
  uint32_t since_ms_ = 0;
  bool armed_ = false;
};

}  // namespace rs485_mux_logic
