// oem/src/app/relay.h: the generator contact (spec §2.4). Auto: closed while a session is wanted. It opens 10 s after
// the session ends (a session returning within that time keeps it closed), but at once on pause or a fault. The caller
// never reports missing AC as a fault: with no AC the modules go offline, and the generator is what brings AC back.
#pragma once
#include <cstdint>

namespace oem {

enum class RelayMode : uint8_t { AUTO = 0, OFF = 1, MODBUS = 2 };

constexpr uint32_t RELAY_OFF_DELAY_MS = 10000;

class RelayLogic {
 public:
  // modbus_cmd is HR 112.
  bool update(RelayMode mode, bool session_active, bool paused, bool faulted, bool modbus_cmd, uint32_t now_ms) {
    // Back in Auto, a contact Modbus left closed gets the full end delay rather than one timed from a stale session.
    if (mode == RelayMode::AUTO && mode_ != RelayMode::AUTO) last_wanted_ms_ = now_ms;
    mode_ = mode;
    if (mode == RelayMode::OFF) {
      closed_ = false;
    } else if (mode == RelayMode::MODBUS) {
      closed_ = modbus_cmd;
    } else if (paused || faulted) {
      closed_ = false;
    } else if (session_active) {
      closed_ = true;
      last_wanted_ms_ = now_ms;
    } else if (closed_ && now_ms - last_wanted_ms_ >= RELAY_OFF_DELAY_MS) {
      closed_ = false;
    }
    return closed_;
  }

  bool closed() const { return closed_; }

 private:
  RelayMode mode_ = RelayMode::AUTO;
  bool closed_ = false;
  uint32_t last_wanted_ms_ = 0;
};

}  // namespace oem
