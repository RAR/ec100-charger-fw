// oem/src/app/keys.h: one front-panel key. Debounce over 2 samples (20 ms), click on release, long press at 1 s,
// then repeat every 150 ms (the stock FlexibleButton timing, docs/history/notes/oem-hw-map.md §4).
#pragma once
#include <cstdint>

namespace oem {

enum class KeyEvent : uint8_t { NONE, CLICK, LONG, REPEAT };

constexpr uint16_t KEY_LONG_TICKS = 100;
constexpr uint16_t KEY_REPEAT_TICKS = 15;

class Key {
 public:
  // Call every 10 ms with the raw, active-high pin level.
  KeyEvent update(bool raw) {
    if (raw != last_raw_) {  // a raw edge starts debouncing; it does not advance the hold counter
      last_raw_ = raw;
      return KeyEvent::NONE;
    }
    if (raw != stable_) {
      stable_ = raw;
      if (stable_) {
        held_ = 0;
        long_sent_ = false;
        return KeyEvent::NONE;
      }
      return long_sent_ ? KeyEvent::NONE : KeyEvent::CLICK;
    }
    return step();
  }

  bool pressed() const { return stable_; }
  uint16_t held_ticks() const { return stable_ ? held_ : 0; }

 private:
  KeyEvent step() {
    if (!stable_) return KeyEvent::NONE;
    if (held_ < 0xFFFF) held_++;
    if (held_ == KEY_LONG_TICKS) {
      long_sent_ = true;
      return KeyEvent::LONG;
    }
    if (long_sent_ && held_ > KEY_LONG_TICKS && (held_ - KEY_LONG_TICKS) % KEY_REPEAT_TICKS == 0)
      return KeyEvent::REPEAT;
    return KeyEvent::NONE;
  }

  bool last_raw_ = false;
  bool stable_ = false;
  bool long_sent_ = false;
  uint16_t held_ = 0;
};

}  // namespace oem
