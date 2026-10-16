// oem/src/app/ui.h: keys, the main screen, the 8-item menu, LEDs and backlight (spec §3). The layout, menu
// indicators, blink and bar animation follow the stock render (docs/history/notes/oem-init-sequences.md §4c).
#pragma once
#include <cmath>
#include <cstdint>

#include "app/controller.h"
#include "app/keys.h"
#include "app/lcd_render.h"
#include "app/settings.h"

namespace oem {

enum KeyId : uint8_t { KEY_SWITCH = 0, KEY_UP = 1, KEY_DOWN = 2, KEY_ENTER = 3 };

constexpr uint16_t PAUSE_HOLD_TICKS = 200;  // Switch held 2 s
constexpr uint16_t RESTART_HOLD_TICKS = 500;  // Up + Down held 5 s outside the menu: standby, then restart
constexpr uint32_t MENU_IDLE_MS = 30000;
constexpr uint32_t BLINK_MS = 400;          // edited value: 400 ms off, 400 ms on (a multiple of the 200 ms render)
constexpr uint32_t BAR_STEP_MS = 300;
constexpr int MENU_ITEMS = 8;

struct UiActions {
  bool charge_now = false;
  bool toggle_pause = false;
  bool settings_changed = false;  // the menu closed with new values (already applied to the live settings)
};

struct MenuItem {
  Hr reg;
  Field field;
  int32_t step;
  int32_t div;     // shown value = setting / div
  int8_t ind_seg;  // menu indicator, -1 for none
  int8_t ind_com;
};

// Items 7 and 8 have no indicator on the stock glass; the AC field shows their number instead.
constexpr MenuItem MENU[MENU_ITEMS] = {
    {HR_MAX_CURRENT, Field::CURRENT, 1, 1, 28, 4},  {HR_CHARGE_V, Field::BATTERY_V, 10, 10, 28, 3},
    {HR_START_SOC, Field::SOC, 1, 1, 25, 7},        {HR_STOP_SOC, Field::SOC, 1, 1, 25, 6},
    {HR_START_V, Field::BATTERY_V, 10, 10, 25, 5},  {HR_STOP_V, Field::BATTERY_V, 10, 10, 25, 4},
    {HR_SOURCE_MODE, Field::SOC, 1, 1, -1, 0},      {HR_MODBUS_ADDR, Field::SOC, 1, 1, -1, 0},
};

// The editable range of menu item i, given the other settings (spec §3).
inline void menu_range(const Settings &s, int i, int32_t &lo, int32_t &hi) {
  const Hr r = MENU[i].reg;
  lo = HR_RANGES[r].min;
  hi = HR_RANGES[r].max;
  if (r == HR_START_SOC) hi = s.get(HR_STOP_SOC) - 1;
  if (r == HR_STOP_SOC) lo = s.get(HR_START_SOC) + 1;
  if (r == HR_START_V) hi = s.get(HR_STOP_V) - START_STOP_GAP_CV;
  if (r == HR_STOP_V) hi = s.get(HR_CHARGE_V);
}

struct Leds {
  bool fault;
  bool charge;
};

// Fault LED: on for E1-E14, blinking (500 ms) for the E15/E16 warnings. Charge LED as stock 0800b444: with a module
// online and no error, solid while charging with current, 100 ms blink while charging without, 500 ms blink idle.
inline Leds leds(const Status &st, uint32_t now) {
  const bool slow = (now / 500) % 2, fast = (now / 100) % 2;
  Leds l{false, false};
  if (is_fault_code(st.error)) l.fault = true;
  else if (st.error != 0) l.fault = slow;
  if (st.online > 0 && st.error == 0) {
    if (st.state == charge::State::CHARGING) l.charge = st.output_a >= 0.5f ? true : fast;
    else l.charge = slow;
  }
  return l;
}

constexpr uint32_t WATCHDOG_BLINK_MS = 10000;

// The Fault LED (true = lit): a 100 ms blink for the first 10 s after a watchdog reset, so the bench can tell one from
// a power-cycle; after that solid while CAN is down, else as leds() says.
inline bool fault_led(const Leds &l, bool can, bool watchdog_reset, uint32_t since_boot_ms) {
  if (watchdog_reset && since_boot_ms < WATCHDOG_BLINK_MS) return (since_boot_ms / 100) % 2;
  return l.fault || !can;
}

class Ui {
 public:
  // Every 10 ms with the raw key levels (Switch, Up, Down, Enter). Menu edits go to a copy, which replaces `live`
  // when the menu closes.
  UiActions on_keys(const bool raw[4], Settings &live, uint32_t now) {
    UiActions a;
    KeyEvent ev[4];
    const bool lit = backlight(live, now);
    for (int i = 0; i < 4; i++) {
      ev[i] = keys_[i].update(raw[i]);
      if (keys_[i].pressed()) {
        if (!was_pressed_[i] && !lit) swallow_[i] = true;  // a wake press does nothing else
        last_key_ms_ = now;
        any_key_ = true;
      }
      was_pressed_[i] = keys_[i].pressed();
      if (swallow_[i]) {
        if (!keys_[i].pressed()) swallow_[i] = false;
        ev[i] = KeyEvent::NONE;
        continue;
      }
    }
    if (item_ < 0) {  // the menu owns the keys while it is open
      if (ev[KEY_SWITCH] == KeyEvent::CLICK) a.charge_now = true;
      if (!swallow_[KEY_SWITCH] && keys_[KEY_SWITCH].held_ticks() == PAUSE_HOLD_TICKS) a.toggle_pause = true;
    }

    if (item_ < 0) {
      if (ev[KEY_ENTER] == KeyEvent::CLICK) {
        item_ = 0;
        edit_ = live;
      }
      return a;
    }
    // LONG steps too: a release between the long press (1.00 s) and the first repeat (1.15 s) would otherwise do
    // nothing.
    auto step = [](KeyEvent e) { return e == KeyEvent::CLICK || e == KeyEvent::LONG || e == KeyEvent::REPEAT; };
    const bool up = step(ev[KEY_UP]), down = step(ev[KEY_DOWN]);
    if (up || down) {
      int32_t lo, hi;
      menu_range(edit_, item_, lo, hi);
      const MenuItem &m = MENU[item_];
      edit_.set(m.reg, clamp_i32(edit_.get(m.reg) + (up ? m.step : -m.step), lo, hi));
      apply_clamps(edit_);
    }
    if (ev[KEY_ENTER] == KeyEvent::CLICK && ++item_ == MENU_ITEMS) close_(live, a);
    else if (ev[KEY_ENTER] == KeyEvent::LONG || now - last_key_ms_ >= MENU_IDLE_MS) close_(live, a);
    return a;
  }

  uint16_t held_ticks(KeyId k) const { return keys_[k].held_ticks(); }

  // Up + Down held together outside the menu: the seconds left until the restart (5 ... 1, then 0), else -1.
  int restart_countdown() const {
    if (item_ >= 0 || swallow_[KEY_UP] || swallow_[KEY_DOWN]) return -1;
    const uint16_t up = keys_[KEY_UP].held_ticks(), down = keys_[KEY_DOWN].held_ticks();
    if (up == 0 || down == 0) return -1;
    const uint16_t held = up < down ? up : down;
    return held >= RESTART_HOLD_TICKS ? 0 : (RESTART_HOLD_TICKS - held + 99) / 100;
  }
  bool restart_held() const { return restart_countdown() == 0; }
  bool in_menu() const { return item_ >= 0; }
  int menu_item() const { return item_; }  // 0-based, -1 outside the menu
  const Settings &editing() const { return edit_; }

  // On until Backlight timeout (HR 11, 0 = always) after the last key press; on at power-up.
  bool backlight(const Settings &s, uint32_t now) const {
    const uint32_t timeout = uint32_t(s.get(HR_BACKLIGHT_S)) * 1000u;
    return timeout == 0 || now - (any_key_ ? last_key_ms_ : 0u) < timeout;
  }

  // Every 200 ms.
  Frame render(const Status &st, uint32_t now) {
    Frame f;
    put_fixed_labels(f);
    if (item_ >= 0) {
      render_menu_(f, now);
      return f;
    }
    if (std::isfinite(st.ac_v) && st.ac_v >= 1.0f) put_number(f, Field::AC_V, uint32_t(std::lround(st.ac_v)));
    else put_dashes(f, Field::AC_V);
    put_number(f, Field::CURRENT, uint32_t(std::lround(std::fmax(st.output_a, 0.0f))));
    put_number(f, Field::POWER, uint32_t(std::lround(std::fmax(st.power_w, 0.0f))));
    if (std::isfinite(st.battery_v) && st.battery_v >= 0.0f)
      put_number(f, Field::BATTERY_V, uint32_t(std::lround(st.battery_v * 10.0f)), 2);
    else
      put_dashes(f, Field::BATTERY_V);
    const int countdown = restart_countdown();
    if (countdown >= 0) put_number(f, Field::SOC, uint32_t(countdown));  // Up + Down: seconds to the restart
    else if (st.error != 0 && (now / 1000) % 2) put_error(f, st.error);  // 1 s SOC, 1 s error, as stock
    else if (st.soc >= 0) put_number(f, Field::SOC, uint32_t(st.soc));
    else put_dashes(f, Field::SOC);

    // CHARGING with no module online delivers nothing (both lost AC): show idle.
    const bool charging = st.state == charge::State::CHARGING && st.online > 0;
    const int bar_soc = st.soc >= 0 ? st.soc : (charging ? 100 : 0);  // no SOC: a sweep while charging
    if (charging && st.error == 0) {
      if (now - bar_ms_ >= BAR_STEP_MS) {
        bar_ms_ = now;
        bar_step_ = bar_step_ * 10 <= bar_soc ? bar_step_ + 1 : 0;
      }
      put_soc_bars(f, bar_soc, bar_step_);
    } else {
      put_soc_bars(f, bar_soc);
    }
    if (st.error != 0) put_segment(f, 28, 7);
    put_segment(f, 28, charging ? 5 : 6);  // charging / idle icon
    return f;
  }

 private:
  void close_(Settings &live, UiActions &a) {
    apply_clamps(edit_);
    if (edit_ != live) {
      live = edit_;
      a.settings_changed = true;
    }
    item_ = -1;
  }

  void render_menu_(Frame &f, uint32_t now) const {
    const MenuItem &m = MENU[item_];
    if (m.ind_seg < 0) put_number(f, Field::AC_V, uint32_t(item_ + 1));
    if ((now / BLINK_MS) % 2 == 0) return;  // the "off" half of the blink
    put_number(f, m.field, uint32_t(edit_.get(m.reg) / m.div), m.field == Field::BATTERY_V ? 2 : 1);
    if (m.ind_seg >= 0) put_segment(f, m.ind_seg, m.ind_com);
  }

  Key keys_[4];
  bool was_pressed_[4] = {false, false, false, false};
  bool swallow_[4] = {false, false, false, false};
  bool any_key_ = false;
  uint32_t last_key_ms_ = 0;
  int item_ = -1;
  Settings edit_ = default_settings();
  uint32_t bar_ms_ = 0;
  int bar_step_ = 0;
};

}  // namespace oem
