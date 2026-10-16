#include <initializer_list>

#include "app/ui.h"
#include "check.h"

using namespace oem;
using charge::State;

struct Rig {
  Ui ui;
  Settings live = default_settings();
  uint32_t now = 10000;
  UiActions last;
  int clicks_now = 0, pauses = 0, saves = 0;

  // Hold `key` (or none, -1) for ms, ticking every 10 ms.
  void hold(int key, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10, now += 10) {
      bool raw[4] = {false, false, false, false};
      if (key >= 0) raw[key] = true;
      last = ui.on_keys(raw, live, now);
      clicks_now += last.charge_now;
      pauses += last.toggle_pause;
      saves += last.settings_changed;
    }
  }
  void hold_up_down(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 10, now += 10) {
      const bool raw[4] = {false, true, true, false};
      last = ui.on_keys(raw, live, now);
    }
  }
  void click(int key) {
    hold(key, 100);
    hold(-1, 50);
  }
};

static bool seg(const Frame &f, int s, int com) { return f.com[com] & (1u << s); }

static bool same_field(const Frame &a, const Frame &b, Field field) {
  for (int p = 0; p < FIELDS[int(field)].digits; p++) {
    const int s = FIELDS[int(field)].segs[p];
    for (int k = 1; k < 8; k++) {
      if (seg(a, s, k) != seg(b, s, k)) return false;
    }
  }
  return true;
}

static Status charging_status() {
  Status st;
  st.state = State::CHARGING;
  st.online = 2;
  st.ac_v = 231.4f;
  st.output_a = 10.4f;
  st.power_w = 560.2f;
  st.battery_v = 52.34f;
  return st;
}

static void test_main_screen_fields() {
  Ui ui;
  const Frame f = ui.render(charging_status(), 2000);
  Frame want;
  put_number(want, Field::AC_V, 231);
  put_number(want, Field::CURRENT, 10);
  put_number(want, Field::POWER, 560);
  put_number(want, Field::BATTERY_V, 523, 2);
  put_dashes(want, Field::SOC);  // no SOC in Panel mode
  for (Field fl : {Field::AC_V, Field::CURRENT, Field::POWER, Field::BATTERY_V, Field::SOC})
    CHECK(same_field(f, want, fl));
  CHECK(seg(f, 28, 5) && !seg(f, 28, 6) && !seg(f, 28, 7));  // charging icon, no idle or error icon
  CHECK(seg(f, 28, 0) && seg(f, 28, 1));                     // fixed labels
  Status idle;
  idle.battery_v = 0.5f;
  const Frame g = ui.render(idle, 2000);
  Frame want_g;
  put_dashes(want_g, Field::AC_V);  // no module online: no AC reading
  put_number(want_g, Field::BATTERY_V, 5, 2);
  CHECK(same_field(g, want_g, Field::AC_V) && same_field(g, want_g, Field::BATTERY_V));
  CHECK(seg(g, 28, 6) && !seg(g, 28, 5));
}

static void test_error_alternates_with_soc() {
  Ui ui;
  Status st = charging_status();
  st.state = State::FAULT;
  st.error = 13;
  const Frame even = ui.render(st, 4000), odd = ui.render(st, 5000);
  Frame dashes, err;
  put_dashes(dashes, Field::SOC);
  put_error(err, 13);
  CHECK(same_field(even, dashes, Field::SOC) && same_field(odd, err, Field::SOC));
  CHECK(seg(even, 28, 7) && seg(odd, 28, 7));
}

static void test_bars_sweep_while_charging() {
  Ui ui;
  auto bars = [](const Frame &f) {
    int n = 0;
    for (int k = 0; k < 8; k++) n += seg(f, 16, k);
    return n;
  };
  int max_bars = 0, min_bars = 8;
  for (uint32_t t = 0; t < 6000; t += 200) {  // one step per 300 ms: 0..11, then 0 again
    const int n = bars(ui.render(charging_status(), 10000 + t));
    if (n > max_bars) max_bars = n;
    if (n < min_bars) min_bars = n;
  }
  CHECK(max_bars == 8 && min_bars == 0);
  Status idle;
  CHECK(bars(ui.render(idle, 20000)) == 0);
}

static void test_switch_click_and_pause_hold() {
  Rig g;
  g.click(KEY_SWITCH);
  CHECK(g.clicks_now == 1 && g.pauses == 0);
  g.hold(KEY_SWITCH, 2500);
  CHECK(g.ui.held_ticks(KEY_SWITCH) >= 240);
  g.hold(-1, 50);
  CHECK(g.ui.held_ticks(KEY_SWITCH) == 0);
  CHECK(g.pauses == 1 && g.clicks_now == 1);  // a hold is not a click
}

static void test_menu_edit_and_save() {
  Rig g;
  g.click(KEY_ENTER);
  CHECK(g.ui.in_menu() && g.ui.menu_item() == 0);
  for (int i = 0; i < 3; i++) g.click(KEY_UP);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 13 && g.live.get(HR_MAX_CURRENT) == 10);  // live only on exit
  g.click(KEY_ENTER);  // item 2: Charge V
  g.click(KEY_DOWN);
  CHECK(g.ui.editing().get(HR_CHARGE_V) == 5750);
  CHECK(g.ui.editing().get(HR_STOP_V) == 5750);  // Stop V follows it down
  for (int i = 0; i < 6; i++) g.click(KEY_ENTER);
  CHECK(g.ui.in_menu() && g.ui.menu_item() == 7 && g.saves == 0);
  g.click(KEY_ENTER);  // past item 8: exit and save
  CHECK(!g.ui.in_menu() && g.saves == 1);
  CHECK(g.live.get(HR_MAX_CURRENT) == 13 && g.live.get(HR_CHARGE_V) == 5750 && g.live.get(HR_STOP_V) == 5750);
  g.click(KEY_ENTER);
  g.click(KEY_ENTER);
  g.hold(KEY_ENTER, 1100);  // long Enter: exit; nothing changed, so nothing to save
  g.hold(-1, 50);
  CHECK(!g.ui.in_menu() && g.saves == 1);
}

static void test_menu_ranges_and_repeat() {
  Rig g;
  g.click(KEY_ENTER);
  g.hold(KEY_UP, 30000);  // auto-repeat up to the top of the range
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 100);
  for (int i = 0; i < 5; i++) g.click(KEY_ENTER);  // item 6: Stop V, at most Charge V
  CHECK(g.ui.menu_item() == 5);
  g.click(KEY_UP);
  CHECK(g.ui.editing().get(HR_STOP_V) == 5760);
  g.click(KEY_ENTER);  // item 7: source mode 0-5
  g.click(KEY_DOWN);
  CHECK(g.ui.editing().get(HR_SOURCE_MODE) == 0);
  for (int i = 0; i < 7; i++) g.click(KEY_UP);  // 0-5: stops at JK-PB
  CHECK(g.ui.editing().get(HR_SOURCE_MODE) == 5);
}

static void test_menu_idle_exit() {
  Rig g;
  g.click(KEY_ENTER);
  g.click(KEY_UP);
  g.hold(-1, 29000);
  CHECK(g.ui.in_menu());
  g.hold(-1, 1500);
  CHECK(!g.ui.in_menu() && g.saves == 1 && g.live.get(HR_MAX_CURRENT) == 11);
}

static void test_menu_render() {
  Rig g;
  g.click(KEY_ENTER);
  const Status st;
  const Frame off = g.ui.render(st, 200), on = g.ui.render(st, 600);
  Frame want;
  put_number(want, Field::CURRENT, 10);
  CHECK(same_field(on, want, Field::CURRENT) && seg(on, 28, 4));  // value and the item-1 indicator
  CHECK(!seg(off, 28, 4) && !seg(off, 22, 2));                   // both blank in the "off" half
  for (int i = 0; i < 6; i++) g.click(KEY_ENTER);                 // item 7: number in the AC field
  Frame seven;
  put_number(seven, Field::AC_V, 7);
  CHECK(same_field(g.ui.render(st, 200), seven, Field::AC_V));
}

static void test_wake_press_does_nothing_else() {
  Rig g;
  CHECK(g.ui.backlight(g.live, g.now));  // on after power-up
  g.hold(-1, 121000);
  CHECK(!g.ui.backlight(g.live, g.now));
  g.click(KEY_SWITCH);
  CHECK(g.clicks_now == 0 && g.ui.backlight(g.live, g.now));
  g.click(KEY_SWITCH);
  CHECK(g.clicks_now == 1);
  g.live.set(HR_BACKLIGHT_S, 0);
  g.hold(-1, 500000);
  CHECK(g.ui.backlight(g.live, g.now));
}

static void test_leds() {
  Status st = charging_status();
  CHECK(leds(st, 0).charge && leds(st, 100).charge && !leds(st, 0).fault);  // solid while delivering
  st.output_a = 0.0f;
  CHECK(leds(st, 0).charge != leds(st, 100).charge);                        // 100 ms blink without current
  st.state = State::HOLD;
  CHECK(leds(st, 0).charge != leds(st, 500).charge && leds(st, 0).charge == leds(st, 100).charge);
  st.error = 4;
  CHECK(leds(st, 0).fault && leds(st, 500).fault && !leds(st, 0).charge && !leds(st, 500).charge);
  st.error = 16;
  CHECK(leds(st, 0).fault != leds(st, 500).fault);  // warnings blink
  st.error = 0;
  st.online = 0;
  CHECK(!leds(st, 0).charge && !leds(st, 500).charge);
}

static void test_no_module_online_not_shown_charging() {
  Ui ui;
  Status st = charging_status();
  st.online = 0;  // CHARGING with nothing to deliver it (both modules lost AC)
  st.output_a = st.power_w = 0.0f;
  auto bars = [](const Frame &f) {
    int n = 0;
    for (int k = 0; k < 8; k++) n += seg(f, 16, k);
    return n;
  };
  for (uint32_t t = 0; t < 3000; t += 200) {
    const Frame f = ui.render(st, 10000 + t);
    CHECK(seg(f, 28, 6) && !seg(f, 28, 5));  // idle icon, not the charging one
    CHECK(bars(f) == 0);                      // no sweep
  }
}

static void test_fault_led_after_watchdog_reset() {
  const Leds off{false, false}, on{true, false};
  bool changed = false, prev = fault_led(off, true, true, 0);
  for (uint32_t t = 0; t < 10000; t += 50) {  // 100 ms blink for 10 s, whatever else
    const bool l = fault_led(on, true, true, t);
    changed = changed || l != prev;
    prev = l;
    CHECK(fault_led(off, true, true, t) == ((t / 100) % 2 == 1));
  }
  CHECK(changed);
  CHECK(!fault_led(off, true, true, 10000) && fault_led(on, true, true, 10000) && fault_led(off, false, true, 10000));
  CHECK(!fault_led(off, true, false, 100) && fault_led(on, true, false, 0) && fault_led(off, false, false, 0));
}

static void test_updown_long_event_steps() {
  Rig g;
  g.click(KEY_ENTER);
  g.hold(KEY_UP, 1050);  // released between the long press (1.00 s) and the first repeat (1.15 s)
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 11);
  g.hold(KEY_UP, 1300);  // long step, then the first repeat at +150 ms
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 13);
}

static void test_switch_ignored_in_menu() {
  Rig g;
  g.click(KEY_ENTER);
  g.click(KEY_SWITCH);
  g.hold(KEY_SWITCH, 2500);
  g.hold(-1, 50);
  CHECK(g.clicks_now == 0 && g.pauses == 0 && g.ui.in_menu());
}

static void test_menu_gap_and_soc_clamps() {
  Rig g;
  g.click(KEY_ENTER);
  for (int i = 0; i < 2; i++) g.click(KEY_ENTER);  // item 3: Start SOC
  const int32_t stop_soc = g.ui.editing().get(HR_STOP_SOC);
  g.hold(KEY_UP, 30000);
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_START_SOC) == stop_soc - 1);
  for (int i = 0; i < 2; i++) g.click(KEY_ENTER);  // item 5: Start V
  g.hold(KEY_UP, 30000);
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_START_V) == g.ui.editing().get(HR_STOP_V) - START_STOP_GAP_CV);
}

static void test_menu_item2_default_glass() {
  Rig g;
  g.click(KEY_ENTER);
  g.click(KEY_ENTER);
  const Status st;
  Frame want;
  put_number(want, Field::BATTERY_V, 576, 2);
  const Frame on = g.ui.render(st, 600);
  CHECK(same_field(on, want, Field::BATTERY_V) && seg(on, 28, 3));
}

static void test_wake_swallows_updown() {
  Rig g;
  g.live.set(HR_BACKLIGHT_S, 20);  // the backlight goes off while the 30 s menu timeout is still running
  g.click(KEY_ENTER);
  g.hold(-1, 25000);
  CHECK(g.ui.in_menu());
  CHECK(!g.ui.backlight(g.live, g.now));
  g.hold(KEY_UP, 1300);
  g.hold(-1, 50);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 10);
  g.click(KEY_UP);
  CHECK(g.ui.editing().get(HR_MAX_CURRENT) == 11);
}

static void test_restart_countdown_in_soc_field() {
  Rig g;
  CHECK(g.ui.restart_countdown() == -1);
  g.hold_up_down(100);
  CHECK(g.ui.restart_countdown() == 5 && !g.ui.restart_held());
  Status st = charging_status();
  st.soc = 64;
  st.error = 15;  // the countdown also replaces the error's turn in the field
  Frame five;
  put_number(five, Field::SOC, 5);
  CHECK(same_field(g.ui.render(st, 5000), five, Field::SOC) && same_field(g.ui.render(st, 6000), five, Field::SOC));
  g.hold_up_down(1500);
  CHECK(g.ui.restart_countdown() == 4);
  g.hold_up_down(2500);
  CHECK(g.ui.restart_countdown() == 1 && !g.ui.restart_held());
  g.hold_up_down(1000);
  CHECK(g.ui.restart_countdown() == 0 && g.ui.restart_held());
  Frame zero;
  put_number(zero, Field::SOC, 0);
  CHECK(same_field(g.ui.render(st, 5000), zero, Field::SOC));
  g.hold(-1, 100);  // released: back to the SOC
  CHECK(g.ui.restart_countdown() == -1);
  Frame soc;
  put_number(soc, Field::SOC, 64);
  CHECK(same_field(g.ui.render(st, 4000), soc, Field::SOC));
  // In the menu, Up + Down edit the value and never restart the board.
  g.click(KEY_ENTER);
  g.hold_up_down(6000);
  CHECK(g.ui.in_menu() && g.ui.restart_countdown() == -1 && !g.ui.restart_held());
}

int main() {
  test_main_screen_fields();
  test_error_alternates_with_soc();
  test_bars_sweep_while_charging();
  test_switch_click_and_pause_hold();
  test_menu_edit_and_save();
  test_menu_ranges_and_repeat();
  test_menu_idle_exit();
  test_menu_render();
  test_wake_press_does_nothing_else();
  test_leds();
  test_updown_long_event_steps();
  test_no_module_online_not_shown_charging();
  test_fault_led_after_watchdog_reset();
  test_switch_ignored_in_menu();
  test_menu_gap_and_soc_clamps();
  test_menu_item2_default_glass();
  test_wake_swallows_updown();
  test_restart_countdown_in_soc_field();
  return test_result("test_oem_ui");
}
