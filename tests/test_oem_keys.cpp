#include "app/keys.h"
#include "check.h"

using oem::Key;
using oem::KeyEvent;
using oem::KEY_LONG_TICKS;
using oem::KEY_REPEAT_TICKS;

static int count(Key &k, bool raw, int ticks, KeyEvent want) {
  int n = 0;
  for (int i = 0; i < ticks; i++) n += k.update(raw) == want;
  return n;
}

static void test_glitch_ignored() {
  Key k;
  CHECK(k.update(true) == KeyEvent::NONE);  // one high sample, then low again
  CHECK(k.update(false) == KeyEvent::NONE);
  CHECK(count(k, false, 10, KeyEvent::CLICK) == 0);
  CHECK(!k.pressed());
}

static void test_click_on_release() {
  Key k;
  CHECK(count(k, true, 5, KeyEvent::CLICK) == 0);
  CHECK(k.pressed());
  CHECK(count(k, false, 5, KeyEvent::CLICK) == 1);
  CHECK(!k.pressed());
}

static void test_long_then_repeat_no_click() {
  Key k;
  int longs = 0, repeats = 0;
  for (int i = 0; i < 2 + 100 + 45; i++) {
    const KeyEvent e = k.update(true);
    longs += e == KeyEvent::LONG;
    repeats += e == KeyEvent::REPEAT;
  }
  CHECK(longs == 1);
  CHECK(repeats == 3);  // 15, 30 and 45 ticks after the long press
  CHECK(k.held_ticks() == 145);
  CHECK(count(k, false, 5, KeyEvent::CLICK) == 0);
}

// Samples are numbered from 0 for the first update(true). Two samples debounce the press (sample 1 makes it stable),
// so held == sample - 1 and the LONG fires on sample 101, the sample where held reaches KEY_LONG_TICKS.
static void test_long_fires_on_sample_held_reaches_100() {
  Key k;
  int long_at = -1;
  for (int i = 0; i < 110; i++) {
    if (k.update(true) == KeyEvent::LONG) {
      CHECK(long_at < 0);
      long_at = i;
      CHECK(k.held_ticks() == KEY_LONG_TICKS);
    }
  }
  CHECK(long_at == 101);
}

// REPEAT fires on exactly the samples 15, 30 and 45 ticks after the LONG (samples 116, 131, 146), each with held
// equal to KEY_LONG_TICKS plus a multiple of KEY_REPEAT_TICKS.
static void test_repeat_at_15_30_45_after_long() {
  Key k;
  int long_at = -1;
  int rep_at[4] = {-1, -1, -1, -1};
  int nrep = 0;
  for (int i = 0; i < 147; i++) {
    const KeyEvent e = k.update(true);
    if (e == KeyEvent::LONG) long_at = i;
    if (e == KeyEvent::REPEAT) {
      if (nrep < 4) rep_at[nrep] = i;
      nrep++;
      CHECK(k.held_ticks() == KEY_LONG_TICKS + KEY_REPEAT_TICKS * nrep);
    }
  }
  CHECK(long_at == 101);
  CHECK(nrep == 3);
  CHECK(rep_at[0] == long_at + 15);
  CHECK(rep_at[1] == long_at + 30);
  CHECK(rep_at[2] == long_at + 45);
}

// Regression: the first low sample of a release must not advance the hold counter. Releasing at held == 99 must give
// exactly one CLICK, and no LONG or REPEAT.
static void test_release_at_99_is_click_only() {
  Key k;
  int longs = 0, repeats = 0, clicks = 0;
  auto tally = [&](KeyEvent e) {
    longs += e == KeyEvent::LONG;
    repeats += e == KeyEvent::REPEAT;
    clicks += e == KeyEvent::CLICK;
  };
  while (k.held_ticks() < 99) tally(k.update(true));
  CHECK(k.held_ticks() == 99);
  for (int i = 0; i < 5; i++) tally(k.update(false));
  CHECK(clicks == 1);
  CHECK(longs == 0);
  CHECK(repeats == 0);
  CHECK(!k.pressed());
}

// A single low sample in the middle of a hold is a glitch: the key stays pressed and no CLICK is produced.
static void test_mid_hold_glitch_keeps_pressed_no_click() {
  Key k;
  int clicks = 0, longs = 0;
  for (int i = 0; i < 20; i++) {
    const KeyEvent e = k.update(true);
    clicks += e == KeyEvent::CLICK;
    longs += e == KeyEvent::LONG;
  }
  const KeyEvent g = k.update(false);
  CHECK(g != KeyEvent::CLICK);
  CHECK(k.pressed());
  for (int i = 0; i < 10; i++) {
    const KeyEvent e = k.update(true);
    clicks += e == KeyEvent::CLICK;
    longs += e == KeyEvent::LONG;
  }
  CHECK(clicks == 0);
  CHECK(longs == 0);
  CHECK(k.pressed());
}

int main() {
  test_glitch_ignored();
  test_click_on_release();
  test_long_then_repeat_no_click();
  test_long_fires_on_sample_held_reaches_100();
  test_repeat_at_15_30_45_after_long();
  test_release_at_99_is_click_only();
  test_mid_hold_glitch_keeps_pressed_no_click();
  return test_result("test_oem_keys");
}
