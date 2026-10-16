#include "check.h"
#include "rs485_mux/mux_logic.h"

using rs485_mux_logic::IdleTimer;

static void test_off_never_fires() {
  IdleTimer t(1000);
  CHECK(!t.update(false, false, 0));
  CHECK(!t.update(false, false, 5000));
}

static void test_on_without_client_fires_after_timeout() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 100));  // switched on at 100
  CHECK(!t.update(true, false, 1099));
  CHECK(t.update(true, false, 1100));
}

static void test_client_holds_it_and_disconnect_restarts_the_count() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(true, true, 500));    // client connects
  CHECK(!t.update(true, true, 50000));  // never times out while connected
  CHECK(!t.update(true, false, 50001)); // disconnect: count from here
  CHECK(!t.update(true, false, 51000));
  CHECK(t.update(true, false, 51001));
}

static void test_reconnect_inside_the_window_restarts() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(true, true, 900));
  CHECK(!t.update(true, false, 950));
  CHECK(!t.update(true, false, 1900));
  CHECK(t.update(true, false, 1950));
}

static void test_turning_off_resets() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(false, false, 900));  // turned off by hand
  CHECK(!t.update(true, false, 950));   // on again: a fresh 1000 ms
  CHECK(!t.update(true, false, 1900));
  CHECK(t.update(true, false, 1950));
}

static void test_millis_wraparound() {
  IdleTimer t(1000);
  CHECK(!t.update(true, false, 0xFFFFFF00u));
  CHECK(!t.update(true, false, 0x000002E7u));  // 999 ms later
  CHECK(t.update(true, false, 0x000002E8u));   // 1000 ms later
}

static void test_set_timeout() {
  IdleTimer t(1000);
  t.set_timeout(600000);
  CHECK(!t.update(true, false, 0));
  CHECK(!t.update(true, false, 599999));
  CHECK(t.update(true, false, 600000));
}

int main() {
  test_off_never_fires();
  test_on_without_client_fires_after_timeout();
  test_client_holds_it_and_disconnect_restarts_the_count();
  test_reconnect_inside_the_window_restarts();
  test_turning_off_resets();
  test_millis_wraparound();
  test_set_timeout();
  return test_result("test_rs485_mux");
}
