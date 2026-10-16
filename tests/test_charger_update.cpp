// tests/test_charger_update.cpp: the bridge's installer (esphome/components/charger_update/installer.h) against a fake
// charger: our app answering Modbus at 9600, the HR 120 reset (standby second, then the bootloader's 1.5 s window), and
// a byte-level stock bootloader as in tools/test_oem_update.py (acks with the sequence echoed, programs every CRC-good
// frame at a running offset, never checks sequence numbers).
#include <cstdio>
#include <cstring>
#include <deque>
#include <vector>

#include "charger_update/installer.h"
#include "check.h"

using namespace charger_update_logic;

namespace {

struct FakeCharger : LineIo {
  // configuration
  bool app = true;                 // our firmware is running (answers Modbus)
  uint16_t version = 0x030B, state = 3;
  int exception = 0;               // answer the IR read with this Modbus exception
  bool menu_open = false;          // 3.12: HR 120 refused with exception 06
  bool ack_reboot = true;
  int hs_replies = -1;             // answer only this many handshakes (-1 = all)
  int bad_crc_seq = -1;
  uint32_t reset_delay_ms = 1000;  // HR 120 -> standby, then the reset
  bool echo = false;               // a half-duplex adapter hearing itself
  int drop_ack_seq = -1, bad_ack_seq = -1;
  uint32_t power_cycle_at = 0;     // when silent: the bootloader appears then (0 = never)
  uint32_t latency_ms = 5;
  uint32_t boot_latency_ms = 1;    // the bootloader's data-frame acks

  // state
  uint32_t now = 0;
  uint32_t baud = 9600;
  bool boot = false;               // the bootloader is listening
  uint32_t boot_since = 0, reset_at = 0;
  bool reset_pending = false, held = false, erased = false;
  int64_t length = -1;
  size_t offset = 0;
  std::vector<uint8_t> flash = std::vector<uint8_t>(MAX_IMAGE, 0xFF);
  std::vector<uint32_t> bauds;
  int modbus_writes = 0, handshakes = 0;
  struct Rx {
    uint32_t at;
    uint8_t b;
  };
  std::deque<Rx> rx;

  void reply(const uint8_t *d, size_t n, uint32_t delay) {
    for (size_t i = 0; i < n; i++) rx.push_back({now + delay, d[i]});
  }
  void reply(const uint8_t *d, size_t n) { reply(d, n, latency_ms); }
  void reply_crc(std::vector<uint8_t> f, uint32_t delay) {
    const uint16_t crc = crc16_modbus(f.data(), f.size());
    f.push_back(uint8_t(crc));
    f.push_back(uint8_t(crc >> 8));
    reply(f.data(), f.size(), delay);
  }
  void reply_crc(std::vector<uint8_t> f) { reply_crc(std::move(f), latency_ms); }

  void tick(uint32_t t) {
    now = t;
    if (reset_pending && t >= reset_at) {
      reset_pending = false;
      app = false;
      boot = true;
      boot_since = t;
    }
    if (!app && !boot && power_cycle_at != 0 && t >= power_cycle_at) {
      boot = true;
      boot_since = t;
      erased = false;  // a new boot: the next header erases again
    }
    // No handshake within 1.5 s: the bootloader starts the app (only if one is installed and nothing was erased).
    if (boot && !held && length < 0 && t - boot_since > 1500 && power_cycle_at == 0) {
      boot = false;
      app = true;
    }
  }

  void write(const uint8_t *d, size_t n) override {
    if (echo) reply(d, n);
    if (baud == 9600 && app) modbus_(d, n);
    else if (baud == BOOT_BAUD && boot) bootloader_(d, n);
  }
  size_t read(uint8_t *d, size_t max) override {
    size_t k = 0;
    while (k < max && !rx.empty() && rx.front().at <= now) {
      d[k++] = rx.front().b;
      rx.pop_front();
    }
    return k;
  }
  void set_baud(uint32_t b) override {
    baud = b;
    bauds.push_back(b);
  }

  void modbus_(const uint8_t *d, size_t n) {
    if (n != 8 || crc16_modbus(d, 6) != uint16_t(d[6] | d[7] << 8)) return;
    modbus_writes++;
    if (d[1] == 0x04) {
      if (exception) return reply_crc({1, 0x84, uint8_t(exception)});
      reply_crc({1, 4, 4, uint8_t(version >> 8), uint8_t(version), uint8_t(state >> 8), uint8_t(state)});
    } else if (d[1] == 0x06 && d[2] == 0 && d[3] == 120 && d[4] == 0xB0 && d[5] == 0x07) {
      if (menu_open) return reply_crc({1, 0x86, 6});
      if (!ack_reboot) {
        reset_pending = true;  // it resets, but the ack is lost
        reset_at = now + reset_delay_ms;
        return;
      }
      reply(d, n);
      reset_pending = true;
      reset_at = now + reset_delay_ms;
    }
  }

  void bootloader_(const uint8_t *d, size_t n) {
    static const uint8_t HS[5] = {0xF9, 0x45, 0xF9, 0x53, 0x23}, HS_REPLY[6] = {0xF9, 0x45, 0x03, 0x00, 0x21, 0x9D};
    if (n == 5 && std::memcmp(d, HS, 5) == 0) {
      handshakes++;
      if (hs_replies == 0) return;
      if (hs_replies > 0) hs_replies--;
      held = true;
      return reply(HS_REPLY, 6, 1);
    }
    if (n < 9 || d[0] != 0xFB || d[3] != 3 || d[4] != 0) return;
    const size_t len = size_t(d[1] << 8 | d[2]);
    if (n != len + 9 || crc16_modbus(d, len + 7) != uint16_t(d[len + 7] | d[len + 8] << 8)) return;
    const uint16_t seq = uint16_t(d[5] << 8 | d[6]);
    if (length < 0) {
      length = int64_t(d[7]) << 24 | d[8] << 16 | d[9] << 8 | d[10];
      if (!erased) {
        std::fill(flash.begin(), flash.end(), 0xFF);
        erased = true;
      }
    } else {
      std::memcpy(flash.data() + offset, d + 7, len);
      offset += len;
      if (int(seq) == drop_ack_seq) return;
    }
    uint8_t s0 = uint8_t(seq >> 8), s1 = uint8_t(seq);
    if (int(seq) == bad_ack_seq) s1 ^= 0x7F;
    if (int(seq) == bad_crc_seq) {
      const uint8_t ack[7] = {0xFB, 0x03, 0x00, s0, s1, 0x12, 0x34};
      return reply(ack, 7, 1);
    }
    reply_crc({0xFB, 0x03, 0x00, s0, s1}, length >= 0 && offset > 0 ? boot_latency_ms : 1);
    if (length >= 0 && offset >= size_t(length)) {
      boot = false;  // after the 2 s quiet it would start the new app
    }
  }
};

std::vector<uint8_t> image(size_t n) {
  std::vector<uint8_t> v(n);
  for (size_t i = 0; i < n; i++) v[i] = uint8_t(i * 7 + 3);
  return v;
}

// Steps the installer every ms until it ends (or limit_ms passes).
uint32_t run(Installer &ins, FakeCharger &c, uint32_t limit_ms = 200000) {
  uint32_t t = c.now;
  for (; ins.active() && t < limit_ms; t++) {
    c.tick(t);
    ins.step(c, t);
  }
  return t;
}

void test_installs_and_programs_the_image() {
  FakeCharger c;
  const auto dat = image(37400);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  const uint32_t t = run(ins, c);
  CHECK(ins.outcome() == Outcome::INSTALLED && ins.phase() == Phase::DONE);
  CHECK(ins.version() == 0x030B && ins.charger_state() == 3 && ins.rebooted());
  CHECK(c.length == 37400 && c.offset == 37400);
  CHECK(std::memcmp(c.flash.data(), dat.data(), dat.size()) == 0);
  CHECK(ins.frames_done() == ins.frames_total() && ins.frames_total() == 585);  // 37400 / 64, rounded up
  CHECK(ins.progress() > 99.9f);
  CHECK(!c.bauds.empty() && c.bauds.back() == 9600);  // the line goes back to Modbus
  CHECK(t > 1000 + QUIET_MS);                        // waited for the standby second and the quiet
}

void test_refuses_while_charging() {
  FakeCharger c;
  c.state = STATE_CHARGING;
  const auto dat = image(64);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::REFUSED_CHARGING && ins.error() != nullptr);
  CHECK(c.modbus_writes == 1 && !c.reset_pending && c.handshakes == 0);  // only the read went out
  CHECK(c.bauds.back() == 9600);
}

void test_refuses_with_the_menu_open() {
  FakeCharger c;
  c.menu_open = true;
  const auto dat = image(64);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::REFUSED_MENU && c.handshakes == 0 && !c.reset_pending);
  CHECK(c.bauds.back() == 9600);
}

void test_read_exception_skips_the_reboot_and_waits() {
  FakeCharger c;
  c.exception = 2;  // something answers, not as our firmware
  c.power_cycle_at = 0;
  const auto dat = image(64);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(c.modbus_writes == 1 && !c.reset_pending);  // no HR 120
  CHECK(ins.outcome() == Outcome::NO_BOOTLOADER);    // and nobody power-cycled it
}

void test_lost_reboot_ack_still_installs() {
  FakeCharger c;
  c.ack_reboot = false;
  const auto dat = image(1024);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::INSTALLED && !ins.rebooted());
}

void test_single_handshake_reply_is_not_a_bootloader() {
  FakeCharger c;
  c.hs_replies = 1;
  const auto dat = image(64);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::NO_BOOTLOADER && c.length < 0);
}

void test_bad_ack_crc_aborts_without_resending() {
  for (int seq : {0, 1}) {
    FakeCharger c;
    c.bad_crc_seq = seq;
    const auto dat = image(1024);
    Installer ins;
    ins.start(dat.data(), dat.size(), 0);
    run(ins, c);
    CHECK(ins.outcome() == Outcome::FAILED_AFTER_HEADER);
    CHECK(c.offset == size_t(seq) * FRAME_BYTES);  // the header's: nothing sent; frame 1's: programmed once
  }
}

void test_silent_charger_waits_for_a_power_cycle() {
  FakeCharger c;
  c.app = false;            // stock, or stuck in the bootloader after a failed install
  c.power_cycle_at = 20000; // the user power-cycles 20 s later
  const auto dat = image(1024);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::INSTALLED && !ins.rebooted() && ins.version() == -1);
  CHECK(std::memcmp(c.flash.data(), dat.data(), dat.size()) == 0);
}

void test_no_bootloader_gives_up_untouched() {
  FakeCharger c;
  c.app = false;  // nobody ever answers
  const auto dat = image(64);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  const uint32_t t = run(ins, c);
  CHECK(ins.outcome() == Outcome::NO_BOOTLOADER && c.length < 0);
  CHECK(t >= WAIT_SILENT_MS && t < WAIT_SILENT_MS + 2000);
  CHECK(c.bauds.back() == 9600);
}

void test_lost_ack_aborts_without_resending() {
  FakeCharger c;
  c.drop_ack_seq = 2;
  const auto dat = image(1024);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::FAILED_AFTER_HEADER);
  CHECK(std::strstr(ins.error(), "power-cycle") != nullptr);
  CHECK(c.offset == 2 * FRAME_BYTES);  // frames 1 and 2 programmed once; frame 2 never sent again
  CHECK(c.bauds.back() == 9600);
}

void test_wrong_sequence_ack_aborts() {
  FakeCharger c;
  c.bad_ack_seq = 1;
  const auto dat = image(1024);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::FAILED_AFTER_HEADER && c.offset == FRAME_BYTES);
}

void test_echoing_adapter_is_compensated() {
  FakeCharger c;
  c.echo = true;
  const auto dat = image(4096);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::INSTALLED);
  CHECK(std::memcmp(c.flash.data(), dat.data(), dat.size()) == 0);
}

void test_slow_acks_within_three_seconds_are_fine() {
  FakeCharger c;
  const auto dat = image(512);
  {
    FakeCharger c2;
    c2.boot_latency_ms = 2900;  // data acks just inside DATA_ACK_MS
    Installer ins;
    ins.start(dat.data(), dat.size(), 0);
    run(ins, c2);
    CHECK(ins.outcome() == Outcome::INSTALLED);
  }
  c.boot_latency_ms = 3100;  // just outside: abort, never resend
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(ins.outcome() == Outcome::FAILED_AFTER_HEADER && c.offset == FRAME_BYTES);
}

void test_frames_fit_the_uart_fifo_and_failures_say_which() {
  FakeCharger c;
  c.drop_ack_seq = 3;
  const auto dat = image(1024);
  Installer ins;
  ins.start(dat.data(), dat.size(), 0);
  run(ins, c);
  CHECK(9 + FRAME_BYTES <= 128);  // one write fills the ESP32's TX FIFO: no refill gap inside a frame
  CHECK(ins.outcome() == Outcome::FAILED_AFTER_HEADER);
  CHECK(std::strncmp(ins.detail(), "frame 3 of 16, 0 bytes back:", 29) == 0);
}

void test_bad_images_are_refused_before_anything_is_sent() {
  FakeCharger c;
  const auto dat = image(MAX_IMAGE + 8);
  for (size_t n : {size_t(0), size_t(13), MAX_IMAGE + 8}) {
    Installer ins;
    ins.start(dat.data(), n, 0);
    CHECK(!ins.active() && ins.outcome() == Outcome::BAD_IMAGE);
  }
  CHECK(c.modbus_writes == 0 && c.bauds.empty());
}

}  // namespace

int main() {
  test_installs_and_programs_the_image();
  test_refuses_while_charging();
  test_refuses_with_the_menu_open();
  test_read_exception_skips_the_reboot_and_waits();
  test_lost_reboot_ack_still_installs();
  test_single_handshake_reply_is_not_a_bootloader();
  test_bad_ack_crc_aborts_without_resending();
  test_silent_charger_waits_for_a_power_cycle();
  test_no_bootloader_gives_up_untouched();
  test_lost_ack_aborts_without_resending();
  test_wrong_sequence_ack_aborts();
  test_echoing_adapter_is_compensated();
  test_slow_acks_within_three_seconds_are_fine();
  test_frames_fit_the_uart_fifo_and_failures_say_which();
  test_bad_images_are_refused_before_anything_is_sent();
  return test_result("test_charger_update");
}
