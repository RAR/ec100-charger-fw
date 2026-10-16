// charger_update installer: puts a .dat on the charger through the stock bootloader, as tools/oem_update.py
// --modbus-reboot does (docs/history/superpowers/specs/2026-10-16-bridge-charger-update-design.md §3). Plain C++ (no
// ESPHome), non-blocking: step() is called every loop with the time and does at most one short exchange of bytes, so
// it builds in the host tests (tests/test_charger_update.cpp).
//
// Bootloader protocol (docs/history/notes/oem-hw-map.md §6.2): handshake F9 45 F9 53 23 -> F9 45 03 00 21 9D (counted
// once it has come twice), a header frame with the image length (the first after a boot erases the app area), then
// data frames of <= 256 bytes, each acked FB 03 00 sH sL with a Modbus CRC. The bootloader doesn't check sequence
// numbers, so a frame is never resent: any error after the header ends the install, and only a power-cycle (the next
// boot's erase) makes a retry safe.
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace charger_update_logic {

constexpr uint8_t MODBUS_ADDR = 1;
constexpr uint32_t MODBUS_BAUD = 9600, BOOT_BAUD = 460800;
constexpr uint16_t IR_VERSION = 0, HR_REBOOT = 120, REBOOT_MAGIC = 0xB007;
constexpr uint16_t STATE_CHARGING = 2;  // IR 1
constexpr size_t MAX_IMAGE = 0x19000;   // the bootloader's app area (tools/oem_image.py MAX_LEN)
// 64 data bytes, 73 on the line: a whole frame fits the ESP32's 128-byte UART TX FIFO in one write. The bootloader
// ends a frame at an idle line, so a 265-byte frame whose FIFO refill came late (WiFi and web traffic) arrived as two
// CRC-bad halves and was never acked (seen on charger B, 2026-10-16). Any multiple of 8 up to 256 is accepted.
constexpr size_t FRAME_BYTES = 64;
constexpr uint32_t SETTLE_MS = 300;          // a poll in flight when the line was taken finishes meanwhile
constexpr uint32_t MODBUS_REPLY_MS = 500;
constexpr uint32_t WAIT_REBOOTED_MS = 30000;  // the charger acked HR 120
constexpr uint32_t WAIT_SILENT_MS = 60000;    // the charger didn't answer Modbus: time for the user to power-cycle it
constexpr uint32_t HANDSHAKE_READ_MS = 100, HANDSHAKE_PAUSE_MS = 200, REPLY_GAP_MS = 1000;
constexpr uint32_t HEADER_ACK_MS = 5000;  // erasing 100 pages
constexpr uint32_t DATA_ACK_MS = 3000;
constexpr uint32_t QUIET_MS = 2000;  // > 1.5 s of silence makes the bootloader start the app

inline uint16_t crc16_modbus(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// The RS485 line while the installer owns it.
class LineIo {
 public:
  virtual ~LineIo() = default;
  virtual void write(const uint8_t *data, size_t len) = 0;
  virtual size_t read(uint8_t *data, size_t max) = 0;  // what has arrived, without waiting
  virtual void set_baud(uint32_t baud) = 0;
};

enum class Phase : uint8_t { IDLE, SETTLE, MODBUS_READ, MODBUS_REBOOT, HANDSHAKE, HEADER, DATA, QUIET, DONE, FAILED };

enum class Outcome : uint8_t {
  NONE,
  INSTALLED,
  REFUSED_CHARGING,   // IR 1 said CHARGING: nothing was sent past the read
  REFUSED_MENU,       // HR 120 refused with exception 06 (3.12+): the panel menu is open
  BAD_IMAGE,          // length 0, not a multiple of 8, or too big
  NO_BOOTLOADER,      // no handshake within the wait: the app is untouched
  FAILED_AFTER_HEADER // the app area is erased or half written: power-cycle, then install again
};

class Installer {
 public:
  // dat must stay valid until the install ends.
  void start(const uint8_t *dat, size_t len, uint32_t now) {
    *this = Installer();
    dat_ = dat;
    len_ = len;
    frames_ = (len + FRAME_BYTES - 1) / FRAME_BYTES;
    if (len == 0 || len % 8 != 0 || len > MAX_IMAGE) {
      finish_(Outcome::BAD_IMAGE, "image length is 0, not a multiple of 8, or too big");
      return;
    }
    enter_(Phase::SETTLE, now);
  }

  // Every loop while active(). The caller hands the line back once it returns false.
  void step(LineIo &io, uint32_t now) {
    switch (phase_) {
      case Phase::SETTLE:
        drain_(io);
        if (now - since_ >= SETTLE_MS) {
          io.set_baud(MODBUS_BAUD);
          send_modbus_(io, read_request_(), now, Phase::MODBUS_READ);
        }
        break;
      case Phase::MODBUS_READ: step_modbus_read_(io, now); break;
      case Phase::MODBUS_REBOOT: step_modbus_reboot_(io, now); break;
      case Phase::HANDSHAKE: step_handshake_(io, now); break;
      case Phase::HEADER:
      case Phase::DATA: step_frame_(io, now); break;
      case Phase::QUIET:
        drain_(io);
        if (now - since_ >= QUIET_MS) {
          io.set_baud(MODBUS_BAUD);
          finish_(Outcome::INSTALLED, nullptr);
        }
        break;
      default: break;
    }
    if (phase_ == Phase::FAILED) io.set_baud(MODBUS_BAUD);
  }

  bool active() const { return phase_ != Phase::IDLE && phase_ != Phase::DONE && phase_ != Phase::FAILED; }
  Phase phase() const { return phase_; }
  Outcome outcome() const { return outcome_; }
  const char *error() const { return error_; }
  // For the log after a failure in the frames: which frame, and what came back.
  const char *detail() const { return detail_; }
  // What the charger reported before the reboot (-1 when it didn't answer).
  int version() const { return version_; }
  int charger_state() const { return state_; }
  bool rebooted() const { return rebooted_; }
  size_t frames_done() const { return frames_done_; }
  size_t frames_total() const { return frames_; }
  float progress() const { return frames_ == 0 ? 0.0f : 100.0f * float(frames_done_) / float(frames_); }

 private:
  void enter_(Phase p, uint32_t now) {
    phase_ = p;
    since_ = now;
    got_ = 0;
  }

  void note_(uint16_t seq, size_t got) {
    int k = std::snprintf(detail_, sizeof detail_, "frame %u of %u, %u bytes back:", unsigned(seq), unsigned(frames_),
                          unsigned(got));
    for (size_t i = 0; i < got && i < 8 && k > 0 && size_t(k) < sizeof detail_ - 4; i++)
      k += std::snprintf(detail_ + k, sizeof detail_ - size_t(k), " %02X", rx_[i]);
  }

  void finish_(Outcome o, const char *error) {
    outcome_ = o;
    error_ = error;
    phase_ = o == Outcome::INSTALLED ? Phase::DONE : Phase::FAILED;
  }

  static void drain_(LineIo &io) {
    uint8_t junk[64];
    while (io.read(junk, sizeof junk) > 0) {
    }
  }

  // Reads what has arrived into rx_ (at most `want` bytes). While our own transmission may still be coming back (an
  // echoing adapter), bytes matching it are dropped as they arrive. A frame's echo (up to 265 bytes) never needs to
  // fit rx_. A mismatch on the first byte means no echo; a mismatch later is a corrupted echo once we know the adapter
  // echoes, or else the bytes were the reply itself and go back into rx_.
  void collect_(LineIo &io, size_t want) {
    uint8_t b;
    while (got_ < want && io.read(&b, 1) == 1) {
      if (echo_pos_ < echo_skip_) {
        if (b == tx_[echo_pos_]) {
          if (++echo_pos_ == echo_skip_) echo_ = true;
          continue;
        }
        if (echo_ && echo_pos_ > 0) {
          bad_echo_ = true;
          echo_skip_ = 0;
          return;
        }
        for (size_t i = 0; i < echo_pos_ && got_ < sizeof rx_; i++) rx_[got_++] = tx_[i];
        echo_skip_ = 0;
        if (got_ >= want) break;
      }
      if (got_ < sizeof rx_) rx_[got_++] = b;
    }
  }

  void send_(LineIo &io, const uint8_t *data, size_t len, bool maybe_echo) {
    drain_(io);  // like the vendor tool: whatever came before belongs to nothing we are waiting for
    std::memcpy(tx_, data, len);
    tx_len_ = len;
    echo_skip_ = maybe_echo ? len : 0;
    echo_pos_ = 0;
    bad_echo_ = false;
    io.write(data, len);
  }

  // ---- Modbus, at 9600 ----
  struct Frame {
    uint8_t b[8];
  };
  static Frame with_crc_(uint8_t fc, uint16_t reg, uint16_t value) {
    Frame f{{MODBUS_ADDR, fc, uint8_t(reg >> 8), uint8_t(reg), uint8_t(value >> 8), uint8_t(value), 0, 0}};
    const uint16_t crc = crc16_modbus(f.b, 6);
    f.b[6] = uint8_t(crc);
    f.b[7] = uint8_t(crc >> 8);
    return f;
  }
  static Frame read_request_() { return with_crc_(0x04, IR_VERSION, 2); }  // IR 0 version, IR 1 state

  // The IR read goes first and tells whether the adapter echoes: its reply can't pass for its echo, whereas the FC 06
  // ack is identical to its request, so that one only skips an echo already known to exist.
  void send_modbus_(LineIo &io, const Frame &f, uint32_t now, Phase next) {
    enter_(next, now);
    send_(io, f.b, sizeof f.b, next == Phase::MODBUS_READ || echo_);
  }

  // True with code set when rx_ holds a whole, CRC-good exception reply for fc.
  bool exception_(uint8_t fc, uint8_t &code) const {
    if (got_ < 5 || rx_[0] != MODBUS_ADDR || rx_[1] != (fc | 0x80)) return false;
    if (crc16_modbus(rx_, 3) != uint16_t(rx_[3] | rx_[4] << 8)) return false;
    code = rx_[2];
    return true;
  }

  void step_modbus_read_(LineIo &io, uint32_t now) {
    collect_(io, 9);
    uint8_t code;
    if (exception_(0x04, code)) {
      to_handshake_(io, now, false);  // something answers, but not as our firmware does
      return;
    }
    if (got_ >= 9) {
      const bool ok = rx_[0] == MODBUS_ADDR && rx_[1] == 0x04 && rx_[2] == 4 &&
                      crc16_modbus(rx_, 7) == uint16_t(rx_[7] | rx_[8] << 8);
      if (ok) {
        version_ = rx_[3] << 8 | rx_[4];
        state_ = rx_[5] << 8 | rx_[6];
        if (state_ == STATE_CHARGING) {
          finish_(Outcome::REFUSED_CHARGING, "the charger is charging; pause it, then install again");
          return;
        }
        send_modbus_(io, with_crc_(0x06, HR_REBOOT, REBOOT_MAGIC), now, Phase::MODBUS_REBOOT);
        return;
      }
      to_handshake_(io, now, false);
      return;
    }
    if (now - since_ >= MODBUS_REPLY_MS) to_handshake_(io, now, false);  // not our firmware: wait for a power-cycle
  }

  void step_modbus_reboot_(LineIo &io, uint32_t now) {
    collect_(io, 8);
    uint8_t code;
    if (exception_(0x06, code)) {
      if (code == 6) {
        finish_(Outcome::REFUSED_MENU, "the menu is open on the charger's panel; close it and install again");
        return;
      }
      to_handshake_(io, now, false);
      return;
    }
    const Frame want = with_crc_(0x06, HR_REBOOT, REBOOT_MAGIC);
    if (got_ >= 8) {
      to_handshake_(io, now, std::memcmp(rx_, want.b, 8) == 0);
      return;
    }
    if (now - since_ >= MODBUS_REPLY_MS) to_handshake_(io, now, false);  // it may or may not have reset
  }

  // ---- bootloader, at 460800 ----
  static constexpr uint8_t HANDSHAKE[5] = {0xF9, 0x45, 0xF9, 0x53, 0x23};
  static constexpr uint8_t HANDSHAKE_REPLY[6] = {0xF9, 0x45, 0x03, 0x00, 0x21, 0x9D};

  void to_handshake_(LineIo &io, uint32_t now, bool rebooted) {
    rebooted_ = rebooted;
    io.set_baud(BOOT_BAUD);
    enter_(Phase::HANDSHAKE, now);
    wait_until_ = now + (rebooted ? WAIT_REBOOTED_MS : WAIT_SILENT_MS);
    replies_ = 0;
    hs_sent_ = false;
    hs_next_ = now;
  }

  void step_handshake_(LineIo &io, uint32_t now) {
    if (!hs_sent_) {
      if (int32_t(now - hs_next_) < 0) return;
      if (int32_t(now - wait_until_) >= 0) {
        finish_(Outcome::NO_BOOTLOADER, "no answer from the charger's bootloader; power-cycle it, then install again");
        return;
      }
      send_(io, HANDSHAKE, sizeof HANDSHAKE, true);
      got_ = 0;
      hs_sent_ = true;
      hs_at_ = now;
      return;
    }
    collect_(io, sizeof HANDSHAKE_REPLY);
    if (got_ == sizeof HANDSHAKE_REPLY && std::memcmp(rx_, HANDSHAKE_REPLY, sizeof HANDSHAKE_REPLY) == 0) {
      // The stock app answers once, then resets into the bootloader, so a reply counts once it has come twice.
      replies_ = replies_ > 0 && now - last_reply_ <= REPLY_GAP_MS ? replies_ + 1 : 1;
      last_reply_ = now;
      hs_sent_ = false;
      if (replies_ == 2) {
        send_frame_(io, 0, now);  // the header
        return;
      }
      hs_next_ = now + HANDSHAKE_PAUSE_MS;
      return;
    }
    if (now - hs_at_ >= HANDSHAKE_READ_MS) {
      hs_sent_ = false;
      hs_next_ = now;
    }
  }

  void send_frame_(LineIo &io, uint16_t seq, uint32_t now) {
    uint8_t f[9 + FRAME_BYTES];
    uint8_t len4[4];
    const uint8_t *body;
    size_t n;
    if (seq == 0) {
      len4[0] = uint8_t(len_ >> 24);
      len4[1] = uint8_t(len_ >> 16);
      len4[2] = uint8_t(len_ >> 8);
      len4[3] = uint8_t(len_);
      body = len4;
      n = 4;
    } else {
      const size_t off = size_t(seq - 1) * FRAME_BYTES;
      body = dat_ + off;
      n = len_ - off < FRAME_BYTES ? len_ - off : FRAME_BYTES;
    }
    f[0] = 0xFB;
    f[1] = uint8_t(n >> 8);
    f[2] = uint8_t(n);
    f[3] = 0x03;
    f[4] = 0x00;
    f[5] = uint8_t(seq >> 8);
    f[6] = uint8_t(seq);
    std::memcpy(f + 7, body, n);
    const uint16_t crc = crc16_modbus(f, 7 + n);
    f[7 + n] = uint8_t(crc);
    f[8 + n] = uint8_t(crc >> 8);
    seq_ = seq;
    enter_(seq == 0 ? Phase::HEADER : Phase::DATA, now);
    send_(io, f, 9 + n, echo_);  // after the handshake we know whether the adapter echoes
  }

  void step_frame_(LineIo &io, uint32_t now) {
    collect_(io, 7);
    if (bad_echo_) {
      finish_(Outcome::FAILED_AFTER_HEADER,
              "the line corrupted a frame (bad echo); power-cycle the charger, then install again");
      return;
    }
    if (got_ >= 7) {
      uint8_t want[5] = {0xFB, 0x03, 0x00, uint8_t(seq_ >> 8), uint8_t(seq_)};
      const bool ok = std::memcmp(rx_, want, 5) == 0 && crc16_modbus(rx_, 5) == uint16_t(rx_[5] | rx_[6] << 8);
      if (!ok) {
        note_(seq_, got_);
        finish_(Outcome::FAILED_AFTER_HEADER,
                "the charger's bootloader answered a frame wrongly; power-cycle the charger, then install again");
        return;
      }
      if (seq_ > 0) frames_done_ = seq_;
      if (seq_ == frames_) {
        enter_(Phase::QUIET, now);
        return;
      }
      send_frame_(io, uint16_t(seq_ + 1), now);
      return;
    }
    if (now - since_ >= (phase_ == Phase::HEADER ? HEADER_ACK_MS : DATA_ACK_MS)) {
      note_(seq_, got_);
      finish_(Outcome::FAILED_AFTER_HEADER,
              "no answer from the charger's bootloader to a frame; power-cycle the charger, then install again");
    }
  }

  Phase phase_ = Phase::IDLE;
  Outcome outcome_ = Outcome::NONE;
  const char *error_ = nullptr;
  const uint8_t *dat_ = nullptr;
  size_t len_ = 0, frames_ = 0, frames_done_ = 0;
  uint32_t since_ = 0, wait_until_ = 0, hs_next_ = 0, hs_at_ = 0, last_reply_ = 0;
  uint8_t rx_[16] = {};
  size_t got_ = 0;
  uint8_t tx_[9 + FRAME_BYTES] = {};
  char detail_[96] = {};
  size_t tx_len_ = 0, echo_skip_ = 0, echo_pos_ = 0;
  bool bad_echo_ = false;
  bool echo_ = false, rebooted_ = false, hs_sent_ = false;
  int replies_ = 0;
  uint16_t seq_ = 0;
  int version_ = -1, state_ = -1;
};

}  // namespace charger_update_logic
