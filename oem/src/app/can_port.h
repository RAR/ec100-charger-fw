// oem/src/app/can_port.h: the CAN interface the rectifiers talk through, and the software TX queue that feeds the
// three hardware mailboxes from the main loop (the driver never waits for a mailbox).
#pragma once
#include <cstdint>

namespace oem {

struct CanFrame {
  uint32_t id;  // 29-bit extended ID
  uint8_t len;
  uint8_t data[8];
};

class CanPort {
 public:
  // Queue an 8-byte extended data frame. False if it can't be queued; nothing waits.
  virtual bool send(uint32_t id, const uint8_t *data8) = 0;
};

constexpr int CAN_TX_QUEUE_LEN = 16;  // as the ESP's tx_queue_len

class CanTxQueue : public CanPort {
 public:
  bool send(uint32_t id, const uint8_t *data8) override {
    if (count_ == CAN_TX_QUEUE_LEN) return false;
    CanFrame &f = ring_[(head_ + count_) % CAN_TX_QUEUE_LEN];
    f.id = id;
    f.len = 8;
    for (int i = 0; i < 8; i++) f.data[i] = data8[i];
    count_++;
    return true;
  }

  // Hands queued frames, oldest first, to hw_send(const CanFrame &) -> bool until it reports no free mailbox.
  template <class F>
  void pump(F &&hw_send) {
    while (count_ > 0 && hw_send(ring_[head_])) {
      head_ = (head_ + 1) % CAN_TX_QUEUE_LEN;
      count_--;
    }
  }

  void clear() { head_ = count_ = 0; }
  int size() const { return count_; }

 private:
  CanFrame ring_[CAN_TX_QUEUE_LEN] = {};
  int head_ = 0;
  int count_ = 0;
};

// With no module online nothing ACKs: bxCAN retransmits its mailboxes forever and the queue fills with frames the
// modules would receive first when they return (setpoints from before the outage). update() is true once each time
// the last module goes offline; the caller then drops the queue and aborts the mailboxes.
class OutageFlush {
 public:
  bool update(bool any_online) {
    const bool flush = was_online_ && !any_online;
    was_online_ = any_online;
    return flush;
  }

 private:
  bool was_online_ = false;
};

}  // namespace oem
