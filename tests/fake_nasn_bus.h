#pragma once
// Host-test stand-ins for the CAN bus and NASN modules, modelled on tools/fake_nasn.py: a module answers cmd 0x82
// reads and cmd 0x81 writes; a running module whose setpoint is above the battery delivers its current limit.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "app/can_port.h"
#include "app/rectifier.h"

struct FakeModule {
  explicit FakeModule(uint8_t a) : addr(a) {}

  uint8_t addr;
  float battery_v = 52.0f, vin = 230.0f, heatsink_c = 35.0f;
  float vout_set = 54.0f, ilimit = 5.0f, default_v = 54.0f, default_a = 5.0f;
  int standby = 0;
  uint32_t alarm = 0;           // low 32 bits of the 0x0183 value
  bool muted = false;
  bool reject_writes = false;   // answer writes with status 2
  bool frozen = false;          // ignore setpoint writes (but ack them): a module not following setpoints
  bool forgets_defaults = false;  // ack 0x0101 / 0x0104 writes but never persist them
  bool load_connected = true;   // false: DC output open, no current flows

  float iout() const { return !standby && load_connected && vout_set > battery_v ? ilimit : 0.0f; }
  // A running module set below the battery can't pull it down: it reads the battery voltage.
  float vout() const { return standby ? battery_v : std::max(vout_set, battery_v); }

  uint64_t read(uint16_t reg, bool &ok) const {
    ok = true;
    auto t = [](float x) { return uint64_t(std::lround(double(x) * 1024.0)); };
    switch (reg) {
      case 0x0100: return uint64_t(std::lround(vout_set * 1023.0f));
      case 0x0101: return uint64_t(std::lround(default_v * 1023.0f));
      case 0x0104: return uint64_t(std::lround(default_a * 1023.0f / 52.0f));
      case 0x0132: return uint64_t(standby);
      case 0x0175: return t(vout());
      case 0x0178: return t(vin);
      case 0x0182: return t(iout());
      case 0x0183: return alarm;
      case 0x0184: return t(vout());
      case 0x0235: return t(heatsink_c);
      case 0x022D: return uint64_t(std::lround(ilimit * 1023.0f / 52.0f));
      case 0x022F: return uint64_t(std::lround(vout_set * 1023.0f));
      default: ok = false; return 0;
    }
  }

  bool handle(const oem::CanFrame &req, oem::CanFrame &rep) {
    if (muted || req.len != 8 || ((req.id >> 23) & 0x3F) != 0x21 || !(req.id & 0x80)) return false;
    if (((req.id >> 16) & 0x7F) != addr) return false;
    const uint8_t cmd = (req.id >> 8) & 0xFF;
    const uint16_t reg = uint16_t((req.data[0] & 0x0F) << 8 | req.data[1]);
    rep.id = req.id & ~0x80u;
    rep.len = 8;
    for (int i = 0; i < 8; i++) rep.data[i] = req.data[i];
    if (cmd == 0x82) {
      bool ok;
      const uint64_t v = read(reg, ok);
      for (int i = 0; i < 6; i++) rep.data[2 + i] = ok ? uint8_t(v >> (8 * (5 - i))) : 0;
      if (!ok) rep.data[0] = uint8_t(0x20 | (req.data[0] & 0x0F));
      return true;
    }
    if (cmd != 0x81) return false;
    if (reject_writes) {
      rep.data[0] = uint8_t(0x20 | (req.data[0] & 0x0F));
      return true;
    }
    const uint16_t raw = uint16_t(req.data[6] << 8 | req.data[7]);
    if (frozen) return true;
    if (reg == 0x0100) vout_set = raw / 1023.0f;
    if (reg == 0x022D) ilimit = raw * 52.0f / 1023.0f;
    if (reg == 0x0132) standby = req.data[3];
    if (forgets_defaults) return true;
    if (reg == 0x0101) default_v = raw / 1023.0f;
    if (reg == 0x0104) default_a = raw * 52.0f / 1023.0f;
    return true;
  }
};

class FakeBus : public oem::CanPort {
 public:
  bool send(uint32_t id, const uint8_t *d) override {
    if (full || accept == 0) {
      refused_writes += ((id >> 8) & 0xFF) == 0x81;
      return false;
    }
    if (accept > 0) accept--;
    oem::CanFrame f{id, 8, {}};
    for (int i = 0; i < 8; i++) f.data[i] = d[i];
    sent.push_back(f);
    pending.push_back(f);
    return true;
  }

  // Every pending request reaches the modules; every reply reaches every rectifier.
  void deliver(std::vector<FakeModule *> mods, std::vector<oem::Rectifier *> rects, uint32_t now) {
    std::vector<oem::CanFrame> reqs;
    reqs.swap(pending);
    for (const auto &q : reqs) {
      for (auto *m : mods) {
        oem::CanFrame rep{};
        if (!m->handle(q, rep)) continue;
        for (auto *r : rects) r->on_frame(rep, now);
      }
    }
  }

  // Writes (cmd 0x81) to reg sent so far.
  int writes(uint16_t reg) const {
    int n = 0;
    for (const auto &f : sent) n += ((f.id >> 8) & 0xFF) == 0x81 && (f.data[0] << 8 | f.data[1]) == reg;
    return n;
  }

  std::vector<oem::CanFrame> sent, pending;
  bool full = false;
  int accept = -1;  // frames still accepted (-1: no limit)
  int refused_writes = 0;
};
