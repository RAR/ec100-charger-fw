// The setpoint delivery path end to end: two Rectifiers and the Controller, the real 16-frame CanTxQueue, and a model
// of the three bxCAN mailboxes that only gets a frame onto the bus while some module ACKs it, as main.cpp wires them.
#include <cmath>
#include <vector>

#include "app/controller.h"
#include "check.h"
#include "fake_nasn_bus.h"

using namespace oem;
using charge::State;

// Three mailboxes in request order (TFO). A frame leaves every 2 ms (request plus reply at 125 kbit/s) while a
// module on the bus ACKs; with none, bxCAN retransmits the oldest one forever.
struct Mailboxes {
  CanFrame slot[3];
  int n = 0;

  bool send(const CanFrame &f) {
    if (n == 3) return false;
    slot[n++] = f;
    return true;
  }
  void abort() { n = 0; }
};

struct Write {
  uint8_t addr;
  uint16_t reg;
  uint16_t raw;   // bytes 6..7
  uint8_t flag;   // byte 3 (standby)
};

// A CanPort that accepts at most `left` more frames (-1: no limit): a queue with only that much room this pass.
struct Gate : CanPort {
  explicit Gate(CanPort &n) : next(n) {}
  bool send(uint32_t id, const uint8_t *d) override {
    if (left == 0) return false;
    if (left > 0) left--;
    return next.send(id, d);
  }
  CanPort &next;
  int left = -1;
};

struct Rig {
  CanTxQueue txq;
  Gate gate{txq};
  Mailboxes mb;
  FakeModule m1{1}, m2{2};
  Rectifier r1{gate, 1, true}, r2{gate, 2, true};
  Controller ctl{r1, r2};
  Settings s = default_settings();
  uint32_t now = 1000;
  Status st;
  int per_ms = -1;                     // frames the gate accepts each pass
  std::vector<Write> writes;           // writes the modules processed
  int on_at_zero[2] = {0, 0}, worst_on_at_zero[2] = {0, 0};  // ms a module ran at 0 A (current and longest run)

  void bus_step() {
    if (now % 2 != 0 || mb.n == 0 || (m1.muted && m2.muted)) return;  // muted: no AC, so no ACK either
    const CanFrame f = mb.slot[0];
    mb.slot[0] = mb.slot[1];
    mb.slot[1] = mb.slot[2];
    mb.n--;
    for (FakeModule *m : {&m1, &m2}) {
      CanFrame rep{};
      if (!m->handle(f, rep)) continue;
      if (((f.id >> 8) & 0xFF) == 0x81)
        writes.push_back({m->addr, uint16_t((f.data[0] & 0x0F) << 8 | f.data[1]), uint16_t(f.data[6] << 8 | f.data[7]),
                          f.data[3]});
      r1.on_frame(rep, now);
      r2.on_frame(rep, now);
    }
  }

  void run_ms(uint32_t ms) {
    for (uint32_t t = 0; t < ms; t++, now++) {
      bus_step();
      gate.left = per_ms;
      r1.loop(now);
      r2.loop(now);
      if (flush.update(r1.is_online() || r2.is_online())) {
        txq.clear();
        mb.abort();
      }
      txq.pump([this](const CanFrame &f) { return mb.send(f); });
      if (now % CONTROLLER_TICK_MS == 0) st = ctl.tick(s, panel_limits(s), 50.0f, now);
      const FakeModule *mods[2] = {&m1, &m2};
      for (int k = 0; k < 2; k++) {
        const bool zero = !mods[k]->muted && mods[k]->standby == 0 && mods[k]->ilimit < 0.05f;
        on_at_zero[k] = zero ? on_at_zero[k] + 1 : 0;
        if (on_at_zero[k] > worst_on_at_zero[k]) worst_on_at_zero[k] = on_at_zero[k];
      }
    }
  }
  void run_s(int s) { run_ms(uint32_t(s) * 1000u); }

  OutageFlush flush;
};

// AC back: a module boots running on its stored defaults (fake_nasn.py's reset_to_defaults).
static void ac_returns(FakeModule &m) {
  m.muted = false;
  m.vout_set = m.default_v;
  m.ilimit = m.default_a;
  m.standby = 0;
}

static void test_outage_pause_no_stale_replay() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 80);
  g.ctl.charge_now();
  g.run_s(60);
  CHECK(g.st.state == State::CHARGING);
  CHECK_NEAR(g.m1.ilimit, 40.0, 0.1);
  CHECK_NEAR(g.m2.ilimit, 40.0, 0.1);
  g.m1.muted = g.m2.muted = true;  // both modules lose AC
  g.run_s(20);
  g.ctl.toggle_pause();            // the user pauses during the outage
  g.run_s(40);
  CHECK(g.st.online == 0 && g.st.paused);
  g.writes.clear();
  ac_returns(g.m1);
  ac_returns(g.m2);
  g.run_s(20);
  CHECK(g.st.online == 2 && g.st.state == State::HOLD && g.st.hold == HoldReason::PAUSED);
  for (uint8_t addr : {1, 2}) {
    int ilimit = 0, standby = 0;
    for (const Write &w : g.writes) {
      if (w.addr != addr) continue;
      if (w.reg == 0x022D) {
        ilimit++;
        CHECK(w.raw == 0);  // no stale ILIMIT 40 A, first or later
      }
      if (w.reg == 0x0132) {
        standby++;
        CHECK(w.flag == 1);  // no stale STANDBY 0
      }
    }
    CHECK(ilimit > 0 && standby > 0);
  }
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
}

static void test_partial_send_never_on_at_zero() {
  Rig g;
  g.s.set(HR_MAX_CURRENT, 40);
  g.ctl.charge_now();
  g.run_s(30);
  CHECK_NEAR(g.m1.ilimit, 20.0, 0.1);
  g.worst_on_at_zero[0] = g.worst_on_at_zero[1] = 0;  // the ramp's first tick legitimately runs at 0 A
  g.per_ms = 2;  // the queue takes only two frames per pass
  g.ctl.toggle_pause();
  g.run_s(5);
  CHECK(g.m1.standby == 1 && g.m2.standby == 1);
  CHECK(g.m1.ilimit < 0.05f && g.m2.ilimit < 0.05f);
  CHECK(g.worst_on_at_zero[0] <= 100 && g.worst_on_at_zero[1] <= 100);
  for (int n : {1, 3}) {  // and with one or three frames a pass
    Rig h;
    h.s.set(HR_MAX_CURRENT, 40);
    h.ctl.charge_now();
    h.run_s(30);
    h.worst_on_at_zero[0] = h.worst_on_at_zero[1] = 0;
    h.per_ms = n;
    h.ctl.toggle_pause();
    h.run_s(5);
    CHECK(h.m1.standby == 1 && h.m2.standby == 1);
    CHECK(h.worst_on_at_zero[0] <= 100 && h.worst_on_at_zero[1] <= 100);
  }
}

int main() {
  test_outage_pause_no_stale_replay();
  test_partial_send_never_on_at_zero();
  return test_result("test_oem_integration");
}
