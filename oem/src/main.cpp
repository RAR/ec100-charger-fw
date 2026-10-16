// oem/src/main.cpp: the charger app (Panel, Pylontech, Modbus, EG4, JK or JK-PB source; RS485 is the Modbus RTU
// server, or with Source mode Pylontech, EG4, JK or JK-PB the BMS poller). A cooperative super-loop (spec §1): CAN RX
// drain, the rectifier polls and the RS485 role every pass, keys 10 ms, ADC 100 ms, LCD 200 ms, controller 1 s,
// settings saved 5 s after the last change, and the watchdog fed once per pass and nowhere else. Holding Up + Down for
// 5 s puts both modules in standby, then restarts the board (service use: it opens the bootloader's update window).
#include <cmath>
#include <initializer_list>

#include "app/battery_adc.h"
#include "app/can_port.h"
#include "app/controller.h"
#include "app/limit_source.h"
#include "app/modbus_map.h"
#include "app/modbus_rtu.h"
#include "app/pause_memory.h"
#include "app/eg4_source.h"
#include "app/jk_source.h"
#include "app/jkpb_source.h"
#include "app/polled_source.h"
#include "app/pylon_source.h"
#include "app/rectifier.h"
#include "app/relay.h"
#include "app/settings_store.h"
#include "app/ui.h"
#include "board.h"
#include "hw/adc.h"
#include "hw/backup.h"
#include "hw/can.h"
#include "hw/clock.h"
#include "hw/flash.h"
#include "hw/rs485.h"
#include "hw/slcd.h"

using namespace oem;

uint32_t g_rs485_tx_aborts = 0;  // replies cut off by MB_TX_TIMEOUT_MS; for SWD

namespace {

constexpr uint32_t KEYS_MS = 10, ADC_MS = 100, LCD_MS = 200, SAVE_DELAY_MS = 5000, CAN_RETRY_MS = 5000;
constexpr uint32_t FWDGTRSTF = 1u << 29;         // RCU_RSTSCK: the last reset was the watchdog
constexpr uint32_t SWRSTF = 1u << 28;            // RCU_RSTSCK: a software reset (ours, for HR 120)
constexpr uint32_t MB_TX_TIMEOUT_MS = 300;       // the longest legal reply, 256 bytes at 9600, takes about 270 ms
constexpr uint32_t RESTART_STANDBY_MS = 1000;    // the standby frames' time to reach the modules
constexpr uint32_t RESTART_MAX_MS = 3000;        // restart by then even if CAN or RS485 is still busy
constexpr bool WRITE_STORED_DEFAULTS = true;     // spec §2.2; false leaves 0x0101 / 0x0104 alone

CanTxQueue g_txq;
Rectifier g_psu1(g_txq, 1, WRITE_STORED_DEFAULTS);
Rectifier g_psu2(g_txq, 2, WRITE_STORED_DEFAULTS);
Controller g_ctl(g_psu1, g_psu2);
OutageFlush g_outage;
Ui g_ui;
RelayLogic g_relay;
hw::Flash g_flash;
SettingsStore g_store(g_flash);
Settings g_settings;
Telemetry g_tel{};
OemRegisters g_regs(g_settings, g_tel);
FrameAssembler g_mb_rx;
uint8_t g_mb_resp[MB_MAX_FRAME];
PylonSource g_pylon;
Eg4Source g_eg4;
JkSource g_jk;
JkpbSource g_jkpb;

// HR 6: a BMS source makes the board its poller on RS485; Panel and Modbus keep the Modbus server (nullptr).
PolledSource *poller_for(const Settings &s) {
  switch (SourceMode(s.get(HR_SOURCE_MODE))) {
    case SourceMode::PYLONTECH: return &g_pylon;
    case SourceMode::EG4: return &g_eg4;
    case SourceMode::JK: return &g_jk;
    case SourceMode::JKPB: return &g_jkpb;
    default: return nullptr;
  }
}

uint32_t modbus_baud(const Settings &s) {  // HR 8
  constexpr uint32_t BAUDS[4] = {9600, 19200, 38400, 115200};
  const int32_t i = s.get(HR_MODBUS_BAUD);
  return i >= 0 && i < 4 ? BAUDS[i] : BAUDS[0];
}

// True once the line runs at baud; false (busy or not ready) keeps the change pending for a later pass.
bool set_line_baud(uint32_t baud) { return hw::rs485_set_baud(baud); }

bool due(uint32_t &last, uint32_t period, uint32_t now) {
  if (now - last < period) return false;
  last = now;
  return true;
}

// Ten polled conversions, trimmed mean, scaled; NaN if the ADC doesn't answer.
float read_adc_volts(const Settings &s) {
  uint16_t samples[ADC_SAMPLES];
  for (uint16_t &v : samples) {
    if (!hw::adc_read(v)) return NAN;
  }
  return adc_volts(adc_trimmed_mean(samples), s.get(HR_ADC_TRIM));
}

void init_pins() {
  using board::Pin;
  for (Pin p : {board::KEY_SWITCH, board::KEY_UP, board::KEY_DOWN, board::KEY_ENTER})
    hw::pin_config(p, hw::Mode::INPUT, hw::Pull::DOWN);
  hw::pin_write(board::RS485_DE, false);  // receive until rs485_init takes the line
  hw::pin_write(board::LED_FAULT, true);  // LEDs are active low: off
  hw::pin_write(board::LED_CHARGE, true);
  hw::pin_write(board::RELAY, false);
  hw::pin_write(board::BACKLIGHT, true);
  for (Pin p : {board::RS485_DE, board::LED_FAULT, board::LED_CHARGE, board::RELAY, board::BACKLIGHT})
    hw::pin_config(p, hw::Mode::OUTPUT, hw::Pull::NONE);
}

}  // namespace

int main() {
  // CAN timing assumes APB1 = 30 MHz; on the IRC16M fallback the bus is left alone rather than run at a wrong rate.
  const bool hxtal = hw::clock_init();
  hw::fwdgt_init();
  hw::systick_init();
  board::enable_gpio_clocks();
  init_pins();
  const bool lcd = hw::slcd_init();
  const bool adc_ok = hw::adc_init();  // phase 3 reports it; a dead ADC reads NaN and the controller holds
  (void)adc_ok;
  bool can = hxtal && hw::can_init();
  const bool watchdog_reset = hw::reset_flags() & FWDGTRSTF;
  hw::backup_init();
  g_ctl.set_paused(restore_pause(watchdog_reset, hw::reset_flags() & SWRSTF, hw::backup_word()));
  hw::set_backup_word(pause_word(g_ctl.paused()));
  g_regs.sync_pause(g_ctl.paused());

  Settings &settings = g_settings;  // global so the Modbus register map can hold it
  if (!g_store.load(settings)) settings = default_settings();  // phase 3: import the stock v1 record first
  apply_clamps(settings);

  // Stored-defaults fallback (HR 15 V x 100, HR 16 A) before the first poll, so the modules never see 54.0 V / 5 A.
  const float fb_v = float(settings.get(HR_MODULE_FALLBACK_V)) / 100.0f;
  const float fb_a = float(settings.get(HR_MODULE_FALLBACK_A));
  g_psu1.set_fallback(fb_v, fb_a);
  g_psu2.set_fallback(fb_v, fb_a);

  // A USART that won't start leaves RS485 off (no Modbus, and a Pylontech source that stays stale); charging doesn't
  // depend on it. Pylontech starts the line at its own baud; mb_baud is what the Modbus server would use.
  PolledSource *active = poller_for(settings);
  SourceMode last_mode = SourceMode(settings.get(HR_SOURCE_MODE));
  uint32_t mb_baud = modbus_baud(settings);
  const bool rs485 = hw::rs485_init(active ? active->baud() : mb_baud);
  g_mb_rx.set_gap_ms(modbus_gap_ms(mb_baud));
  if (active) active->reset();
  bool mb_was_busy = false, mb_reboot = false;
  uint32_t mb_tx_start = 0;

  Status st;
  g_tel.st = &st;
  g_tel.psu[0] = &g_psu1;
  g_tel.psu[1] = &g_psu2;
  g_tel.ctl = &g_ctl;
  float adc_v = NAN;
  bool dirty = false, restarting = false, relay = false;
  const uint32_t t0 = hw::millis();
  uint32_t last_keys = t0, last_adc = t0, last_lcd = t0, last_ctl = t0, last_can_init = t0, changed_ms = t0;
  uint32_t last_second = t0, uptime_s = 0, restart_ms = t0;

  for (;;) {
    const uint32_t now = hw::millis();

    // A transient CAN init failure (no 11 recessive bits yet) shouldn't need a power cycle.
    if (hxtal && !can && due(last_can_init, CAN_RETRY_MS, now)) can = hw::can_init();

    CanFrame rx;
    while (can && hw::can_receive(rx)) {
      g_psu1.on_frame(rx, now);
      g_psu2.on_frame(rx, now);
    }
    g_psu1.loop(now);
    g_psu2.loop(now);
    if (g_outage.update(g_psu1.is_online() || g_psu2.is_online())) {
      g_txq.clear();  // both modules lost: what's queued or retrying is stale by the time they return
      if (can) hw::can_abort_tx();
    }
    if (can) g_txq.pump(hw::can_send);

    // Up + Down restart: the modules have been in standby for RESTART_STANDBY_MS and nothing is left to send.
    if (restarting) {
      const uint32_t held = now - restart_ms;
      if ((held >= RESTART_STANDBY_MS && g_txq.size() == 0 && !hw::rs485_tx_busy()) || held >= RESTART_MAX_MS) {
        if (dirty) g_store.save(settings);
        if (g_ctl.paused()) hw::set_backup_word(PAUSE_REBOOT_MAGIC);  // restore_pause keeps it across this reset
        hw::system_reset();
      }
    }

    if (due(last_keys, KEYS_MS, now)) {
      const bool raw[4] = {hw::pin_read(board::KEY_SWITCH), hw::pin_read(board::KEY_UP), hw::pin_read(board::KEY_DOWN),
                           hw::pin_read(board::KEY_ENTER)};
      const UiActions a = g_ui.on_keys(raw, settings, now);
      if (a.charge_now) g_ctl.charge_now();
      if (a.toggle_pause) {
        g_ctl.toggle_pause();
        hw::set_backup_word(pause_word(g_ctl.paused()));
        g_regs.sync_pause(g_ctl.paused());
      }
      if (a.settings_changed) {
        dirty = true;
        changed_ms = now;
      }
      if (!restarting && g_ui.restart_held()) {  // the SOC field has counted down 5 ... 1
        g_ctl.halt();  // both modules to standby now; the restart follows once that has gone out
        restarting = true;
        restart_ms = now;
      }
      const Leds l = leds(st, now);  // the pins are active low
      hw::pin_write(board::LED_FAULT, !fault_led(l, can, watchdog_reset, now - t0));
      hw::pin_write(board::LED_CHARGE, !l.charge);
      hw::pin_write(board::BACKLIGHT, g_ui.backlight(settings, now));
    }

    if (due(last_adc, ADC_MS, now)) adc_v = read_adc_volts(settings);

    // Spec §6: the RS485 role follows HR 6 (the menu changes it), switched only while nothing is going out and no
    // Modbus reboot is pending (that reboot comes up in the new role).
    PolledSource *const want = poller_for(settings);
    if (want != active && !hw::rs485_tx_busy() && !mb_reboot) {
      if (!rs485 || set_line_baud(want ? want->baud() : modbus_baud(settings))) {  // else retried next pass
        if (want) {
          want->reset();  // stale until the BMS answers at the new role
        } else {
          mb_baud = modbus_baud(settings);
          g_mb_rx.set_gap_ms(modbus_gap_ms(mb_baud));
        }
        g_mb_rx.discard_until(now);  // nothing half-received survives a role change
        mb_was_busy = false;
        active = want;
      }
    }

    if (due(last_ctl, CONTROLLER_TICK_MS, now)) {
      // Without CAN the board can't charge, so a faulted relay stays open rather than run a generator unloaded.
      // E17 (battery reading implausible) counts as a fault here too: the session has been ended under it.
      // Source mode selects Panel, BMS (Pylontech, EG4, JK, JK-PB) or Modbus limits.
      const SourceMode mode = SourceMode(settings.get(HR_SOURCE_MODE));
      if (mode != last_mode) {
        g_regs.forget_modbus_stop();  // a mode change forgets a source's stop (spec §1)
        g_ctl.end_session();          // and ends the old source's session
        last_mode = mode;
      }
      const Limits lim = active                        ? active->limits(settings, now)
                         : mode == SourceMode::MODBUS ? g_regs.modbus_limits(settings, now)
                                                      : panel_limits(settings);
      st = g_ctl.tick(settings, lim, adc_v, now);
      const bool faulted = st.state == charge::State::FAULT || st.error == E_BATTERY_SENSE || !can;
      relay = g_relay.update(RelayMode(settings.get(HR_RELAY_MODE)), st.session_active, st.paused, faulted,
                             g_regs.relay_command(), now);
      hw::pin_write(board::RELAY, relay);
    }

    while (now - last_second >= 1000) {
      last_second += 1000;
      uptime_s++;
    }

    if (rs485) {
      g_tel.relay_closed = relay;
      g_tel.adc_v = adc_v;
      g_tel.uptime_s = uptime_s;
      g_tel.now_ms = now;
      g_tel.menu_open = g_ui.in_menu();  // M7: settings writes are refused while the menu holds its copy

      // A send that never finishes mustn't hold DE high, or block the save, a line change or a reboot, for good.
      if (hw::rs485_tx_busy() && now - mb_tx_start > MB_TX_TIMEOUT_MS) {
        hw::rs485_abort_tx();
        g_rs485_tx_aborts++;
      }
      const bool busy = hw::rs485_tx_busy();
      uint8_t rx[64];
      if (active) {
        // The BMS poller: the board is the master and never answers Modbus. No commands, baud changes or reboot here.
        for (size_t n; (n = hw::rs485_read(rx, sizeof rx)) > 0;) active->push(rx, n, now);
        uint8_t req[POLL_REQ_MAX];
        const size_t n = busy ? 0 : active->poll(now, req);
        if (n > 0 && hw::rs485_send(req, n)) mb_tx_start = now;
      } else {
        for (size_t n; (n = hw::rs485_read(rx, sizeof rx)) > 0;) g_mb_rx.push(rx, n, now);
        // M4: drop whatever is buffered once a reply has gone out (an echo, had the transceiver one), but nothing after
        // it. The bench transceiver doesn't echo, and a master may start its next request within a few ms of our last
        // bit: a floor a gap past tx_done dropped about one fast back-to-back request in seven.
        if (mb_was_busy && !busy) g_mb_rx.discard_until(now);
        mb_was_busy = busy;

        if (!busy) {
          // M8: line changes (by Modbus or the menu) and a reboot wait until no reply is going out. The address is
          // read per frame, so it needs nothing here.
          if (mb_reboot) {
            if (dirty) g_store.save(settings);  // a write just before the reboot would otherwise be lost
            if (g_ctl.paused()) hw::set_backup_word(PAUSE_REBOOT_MAGIC);  // restore_pause keeps it across this reset
            hw::system_reset();  // the bootloader's 1.5 s window follows (oem_update.py --modbus-reboot)
          }
          const uint32_t want_baud = modbus_baud(settings);
          if (want_baud != mb_baud && set_line_baud(want_baud)) {
            mb_baud = want_baud;
            g_mb_rx.set_gap_ms(modbus_gap_ms(mb_baud));
            g_mb_rx.discard_until(now);  // a frame half-received at the old baud can't be completed at the new one
          }

          const uint8_t *req;
          const size_t len = g_mb_rx.poll(now, req);
          if (len > 0) {
            const size_t out = modbus_handle(uint8_t(settings.get(HR_MODBUS_ADDR)), req, len, g_mb_resp, g_regs);
            const Commands c = g_regs.take_commands();
            if (c.settings_changed) {
              dirty = true;
              changed_ms = now;
            }
            if (c.pause_set) {
              g_ctl.set_paused(c.pause);
              hw::set_backup_word(pause_word(c.pause));
            }
            if (c.clear_fault) g_ctl.clear_fault();
            if (c.charge_now) g_ctl.charge_now();
            if (c.diag_stop) g_ctl.stop_diag();
            if (c.diag_start) {
              g_ctl.start_diag(float(c.diag_v100) / 100.0f, float(c.diag_a10) / 10.0f, uint8_t(c.diag_mask),
                               c.diag_seconds, now);
            }
            mb_reboot = c.reboot;
            if (out > 0 && hw::rs485_send(g_mb_resp, out)) {
              mb_was_busy = true;
              mb_tx_start = now;
            }
          }
        }
      }
    }

    if (due(last_lcd, LCD_MS, now)) {
      Status shown = st;  // an external SOC (HR 104, kept fresh by HR 100) fills in for display only
      if (shown.soc < 0) shown.soc = g_regs.external_soc(now);
      const Frame f = g_ui.render(shown, now);
      if (lcd) hw::slcd_write(f.com);
    }

    // Not during a reply: the flash erase masks interrupts, which would stall the TX interrupt mid-frame.
    if (dirty && now - changed_ms >= SAVE_DELAY_MS && !hw::rs485_tx_busy()) {
      // A failed save isn't retried until the settings change again: each retry could erase a page.
      g_store.save(settings);
      dirty = false;
    }

    hw::fwdgt_feed();
  }
}
