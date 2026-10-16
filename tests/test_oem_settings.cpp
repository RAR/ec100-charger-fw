#include <cstring>

#include "app/settings_store.h"
#include "check.h"

using namespace oem;

// 2 KB of RAM standing in for the two settings pages. fail_after_dw > 0 makes a program call stop after that many
// doublewords (a torn write); erase_fails makes erase_page fail.
class FakeFlash : public FlashIo {
 public:
  FakeFlash() { std::memset(mem, 0xFF, sizeof(mem)); }
  const uint8_t *read(uint32_t addr) override { return mem + (addr - SETTINGS_PAGE[0]); }
  bool erase_page(uint32_t addr) override {
    erases++;
    if (erase_fails || dead) return false;
    uint8_t *page = mem + (addr - SETTINGS_PAGE[0]);
    for (uint32_t i = 0; i < SETTINGS_PAGE_BYTES; i += 8) {
      if (budget == 0) {  // power lost mid-erase: this doubleword is half erased, the rest still hold their old bytes
        dead = true;
        std::memset(page + i, 0xFF, 4);
        for (int b = 4; b < 8; b++) page[i + b] |= 0x55;
        return false;
      }
      if (budget > 0) budget--;
      std::memset(page + i, 0xFF, 8);
    }
    return true;
  }
  bool program(uint32_t addr, const uint8_t *data, uint32_t len) override {
    for (uint32_t i = 0; i < len; i += 8) {
      if (dead) return false;
      if (budget == 0) {  // power lost: nothing after this point reaches the flash, not even the store's fallback
        dead = true;
        return false;
      }
      if (budget > 0) budget--;
      if (fail_after_dw > 0 && dw_written >= fail_after_dw) return false;
      uint8_t *p = mem + (addr - SETTINGS_PAGE[0]) + i;
      for (int b = 0; b < 8; b++) p[b] &= data[i + b];  // flash can only clear bits
      dw_written++;
    }
    return true;
  }
  uint8_t mem[2 * SETTINGS_PAGE_BYTES];
  int erases = 0;
  int dw_written = 0;
  int fail_after_dw = 0;
  bool erase_fails = false;
  // Power-cut model: budget = doublewords (programmed or erased) left before power is lost, -1 = unlimited. Once dead,
  // every call does nothing, so a store that falls back after the cut cannot change the flash.
  int budget = -1;
  bool dead = false;
  void power_cycle() {
    budget = -1;
    dead = false;
  }
};

static void test_crc_check_value() {
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK(crc16_modbus(s, 9) == 0x4B37);  // same check value as tools/oem_image.py
}

static void test_defaults_and_ranges() {
  const Settings s = default_settings();
  CHECK(s.get(HR_MAX_CURRENT) == 10);
  CHECK(s.get(HR_CHARGE_V) == 5760);
  CHECK(s.get(HR_START_SOC) == 20 && s.get(HR_STOP_SOC) == 100);
  CHECK(s.get(HR_START_V) == 4200 && s.get(HR_STOP_V) == 5760);
  CHECK(s.get(HR_FALLBACK_A) == 10 && s.get(HR_BACKLIGHT_S) == 120);
  CHECK(s.get(HR_MODULE_FALLBACK_V) == 5400 && s.get(HR_MODULE_FALLBACK_A) == 5);
  Settings c = s;
  apply_clamps(c);
  CHECK(c == s);  // the defaults already satisfy every clamp
}

static void test_source_mode_range() {
  Settings s = default_settings();
  s.set(HR_SOURCE_MODE, 4);
  CHECK(!violates_clamps(s));  // JK is valid
  s.set(HR_SOURCE_MODE, 5);
  CHECK(!violates_clamps(s));  // JK-PB is valid
  s.set(HR_SOURCE_MODE, 6);
  CHECK(violates_clamps(s));
  apply_clamps(s);
  CHECK(s.get(HR_SOURCE_MODE) == 5);
}

static void test_clamps() {
  Settings s = default_settings();
  s.set(HR_CHARGE_V, 5000);  // Stop V follows Charge V down, Start V stays 5.0 V below Stop V
  s.set(HR_START_V, 5280);
  apply_clamps(s);
  CHECK(s.get(HR_STOP_V) == 5000);
  CHECK(s.get(HR_START_V) == 4500);
  s.set(HR_START_SOC, 80);
  s.set(HR_STOP_SOC, 80);
  apply_clamps(s);
  CHECK(s.get(HR_START_SOC) == 79);
  s.set(HR_MAX_CURRENT, 500);
  s.set(HR_ADC_TRIM, -80);
  s.set(HR_CELL_RESUME_MV, 3550);
  s.set(HR_MAX_CELL_MV, 3500);
  apply_clamps(s);
  CHECK(s.get(HR_MAX_CURRENT) == 100);
  CHECK(s.get(HR_ADC_TRIM) == -50);
  CHECK(s.get(HR_CELL_RESUME_MV) == 3499);
}

static void test_record_round_trip() {
  Settings s = default_settings();
  s.set(HR_ADC_TRIM, -7);
  uint8_t rec[RECORD_BYTES];
  encode_record(s, 0x01020304, rec);
  CHECK(rec[0] == 'O' && rec[1] == 'S' && rec[2] == 1 && rec[3] == 17);
  CHECK(rec[4] == 0x04 && rec[7] == 0x01);
  Settings d{};
  uint32_t seq = 0;
  CHECK(decode_record(rec, d, seq));
  CHECK(d == s && seq == 0x01020304 && d.get(HR_ADC_TRIM) == -7);
  rec[20] ^= 1;
  CHECK(!decode_record(rec, d, seq));
}

// Re-signs a record after its count or registers were edited.
static void resign(uint8_t *rec) {
  const uint16_t crc = crc16_modbus(rec, RECORD_BYTES - 2);
  rec[RECORD_BYTES - 2] = uint8_t(crc);
  rec[RECORD_BYTES - 1] = uint8_t(crc >> 8);
}

static void test_record_with_other_register_count() {
  Settings s = default_settings();
  s.set(HR_MAX_CURRENT, 75);
  s.set(HR_CHARGE_V, 5520);
  s.set(HR_SOURCE_MODE, 2);
  uint8_t rec[RECORD_BYTES];
  encode_record(s, 5, rec);
  Settings d{};
  uint32_t seq = 0;
  // An older firmware's record with fewer registers: those are kept, the rest take their defaults.
  rec[3] = 16;
  rec[8 + 2 * 16] = 0x55;  // HR 16's bytes are ignored
  resign(rec);
  CHECK(decode_record(rec, d, seq) && seq == 5);
  CHECK(d.get(HR_MAX_CURRENT) == 75 && d.get(HR_CHARGE_V) == 5520 && d.get(HR_SOURCE_MODE) == 2);
  CHECK(d.get(Hr(16)) == default_settings().get(Hr(16)));
  // A newer firmware's record with more registers: the first HR_COUNT are kept.
  encode_record(s, 6, rec);
  rec[3] = RECORD_MAX_REGS;
  resign(rec);
  CHECK(decode_record(rec, d, seq) && d == s);
  // No registers, or more than fit: refused.
  rec[3] = 0;
  resign(rec);
  CHECK(!decode_record(rec, d, seq));
  rec[3] = RECORD_MAX_REGS + 1;
  resign(rec);
  CHECK(!decode_record(rec, d, seq));
}

static void test_empty_flash_loads_nothing_then_saves() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s;
  CHECK(!st.load(s));
  Settings want = default_settings();
  want.set(HR_MAX_CURRENT, 42);
  CHECK(st.save(want));
  CHECK(f.erases == 1 && st.active_page() == 0 && st.seq() == 1);
  SettingsStore again(f);
  Settings got{};
  CHECK(again.load(got) && got == want && again.seq() == 1);
}

static void test_appends_then_rotates() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  for (uint32_t i = 0; i < RECORD_SLOTS; i++) {
    s.set(HR_MAX_CURRENT, int32_t(2 + i));
    CHECK(st.save(s));
  }
  CHECK(f.erases == 1 && st.active_page() == 0);  // 21 records fill page 0 after one erase
  s.set(HR_MAX_CURRENT, 77);
  CHECK(st.save(s));
  CHECK(f.erases == 2 && st.active_page() == 1);  // page 1 erased, record 22 in its first slot
  SettingsStore again(f);
  Settings got{};
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 77 && again.seq() == 22);
  for (uint32_t i = 0; i < RECORD_SLOTS; i++) CHECK(st.save(s));
  CHECK(st.active_page() == 0 && f.erases == 3);  // back to page 0, never erasing the page with the newest record
}

static void test_torn_write_keeps_previous() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  s.set(HR_MAX_CURRENT, 30);
  CHECK(st.save(s));
  f.fail_after_dw = f.dw_written + 3;  // half of the next record, then power is lost
  s.set(HR_MAX_CURRENT, 31);
  st.save(s);
  SettingsStore after(f);  // reboot
  Settings got{};
  CHECK(after.load(got) && got.get(HR_MAX_CURRENT) == 30);
  f.fail_after_dw = 0;
  s.set(HR_MAX_CURRENT, 32);
  CHECK(after.save(s));
  SettingsStore again(f);
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 32 && again.active_page() == 0);
  // the new record went after the torn one, not into it
  Settings probe{};
  uint32_t seq;
  CHECK(!decode_record(f.mem + RECORD_BYTES, probe, seq));
  CHECK(decode_record(f.mem + 2 * RECORD_BYTES, probe, seq));
}

static void test_failed_write_moves_to_other_page() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  s.set(HR_MAX_CURRENT, 10);
  CHECK(st.save(s));
  f.fail_after_dw = f.dw_written + 1;
  s.set(HR_MAX_CURRENT, 11);
  CHECK(!st.save(s));  // the active-page write fails, and so does the other page (the fake stays broken)
  f.fail_after_dw = 0;
  s.set(HR_MAX_CURRENT, 55);
  CHECK(st.save(s));
  Settings got{};
  SettingsStore again(f);
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 55);
  // Page 0 now holds slot 0 (10), a torn slot 1 and slot 2 (55): 18 free slots. With erases failing, those succeed
  // and the rest fail; the last value that was saved successfully is what a reboot returns.
  f.erase_fails = true;
  int32_t last_ok = 55;
  for (int i = 0; i < int(RECORD_SLOTS); i++) {
    s.set(HR_MAX_CURRENT, 20 + i);
    const bool ok = st.save(s);
    CHECK(ok == (i < 18));
    if (ok) last_ok = 20 + i;
  }
  CHECK(last_ok == 37);
  SettingsStore fresh(f);
  CHECK(fresh.load(got) && got.get(HR_MAX_CURRENT) == last_ok && fresh.seq() == 20);
}

static void test_load_clamps_out_of_range() {
  FakeFlash f;
  Settings s = default_settings();
  s.set(HR_MAX_CURRENT, 300);
  uint8_t rec[RECORD_BYTES];
  encode_record(s, 9, rec);
  f.program(SETTINGS_PAGE[1], rec, RECORD_BYTES);
  SettingsStore st(f);
  Settings got{};
  CHECK(st.load(got) && got.get(HR_MAX_CURRENT) == 100 && st.active_page() == 1 && st.seq() == 9);
}

// Value seen after a reboot, or -1 if no record loads.
static int32_t reboot_value(FakeFlash &f) {
  f.power_cycle();
  SettingsStore st(f);
  Settings got{};
  return st.load(got) ? got.get(HR_MAX_CURRENT) : -1;
}

// After a cut and reboot, the store must accept the next save and return it.
static void check_next_save_works(FakeFlash &f) {
  f.power_cycle();
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  s.set(HR_MAX_CURRENT, 99);
  CHECK(st.save(s));
  SettingsStore again(f);
  Settings got{};
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 99);
}

static void test_power_cut_sweep_append() {
  const int record_dw = int(RECORD_BYTES / 8);
  for (int k = 0; k <= record_dw; k++) {
    FakeFlash f;
    SettingsStore st(f);
    Settings s = default_settings();
    st.load(s);
    s.set(HR_MAX_CURRENT, 30);
    CHECK(st.save(s));
    s.set(HR_MAX_CURRENT, 31);
    f.budget = k;
    const bool ok = st.save(s);
    CHECK(ok == (k == record_dw));
    CHECK(reboot_value(f) == (k == record_dw ? 31 : 30));  // new value only once the CRC doubleword is in
    check_next_save_works(f);
  }
}

// Fill page 0, then page 1, so the next save rotates into page 0, which still holds old records to erase.
static void fill_both_pages(FakeFlash &f, SettingsStore &st, Settings &s) {
  st.load(s);
  for (uint32_t i = 0; i < 2 * RECORD_SLOTS; i++) {
    s.set(HR_MAX_CURRENT, int32_t(2 + i));
    CHECK(st.save(s));
  }
  CHECK(st.active_page() == 1 && f.erases == 2);
}

static void test_power_cut_sweep_rotation() {
  const int erase_dw = int(SETTINGS_PAGE_BYTES / 8);
  const int total = erase_dw + int(RECORD_BYTES / 8);
  for (int k = 0; k <= total; k++) {
    FakeFlash f;
    SettingsStore st(f);
    Settings s = default_settings();
    fill_both_pages(f, st, s);
    const int32_t old_value = int32_t(2 + 2 * RECORD_SLOTS - 1);
    s.set(HR_MAX_CURRENT, 90);
    f.budget = k;
    const bool ok = st.save(s);
    CHECK(ok == (k == total));
    CHECK(reboot_value(f) == (k == total ? 90 : old_value));
    check_next_save_works(f);
  }
}

static void test_power_cut_sweep_rotation_into_fresh_page() {
  const int total = int(SETTINGS_PAGE_BYTES / 8 + RECORD_BYTES / 8);
  for (int k = 0; k <= total; k++) {  // first rotation: the target page was never written
    FakeFlash f;
    SettingsStore st(f);
    Settings s = default_settings();
    st.load(s);
    for (uint32_t i = 0; i < RECORD_SLOTS; i++) {
      s.set(HR_MAX_CURRENT, int32_t(2 + i));
      CHECK(st.save(s));
    }
    s.set(HR_MAX_CURRENT, 90);
    f.budget = k;
    st.save(s);
    CHECK(reboot_value(f) == (k == total ? 90 : int32_t(2 + RECORD_SLOTS - 1)));
    check_next_save_works(f);
  }
}

static void test_save_without_load_keeps_old_record() {
  FakeFlash f;
  {
    SettingsStore first(f);
    Settings s = default_settings();
    first.load(s);
    s.set(HR_MAX_CURRENT, 40);
    CHECK(first.save(s));
  }
  Settings s = default_settings();
  s.set(HR_MAX_CURRENT, 41);
  {  // a cut mid-save must still leave the old record
    FakeFlash g = f;
    SettingsStore blind(g);
    g.budget = 3;
    CHECK(!blind.save(s));  // no load() first
    CHECK(reboot_value(g) == 40);
  }
  SettingsStore blind(f);
  CHECK(blind.save(s));
  CHECK(f.erases == 1);  // appended into page 0, no further erase
  SettingsStore again(f);
  Settings got{};
  CHECK(again.load(got) && got.get(HR_MAX_CURRENT) == 41 && again.seq() == 2);
}

static void test_newer_bad_crc_falls_back() {
  FakeFlash f;
  SettingsStore st(f);
  Settings s = default_settings();
  st.load(s);
  s.set(HR_MAX_CURRENT, 50);
  CHECK(st.save(s));
  s.set(HR_MAX_CURRENT, 51);
  CHECK(st.save(s));
  f.mem[RECORD_BYTES + 20] ^= 1;  // corrupt the newer record
  SettingsStore after(f);
  Settings got{};
  CHECK(after.load(got) && got.get(HR_MAX_CURRENT) == 50 && after.seq() == 1);
}

int main() {
  test_crc_check_value();
  test_defaults_and_ranges();
  test_clamps();
  test_source_mode_range();
  test_record_round_trip();
  test_record_with_other_register_count();
  test_empty_flash_loads_nothing_then_saves();
  test_appends_then_rotates();
  test_torn_write_keeps_previous();
  test_failed_write_moves_to_other_page();
  test_load_clamps_out_of_range();
  test_power_cut_sweep_append();
  test_power_cut_sweep_rotation();
  test_power_cut_sweep_rotation_into_fresh_page();
  test_save_without_load_keeps_old_record();
  test_newer_bad_crc_falls_back();
  return test_result("test_oem_settings");
}
