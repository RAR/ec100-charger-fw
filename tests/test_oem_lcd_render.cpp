#include <initializer_list>

#include "app/lcd_render.h"
#include "check.h"

using namespace oem;

static void test_matches_stock_zero_frame() {
  Frame f;
  put_fixed_labels(f);
  put_dashes(f, Field::AC_V);
  put_number(f, Field::CURRENT, 0);
  put_number(f, Field::POWER, 0);
  put_number(f, Field::SOC, 0);
  put_number(f, Field::BATTERY_V, 0, 2);
  put_segment(f, 28, 2);
  put_segment(f, 28, 6);
  const uint32_t want[8] = {0xDCC27B18, 0x50801088, 0x50801088, 0x40801088,
                            0x0C020000, 0x40801088, 0x50801088, 0x40801088};
  for (int k = 0; k < 8; k++) CHECK(f.com[k] == want[k]);
}

static void test_number_places_digits_right_aligned() {
  Frame f;
  put_number(f, Field::POWER, 1234);  // SEG 11, 13, 14, 23 = 1, 2, 3, 4 in font A
  CHECK(f.com[2] & (1u << 11));       // '1' = 0x24: COM2 and COM5
  CHECK(f.com[5] & (1u << 11));
  CHECK(!(f.com[1] & (1u << 11)));
  CHECK(f.com[4] & (1u << 23));       // '4' = 0x3C: COM2..COM5
}

static void test_overflow_shows_dashes() {
  Frame f;
  put_number(f, Field::CURRENT, 1000);
  for (int seg : {22, 4, 3}) CHECK(f.com[4] & (1u << seg));
}

static void test_ac_uses_font_b() {
  Frame f;
  put_number(f, Field::AC_V, 1);  // font B '1' = 0x48: COM3 and COM6 on SEG17
  CHECK(f.com[3] & (1u << 17));
  CHECK(f.com[6] & (1u << 17));
  CHECK(!(f.com[2] & (1u << 17)));
}

static void test_soc_bars() {
  Frame f;
  put_soc_bars(f, 50);  // > 0, 13, 27, 39 → bars 0..3 = SEG16 COM7..COM4
  for (int k = 4; k <= 7; k++) CHECK(f.com[k] & (1u << 16));
  for (int k = 0; k <= 3; k++) CHECK(!(f.com[k] & (1u << 16)));
}

static void test_battery_v_leading_zero() {
  Frame f;
  put_number(f, Field::BATTERY_V, 5, 2);  // "0.5": SEG30 = '0' (0xEE), SEG12 = '5', SEG31 blank
  CHECK(f.com[1] & (1u << 30));
  CHECK(f.com[1] & (1u << 12));  // '5' = 0xBA: COM1, 3, 4, 5, 7
  CHECK(f.com[7] & (1u << 12));
  CHECK(!(f.com[2] & (1u << 12)));
  for (int k = 0; k < 8; k++) CHECK(!(f.com[k] & (1u << 31)));
}

static void test_power_limits() {
  Frame f;
  put_number(f, Field::POWER, 9999);  // '9' = 0xBE: COM1..5 and COM7, not COM6
  for (int seg : {11, 13, 14, 23}) {
    CHECK(f.com[1] & (1u << seg));
    CHECK(f.com[7] & (1u << seg));
    CHECK(!(f.com[6] & (1u << seg)));
  }
  Frame g;
  put_number(g, Field::POWER, 10000);  // dashes: '-' = 0x10, COM4 only
  for (int seg : {11, 13, 14, 23}) {
    CHECK(g.com[4] & (1u << seg));
    CHECK(!(g.com[1] & (1u << seg)));
  }
}

static void test_soc_bar_thresholds() {
  auto bars = [](int soc) {
    Frame f;
    put_soc_bars(f, soc);
    int n = 0;
    for (int k = 0; k < 8; k++) n += (f.com[k] >> 16) & 1u;
    return n;
  };
  CHECK(bars(0) == 0);
  Frame f13;
  put_soc_bars(f13, 13);
  CHECK((f13.com[7] & (1u << 16)) && bars(13) == 1);
  CHECK(bars(14) == 2);
  CHECK(bars(100) == 8);
}

static void test_glyph_bounds() {
  Frame f;
  put_glyph(f, Field::SOC, 3, 8);         // SOC has 3 digits
  put_glyph(f, Field::CURRENT, -1, 8);
  put_glyph(f, Field::CURRENT, 0, 13);    // no glyph 13
  for (int k = 0; k < 8; k++) CHECK(f.com[k] == 0);
  put_glyph(f, Field::POWER, 3, 1);       // the power field's 4th digit is real
  CHECK(f.com[2] & (1u << 23));
}

static void test_error_code() {
  Frame f;
  put_fixed_labels(f);
  put_error(f, 13);  // SEG9 'E' (0xDA: COM1, 3, 4, 6, 7), SEG8 '1', SEG7 '3'
  for (int k : {1, 3, 4, 6, 7}) CHECK(f.com[k] & (1u << 9));
  CHECK(!(f.com[2] & (1u << 9)) && !(f.com[5] & (1u << 9)));
  CHECK((f.com[2] & (1u << 8)) && (f.com[5] & (1u << 8)) && !(f.com[1] & (1u << 8)));  // '1' = 0x24
  CHECK((f.com[1] & (1u << 7)) && (f.com[7] & (1u << 7)));                           // '3' = 0xB6
  CHECK(!(f.com[0] & (1u << 8)) && !(f.com[0] & (1u << 9)));
  CHECK(f.com[0] & (1u << 28));  // other labels stay
}

int main() {
  test_glyph_bounds();
  test_error_code();
  test_matches_stock_zero_frame();
  test_number_places_digits_right_aligned();
  test_overflow_shows_dashes();
  test_ac_uses_font_b();
  test_soc_bars();
  test_battery_v_leading_zero();
  test_power_limits();
  test_soc_bar_thresholds();
  return test_result("test_oem_lcd_render");
}
