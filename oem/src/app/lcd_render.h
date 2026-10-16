// oem/src/app/lcd_render.h: the stock glass (docs/history/notes/oem-init-sequences.md §4). Bit s of com[k] is SEGs x
// COMk; each digit is one SEG column with glyph bit k on COMk, and bit 0 of every glyph is clear because COM0 of a
// digit column is a separate label or decimal point.
#pragma once
#include <cstdint>

namespace oem {

struct Frame {
  uint32_t com[8] = {};
};

enum class Field : uint8_t { AC_V, CURRENT, POWER, SOC, BATTERY_V };

constexpr uint8_t GLYPH_BLANK = 10;
constexpr uint8_t GLYPH_DASH = 11;
constexpr uint8_t GLYPH_E = 12;

// Font B has no 'E' (its entry 12 is blank): only the SOC field, which uses font A, shows error codes.
constexpr uint8_t FONT_A[13] = {0xEE, 0x24, 0xD6, 0xB6, 0x3C, 0xBA, 0xFA, 0x26, 0xFE, 0xBE, 0x00, 0x10, 0xDA};
constexpr uint8_t FONT_B[13] = {0xEE, 0x48, 0xD6, 0xDA, 0x78, 0xBA, 0xBE, 0xC8, 0xFE, 0xFA, 0x00, 0x10, 0x00};

struct FieldDef {
  uint8_t segs[4];  // most significant digit first
  uint8_t digits;
  bool font_b;
};

constexpr FieldDef FIELDS[5] = {
    {{26, 27, 17, 0}, 3, true},    // AC_V
    {{22, 4, 3, 0}, 3, false},     // CURRENT
    {{11, 13, 14, 23}, 4, false},  // POWER
    {{9, 8, 7, 0}, 3, false},      // SOC (or "E" + 2 digits)
    {{31, 30, 12, 0}, 3, false},   // BATTERY_V, 0.1 V; the decimal point is a fixed COM0 label
};

constexpr uint8_t LCD_SEGS[19] = {3, 4, 7, 8, 9, 11, 12, 13, 14, 16, 17, 22, 23, 25, 26, 27, 28, 30, 31};

inline void put_segment(Frame &f, int seg, int com) { f.com[com] |= 1u << seg; }

inline void put_fixed_labels(Frame &f) {
  f.com[0] |= 0xDCC27B18u;
  f.com[1] |= 0x10000000u;
}

// Out-of-range positions and glyphs draw nothing.
inline void put_glyph(Frame &f, Field field, int pos, uint8_t glyph) {
  const FieldDef &d = FIELDS[int(field)];
  if (pos < 0 || pos >= d.digits || glyph > GLYPH_E) return;
  const uint8_t bits = (d.font_b ? FONT_B : FONT_A)[glyph];
  for (int k = 1; k < 8; k++) {
    if (bits & (1u << k)) put_segment(f, d.segs[pos], k);
  }
}

inline void put_dashes(Frame &f, Field field) {
  for (int pos = 0; pos < FIELDS[int(field)].digits; pos++) put_glyph(f, field, pos, GLYPH_DASH);
}

// Right-aligned, leading zeros blanked down to min_digits; dashes if it doesn't fit.
inline void put_number(Frame &f, Field field, uint32_t value, int min_digits = 1) {
  const int n = FIELDS[int(field)].digits;
  uint32_t limit = 1;
  for (int i = 0; i < n; i++) limit *= 10;
  if (value >= limit) {
    put_dashes(f, field);
    return;
  }
  for (int pos = n - 1, i = 0; pos >= 0; pos--, i++) {
    if (i >= min_digits && value == 0) break;
    put_glyph(f, field, pos, uint8_t(value % 10));
    value /= 10;
  }
}

// "E" and two digits in the SOC field, with its COM0 labels (SEG8/SEG9) off, as stock does for error codes.
inline void put_error(Frame &f, uint8_t code) {
  put_glyph(f, Field::SOC, 0, GLYPH_E);
  put_glyph(f, Field::SOC, 1, uint8_t(code / 10 % 10));
  put_glyph(f, Field::SOC, 2, uint8_t(code % 10));
  f.com[0] &= ~((1u << 8) | (1u << 9));
}

// Eight bars on SEG16, COM7 first, lit above the stock thresholds. While charging, stock animates them: only the
// first `step` bars are drawn, step counting up every 300 ms.
inline void put_soc_bars(Frame &f, int soc, int step = 8) {
  static constexpr uint8_t THRESHOLDS[8] = {0, 13, 27, 39, 51, 63, 75, 87};
  for (int i = 0; i < 8; i++) {
    if (soc > THRESHOLDS[i] && step > i) put_segment(f, 16, 7 - i);
  }
}

}  // namespace oem
