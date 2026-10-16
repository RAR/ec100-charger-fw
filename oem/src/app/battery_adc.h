// oem/src/app/battery_adc.h: PC0 samples to volts. Stock (08007630 / 08007668): sort 10 samples, average the middle
// 8, scale by 0.16923 to 0.1 V units. We keep volts as a float and apply the HR 14 trim (per mille).
#pragma once
#include <cstdint>

namespace oem {

constexpr int ADC_SAMPLES = 10;
constexpr float ADC_VOLTS_PER_COUNT = 0.016923f;  // 4095 counts = 69.3 V

inline uint16_t adc_trimmed_mean(const uint16_t (&in)[ADC_SAMPLES]) {
  uint16_t a[ADC_SAMPLES];
  for (int i = 0; i < ADC_SAMPLES; i++) a[i] = in[i];
  for (int i = 0; i < ADC_SAMPLES - 1; i++) {
    for (int j = 0; j < ADC_SAMPLES - 1 - i; j++) {
      if (a[j] > a[j + 1]) {
        const uint16_t t = a[j];
        a[j] = a[j + 1];
        a[j + 1] = t;
      }
    }
  }
  uint32_t sum = 0;
  for (int i = 1; i < ADC_SAMPLES - 1; i++) sum += a[i];
  return uint16_t(sum >> 3);
}

inline float adc_volts(uint16_t counts, int32_t trim_permille) {
  return float(counts) * ADC_VOLTS_PER_COUNT * (1.0f + float(trim_permille) / 1000.0f);
}

}  // namespace oem
