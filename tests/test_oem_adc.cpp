#include "app/battery_adc.h"
#include "check.h"

using namespace oem;

static void test_trimmed_mean_drops_min_and_max() {
  const uint16_t s[ADC_SAMPLES] = {3000, 3100, 3100, 3100, 3100, 3100, 3100, 3100, 3100, 4095};
  CHECK(adc_trimmed_mean(s) == 3100);
  const uint16_t r[ADC_SAMPLES] = {9, 1, 8, 2, 7, 3, 6, 4, 5, 0};
  CHECK(adc_trimmed_mean(r) == (1 + 2 + 3 + 4 + 5 + 6 + 7 + 8) / 8);
}

static void test_scale_and_trim() {
  CHECK_NEAR(adc_volts(4095, 0), 69.30, 0.01);  // stock full scale: 4095 x 0.16923 = 693.0 (0.1 V)
  CHECK_NEAR(adc_volts(3073, 0), 52.00, 0.01);
  CHECK_NEAR(adc_volts(3073, 10), 52.52, 0.01);   // +1.0 %
  CHECK_NEAR(adc_volts(3073, -50), 49.40, 0.01);  // -5.0 %
}

int main() {
  test_trimmed_mean_drops_min_and_max();
  test_scale_and_trim();
  return test_result("test_oem_adc");
}
