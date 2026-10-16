// oem/src/hw/adc.h: PC0 = ADC_IN10, the 48 V sense. Polled single conversions; stock free-runs a 10-sample DMA scan
// instead (docs/history/notes/oem-adc-flash-sequences.md §2-§4).
#pragma once
#include <cstdint>

namespace hw {

bool adc_init();  // stock clock (APB2 / 12 = 5 MHz), 41.5-cycle sampling, calibration; false if calibration hangs
// One 12-bit conversion of channel 10 (about 11 us). False on a timeout (out is left alone).
bool adc_read(uint16_t &out);

}  // namespace hw
