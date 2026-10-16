// oem/src/hw/adc.cpp: docs/history/notes/oem-adc-flash-sequences.md §4, the polled form of the stock init (0800816c):
// same clock, sample time, charge pulse width and calibration, one regular rank, single conversions by software
// trigger.
#include "hw/adc.h"

#include "hw/gpio.h"

namespace hw {
namespace {

constexpr uint32_t ADCON = 1u << 0, CLB = 1u << 2, RSTCLB = 1u << 3;
constexpr uint32_t ETSRC_SW = 7u << 17, ETERC = 1u << 20, SWRCST = 1u << 22;
constexpr uint32_t EOC = 1u << 1;

bool wait_clear(uint32_t addr, uint32_t bit) {
  for (uint32_t t = 0; t < 0x10000; t++) {
    if (!(reg(addr) & bit)) return true;
  }
  return false;
}

}  // namespace

bool adc_init() {
  reg(RCU_APB2EN) |= 1u << 9;   // ADCEN
  reg(RCU_AHBEN) |= 1u << 19;   // GPIOC
  pin_config({GPIOC, 0}, Mode::ANALOG, Pull::NONE);
  rmw(RCU_CFG0, 0xC000u, 0x4000u);           // ADC prescaler bits, as stock rcu_adc_clock_config(5)
  rmw(RCU_CFG2, 0xC0000100u, 0x40000100u);   // ADCSEL + ADCPSC2: CK_ADC = APB2 / 12 = 5 MHz
  reg(ADC_RSQ0) = 0;                         // one conversion
  reg(ADC_RSQ2) = 10;                        // rank 0 = channel 10
  rmw(ADC_SAMPT0, 0x7u, 0x4u);               // channel 10: 41.5 cycles
  reg(ADC_CTL0) = 0;                         // no scan; 12-bit
  reg(ADC_CTL1) = ETSRC_SW | ETERC;          // software trigger, single, right-aligned, no DMA
  reg(ADC_CCTL) = 0x140;
  reg(ADC_CTL1) |= ADCON;
  for (volatile uint32_t i = 0; i < 600; i = i + 1) {  // >= 5 us start-up (stock waits 1 ms)
  }
  reg(ADC_CTL1) |= RSTCLB;
  if (!wait_clear(ADC_CTL1, RSTCLB)) return false;
  reg(ADC_CTL1) |= CLB;
  return wait_clear(ADC_CTL1, CLB);
}

bool adc_read(uint16_t &out) {
  reg(ADC_STAT) &= ~EOC;
  reg(ADC_CTL1) |= SWRCST;
  for (uint32_t t = 0; t < 0x4000; t++) {
    if (reg(ADC_STAT) & EOC) {
      out = uint16_t(reg(ADC_RDATA) & 0xFFFu);
      return true;
    }
  }
  return false;
}

}  // namespace hw
