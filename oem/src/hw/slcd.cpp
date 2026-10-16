// oem/src/hw/slcd.cpp: the stock v1 sequence (08008588 / 0800cafc): COM and SEG pins on AF3 with pull-downs, VSLCD
// on PD6, the SLCD clocked from IRC32K through the backup domain, 1/8 duty, 1/4 bias, external bias voltage.
#include "hw/slcd.h"
#include "hw/gpio.h"

namespace hw {
namespace {

constexpr Pin LCD_PINS[] = {
    // COM0..COM7
    {GPIOA, 8}, {GPIOA, 9}, {GPIOA, 10}, {GPIOB, 9}, {GPIOC, 10}, {GPIOC, 11}, {GPIOC, 12}, {GPIOD, 2},
    // SEG3, 4, 17 | 7, 8, 9, 16, 11, 12, 13, 14 | 22, 23, 25, 26, 27 | 28, 30, 31
    {GPIOA, 6}, {GPIOA, 7}, {GPIOA, 15}, {GPIOB, 3}, {GPIOB, 4}, {GPIOB, 5}, {GPIOB, 8}, {GPIOB, 11},
    {GPIOB, 12}, {GPIOB, 13}, {GPIOB, 14}, {GPIOC, 4}, {GPIOC, 5}, {GPIOC, 7}, {GPIOC, 8}, {GPIOC, 9},
    {GPIOD, 4}, {GPIOD, 8}, {GPIOD, 9},
};

bool wait_set(uint32_t addr, uint32_t bit) {
  for (uint32_t t = 0; t < 0x100000; t++) {
    if (reg(addr) & bit) return true;
  }
  return false;
}

}  // namespace

bool slcd_init() {
  reg(RCU_AHBEN) |= 0xFu << 17;
  for (const Pin &p : LCD_PINS) pin_config(p, Mode::AF, Pull::DOWN, 3);
  pin_config({GPIOD, 6}, Mode::ANALOG, Pull::NONE);  // VSLCD

  reg(RCU_APB1EN) |= 1u << 28;     // PMU
  reg(PMU_CTL0) |= 0x100u;         // backup-domain write enable
  reg(RCU_RSTSCK) |= 1u;           // IRC32K
  if (!wait_set(RCU_RSTSCK, 2u)) return false;
  rmw(RCU_BDCTL, 0x300u, 0x200u);  // RTC/SLCD clock = IRC32K
  reg(RCU_APB1EN) |= 1u << 10;     // SLCD
  for (volatile uint32_t i = 0; i < 1000; i = i + 1) {
  }
  reg(RCU_APB1RST) |= 1u << 10;
  reg(RCU_APB1RST) &= ~(1u << 10);

  reg(SLCD_CFG) = 0x0042E081u;  // PSC /2, DIV 16, dead time 1, blink /1024 on SEG0 (unwired), high drive
  reg(SLCD_CTL) = 0x00000012u;  // DUTY 1/8, BIAS 1/4, VSRC external
  if (!wait_set(SLCD_STAT, 1u << 5)) return false;  // SYNF
  reg(SLCD_CTL) |= 1u;                              // SLCDON
  return wait_set(SLCD_STAT, 1u);                   // ONF
}

bool slcd_write(const uint32_t com[8]) {
  if (reg(SLCD_STAT) & 4u) return false;  // UPRF: the last update hasn't been taken yet
  for (int k = 0; k < 8; k++) reg(slcd_data(k)) = com[k];
  reg(SLCD_STAT) |= 4u;  // request an update
  return true;
}

}  // namespace hw
