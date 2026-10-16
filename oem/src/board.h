// oem/src/board.h: v1 board (silkscreen "100A充电器-主控板_V1.1-250911"); see docs/history/notes/oem-hw-map.md §2 and §10.
#pragma once
#include "hw/gpio.h"

namespace board {

using hw::Pin;
constexpr Pin KEY_SWITCH{hw::GPIOC, 6};  // keys: active high, pull-down
constexpr Pin KEY_UP{hw::GPIOB, 15};
constexpr Pin KEY_DOWN{hw::GPIOA, 12};
constexpr Pin KEY_ENTER{hw::GPIOA, 11};
constexpr Pin RS485_DE{hw::GPIOB, 10};   // high = transmit; shared by USART1 and UART4
constexpr Pin USART1_RX{hw::GPIOA, 3};
constexpr Pin UART4_RX{hw::GPIOB, 1};
constexpr Pin LED_FAULT{hw::GPIOD, 0};   // active low in stock v1
constexpr Pin LED_CHARGE{hw::GPIOD, 1};  // active low in stock v1
constexpr Pin OUT_PC1{hw::GPIOC, 1};     // relay (bench: clicks when high); stock: high while charging in voltage mode
constexpr Pin BACKLIGHT{hw::GPIOC, 2};   // stock: high = on
constexpr Pin PROBE_PB6{hw::GPIOB, 6};   // unused by stock v1; driving it high had no visible effect on the bench
// Generator contact, high = closed. Confirmed on the bench (docs/history/notes/oem-bringup.md): PC1 high clicks the
// relay.
constexpr Pin RELAY = OUT_PC1;

inline void enable_gpio_clocks() { hw::reg(hw::RCU_AHBEN) |= 0xFu << 17; }  // PA..PD

}  // namespace board
