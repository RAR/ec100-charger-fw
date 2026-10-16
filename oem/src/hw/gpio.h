// oem/src/hw/gpio.h
#pragma once
#include "hw/regs.h"

namespace hw {

enum class Mode : uint32_t { INPUT = 0, OUTPUT = 1, AF = 2, ANALOG = 3 };
enum class Pull : uint32_t { NONE = 0, UP = 1, DOWN = 2 };

struct Pin {
  uint32_t port;
  uint8_t n;
};

// Push-pull, high speed (OSPD 3), as the stock firmware configures every output and AF pin.
inline void pin_config(Pin p, Mode mode, Pull pull, uint8_t af = 0) {
  const uint32_t s2 = 2u * p.n, s4 = 4u * (p.n & 7u);
  rmw(p.port + GPIO_PUD, 3u << s2, uint32_t(pull) << s2);
  rmw(p.port + GPIO_OSPD, 3u << s2, 3u << s2);
  rmw(p.port + GPIO_OMODE, 1u << p.n, 0);
  rmw(p.port + (p.n < 8 ? GPIO_AFSEL0 : GPIO_AFSEL1), 0xFu << s4, uint32_t(af) << s4);
  rmw(p.port + GPIO_CTL, 3u << s2, uint32_t(mode) << s2);
}

inline void pin_write(Pin p, bool high) { reg(p.port + (high ? GPIO_BOP : GPIO_BC)) = 1u << p.n; }
inline bool pin_read(Pin p) { return (reg(p.port + GPIO_ISTAT) >> p.n) & 1u; }

}  // namespace hw
