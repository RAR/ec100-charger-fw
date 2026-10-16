// oem/src/hw/slcd.h
#pragma once
#include <cstdint>

namespace hw {

bool slcd_init();                         // false if the controller never synced or turned on
// Never waits: false, and the frame is dropped, while the previous update is still pending (the next call retries).
bool slcd_write(const uint32_t com[8]);

}  // namespace hw
