// oem/src/hw/can.h: the bxCAN at 0x40006400 on PA4 (RX) / PA5 (TX), 125 kbit/s, polled (IRQ20 is shared with USBD).
#pragma once
#include <cstdint>

#include "app/can_port.h"

namespace hw {

// Stock init (docs/history/notes/oem-can-sequences.md §1-§3) with one filter bank passing only NASN replies to FIFO0,
// and no interrupts. False if the controller never entered or left init mode.
bool can_init();
// Into a free TX mailbox as an 8-byte extended data frame (f.len is ignored: every NASN frame is 8 bytes).
// False if all three are busy; never waits.
bool can_send(const oem::CanFrame &f);
// One frame from FIFO0. False if it is empty. A frame that isn't an extended data frame comes back with len 0.
bool can_receive(oem::CanFrame &f);
// Abort all three TX mailboxes (a frame already on the wire finishes or fails first).
void can_abort_tx();
// CAN_ERR (TEC, REC, error state) for diagnostics.
uint32_t can_error();

}  // namespace hw
