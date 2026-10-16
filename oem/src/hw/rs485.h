// oem/src/hw/rs485.h: USART1 on PA2 (TX) / PA3 (RX) through the RS485 transceiver, DE on PB10 (high = transmit).
// RX by circular DMA ch1 into a 512-byte ring, read from the main loop; TX by interrupt (IRQ28), which also drops DE
// on TC. docs/history/notes/oem-usart-sequences.md, plan decisions M2-M4.
#pragma once
#include <cstddef>
#include <cstdint>

namespace hw {

// TEN acknowledges (STAT TEA) that never came, from rs485_init or rs485_set_baud. Counted for SWD, never fatal.
extern uint32_t g_rs485_ack_timeout;

// 8N1, DE low, RX DMA running, IRQ28 enabled. False only if the baud can't be set from the APB1 clock. Leaves PB0/PB1
// alone (inputs since the bootloader takeover).
bool rs485_init(uint32_t baud);
// False (no change) if not initialised, busy, or a baud the divider can't reach. RX DMA keeps running.
bool rs485_set_baud(uint32_t baud);
// Bytes received since the last call, oldest first, up to max (the rest stay for the next call). Also clears the
// USART error flags. If the ring nearly filled up unread, counts an overrun, drops what was there and returns 0.
size_t rs485_read(uint8_t *buf, size_t max);
// Copies 1-256 bytes, raises DE and starts sending. False (nothing sent, DE untouched) if rs485_init hasn't succeeded,
// busy, empty or longer than 256.
bool rs485_send(const uint8_t *data, size_t n);
bool rs485_tx_busy();        // true from rs485_send until TC after the last byte, when the ISR drops DE
// Ends a send that never reached TC: interrupts off, DE low, not busy (tx_done_ms is now). Safe when not busy.
void rs485_abort_tx();
uint32_t rs485_tx_done_ms();  // millis() at the last TC (for FrameAssembler::discard_until)
uint32_t rs485_rx_overruns();  // ring overruns plus any USART ORERR seen

}  // namespace hw
