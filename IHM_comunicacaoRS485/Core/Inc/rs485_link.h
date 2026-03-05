#ifndef RS485_LINK_H
#define RS485_LINK_H

#include <stdint.h>
#include <stdbool.h>

#define RX_DMA_BUF_SZ   256

// Double-buffer RX
extern uint8_t rx_dma_buf[2][RX_DMA_BUF_SZ];
extern volatile uint16_t rx_ready_len;
extern volatile uint8_t  rx_ready_idx;
extern volatile bool     rx_dma_ready;

// Índice do buffer em uso pelo DMA
extern volatile uint8_t rx_active_idx;

#endif
