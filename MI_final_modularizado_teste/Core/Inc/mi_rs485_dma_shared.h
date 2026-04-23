#ifndef MI_RS485_DMA_SHARED_H
#define MI_RS485_DMA_SHARED_H

#include <stdbool.h>
#include <stdint.h>
#include "mi_types.h"

extern uint8_t rx_dma_buf[2][MI_RX_DMA_BUF_SZ];
extern volatile uint16_t rx_ready_len;
extern volatile uint8_t  rx_ready_idx;
extern volatile bool     rx_dma_ready;
extern volatile uint8_t  rx_active_idx;

#endif
