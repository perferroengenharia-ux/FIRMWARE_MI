#ifndef RS485_LINK_H
#define RS485_LINK_H

#include <stdint.h>
#include <stdbool.h>

// tamanhos (precisam ser visíveis no IT.c)
#define RX_DMA_BUF_SZ   256

// buffers/flags usados no IRQ (definidos no main.c)
extern uint8_t rx_dma_buf[RX_DMA_BUF_SZ];
extern volatile uint16_t rx_dma_len;
extern volatile bool rx_dma_ready;

#endif
