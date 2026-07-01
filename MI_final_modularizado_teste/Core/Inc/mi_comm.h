#ifndef MI_COMM_H
#define MI_COMM_H

#include "main.h"
#include "mi_config.h"
#include <stdint.h>
#include <stdbool.h>

extern uint8_t rx_dma_buf[2][MI_RX_DMA_BUF_SZ];
extern volatile uint16_t rx_ready_len;
extern volatile uint8_t  rx_ready_idx;
extern volatile bool     rx_dma_ready;
extern volatile uint8_t  rx_active_idx;

void mi_comm_init(void);
void mi_comm_start_rx(void);
void mi_comm_process_rx(void);
void mi_comm_uart_irq_handler(UART_HandleTypeDef *huart);
void mi_comm_apply_received_command(void);
void mi_comm_send_status_reply(uint8_t seq);
void mi_comm_send_ack_mi(uint8_t seq);

#endif /* MI_COMM_H */
