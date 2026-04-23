/* Copy only the USART1 IRQ body into your CubeMX-generated stm32f3xx_it.c.
 * The actual DMA IRQ name depends on the DMA channel chosen by CubeMX for USART1_RX.
 */
#include "main.h"
#include "mi_rs485_dma_shared.h"

extern UART_HandleTypeDef huart2;

void USART1_IRQHandler(void)
{
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET) {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);

    HAL_UART_DMAStop(&huart2);
    uint16_t remaining = __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    uint16_t len = (uint16_t)(MI_RX_DMA_BUF_SZ - remaining);

    if (len > 0u) {
      rx_ready_idx = rx_active_idx;
      rx_ready_len = len;
      rx_dma_ready = true;
    }

    rx_active_idx ^= 1u;
    HAL_UART_Receive_DMA(&huart2, rx_dma_buf[rx_active_idx], MI_RX_DMA_BUF_SZ);
  }

  HAL_UART_IRQHandler(&huart2);
}
