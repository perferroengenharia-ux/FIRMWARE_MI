/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32c0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32c0xx_it.h"
#include <stdbool.h>
#include "mi_rs485_dma_shared.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint16_t dbg_len = 0;
volatile uint8_t  dbg_b0 = 0;
volatile uint8_t  dbg_b1 = 0;
volatile uint8_t  dbg_b2 = 0;
volatile uint8_t  dbg_b3 = 0;
volatile uint8_t  dbg_b4 = 0;
volatile uint8_t  dbg_b5 = 0;
volatile uint8_t  dbg_b6 = 0;
volatile uint8_t  dbg_b7 = 0;
volatile uint8_t  dbg_b8 = 0;
volatile uint8_t  dbg_b9 = 0;
volatile uint32_t dbg_irq_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32C0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32c0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(User_Button_Pin);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
void TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */

  /* USER CODE END TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim14);
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
  * @brief This function handles USART1 interrupt.
  */
void USART1_IRQHandler(void)
{
	  // IDLE detectado?
	  if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET) {
	    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

	    // Para DMA e calcula quantos bytes chegaram no buffer ativo
	    HAL_UART_DMAStop(&huart1);
	    uint16_t remaining = __HAL_DMA_GET_COUNTER(huart1.hdmarx);
	    uint16_t len = (uint16_t)(RX_DMA_BUF_SZ - remaining);

	    dbg_len = len;

	    if (len > 0u) dbg_b0 = rx_dma_buf[rx_active_idx][0];
	    if (len > 1u) dbg_b1 = rx_dma_buf[rx_active_idx][1];
	    if (len > 2u) dbg_b2 = rx_dma_buf[rx_active_idx][2];
	    if (len > 3u) dbg_b3 = rx_dma_buf[rx_active_idx][3];
	    if (len > 4u) dbg_b4 = rx_dma_buf[rx_active_idx][4];
	    if (len > 5u) dbg_b5 = rx_dma_buf[rx_active_idx][5];
	    if (len > 6u) dbg_b6 = rx_dma_buf[rx_active_idx][6];
	    if (len > 7u) dbg_b7 = rx_dma_buf[rx_active_idx][7];
	    dbg_irq_count++;

	    if (len > 8u) dbg_b8 = rx_dma_buf[rx_active_idx][8];
	    if (len > 9u) dbg_b9 = rx_dma_buf[rx_active_idx][9];

	    // Publica buffer pronto
	    rx_ready_idx = rx_active_idx;
	    rx_ready_len = len;
	    rx_dma_ready = true;

	    // Alterna buffer e reinicia DMA no outro buffer
	    rx_active_idx ^= 1U;
	    HAL_UART_Receive_DMA(&huart1, rx_dma_buf[rx_active_idx], RX_DMA_BUF_SZ);
	  }

	  HAL_UART_IRQHandler(&huart1);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
