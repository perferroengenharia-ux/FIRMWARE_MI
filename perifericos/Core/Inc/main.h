/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32c0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RCC_OSCX_IN_Pin GPIO_PIN_14
#define RCC_OSCX_IN_GPIO_Port GPIOC
#define RCC_OSCX_OUT_Pin GPIO_PIN_15
#define RCC_OSCX_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define ONOFF_BTN_Pin GPIO_PIN_0
#define ONOFF_BTN_GPIO_Port GPIOA
#define ONOFF_BTN_EXTI_IRQn EXTI0_1_IRQn
#define SENSOR_Pin GPIO_PIN_1
#define SENSOR_GPIO_Port GPIOA
#define VCP_USART2_TX_Pin GPIO_PIN_2
#define VCP_USART2_TX_GPIO_Port GPIOA
#define VCP_USART2_RX_Pin GPIO_PIN_3
#define VCP_USART2_RX_GPIO_Port GPIOA
#define led_Pin GPIO_PIN_5
#define led_GPIO_Port GPIOA
#define BOMBA_Pin GPIO_PIN_0
#define BOMBA_GPIO_Port GPIOB
#define MOTOR_Pin GPIO_PIN_1
#define MOTOR_GPIO_Port GPIOB
#define SWING_Pin GPIO_PIN_2
#define SWING_GPIO_Port GPIOB
#define DEBUG_SWDIO_Pin GPIO_PIN_13
#define DEBUG_SWDIO_GPIO_Port GPIOA
#define DEBUG_SWCLK_Pin GPIO_PIN_14
#define DEBUG_SWCLK_GPIO_Port GPIOA
#define DRENO_Pin GPIO_PIN_3
#define DRENO_GPIO_Port GPIOB
#define BOMBA_BTN_Pin GPIO_PIN_4
#define BOMBA_BTN_GPIO_Port GPIOB
#define BOMBA_BTN_EXTI_IRQn EXTI4_15_IRQn
#define SWING_BTN_Pin GPIO_PIN_5
#define SWING_BTN_GPIO_Port GPIOB
#define SWING_BTN_EXTI_IRQn EXTI4_15_IRQn
#define DRENO_BTN_Pin GPIO_PIN_6
#define DRENO_BTN_GPIO_Port GPIOB
#define DRENO_BTN_EXTI_IRQn EXTI4_15_IRQn
#define EXAUSTOR_BTN_Pin GPIO_PIN_7
#define EXAUSTOR_BTN_GPIO_Port GPIOB
#define EXAUSTOR_BTN_EXTI_IRQn EXTI4_15_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
