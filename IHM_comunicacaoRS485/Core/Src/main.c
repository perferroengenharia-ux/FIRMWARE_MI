/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Módulo Inversor (MI) - STM32 RS485 Slave
  * Processa comandos da IHM e retorna sensores.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* Private typedef -----------------------------------------------------------*/
typedef struct {
    uint8_t addr, type, seq, len;
    uint8_t payload[128];
} frame_t;

typedef enum {
    PS_WAIT_SOF = 0, PS_HDR_ADDR, PS_HDR_TYPE, PS_HDR_SEQ, PS_HDR_LEN, PS_PAYLOAD, PS_CRC_L, PS_CRC_H
} parse_state_t;

typedef struct {
    parse_state_t st;
    bool esc_next;
    uint8_t addr, type, seq, len;
    uint8_t payload[128];
    uint8_t pay_i;
    uint8_t crc_l, crc_h;
} frame_parser_t;

/* --- Estruturas do Inversor --- */
typedef struct {
    uint8_t  buttons;      // Bit 0: Start, Bit 1: Stop, Bit 2: Reset
    uint16_t target_freq;  // Frequência desejada
    uint16_t ramp_time;    // Tempo de rampa
    uint8_t  brake;        // Estado do freio
} mi_commands_t;

typedef struct {
    uint16_t current_speed; // RPM
    uint16_t motor_current; // mA
    uint16_t bus_voltage;   // V
    uint8_t  temp;          // °C
} mi_sensors_t;

/* Private define ------------------------------------------------------------*/
#define SOF             0x7E
#define ESC             0x7D
#define ESC_XOR         0x20

#define ADDR_STM32      0x01
#define ADDR_MASTER     0xF0

#define TYPE_READ_STATUS 0x04
#define TYPE_ACK        0x80
#define TYPE_NACK       0x81

#define MAX_PAYLOAD     128
#define RX_DMA_BUF_SZ   256

/* Watchdog Config */
#define COMMS_TIMEOUT_MS 2000

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Double-buffer RX globals */
uint8_t rx_dma_buf[2][RX_DMA_BUF_SZ];
volatile uint16_t rx_ready_len = 0;
volatile uint8_t  rx_ready_idx = 0;
volatile bool     rx_dma_ready = false;
volatile uint8_t  rx_active_idx = 0;

static frame_parser_t g_parser;

/* Variáveis de Aplicação */
mi_commands_t g_cmd = {0};
mi_sensors_t  g_sns = { .current_speed = 0, .motor_current = 0, .bus_voltage = 310, .temp = 35 };
uint32_t last_packet_tick = 0;
bool comms_ok = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);

/* ====================================================================
 * PROTOCOLO & COMUNICAÇÃO
 * ==================================================================== */

static uint16_t crc16_ibm(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

static void parser_reset(frame_parser_t *p) {
    p->st = PS_WAIT_SOF; p->esc_next = false; p->pay_i = 0;
}

static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame) {
    if (byte == SOF) { p->st = PS_HDR_ADDR; p->esc_next = false; p->pay_i = 0; return false; }
    if (p->st == PS_WAIT_SOF) return false;
    if (p->esc_next) { byte ^= ESC_XOR; p->esc_next = false; }
    else if (byte == ESC) { p->esc_next = true; return false; }

    switch (p->st) {
        case PS_HDR_ADDR: p->addr = byte; p->st = PS_HDR_TYPE; break;
        case PS_HDR_TYPE: p->type = byte; p->st = PS_HDR_SEQ; break;
        case PS_HDR_SEQ:  p->seq  = byte; p->st = PS_HDR_LEN; break;
        case PS_HDR_LEN:
            p->len = byte;
            if (p->len > MAX_PAYLOAD) { parser_reset(p); return false; }
            p->st = (p->len == 0) ? PS_CRC_L : PS_PAYLOAD;
            break;
        case PS_PAYLOAD:
            p->payload[p->pay_i++] = byte;
            if (p->pay_i >= p->len) p->st = PS_CRC_L;
            break;
        case PS_CRC_L: p->crc_l = byte; p->st = PS_CRC_H; break;
        case PS_CRC_H: {
            p->crc_h = byte;
            uint8_t tmp[4 + MAX_PAYLOAD];
            tmp[0] = p->addr; tmp[1] = p->type; tmp[2] = p->seq; tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);
            uint16_t calc = crc16_ibm(tmp, 4 + p->len);
            uint16_t rx_crc = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);
            if (calc == rx_crc) {
                out_frame->addr = p->addr; out_frame->type = p->type; out_frame->seq = p->seq; out_frame->len = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);
                parser_reset(p); return true;
            }
            parser_reset(p);
        } break;
        default: parser_reset(p); break;
    }
    return false;
}

static void rs485_send_reply(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len) {
    uint8_t raw[140], esc[280];
    uint16_t r_idx = 0, e_idx = 0;
    raw[r_idx++] = SOF;
    raw[r_idx++] = ADDR_MASTER;
    raw[r_idx++] = type;
    raw[r_idx++] = seq;
    raw[r_idx++] = len;
    if (len && payload) { memcpy(&raw[r_idx], payload, len); r_idx += len; }
    uint16_t crc = crc16_ibm(&raw[1], r_idx - 1);
    raw[r_idx++] = (uint8_t)(crc & 0xFF);
    raw[r_idx++] = (uint8_t)((crc >> 8) & 0xFF);

    esc[e_idx++] = SOF;
    for (uint16_t i = 1; i < r_idx; i++) {
        if (raw[i] == SOF || raw[i] == ESC) {
            esc[e_idx++] = ESC; esc[e_idx++] = raw[i] ^ ESC_XOR;
        } else esc[e_idx++] = raw[i];
    }

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
    for(volatile int i=0; i<500; i++);
    HAL_UART_Transmit(&huart1, esc, e_idx, 50);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

/* ====================================================================
 * LÓGICA DA APLICAÇÃO (INVERSOR)
 * ==================================================================== */

static void process_frame(const frame_t *fr) {
    if (fr->addr != ADDR_STM32) return;

    // Reset do Watchdog
    last_packet_tick = HAL_GetTick();
    comms_ok = true;

    if (fr->type == TYPE_READ_STATUS) {
        // 1. Recebe 6 bytes da IHM (Comandos)
        if (fr->len == 6) {
            g_cmd.buttons     = fr->payload[0];
            g_cmd.target_freq = (fr->payload[1] << 8) | fr->payload[2];
            g_cmd.ramp_time   = (fr->payload[3] << 8) | fr->payload[4];
            g_cmd.brake       = fr->payload[5];

            // Aqui você processaria os botões (ex: bit 0 = Start)
            if (g_cmd.buttons & 0x01) HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
            else if (g_cmd.buttons & 0x02) HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
        }

        // 2. Prepara 7 bytes de resposta (Sensores)
        uint8_t resp[7];
        resp[0] = (uint8_t)(g_sns.current_speed >> 8);
        resp[1] = (uint8_t)(g_sns.current_speed & 0xFF);
        resp[2] = (uint8_t)(g_sns.motor_current >> 8);
        resp[3] = (uint8_t)(g_sns.motor_current & 0xFF);
        resp[4] = (uint8_t)(g_sns.bus_voltage >> 8);
        resp[5] = (uint8_t)(g_sns.bus_voltage & 0xFF);
        resp[6] = g_sns.temp;

        rs485_send_reply(TYPE_ACK, fr->seq, resp, 7);
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();

    parser_reset(&g_parser);
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET); // Modo RX

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, rx_dma_buf[rx_active_idx], RX_DMA_BUF_SZ);

    while (1) {
        // 1. Processa dados da UART (DMA/IDLE)
        uint16_t n = 0;
        uint8_t idx = 0;

        __disable_irq();
        if (rx_dma_ready) {
            rx_dma_ready = false;
            n = rx_ready_len;
            idx = rx_ready_idx;
        }
        __enable_irq();

        if (n > 0) {
            frame_t fr;
            for (uint16_t i = 0; i < n; i++) {
                if (parser_feed(&g_parser, rx_dma_buf[idx][i], &fr)) {
                    process_frame(&fr);
                }
            }
        }

        // 2. Watchdog de Segurança
        if (HAL_GetTick() - last_packet_tick > COMMS_TIMEOUT_MS) {
            if (comms_ok) {
                // FALHA: IHM desconectada. Desligar saídas do MI por segurança.
                g_cmd.buttons = 0;
                g_sns.current_speed = 0;
                HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
                comms_ok = false;
            }
        }

        // 3. Simulação de leitura de sensores (em produção viria do ADC/Encoder)
        if (comms_ok) {
            g_sns.motor_current = 1250; // 1.25A
            // g_sns.current_speed aumentaria conforme rampa...
        }
    }
}

// ... MX_Init Functions (Clock, UART, DMA, GPIO) seguem o padrão CubeMX anterior

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : VCP_USART2_TX_Pin VCP_USART2_RX_Pin */
  GPIO_InitStruct.Pin = VCP_USART2_TX_Pin|VCP_USART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Led_Pin */
  GPIO_InitStruct.Pin = Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_EN_Pin */
  GPIO_InitStruct.Pin = RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485_EN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
