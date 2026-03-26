/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Módulo Inversor (MI) - STM32 RS485 Slave
  * Protocolo: 9 bytes CMD (IHM->MI) | 9 bytes TELEMETRIA (MI->IHM)
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* ====================================================================
 * DEFINIÇÕES DO PROTOCOLO RS485
 * ==================================================================== */
#define SOF              0x7E
#define ESC              0x7D
#define ESC_XOR          0x20

#define ADDR_STM32       0x01
#define ADDR_MASTER      0xF0

#define TYPE_READ_STATUS 0x04  // Loop de operação e telemetria
#define TYPE_WRITE_PARAM 0x05  // Handshake de parâmetros
#define TYPE_ACK_MI      0x06  // Resposta Handshake
#define TYPE_ACK         0x80  // Resposta Status (Telemetria)

#define MAX_PAYLOAD      128
#define RX_DMA_BUF_SZ    256
#define COMMS_TIMEOUT_MS 2000

/* ====================================================================
 * ESTRUTURAS DE DADOS
 * ==================================================================== */
typedef struct {
    uint8_t addr, type, seq, len;
    uint8_t payload[MAX_PAYLOAD];
} frame_t;

typedef enum {
    PS_WAIT_SOF = 0, PS_HDR_ADDR, PS_HDR_TYPE, PS_HDR_SEQ, PS_HDR_LEN, PS_PAYLOAD, PS_CRC_L, PS_CRC_H
} parse_state_t;

typedef struct {
    parse_state_t st;
    bool esc_next;
    uint8_t addr, type, seq, len, payload[MAX_PAYLOAD], pay_i, crc_l, crc_h;
} frame_parser_t;

/* Parâmetros do Inversor */
typedef struct {
    uint16_t p10_acel, p11_desacel, p20_freq_min, p21_freq_max;
    uint16_t p35_torque, p42_igbt_khz, p43_i_motor, p45_v_min;
    uint8_t  p44_autoreset, p85_sensor_mode;
} mi_settings_t;

/* Comandos recebidos do IHM */
typedef struct {
    uint8_t  buttons;
    uint16_t target_freq;
    uint8_t  direction;    // 0=FWD, 1=REV
    uint8_t  e08_active;
} mi_commands_t;

/* Telemetria Real/Simulada para o IHM */
typedef struct {
    uint16_t current_freq;
    uint16_t motor_current; // mA
    uint16_t bus_voltage;   // VDC
    uint16_t out_voltage;   // Vout RMS
    uint8_t  temp_igbt;     // Celsius
} mi_telemetry_t;

/* ====================================================================
 * VARIÁVEIS GLOBAIS
 * ==================================================================== */
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Variáveis para Simulação (Ajuste via Debug/Live Expressions) */
volatile uint16_t sim_v_out = 220;
volatile uint16_t sim_v_bus = 311;
volatile uint16_t sim_i_out = 520;
volatile uint8_t  sim_temp  = 32;

static frame_parser_t g_parser;
mi_settings_t  g_settings = {0};
mi_commands_t   g_cmd      = {0};
mi_telemetry_t g_tel      = {0};

uint32_t last_packet_tick = 0;
bool hardware_comms_ok = false;
bool handshake_done = false;

/* Buffers DMA */
uint8_t rx_dma_buf[2][RX_DMA_BUF_SZ];
volatile uint16_t rx_ready_len = 0;
volatile uint8_t  rx_ready_idx = 0;
volatile bool     rx_dma_ready = false;
volatile uint8_t  rx_active_idx = 0;

/* Protótipos */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);

/* ====================================================================
 * UTILITÁRIOS (CRC, PARSER, TX)
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
    if (byte == SOF) { parser_reset(p); p->st = PS_HDR_ADDR; return false; }
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
            if (calc == ((uint16_t)p->crc_l | (p->crc_h << 8))) {
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
    uint8_t raw[MAX_PAYLOAD + 10], esc[(MAX_PAYLOAD + 10) * 2];
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
    HAL_UART_Transmit(&huart1, esc, e_idx, 50);
    // Aguarda o fim físico da transmissão antes de baixar o DE/RE
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

/* ====================================================================
 * PROCESSAMENTO DE FRAMES
 * ==================================================================== */

static void process_frame(const frame_t *fr) {
    if (fr->addr != ADDR_STM32) return;

    last_packet_tick = HAL_GetTick();
    hardware_comms_ok = true;

    switch (fr->type) {

    case TYPE_WRITE_PARAM:
        if (fr->len == 3) {
            uint8_t  p_id = fr->payload[0];
            uint16_t p_val = (fr->payload[1] << 8) | fr->payload[2];

            if      (p_id == 10) g_settings.p10_acel      = p_val;
            else if (p_id == 11) g_settings.p11_desacel   = p_val;
            else if (p_id == 20) g_settings.p20_freq_min  = p_val;
            else if (p_id == 21) g_settings.p21_freq_max  = p_val;
            else if (p_id == 35) g_settings.p35_torque    = p_val;
            else if (p_id == 42) g_settings.p42_igbt_khz  = p_val;
            else if (p_id == 43) g_settings.p43_i_motor   = p_val;
            else if (p_id == 44) g_settings.p44_autoreset = (uint8_t)p_val;
            else if (p_id == 45) g_settings.p45_v_min     = p_val;
            else if (p_id == 85) {
                 g_settings.p85_sensor_mode = (uint8_t)p_val;
                 handshake_done = true;
            }
            rs485_send_reply(TYPE_ACK_MI, fr->seq, NULL, 0);
        }
        break;

    case TYPE_READ_STATUS:
        if (fr->len == 9) {
            // --- Extração de Comandos ---
            g_cmd.buttons     = fr->payload[0];
            g_cmd.target_freq = (fr->payload[1] << 8) | fr->payload[2];
            g_cmd.direction   = fr->payload[3]; // 0=FWD, 1=REV
            g_cmd.e08_active  = fr->payload[8];

            // Lógica de acionamento do motor (Exemplo simples com LED)
            if (!g_cmd.e08_active && handshake_done) {
                if (g_cmd.buttons & 0x01) { // START
                    HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_SET);
                } else if (g_cmd.buttons & 0x02) { // STOP
                    HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
                }
            } else {
                HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
            }
        }

        // --- Montagem da Telemetria (9 Bytes) ---
        uint8_t resp[9];
        resp[0] = (uint8_t)(g_tel.current_freq >> 8);
        resp[1] = (uint8_t)(g_tel.current_freq & 0xFF);
        resp[2] = (uint8_t)(g_tel.motor_current >> 8);
        resp[3] = (uint8_t)(g_tel.motor_current & 0xFF);
        resp[4] = (uint8_t)(g_tel.bus_voltage >> 8);
        resp[5] = (uint8_t)(g_tel.bus_voltage & 0xFF);
        resp[6] = (uint8_t)(g_tel.out_voltage >> 8);
        resp[7] = (uint8_t)(g_tel.out_voltage & 0xFF);
        resp[8] = g_tel.temp_igbt;

        rs485_send_reply(TYPE_ACK, fr->seq, resp, 9);
        break;
    }
}

/* ====================================================================
 * LOOP PRINCIPAL
 * ==================================================================== */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();

    parser_reset(&g_parser);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, rx_dma_buf[rx_active_idx], RX_DMA_BUF_SZ);

    while (1) {
        // 1. Manutenção de Barramento
        if (HAL_GetTick() - last_packet_tick > 500) {
            parser_reset(&g_parser);
            if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
                __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
                HAL_UART_Receive_DMA(&huart1, rx_dma_buf[rx_active_idx], RX_DMA_BUF_SZ);
            }
        }

        // 2. Processamento de dados via DMA (Troca de buffers)
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

        // 3. Watchdog de Comunicação
        if (HAL_GetTick() - last_packet_tick > COMMS_TIMEOUT_MS) {
            HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, GPIO_PIN_RESET);
            hardware_comms_ok = false;
        }

        // 4. Lógica de Simulação de Hardware
        if (hardware_comms_ok && !g_cmd.e08_active && handshake_done) {
            // Se o LED (motor) estiver ligado, simula rampa
            if (HAL_GPIO_ReadPin(Led_GPIO_Port, Led_Pin) == GPIO_PIN_SET) {
                if (g_tel.current_freq < g_cmd.target_freq) g_tel.current_freq += 5;
                else if (g_tel.current_freq > g_cmd.target_freq) g_tel.current_freq -= 5;
            } else {
                if (g_tel.current_freq > 0) g_tel.current_freq -= 10;
                else g_tel.current_freq = 0;
            }

            // Simulações de sensores baseadas na frequência
            g_tel.motor_current = (g_tel.current_freq / 2) + 200; // mA
            g_tel.out_voltage   = (g_tel.current_freq * 220) / 6000; // Vout proporcional
        } else {
            g_tel.current_freq = 0;
            g_tel.motor_current = 0;
            g_tel.out_voltage = 0;
        }

        // Valores fixos/debug
        g_tel.bus_voltage = sim_v_bus;
        g_tel.temp_igbt   = sim_temp;

        HAL_Delay(10);
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
