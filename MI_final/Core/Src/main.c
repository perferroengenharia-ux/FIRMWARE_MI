/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Módulo Inversor (MI) unificado
  *
  * Integra:
  * - Comunicação RS485 com a IHM (ESP32)
  * - Controle do motor / SPWM
  * - Periféricos (bomba, swing, dreno e sensor)
  *
  * Observações importantes:
  * - Este arquivo foi montado para substituir os três códigos separados do MI
  *   dentro do projeto STM32CubeMX/CubeIDE já existente.
  * - Mantém compatibilidade com a IHM unificada v2:
  *      payload[0] = botões pulsados
  *      payload[1..2] = frequência alvo em centésimos de Hz
  *      payload[3] = direção nominal (0=FWD, 1=REV)
  *      payload[4] = flags auxiliares
  *      payload[5] = estado do dreno
  *      payload[6] = P90 (reservado)
  *      payload[7] = P91 (reservado)
  *      payload[8] = E08 ativo na IHM
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "mi_rs485_dma_shared.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t addr, type, seq, len;
    uint8_t payload[128];
} frame_t;

typedef enum {
    PS_WAIT_SOF = 0,
    PS_HDR_ADDR,
    PS_HDR_TYPE,
    PS_HDR_SEQ,
    PS_HDR_LEN,
    PS_PAYLOAD,
    PS_CRC_L,
    PS_CRC_H
} parse_state_t;

typedef struct {
    parse_state_t st;
    bool esc_next;
    uint8_t addr, type, seq, len;
    uint8_t payload[128];
    uint8_t pay_i, crc_l, crc_h;
} frame_parser_t;

typedef struct {
    uint16_t p10_acel;
    uint16_t p11_desacel;
    uint16_t p20_freq_min;
    uint16_t p21_freq_max;
    uint16_t p35_torque;
    uint16_t p42_igbt_khz;
    uint16_t p43_i_motor;
    uint16_t p45_v_min;
    uint8_t  p44_autoreset;
    uint8_t  p85_sensor_mode;
} mi_settings_t;

typedef struct {
    uint8_t  buttons;
    uint16_t target_freq_centi_hz;
    uint8_t  direction;
    uint8_t  aux_flags;
    uint8_t  dreno_status;
    uint8_t  p90;
    uint8_t  p91;
    uint8_t  e08_active;
} mi_commands_t;

typedef struct {
    uint16_t current_freq_centi_hz;
    uint16_t motor_current_ma;
    uint16_t bus_voltage_vdc;
    uint16_t out_voltage_vrms;
    uint8_t  temp_igbt_c;
} mi_telemetry_t;

typedef struct {
    float    saved_freq_hz;
    uint8_t  saved_P10;
    uint8_t  saved_P11;
    uint8_t  saved_P20;
    uint8_t  saved_P21;
    uint8_t  saved_P35;
    uint8_t  saved_P42;
    uint8_t  saved_P12;
    uint8_t  saved_locked;
    uint32_t padding;
} SystemParams;

typedef enum {
    DRENO_IDLE = 0,
    DRENO_AGUARDANDO_LED = 1,
    DRENO_EM_CURSO = 2
} dreno_state_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SOF                      0x7E
#define ESC                      0x7D
#define ESC_XOR                  0x20

#define ADDR_STM32               0x01
#define ADDR_MASTER              0xF0

#define TYPE_READ_STATUS         0x04
#define TYPE_WRITE_PARAM         0x05
#define TYPE_ACK_MI              0x06
#define TYPE_ACK                 0x80

#define MAX_PAYLOAD              128
#define RX_DMA_BUF_SZ            256
#define COMMS_TIMEOUT_MS         2000U

#define MI_STATUS_WATER_SHORTAGE (1u << 0)

#define BTN_BIT_START            (1u << 0)
#define BTN_BIT_STOP             (1u << 1)
#define BTN_BIT_UP               (1u << 2)
#define BTN_BIT_DOWN             (1u << 3)

#define AUX_BIT_BOMBA            (1u << 0)
#define AUX_BIT_SWING            (1u << 1)
#define AUX_BIT_EXAUSTAO         (1u << 2)
#define AUX_BIT_DRENO            (1u << 3)
#define AUX_BIT_SYSTEM_ON        (1u << 4)

#ifndef FLASH_USER_ADDR
#define FLASH_USER_ADDR          0x08007FF0U
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CLAMP_F(v, lo, hi) (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
static frame_parser_t g_parser;
static mi_settings_t  g_settings = {0};
static mi_commands_t  g_cmd      = {0};
static mi_telemetry_t g_tel      = {0};

static uint32_t last_packet_tick = 0;
static bool hardware_comms_ok = false;
static bool handshake_done = false;

/* Compartilhados com stm32c0xx_it.c para o tratamento RX DMA + IDLE */
uint8_t rx_dma_buf[2][RX_DMA_BUF_SZ];
volatile uint16_t rx_ready_len = 0;
volatile uint8_t  rx_ready_idx = 0;
volatile bool     rx_dma_ready = false;
volatile uint8_t  rx_active_idx = 0;

volatile uint32_t dbg_main_rx_count = 0;
volatile uint32_t dbg_parser_ok_count = 0;
volatile uint32_t dbg_type_write_param_count = 0;
volatile uint32_t dbg_crc_fail_count = 0;
volatile uint16_t dbg_last_n = 0;
volatile uint16_t dbg_last_crc_calc = 0;
volatile uint16_t dbg_last_crc_recv = 0;

/* ===== Motor / parâmetros ===== */
volatile uint8_t  P00 = 0;
volatile uint8_t  P01 = 0;
volatile uint32_t P10 = 15;
volatile uint32_t P11 = 5;
volatile uint8_t  P12 = 1;
volatile uint8_t  P20 = 1;
volatile uint8_t  P21 = 90;
volatile uint8_t  P35 = 0;
volatile uint8_t  P42 = 10;
volatile uint8_t  P43 = 5;
volatile uint8_t  P44 = 0;
volatile uint16_t P45 = 180;
volatile bool params_locked = false;

volatile bool  cmd_ligar_motor = false;
volatile float cmd_frequencia_alvo = 0.0f;
volatile float f_atual = 0.0f;
volatile uint32_t debug_isr_cnt = 0;

static uint32_t backup_P10 = 15;
static uint32_t backup_P11 = 5;
static uint8_t  backup_P12 = 1;
static uint8_t  backup_P20 = 1;
static uint8_t  backup_P21 = 90;
static uint8_t  backup_P35 = 0;
static uint8_t  backup_P42 = 10;

static uint8_t  P20_run = 1;
static uint8_t  P21_run = 90;
static uint32_t P10_run = 15;
static uint32_t P11_run = 5;
static bool     last_motor_state = false;
static uint8_t  last_P42 = 0;

volatile float ramp_inc_up = 0.0f;
volatile float ramp_inc_down = 0.0f;
volatile float inv_fs_2pi = 0.0f;
volatile uint32_t current_arr = 99;
volatile float amp_max_atual = 50.0f;
volatile float freq_ISR = 10000.0f;

static float theta_u = 0.0f;
static float theta_v = 2.094395f;
static float theta_w = 4.188790f;
static const float TWO_PI = 6.283185307f;

/* ===== Periféricos ===== */
volatile uint32_t P30 = 0;
volatile uint32_t P31 = 1;
volatile uint8_t  P32 = 60;
volatile uint8_t  P33 = 0;
volatile uint8_t  P51 = 0;
volatile uint8_t  P80 = 1;
volatile uint8_t  P81 = 1;
volatile uint8_t  P82 = 1;
volatile uint32_t P83 = 1;
volatile uint32_t P84 = 1;
volatile uint8_t  P85 = 1;
volatile uint8_t  P86 = 1;

static bool remote_system_on = false;
static bool remote_start_latched = false;
static bool remote_bomba_cmd = false;
static bool remote_swing_cmd = false;
static bool remote_exaustao_cmd = false;
static dreno_state_t remote_dreno_status = DRENO_IDLE;
static bool motor_reverse = false;

static uint32_t sensor_delay_counter = 0;
static uint32_t tsensor = 5;
static uint8_t  sensor_estavel = 0;
static uint8_t  sensor_ultimo_raw = 0;
static bool     motor_desligado = true;

static GPIO_PinState ligar_bomba = GPIO_PIN_SET;
static GPIO_PinState desligar_bomba = GPIO_PIN_RESET;
static GPIO_PinState ligar_swing = GPIO_PIN_SET;
static GPIO_PinState desligar_swing = GPIO_PIN_RESET;
static GPIO_PinState ligar_dreno = GPIO_PIN_SET;
static GPIO_PinState desligar_dreno = GPIO_PIN_RESET;
static GPIO_PinState ligar_motor_pin = GPIO_PIN_SET;
static GPIO_PinState desligar_motor_pin = GPIO_PIN_RESET;

/* ===== Telemetria simulada / base ===== */
volatile uint16_t sim_v_out = 220;
volatile uint16_t sim_v_bus = 311;
volatile uint16_t sim_i_out = 520;
volatile uint8_t  sim_temp  = 32;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
static inline float fast_sin_approx(float x) {
    while (x >= TWO_PI) x -= TWO_PI;
    while (x < 0.0f) x += TWO_PI;

    float sign = 1.0f;
    if (x > 3.1415926535f) {
        x -= 3.1415926535f;
        sign = -1.0f;
    }

    if (x > 1.5707963268f) {
        x = 3.1415926535f - x;
    }

    float y = (1.27323954f * x) - (0.405284735f * x * x);
    return sign * y;
}

static uint16_t crc16_ibm(const uint8_t *data, uint16_t len);
static void parser_reset(frame_parser_t *p);
static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame);
static void rs485_send_reply(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);
static void process_frame(const frame_t *fr);
static void comm_apply_received_command(void);
static void comm_safe_stop(void);
static void telemetry_update(void);
static uint8_t telemetry_status_flags(void);
static void atualiza_P42(void);
static void load_flash_data(void);
static void write_flash_data(void);
static void calcula_rampa(void);
static void spwm(void);
static void motor_init_runtime(void);
static void motor_task_runtime(void);
static void peripherals_set_modes(void);
static uint8_t sensor_read(void);
static void peripherals_tick_1s(void);
static void peripherals_apply_remote(void);
static void set_output_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t crc16_ibm(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void parser_reset(frame_parser_t *p) {
    p->st = PS_WAIT_SOF;
    p->esc_next = false;
    p->pay_i = 0;
}

static bool parser_feed(frame_parser_t *p, uint8_t byte, frame_t *out_frame) {
    if (byte == SOF) {
        parser_reset(p);
        p->st = PS_HDR_ADDR;
        return false;
    }
    if (p->st == PS_WAIT_SOF) return false;

    if (p->esc_next) {
        byte ^= ESC_XOR;
        p->esc_next = false;
    } else if (byte == ESC) {
        p->esc_next = true;
        return false;
    }

    switch (p->st) {
        case PS_HDR_ADDR: p->addr = byte; p->st = PS_HDR_TYPE; break;
        case PS_HDR_TYPE: p->type = byte; p->st = PS_HDR_SEQ; break;
        case PS_HDR_SEQ:  p->seq  = byte; p->st = PS_HDR_LEN; break;
        case PS_HDR_LEN:
            p->len = byte;
            if (p->len > MAX_PAYLOAD) {
                parser_reset(p);
                return false;
            }
            p->st = (p->len == 0u) ? PS_CRC_L : PS_PAYLOAD;
            break;
        case PS_PAYLOAD:
            p->payload[p->pay_i++] = byte;
            if (p->pay_i >= p->len) p->st = PS_CRC_L;
            break;
        case PS_CRC_L:
            p->crc_l = byte;
            p->st = PS_CRC_H;
            break;
        case PS_CRC_H: {
            p->crc_h = byte;
            uint8_t tmp[4 + MAX_PAYLOAD];
            tmp[0] = p->addr;
            tmp[1] = p->type;
            tmp[2] = p->seq;
            tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);
            uint16_t calc = crc16_ibm(tmp, (uint16_t)(4 + p->len));
            uint16_t recv = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);

            dbg_last_crc_calc = calc;
            dbg_last_crc_recv = recv;

            if (calc == recv) {
                out_frame->addr = p->addr;
                out_frame->type = p->type;
                out_frame->seq = p->seq;
                out_frame->len = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);

                dbg_crc_fail_count++;

                parser_reset(p);
                return true;
            }

            dbg_crc_fail_count++;

            parser_reset(p);
        } break;
        default:
            parser_reset(p);
            break;
    }
    return false;
}

static void rs485_send_reply(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len) {
    uint8_t raw[MAX_PAYLOAD + 10];
    uint8_t esc[(MAX_PAYLOAD + 10) * 2];
    uint16_t r_idx = 0;
    uint16_t e_idx = 0;

    raw[r_idx++] = SOF;
    raw[r_idx++] = ADDR_MASTER;
    raw[r_idx++] = type;
    raw[r_idx++] = seq;
    raw[r_idx++] = len;
    if (len && payload) {
        memcpy(&raw[r_idx], payload, len);
        r_idx += len;
    }

    uint16_t crc = crc16_ibm(&raw[1], (uint16_t)(r_idx - 1));
    raw[r_idx++] = (uint8_t)(crc & 0xFFu);
    raw[r_idx++] = (uint8_t)((crc >> 8) & 0xFFu);

    esc[e_idx++] = SOF;
    for (uint16_t i = 1; i < r_idx; i++) {
        if (raw[i] == SOF || raw[i] == ESC) {
            esc[e_idx++] = ESC;
            esc[e_idx++] = raw[i] ^ ESC_XOR;
        } else {
            esc[e_idx++] = raw[i];
        }
    }

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1, esc, e_idx, 50);
    while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

static void set_output_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    HAL_GPIO_WritePin(port, pin, state);
}

static void peripherals_set_modes(void) {
    ligar_motor_pin = GPIO_PIN_SET;
    desligar_motor_pin = GPIO_PIN_RESET;

    /* BOMBA: P82=1 -> NA, P82=2 -> NF, P82=0 -> bloqueada logicamente */
    ligar_bomba    = (P82 == 2u) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    desligar_bomba = (P82 == 2u) ? GPIO_PIN_SET   : GPIO_PIN_RESET;

    /* SWING: somente P81=1 habilita; o hardware fica em lógica normal. */
    ligar_swing    = GPIO_PIN_SET;
    desligar_swing = GPIO_PIN_RESET;

    /* P80 representa modo de dreno, não polaridade da saída. */
    ligar_dreno    = GPIO_PIN_SET;
    desligar_dreno = GPIO_PIN_RESET;
}

static uint8_t sensor_read(void) {
    if (P85 == 0u) {
        /* Sensor desabilitado: sem confirmação de água, bloqueia climatização. */
        return 0u;
    }

    GPIO_PinState raw = HAL_GPIO_ReadPin(SENSOR_GPIO_Port, SENSOR_Pin);
    if (P85 == 1u) {
        /* NA: nível alto indica água presente. */
        return (raw == GPIO_PIN_SET) ? 1u : 0u;
    }
    /* P85 == 2 -> NF: nível baixo indica água presente. */
    return (raw == GPIO_PIN_RESET) ? 1u : 0u;
}

static void peripherals_tick_1s(void) {
    uint8_t sensor_raw = sensor_read();

    if (sensor_raw != sensor_ultimo_raw) {
        sensor_ultimo_raw = sensor_raw;
        sensor_delay_counter = tsensor;
    } else {
        if (sensor_delay_counter > 0u) {
            sensor_delay_counter--;
        } else {
            sensor_estavel = sensor_raw;
        }
    }
}

static void comm_safe_stop(void) {
    remote_start_latched = false;
    cmd_ligar_motor = false;
    set_output_pin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
    set_output_pin(SWING_GPIO_Port, SWING_Pin, desligar_swing);
    set_output_pin(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);
    if (f_atual <= 0.1f) {
        set_output_pin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor_pin);
    }
}

static void comm_apply_received_command(void) {
    uint8_t btn = g_cmd.buttons;

    if (btn & BTN_BIT_START) remote_start_latched = true;
    if (btn & BTN_BIT_STOP)  remote_start_latched = false;

    remote_system_on    = ((g_cmd.aux_flags & AUX_BIT_SYSTEM_ON) != 0u);
    remote_bomba_cmd    = ((g_cmd.aux_flags & AUX_BIT_BOMBA) != 0u);
    remote_swing_cmd    = ((g_cmd.aux_flags & AUX_BIT_SWING) != 0u);
    remote_exaustao_cmd = ((g_cmd.aux_flags & AUX_BIT_EXAUSTAO) != 0u);
    remote_dreno_status = (g_cmd.dreno_status == 0u) ? DRENO_IDLE :
                          (g_cmd.dreno_status == 1u) ? DRENO_AGUARDANDO_LED : DRENO_EM_CURSO;

    if (!remote_system_on) {
        remote_start_latched = false;
    }

    P51 = (g_cmd.direction != 0u) ? 1u : 0u;
    cmd_frequencia_alvo = ((float)g_cmd.target_freq_centi_hz) / 100.0f;
}

static void peripherals_apply_remote(void) {
    bool dreno_ativo = (remote_dreno_status != DRENO_IDLE) && (P80 != 0u);
    bool sensor_ok = (P85 != 0u) && (sensor_estavel == 1u);
    bool permite_motor = hardware_comms_ok && handshake_done && !g_cmd.e08_active && remote_system_on && remote_start_latched && !dreno_ativo;
    bool bomba_permitida = remote_system_on && remote_bomba_cmd && !remote_exaustao_cmd && !dreno_ativo && (P82 != 0u) && sensor_ok;
    bool swing_permitido = remote_system_on && remote_swing_cmd && !dreno_ativo && (P81 == 1u);

    if (!hardware_comms_ok || !handshake_done || g_cmd.e08_active) {
        dreno_ativo = false;
        bomba_permitida = false;
        swing_permitido = false;
        permite_motor = false;
    }

    if (dreno_ativo) {
        set_output_pin(DRENO_GPIO_Port, DRENO_Pin, ligar_dreno);
        set_output_pin(BOMBA_GPIO_Port, BOMBA_Pin, desligar_bomba);
        set_output_pin(SWING_GPIO_Port, SWING_Pin, desligar_swing);
        cmd_ligar_motor = false;
    } else {
        set_output_pin(DRENO_GPIO_Port, DRENO_Pin, desligar_dreno);
        set_output_pin(BOMBA_GPIO_Port, BOMBA_Pin, bomba_permitida ? ligar_bomba : desligar_bomba);
        set_output_pin(SWING_GPIO_Port, SWING_Pin, swing_permitido ? ligar_swing : desligar_swing);
        cmd_ligar_motor = permite_motor;
    }

    motor_reverse = remote_exaustao_cmd ? !((bool)P51) : ((bool)P51);

    if (cmd_ligar_motor || f_atual > 0.1f) {
        set_output_pin(MOTOR_GPIO_Port, MOTOR_Pin, ligar_motor_pin);
        motor_desligado = false;
    } else {
        set_output_pin(MOTOR_GPIO_Port, MOTOR_Pin, desligar_motor_pin);
        motor_desligado = true;
    }
}

static void atualiza_P42(void) {
    if (P42 == 5u) current_arr = 199u;
    else if (P42 == 15u) current_arr = 66u;
    else {
        P42 = 10u;
        current_arr = 99u;
    }

    amp_max_atual = ((float)(current_arr + 1u)) / 2.0f;
    freq_ISR = 1000000.0f / ((float)(current_arr + 1u));

    __HAL_TIM_SET_AUTORELOAD(&htim1, current_arr);
    __HAL_TIM_SET_AUTORELOAD(&htim3, current_arr);
}

static void load_flash_data(void) {
    SystemParams ram_params;
    uint64_t *pFlash = (uint64_t*)FLASH_USER_ADDR;
    uint64_t dw1 = pFlash[0];
    uint64_t dw2 = pFlash[1];

    uint8_t *pDst = (uint8_t*)&ram_params;
    *(uint64_t*)(pDst) = dw1;
    *(uint64_t*)(pDst + 8) = dw2;

    if (ram_params.saved_P12 == 0xFFu) {
        return;
    }

    if (!isnan(ram_params.saved_freq_hz)) cmd_frequencia_alvo = ram_params.saved_freq_hz;
    P10 = (uint32_t)ram_params.saved_P10;
    P11 = (uint32_t)ram_params.saved_P11;
    P20 = ram_params.saved_P20;
    P21 = ram_params.saved_P21;
    P35 = ram_params.saved_P35;
    P42 = ram_params.saved_P42;
    P12 = ram_params.saved_P12;
    params_locked = (ram_params.saved_locked == 1u);
}

static void write_flash_data(void) {
    __disable_irq();
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    uint32_t page_number = (FLASH_USER_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Page = page_number;
    EraseInitStruct.NbPages = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        SystemParams data;
        data.saved_freq_hz = cmd_frequencia_alvo;
        data.saved_P10 = (uint8_t)P10;
        data.saved_P11 = (uint8_t)P11;
        data.saved_P20 = P20;
        data.saved_P21 = P21;
        data.saved_P35 = P35;
        data.saved_P42 = P42;
        data.saved_P12 = P12;
        data.saved_locked = params_locked ? 1u : 0u;
        data.padding = 0u;

        uint64_t dw1 = *(uint64_t*)((uint8_t*)&data);
        uint64_t dw2 = *(uint64_t*)((uint8_t*)&data + 8);

        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR, dw1);
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_USER_ADDR + 8u, dw2);
    }

    HAL_FLASH_Lock();
    __enable_irq();
}

static void calcula_rampa(void) {
    float t_acc = (float)P10_run;
    float t_dec = (float)P11_run;
    float f_max_ref = cmd_frequencia_alvo;

    if (t_acc < 0.1f) t_acc = 0.1f;
    if (t_dec < 0.1f) t_dec = 0.1f;

    if (last_P42 == 15u) {
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 6.7f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 6.7f);
    } else if (last_P42 == 10u) {
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 3.9f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 3.9f);
    } else {
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 2.2f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 2.2f);
    }

    if (cmd_ligar_motor) {
        float diferenca = fabsf(f_atual - cmd_frequencia_alvo);
        if (diferenca < 0.1f) f_atual = cmd_frequencia_alvo;
    }

    if (!cmd_ligar_motor && f_atual > (cmd_frequencia_alvo - 0.1f)) {
        f_atual = cmd_frequencia_alvo - 0.15f;
    }
}

static void spwm(void) {
    debug_isr_cnt++;

    float alvo = 0.0f;
    float step = 0.0f;
    float amp = 0.0f;

    if (cmd_ligar_motor) {
        alvo = cmd_frequencia_alvo;
        if (alvo > (float)P21_run) alvo = (float)P21_run;
        if (alvo < (float)P20_run) alvo = (float)P20_run;

        if (f_atual < alvo) {
            step = ramp_inc_up;
            f_atual += step;
            if (f_atual > alvo) f_atual = alvo;
        } else if (f_atual > alvo) {
            step = ramp_inc_down;
            f_atual -= step;
            if (f_atual < alvo) f_atual = alvo;
        }
    } else {
        step = ramp_inc_down;
        if (f_atual > 0.0f) {
            f_atual -= step;
            if (f_atual < 0.0f) f_atual = 0.0f;
        }
    }

    P01 = (uint8_t)f_atual;

    if (!cmd_ligar_motor && f_atual <= 0.1f) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        HAL_TIM_Base_Stop_IT(&htim3);
        motor_desligado = true;
        return;
    }

    float inc = 0.0f;
    if (last_P42 == 15u) inc = f_atual * inv_fs_2pi * 6.45f;
    else if (last_P42 == 10u) inc = f_atual * inv_fs_2pi * 4.31f;
    else inc = f_atual * inv_fs_2pi * 2.16f;

    theta_u += inc;
    if (theta_u >= TWO_PI) theta_u -= TWO_PI;
    theta_v = theta_u + 2.094395f;
    if (theta_v >= TWO_PI) theta_v -= TWO_PI;
    theta_w = theta_u + 4.188790f;
    if (theta_w >= TWO_PI) theta_w -= TWO_PI;

    float prop_vf = f_atual * 0.01666f;
    if (prop_vf > 1.0f) prop_vf = 1.0f;

    if (f_atual < 20.0f) amp = amp_max_atual * (1.0f + (P35 / 10.0f));
    else amp = amp_max_atual;

    float amp_atual = prop_vf * amp;
    if (amp_atual > amp_max_atual) amp_atual = amp_max_atual;

    float offset = amp_max_atual - amp_atual;

    uint16_t p1 = (uint16_t)((amp_atual * (fast_sin_approx(theta_u) + 1.0f)) + offset);
    uint16_t p2 = (uint16_t)((amp_atual * (fast_sin_approx(theta_v) + 1.0f)) + offset);
    uint16_t p3 = (uint16_t)((amp_atual * (fast_sin_approx(theta_w) + 1.0f)) + offset);

    if (!motor_reverse) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
    } else {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p3);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p2);
    }
}

static void motor_init_runtime(void) {
    P10 = 15u; P11 = 5u; P12 = 1u;
    P20 = 1u;  P21 = 90u;
    P35 = 0u;  P42 = 10u;
    P43 = 5u;  P44 = 0u; P45 = 180u;
    cmd_frequencia_alvo = 5.0f;
    params_locked = false;

    load_flash_data();

    backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
    backup_P20 = P20; backup_P21 = P21;
    backup_P35 = P35; backup_P42 = P42;

    P10_run = P10;
    P11_run = P11;
    P20_run = P20;
    P21_run = P21;

    if (P42 <= 7u) P42 = 5u;
    else if (P42 <= 12u) P42 = 10u;
    else P42 = 15u;

    last_P42 = 0u;
    f_atual = 0.0f;

    peripherals_set_modes();
    motor_task_runtime();
}

static void motor_task_runtime(void) {
    if (P00 != 0u) {
        if (P00 == 7u) {
            params_locked = !params_locked;
            write_flash_data();
        } else if (P00 == 101u) {
            P10 = 15u; P11 = 5u; P12 = 1u;
            P20 = 1u;  P21 = 90u;
            P35 = 0u;  P42 = 10u;
            cmd_frequencia_alvo = 5.0f;
            params_locked = false;
            backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
            backup_P20 = P20; backup_P21 = P21;
            backup_P35 = P35; backup_P42 = P42;
            write_flash_data();
        }
        P00 = 0u;
    }

    if (params_locked) {
        if (P10 != backup_P10) P10 = backup_P10;
        if (P11 != backup_P11) P11 = backup_P11;
        if (P12 != backup_P12) P12 = backup_P12;
        if (P20 != backup_P20) P20 = backup_P20;
        if (P21 != backup_P21) P21 = backup_P21;
        if (P35 != backup_P35) P35 = backup_P35;
        if (P42 != backup_P42) P42 = backup_P42;
    } else {
        if (P10 < 5u) P10 = 5u; if (P10 > 60u) P10 = 60u;
        if (P11 < 5u) P11 = 5u; if (P11 > 60u) P11 = 60u;
        if (P20 < 1u) P20 = 1u; if (P20 > 24u) P20 = 24u;
        if (P21 < 23u) P21 = 23u; if (P21 > 90u) P21 = 90u;
        if (P35 > 9u) P35 = 9u;
        if (cmd_frequencia_alvo < (float)P20) cmd_frequencia_alvo = (float)P20;
        if (cmd_frequencia_alvo > (float)P21) cmd_frequencia_alvo = (float)P21;

        backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
        backup_P20 = P20; backup_P21 = P21;
        backup_P35 = P35; backup_P42 = P42;
    }

    if (P42 != last_P42) {
        if (P42 <= 7u) P42 = 5u;
        else if (P42 <= 12u) P42 = 10u;
        else P42 = 15u;
        atualiza_P42();
        last_P42 = P42;
    }

    if (!last_motor_state && cmd_ligar_motor) {
        P20_run = P20;
        P21_run = P21;
        P10_run = P10;
        P11_run = P11;
    }

    if (!cmd_ligar_motor) {
        SystemParams flash_data;
        uint64_t *pFlash = (uint64_t*)FLASH_USER_ADDR;
        uint64_t dw1 = pFlash[0];
        uint64_t dw2 = pFlash[1];
        uint8_t *pDst = (uint8_t*)&flash_data;
        *(uint64_t*)(pDst) = dw1;
        *(uint64_t*)(pDst + 8) = dw2;

        bool changed = false;
        if (fabsf(flash_data.saved_freq_hz - cmd_frequencia_alvo) > 0.1f) changed = true;
        if (flash_data.saved_P10 != (uint8_t)P10) changed = true;
        if (flash_data.saved_P11 != (uint8_t)P11) changed = true;
        if (flash_data.saved_P20 != P20) changed = true;
        if (flash_data.saved_P21 != P21) changed = true;
        if (flash_data.saved_P35 != P35) changed = true;
        if (flash_data.saved_P42 != P42) changed = true;
        if (flash_data.saved_P12 != P12) changed = true;
        if (flash_data.saved_locked != (params_locked ? 1u : 0u)) changed = true;

        if (changed && P12 == 1u) {
            write_flash_data();
        }
    }

    last_motor_state = cmd_ligar_motor;
    inv_fs_2pi = TWO_PI / freq_ISR;
    calcula_rampa();

    if (cmd_ligar_motor) {
        if (cmd_frequencia_alvo < (float)P20) cmd_frequencia_alvo = (float)P20;
        if (cmd_frequencia_alvo > (float)P21) cmd_frequencia_alvo = (float)P21;
        if ((TIM3->DIER & TIM_DIER_UIE) == 0u) {
            __HAL_TIM_SET_COUNTER(&htim3, 0u);
            HAL_TIM_Base_Start_IT(&htim3);
        }
    } else {
        if (f_atual <= 0.1f) {
            HAL_TIM_Base_Stop_IT(&htim3);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
        }
    }
}

static uint8_t telemetry_status_flags(void) {
    uint8_t flags = 0u;
    if (P85 != 0u && sensor_estavel == 0u) {
        flags |= MI_STATUS_WATER_SHORTAGE;
    }
    return flags;
}

static void telemetry_update(void) {
    float f_hz = f_atual;
    if (f_hz < 0.0f) f_hz = 0.0f;

    g_tel.current_freq_centi_hz = (uint16_t)(f_hz * 100.0f + 0.5f);

    if (cmd_ligar_motor || f_hz > 0.1f) {
        uint16_t dynamic_current = (uint16_t)(sim_i_out + (uint16_t)(f_hz * 15.0f));
        uint16_t dynamic_vout = (uint16_t)((f_hz * (float)sim_v_out) / 60.0f);
        g_tel.motor_current_ma = dynamic_current;
        g_tel.out_voltage_vrms = dynamic_vout;
    } else {
        g_tel.motor_current_ma = 0u;
        g_tel.out_voltage_vrms = 0u;
    }

    g_tel.bus_voltage_vdc = sim_v_bus;
    g_tel.temp_igbt_c = sim_temp;
}

static void process_frame(const frame_t *fr) {
    if (fr->addr != ADDR_STM32) return;

    last_packet_tick = HAL_GetTick();
    hardware_comms_ok = true;

    switch (fr->type) {
        case TYPE_WRITE_PARAM:
            if (fr->len == 3u) {
                uint8_t p_id = fr->payload[0];
                uint16_t p_val = ((uint16_t)fr->payload[1] << 8) | fr->payload[2];

                switch (p_id) {
                    case 10: g_settings.p10_acel = p_val; P10 = p_val; break;
                    case 11: g_settings.p11_desacel = p_val; P11 = p_val; break;
                    case 20: g_settings.p20_freq_min = p_val; P20 = (uint8_t)p_val; break;
                    case 21: g_settings.p21_freq_max = p_val; P21 = (uint8_t)p_val; break;
                    case 35: g_settings.p35_torque = p_val; P35 = (uint8_t)p_val; break;
                    case 42: g_settings.p42_igbt_khz = p_val; P42 = (uint8_t)p_val; break;
                    case 43: g_settings.p43_i_motor = p_val; P43 = (uint8_t)p_val; break;
                    case 44: g_settings.p44_autoreset = (uint8_t)p_val; P44 = (uint8_t)p_val; break;
                    case 45: g_settings.p45_v_min = p_val; P45 = p_val; break;
                    case 30: P30 = p_val; break;
                    case 31: P31 = p_val; break;
                    case 32: P32 = (uint8_t)p_val; break;
                    case 33: P33 = (uint8_t)p_val; break;
                    case 80: P80 = (uint8_t)p_val; break;
                    case 81: P81 = (uint8_t)p_val; break;
                    case 82: P82 = (uint8_t)p_val; break;
                    case 83: P83 = p_val; break;
                    case 84: P84 = p_val; break;
                    case 85:
                        g_settings.p85_sensor_mode = (uint8_t)p_val;
                        P85 = (uint8_t)p_val;
                        handshake_done = true;
                        break;
                    case 86: P86 = (uint8_t)p_val; break;
                    default: break;
                }

                peripherals_set_modes();
                rs485_send_reply(TYPE_ACK_MI, fr->seq, NULL, 0u);
            }
            break;

        case TYPE_READ_STATUS:
            if (fr->len == 9u) {
                g_cmd.buttons = fr->payload[0];
                g_cmd.target_freq_centi_hz = ((uint16_t)fr->payload[1] << 8) | fr->payload[2];
                g_cmd.direction = fr->payload[3];
                g_cmd.aux_flags = fr->payload[4];
                g_cmd.dreno_status = fr->payload[5];
                g_cmd.p90 = fr->payload[6];
                g_cmd.p91 = fr->payload[7];
                g_cmd.e08_active = fr->payload[8];
                comm_apply_received_command();
            }

            telemetry_update();
            {
                uint8_t resp[10];
                resp[0] = (uint8_t)(g_tel.current_freq_centi_hz >> 8);
                resp[1] = (uint8_t)(g_tel.current_freq_centi_hz & 0xFFu);
                resp[2] = (uint8_t)(g_tel.motor_current_ma >> 8);
                resp[3] = (uint8_t)(g_tel.motor_current_ma & 0xFFu);
                resp[4] = (uint8_t)(g_tel.bus_voltage_vdc >> 8);
                resp[5] = (uint8_t)(g_tel.bus_voltage_vdc & 0xFFu);
                resp[6] = (uint8_t)(g_tel.out_voltage_vrms >> 8);
                resp[7] = (uint8_t)(g_tel.out_voltage_vrms & 0xFFu);
                resp[8] = g_tel.temp_igbt_c;
                resp[9] = telemetry_status_flags();
                rs485_send_reply(TYPE_ACK, fr->seq, resp, 10u);
            }
            break;

        default:
            break;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  parser_reset(&g_parser);
  peripherals_set_modes();
  motor_init_runtime();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim14);

  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, rx_dma_buf[rx_active_idx], RX_DMA_BUF_SZ);
  last_packet_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if ((HAL_GetTick() - last_packet_tick) > 500u) {
        parser_reset(&g_parser);
        if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_ORE);
            HAL_UART_Receive_DMA(&huart1, rx_dma_buf[rx_active_idx], RX_DMA_BUF_SZ);
        }
    }

    uint16_t n = 0;
    uint8_t idx = 0;
    __disable_irq();
    if (rx_dma_ready) {
        rx_dma_ready = false;
        n = rx_ready_len;
        idx = rx_ready_idx;
        if (n > 0u) {
            dbg_main_rx_count++;
            dbg_last_n = n;
        }
    }
    __enable_irq();

    if (n > 0u) {
        frame_t fr;
        for (uint16_t i = 0; i < n; i++) {
            if (parser_feed(&g_parser, rx_dma_buf[idx][i], &fr)) {
                dbg_parser_ok_count++;
                if (fr.type == TYPE_WRITE_PARAM) {
                    dbg_type_write_param_count++;
                }
                process_frame(&fr);
            }
        }
    }

    if ((HAL_GetTick() - last_packet_tick) > COMMS_TIMEOUT_MS) {
        hardware_comms_ok = false;
        comm_safe_stop();
    }

    peripherals_apply_remote();
    motor_task_runtime();
    telemetry_update();

    HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, hardware_comms_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 48-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 96;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 48-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, BOMBA_Pin|MOTOR_Pin|SWING_Pin|DRENO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Button_Pin */
  GPIO_InitStruct.Pin = User_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(User_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SENSOR_Pin */
  GPIO_InitStruct.Pin = SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(SENSOR_GPIO_Port, &GPIO_InitStruct);

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

  /*Configure GPIO pins : BOMBA_Pin MOTOR_Pin SWING_Pin DRENO_Pin */
  GPIO_InitStruct.Pin = BOMBA_Pin|MOTOR_Pin|SWING_Pin|DRENO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    spwm();
  }
  else if (htim->Instance == TIM14)
  {
    peripherals_tick_1s();
  }
}

/*
 * Importante:
 * o tratamento de IDLE/DMA do USART1 deve permanecer no arquivo de interrupções
 * já existente do projeto (stm32c0xx_it.c), como estava no código de comunicação
 * que já funcionava. Este main.c assume que esse handler continua presente.
 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
