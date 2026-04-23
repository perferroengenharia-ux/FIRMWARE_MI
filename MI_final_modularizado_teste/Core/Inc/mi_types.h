#ifndef MI_TYPES_H
#define MI_TYPES_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#define MI_SOF                      0x7E
#define MI_ESC                      0x7D
#define MI_ESC_XOR                  0x20

#define MI_ADDR_STM32               0x01
#define MI_ADDR_MASTER              0xF0

#define MI_TYPE_READ_STATUS         0x04
#define MI_TYPE_WRITE_PARAM         0x05
#define MI_TYPE_ACK_MI              0x06
#define MI_TYPE_ACK                 0x80

#define MI_MAX_PAYLOAD              128u
#define MI_RX_DMA_BUF_SZ            256u
#define MI_COMMS_TIMEOUT_MS         2000u
#define MI_STATUS_WATER_SHORTAGE    (1u << 0)

#define MI_BTN_BIT_START            (1u << 0)
#define MI_BTN_BIT_STOP             (1u << 1)
#define MI_BTN_BIT_UP               (1u << 2)
#define MI_BTN_BIT_DOWN             (1u << 3)

#define MI_AUX_BIT_BOMBA            (1u << 0)
#define MI_AUX_BIT_SWING            (1u << 1)
#define MI_AUX_BIT_EXAUSTAO         (1u << 2)
#define MI_AUX_BIT_DRENO            (1u << 3)
#define MI_AUX_BIT_SYSTEM_ON        (1u << 4)

#ifndef MI_FLASH_USER_ADDR
#define MI_FLASH_USER_ADDR          0x0800F800u /* last 2 KB page on STM32F301C8T6 (64 KB flash) */
#endif

#define MI_CLAMP_F(v, lo, hi) (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))

typedef struct {
    uint8_t addr, type, seq, len;
    uint8_t payload[MI_MAX_PAYLOAD];
} mi_frame_t;

typedef enum {
    MI_PS_WAIT_SOF = 0,
    MI_PS_HDR_ADDR,
    MI_PS_HDR_TYPE,
    MI_PS_HDR_SEQ,
    MI_PS_HDR_LEN,
    MI_PS_PAYLOAD,
    MI_PS_CRC_L,
    MI_PS_CRC_H
} mi_parse_state_t;

typedef struct {
    mi_parse_state_t st;
    bool esc_next;
    uint8_t addr, type, seq, len;
    uint8_t payload[MI_MAX_PAYLOAD];
    uint8_t pay_i, crc_l, crc_h;
} mi_frame_parser_t;

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
    uint8_t  status_flags;
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
} mi_system_params_t;

typedef enum {
    MI_DRENO_IDLE = 0,
    MI_DRENO_AGUARDANDO_LED = 1,
    MI_DRENO_EM_CURSO = 2
} mi_dreno_state_t;

#endif
