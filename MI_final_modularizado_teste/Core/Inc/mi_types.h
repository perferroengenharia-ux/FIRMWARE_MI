#ifndef MI_TYPES_H
#define MI_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include "mi_config.h"

typedef struct {
    uint8_t addr;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
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
    uint8_t addr;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[MI_MAX_PAYLOAD];
    uint8_t pay_i;
    uint8_t crc_l;
    uint8_t crc_h;
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
    DRENO_IDLE = 0,
    DRENO_AGUARDANDO_LED = 1,
    DRENO_EM_CURSO = 2
} dreno_state_t;

/* Aliases para compatibilidade com versões intermediárias. */
#define MI_DRENO_IDLE DRENO_IDLE
#define MI_DRENO_AGUARDANDO_LED DRENO_AGUARDANDO_LED
#define MI_DRENO_EM_CURSO DRENO_EM_CURSO

typedef struct {
    uint16_t temp_raw;
    uint16_t vbus_raw;
    uint16_t current_raw;

    float temp_adc_v;
    float temp_c;
    float temp_c_filt;

    float vbus_adc_v;
    float vbus_v;
    float vbus_v_filt;

    float current_adc_v;
    float current_offset_counts;
    float current_offset_v;
    float current_a;
    float current_a_filt;
} mi_analog_t;

#endif /* MI_TYPES_H */
