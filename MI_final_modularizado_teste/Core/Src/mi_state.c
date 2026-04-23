#include "mi_state.h"

mi_frame_parser_t g_parser = {0};
mi_settings_t  g_settings = {0};
mi_commands_t  g_cmd      = {0};
mi_telemetry_t g_tel      = {0};

uint32_t last_packet_tick = 0;
bool hardware_comms_ok = false;
bool handshake_done = false;

uint8_t rx_dma_buf[2][MI_RX_DMA_BUF_SZ] = {{0}};
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

volatile float ramp_inc_up = 0.0f;
volatile float ramp_inc_down = 0.0f;
volatile float inv_fs_2pi = 0.0f;
volatile uint32_t current_arr = 99;
volatile float amp_max_atual = 50.0f;
volatile float freq_ISR = 10000.0f;

uint32_t backup_P10 = 15;
uint32_t backup_P11 = 5;
uint8_t  backup_P12 = 1;
uint8_t  backup_P20 = 1;
uint8_t  backup_P21 = 90;
uint8_t  backup_P35 = 0;
uint8_t  backup_P42 = 10;
uint8_t  P20_run = 1;
uint8_t  P21_run = 90;
uint32_t P10_run = 15;
uint32_t P11_run = 5;
bool     last_motor_state = false;
uint8_t  last_P42 = 0;
float theta_u = 0.0f;
float theta_v = 2.094395f;
float theta_w = 4.188790f;
const float MI_TWO_PI = 6.283185307f;

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

bool remote_system_on = false;
bool remote_start_latched = false;
bool remote_bomba_cmd = false;
bool remote_swing_cmd = false;
bool remote_exaustao_cmd = false;
mi_dreno_state_t remote_dreno_status = MI_DRENO_IDLE;
bool motor_reverse = false;

uint32_t sensor_delay_counter = 0;
uint32_t tsensor = 5;
uint8_t  sensor_estavel = 0;
uint8_t  sensor_ultimo_raw = 0;
bool     motor_desligado = true;

GPIO_PinState ligar_bomba = GPIO_PIN_SET;
GPIO_PinState desligar_bomba = GPIO_PIN_RESET;
GPIO_PinState ligar_swing = GPIO_PIN_SET;
GPIO_PinState desligar_swing = GPIO_PIN_RESET;
GPIO_PinState ligar_dreno = GPIO_PIN_SET;
GPIO_PinState desligar_dreno = GPIO_PIN_RESET;
GPIO_PinState ligar_motor_pin = GPIO_PIN_SET;
GPIO_PinState desligar_motor_pin = GPIO_PIN_RESET;

volatile uint16_t sim_v_out = 220;
volatile uint16_t sim_v_bus = 311;
volatile uint16_t sim_i_out = 520;
volatile uint8_t  sim_temp  = 32;
