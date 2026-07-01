#include "mi_config.h"
#include "mi_comm.h"
#include "mi_state.h"
#include "mi_platform.h"
#include "mi_periph.h"
#include "mi_motor.h"
#include <string.h>

uint8_t rx_dma_buf[2][MI_RX_DMA_BUF_SZ];
volatile uint16_t rx_ready_len = 0;
volatile uint8_t  rx_ready_idx = 0;
volatile bool     rx_dma_ready = false;
volatile uint8_t  rx_active_idx = 0;

static mi_frame_parser_t g_parser;

static uint16_t mi_crc16_ibm(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0u; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void mi_parser_reset(mi_frame_parser_t *p)
{
    p->st = MI_PS_WAIT_SOF;
    p->esc_next = false;
    p->pay_i = 0u;
}

static bool mi_parser_feed(mi_frame_parser_t *p, uint8_t byte, mi_frame_t *out_frame)
{
    if (byte == MI_SOF) {
        mi_parser_reset(p);
        p->st = MI_PS_HDR_ADDR;
        return false;
    }

    if (p->st == MI_PS_WAIT_SOF) {
        return false;
    }

    if (p->esc_next) {
        byte ^= MI_ESC_XOR;
        p->esc_next = false;
    } else if (byte == MI_ESC) {
        p->esc_next = true;
        return false;
    }

    switch (p->st) {
        case MI_PS_HDR_ADDR:
            p->addr = byte;
            p->st = MI_PS_HDR_TYPE;
            break;
        case MI_PS_HDR_TYPE:
            p->type = byte;
            p->st = MI_PS_HDR_SEQ;
            break;
        case MI_PS_HDR_SEQ:
            p->seq = byte;
            p->st = MI_PS_HDR_LEN;
            break;
        case MI_PS_HDR_LEN:
            p->len = byte;
            if (p->len > MI_MAX_PAYLOAD) {
                mi_parser_reset(p);
                return false;
            }
            p->st = (p->len == 0u) ? MI_PS_CRC_L : MI_PS_PAYLOAD;
            break;
        case MI_PS_PAYLOAD:
            p->payload[p->pay_i++] = byte;
            if (p->pay_i >= p->len) {
                p->st = MI_PS_CRC_L;
            }
            break;
        case MI_PS_CRC_L:
            p->crc_l = byte;
            p->st = MI_PS_CRC_H;
            break;
        case MI_PS_CRC_H: {
            p->crc_h = byte;
            uint8_t tmp[4u + MI_MAX_PAYLOAD];
            tmp[0] = p->addr;
            tmp[1] = p->type;
            tmp[2] = p->seq;
            tmp[3] = p->len;
            if (p->len != 0u) {
                memcpy(&tmp[4], p->payload, p->len);
            }

            uint16_t calc = mi_crc16_ibm(tmp, (uint16_t)(4u + p->len));
            uint16_t recv = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);

            dbg_last_crc_calc = calc;
            dbg_last_crc_recv = recv;

            if (calc == recv) {
                out_frame->addr = p->addr;
                out_frame->type = p->type;
                out_frame->seq = p->seq;
                out_frame->len = p->len;
                if (p->len != 0u) {
                    memcpy(out_frame->payload, p->payload, p->len);
                }
                mi_parser_reset(p);
                return true;
            }

            dbg_crc_fail_count++;
            mi_parser_reset(p);
            break;
        }
        default:
            mi_parser_reset(p);
            break;
    }

    return false;
}

static void mi_rs485_send_reply(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint8_t raw[MI_MAX_PAYLOAD + 10u];
    uint8_t esc[(MI_MAX_PAYLOAD + 10u) * 2u];
    uint16_t r_idx = 0u;
    uint16_t e_idx = 0u;

    if (mi_huart == NULL) {
        return;
    }

    raw[r_idx++] = MI_SOF;
    raw[r_idx++] = MI_ADDR_MASTER;
    raw[r_idx++] = type;
    raw[r_idx++] = seq;
    raw[r_idx++] = len;

    if (len != 0u && payload != NULL) {
        memcpy(&raw[r_idx], payload, len);
        r_idx += len;
    }

    uint16_t crc = mi_crc16_ibm(&raw[1], (uint16_t)(r_idx - 1u));
    raw[r_idx++] = (uint8_t)(crc & 0xFFu);
    raw[r_idx++] = (uint8_t)((crc >> 8) & 0xFFu);

    esc[e_idx++] = MI_SOF;
    for (uint16_t i = 1u; i < r_idx; i++) {
        if (raw[i] == MI_SOF || raw[i] == MI_ESC) {
            esc[e_idx++] = MI_ESC;
            esc[e_idx++] = raw[i] ^ MI_ESC_XOR;
        } else {
            esc[e_idx++] = raw[i];
        }
    }

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(mi_huart, esc, e_idx, 50u);
    while (__HAL_UART_GET_FLAG(mi_huart, UART_FLAG_TC) == RESET) {}
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

static uint8_t mi_telemetry_status_flags(void)
{
    uint8_t flags = 0u;
    if (P85 != 0u && sensor_estavel == 0u) {
        flags |= MI_STATUS_WATER_SHORTAGE;
    }
    return flags;
}

void mi_comm_apply_received_command(void)
{
    uint8_t btn = g_cmd.buttons;

    if (btn & MI_BTN_BIT_START) remote_start_latched = true;
    if (btn & MI_BTN_BIT_STOP)  remote_start_latched = false;

    remote_system_on    = ((g_cmd.aux_flags & MI_AUX_BIT_SYSTEM_ON) != 0u);
    remote_bomba_cmd    = ((g_cmd.aux_flags & MI_AUX_BIT_BOMBA) != 0u);
    remote_swing_cmd    = ((g_cmd.aux_flags & MI_AUX_BIT_SWING) != 0u);
    remote_exaustao_cmd = ((g_cmd.aux_flags & MI_AUX_BIT_EXAUSTAO) != 0u);
    remote_dreno_status = (g_cmd.dreno_status == 0u) ? DRENO_IDLE :
                          (g_cmd.dreno_status == 1u) ? DRENO_AGUARDANDO_LED : DRENO_EM_CURSO;

    if (!remote_system_on) {
        remote_start_latched = false;
    }

    P51 = (g_cmd.direction != 0u) ? 1u : 0u;
    cmd_frequencia_alvo = ((float)g_cmd.target_freq_centi_hz) / 100.0f;
}

static void mi_telemetry_update(void)
{
    float f_hz = f_atual;
    if (f_hz < 0.0f) f_hz = 0.0f;

    g_tel.current_freq_centi_hz = (uint16_t)(f_hz * 100.0f + 0.5f);

    if (cmd_ligar_motor || f_hz > 0.1f) {
        uint16_t dynamic_vout = (uint16_t)((f_hz * (float)sim_v_out) / 60.0f);
        g_tel.motor_current_ma =
            (uint16_t)(g_analog.current_a_filt * 1000.0f + 0.5f);
        g_tel.out_voltage_vrms = dynamic_vout;
    } else {
#if MI_CURRENT_BENCH_TEST_MODE
        g_tel.motor_current_ma =
            (uint16_t)(g_analog.current_a_filt * 1000.0f + 0.5f);
#else
        g_tel.motor_current_ma = 0u;
#endif
        g_tel.out_voltage_vrms = 0u;
    }

    g_tel.bus_voltage_vdc = (uint16_t)(g_analog.vbus_v_filt + 0.5f);
    g_tel.temp_igbt_c = (uint8_t)(g_analog.temp_c_filt + 0.5f);
}

void mi_comm_send_ack_mi(uint8_t seq)
{
    mi_rs485_send_reply(MI_TYPE_ACK_MI, seq, NULL, 0u);
}

void mi_comm_send_status_reply(uint8_t seq)
{
    uint8_t resp[10];

    mi_telemetry_update();

    resp[0] = (uint8_t)(g_tel.current_freq_centi_hz >> 8);
    resp[1] = (uint8_t)(g_tel.current_freq_centi_hz & 0xFFu);
    resp[2] = (uint8_t)(g_tel.motor_current_ma >> 8);
    resp[3] = (uint8_t)(g_tel.motor_current_ma & 0xFFu);
    resp[4] = (uint8_t)(g_tel.bus_voltage_vdc >> 8);
    resp[5] = (uint8_t)(g_tel.bus_voltage_vdc & 0xFFu);
    resp[6] = (uint8_t)(g_tel.out_voltage_vrms >> 8);
    resp[7] = (uint8_t)(g_tel.out_voltage_vrms & 0xFFu);
    resp[8] = g_tel.temp_igbt_c;
    resp[9] = mi_telemetry_status_flags();

    mi_rs485_send_reply(MI_TYPE_ACK, seq, resp, 10u);
}

static void mi_process_frame(const mi_frame_t *fr)
{
    if (fr->addr != MI_ADDR_STM32) {
        return;
    }

    last_packet_tick = HAL_GetTick();
    hardware_comms_ok = true;

    switch (fr->type) {
        case MI_TYPE_WRITE_PARAM:
            if (fr->len == 3u) {
                uint8_t p_id = fr->payload[0];
                uint16_t p_val = ((uint16_t)fr->payload[1] << 8) | fr->payload[2];

                switch (p_id) {
                    case 0:  P00 = (uint8_t)p_val; break;
                    case 10: g_settings.p10_acel = p_val; P10 = p_val; break;
                    case 11: g_settings.p11_desacel = p_val; P11 = p_val; break;
                    case 20: g_settings.p20_freq_min = p_val; P20 = (uint8_t)p_val; break;
                    case 21: g_settings.p21_freq_max = p_val; P21 = (uint8_t)p_val; break;
                    case 35: g_settings.p35_torque = p_val; P35 = (uint8_t)p_val; break;
                    case 42: g_settings.p42_igbt_khz = p_val; P42 = (uint8_t)p_val; break;
                    case 43: g_settings.p43_i_motor = p_val; P43 = (uint8_t)p_val; break;
                    case 44: g_settings.p44_autoreset = (uint8_t)p_val; P44 = (uint8_t)p_val; break;
                    case 45: g_settings.p45_v_min = p_val; P45 = p_val; break;
                    case 51: P51 = (uint8_t)p_val; break;
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
                        sensor_ultimo_raw = mi_periph_sensor_read();
                        sensor_estavel = sensor_ultimo_raw;
                        sensor_delay_counter = 0u;
                        handshake_done = true;
                        break;
                    case 86: P86 = (uint8_t)p_val; break;
                    default: break;
                }

                mi_periph_set_modes();
                mi_comm_send_ack_mi(fr->seq);
            }
            break;

        case MI_TYPE_READ_STATUS:
            if (fr->len == 9u) {
                g_cmd.buttons = fr->payload[0];
                g_cmd.target_freq_centi_hz = ((uint16_t)fr->payload[1] << 8) | fr->payload[2];
                g_cmd.direction = fr->payload[3];
                g_cmd.aux_flags = fr->payload[4];
                g_cmd.dreno_status = fr->payload[5];
                g_cmd.p90 = fr->payload[6];
                g_cmd.p91 = fr->payload[7];
                g_cmd.e08_active = fr->payload[8];
                mi_comm_apply_received_command();
            }
            mi_comm_send_status_reply(fr->seq);
            break;

        default:
            break;
    }
}

void mi_comm_init(void)
{
    mi_parser_reset(&g_parser);
}

void mi_comm_start_rx(void)
{
    if (mi_huart == NULL) return;

    rx_ready_len = 0u;
    rx_ready_idx = 0u;
    rx_dma_ready = false;
    rx_active_idx = 0u;

    __HAL_UART_ENABLE_IT(mi_huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(mi_huart, rx_dma_buf[rx_active_idx], MI_RX_DMA_BUF_SZ);
    last_packet_tick = HAL_GetTick();
}

void mi_comm_uart_irq_handler(UART_HandleTypeDef *huart)
{
    if (mi_huart == NULL || huart != mi_huart) {
        return;
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        uint16_t n = 0u;
        if (huart->hdmarx != NULL) {
            n = (uint16_t)(MI_RX_DMA_BUF_SZ - __HAL_DMA_GET_COUNTER(huart->hdmarx));
        }

        HAL_UART_DMAStop(huart);

        if (n > 0u && n <= MI_RX_DMA_BUF_SZ) {
            rx_ready_len = n;
            rx_ready_idx = rx_active_idx;
            rx_dma_ready = true;
            rx_active_idx ^= 1u;
        }

        HAL_UART_Receive_DMA(huart, rx_dma_buf[rx_active_idx], MI_RX_DMA_BUF_SZ);
    }
}

void mi_comm_process_rx(void)
{
    uint16_t n = 0u;
    uint8_t idx = 0u;

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
        mi_frame_t fr;
        for (uint16_t i = 0u; i < n; i++) {
            if (mi_parser_feed(&g_parser, rx_dma_buf[idx][i], &fr)) {
                dbg_parser_ok_count++;
                if (fr.type == MI_TYPE_WRITE_PARAM) {
                    dbg_type_write_param_count++;
                }
                mi_process_frame(&fr);
            }
        }
    }
}
