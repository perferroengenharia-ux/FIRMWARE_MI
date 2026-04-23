#include "mi_comm.h"
#include "mi_peripherals.h"
#include "mi_motor.h"
#include "mi_sensors.h"
#include <string.h>

static void mi_parser_reset(mi_frame_parser_t *p);
static bool mi_parser_feed(mi_frame_parser_t *p, uint8_t byte, mi_frame_t *out_frame);
static void mi_rs485_send_reply(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len);
static void mi_process_frame(const mi_frame_t *fr);

uint16_t mi_crc16_ibm(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 1u) ? (uint16_t)((crc >> 1) ^ 0xA001u) : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}

static void mi_parser_reset(mi_frame_parser_t *p) {
    p->st = MI_PS_WAIT_SOF;
    p->esc_next = false;
    p->pay_i = 0;
}

static bool mi_parser_feed(mi_frame_parser_t *p, uint8_t byte, mi_frame_t *out_frame) {
    if (byte == MI_SOF) {
        mi_parser_reset(p);
        p->st = MI_PS_HDR_ADDR;
        return false;
    }
    if (p->st == MI_PS_WAIT_SOF) return false;

    if (p->esc_next) {
        byte ^= MI_ESC_XOR;
        p->esc_next = false;
    } else if (byte == MI_ESC) {
        p->esc_next = true;
        return false;
    }

    switch (p->st) {
        case MI_PS_HDR_ADDR: p->addr = byte; p->st = MI_PS_HDR_TYPE; break;
        case MI_PS_HDR_TYPE: p->type = byte; p->st = MI_PS_HDR_SEQ; break;
        case MI_PS_HDR_SEQ:  p->seq  = byte; p->st = MI_PS_HDR_LEN; break;
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
            if (p->pay_i >= p->len) p->st = MI_PS_CRC_L;
            break;
        case MI_PS_CRC_L:
            p->crc_l = byte;
            p->st = MI_PS_CRC_H;
            break;
        case MI_PS_CRC_H: {
            p->crc_h = byte;
            uint8_t tmp[4 + MI_MAX_PAYLOAD];
            tmp[0] = p->addr;
            tmp[1] = p->type;
            tmp[2] = p->seq;
            tmp[3] = p->len;
            if (p->len) memcpy(&tmp[4], p->payload, p->len);
            uint16_t calc = mi_crc16_ibm(tmp, (uint16_t)(4 + p->len));
            uint16_t recv = (uint16_t)p->crc_l | ((uint16_t)p->crc_h << 8);

            dbg_last_crc_calc = calc;
            dbg_last_crc_recv = recv;

            if (calc == recv) {
                out_frame->addr = p->addr;
                out_frame->type = p->type;
                out_frame->seq = p->seq;
                out_frame->len = p->len;
                if (p->len) memcpy(out_frame->payload, p->payload, p->len);
                mi_parser_reset(p);
                return true;
            }
            dbg_crc_fail_count++;
            mi_parser_reset(p);
        } break;
        default:
            mi_parser_reset(p);
            break;
    }
    return false;
}

static void mi_rs485_send_reply(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len) {
    uint8_t raw[MI_MAX_PAYLOAD + 12];
    uint8_t esc[(MI_MAX_PAYLOAD + 12) * 2u];
    uint16_t r_idx = 0;
    uint16_t e_idx = 0;

    raw[r_idx++] = MI_SOF;
    raw[r_idx++] = MI_ADDR_MASTER;
    raw[r_idx++] = type;
    raw[r_idx++] = seq;
    raw[r_idx++] = len;
    if (len && payload) {
        memcpy(&raw[r_idx], payload, len);
        r_idx += len;
    }

    uint16_t crc = mi_crc16_ibm(&raw[1], (uint16_t)(r_idx - 1));
    raw[r_idx++] = (uint8_t)(crc & 0xFFu);
    raw[r_idx++] = (uint8_t)((crc >> 8) & 0xFFu);

    esc[e_idx++] = MI_SOF;
    for (uint16_t i = 1; i < r_idx; i++) {
        if (raw[i] == MI_SOF || raw[i] == MI_ESC) {
            esc[e_idx++] = MI_ESC;
            esc[e_idx++] = raw[i] ^ MI_ESC_XOR;
        } else {
            esc[e_idx++] = raw[i];
        }
    }

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, esc, e_idx, 50);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) == RESET) {}
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

static void mi_process_frame(const mi_frame_t *fr) {
    if (fr->addr != MI_ADDR_STM32) return;

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
                    case 30: P30 = p_val; break;
                    case 31: P31 = p_val; break;
                    case 32: P32 = (uint8_t)p_val; break;
                    case 33: P33 = (uint8_t)p_val; break;
                    case 51: P51 = (uint8_t)p_val; break;
                    case 80: P80 = (uint8_t)p_val; break;
                    case 81: P81 = (uint8_t)p_val; break;
                    case 82: P82 = (uint8_t)p_val; break;
                    case 83: P83 = p_val; break;
                    case 84: P84 = p_val; break;
                    case 85: g_settings.p85_sensor_mode = (uint8_t)p_val; P85 = (uint8_t)p_val; handshake_done = true; break;
                    case 86: P86 = (uint8_t)p_val; break;
                    default: break;
                }

                mi_periph_set_modes();
                mi_rs485_send_reply(MI_TYPE_ACK_MI, fr->seq, NULL, 0u);
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

            mi_telemetry_update();
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
                resp[8] = (uint8_t)(g_sensors.temp_c_filt + 0.5f);
                resp[9] = g_tel.status_flags;
                mi_rs485_send_reply(MI_TYPE_ACK, fr->seq, resp, 10u);
            }
            break;

        default:
            break;
    }
}

void mi_comm_init(void) {
    mi_parser_reset(&g_parser);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart2, rx_dma_buf[rx_active_idx], MI_RX_DMA_BUF_SZ);
    last_packet_tick = HAL_GetTick();
}

void mi_comm_process(void) {
    if ((HAL_GetTick() - last_packet_tick) > 500u) {
        mi_parser_reset(&g_parser);
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(&huart2);
            HAL_UART_Receive_DMA(&huart2, rx_dma_buf[rx_active_idx], MI_RX_DMA_BUF_SZ);
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
        mi_frame_t fr;
        for (uint16_t i = 0; i < n; i++) {
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
