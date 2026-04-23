#include "mi_motor.h"
#include "mi_peripherals.h"
#include "mi_sensors.h"
#include <string.h>
#include <math.h>

static inline float fast_sin_approx(float x) {
    while (x >= MI_TWO_PI) x -= MI_TWO_PI;
    while (x < 0.0f) x += MI_TWO_PI;

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

static void mi_motor_update_p42(void);
static void mi_motor_load_flash(void);
static void mi_motor_write_flash(void);
static void mi_motor_calc_ramp(void);
static void mi_motor_spwm(void);

static void mi_flash_program_halfwords(uint32_t address, const uint16_t *data, uint32_t count) {
    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = MI_FLASH_USER_ADDR;
    erase.NbPages = 1;
    (void)HAL_FLASHEx_Erase(&erase, &page_error);

    for (uint32_t i = 0; i < count; i++) {
        (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + (i * 2u), data[i]);
    }

    HAL_FLASH_Lock();
}

static void mi_motor_update_p42(void) {
    if (P42 == 5u) current_arr = 199u;
    else if (P42 == 15u) current_arr = 66u;
    else {
        P42 = 10u;
        current_arr = 99u;
    }

    amp_max_atual = ((float)(current_arr + 1u)) / 2.0f;
    freq_ISR = 1000000.0f / ((float)(current_arr + 1u));

    __HAL_TIM_SET_AUTORELOAD(&htim1, current_arr);
    __HAL_TIM_SET_AUTORELOAD(&htim2, current_arr);
}

static void mi_motor_load_flash(void) {
    mi_system_params_t ram_params;
    memcpy(&ram_params, (void *)MI_FLASH_USER_ADDR, sizeof(ram_params));

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

static void mi_motor_write_flash(void) {
    mi_system_params_t data;
    memset(&data, 0xFF, sizeof(data));
    data.saved_freq_hz = cmd_frequencia_alvo;
    data.saved_P10 = (uint8_t)P10;
    data.saved_P11 = (uint8_t)P11;
    data.saved_P20 = P20;
    data.saved_P21 = P21;
    data.saved_P35 = P35;
    data.saved_P42 = P42;
    data.saved_P12 = P12;
    data.saved_locked = params_locked ? 1u : 0u;
    mi_flash_program_halfwords(MI_FLASH_USER_ADDR, (const uint16_t *)&data, sizeof(data) / 2u);
}

static void mi_motor_calc_ramp(void) {
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

static void mi_motor_spwm(void) {
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
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
        HAL_TIM_Base_Stop_IT(&htim2);
        motor_desligado = true;
        return;
    }

    float inc = 0.0f;
    if (last_P42 == 15u) inc = f_atual * inv_fs_2pi * 6.45f;
    else if (last_P42 == 10u) inc = f_atual * inv_fs_2pi * 4.31f;
    else inc = f_atual * inv_fs_2pi * 2.16f;

    theta_u += inc;
    if (theta_u >= MI_TWO_PI) theta_u -= MI_TWO_PI;
    theta_v = theta_u + 2.094395f;
    if (theta_v >= MI_TWO_PI) theta_v -= MI_TWO_PI;
    theta_w = theta_u + 4.188790f;
    if (theta_w >= MI_TWO_PI) theta_w -= MI_TWO_PI;

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

void mi_motor_factory_reset_defaults(void) {
    P10 = 15u; P11 = 5u; P12 = 1u;
    P20 = 1u;  P21 = 90u;
    P35 = 0u;  P42 = 10u;
    P43 = 5u;  P44 = 0u; P45 = 180u;
    P30 = 0u;  P31 = 1u; P32 = 60u; P33 = 0u;
    P51 = 0u;
    P80 = 1u; P81 = 1u; P82 = 1u; P83 = 1u; P84 = 1u; P85 = 1u; P86 = 1u;
    cmd_frequencia_alvo = 5.0f;
    params_locked = false;
}

void mi_motor_init_runtime(void) {
    mi_motor_factory_reset_defaults();
    mi_motor_load_flash();

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
    mi_periph_set_modes();
    mi_motor_task_runtime();
}

void mi_motor_task_runtime(void) {
    if (P00 != 0u) {
        if (P00 == 7u) {
            params_locked = !params_locked;
            mi_motor_write_flash();
        } else if (P00 == 101u) {
            mi_motor_factory_reset_defaults();
            backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
            backup_P20 = P20; backup_P21 = P21;
            backup_P35 = P35; backup_P42 = P42;
            mi_motor_write_flash();
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
    	if (P10 < 5u)  P10 = 5u;
    	if (P10 > 60u) P10 = 60u;
    	if (P11 < 5u)  P11 = 5u;
    	if (P11 > 60u) P11 = 60u;
    	if (P20 < 1u)  P20 = 1u;
    	if (P20 > 24u) P20 = 24u;
    	if (P21 < 23u) P21 = 23u;
    	if (P21 > 90u) P21 = 90u;
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
        mi_motor_update_p42();
        last_P42 = P42;
    }

    if (!last_motor_state && cmd_ligar_motor) {
        P20_run = P20;
        P21_run = P21;
        P10_run = P10;
        P11_run = P11;
    }

    if (!cmd_ligar_motor) {
        mi_system_params_t flash_data;
        memcpy(&flash_data, (void *)MI_FLASH_USER_ADDR, sizeof(flash_data));
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
            mi_motor_write_flash();
        }
    }

    last_motor_state = cmd_ligar_motor;
    inv_fs_2pi = MI_TWO_PI / freq_ISR;
    mi_motor_calc_ramp();

    if (cmd_ligar_motor) {
        if (cmd_frequencia_alvo < (float)P20) cmd_frequencia_alvo = (float)P20;
        if (cmd_frequencia_alvo > (float)P21) cmd_frequencia_alvo = (float)P21;
        if ((TIM2->DIER & TIM_DIER_UIE) == 0u) {
            __HAL_TIM_SET_COUNTER(&htim2, 0u);
            HAL_TIM_Base_Start_IT(&htim2);
        }
    } else {
        if (f_atual <= 0.1f) {
            HAL_TIM_Base_Stop_IT(&htim2);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0u);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0u);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0u);
        }
    }
}

uint8_t mi_telemetry_status_flags(void) {
    uint8_t flags = 0u;
    if ((P85 != 0u) && (sensor_estavel == 0u)) {
        flags |= MI_STATUS_WATER_SHORTAGE;
    }
    return flags;
}

void mi_telemetry_update(void) {
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
    g_tel.temp_igbt_c = (uint8_t)(g_sensors.temp_c_filt + 0.5f);
    g_tel.status_flags = mi_telemetry_status_flags();
}

void mi_motor_on_tim2_irq(void) {
    mi_motor_spwm();
}
