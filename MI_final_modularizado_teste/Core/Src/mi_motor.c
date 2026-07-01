#include "mi_motor.h"
#include "mi_state.h"
#include "mi_platform.h"
#include "mi_storage.h"
#include "mi_periph.h"
#include <math.h>

static inline float mi_fast_sin_approx(float x)
{
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

static void mi_motor_update_p42(void)
{
    if (P42 == 5u) current_arr = 199u;
    else if (P42 == 15u) current_arr = 66u;
    else {
        P42 = 10u;
        current_arr = 99u;
    }

    amp_max_atual = ((float)(current_arr + 1u)) / 2.0f;
    freq_ISR = MI_TIMER_TICK_HZ / ((float)(current_arr + 1u));

    if (mi_htim_pwm != NULL) {
        __HAL_TIM_SET_AUTORELOAD(mi_htim_pwm, current_arr);
    }
    if (mi_htim_spwm != NULL) {
        __HAL_TIM_SET_AUTORELOAD(mi_htim_spwm, current_arr);
    }
}

static void mi_motor_calc_ramp(void)
{
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

void mi_motor_spwm_isr(void)
{
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
        if (mi_htim_pwm != NULL) {
            __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_1, 0u);
            __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_2, 0u);
            __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_3, 0u);
        }
        if (mi_htim_spwm != NULL) {
            HAL_TIM_Base_Stop_IT(mi_htim_spwm);
        }
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

    uint16_t p1 = (uint16_t)((amp_atual * (mi_fast_sin_approx(theta_u) + 1.0f)) + offset);
    uint16_t p2 = (uint16_t)((amp_atual * (mi_fast_sin_approx(theta_v) + 1.0f)) + offset);
    uint16_t p3 = (uint16_t)((amp_atual * (mi_fast_sin_approx(theta_w) + 1.0f)) + offset);

    if (mi_htim_pwm == NULL) return;

    if (!motor_reverse) {
        __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_1, p1);
        __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_2, p2);
        __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_3, p3);
    } else {
        __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_1, p1);
        __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_2, p3);
        __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_3, p2);
    }
}

void mi_motor_init_runtime(void)
{
    P10 = 15u; P11 = 5u; P12 = 1u;
    P20 = 1u;  P21 = 90u;
    P35 = 0u;  P42 = 10u;
    P43 = 5u;  P44 = 0u; P45 = 180u;
    cmd_frequencia_alvo = 5.0f;
    params_locked = false;

    mi_storage_load();

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
    sensor_ultimo_raw = mi_periph_sensor_read();
    sensor_estavel = sensor_ultimo_raw;
    sensor_delay_counter = 0u;

    mi_motor_task_runtime();
}

void mi_motor_task_runtime(void)
{
    if (P00 != 0u) {
        if (P00 == 7u) {
            params_locked = !params_locked;
            mi_storage_write();
        } else if (P00 == 101u) {
            P10 = 15u; P11 = 5u; P12 = 1u;
            P20 = 1u;  P21 = 90u;
            P30 = 10u; P31 = 5u; P32 = 30u; P33 = 0u;
            P35 = 0u;  P42 = 10u;
            P43 = 5u;  P44 = 0u; P45 = 180u;
            P51 = 0u;
            P80 = 0u;  P81 = 1u; P82 = 1u; P83 = 10u; P84 = 5u; P85 = 1u; P86 = 1u;
            cmd_frequencia_alvo = 5.0f;
            params_locked = false;
            backup_P10 = P10; backup_P11 = P11; backup_P12 = P12;
            backup_P20 = P20; backup_P21 = P21;
            backup_P35 = P35; backup_P42 = P42;
            mi_periph_set_modes();
            sensor_ultimo_raw = mi_periph_sensor_read();
            sensor_estavel = sensor_ultimo_raw;
            sensor_delay_counter = 0u;
            mi_storage_write();
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
        if (P10 < 5u) P10 = 5u;
        if (P10 > 60u) P10 = 60u;
        if (P11 < 5u) P11 = 5u;
        if (P11 > 60u) P11 = 60u;
        if (P20 < 1u) P20 = 1u;
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

    last_motor_state = cmd_ligar_motor;
    inv_fs_2pi = TWO_PI / freq_ISR;
    mi_motor_calc_ramp();

    if (cmd_ligar_motor) {
        if (cmd_frequencia_alvo < (float)P20) cmd_frequencia_alvo = (float)P20;
        if (cmd_frequencia_alvo > (float)P21) cmd_frequencia_alvo = (float)P21;

        if (mi_htim_spwm != NULL && ((mi_htim_spwm->Instance->DIER & TIM_DIER_UIE) == 0u)) {
            __HAL_TIM_SET_COUNTER(mi_htim_spwm, 0u);
            HAL_TIM_Base_Start_IT(mi_htim_spwm);
        }
    } else {
        if (f_atual <= 0.1f) {
            if (mi_htim_spwm != NULL) {
                HAL_TIM_Base_Stop_IT(mi_htim_spwm);
            }
            if (mi_htim_pwm != NULL) {
                __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_1, 0u);
                __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_2, 0u);
                __HAL_TIM_SET_COMPARE(mi_htim_pwm, TIM_CHANNEL_3, 0u);
            }
        }
    }
}
