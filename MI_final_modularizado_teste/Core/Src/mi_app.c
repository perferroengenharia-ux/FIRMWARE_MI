#include "mi_app.h"
#include "mi_config.h"
#include "mi_state.h"
#include "mi_comm.h"
#include "mi_motor.h"
#include "mi_periph.h"
#include "mi_sensors.h"
#include "mi_platform.h"

static void mi_start_pwm_outputs(void)
{
    if (mi_htim_pwm == NULL) return;

    HAL_TIM_PWM_Start(mi_htim_pwm, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(mi_htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(mi_htim_pwm, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(mi_htim_pwm, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(mi_htim_pwm, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(mi_htim_pwm, TIM_CHANNEL_3);
}

void mi_app_init(void)
{
    mi_sensors_init();
    mi_comm_init();
    mi_periph_set_modes();
    mi_motor_init_runtime();

    mi_start_pwm_outputs();

    if (mi_htim_sched != NULL) {
        HAL_TIM_Base_Start_IT(mi_htim_sched);
    }

    mi_comm_start_rx();
    last_packet_tick = HAL_GetTick();
}

void mi_app_process(void)
{
    if ((HAL_GetTick() - last_packet_tick) > 500u) {
        if (mi_huart != NULL && __HAL_UART_GET_FLAG(mi_huart, UART_FLAG_ORE)) {
            __HAL_UART_CLEAR_OREFLAG(mi_huart);
            mi_comm_start_rx();
        }
    }

    mi_comm_process_rx();

    if ((HAL_GetTick() - last_packet_tick) > MI_COMMS_TIMEOUT_MS) {
        hardware_comms_ok = false;
        mi_periph_safe_stop();
    }

    mi_periph_apply_remote();
    mi_motor_task_runtime();
    mi_sensors_process();

    HAL_GPIO_WritePin(MI_STATUS_LED_GPIO_Port, MI_STATUS_LED_Pin, hardware_comms_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void mi_app_on_period_elapsed(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) return;

    if (htim->Instance == MI_SPWM_TIM_INSTANCE) {
        mi_motor_spwm_isr();
    } else if (htim->Instance == MI_SCHED_TIM_INSTANCE) {
        dbg_tim16_count++;
        mi_sensors_request_update();

        tim16_100ms_count++;
        if (tim16_100ms_count >= 10u) {
            tim16_100ms_count = 0u;
            mi_periph_tick_1s();
        }
    }
}
