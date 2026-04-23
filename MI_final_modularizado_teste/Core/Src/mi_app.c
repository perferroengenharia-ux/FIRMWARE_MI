#include "mi_app.h"
#include "mi_comm.h"
#include "mi_motor.h"
#include "mi_peripherals.h"
#include "mi_sensors.h"

void mi_app_init(void) {
    mi_periph_init_runtime();
    mi_motor_init_runtime();

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    HAL_TIM_Base_Start_IT(&htim16);
    mi_comm_init();
    mi_sensors_init();
}

void mi_app_process(void) {
    mi_comm_process();
    mi_sensors_task();

    if ((HAL_GetTick() - last_packet_tick) > MI_COMMS_TIMEOUT_MS) {
        hardware_comms_ok = false;
        mi_comm_safe_stop();
    }

    mi_periph_apply_remote();
    mi_motor_task_runtime();
    mi_telemetry_update();

    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin,
                      hardware_comms_ok ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_Delay(5);
}

void mi_app_on_period_elapsed(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        mi_motor_on_tim2_irq();
    } else if (htim->Instance == TIM16) {
        mi_periph_tick_1s();
    }
}
