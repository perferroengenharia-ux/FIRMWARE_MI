#include "mi_platform.h"

UART_HandleTypeDef *mi_huart = NULL;
TIM_HandleTypeDef  *mi_htim_pwm = NULL;
TIM_HandleTypeDef  *mi_htim_spwm = NULL;
TIM_HandleTypeDef  *mi_htim_sched = NULL;
ADC_HandleTypeDef  *mi_hadc = NULL;

void mi_platform_bind(UART_HandleTypeDef *huart,
                      TIM_HandleTypeDef *htim_pwm,
                      TIM_HandleTypeDef *htim_spwm,
                      TIM_HandleTypeDef *htim_sched,
                      ADC_HandleTypeDef *hadc)
{
    mi_huart = huart;
    mi_htim_pwm = htim_pwm;
    mi_htim_spwm = htim_spwm;
    mi_htim_sched = htim_sched;
    mi_hadc = hadc;
}
