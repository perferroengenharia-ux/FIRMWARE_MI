#ifndef MI_PLATFORM_H
#define MI_PLATFORM_H

#include "main.h"

extern UART_HandleTypeDef *mi_huart;
extern TIM_HandleTypeDef  *mi_htim_pwm;
extern TIM_HandleTypeDef  *mi_htim_spwm;
extern TIM_HandleTypeDef  *mi_htim_sched;
extern ADC_HandleTypeDef  *mi_hadc;

void mi_platform_bind(UART_HandleTypeDef *huart,
                      TIM_HandleTypeDef *htim_pwm,
                      TIM_HandleTypeDef *htim_spwm,
                      TIM_HandleTypeDef *htim_sched,
                      ADC_HandleTypeDef *hadc);

#endif /* MI_PLATFORM_H */
