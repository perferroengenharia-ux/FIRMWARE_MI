#ifndef MI_APP_H
#define MI_APP_H

#include "mi_state.h"

void mi_app_init(void);
void mi_app_process(void);
void mi_app_on_period_elapsed(TIM_HandleTypeDef *htim);

#endif
