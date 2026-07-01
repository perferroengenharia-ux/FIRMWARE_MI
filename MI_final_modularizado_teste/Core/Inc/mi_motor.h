#ifndef MI_MOTOR_H
#define MI_MOTOR_H

#include "main.h"

void mi_motor_init_runtime(void);
void mi_motor_task_runtime(void);
void mi_motor_spwm_isr(void);

#endif /* MI_MOTOR_H */
