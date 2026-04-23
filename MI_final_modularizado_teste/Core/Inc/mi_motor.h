#ifndef MI_MOTOR_H
#define MI_MOTOR_H

#include "mi_state.h"

void mi_motor_init_runtime(void);
void mi_motor_task_runtime(void);
void mi_motor_on_tim2_irq(void);
void mi_motor_factory_reset_defaults(void);
void mi_telemetry_update(void);
uint8_t mi_telemetry_status_flags(void);

#endif
