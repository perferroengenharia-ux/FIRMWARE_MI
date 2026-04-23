#ifndef MI_PERIPHERALS_H
#define MI_PERIPHERALS_H

#include "mi_state.h"

void mi_periph_init_runtime(void);
void mi_periph_set_modes(void);
void mi_periph_tick_1s(void);
void mi_periph_apply_remote(void);
void mi_comm_apply_received_command(void);
void mi_comm_safe_stop(void);
uint8_t mi_sensor_read(void);

#endif
