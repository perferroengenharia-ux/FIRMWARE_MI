#ifndef MI_PERIPH_H
#define MI_PERIPH_H

#include <stdint.h>

void mi_periph_set_modes(void);
uint8_t mi_periph_sensor_read(void);
void mi_periph_tick_1s(void);
void mi_periph_apply_remote(void);
void mi_periph_safe_stop(void);

#endif /* MI_PERIPH_H */
