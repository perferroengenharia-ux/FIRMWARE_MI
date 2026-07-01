#ifndef MI_SENSORS_H
#define MI_SENSORS_H

#include <stdbool.h>
#include <stdint.h>
#include "main.h"

void mi_sensors_init(void);
void mi_sensors_request_update(void);
void mi_sensors_process(void);
void mi_sensors_current_offset_calibrate(void);

#endif /* MI_SENSORS_H */
