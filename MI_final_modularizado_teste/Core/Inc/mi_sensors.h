#ifndef MI_SENSORS_H
#define MI_SENSORS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t adc_temp_raw;
    float    temp_voltage;
    float    temp_c;
    float    temp_c_filt;
    bool     temp_fault;
} mi_sensors_data_t;

extern mi_sensors_data_t g_sensors;

void mi_sensors_init(void);
void mi_sensors_task(void);
float mi_sensors_temp_from_adc(uint16_t adc_raw);

#endif
