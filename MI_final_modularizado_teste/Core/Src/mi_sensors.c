#include "mi_sensors.h"

mi_sensors_data_t g_sensors = {0};

static float clampf_local(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float mi_sensors_temp_from_adc(uint16_t adc_raw)
{
    float v_tso = ((float)adc_raw * 3.3f) / 4095.0f;
    float temp_c = (v_tso - 0.7f) / 0.0184f;
    temp_c = clampf_local(temp_c, 0.0f, 125.0f);
    return temp_c;
}

void mi_sensors_init(void)
{
    g_sensors.adc_temp_raw = 0;
    g_sensors.temp_voltage = 0.0f;
    g_sensors.temp_c = 0.0f;
    g_sensors.temp_c_filt = 0.0f;
    g_sensors.temp_fault = false;
}

void mi_sensors_task(void)
{
    float temp_c;
    float v_tso;

    v_tso = ((float)g_sensors.adc_temp_raw * 3.3f) / 4095.0f;
    temp_c = mi_sensors_temp_from_adc(g_sensors.adc_temp_raw);

    g_sensors.temp_voltage = v_tso;
    g_sensors.temp_c = temp_c;
    g_sensors.temp_c_filt = g_sensors.temp_c_filt + 0.1f * (temp_c - g_sensors.temp_c_filt);

    if (g_sensors.temp_c_filt >= 100.0f) {
        g_sensors.temp_fault = true;
    } else if (g_sensors.temp_c_filt <= 95.0f) {
        g_sensors.temp_fault = false;
    }
}
