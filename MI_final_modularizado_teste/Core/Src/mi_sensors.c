#include "mi_sensors.h"
#include "mi_platform.h"
#include "mi_state.h"
#include "mi_config.h"

static bool mi_adc_read_channel(uint32_t channel, uint16_t *out_raw)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    if (mi_hadc == NULL || out_raw == NULL) {
        return false;
    }

    HAL_ADC_Stop(mi_hadc);

#ifdef ADC_FLAG_OVR
    __HAL_ADC_CLEAR_FLAG(mi_hadc, ADC_FLAG_OVR);
#endif
#ifdef ADC_FLAG_EOC
    __HAL_ADC_CLEAR_FLAG(mi_hadc, ADC_FLAG_EOC);
#endif
#ifdef ADC_FLAG_EOS
    __HAL_ADC_CLEAR_FLAG(mi_hadc, ADC_FLAG_EOS);
#endif

    sConfig.Channel = channel;
#ifdef ADC_REGULAR_RANK_1
    sConfig.Rank = ADC_REGULAR_RANK_1;
#else
    sConfig.Rank = 1;
#endif
#ifdef ADC_SAMPLETIME_1CYCLE_5
    sConfig.SamplingTime = MI_ADC_SAMPLING_TIME;
#endif
#ifdef ADC_SINGLE_ENDED
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
#endif
#ifdef ADC_OFFSET_NONE
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
#endif
#if defined(ADC_OFFSET_NONE) || defined(ADC_SINGLE_ENDED)
    sConfig.Offset = 0;
#endif

    if (HAL_ADC_ConfigChannel(mi_hadc, &sConfig) != HAL_OK) {
        return false;
    }

    if (HAL_ADC_Start(mi_hadc) != HAL_OK) {
        dbg_adc_fail_stage = 1u;
        return false;
    }

    if (HAL_ADC_PollForConversion(mi_hadc, 5) != HAL_OK) {
        dbg_adc_fail_stage = 2u;
        HAL_ADC_Stop(mi_hadc);
        return false;
    }

    *out_raw = (uint16_t)HAL_ADC_GetValue(mi_hadc);
    HAL_ADC_Stop(mi_hadc);
    return true;
}

static bool mi_sensors_read_all(uint16_t *raw_current, uint16_t *raw_temp, uint16_t *raw_vbus)
{
    if (raw_current == NULL || raw_temp == NULL || raw_vbus == NULL) {
        return false;
    }

    dbg_adc_fail_stage = 0u;
    dbg_adc_state_before = (mi_hadc != NULL) ? HAL_ADC_GetState(mi_hadc) : 0u;

    if (!mi_adc_read_channel(MI_ADC_CH_CURRENT, raw_current)) {
        if (dbg_adc_fail_stage == 0u) dbg_adc_fail_stage = 10u;
        goto fail;
    }

    if (!mi_adc_read_channel(MI_ADC_CH_TEMP, raw_temp)) {
        if (dbg_adc_fail_stage == 0u) dbg_adc_fail_stage = 11u;
        goto fail;
    }

    if (!mi_adc_read_channel(MI_ADC_CH_VBUS, raw_vbus)) {
        if (dbg_adc_fail_stage == 0u) dbg_adc_fail_stage = 12u;
        goto fail;
    }

    dbg_adc_state_after = (mi_hadc != NULL) ? HAL_ADC_GetState(mi_hadc) : 0u;
    return true;

fail:
    if (mi_hadc != NULL) {
        HAL_ADC_Stop(mi_hadc);
        dbg_adc_state_after = HAL_ADC_GetState(mi_hadc);
    }
    return false;
}

static void mi_sensors_process_current(uint16_t raw_current)
{
    float delta_counts;

    g_analog.current_raw = raw_current;
    g_analog.current_adc_v = ((float)raw_current * MI_ADC_VREF_VOLTS) / MI_ADC_MAX_COUNTS;

    delta_counts = (float)raw_current - g_analog.current_offset_counts;
    if (delta_counts < 0.0f) {
        delta_counts = 0.0f;
    }

    g_analog.current_a = delta_counts / MI_CURRENT_COUNTS_PER_A;

    if (!current_filter_initialized) {
        g_analog.current_a_filt = g_analog.current_a;
        current_filter_initialized = true;
    } else {
        g_analog.current_a_filt =
            g_analog.current_a_filt +
            MI_CURRENT_EMA_ALPHA * (g_analog.current_a - g_analog.current_a_filt);
    }
}

void mi_sensors_current_offset_calibrate(void)
{
    uint32_t acc = 0u;
    uint32_t valid = 0u;
    const uint32_t samples = 64u;

    g_analog.current_offset_counts = MI_CURRENT_OFFSET_DEFAULT;
    g_analog.current_offset_v =
        (MI_CURRENT_OFFSET_DEFAULT * MI_ADC_VREF_VOLTS) / MI_ADC_MAX_COUNTS;

    for (uint32_t i = 0u; i < samples; i++) {
        uint16_t raw_current = 0u;
        uint16_t raw_temp = 0u;
        uint16_t raw_vbus = 0u;

        if (mi_sensors_read_all(&raw_current, &raw_temp, &raw_vbus)) {
            acc += raw_current;
            valid++;
        }
        HAL_Delay(2);
    }

    if (valid > 0u) {
        g_analog.current_offset_counts = (float)acc / (float)valid;
        g_analog.current_offset_v =
            (g_analog.current_offset_counts * MI_ADC_VREF_VOLTS) / MI_ADC_MAX_COUNTS;
    }

    current_filter_initialized = false;
    g_analog.current_a = 0.0f;
    g_analog.current_a_filt = 0.0f;
}

void mi_sensors_init(void)
{
    if (mi_hadc == NULL) {
        return;
    }

    HAL_ADC_Stop(mi_hadc);

#if defined(ADC_SINGLE_ENDED)
    if (HAL_ADCEx_Calibration_Start(mi_hadc, ADC_SINGLE_ENDED) != HAL_OK) {
        Error_Handler();
    }
#else
    if (HAL_ADCEx_Calibration_Start(mi_hadc) != HAL_OK) {
        Error_Handler();
    }
#endif

    HAL_Delay(2);
    mi_sensors_current_offset_calibrate();
}

void mi_sensors_request_update(void)
{
    analog_update_request = true;
}

void mi_sensors_process(void)
{
    uint16_t raw_current = 0u;
    uint16_t raw_temp = 0u;
    uint16_t raw_vbus = 0u;

    if (!analog_update_request) {
        return;
    }
    analog_update_request = false;

    dbg_analog_call_count++;
    dbg_analog_last_call_ms = HAL_GetTick();

    if (!mi_sensors_read_all(&raw_current, &raw_temp, &raw_vbus)) {
        dbg_analog_fail_count++;
        return;
    }

    dbg_analog_ok_count++;
    dbg_analog_last_ok_ms = HAL_GetTick();

    dbg_adc_raw_current = raw_current;
    dbg_adc_raw_temp = raw_temp;
    dbg_adc_raw_vbus = raw_vbus;

    g_analog.temp_raw = raw_temp;
    g_analog.vbus_raw = raw_vbus;

    v_tso = ((float)g_analog.temp_raw * MI_ADC_VREF_VOLTS) / MI_ADC_MAX_COUNTS;
    temp_c = (v_tso - MI_TSO_V_AT_0C) / MI_TSO_VOLTS_PER_C;
    temp_c = mi_clampf(temp_c, 0.0f, 125.0f);

    g_analog.temp_adc_v = v_tso;
    g_analog.temp_c = temp_c;
    g_analog.temp_c_filt = temp_c;

    v_adc = ((float)g_analog.vbus_raw * MI_ADC_VREF_VOLTS) / MI_ADC_MAX_COUNTS;
    v_bus = v_adc * MI_VBUS_DIV_GAIN;
    if (v_bus < 0.0f) {
        v_bus = 0.0f;
    }

    g_analog.vbus_adc_v = v_adc;
    g_analog.vbus_v = v_bus;
    g_analog.vbus_v_filt = v_bus;

    mi_sensors_process_current(raw_current);
}
