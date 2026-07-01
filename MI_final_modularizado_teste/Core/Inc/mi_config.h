#ifndef MI_CONFIG_H
#define MI_CONFIG_H

#include "main.h"

/* ============================================================
 * Configuração principal para STM32F301C8T6
 * ============================================================ */

#define MI_ADDR_STM32               0x01u
#define MI_ADDR_MASTER              0xF0u

#define MI_SOF                      0x7Eu
#define MI_ESC                      0x7Du
#define MI_ESC_XOR                  0x20u

#define MI_TYPE_READ_STATUS         0x04u
#define MI_TYPE_WRITE_PARAM         0x05u
#define MI_TYPE_ACK_MI              0x06u
#define MI_TYPE_ACK                 0x80u

#define MI_MAX_PAYLOAD              128u
#define MI_RX_DMA_BUF_SZ            256u
#define MI_COMMS_TIMEOUT_MS         2000u

#define MI_STATUS_WATER_SHORTAGE    (1u << 0)

#define MI_BTN_BIT_START            (1u << 0)
#define MI_BTN_BIT_STOP             (1u << 1)
#define MI_BTN_BIT_UP               (1u << 2)
#define MI_BTN_BIT_DOWN             (1u << 3)

#define MI_AUX_BIT_BOMBA            (1u << 0)
#define MI_AUX_BIT_SWING            (1u << 1)
#define MI_AUX_BIT_EXAUSTAO         (1u << 2)
#define MI_AUX_BIT_DRENO            (1u << 3)
#define MI_AUX_BIT_SYSTEM_ON        (1u << 4)

/* Para STM32F301C8T6, reserve a última página da flash no linker.
 * Para 64 KiB e página de 2 KiB, último início típico: 0x0800F800.
 */
#ifndef MI_FLASH_USER_ADDR
#define MI_FLASH_USER_ADDR          0x0800F800u
#endif

/* Timers usados no projeto F301.
 * TIM1: PWM complementar do inversor.
 * TIM2: interrupção de atualização do SPWM.
 * TIM16: agendamento lento de sensores/periféricos.
 */
#define MI_SPWM_TIM_INSTANCE        TIM2
#define MI_SCHED_TIM_INSTANCE       TIM16

/* O algoritmo assume base de tempo de 1 MHz para TIM1/TIM2.
 * Se o clock dos timers no CubeMX não for 48 MHz com prescaler 48-1,
 * ajuste CubeMX ou adapte esta constante e a inicialização dos timers.
 */
#define MI_TIMER_TICK_HZ            1000000.0f

/* ADC no STM32F301.
 * Ajuste conforme o pino real escolhido no CubeMX.
 * Memória do projeto: corrente em ADC1_IN4 e barramento CC em ADC1_IN3.
 */
#define MI_ADC_CH_CURRENT           ADC_CHANNEL_4
#define MI_ADC_CH_TEMP              ADC_CHANNEL_6
#define MI_ADC_CH_VBUS              ADC_CHANNEL_3

#if defined(ADC_SAMPLETIME_181CYCLES_5)
#define MI_ADC_SAMPLING_TIME        ADC_SAMPLETIME_181CYCLES_5
#elif defined(ADC_SAMPLETIME_160CYCLES_5)
#define MI_ADC_SAMPLING_TIME        ADC_SAMPLETIME_160CYCLES_5
#else
#define MI_ADC_SAMPLING_TIME        ADC_SAMPLETIME_239CYCLES_5
#endif

#define MI_ADC_VREF_VOLTS           3.3f
#define MI_ADC_MAX_COUNTS           4095.0f

/* Sensor TSO do STGIB15CH60S-L */
#define MI_TSO_V_AT_0C              0.7f
#define MI_TSO_VOLTS_PER_C          0.0184f

/* Divisor do barramento CC */
#define MI_VBUS_R1                  100000.0f
#define MI_VBUS_R2                  680.0f
#define MI_VBUS_DIV_GAIN            ((MI_VBUS_R1 + MI_VBUS_R2) / MI_VBUS_R2)

/* Corrente pelo circuito atualizado: I = (ADC - OFFSET) / 54,4 */
#define MI_CURRENT_COUNTS_PER_A     54.4f
#define MI_CURRENT_OFFSET_DEFAULT   1917.0f
#define MI_CURRENT_EMA_ALPHA        0.25f
#define MI_CURRENT_BENCH_TEST_MODE  0

/* Compatibilidade para o LED de status.
 * Preferencialmente, renomeie o User Label no CubeMX para Led.
 * Se o projeto antigo usa ED, este fallback permite compilar.
 */
#ifndef Led_Pin
  #ifdef ED_Pin
    #define Led_Pin ED_Pin
    #define Led_GPIO_Port ED_GPIO_Port
  #endif
#endif

#ifndef MI_STATUS_LED_Pin
  #define MI_STATUS_LED_Pin       Led_Pin
  #define MI_STATUS_LED_GPIO_Port Led_GPIO_Port
#endif

#endif /* MI_CONFIG_H */
