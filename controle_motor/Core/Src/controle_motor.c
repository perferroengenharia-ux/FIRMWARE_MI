#include "controle_motor.h"
#include "math.h"
#include "tim.h"

// ============================================================================
// 1. VARIÁVEIS GLOBAIS
// ============================================================================

// Inputs
volatile uint8_t  P01 = 0;
volatile uint32_t P10 = 5;
volatile uint32_t P11 = 5;
volatile uint8_t  P20 = 5;
volatile uint8_t  P21 = 60;
volatile uint8_t  P42 = 50; // 5kHz

volatile bool  cmd_ligar_motor = false;
volatile float cmd_frequencia_alvo = 0.0f;

// Outputs e Debug
volatile float f_atual = 0.0f;
volatile uint32_t debug_isr_cnt = 0;
volatile uint32_t debug_task_cnt = 0;

// Variáveis INTERNAS OTIMIZADAS
// Transferimos o cálculo pesado para cá:
volatile float angle_step = 0.0f; // O quanto o angulo anda por interrupção
volatile float amplitude_atual = 0.0f; // Amplitude (0 a 50) já calculada

// Ângulos em Float (Radianos)
static float theta_u = 0.0f;
static float theta_v = 2.094395f; // 2PI/3
static float theta_w = 4.188790f; // 4PI/3

// Constantes
static const float TWO_PI = 6.283185307f;
static const float FATOR_VF = 0.83f; // V/F
#define AMP_MAX 50.0f

// Sincronismo
volatile bool flag_executa_rampa = false;
static uint16_t tick_counter = 0;
static uint16_t ticks_para_10ms = 50;

// ============================================================================
// 2. INICIALIZAÇÃO
// ============================================================================
void motor_init(void) {
    if (P21 == 0) P21 = 60;

    float fs = (float)P42 * 100.0f;
    if (fs < 1000.0f) fs = 1000.0f;

    ticks_para_10ms = (uint16_t)(fs / 100.0f);
}

// ============================================================================
// 3. MOTOR TASK (Loop Lento - Calcula a Física e Prepara Variáveis)
// ============================================================================
void motor_task(void) {
    if (!flag_executa_rampa) return;
    flag_executa_rampa = false;
    debug_task_cnt++;

    // --- A. Lógica da Rampa (Idêntica à anterior) ---
    float f_max = (float)P21;
    float t_acc = (float)P10; if (t_acc < 0.1f) t_acc = 0.1f;
    float t_dec = (float)P11; if (t_dec < 0.1f) t_dec = 0.1f;
    float dt = 0.01f;

    float alvo = 0.0f;
    if (cmd_ligar_motor) {
        alvo = cmd_frequencia_alvo;
        if (alvo > f_max) alvo = f_max;
        if (alvo < (float)P20) alvo = (float)P20;
    }

    float step_up = (f_max / t_acc) * dt;
    float step_down = (f_max / t_dec) * dt;

    if (f_atual < alvo) {
        f_atual += step_up;
        if (f_atual > alvo) f_atual = alvo;
    } else if (f_atual > alvo) {
        f_atual -= step_down;
        if (f_atual < alvo) f_atual = alvo;
    }

    if (f_atual < 0.0f) f_atual = 0.0f;
    P01 = (uint8_t)f_atual;

    // --- B. PRE-CÁLCULO PARA O ISR (O SEGREDO DO DESEMPENHO) ---

    // 1. Calcula fs atualizado
    float fs = (float)P42 * 100.0f;
    if (fs < 100.0f) fs = 1000.0f;

    // 2. Calcula o Passo Angular (Delta Theta)
    // Isso contém a divisão pesada. Fazemos aqui para não fazer na interrupção.
    // angle_step = (2 * PI * f) / fs
    angle_step = (TWO_PI * f_atual) / fs;

    // 3. Calcula Amplitude V/F
    // Também fazemos aqui fora.
    float amp_calc = f_atual * FATOR_VF;
    if (amp_calc > AMP_MAX) amp_calc = AMP_MAX;
    amplitude_atual = amp_calc;
}

// ============================================================================
// 4. SPWM (Interrupção Rápida - Float Otimizado)
// ============================================================================
void spwm(void) {
    debug_isr_cnt++;

    // Sincronismo (Contador de Inteiros é rápido)
    tick_counter++;
    if (tick_counter >= ticks_para_10ms) {
        tick_counter = 0;
        flag_executa_rampa = true;
    }

    // Se motor desligado ou parado
    if (amplitude_atual < 0.5f || !cmd_ligar_motor) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        return;
    }

    // --- 1. Atualização dos Ângulos (Soma Simples de Float) ---
    // Usamos a variável pre-calculada 'angle_step'.
    theta_u += angle_step;
    theta_v += angle_step;
    theta_w += angle_step;

    // --- 2. Limitação 0..2PI (Substituição Rápida do fmodf) ---
    // Muito mais rápido que chamar a função de biblioteca
    if (theta_u >= TWO_PI) theta_u -= TWO_PI;
    if (theta_v >= TWO_PI) theta_v -= TWO_PI;
    if (theta_w >= TWO_PI) theta_w -= TWO_PI;

    // --- 3. Cálculo do Seno (Math.h) ---
    // Aqui usamos o processamento para ter precisão
    float s1 = sinf(theta_u);
    float s2 = sinf(theta_v);
    float s3 = sinf(theta_w);

    // --- 4. Cálculo do Duty Cycle ---
    // (seno + 1.0) -> intervalo 0..2
    // * amplitude -> intervalo 0..2*amp
    // + offset -> centraliza
    // A conta simplificada para SPWM Unipolar é:
    // Duty = Amplitude * (seno + 1) / 2  <-- Ajuste conforme seu driver
    // Mas vamos manter sua lógica original de offset que funciona bem:

    // Offset dinâmico: Se amp é 50 (máx), offset é 0. Se amp é 0, offset é 50.
    // Essa lógica mantém o sinal centralizado em 50% do PWM se amplitude for 0.
    float offset = AMP_MAX - amplitude_atual;

    uint16_t ccr1 = (uint16_t)((amplitude_atual * (s1 + 1.0f)) + offset);
    uint16_t ccr2 = (uint16_t)((amplitude_atual * (s2 + 1.0f)) + offset);
    uint16_t ccr3 = (uint16_t)((amplitude_atual * (s3 + 1.0f)) + offset);

    // --- 5. Grava no Hardware ---
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr3);
}
