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
volatile uint8_t  P42 = 100; // 10kHz (Cuidado no C031, ideal é 50/5kHz)

volatile bool  cmd_ligar_motor = false;
volatile float cmd_frequencia_alvo = 0.0f;

// Outputs
volatile float f_atual = 0.0f;
volatile uint32_t debug_isr_cnt = 0;

// Variáveis de Controle da Rampa (Calculadas no motor_task, usadas no ISR)
volatile float ramp_inc_up = 0.0f;   // Quanto somar para acelerar
volatile float ramp_inc_down = 0.0f; // Quanto subtrair para desacelerar
volatile float inv_fs_2pi = 0.0f;    // Otimização: (2*PI)/fs

// Ângulos
static float theta_u = 0.0f;
static float theta_v = 2.094395f; // 2PI/3
static float theta_w = 4.188790f; // 4PI/3

// Constantes
static const float TWO_PI = 6.283185307f;
#define AMP_MAX 50.0f // Seu ARR do Timer (ajuste se necessário)
static const float FATOR_VF = 0.83f; // V/F

// ============================================================================
// 2. INICIALIZAÇÃO
// ============================================================================
void motor_init(void) {
    if (P21 == 0) P21 = 60;

    // Inicializa variáveis para evitar NaN ou Infinito
    f_atual = 0.0f;
    cmd_frequencia_alvo = 0.0f;

    // Força uma primeira execução da lógica de parâmetros
    motor_task();
}

// ============================================================================
// 3. MOTOR TASK (Loop Lento - Apenas Parametrização)
// ============================================================================
/* Esta função deve ficar no while(1).
 * Ela NÃO executa a rampa, apenas calcula as taxas de aceleração
 * baseadas em P10/P11 para o ISR usar. Isso é muito leve. */
void motor_task(void) {
    // 1. Calcula Frequência de Amostragem Real
    float fs = (float)P42 * 100.0f;
    if (fs < 1000.0f) fs = 1000.0f;

    // 2. Prepara constante para cálculo de angulo (evita divisão no ISR)
    // No ISR faremos: angulo += f_atual * inv_fs_2pi
    inv_fs_2pi = TWO_PI / fs;

    // 3. Calcula incremento da Rampa (Hertz por Interrupção)
    // Se P10 = 5s, fs = 5000Hz. Fmax = 60Hz.
    // Step = 60 / (5 * 5000) = 0.0024 Hz/int
    //float f_max = (float)P21;

    float delta = cmd_frequencia_alvo - f_atual;

    float t_acc = (float)P10;
    if (t_acc < 0.1f) t_acc = 0.1f;

    float t_dec = (float)P11;
    if (t_dec < 0.1f) t_dec = 0.1f;

    // Atualiza variáveis globais atômicas (float é atômico em 32-bit geralmente)
    ramp_inc_up   = delta / (t_acc * fs);
    ramp_inc_down = delta / (t_dec * fs);
}

// ============================================================================
// 4. SPWM (Interrupção - Lógica Completa)
// ============================================================================
/* Esta função deve ser chamada no Callback do TIM3.
 * Ela contém a Lógica da Rampa + Lógica do Seno (Fusion do código validado) */
void spwm(void) {
    debug_isr_cnt++;

    //motor_task();

    // --- A. Lógica da Rampa (Inspirado no código validado) ---
    float alvo = 0.0f;
    float step = 0.0f;

    if (cmd_ligar_motor) {
        // Modo Aceleração / Operação
        alvo = cmd_frequencia_alvo;
        // Clamp do alvo
        if (alvo > (float)P21) alvo = (float)P21;
        if (alvo < (float)P20) alvo = (float)P20;

        step = ramp_inc_up; // Usa taxa de subida

        if (f_atual < alvo) {
            f_atual += step;
            if (f_atual > alvo) f_atual = alvo;
        } else if (f_atual > alvo) {
            f_atual -= step; // Se mudou setpoint pra baixo, desacelera suave
            if (f_atual < alvo) f_atual = alvo;
        }
    }
    else {
        // Modo Parada
        alvo = 0.0f;
        step = ramp_inc_down; // Usa taxa de descida

        if (f_atual > 0.0f) {
            f_atual -= step;
            if (f_atual < 0.0f) f_atual = 0.0f;
        }
    }

    // Atualiza visualização
    P01 = (uint8_t)f_atual;

    // --- B. Verificação de Parada Total ---
    if (!cmd_ligar_motor && f_atual <= 0.1f) {
        // Desliga PWM e sai para economizar CPU
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        return;
    }

    // --- C. Cálculo de Ângulo (Otimizado) ---
    // inc = 2PI * f / fs. Usamos a multiplicação pré-calculada.
    float inc = f_atual * inv_fs_2pi;

    theta_u += inc;
    theta_v += inc;
    theta_w += inc;

    // Wrap-around (Substitui fmodf para performance no C031)
    if (theta_u >= TWO_PI) theta_u -= TWO_PI;
    if (theta_v >= TWO_PI) theta_v -= TWO_PI;
    if (theta_w >= TWO_PI) theta_w -= TWO_PI;

    // --- D. Cálculo Amplitude e PWM ---
    // V/F Escalar
    float amp_atual = f_atual * FATOR_VF;
    if (amp_atual > AMP_MAX) amp_atual = AMP_MAX;

    float offset = AMP_MAX - amp_atual;

    // Seno
    uint16_t p1 = (uint16_t)((amp_atual * (sinf(theta_u) + 1.0f)) + offset);
    uint16_t p2 = (uint16_t)((amp_atual * (sinf(theta_v) + 1.0f)) + offset);
    uint16_t p3 = (uint16_t)((amp_atual * (sinf(theta_w) + 1.0f)) + offset);

    // Hardware
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
}
