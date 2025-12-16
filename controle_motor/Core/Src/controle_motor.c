#include "controle_motor.h"
#include "math.h"
#include "tim.h"

// ============================================================================
// 1. VARIÁVEIS GLOBAIS
// ============================================================================

// Inputs
volatile uint8_t P01 = 0;
volatile uint32_t P10 = 15;
volatile uint32_t P11 = 5;
volatile uint8_t P20 = 1;
volatile uint8_t P21 = 90;
volatile uint8_t P42 = 10;

volatile bool cmd_ligar_motor = false;
volatile float cmd_frequencia_alvo = 0.0f;

// Outputs
volatile float f_atual = 0.0f;
volatile uint32_t debug_isr_cnt = 0;

// Variáveis de Memória (Shadow Registers)
static uint8_t P20_run = 1;
static uint8_t P21_run = 90;
static uint32_t P10_run = 15;
static uint32_t P11_run = 5;

// Controle de Estado
static bool last_motor_state = false;
static uint8_t last_P42 = 0;

// Variáveis de Controle da Rampa
volatile float ramp_inc_up = 0.0f;
volatile float ramp_inc_down = 0.0f;
volatile float inv_fs_2pi = 0.0f;

// Variáveis Dinâmicas de Hardware
volatile uint32_t current_arr = 99;
volatile float amp_max_atual = 50.0f;
volatile float freq_ISR = 10000.0f;

// Ângulos
static float theta_u = 0.0f;
static float theta_v = 2.094395f;
static float theta_w = 4.188790f;

static const float TWO_PI = 6.283185307f;

// ============================================================================
// 2. INICIALIZAÇÃO
// ============================================================================
void motor_init(void) {
    if (P21 == 0) P21 = 60;

    f_atual = (float) P20;
    cmd_frequencia_alvo = 5.0f;
    last_P42 = 0;

    P10_run = P10;
    P11_run = P11;
    P20_run = P20;
    P21_run = P21;

    motor_task();
}

// ============================================================================
// 3. MOTOR TASK (Loop Lento)
// ============================================================================
void motor_task(void) {
    // --- LÓGICA DO P42 ---
    if (P42 != last_P42) {
        if (P42 <= 7) P42 = 5;
        else if (P42 <= 12) P42 = 10;
        else P42 = 15;

        atualiza_P42();
        last_P42 = P42;
    }

    // --- LÓGICA DE ESTADO DO MOTOR ---
    if (!last_motor_state && cmd_ligar_motor) {
        // Travamento dos parâmetros na partida
        P20_run = P20;
        P21_run = P21;
        P10_run = P10;
        P11_run = P11;

        if (P20_run < 1)  P20_run = 1;
        if (P20_run > 24) P20_run = 24;

        if (P21_run < 23) P21_run = 23;
        if (P21_run > 90) P21_run = 90;

        if (P10_run < 5)  P10_run = 5;
        if (P10_run > 60) P10_run = 60;

        if (P11_run < 5)  P11_run = 5;
        if (P11_run > 60) P11_run = 60;
    }
    last_motor_state = cmd_ligar_motor;

    if (!cmd_ligar_motor) {
        if (P10 < 5) P10 = 5;
        if (P10 > 60) P10 = 60;

        if (P11 < 5) P11 = 5;
        if (P11 > 60) P11 = 60;

        if (P20 < 1) P20 = 1;
        if (P20 > 24) P20 = 24;

        if (P21 < 23) P21 = 23;
        if (P21 > 90) P21 = 90;

        if (cmd_frequencia_alvo < P20) cmd_frequencia_alvo = P20;
        if (cmd_frequencia_alvo > P21) cmd_frequencia_alvo = P21;
    }

    // --- CÁLCULOS ---
    inv_fs_2pi = TWO_PI / freq_ISR;

    // Atualiza steps da rampa
    calcula_rampa();

    // Controle Timer
    if (cmd_ligar_motor) {
        if (cmd_frequencia_alvo < P20) cmd_frequencia_alvo = P20;
        if (cmd_frequencia_alvo > P21) cmd_frequencia_alvo = P21;

        if ((TIM3->DIER & TIM_DIER_UIE) == 0) {
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            HAL_TIM_Base_Start_IT(&htim3);
        }
    }
    else {
        if (f_atual <= 0.1f) {
            HAL_TIM_Base_Stop_IT(&htim3);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        }
    }
}

// ============================================================================
// FUNÇÃO AUXILIAR: ATUALIZA HARDWARE
// ============================================================================
void atualiza_P42(void) {
    if (P42 == 5) {
        current_arr = 199;
    } else if (P42 == 15) {
        current_arr = 66;
    } else {
        P42 = 10;
        current_arr = 99;
    }
    amp_max_atual = (float)(current_arr + 1) / 2.0f;
    freq_ISR = 1000000.0f / (float)(current_arr + 1);

    __HAL_TIM_SET_AUTORELOAD(&htim1, current_arr);
    __HAL_TIM_SET_AUTORELOAD(&htim3, current_arr);
}

// ============================================================================
// CÁLCULO DA RAMPA (CORRIGIDO)
// ============================================================================
void calcula_rampa(){
    float t_acc = (float)P10_run; if (t_acc < 0.1f) t_acc = 0.1f;
    float t_dec = (float)P11_run; if (t_dec < 0.1f) t_dec = 0.1f;
    float f_max_ref = (float)cmd_frequencia_alvo; // Baseado no alvo atual

    // Multiplicadores
    if (last_P42 == 15){
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 6.7f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 6.7f);
    } else if (last_P42 == 10){
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 3.9f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 3.9f);
    } else { // P42 == 5
        ramp_inc_up   = f_max_ref / ((t_acc * freq_ISR) / 2.2f);
        ramp_inc_down = f_max_ref / ((t_dec * freq_ISR) / 2.2f);
    }

    // --- CORREÇÃO DO TRUNCAMENTO ---
    // Removemos o "if" que forçava f_atual = cmd se f_atual > cmd.
    // Substituímos por uma lógica de histerese para evitar oscilação no estado estacionário.

    if (cmd_ligar_motor) {
        float diferenca = f_atual - cmd_frequencia_alvo;
        // Pega valor absoluto da diferença
        if (diferenca < 0.0f) diferenca = -diferenca;

        // Só força o valor se estiver MUITO PERTO (menos de 0.1Hz de erro)
        if (diferenca < 0.1f) {
            f_atual = cmd_frequencia_alvo;
        }
    }

    // Mantemos a histerese de desligamento
    if (f_atual > (cmd_frequencia_alvo - 0.1f) && cmd_ligar_motor == false){
        f_atual = cmd_frequencia_alvo - 0.15f;
    }
}

// ============================================================================
// 4. SPWM (Interrupção) - (CORRIGIDO PARA SUAVIZAÇÃO)
// ============================================================================
void spwm(void) {
    debug_isr_cnt++;

    // --- A. Lógica da Rampa ---
    float alvo = 0.0f;
    float step = 0.0f;

    if (cmd_ligar_motor) {
        alvo = cmd_frequencia_alvo;

        // Clamp dinâmico
        if (alvo > (float)P21_run) alvo = (float)P21_run;
        if (alvo < (float)P20_run) alvo = (float)P20_run;

        // Decide se acelera ou desacelera para chegar no alvo
        if (f_atual < alvo) {
            // ACELERANDO (Subindo para o alvo) -> Usa P10 (ramp_inc_up)
            step = ramp_inc_up;
            f_atual += step;
            if (f_atual > alvo) f_atual = alvo;
        }
        else if (f_atual > alvo) {
            // DESACELERANDO (Descendo para o novo alvo) -> Agora usa P11 (ramp_inc_down)
            // Isso garante a suavização igual ao desligamento
            step = ramp_inc_down;
            f_atual -= step;
            if (f_atual < alvo) f_atual = alvo;
        }
    }
    else {
        // Desligando -> Usa P11 (ramp_inc_down)
        alvo = 0.0f;
        step = ramp_inc_down;

        if (f_atual > 0.0f) {
            f_atual -= step;
            if (f_atual < 0.0f) f_atual = 0.0f;
        }
    }

    P01 = (uint8_t)f_atual;

    // --- B. Verificação de Parada ---
    if (!cmd_ligar_motor && f_atual <= 0.1f) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        HAL_TIM_Base_Stop_IT(&htim3);
        return;
    }

    // --- C. Ângulos ---
    float inc = 0.0f;

    if (last_P42 == 15){
    	inc = f_atual * inv_fs_2pi * 6.45f;
    } else if (last_P42 == 10){
    	inc = f_atual * inv_fs_2pi * 4.31f;
    } else {
    	inc = f_atual * inv_fs_2pi * 2.16f;
    }

    theta_u += inc;
    if (theta_u >= TWO_PI) theta_u -= TWO_PI;

    theta_v = theta_u + 2.094395f;
    if (theta_v >= TWO_PI) theta_v -= TWO_PI;

    theta_w = theta_u + 4.188790f;
    if (theta_w >= TWO_PI) theta_w -= TWO_PI;

    // --- D. Amplitude Dinâmica ---
    float prop_vf = f_atual * 0.01666f;
    if (prop_vf > 1.0f) prop_vf = 1.0f;

    float amp_atual = prop_vf * amp_max_atual;
    float offset = amp_max_atual - amp_atual;

    uint16_t p1 = (uint16_t)((amp_atual * (sinf(theta_u) + 1.0f)) + offset);
    uint16_t p2 = (uint16_t)((amp_atual * (sinf(theta_v) + 1.0f)) + offset);
    uint16_t p3 = (uint16_t)((amp_atual * (sinf(theta_w) + 1.0f)) + offset);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
}
