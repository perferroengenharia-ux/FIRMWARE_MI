/*
 * controle_motor.c
 *
 *  Created on: Nov 19, 2025
 *      Author: muril
 */

#include "controle_motor.h"
#include "gpio.h"
#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "tim.h"
#include "main.h"

// -----------------------------DEFINIÇÃO DOS PARÂMETROS------------------------

volatile uint8_t P01 = 0; // Freq atual (visualização)
volatile uint32_t P10 = 5; // Tempo em segundos de rampa de subida
volatile uint32_t P11 = 5; // Tempo em segundos de rampa de descida
volatile bool P12 = false; // Armazenar ultima freq. em flash
volatile uint8_t P20 = 1; // Freq mínima (1 a 24 Hz)
volatile uint8_t P21 = 23; // Freq máxima (23 a 90 Hz)
volatile uint8_t P35 = 0; // Compensação de torque
volatile uint8_t P41 = 0; // Freq nominal do motor (50 ou 60 Hz)
volatile uint8_t P42 = 0; // Freq de amostragem (5, 10 ou 15 kHz)

//----------------------------DEFINIÇÃO DAS VARIÁVEIS---------------------------

volatile float f_target = 0.0f; // Freq desejada na rampa
volatile float f_atual = 0.0f; // Freq atual
volatile float f_min = 0.0f; // Freq mínima
volatile float f_max = 0.0f; // Freq máxima
volatile float f_ramp_step = 0.0f; // Freq de passo na rampa
volatile float fs = 0.0f;  // Freq de amostragem

volatile bool motor_sentido = false; // Sentido giro motor (periféricos)
volatile bool motor_desligado = false; // motor desligado ou não(periféricos)

volatile float angulo1 = 0.0f; // angulo fase U
volatile float angulo2 = (120.0f * M_PI)/180; // angulo fase V
volatile float angulo3 = (240.0f * M_PI)/180; // angulo fase W
volatile float ma_ind = 0.0f;

static const float fator_escala  = 0.83f;
static const float limite_angulo = 2.0f * M_PI;

#define amp_max 50

// --- CONTROLE DE SINCRONISMO (RAMPA) ---
// Define quantas vezes a rampa é calculada por segundo.
// 100Hz = 10ms de atualização (padrão industrial para rampas)
#define RAMP_FREQ_HZ 100.0f

// Flag que o TIM3 levanta para o Main calcular a rampa
volatile bool flag_calc_rampa = false;

// Contador para dividir a frequência do TIM3
static uint16_t tick_count = 0;
static uint16_t ticks_threshold = 50; // Calculado em motor_set()

//-----------------------------FUNÇÕES---------------------------------

void motor_set() {
    f_min = (float)P20;
    f_max = (float)P21;

    // Converte P42 para Hz (Assumindo que P42=50 significa 5kHz)
    // Ajuste este multiplicador conforme sua lógica real de P42
    fs = (float)P42 * 100.0f;
    if (fs < 100.0f) fs = 1000.0f; // Proteção contra fs muito baixo

    // Calcula quantos "estouros" do TIM3 são necessários para dar 10ms
    // Ex: 5000Hz / 100Hz = 50 ticks.
    ticks_threshold = (uint16_t)(fs / RAMP_FREQ_HZ);
    if (ticks_threshold < 1) ticks_threshold = 1;

    // Proteção divisão por zero nos tempos
    if (P10 == 0) P10 = 1;
    if (P11 == 0) P11 = 1;
}

/* * Função chamada periodicamente pelo loop principal (while 1)
 * Responsável pela física lenta (Aceleração/Desaceleração)
 */
void motor_task() {
    // Verifica se o TIM3 autorizou o cálculo (Passou 10ms?)
    if (flag_calc_rampa) {
        flag_calc_rampa = false; // Baixa a flag imediatamente

        // 1. Define o Alvo (Setpoint)
        if (motor_desligado) {
            f_target = 0.0f;
        } else {
            // Aqui você pode ler um ADC ou usar P21 como alvo fixo
            f_target = f_max;
            if (f_target < f_min) f_target = f_min;
        }

        // 2. Calcula o passo da rampa (Delta Freq)
        // Passo = (F_Total / Tempo_Segundos) * Periodo_Task
        // Periodo_Task = 1 / RAMP_FREQ_HZ (ex: 0.01s)
        float dt = 1.0f / RAMP_FREQ_HZ;
        float step_up = 0.0f;
        float step_down = 0.0f;

        // Usa f_max como referência de escala para a rampa
        if (P10 > 0) step_up = (f_max / (float)P10) * dt;
        else step_up = f_max;

        if (P11 > 0) step_down = (f_max / (float)P11) * dt;
        else step_down = f_max;

        // 3. Aplica a Rampa
        if (f_atual < f_target) {
            f_atual += step_up;
            if (f_atual > f_target) f_atual = f_target;
        }
        else if (f_atual > f_target) {
            f_atual -= step_down;
            if (f_atual < f_target) f_atual = f_target;
        }

        // 4. Limites Globais
        if (f_atual < 0.0f) f_atual = 0.0f;
        if (f_atual > f_max) f_atual = f_max;

        // 5. Atualiza parâmetro de visualização
        P01 = (uint8_t)f_atual;
    }
}

/* * Função chamada DENTRO da interrupção do TIM3
 * Responsável pela física rápida (Cálculo de seno e PWM)
 * NÃO deve conter loops ou delays.
 */
void spwm() {

    // --- 1. Divisor de frequência para a Rampa (Software Prescaler) ---
    tick_count++;
    if (tick_count >= ticks_threshold) {
        tick_count = 0;
        flag_calc_rampa = true; // Avisa o motor_task que passaram 10ms
    }

    // --- 2. Se motor desligado e parado, zera PWM e sai ---
    if (motor_desligado && f_atual <= 0.1f) {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        return;
    }

    // --- 3. Lógica SPWM Original ---

    // Incremento do ângulo: dTheta = 2*PI * f * T_amostragem
    // T_amostragem = 1/fs. Logo: inc = (2*PI * f) / fs
    float inc = (2.0f * M_PI * f_atual) / fs;

    // Inverte sequência de fases se necessário
    if (motor_sentido) {
        angulo1 -= inc;
        angulo2 -= inc;
        angulo3 -= inc;
    } else {
        angulo1 += inc;
        angulo2 += inc;
        angulo3 += inc;
    }

    // Mantém os ângulos dentro de 0 a 2PI (fmodf é pesado, mas necessário aqui)
    // Otimização futura: Usar if(angulo > 2PI) angulo -= 2PI; é mais rápido que fmodf
    angulo1 = fmodf(angulo1, limite_angulo);
    angulo2 = fmodf(angulo2, limite_angulo);
    angulo3 = fmodf(angulo3, limite_angulo);

    // Correção para ângulos negativos (caso fmodf retorne negativo ou subtração)
    if(angulo1 < 0) angulo1 += limite_angulo;
    if(angulo2 < 0) angulo2 += limite_angulo;
    if(angulo3 < 0) angulo3 += limite_angulo;

    // Amplitude baseada na V/F (escalar)
    uint16_t amplitude = (uint16_t)(f_atual * fator_escala);
    if (amplitude > amp_max) amplitude = amp_max;

    // Índice de modulação para debug
    ma_ind = (float)amplitude / (float)amp_max;

    // Offset para centralizar a senoide (SPWM unipolar ou SVM simplificado)
    uint16_t offset = amp_max - amplitude;

    // Cálculo dos Duty Cycles (CCRs)
    // (sinf + 1.0) coloca o seno entre 0 e 2. Multiplicado pela amplitude.
    uint16_t p1 = (uint16_t)((amplitude * (sinf(angulo1) + 1.0f)) + offset);
    uint16_t p2 = (uint16_t)((amplitude * (sinf(angulo2) + 1.0f)) + offset);
    uint16_t p3 = (uint16_t)((amplitude * (sinf(angulo3) + 1.0f)) + offset);

    // Grava nos registradores do Timer 1
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
}
