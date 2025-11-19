/*
 * controle_motor.h
 *
 * Created on: Nov 19, 2025
 * Author: muril
 */

#ifndef CONTROLE_MOTOR_H_
#define CONTROLE_MOTOR_H_

#include "main.h"
#include "stdbool.h"

// --- Parâmetros Globais (Acessíveis para debug) ---
extern volatile uint8_t P01; // Visualização Freq
extern volatile uint32_t P10; // Rampa Subida (s)
extern volatile uint32_t P11; // Rampa Descida (s)
extern volatile bool P12;
extern volatile uint8_t P20; // Freq Mínima
extern volatile uint8_t P21; // Freq Máxima
extern volatile uint8_t P41; // Freq Nominal
extern volatile uint8_t P42; // Freq Amostragem (x100Hz)

// --- Variáveis de Controle ---
extern volatile bool motor_sentido;
extern volatile bool motor_desligado;
extern volatile float f_atual;
extern volatile float f_target;

// --- Protótipos das Funções ---
void motor_set(void);   // Inicializa variáveis
void motor_task(void);  // Chamada no while(1) - Lógica da Rampa
void spwm(void);        // Chamada na Interrupção TIM3 - Geração de PWM

#endif /* CONTROLE_MOTOR_H_ */
