#ifndef CONTROLE_MOTOR_H_
#define CONTROLE_MOTOR_H_

#include "main.h"
#include "stdbool.h"

// --- COMANDOS (Inputs) ---
extern volatile bool cmd_ligar_motor;
extern volatile float cmd_frequencia_alvo;

// --- MONITORAMENTO (Outputs) ---
extern volatile float f_atual;
extern volatile uint8_t P01;
extern volatile uint32_t debug_isr_cnt; // Contador de interrupções

// --- PARÂMETROS ---
extern volatile uint32_t P10; // Tempo Aceleração (s)
extern volatile uint32_t P11; // Tempo Desaceleração (s)
extern volatile uint8_t  P20; // Freq Min
extern volatile uint8_t  P21; // Freq Max
extern volatile uint8_t  P42; // Freq Amostragem (x100)

void motor_init(void);
void motor_task(void); // Agora serve apenas para pré-cálculos leves
void spwm(void);       // Roda toda a mágica
void calcula_rampa(void);
void atualiza_P42(void);

#endif /* CONTROLE_MOTOR_H_ */
