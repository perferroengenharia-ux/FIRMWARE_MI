#ifndef CONTROLE_MOTOR_H_
#define CONTROLE_MOTOR_H_

#include "main.h"
#include "stdbool.h"

// --- CONTROLO (Live Watch) ---
extern volatile bool cmd_ligar_motor;
extern volatile float cmd_frequencia_alvo;

// --- LEITURA ---
extern volatile float f_atual;
extern volatile uint8_t P01;

// --- PARÂMETROS ---
extern volatile uint32_t P10;
extern volatile uint32_t P11;
extern volatile uint8_t P20;
extern volatile uint8_t P21;
extern volatile uint8_t P42;

// --- DEBUG ---
extern volatile uint32_t debug_isr_cnt;
extern volatile uint32_t debug_task_cnt;
extern volatile uint32_t debug_isr_time; // Para medir carga da CPU (opcional)

void motor_init(void);
void motor_task(void);
void spwm(void);

#endif /* CONTROLE_MOTOR_H_ */
