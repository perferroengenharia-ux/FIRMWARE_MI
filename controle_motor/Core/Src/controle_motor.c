/*
 * controle_motor.c
 *
 *  Created on: Nov 19, 2025
 *      Author: muril
 */

#include "math.h"
#include "stdbool.h"
#include "stdint.h"
#include "tim.h"

// -----------------------------DEFINIÇÃO DOS PARÂMETROS------------------------

volatile uint8_t P01 = 0; // Freq atual (visualização)
volatile uint32_t P10 = 0; // Tempo de rampa de subida
volatile uint32_t P11 = 0; // Tempo de rampa de descida
volatile bool P12 = false; // Armazenar ultima freq. em flash
volatile uint8_t P20 = 0; // Freq mínima
volatile uint8_t P21 = 0; // Freq máxima
volatile uint8_t P35 = 0; // Compensação de torque
volatile uint8_t P41 = 0; // Freq nominal do motor
volatile uint8_t P42 = 0; // Freq de amostragem

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

//-----------------------------FUNÇÕES---------------------------------

void motor_set (){
	f_min = P20;
	f_max = P21;
	fs = P42;
}

void spwm (){
    // Stop TIM2 interrupt once ramp-down is complete
   HAL_TIM_Base_Stop_IT(&htim3);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//U
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);	//V
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);	//W

/* 2. ângulos -------------------------------------------------------*/
   	float inc = (2.0f * M_PI * f_atual) / fs;
	angulo1 = fmodf(angulo1 + inc, limite_angulo);		//U
	angulo2 = fmodf(angulo2 + inc, limite_angulo);		//V
	angulo3 = fmodf(angulo3 + inc, limite_angulo);	    //W

	/* 3. amplitude e offset -------------------------------------------*/
	uint16_t amplitude = (uint16_t)(f_atual * fator_escala);
	ma_ind = (float)amplitude / (float)amp_max;

	if (amplitude > amp_max) amplitude = amp_max;

	uint16_t offset = amp_max - amplitude;

	/* 4. calcula CCRs --------------------------------------------------*/
	volatile uint16_t p1, p2, p3;
	p1 = (amplitude * (sinf(angulo1) + 1.0f)) + offset;		//U
	p2 = (amplitude * (sinf(angulo2) + 1.0f)) + offset;	    //V
	p3 = (amplitude * (sinf(angulo3) + 1.0f)) + offset;		//W

	/* 5. grava nos registradores --------------------------------------*/
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);		//U
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);		//V
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);		//W
}

void motor_task(){

}
