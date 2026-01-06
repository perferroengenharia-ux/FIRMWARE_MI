/*
 * perifericos.h
 *
 *  Created on: Nov 13, 2025
 *      Author: murilo
 */

#ifndef INC_PERIFERICOS_H_
#define INC_PERIFERICOS_H_

#include "main.h"
#include <stdint.h>
#include "stdbool.h"

typedef enum {
    IDLE = 0,
    BOMBA_ON,
    VENTILADOR_ON,
    OFF,
    BOMBA_OFF,
    VENTILADOR_OFF,
    DRENO_ON,
	DRENO_OFF_DELAY,
    EXAUSTOR_ON,
    EXAUSTOR_WAIT,
	EXAUSTAO_INVERSAO,
	EXAUSTAO_SECAR
}Estado;

extern volatile bool dreno_solicitado;
extern volatile Estado estado_atual;

void perifericos_read_buttons(void);
void perifericos_set (void);
uint8_t sensor_read (void);
void perifericos_tick (void);
void perifericos_task (void);

#endif /* INC_PERIFERICOS_H_ */
