#ifndef MI_COMM_H
#define MI_COMM_H

#include "mi_state.h"

void mi_comm_init(void);
void mi_comm_process(void);
uint16_t mi_crc16_ibm(const uint8_t *data, uint16_t len);

#endif
