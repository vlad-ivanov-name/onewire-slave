#ifndef ONE_SLAVE_H
#define ONE_SLAVE_H

#include <msp430.h>
#include <stdint.h>
#include "memory/offset.h"

#define ONE_PORT		2
#define ONE_B_IN		5
#define ONE_B_OUT		4

typedef struct {
	uint8_t crc;
	uint8_t serial[6];
	uint8_t family;
} one_rom;

void one_init();
void one_process_state();

#endif
