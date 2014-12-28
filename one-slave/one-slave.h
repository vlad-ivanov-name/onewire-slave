#ifndef ONE_SLAVE_H
#define ONE_SLAVE_H

#include <msp430.h>
#include <stdint.h>
#include <string.h>
#include "memory/offset.h"

#define ONE_PORT		2
#define ONE_B_IN		5
#define ONE_B_OUT		4

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))

typedef struct {
	uint64_t rom;
	void (* process)(uint8_t data, void * device);
	void (* init)(void * device);
	void * device;
} one_device;

void one_init(one_device * d, const uint8_t count);
void one_process_state();

#endif
