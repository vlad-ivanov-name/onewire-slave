#ifndef ONE_2408_H
#define ONE_2408_H

#include <msp430.h>
#include <stdint.h>

#define ONE_2408_FC					0x29

void one_2408_process(uint8_t data, void * device);
void one_2408_init(void * device);

typedef struct {
	volatile unsigned char * port_base;
} one_2408;

#endif
