#ifndef ONE_2408_H
#define ONE_2408_H

#include <msp430.h>
#include <stdint.h>

#define ONE_2408_FC					0x29

void one_2408_process(void * device);
void one_2408_init(void * device);

typedef volatile unsigned char reg_type;

typedef struct {
	reg_type * port_base;
	uint8_t reg_csr;
	uint8_t reg_cond_mask;
	uint8_t reg_cond_pol;
} one_2408;

#endif
