#ifndef ONE_2408_PWM_H
#define ONE_2408_PWM_H

#include <msp430.h>
#include <stdint.h>

#define ONE_2408_PWM_FC					0x29

void one_2408_pwm_process(void * device);
void one_2408_pwm_init(void * device);

typedef volatile unsigned char reg8_type;
typedef volatile unsigned int reg16_type;

typedef struct {
	reg8_type  * port_base;
	reg16_type * timer_ccr;
	uint8_t out_bit;
	uint8_t reg_csr;
} one_2408_pwm;

#endif
