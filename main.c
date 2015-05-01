#include <msp430.h> 
#include <stdint.h>

#include "one-slave.h"
#include "one-2408.h"
#include "one-2408-pwm.h"

static one_device devices[] = {
	{
		.rom = 0x00AABBCCEEFF1100 | ONE_2408_FC,
		.init = &one_2408_init,
		.device = &(one_2408) {
			.port_base = &P1IN
		}
	},
	{
		.rom = 0x00FF02030406FF00 | ONE_2408_PWM_FC,
		.init = &one_2408_pwm_init,
		.device = &(one_2408_pwm) {
			.port_base = &P1IN,
			.out_bit = BIT1
		}
	}
};

void event_loop() {
	P1OUT ^= BIT0;
}

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    one_init(
    	(one_device *) &devices,
    	ARRAY_SIZE(devices)
    );

    P1DIR |= BIT0;
    P1OUT |= BIT0;

    one_set_event_loop(&event_loop);

    _EINT();
    one_start();
}
