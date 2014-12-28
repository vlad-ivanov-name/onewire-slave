#include <msp430.h> 
#include <stdint.h>

#include "one-slave.h"
#include "one-2408.h"

static one_device devices[] = {
	{
		.rom = 0x00AABBCCEEFF1100 | ONE_2408_FC,
		.process = &one_2408_process,
		.init = &one_2408_init,
		.device = &(one_2408) {
			.port_base = &P1IN
		}
	},
	{
		.rom = 0x00FF02030406FF00 | ONE_2408_FC,
		.process = &one_2408_process,
		.init = &one_2408_init,
		.device = &(one_2408) {
			.port_base = &P1IN
		}
	}
};

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    one_init(
    	(one_device *) &devices,
    	ARRAY_SIZE(devices)
    );

    _EINT();
    while (1) {
		LPM1;
	    one_process_state();
	}
}
