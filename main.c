#include <msp430.h> 
#include "one-slave.h"

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;
    one_init();

    _EINT();
    while (1) {
		LPM1;
	    one_process_state();
	}
}
