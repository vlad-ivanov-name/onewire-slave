#include "one-slave.h"

#define P6(A, B, C, D, E, F)		A ## B ## C ## D ## E ## F
#define E6(A, B, C, D, E, F)		P6(A, B, C, D, E, F)

#define P3(A, B, C)					A ## B ## C
#define E3(A, B, C)					P3(A, B, C)

#define P2(A, B)					A ## B
#define E2(A, B)					P2(A, B)

#define P1(A)						A
#define E1(A)						P1(A)

#define ONE_PDIR					E3(P, ONE_PORT, DIR)
#ifdef OUT
#undef OUT
#endif
#define ONE_POUT					E3(P, ONE_PORT, OUT)
#define ONE_PIN						E3(P, ONE_PORT, IN)
#define ONE_IE						E3(P, ONE_PORT, IE)
#define ONE_IV						E3(P, ONE_PORT, IV)
#define ONE_IES						E3(P, ONE_PORT, IES)
#define ONE_IFG						E3(P, ONE_PORT, IFG)
#define ONE_REN						E3(P, ONE_PORT, REN)

#define ONE_BIT_IN					E2(BIT, ONE_B_IN)
#define ONE_BIT_OUT					E2(BIT, ONE_B_OUT)

#define ONE_IV_IFG					E6(P, ONE_PORT, IV_P, ONE_PORT, IFG, ONE_B_IN)

#define ONE_VECTOR					E3(PORT, ONE_PORT, _VECTOR)

#define DELAY(US, CLOCK_FREQ)		(CLOCK_FREQ / 1'000'000 * US)

#define TIME_RESET_MIN				480
#define TIME_PRESENCE				150
#define TIME_TS_VALUE				30

#define COMMAND_ROM_SEARCH			0xF0
#define COMMAND_ROM_SEARCH_COND		0xEC
#define COMMAND_ROM_READ			0x33
#define COMMAND_ROM_MATCH			0x55
#define COMMAND_ROM_SKIP			0xCC

#define OUTPUT_ASSERT				(ONE_POUT |= ONE_BIT_OUT)
#define OUTPUT_DEASSERT				(ONE_POUT &= ~ONE_BIT_OUT)

#define EDGE_FALL					(ONE_IES |= ONE_BIT_IN)
#define EDGE_RISE					(ONE_IES &= ~ONE_BIT_IN)

#define INT_ENABLE					(ONE_IE |= ONE_BIT_IN)
#define INT_DISABLE					(ONE_IE &= ~ONE_BIT_IN)

typedef enum {
	state_idle,
	state_waiting_for_reset,
	state_reset,
	state_waiting_for_timeslot,
	state_timeslot
} one_state_type;

one_state_type state;
uint8_t timer_flag = 0;
uint8_t data;
uint8_t data_bits_processed;

void clock_system_setup() {
	/*
	 * Basic clock system+ setup
	 *
	 * MCLK = 8 MHz
	 * SMCLK = 1 MHz
	 */
	DCOCTL = CALDCO_8MHZ;
	BCSCTL2 |= DIVS_3;
}

inline void timer_start() {
	// Upmode
	TA0CTL |= MC_1;
}

void timer_init() {
	// SMCLK source
	// Enable interrupt
	// Upmode
	// Clear timer
	TA0CTL = TACLR + MC_0 + TAIE + TASSEL_2;
	TA0CCTL0 |= CCIE;
}

void one_init() {
	clock_system_setup();
	timer_init();
	state = state_idle;
	// GPIO setup
	ONE_POUT &= ~(ONE_BIT_IN | ONE_BIT_OUT);
	ONE_PDIR |= ONE_BIT_OUT;
	ONE_PDIR &= ~ONE_BIT_IN;
	ONE_IE |= ONE_BIT_IN;
	EDGE_FALL;
}

void process_command(uint8_t command) {
	switch (command) {
	case COMMAND_ROM_SEARCH:

		break;
	case COMMAND_ROM_SEARCH_COND:
		break;
	case COMMAND_ROM_READ:
		break;
	case COMMAND_ROM_MATCH:
		break;
	case COMMAND_ROM_SKIP:
		break;
	default:
		break;
	}
}

void process_state_idle() {
	TA0CCR0 = TIME_RESET_MIN;
	timer_start();
	//
	state = state_waiting_for_reset;
	EDGE_RISE;
}

void process_state_waiting_for_reset() {
	// Check if reset was long enough
	if (0 == timer_flag) {
		EDGE_FALL;
		state = state_idle;
	} else {
		TA0CCR0 = TIME_PRESENCE;
		timer_start();
		//
		INT_DISABLE;
		OUTPUT_ASSERT;
		//
		state = state_reset;
		timer_flag = 0;
	}
}

void process_state_reset() {
	OUTPUT_DEASSERT;
	EDGE_FALL;
	INT_ENABLE;
	state = state_waiting_for_timeslot;
	data_bits_processed = 0;
}

void process_state_waiting_for_timeslot() {
	TA0CCR0 = 0xFF00;
	timer_start();
	EDGE_RISE;
	state = state_timeslot;
}

void process_state_timeslot() {
	if (0 == timer_flag) {
		TA0CTL &= ~(BIT4 | BIT5);
		data |= (TA0R < TIME_TS_VALUE) << data_bits_processed;
		if (7 == data_bits_processed) {
			process_command(data);
			return;
		}
		data_bits_processed++;
	} else {
		EDGE_FALL;
		data_bits_processed = 0;
		timer_flag = 0;
		state = state_idle;
	}
}

void one_process_state() {
	switch (state) {
	case state_idle:
		process_state_idle();
		break;
	case state_waiting_for_reset:
		process_state_waiting_for_reset();
		break;
	case state_reset:
		process_state_reset();
		break;
	case state_waiting_for_timeslot:
		process_state_waiting_for_timeslot();
		break;
	case state_timeslot:
		process_state_timeslot();
		break;
	default:
		break;
	}
}

#pragma vector=ONE_VECTOR
__interrupt void one_interrupt() {
	if (ONE_IFG & ONE_BIT_IN) {
		ONE_IFG &= ~ONE_BIT_IN;
		LPM3_EXIT;
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void one_timer_interrupt() {
	if (TA0CTL & CCIFG) {
		timer_flag = 1;
		// Mode = MC__STOP
		TA0CTL &= ~(BIT4 | BIT5);
		TA0R = 0;
		// Clear interrupt flag
		TA0CTL &= ~CCIFG;
		LPM3_EXIT;
	}
}
