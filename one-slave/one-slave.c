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
#define TIME_TS_OUT					20

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

#define TIMER_DIV					1
#define TIMER_MUL					1
#define DELAY_US(us)				(us / TIMER_DIV * TIMER_MUL)

typedef enum {
	state_idle,
	state_waiting_for_reset,
	state_search_rom
} one_state_type;

one_state_type state;

uint8_t timer_flag = 0;
uint8_t search_rom_cond = 0;

one_device * device;
uint8_t device_count;

void clock_system_setup() {
	/*
	 * Basic clock system+ setup
	 *
	 * MCLK = 8 MHz
	 * SMCLK = 1 MHz
	 */
	DCOCTL = CALDCO_8MHZ;
	BCSCTL1 = CALBC1_8MHZ;

	BCSCTL2 |= DIVS_3;
}

inline void timer_start() {
	// Upmode
	TA0CTL |= MC_1;
}

inline void timer_stop() {
	TA0CTL &= ~(BIT4 | BIT5);
}

void timer_init() {
	// SMCLK source
	// Enable interrupt
	// Upmode
	// Clear timer
	TA0CTL = TACLR + MC_0 + TAIE + TASSEL_2;
	TA0CCTL0 |= CCIE;
}

/*
 * Copyright: https://github.com/pbrook/arduino-onewire/
 */
uint8_t crc8(uint8_t * buffer, uint8_t size) {
	uint8_t crc = 0;
	uint8_t byte, i, m;
	buffer += size - 1;
	while (size--) {
		byte = * buffer--;
		for (i = 8; i; i--) {
			m = (crc ^ byte) & 0x01;
			crc >>= 1;
			crc ^= m ? 0x8C : 0;
			byte >>= 1;
		}
	}
	return crc;
}

void one_init(one_device * d, uint8_t count) {
	clock_system_setup();
	timer_init();
	state = state_idle;
	// GPIO setup
	ONE_POUT &= ~(ONE_BIT_IN | ONE_BIT_OUT);
	ONE_PDIR |= ONE_BIT_OUT;
	ONE_PDIR &= ~ONE_BIT_IN;
	ONE_IE |= ONE_BIT_IN;
	EDGE_FALL;
	// Devices setup
	device = d;
	device_count = count;
	uint8_t i = count - 1;
	do {
		// Calculate CRC8
		device[i].rom[0] =
			crc8(
				&(device[i].rom[1]),
				(sizeof(device[i].rom) / sizeof(typeof(device[i].rom[0]))) - 1
			);
		device[i].init(device[i].device);
	} while (i--);
}

void process_command(uint8_t command) {
	switch (command) {
	case COMMAND_ROM_SEARCH_COND:
		search_rom_cond = 1;
	case COMMAND_ROM_SEARCH:
		EDGE_FALL;
		INT_ENABLE;
		state = state_search_rom;
		one_process_state();
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
	TA0CCR0 = DELAY_US(TIME_RESET_MIN);
	timer_start();
	//
	state = state_waiting_for_reset;
	EDGE_RISE;
}

uint8_t timeslot_read() {
	TA0CCR0 = DELAY_US(TIME_TS_VALUE);
	timer_start();
	LPM1;
	return ((ONE_PIN & ONE_BIT_IN) > 0);
}

void timeslot_write(uint8_t bit) {
	LPM1;
	TA0CCR0 = DELAY_US(TIME_TS_OUT);
	if (!bit) {
		OUTPUT_ASSERT;
	}
	timer_start();
	LPM1;
	timer_stop();
	OUTPUT_DEASSERT;
}

uint8_t one_read_byte() {
	uint8_t data = 0;
	uint8_t data_bits_processed = 0;
	EDGE_FALL;
	TA0CCR0 = 0xFFFF;
	while (data_bits_processed < 8) {
		LPM1;
		data |= (timeslot_read()) << data_bits_processed;
		data_bits_processed++;
	}
	return data;
}

void process_state_waiting_for_reset() {
	// Check if reset was long enough
	if (0 == timer_flag) {
		EDGE_FALL;
		state = state_idle;
	} else {
		LPM1;
		TA0CCR0 = DELAY_US(TIME_PRESENCE);
		timer_start();
		//
		INT_DISABLE;
		OUTPUT_ASSERT;
		//
		timer_flag = 0;
		LPM1;
		//
		OUTPUT_DEASSERT;
		INT_ENABLE;
		process_command(one_read_byte());
	}
}

void process_state_search_rom() {
	uint8_t i = 63;
	uint8_t bit = 1;
	uint8_t j = 0;
	do {
		for (j = device_count; j; j--) {
			bit &= (device[j].rom[i / 8] >> (i % 8));
		}
		timeslot_write(bit);
		timeslot_write(~bit);
		timeslot_read();
		timer_stop();
	} while (i--);
	EDGE_FALL;
	state = state_idle;
}

void one_process_state() {
	switch (state) {
	case state_idle:
		process_state_idle();
		break;
	case state_waiting_for_reset:
		process_state_waiting_for_reset();
		break;
	case state_search_rom:
		process_state_search_rom();
		break;
	default:
		break;
	}
}

#pragma vector=ONE_VECTOR
__interrupt void one_interrupt() {
	if (ONE_IFG & ONE_BIT_IN) {
		ONE_IFG &= ~ONE_BIT_IN;
		LPM1_EXIT;
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void one_timer_interrupt() {
	if (TA0CTL & CCIFG) {
		TA0R = 0;
		timer_stop();
		// Clear interrupt flag
		TA0CTL &= ~CCIFG;
		timer_flag = 1;
		LPM1_EXIT;
	}
}
