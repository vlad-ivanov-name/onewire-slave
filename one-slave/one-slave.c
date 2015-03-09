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
#define TIME_PRESENCE				130
#define TIME_PRESENCE_DELAY			15
#define TIME_TS_VALUE				30
#define TIME_TS_OUT					25

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
#define TIMER_MUL					2
#define DELAY_US(us)				((us / TIMER_DIV) * TIMER_MUL)

#define MAX_DEVICE_COUNT			4

#define MASK_LOWER_4 				0x0F
#define MASK_HIGH_4 				0xF0

#define LSB_EXTEND(a)				((a & 0x01) ? (~(0 & a)) : 0)

typedef enum {
	state_idle,
	state_waiting_for_reset,
	state_search_rom,
	state_match_rom,
	state_device_selected
} one_state_type;

one_state_type state;

uint8_t search_rom_cond = 0;
uint8_t selected_device;
uint8_t reset_lpm_exit = 0;
uint8_t one_reset_flag = 0;

one_device * device;
uint8_t device_count;

union {
	uint8_t data_int8[32];
	uint16_t data_int16[16];
} device_addr = { 0 };

void clock_system_setup() {
	/*
	 * Basic clock system+ setup
	 *
	 * MCLK = 16 MHz
	 * SMCLK = 2 MHz
	 */
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;

	BCSCTL2 |= DIVS_3;
}

inline void timer_start() {
	TA0R = 0;
	// Upmode
	TA0CTL |= MC_2;
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
	TA0CCTL1 |= CCIE;
	TA0CCR1 = DELAY_US(TIME_RESET_MIN);
}

/*
 * Copyright: https://github.com/pbrook/arduino-onewire/
 */
uint8_t crc8(uint8_t * buffer, uint8_t size) {
	uint8_t crc = 0;
	uint8_t byte, i, m;
	while (size--) {
		byte = * buffer++;
		for (i = 8; i; i--) {
			m = (crc ^ byte) & 0x01;
			crc >>= 1;
			crc ^= m ? 0x8C : 0;
			byte >>= 1;
		}
	}
	return crc;
}

/*
 * Copyright: https://github.com/pbrook/arduino-onewire/
 */
uint16_t crc16(uint8_t input, uint16_t seed) {
	static const uint8_t oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };
	uint16_t cdata = input;
	cdata = (cdata ^ (seed & 0xff)) & 0xff;
	seed >>= 8;
	if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4]) {
		seed ^= 0xC001;
	}
	cdata <<= 6;
	seed ^= cdata;
	cdata <<= 1;
	seed ^= cdata;
	return seed;
}

void one_init(one_device * d, const uint8_t count) {
	if (count > 4) {
		return;
	}
	//
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
	uint8_t crc_array[7];
	do {
		// Calculate CRC8
		memcpy(&crc_array, (void *) &device[i].rom, 7);
		device[i].rom |= (uint64_t) crc8((uint8_t *) &crc_array, 7) << 56;
		device[i].init(&device[i]);
	} while (i--);
	// Fill device address array
	uint8_t device_index;
	uint8_t current;
	for (i = 0; i < 64; i++) {
		for (device_index = 0; device_index < device_count; device_index++) {
			current = device[device_index].rom & ((uint64_t)1 << i) ? 1 : 0;
			if (i & 0x01) {
				device_addr.data_int8[i >> 1] |= current << (device_index + 4);
			} else {
				device_addr.data_int8[i >> 1] |= current << (device_index);
			}
		}
	}
}

void process_command(uint8_t command) {
	switch (command) {
	case COMMAND_ROM_SEARCH_COND:
		search_rom_cond = 1;
	case COMMAND_ROM_SEARCH:
		INT_ENABLE;
		state = state_search_rom;
		break;
	case COMMAND_ROM_MATCH:
		state = state_match_rom;
		break;
	default:
		state = state_idle;
		break;
	}
}

void process_state_idle() {
	LPM1;
	TA0CCR0 = 0xFFFF;
	timer_start();
	reset_lpm_exit = 1;
	//
	LPM1;
}

uint8_t timeslot_read() {
	LPM1;
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
	OUTPUT_DEASSERT;
}

void one_write_byte(uint8_t data) {
	uint8_t data_bits_processed = 0;
	EDGE_FALL;
	TA0CCR0 = 0xFFFF;
	while ((data_bits_processed < 8) && !one_reset_flag) {
		timeslot_write(data & (1 << data_bits_processed));
		data_bits_processed++;
	}
	timer_stop();
}

uint8_t one_read_byte() {
	uint8_t data = 0;
	uint8_t data_bits_processed = 0;
	EDGE_FALL;
	TA0CCR0 = 0xFFFF;
	while ((data_bits_processed < 8) && !one_reset_flag) {
		data |= (timeslot_read()) << data_bits_processed;
		data_bits_processed++;
	}
	timer_stop();
	return data;
}

void process_state_waiting_for_reset() {
	one_reset_flag = 0;
	while ((ONE_BIT_IN & ONE_PIN) == 0);
	// Send presence response
	TA0CCR0 = DELAY_US(TIME_PRESENCE_DELAY);
	timer_start();
	LPM1;
	//
	TA0CCR0 = DELAY_US(TIME_PRESENCE);
	timer_start();
	//
	INT_DISABLE;
	OUTPUT_ASSERT;
	//
	LPM1;
	//
	OUTPUT_DEASSERT;
	INT_ENABLE;
	timer_stop();
	process_command(one_read_byte());
}

void process_state_search_rom() {
	uint8_t mask = (MASK_HIGH_4 >> (4 - device_count)) & MASK_LOWER_4;
	union {
		uint8_t data_int8[32];
		uint16_t data_int16[16];
	} search_addr;
	uint8_t index;
	uint8_t direction;
	uint8_t addr_bits;

	for (index = 0; index < 16; index++) {
		search_addr.data_int16[index] = device_addr.data_int16[index];
	}

	if (search_rom_cond) {
		for (index = 0; index < device_count; index++) {
			mask |= ((!device[index].condition(device[index].device)) << index);
		}
	}

	if (mask == 0) {
		goto exit;
	}

	index = 0;

	while (!one_reset_flag) {
		addr_bits =
			(index & 0x01) ?
			search_addr.data_int8[index >> 1] >> 4 :
			search_addr.data_int8[index >> 1] & MASK_LOWER_4;

		addr_bits |= mask;
		timeslot_write(addr_bits == MASK_LOWER_4);
		timeslot_write(addr_bits == mask);

		direction = timeslot_read();
		direction = LSB_EXTEND(direction);
		// direction: either 0xFF or 0x00
		// addr_bits: 0000 ABCD
		mask |= (addr_bits ^ direction) & MASK_LOWER_4;
		if ((mask == MASK_LOWER_4) || (index == 63)) {
			break;
		}
		index++;
	}

exit:
	search_rom_cond = 0;
	state = state_idle;
}

void process_state_match_rom() {
	volatile union {
		uint64_t data_int64;
		uint8_t data_int8[8];
	} address;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		address.data_int8[i] = one_read_byte();
	}

	for (i = 0; i < device_count; i++) {
		if (device[i].rom == address.data_int64) {
			state = state_device_selected;
			selected_device = i;
			return;
		}
	}

	state = state_idle;
}

void process_state_device_selected() {
	void * device_data = device[selected_device].device;
	device[selected_device].process(device_data);
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
	case state_match_rom:
		process_state_match_rom();
		break;
	case state_device_selected:
		process_state_device_selected();
		break;
	default:
		break;
	}
	if (one_reset_flag) {
		state = state_waiting_for_reset;
	}
}

uint8_t one_condition_dummy(void * device) {
	return 0;
}

#pragma vector=ONE_VECTOR
__interrupt void one_interrupt() {
	if (ONE_IFG & ONE_BIT_IN) {
		ONE_IFG &= ~ONE_BIT_IN;
		LPM1_EXIT;
	}
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void one_timer_timeslot() {
	LPM1_EXIT;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void one_timer_reset() {
	switch (__even_in_range(TA0IV, TA0IV_TAIFG)) {
	case TA0IV_TACCR1:
		timer_stop();
		one_reset_flag = (ONE_PIN & ONE_BIT_IN) == 0;
		if (reset_lpm_exit || one_reset_flag) {
			reset_lpm_exit = 0;
			LPM1_EXIT;
		}
		break;
	default:
		timer_stop();
		break;
	}
}
