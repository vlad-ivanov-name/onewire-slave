#include "one-2408-pwm.h"
#include "one-slave.h"

#define REG_OFFSET			0x88
#define REG_PAGE_LENGTH		0x08

#define REG_PIO_LOGIC		0x00
#define REG_OUT_LATCH		0x01
#define REG_ACT_LATCH		0x02
#define REG_COND_SEL		0x03
#define REG_COND_POL		0x04
#define REG_CSR				0x05

#define CSR_PLS				(1 << 0)
#define CSR_CT				(1 << 1)
#define CSR_POR				(1 << 3)
#define CSR_VCC				(1 << 7)

#define CMD_PIO_READ		0xF0
#define CMD_CH_READ			0xF5
#define CMD_CH_WRITE		0x5A
#define CMD_WRITE_CS		0xCC
#define CMD_RESET_AL		0xC3

#define PWM_STEPS			128
#define UINT8_T_MAX			0xFF

uint8_t instance_count = 0;

inline uint16_t reg_to_steps(uint8_t reg) {
	uint16_t v = reg * PWM_STEPS / UINT8_T_MAX;
	if (reg == 0) {
		v = PWM_STEPS;
	} else if (reg == UINT8_T_MAX) {
		/* FIXME needs testing */
		v = 0;
	}
	return v;
}

inline uint8_t steps_to_reg(uint8_t steps) {
	uint8_t v = steps * UINT8_T_MAX / PWM_STEPS;
	if (steps == PWM_STEPS) {
		v = 0;
	} else if (steps == 0) {
		v = UINT8_T_MAX;
	}
	return v;
}

static void reg_write(uint8_t address, one_2408_pwm * d2408, uint8_t data) {
	switch (address) {
	case REG_OUT_LATCH:
		* d2408->timer_ccr = reg_to_steps(~data);
		break;
	case REG_CSR:
		d2408->reg_csr = data;
		break;
	default:
		break;
	}
}

static inline uint8_t reg_read(uint8_t address, one_2408_pwm * d2408) {
	uint8_t reg_data;
	switch (address) {
		case REG_OUT_LATCH:
			reg_data = ~steps_to_reg(* d2408->timer_ccr);
			break;
		case REG_PIO_LOGIC:
		case REG_ACT_LATCH:
		case REG_COND_SEL:
		case REG_COND_POL:
			reg_data = 0;
			break;
		case REG_CSR:
			reg_data = d2408->reg_csr;
			break;
		default:
			reg_data = 0xFF;
			break;
	}
	return reg_data;
}

static void pio_read(void * device) {
	union {
		uint16_t data_int16;
		uint8_t data_int8[2];
	} 				crc 		= { 0 };
	uint8_t 		reg_addr;
	one_2408_pwm * 	d2408 		= (one_2408_pwm *) device;
	uint8_t 		i;
	uint8_t 		reg_data;

	reg_addr = one_read_byte();
	one_read_byte();

	crc.data_int16 = crc16(CMD_PIO_READ, crc.data_int16);
	crc.data_int16 = crc16(reg_addr, crc.data_int16);
	crc.data_int16 = crc16(0, crc.data_int16);

	for (
		i = reg_addr - REG_OFFSET;
		i < REG_PAGE_LENGTH;
		i++
	) {
		reg_data = reg_read(i, d2408);
		one_write_byte(reg_data);
		crc.data_int16 = crc16(reg_data, crc.data_int16);
	}

	one_write_byte(~crc.data_int8[0]);
	one_write_byte(~crc.data_int8[1]);
}

static void write_cs_reg(void * device) {
	uint8_t 		reg_addr;
	one_2408_pwm * 	d2408 	= (one_2408_pwm *) device;
	uint8_t 		data;

	reg_addr = one_read_byte();
	one_read_byte();

	if (
		(reg_addr < 0x8B) ||
		(reg_addr > 0x8D)
	) {
		return;
	}

	while (1) {
		if (reg_addr > 0x8D) {
			return;
		}
		data = one_read_byte();
		if (one_reset_flag) {
			return;
		}
		reg_write(reg_addr - REG_OFFSET, d2408, data);
		reg_addr++;
	}
}

static void channel_access_write(void * device) {
	one_2408_pwm *	d2408 		= (one_2408_pwm *) device;
	uint8_t 		address		= REG_OUT_LATCH;
	uint8_t 		data;

	while (1) {
		data = one_read_byte();

		if (one_reset_flag) {
			return;
		}

		if ((uint8_t) (~data) == one_read_byte()) {
			reg_write(address, d2408, data);
		}

		if (one_reset_flag) {
			return;
		}

		one_write_byte(0xAA);
		one_write_byte(data);

		address++;
	}
}

static void reset_activity_latch(void * device) {
	(void) device;
	one_write_byte(0xAA);
}

static void channel_access_read(void * device) {
	union {
		uint16_t data_int16;
		uint8_t data_int8[2];
	} crc;
	uint8_t i;
	uint8_t data;
	one_2408_pwm * d2408 = (one_2408_pwm *) device;

	crc.data_int16 = crc16(CMD_CH_READ, 0);

	for (i = 0; i < 32; i++) {
		data = steps_to_reg(* (d2408->timer_ccr));
		one_write_byte(data);
		crc.data_int16 = crc16(data, crc.data_int16);
	}

	one_write_byte(~crc.data_int8[0]);
	one_write_byte(~crc.data_int8[1]);
}

void one_2408_pwm_process(void * device) {
	uint8_t command = one_read_byte();

	switch (command) {
		case CMD_PIO_READ:
			pio_read(device);
			break;
	    case CMD_CH_READ:
	    	channel_access_read(device);
	    	break;
	    case CMD_CH_WRITE:
	    	channel_access_write(device);
	    	break;
	    case CMD_WRITE_CS:
	    	write_cs_reg(device);
	    	break;
	    case CMD_RESET_AL:
	    	reset_activity_latch(device);
	    	break;
	    default:
	    	break;
	}
}

static void pwm_timer_setup() {
    TA1CCR0 = PWM_STEPS;

    TA1CCTL1 = OUTMOD_6;
    TA1CCTL2 = OUTMOD_6;

    TA1CCR1 = PWM_STEPS;
    TA1CCR2 = PWM_STEPS;

    TA1CTL = TASSEL_2 + MC_3;
}

void one_2408_pwm_init(void * device) {
	if (instance_count == 0) {
		pwm_timer_setup();
	}

	instance_count++;

	if (instance_count > 2) {
		return;
	}

	one_device * d;
	one_2408_pwm * d2408_pwm;

	d = (one_device *) device;
	d2408_pwm = (one_2408_pwm *) d->device;

	* (d2408_pwm->port_base + OFS_DIR) |= d2408_pwm->out_bit;
	* (d2408_pwm->port_base + OFS_SEL) |= d2408_pwm->out_bit;

	switch(instance_count) {
	case 1:
		d2408_pwm->timer_ccr = &TA1CCR1;
	case 2:
		d2408_pwm->timer_ccr = &TA1CCR2;
	default:
		break;
	}

	d->process = &one_2408_pwm_process;
	d->condition = &one_condition_dummy;

	d2408_pwm->reg_cond_mask = 0;
	d2408_pwm->reg_cond_pol = 0;
	d2408_pwm->reg_csr |= CSR_VCC | CSR_POR;
}
