#include "one-2408.h"
#include "one-slave.h"

#define OR_REDUCE(a)		(a != 0)
#define AND_REDUCE(a)		(a == ~(a & 0))

#define REG_OFFSET			0x88
#define REG_PAGE_LENGTH		0x08

// PIO logic state
#define REG_PIO_LOGIC		0x00
// PIO output latch
#define REG_OUT_LATCH		0x01
// PIO activity latch
#define REG_ACT_LATCH		0x02
// Conditional search channel selection mask
#define REG_COND_SEL		0x03
// Conditional search channel polarity selection
#define REG_COND_POL		0x04
// Control/status register
#define REG_CSR				0x05

// Control/status register
// Pin/activity latch select
#define CSR_PLS				(1 << 0)
// Conditional search logical term
#define CSR_CT				(1 << 1)
// Power-on reset latch
#define CSR_POR				(1 << 3)
// Vcc status
#define CSR_VCC				(1 << 7)

#define CMD_PIO_READ		0xF0
#define CMD_CH_READ			0xF5
#define CMD_CH_WRITE		0x5A
#define CMD_WRITE_CS		0xCC
#define CMD_RESET_AL		0xC3

void reg_write(uint8_t address, one_2408 * d2408, uint8_t data) {
	switch (address) {
	case REG_OUT_LATCH:
		* (d2408->port_base + OFS_OUT) = ~data;
		break;
	case REG_COND_SEL:
		d2408->reg_cond_mask = data;
		break;
	case REG_COND_POL:
		* (d2408->port_base + OFS_DIR) = data;
		break;
	case REG_ACT_LATCH:
		* (d2408->port_base + OFS_IFG) = data;
		break;
	default:
		break;
	}
}

inline uint8_t reg_read(uint8_t address, one_2408 * d2408) {
	uint8_t reg_data;
	switch (address) {
		case REG_PIO_LOGIC:
			reg_data = * (d2408->port_base + OFS_IN);
			break;
		case REG_OUT_LATCH:
			reg_data = ~(* (d2408->port_base + OFS_OUT));
			break;
		case REG_ACT_LATCH:
			reg_data = * (d2408->port_base + OFS_IFG);
			break;
		case REG_COND_SEL:
			reg_data = d2408->reg_cond_mask;
			break;
		case REG_COND_POL:
			reg_data = d2408->reg_cond_pol;
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

void pio_read(void * device) {
	union {
		uint16_t data_int16;
		uint8_t data_int8[2];
	} 			crc 		= { 0 };
	uint8_t 	reg_addr;
	one_2408 * 	d2408 		= (one_2408 *) device;
	uint8_t 	i;
	uint8_t 	reg_data;

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

void write_cs_reg(void * device) {
	uint8_t 	reg_addr;
	one_2408 * 	d2408 	= (one_2408 *) device;
	uint8_t 	data;

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

void channel_access_write(void * device) {
	one_2408 *	d2408 		= (one_2408 *) device;
	uint8_t 	address		= REG_OUT_LATCH;
	uint8_t 	data;

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

void reset_activity_latch(void * device) {
	one_2408 * d2408 = (one_2408 *) device;

	reg_write(REG_ACT_LATCH, d2408, 0);

	one_write_byte(0xAA);
}

void channel_access_read(void * device) {
	union {
		uint16_t data_int16;
		uint8_t data_int8[2];
	} crc;
	uint8_t i;
	uint8_t data;
	one_2408 * d2408 = (one_2408 *) device;

	crc.data_int16 = crc16(CMD_CH_READ, 0);

	for (i = 0; i < 32; i++) {
		data = * (d2408->port_base + OFS_IN);
		one_write_byte(data);
		crc.data_int16 = crc16(data, crc.data_int16);
	}

	one_write_byte(~crc.data_int8[0]);
	one_write_byte(~crc.data_int8[1]);
}

void one_2408_process(void * device) {
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

void one_2408_init(void * device) {
	one_device * d;
	one_2408 * d2408;

	d = (one_device *) device;
	d2408 = (one_2408 *) d->device;

	* (d2408->port_base + OFS_OUT) = 0;
	* (d2408->port_base + OFS_DIR) = 0;
	* (d2408->port_base + OFS_IFG) = 0;

	d->process = &one_2408_process;
	d2408->reg_cond_mask = 0;
	d2408->reg_cond_pol = 0;
	d2408->reg_csr |= CSR_VCC | CSR_POR;
}
