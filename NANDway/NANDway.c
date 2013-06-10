/************************************************************************
NANDway.c (v0.61) - Teensy++ 2.0 NAND flasher for PS3

Copyright (C) 2013	Effleurage
					judges <judges@eEcho.com>

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "delay_x.h"
#include "usb_serial.h"
//#include "clz_ctz.h"

#define VERSION_MAJOR			0
#define VERSION_MINOR			61

#define BUILD_DUAL_NAND			1
#define BUILD_SIGNAL_BOOSTER	2

#if ((BUILD_VERSION != BUILD_DUAL_NAND) && (BUILD_VERSION != BUILD_SIGNAL_BOOSTER))
	#error BUILD_VERSION must be defined!
#endif

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

// Define state
#define S_IDLE			0
#define S_DELAY			1
#define S_ADDR2			2
#define S_ADDR3			3
#define S_READING		4
#define S_WDATA1		5
#define S_WDATA2		6
#define S_WRITING		7
#define S_WAITING		8
#define S_WRITEWORD		9
#define S_WRITEWORDUBM	10
#define S_WRITEWBP		11

/*! \brief NAND flash read page command start. */
#define NAND_COMMAND_READ1				0x00
/*! \brief NAND flash read page command end. */
#define NAND_COMMAND_READ2				0x30
/*! \brief NAND flash read ID command. */
#define NAND_COMMAND_READID				0x90
/*! \brief NAND flash reset command. */
#define NAND_COMMAND_RESET				0xFF
/*! \brief NAND flash program page command start. */
#define NAND_COMMAND_PAGEPROG1			0x80
/*! \brief NAND flash program page command end. */
#define NAND_COMMAND_PAGEPROG2			0x10
/*! \brief NAND flash erase block command start. */
#define NAND_COMMAND_ERASE1				0x60
/*! \brief NAND flash erase block command end. */
#define NAND_COMMAND_ERASE2				0xD0
/*! \brief NAND flash read status command. */
#define NAND_COMMAND_STATUS				0x70
/*! \brief NAND flash random program page command start. */
#define NAND_COMMAND_RANDOM_PAGEPROG	0x85
/*! \brief NAND flash random read page command start. */
#define NAND_COMMAND_RANDOM_READ1		0x05
/*! \brief NAND flash random read page command end. */
#define NAND_COMMAND_RANDOM_READ2		0xE0

#define NAND_STATUS_FAIL 			(1<<0) /* HIGH - FAIL,  LOW - PASS */
#define NAND_STATUS_IDLE 			(1<<5) /* HIGH - IDLE,  LOW - ACTIVE */
#define NAND_STATUS_READY 			(1<<6) /* HIGH - READY, LOW - BUSY */
#define NAND_STATUS_NOT_PROTECTED 	(1<<7) /* HIGH - NOT,   LOW - PROTECTED */

#define BUF_SIZE_RW		4320
#define BUF_SIZE_ADDR	3

uint16_t 	PAGE_PLUS_RAS_SZ = 0; /* page size + Redundant Area Size */
uint8_t		buf_rw[BUF_SIZE_RW];
uint8_t		buf_addr[BUF_SIZE_ADDR];

#if BUILD_VERSION == BUILD_DUAL_NAND
	#define NAND0_IO_PORT 	PORTF
	#define NAND0_IO_PIN	PINF
	#define NAND0_IO_DDR	DDRF

	#define NAND0_CONT_PORT PORTB
	#define NAND0_CONT_PIN	PINB
	#define NAND0_CONT_DDR	DDRB

	#define NAND1_IO_PORT 	PORTC
	#define NAND1_IO_PIN	PINC
	#define NAND1_IO_DDR	DDRC

	#define NAND1_CONT_PORT PORTD
	#define NAND1_CONT_PIN	PIND
	#define NAND1_CONT_DDR	DDRD
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
	#define NAND0_IO_PORT 	PORTF
	#define NAND0_IO_PIN	PINF
	#define NAND0_IO_DDR	DDRF

	#define NAND0_CONT_PORT PORTE
	#define NAND0_CONT_PIN	PINE
	#define NAND0_CONT_DDR	DDRE

	#define NAND0_CONT_RE_PORT PORTA
	#define NAND0_CONT_RE_PIN	PINA
	#define NAND0_CONT_RE_DDR	DDRA

	#define NAND0_CONT_CLE_PORT PORTB
	#define NAND0_CONT_CLE_PIN	PINB
	#define NAND0_CONT_CLE_DDR	DDRB

	#define NAND0_CONT_ALE_PORT PORTD
	#define NAND0_CONT_ALE_PIN	PIND
	#define NAND0_CONT_ALE_DDR	DDRD

	#define NAND0_CONT_WE_PORT PORTC
	#define NAND0_CONT_WE_PIN	PINC
	#define NAND0_CONT_WE_DDR	DDRC
#endif

/*! \brief NAND flash information about maker, device, size and timing.
*/
typedef struct _nand_info {
	uint8_t raw_data[5];
	
	/*! \brief Out of bounce layout information. */
	uint16_t	oob_size;

	/*! \brief Maker code. */
	uint8_t		maker_code;
	/*! \brief Device code. */
	uint8_t		device_code;

	/*! \brief Page size in bytes. */
	uint32_t	page_size;
	/*! \brief Number of positions to shift when converting an offset in
	 *         a block to page. Used when calculating NAND flash address.
	 */ 
	//uint32_t	page_shift;
	/*! \brief Number of pages per block. */
	uint32_t	pages_per_block;
	/*! \brief Block size in bytes. */
	uint32_t	block_size;
	/*! \brief Number of positions to shift when converting block number
	 *         to NAND flash address.
	 */ 
	//uint32_t	block_shift;
	/*! \brief Number of blocks. */
	uint32_t	num_blocks;
	/*! \brief NAND flash I/O bus width in bits. */
	uint8_t		bus_width;

	/*! \brief NAND flash number of planes. */
	uint8_t		num_planes;
	/*! \brief NAND flash plane size. */
	uint32_t 	plane_size;
} nand_info;

typedef struct _nand_port {
	nand_info info;
	
	volatile uint8_t *io_port;
	volatile uint8_t *io_pin;
	volatile uint8_t *io_ddr;
	
	volatile uint8_t *cont_port;
	volatile uint8_t *cont_pin;
	volatile uint8_t *cont_ddr;

#if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
	volatile uint8_t *cont_re_port;
	volatile uint8_t *cont_re_pin;
	volatile uint8_t *cont_re_ddr;

	volatile uint8_t *cont_cle_port;
	volatile uint8_t *cont_cle_pin;
	volatile uint8_t *cont_cle_ddr;

	volatile uint8_t *cont_ale_port;
	volatile uint8_t *cont_ale_pin;
	volatile uint8_t *cont_ale_ddr;

	volatile uint8_t *cont_we_port;
	volatile uint8_t *cont_we_pin;
	volatile uint8_t *cont_we_ddr;
#endif

} nand_port;


#if BUILD_VERSION == BUILD_DUAL_NAND
	nand_port nand0 = { .io_port = &NAND0_IO_PORT, .io_pin = &NAND0_IO_PIN, .io_ddr = &NAND0_IO_DDR, .cont_port = &NAND0_CONT_PORT, .cont_pin = &NAND0_CONT_PIN, .cont_ddr = &NAND0_CONT_DDR };
	nand_port nand1 = { .io_port = &NAND1_IO_PORT, .io_pin = &NAND1_IO_PIN, .io_ddr = &NAND1_IO_DDR, .cont_port = &NAND1_CONT_PORT, .cont_pin = &NAND1_CONT_PIN, .cont_ddr = &NAND1_CONT_DDR };

	#define NAND_CONT_CEal		(1<<0)		// Chip Enable
	#define NAND_CONT_REal		(1<<1)		// Read Enable
	#define NAND_CONT_CLE		(1<<2)		// Command Latch Enable
	#define NAND_CONT_ALE		(1<<3)		// Address Latch Enable
	#define NAND_CONT_WPal		(1<<4)		// Write Protect
	#define NAND_CONT_WEal		(1<<5)		// Write Enable
	#define NAND_CONT_RYBY		(1<<6)		// Ready/Busy (ready - high, busy - low)
	#define NAND_CONT_TRI		(1<<7)		// Tristate (ready - high, busy - low)

	#define NAND_TOGGLE_WE(_nand_)			*((_nand_)->cont_port) &= ~(NAND_CONT_WEal); *((_nand_)->cont_port) |= NAND_CONT_WEal
	#define NAND_IO_READ(_nand_, _data_)	*((_nand_)->cont_port) &= ~(NAND_CONT_REal); _delay_ns(100);  (_data_) = *((_nand_)->io_pin); *((_nand_)->cont_port) |= NAND_CONT_REal
	#define NAND_COMMAND(_nand_, _cmd_)		*((_nand_)->io_port)=(_cmd_); *((_nand_)->cont_port) |= NAND_CONT_CLE; NAND_TOGGLE_WE(_nand_); *((_nand_)->cont_port) &= ~(NAND_CONT_CLE)
	#define NAND_ALE_HIGH(_nand_)			*((_nand_)->cont_port) |= NAND_CONT_ALE
	#define NAND_ALE_LOW(_nand_)			*((_nand_)->cont_port) &= ~(NAND_CONT_ALE)
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
	nand_port nand0 = { .io_port = &NAND0_IO_PORT, .io_pin = &NAND0_IO_PIN, .io_ddr = &NAND0_IO_DDR,
						.cont_port = &NAND0_CONT_PORT, .cont_pin = &NAND0_CONT_PIN, .cont_ddr = &NAND0_CONT_DDR,
						.cont_re_port = &NAND0_CONT_RE_PORT, .cont_re_pin = &NAND0_CONT_RE_PIN, .cont_re_ddr = &NAND0_CONT_RE_DDR,
						.cont_cle_port = &NAND0_CONT_CLE_PORT, .cont_cle_pin = &NAND0_CONT_CLE_PIN, .cont_cle_ddr = &NAND0_CONT_CLE_DDR,
						.cont_ale_port = &NAND0_CONT_ALE_PORT, .cont_ale_pin = &NAND0_CONT_ALE_PIN, .cont_ale_ddr = &NAND0_CONT_ALE_DDR,
						.cont_we_port = &NAND0_CONT_WE_PORT, .cont_we_pin = &NAND0_CONT_WE_PIN, .cont_we_ddr = &NAND0_CONT_WE_DDR };

	#define NAND_CONT_WPal		(1<<6)		// Write Protect
	#define NAND_CONT_RYBY		(1<<7)		// Ready/Busy (ready - high, busy - low)

	#define NAND_TOGGLE_WE(_nand_)			*((_nand_)->cont_we_port) = 0; *((_nand_)->cont_we_port) = 0xFF
	#define NAND_IO_READ(_nand_, _data_)	*((_nand_)->cont_re_port) = 0; _delay_ns(100); (_data_) = *((_nand_)->io_pin); *((_nand_)->cont_re_port) = 0xFF
	#define NAND_COMMAND(_nand_, _cmd_)		*((_nand_)->io_port)=(_cmd_); *((_nand_)->cont_cle_port) = 0xFF; NAND_TOGGLE_WE(_nand_); *((_nand_)->cont_cle_port) = 0
	#define NAND_ALE_HIGH(_nand_)			*((_nand_)->cont_ale_port) = 0xFF
	#define NAND_ALE_LOW(_nand_)			*((_nand_)->cont_ale_port) = 0
#endif

#define NAND_IO_INPUT(_nand_)				*((_nand_)->io_ddr) = 0x00; *((_nand_)->io_port) = 0xFF
#define NAND_IO_OUTPUT(_nand_)				*((_nand_)->io_ddr) = 0xFF
#define NAND_IO_SET(_nand_, _data_)			*((_nand_)->io_port)=(_data_); NAND_TOGGLE_WE(_nand_)
#define NAND_BUSY_WAIT(_nand_, _us_)		while (1) { uint8_t status; _delay_us(_us_); status = nand_status(nandp); if (status & NAND_STATUS_READY) {	break; } }


void nand_enable(nand_port *nandp)
{
#if BUILD_VERSION == BUILD_DUAL_NAND
	*(nandp->cont_ddr) = 0xFF; 			// all control ports - output
	*(nandp->cont_ddr) &= ~NAND_CONT_RYBY; /* ready / busy - input */

	*(nandp->cont_port) = 	NAND_CONT_WEal |
							NAND_CONT_REal |
							NAND_CONT_WPal |
							NAND_CONT_RYBY; /* input - pull up */
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
	*(nandp->cont_ddr) = 0xFF; 			// all control ports - output
	*(nandp->cont_ddr) &= ~NAND_CONT_RYBY; /* ready / busy - input */
	*(nandp->cont_port) = NAND_CONT_WPal | NAND_CONT_RYBY; /* input - pull up */

	*(nandp->cont_we_ddr) = 0xFF; 		// all control ports - output
	*(nandp->cont_we_port) = 0xFF;

	*(nandp->cont_re_ddr) = 0xFF; 		// all control ports - output
	*(nandp->cont_re_port) = 0xFF;

	*(nandp->cont_ale_ddr) = 0xFF; 		// all control ports - output
	*(nandp->cont_ale_port) = 0;

	*(nandp->cont_cle_ddr) = 0xFF; 		// all control ports - output
	*(nandp->cont_cle_port) = 0;
#endif

	NAND_IO_OUTPUT(nandp); // io set as output
}

void releaseports()
{
	DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
}

uint8_t wait_ryby(nand_port *nandp)
{
	/* Should be done within 3 milliseconds for all commands. */
	volatile uint32_t timeout = 0x200000; //approx. 3secs

	while (timeout > 0) {
		if ( *(nandp->cont_pin) & NAND_CONT_RYBY ) {
			return 1;
		}
		--timeout;
	}

	return 0;
}

void nand_reset(nand_port *nandp)
{
	nand_enable(nandp);

	NAND_COMMAND(nandp, NAND_COMMAND_RESET);

	wait_ryby(nandp);
}

uint8_t nand_read_id(nand_port *nandp)
{
	uint32_t spare_size;
	uint8_t maker_code;
	uint8_t device_code;
	uint8_t chip_data;
	uint8_t size_data;
	uint8_t plane_data;
	uint8_t plane_size;
	uint8_t block_size;
	
	nand_enable(nandp);
	
	NAND_COMMAND(nandp, NAND_COMMAND_READID);

	/* follow by address - 0 */
	NAND_ALE_HIGH(nandp);
	NAND_IO_SET(nandp, 0x00);
	NAND_ALE_LOW(nandp);

	NAND_IO_INPUT(nandp);
	NAND_IO_READ(nandp, maker_code);
	NAND_IO_READ(nandp, device_code);
	NAND_IO_READ(nandp, chip_data);
	NAND_IO_READ(nandp, size_data);
	NAND_IO_READ(nandp, plane_data);
	
	//maker_code = 0xec;
	//device_code = 0xf1;
	//chip_data = 0xa0;
	//size_data = 0x15;
	//plane_data = 0x40;
	
	nandp->info.raw_data[0] = maker_code;
	nandp->info.raw_data[1] = device_code;
	nandp->info.raw_data[2] = chip_data;
	nandp->info.raw_data[3] = size_data;
	nandp->info.raw_data[4] = plane_data;

	nandp->info.maker_code = maker_code;
	nandp->info.device_code = device_code;

	if ((maker_code == 0xAD) && (device_code == 0xD7)) { // Hynix H27UBG8T2A
		/* Fill the NAND structure parameters */
		nandp->info.page_size  = 0x800 << (size_data & 0x03);
		//nandp->info.page_shift = ctz(nandp->info.page_size);
		nandp->info.bus_width = 8;
		nandp->info.num_planes = (1 << ((plane_data >> 2) & 0x03));
		nandp->info.plane_size = 2048UL * 1024UL * 1024UL;

		block_size = (size_data & 0xB0);
		/* Store the plane size in bytes. */
		switch (block_size) {
			case 0x0:
				nandp->info.block_size = 128UL * 1024UL;
			break;
			case 0x10:
				nandp->info.block_size = 256UL * 1024UL;
				break;
			case 0x20:
				nandp->info.block_size = 512UL * 1024UL;
				break;
			case 0x30:
				nandp->info.block_size = 768UL * 1024UL;
				break;
			case 0x80:
				nandp->info.block_size = 1024UL * 1024UL;
				break;
			case 0x90:
				nandp->info.block_size = 2048UL * 1024UL;
				break;
			default:
				return 0;
		}

		spare_size = (size_data >> 2) & 0x03;
		switch (spare_size) {
			case 0x0:
				nandp->info.oob_size = 128;
				break;
			case 0x1:
				nandp->info.oob_size = 224;
				break;
			case 0x2:
				nandp->info.oob_size = 448;
				break;
			default:
				return 0;
		}
	}
	else if ((maker_code == 0xAD) && (device_code == 0x73)) { // Hynix HY27US08281A
		nandp->info.page_size  = 512;
		nandp->info.block_size = 32UL * nandp->info.page_size;
		nandp->info.num_planes = 1;
		nandp->info.oob_size = 64;
		nandp->info.plane_size = 1UL << 24;
		nandp->info.bus_width = 8;
	}
	else {
		if ((maker_code == 0xEC) && (device_code == 0xF1)) // Samsung K9F1G08U0A
			plane_data = 0x40;
		else if ((maker_code == 0xEC) && (device_code == 0xA1)) // Samsung K9F1G08R0A
			plane_data = 0x40;
		
		/* Fill the NAND structure parameters */
		nandp->info.page_size  = 0x400 << (size_data & 0x03);
		//nandp->info.page_shift = ctz(nandp->info.page_size);
		nandp->info.block_size = (64UL * 1024UL) << ((size_data & 0x30) >> 4);

		nandp->info.num_planes = (1 << ((plane_data >> 2) & 0x03));

		spare_size = (8 << ((size_data & 0x04) >> 2)) * (nandp->info.page_size / 512);
		switch (spare_size) {
			case 16:
			case 64:
				nandp->info.oob_size = spare_size;
				break;
			default:
				return 0;
		}

		plane_size = (plane_data >> 4) & 0x07;
		/* Store the plane size in bytes. */
		switch (plane_size) {
			case 0x0:
				nandp->info.plane_size = 1UL << 23;
				break;
			case 0x1:
				nandp->info.plane_size = 1UL << 24;
				break;
			case 0x2:
				nandp->info.plane_size = 1UL << 25;
				break;
			case 0x3:
				nandp->info.plane_size = 1UL << 26;
				break;
			case 0x4:
				nandp->info.plane_size = 1UL << 27;
				break;
			case 0x5:
				nandp->info.plane_size = 1UL << 28;
				break;
			case 0x6:
				nandp->info.plane_size = 1UL << 29;
				break;
			case 0x7:
				nandp->info.plane_size = 1UL << 30;
				break;
			default:
				return 0;
		}

		if ((size_data & 0x40) == 0) {
			nandp->info.bus_width = 8;
		} else {
			nandp->info.bus_width = 16;
		}
	}

	nandp->info.num_blocks = nandp->info.num_planes * nandp->info.plane_size / nandp->info.block_size;
	nandp->info.pages_per_block = nandp->info.block_size / nandp->info.page_size;
	//nandp->info.block_shift = ctz(nandp->info.pages_per_block);

	PAGE_PLUS_RAS_SZ = nandp->info.page_size + nandp->info.oob_size;

	return 1;
}

uint8_t nand_read_page(nand_port *nandp) {
	uint16_t i;
	uint8_t sreg;
	
	nand_enable(nandp);
	
	/* read command */
	NAND_COMMAND(nandp, NAND_COMMAND_READ1);

	/* address */
	NAND_ALE_HIGH(nandp);
	if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73)) {
		NAND_IO_SET(nandp, 0);
		NAND_IO_SET(nandp, buf_addr[0]);
		NAND_IO_SET(nandp, buf_addr[1]);
	}
	else {
		NAND_IO_SET(nandp, 0);
		NAND_IO_SET(nandp, 0);
		NAND_IO_SET(nandp, buf_addr[0]);
		NAND_IO_SET(nandp, buf_addr[1]);
		NAND_IO_SET(nandp, buf_addr[2]);
	}	
	NAND_ALE_LOW(nandp);

	if ((nandp->info.maker_code != 0xAD) && (nandp->info.device_code != 0x73))
		NAND_COMMAND(nandp, NAND_COMMAND_READ2);
	
	NAND_IO_INPUT(nandp);
	
	/* Save global interrupt flag and disable interrupts */
	sreg = SREG;
	cli();
	
	/* wait for the nand to read this page to the internal page register */
	wait_ryby(nandp);

	for (uint8_t k = 0; k < PAGE_PLUS_RAS_SZ / BUF_SIZE_RW; k++) {
		for (i = 0; i < BUF_SIZE_RW; i++) {
			NAND_IO_READ(nandp, buf_rw[i]);
		}
		usb_serial_write(buf_rw, BUF_SIZE_RW);
	}
		
	uint16_t rest = PAGE_PLUS_RAS_SZ - ((PAGE_PLUS_RAS_SZ / BUF_SIZE_RW) * BUF_SIZE_RW);
	for (i = 0; i < rest; i++) {
		NAND_IO_READ(nandp, buf_rw[i]);
	}
	usb_serial_write(buf_rw, rest);

	/* Restore global interrupt flag */
	SREG = sreg;

	return 1;
}

uint8_t nand_status(nand_port *nandp) {
	uint8_t status;
	
	NAND_COMMAND(nandp, NAND_COMMAND_STATUS);

	NAND_IO_INPUT(nandp);
	
	NAND_IO_READ(nandp, status);

	NAND_IO_OUTPUT(nandp);

	return status;
}

int8_t nand_write_page(nand_port *nandp) {
	uint16_t i;
	int16_t in_data;
	
	nand_enable(nandp);

	/* Serial Data Input command */
	/* CLE - high & CE# - low */
	NAND_COMMAND(nandp, NAND_COMMAND_PAGEPROG1);

	/* address */
	/* ALE on & CLE off */ 
	NAND_ALE_HIGH(nandp);
	if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73)) {
		NAND_IO_SET(nandp, 0);
		NAND_IO_SET(nandp, buf_addr[0]);
		NAND_IO_SET(nandp, buf_addr[1]);
	}
	else {
		NAND_IO_SET(nandp, 0);
		NAND_IO_SET(nandp, 0);
		NAND_IO_SET(nandp, buf_addr[0]);
		NAND_IO_SET(nandp, buf_addr[1]);
		NAND_IO_SET(nandp, buf_addr[2]);
	}
	NAND_ALE_LOW(nandp);
	
	for (i = 0; i < PAGE_PLUS_RAS_SZ; i++) {
		if ((in_data = usb_serial_getchar()) != -1) {
			NAND_IO_SET(nandp, in_data);
		}
		else {
			break;
		}
	}

	/* Page Program confirm command */
	NAND_COMMAND(nandp, NAND_COMMAND_PAGEPROG2);

	if (i < PAGE_PLUS_RAS_SZ) { // timeout
		return -1;		// and exit
	}

	/* wait for the internal controller to finish the program command 
		TBD - up to 200us */
	NAND_BUSY_WAIT(nandp, 5.0);

	return !(nand_status(nandp) & NAND_STATUS_FAIL);
}

int8_t nand_erase_block(nand_port *nandp) {
	nand_enable(nandp);

	/* block erase setup command */
	NAND_COMMAND(nandp, NAND_COMMAND_ERASE1);

	/* block address */
	/* ALE on & CLE off */ 
	NAND_ALE_HIGH(nandp);
	if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73)) {
		NAND_IO_SET(nandp, buf_addr[0]);
		NAND_IO_SET(nandp, buf_addr[1]);
	}
	else {
		NAND_IO_SET(nandp, buf_addr[0]);
		NAND_IO_SET(nandp, buf_addr[1]);
		NAND_IO_SET(nandp, buf_addr[2]);
	}
	NAND_ALE_LOW(nandp);
	
	/* block erase confirm command */
	NAND_COMMAND(nandp, NAND_COMMAND_ERASE2);

	if (!(nand_status(nandp) & NAND_STATUS_NOT_PROTECTED)) {
		return -1;
	}
		
	/* wait for the internal controller to finish the erase command 
		TBD - up to 2ms */
	NAND_BUSY_WAIT(nandp, 20.0);

	return !(nand_status(nandp) & NAND_STATUS_FAIL);
}

void handle_read_page(nand_port *nand) {
	uint16_t i;
	int16_t in_data;

	for (i = 0; i < BUF_SIZE_ADDR; i++) {
		if ((in_data = usb_serial_getchar()) != -1)
			buf_addr[i] = in_data;
		else
			break;
	}
	
	if (i < BUF_SIZE_ADDR) {	// timeout
		usb_serial_putchar('R');
		return;		// and exit
	}
	
	usb_serial_putchar('K');
	nand_read_page(nand);
}

void handle_write_page(nand_port *nand) {
	int8_t res;
	uint16_t i;
	int16_t in_data;
	
	for (i = 0; i < BUF_SIZE_ADDR; i++) {
		if ((in_data = usb_serial_getchar()) != -1)
			buf_addr[i] = in_data;
		else
			break;
	}
	
	if (i < BUF_SIZE_ADDR) {	// timeout
		usb_serial_putchar('R');
		return;		// and exit
	}

	res = nand_write_page(nand);
	if (res > 0)
		usb_serial_putchar('K');
	else if (res < 0)
		usb_serial_putchar('R');
	else
		usb_serial_putchar('V');
}

void handle_erase_block(nand_port *nand) {
	int8_t res;
	uint16_t i;
	int16_t in_data;

	for (i = 0; i < BUF_SIZE_ADDR; i++) {
		if ((in_data = usb_serial_getchar()) != -1)
			buf_addr[i] = in_data;
		else
			break;
	}
	
	if (i < BUF_SIZE_ADDR) {	// if receiving timeout, send FAIL!
		usb_serial_putchar('R');
		return;
	}
	
	res = nand_erase_block(nand);
	if (res > 0)
		usb_serial_putchar('K');
	else if (res < 0)
		usb_serial_putchar('P');
	else
		usb_serial_putchar('V');
}

void handle_read_id(nand_port *nand) {
	nand_reset(nand);
	if (nand_read_id(nand)) {
		//25 bytes
		usb_serial_putchar(nand->info.raw_data[0]); //maker_code
		usb_serial_putchar(nand->info.raw_data[1]); //device_code
		usb_serial_putchar(nand->info.raw_data[2]);
		usb_serial_putchar(nand->info.raw_data[3]);
		usb_serial_putchar(nand->info.raw_data[4]);
		usb_serial_putchar((nand->info.page_size >> 24) & 0xFF);
		usb_serial_putchar((nand->info.page_size >> 16) & 0xFF);
		usb_serial_putchar((nand->info.page_size >> 8) & 0xFF);
		usb_serial_putchar(nand->info.page_size & 0xFF);
		usb_serial_putchar((nand->info.oob_size >> 8) & 0xFF);
		usb_serial_putchar(nand->info.oob_size & 0xFF);
		usb_serial_putchar(nand->info.bus_width);
		usb_serial_putchar((nand->info.block_size >> 24) & 0xFF);
		usb_serial_putchar((nand->info.block_size >> 16) & 0xFF);
		usb_serial_putchar((nand->info.block_size >> 8) & 0xFF);
		usb_serial_putchar(nand->info.block_size & 0xFF);
		usb_serial_putchar((nand->info.num_blocks >> 24) & 0xFF);
		usb_serial_putchar((nand->info.num_blocks >> 16) & 0xFF);
		usb_serial_putchar((nand->info.num_blocks >> 8) & 0xFF);
		usb_serial_putchar(nand->info.num_blocks & 0xFF);
		usb_serial_putchar(nand->info.num_planes);
		usb_serial_putchar((nand->info.plane_size >> 24) & 0xFF);
		usb_serial_putchar((nand->info.plane_size >> 16) & 0xFF);
		usb_serial_putchar((nand->info.plane_size >> 8) & 0xFF);
		usb_serial_putchar(nand->info.plane_size & 0xFF);
	}
	else {
		usb_serial_putchar(nand->info.raw_data[0]); //maker_code
		usb_serial_putchar(nand->info.raw_data[1]); //device_code
		usb_serial_putchar(nand->info.raw_data[2]);
		usb_serial_putchar(nand->info.raw_data[3]);
		usb_serial_putchar(nand->info.raw_data[4]);
		for (uint8_t i = 0; i < 20; ++i) {
			usb_serial_putchar(0);
		}
	}
}
	
void bootloader() {
	cli();
	// disable watchdog, if enabled
	// disable all peripherals
	UDCON = 1;
	USBCON = (1<<FRZCLK);  // disable USB
	UCSR1B = 0;
	_delay_ms(50);

	EIMSK = 0; PCICR = 0; SPCR = 0; ACSR = 0; EECR = 0; ADCSRA = 0;
	TIMSK0 = 0; TIMSK1 = 0; TIMSK2 = 0; TIMSK3 = 0; UCSR1B = 0; TWCR = 0;
	DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;

	__asm volatile("jmp 0x1FC00");
}

int freeRam() {
	extern int __heap_start, *__brkval;
	int v;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

int main(void) {
	int16_t in_data;
	uint8_t state, cycle, tx_data, tx_wr;
	uint16_t freemem;
	
	// set for 8 MHz clock because of 3.3v regulator
	CPU_PRESCALE(1);
	
	// set for 16 MHz clock
	//CPU_PRESCALE(0);

	//disable JTAG
	MCUCR = (1<<JTD) | (1<<IVCE) | (0<<PUD);
	MCUCR = (1<<JTD) | (0<<IVSEL) | (0<<IVCE) | (0<<PUD);

	// set all i/o lines to input
	releaseports();

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();

	while (!usb_configured()) /* wait */ ;

#if BUILD_VERSION == BUILD_DUAL_NAND
	nand_enable(&nand0);
	nand_enable(&nand1);
#endif

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	state = S_IDLE;
	cycle = tx_data = tx_wr = 0;
	
	while (1) {
		// wait for the user to run client app
		// which sets DTR to indicate it is ready to receive.
		//while (!(usb_serial_get_control() & USB_SERIAL_RTS)) /* wait */ ;

		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();

		//while (usb_configured() && (usb_serial_get_control() & USB_SERIAL_RTS)) { // is user still connected?
		while (usb_configured()) { // is user still connected?
			tx_wr = 0;
			switch (state) {
			case S_IDLE:
				if ((in_data = usb_serial_getchar()) != -1) {
					// command
					if (in_data == 0) {				//8'b00000000: NOP
					}
					else if (in_data == 1) {		//8'b00000001: READSTATE
					}
					else if (in_data == 2) {		//8'b00000010: PING1
						tx_data = VERSION_MAJOR;
						tx_wr = 1;
					}
					else if (in_data == 3) {		//8'b00000011: PING2
						freemem = freeRam();
						usb_serial_putchar(VERSION_MINOR);
						usb_serial_putchar((freemem >> 8) & 0xFF);
						usb_serial_putchar(freemem & 0xFF);
					}
					else if (in_data == 4) {		//8'b00000100: BOOTLOADER
						bootloader();
					}
					else if (in_data == 5) {		//8'b00000101: ADDR_INCR
						
					}
					else if (in_data == 6) {		//8'b0000011z: TRISTATE
						releaseports();
					}
					else if (in_data == 15) {		//8'b0000111z: WAIT
						state = S_WAITING;
					}
					else if (in_data == 12) {		//8'b00001100: READ_ID - NAND0
						usb_serial_putchar('Y');
						handle_read_id(&nand0);
						#if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
							releaseports();
						#endif
					}
					else if (in_data == 13) {		//8'b00001101: READ_PAGE - NAND0
						handle_read_page(&nand0);
						#if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
							releaseports();
						#endif
					}
					else if (in_data == 16) {		//8'b00010zzz: WRITE_PAGE - NAND0
						handle_write_page(&nand0);
						#if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
							releaseports();
						#endif
					}
					else if (in_data == 17) {		//8'b0001100z: ERASE_BLOCK - NAND0
						handle_erase_block(&nand0);
						#if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
							releaseports();
						#endif
					}
					else if (in_data == 22) {		//8'b00001100: READ_ID - NAND1
						#if BUILD_VERSION == BUILD_DUAL_NAND
							usb_serial_putchar('Y');
							handle_read_id(&nand1);
						#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
							usb_serial_putchar('N');
						#endif
					}
					else if (in_data == 23) {		//8'b00001101: READ_PAGE - NAND1
						#if BUILD_VERSION == BUILD_DUAL_NAND
							handle_read_page(&nand1);
						#endif
					}
					else if (in_data == 26) {		//8'b00010zzz: WRITE_PAGE - NAND1
						#if BUILD_VERSION == BUILD_DUAL_NAND
							handle_write_page(&nand1);
						#endif
					}
					else if (in_data == 27) {		//8'b0001100z: ERASE_BLOCK - NAND1
						#if BUILD_VERSION == BUILD_DUAL_NAND
							handle_erase_block(&nand1);
						#endif
					}
					else if ((in_data>>6)==1) {		//8'b01zzzzzz: DELAY
						cycle = (in_data<<2)>>2;
						state = S_DELAY;
					}
					else if ((in_data>>7)==1) {		//8'b1zzzzzzz: ADDR
						//CONT_PORT |= (1<<CONT_CE); //HIGH
						state = S_ADDR2;
					}
				}
				break;

			case S_DELAY:
				if (cycle == 0)
					state = S_IDLE;
				else
					cycle -= 1;
				break;

			case S_ADDR2:
				state = S_IDLE;
				break;

			case S_ADDR3:
				state = S_IDLE;
				break;

			case S_READING:
				state = S_IDLE;
				break;

			case S_WDATA1:
				state = S_IDLE;
				break;

			case S_WDATA2:
				state = S_IDLE;
				break;

			case S_WRITING:
				state = S_IDLE;
				break;

			case S_WRITEWORD: //"single word program mode"
				state = S_IDLE;
				break;

			case S_WRITEWORDUBM: //"single word unlock bypass mode"
				state = S_IDLE;
				break;

			case S_WRITEWBP: //"write buffer programming"
				state = S_IDLE;

				break;

			case S_WAITING:
				state = S_IDLE;
				break;

			default:
				state = S_IDLE;
				break;
			}
				
			if (tx_wr == 1) {
				usb_serial_putchar(tx_data);
			}				
		}		
	}
}
