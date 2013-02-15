/************************************************************************
  norflash.v - NOR flasher for PS3

Copyright (C) 2010-2011  Hector Martin "marcan" <hector@marcansoft.com>

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************
 NORway.c (v0.6 beta) - Teensy++ 2.0 port by judges@eEcho.com
*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "usb_serial.h"

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

#define NAND_STATUS_FAIL 			(1<<0) /* HIGH - FAIL,  LOW - PASS */
#define NAND_STATUS_IDLE 			(1<<5) /* HIGH - IDLE,  LOW - ACTIVE */
#define NAND_STATUS_READY 			(1<<6) /* HIGH - READY, LOW - BUSY */
#define NAND_STATUS_NOT_PROTECTED 	(1<<7) /* HIGH - NOT,   LOW - PROTECTED */

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

/* #define RYBY_CONNECTED */

typedef struct _nand_port {

	volatile uint8_t *io_port;
	volatile uint8_t *io_pin;
	volatile uint8_t *io_ddr;
	
	volatile uint8_t *cont_port;
	volatile uint8_t *cont_pin;
	volatile uint8_t *cont_ddr;

} nand_port;

nand_port nand0 = { &NAND0_IO_PORT, &NAND0_IO_PIN, &NAND0_IO_DDR, &NAND0_CONT_PORT, &NAND0_CONT_PIN, &NAND0_CONT_DDR };
nand_port nand1 = { &NAND1_IO_PORT, &NAND1_IO_PIN, &NAND1_IO_DDR, &NAND1_CONT_PORT, &NAND1_CONT_PIN, &NAND1_CONT_DDR };


#define NAND_CONT_CEal		(1<<0)		// Chip Enable
#define NAND_CONT_REal		(1<<1)		// Read Enable
#define NAND_CONT_CLE		(1<<2)		// Command Latch Enable
#define NAND_CONT_ALE		(1<<3)		// Address Latch Enable
#define NAND_CONT_WPal		(1<<4)		// Write Protect
#define NAND_CONT_WEal		(1<<5)		// Write Enable
#define NAND_CONT_RYBY		(1<<6)		// Ready/Busy (ready - high, busy - low)
#define NAND_CONT_TRI		(1<<7)		// Tristate (ready - high, busy - low)

#define NAND_TOGGLE_WE(_nand_) 		*((_nand_)->cont_port) &= ~(NAND_CONT_WEal);	*((_nand_)->cont_port) |= NAND_CONT_WEal
#define NAND_IO_INPUT(_nand_) 		*((_nand_)->io_ddr) = 0x00
#define NAND_IO_OUTPUT(_nand_) 		*((_nand_)->io_ddr) = 0xFF
#define NAND_IO_SET(_nand_, _cmd_) 	*((_nand_)->io_port)=(_cmd_); NAND_TOGGLE_WE(_nand_)
#define NAND_IO_READ(_nand_, _data_) *((_nand_)->cont_port) &= ~(NAND_CONT_REal);   _delay_us(0.3);  (_data_) = *((_nand_)->io_pin); *((_nand_)->cont_port) |= NAND_CONT_REal; _delay_us(0.1)
#define NAND_COMMAND(_nand_, _cmd_) *((_nand_)->cont_port) |= NAND_CONT_CLE; NAND_IO_SET(_nand_, _cmd_); *((_nand_)->cont_port) &= ~(NAND_CONT_CLE)
#define NAND_BUSY_WAIT(_nand_, _us_) while (1) { uint8_t status; _delay_us(_us_); 	status = nand_status(nandp);	if (status & NAND_STATUS_READY) {	break;	} }


void
nand_initports(nand_port *nandp)
{
	*(nandp->cont_ddr) = 0xFF; 			// all control ports - output
	*(nandp->cont_ddr) &= ~NAND_CONT_RYBY; /* ready / busy - input */
	*(nandp->cont_port) |= NAND_CONT_RYBY; /* ready / busy - enable pull up */

	*(nandp->cont_port) &= ~NAND_CONT_TRI; /* tristate - low */

	NAND_IO_OUTPUT(nandp); // io set as output
}

void
nand_releaseports(nand_port *nandp)
{
	*(nandp->cont_ddr) = 0; // all control ports - input
	NAND_IO_INPUT(nandp); // io set as input

	/* disable pull up */
	*(nandp->cont_port) = 0;
	*(nandp->io_port) = 0;
}

void initports()
{
	nand_initports(&nand0);
	nand_initports(&nand1);
}

void releaseports()
{
	nand_releaseports(&nand0);
	nand_releaseports(&nand1);
}


void nand_enable(nand_port *nandp)
{
	*(nandp->cont_port) = 	NAND_CONT_WEal | 
							NAND_CONT_REal |
							NAND_CONT_RYBY; /* input - pull up */

	NAND_IO_OUTPUT(nandp);
}

void reset(uint8_t set)
{
}

uint8_t 	device_id[5];

uint16_t 	NBLOCKS = 0;
uint16_t 	PAGE_SZ = 0;
uint32_t 	BLOCK_SZ = 0; /* 0=64KB, 1=128KB, 2=256KB */
uint16_t 	PAGE_PLUS_RAS_SZ = 0; /* page size + Redundent Area Size */
uint16_t	PAGES_PER_BLOCK = 0;



uint8_t *
nand_read_id(nand_port *nandp)
{
	nand_enable(nandp);

	NAND_COMMAND(nandp, 0x90);

	/* follow by address - 0 */
	*(nandp->cont_port) |= NAND_CONT_ALE;
	NAND_IO_SET(nandp, 0x00);
	*(nandp->cont_port) &= ~NAND_CONT_ALE;

	NAND_IO_INPUT(nandp);
	NAND_IO_READ(nandp, device_id[0]);
	NAND_IO_READ(nandp, device_id[1]);

	/* hynix */
	if (device_id[0] == 0xAD && device_id[1] == 0x73) {
		/* hynix flash device */
		PAGE_SZ = 512;
		NBLOCKS = 1024;
		PAGES_PER_BLOCK = 32;
		PAGE_PLUS_RAS_SZ = PAGE_SZ+16; 
		BLOCK_SZ = PAGES_PER_BLOCK * PAGE_SZ;
		return device_id;	
	}

	/* samsung */
	if (device_id[0] == 0xEC) {
		NAND_IO_READ(nandp, device_id[2]);
		NAND_IO_READ(nandp, device_id[3]);
		
		NBLOCKS = 1024;
		PAGE_SZ = (device_id[3] & 0x1) ? 2048 : 1024; /* !! 2048 */
		switch ( ((device_id[3] >> 4) & 0x3) ) {
		case 0:
			BLOCK_SZ = (uint32_t)64 * (uint32_t)1024;
			break;
		case 1:
			BLOCK_SZ = (uint32_t)128 * (uint32_t)1024; /* should be 128 !! */
			break;
		case 2:
			BLOCK_SZ = (uint32_t)256 * (uint32_t)1024;
			break;
		}
		
	//	PAGE_PLUS_RAS_SZ 	 = PAGE_SZ + ((device_id[3] & 4) ? (16 * (PAGE_SZ/512)) : (8 * (PAGE_SZ/512)));
		PAGE_PLUS_RAS_SZ = 2112;

		PAGES_PER_BLOCK = BLOCK_SZ / PAGE_SZ; /* !! 64 */
		return device_id;
	}

	return device_id;
}

uint8_t 
nand_read_page(nand_port *nandp, const uint8_t *addr, const uint8_t naddr, uint8_t *data)
{
	uint16_t i;
	uint8_t sreg;
	
	nand_enable(nandp);
	
	/* read command */
	NAND_COMMAND(nandp, 0x0);

	/* address */
	*(nandp->cont_port) |= NAND_CONT_ALE;
	for (i=0;i<naddr;i++) {
		NAND_IO_SET(nandp, addr[i]); 
	}
	*(nandp->cont_port) &= ~NAND_CONT_ALE;

	/* read command - state 2 - (samsung) */
	if (device_id[0] == 0xEC) {
		NAND_COMMAND(nandp, 0x30);
	}
	
	NAND_IO_INPUT(nandp);
	
	/* Save global interrupt flag and disable interrupts */
	sreg = SREG;
	cli();
	
	/* wait for the nand to read this page to the iternal page register */
#if !defined(RYBY_CONNECTED)
	_delay_us(200.0);
#else
	/* TODO - do not wait forever - (should take less than 25us) */
	while (1) {
		if ( *(nandp->cont_pin) & NAND_CONT_RYBY ) {
			break;
		}
	}
#endif

	/* read the entire page from the nand */
	for (i=0;i<PAGE_PLUS_RAS_SZ;i++) {
		NAND_IO_READ(nandp, data[i]);
	}
		
	/* Restore global interrupt flag */
	SREG = sreg;

	return 1;
}

uint8_t 
nand_status(nand_port *nandp)
{
	uint8_t status;
	
	NAND_COMMAND(nandp, 0x70);

	NAND_IO_INPUT(nandp);
	
	NAND_IO_READ(nandp, status);

	NAND_IO_OUTPUT(nandp);

	return status;
}

uint8_t 
nand_write_page(nand_port *nandp, const uint8_t *addr, const uint8_t naddr, uint8_t *data)
{
	uint16_t i;
	
	nand_enable(nandp);

	/* Write protection - high (program/erase enabled) */
	*(nandp->cont_port) |= NAND_CONT_WPal;
	_delay_us(1.0);
	
	/* Serial Data Input command */
	/* CLE - high & CE# - low */
	NAND_COMMAND(nandp, 0x80);

	/* address */
	/* ALE on & CLE off */ 
	*(nandp->cont_port) |= NAND_CONT_ALE;
	for (i=0;i<naddr;i++) { /* Hynix - naddr=3 */
		NAND_IO_SET(nandp, addr[i]); 
	}
	*(nandp->cont_port) &= ~(NAND_CONT_ALE);
	
	/* data */
	for (i=0;i<PAGE_PLUS_RAS_SZ;i++) {
		NAND_IO_SET(nandp, data[i]);
	}

	/* Page Program confirm command */
	NAND_COMMAND(nandp, 0x10);

	/* wait for the internal controller to finish the program command 
		TBD - up to 200us */
	NAND_BUSY_WAIT(nandp, 5.0);

	/* Write protection - low (protected) */
	*(nandp->cont_port) &= ~(NAND_CONT_WPal);

	return !(nand_status(nandp) & NAND_STATUS_FAIL);
}


int8_t 
nand_erase_block(nand_port *nandp, const uint8_t *rowaddr, const uint8_t nrowaddr)
{
	uint8_t i;

	nand_enable(nandp);

	/* Write protection - high (program/erase enabled) */
	*(nandp->cont_port) |= NAND_CONT_WPal;
	_delay_us(1.0);

	/* block erase setup command */
	NAND_COMMAND(nandp, 0x60);

	/* block address */
	/* ALE on & CLE off */ 
	*(nandp->cont_port) |= NAND_CONT_ALE;
	for (i=0;i<nrowaddr;i++) {
		NAND_IO_SET(nandp, rowaddr[i]); 
	}
	*(nandp->cont_port) &= ~NAND_CONT_ALE;
	
	/* block erase confirm command */
	NAND_COMMAND(nandp, 0xD0);

	if (!(nand_status(nandp) & NAND_STATUS_NOT_PROTECTED)) {
		/* Write protection - low (protected) */
		*(nandp->cont_port) &= ~(NAND_CONT_WPal);
		return -1;
	}
		
	/* wait for the internal controller to finish the erase command 
		TBD - up to 2ms */
	NAND_BUSY_WAIT(nandp, 20.0);

	/* Write protection - low (protected) */
	*(nandp->cont_port) &= ~(NAND_CONT_WPal);

	return !(nand_status(nandp) & NAND_STATUS_FAIL);
}



void bootloader()
{
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

uint8_t buf_read[2048+100];
uint8_t addr_buf[16];

void handle_read_page(nand_port *nand)
{
	uint16_t i;
	int16_t in_data;
	uint8_t naddr = 4;
	uint8_t *addr_buf_p = addr_buf;
	for (i = 0; i < naddr; i++) {
		if ((in_data = usb_serial_getchar()) != -1) {
			addr_buf[i] = in_data;
		}
		else {
			break;
		}
	}
	if (i < naddr) {	// timeout
		usb_serial_putchar('R');
		return;		// and exit
	}
	if (device_id[0] == 0xAD) {
		naddr = 3; // Hynix
		addr_buf_p = &addr_buf[1];
	}
	if (nand_read_page(nand, addr_buf_p, naddr, buf_read)) {
		usb_serial_putchar('K');
		usb_serial_write(buf_read, PAGE_PLUS_RAS_SZ);
	}
	else {
		usb_serial_putchar('R');
		return;
	}
}


void handle_write_page(nand_port *nand)
{
	uint16_t i;
	int16_t in_data;
	uint8_t naddr = 4;
	uint8_t *addr_buf_p = addr_buf;

	for (i = 0; i < naddr; i++) {
		if ((in_data = usb_serial_getchar()) != -1) {
			addr_buf[i] = in_data;
		}
		else {
			break;
		}
	}
	if (i < naddr) {	// timeout
		usb_serial_putchar('R');
		return;		// and exit
	}

		
	for (i = 0; i < PAGE_PLUS_RAS_SZ; i++) {
		if ((in_data = usb_serial_getchar()) != -1) {
			buf_read[i] = in_data;
		}
		else {
			break;
		}
	}
	
	if (i < PAGE_PLUS_RAS_SZ) { // timeout
		usb_serial_putchar('R');
		return;		// and exit
	}


	if (device_id[0] == 0xAD) {
		naddr = 3; // Hynix
		addr_buf_p = &addr_buf[1];
	}
	if (nand_write_page(nand, addr_buf_p, naddr, buf_read)) {
		usb_serial_putchar('K');
	}
	else {
		usb_serial_putchar('V');
	}

}

void handle_erase_block(nand_port *nand)
{
	uint16_t i;
	int8_t res;
	int16_t in_data;
	uint8_t naddr = 2; /* just row address */
	for (i = 0; i < naddr; i++) {
		if ((in_data = usb_serial_getchar()) != -1) {
			addr_buf[i] = in_data;
		}
		else {
			break;
		}
	}
	
	if (i < naddr) {	// if receiving timeout, send FAIL!
		usb_serial_putchar('R');
		return;
	}
	res = nand_erase_block(nand, addr_buf, naddr);
	if (res > 0) {
		usb_serial_putchar('K');
	}
	else if (res < 0) {
		usb_serial_putchar('P');
	}
	else {
		usb_serial_putchar('V');
	}
}

int main(void)
{
	int16_t in_data;
	uint16_t i;
	uint8_t state, cycle, tx_data, tx_wr;

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

	//configure all i/o lines
	initports();

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
						tx_data = 0x43;
						tx_wr = 1;
					}
					else if (in_data == 3) {		//8'b00000011: PING2
						tx_data = 0xbe;
						tx_wr = 1;
					}
					else if (in_data == 4) {		//8'b00000100: BOOTLOADER
						bootloader();
					}
					else if (in_data == 5) {		//8'b00000101: ADDR_INCR
						
					}
					else if ((in_data>>1)==3) {		//8'b0000011z: TRISTATE
						if (in_data & 1)
							initports();
						else
							releaseports();
					}
					else if (in_data == 15) {		//8'b0000111z: WAIT
						state = S_WAITING;
					}
					else if (in_data == 12) {		//8'b00001100: READ_ID - NAND0
						nand_read_id(&nand0);
						usb_serial_write(device_id, sizeof(device_id) / sizeof(device_id[0]));
					}
					else if (in_data == 13) {		//8'b00001101: READ_PAGE - NAND0
						handle_read_page(&nand0);
					}
					else if (in_data == 16) {		//8'b00010zzz: WRITE_PAGE - NAND0
						handle_write_page(&nand0);
					}
					else if (in_data == 17) {		//8'b0001100z: ERASE_BLOCK - NAND0
						handle_erase_block(&nand0);
					}
					else if (in_data == 22) {		//8'b00001100: READ_ID - NAND1
						nand_read_id(&nand1);
						usb_serial_write(device_id, sizeof(device_id) / sizeof(device_id[0]));
					}
					else if (in_data == 23) {		//8'b00001101: READ_PAGE - NAND1
						handle_read_page(&nand1);
					}
					else if (in_data == 26) {		//8'b00010zzz: WRITE_PAGE - NAND1
						handle_write_page(&nand1);
					}
					else if (in_data == 27) {		//8'b0001100z: ERASE_BLOCK - NAND1
						handle_erase_block(&nand1);
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
