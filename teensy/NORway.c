/************************************************************************
  norflash.v - NOR flasher for PS3

Copyright (C) 2010-2011  Hector Martin "marcan" <hector@marcansoft.com>

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************
 NORway.c - Teensy++ 2.0 port by judges@eEcho.com
*************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include "usb_serial.h"

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

// Define data ports
#define DATA1_PORT	PORTD
#define DATA1_PIN	PIND
#define DATA1_DDR	DDRD

#define DATA2_PORT	PORTC
#define DATA2_PIN	PINC
#define DATA2_DDR	DDRC

// Define address line ports
#define ADDR1_PORT	PORTF
#define ADDR1_PIN	PINF
#define ADDR1_DDR	DDRF

#define ADDR2_PORT	PORTA
#define ADDR2_PIN	PINA
#define ADDR2_DDR	DDRA

#define ADDR3_PORT	PORTB
#define ADDR3_PIN	PINB
#define ADDR3_DDR	DDRB

// Define control port and pins
#define CONT_PORT	PORTE
#define CONT_DDR	DDRE
#define CONT_PIN	PINE

#define CONT_CE		0		// Chip Enable
#define CONT_OE		1		// Output Enable
#define CONT_RESET	4		// Reset
#define CONT_WE		5		// Write Enable
#define CONT_RYBY	6		// Ready/Busy
#define CONT_TRI	7		// Tristate

// Define state
#define S_IDLE		0
#define S_DELAY		1
#define S_ADDR2		2
#define S_ADDR3		3
#define S_READING	4
#define S_WDATA1	5
#define S_WDATA2	6
#define S_WRITING	7
#define S_WAITING	8

uint8_t drive;
uint8_t state_byte()
{
	return  (drive ? 0x40 : 0) | ((CONT_PIN & (1<<CONT_TRI)) ? 0x20 : 0) | ((CONT_PIN & (1<<CONT_RESET)) ? 0x10 : 0) |
			((CONT_PIN & (1<<CONT_RYBY)) ? 0x08 : 0) | ((CONT_PIN & (1<<CONT_CE)) ? 0x04 : 0) | ((CONT_PIN & (1<<CONT_WE)) ? 0x02 : 0) | ((CONT_PIN & (1<<CONT_OE)) ? 0x01 : 0);
}

void addr_increment()
{
	if (ADDR1_PORT < 0xFF)
		ADDR1_PORT++;
	else {
		ADDR1_PORT = 0;
		if (ADDR2_PORT < 0xFF)
			ADDR2_PORT++;
		else {
			ADDR2_PORT = 0;
			if (ADDR3_PORT < 0x7F)
				ADDR3_PORT++;
			else
				ADDR3_PORT = 0;
		}
	}
}

int main(void)
{
	int16_t in_data;
	uint16_t addr;
	uint8_t state, cycle, tx_data, tx_wr, buf_ix, do_increment;
	uint8_t buf[64];

	// set for 8 MHz clock
	CPU_PRESCALE(1);

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	state = S_IDLE;
	cycle = tx_data = do_increment = tx_wr = buf_ix = drive = 0;

	DATA1_DDR = DATA2_DDR = 0xFF;	// set for output
	ADDR1_DDR = ADDR2_DDR = ADDR3_DDR = 0xFF; //address ports are always output
	ADDR1_PORT = ADDR2_PORT = ADDR3_PORT = 0;
	
	CONT_DDR = 0xFF; //all control ports are always output

	CONT_DDR &= ~(1<<CONT_RYBY); //except RY/BY# (input)
	CONT_PORT |= (1<<CONT_RYBY); //enable pull up
	
	CONT_PORT &= ~((1<<CONT_TRI) | (1<<CONT_CE)); //LOW
	CONT_PORT |= ((1<<CONT_WE) | (1<<CONT_OE) | (1<<CONT_RESET)); //HIGH

	while (1) {
		// wait for the user to run client app
		// which sets DTR to indicate it is ready to receive.
		while (!(usb_serial_get_control() & USB_SERIAL_DTR)) /* wait */ ;

		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();

		while (usb_configured() && (usb_serial_get_control() & USB_SERIAL_DTR)) { // is user still connected?
			tx_wr = 0;
			switch (state) {
			case S_IDLE:
				if ((in_data = usb_serial_getchar()) != -1) {
					// command
					if (in_data == 0) {				//8'b00000000: NOP
					}
					else if (in_data == 1) {		//8'b00000001: READSTATE
						tx_data = state_byte();
						tx_wr = 1;
					}
					else if (in_data == 2) {		//8'b00000010: PING1
						tx_data = 0x42;
						tx_wr = 1;
					}
					else if (in_data == 3) {		//8'b00000011: PING2
						tx_data = 0xbd;
						tx_wr = 1;
					}
					else if ((in_data>>1)==2) {		//8'b0000010z: DRIVE
						drive = (in_data & 1);
					}
					else if ((in_data>>1)==3) {		//8'b0000011z: TRISTATE
						if (in_data & 1)
							CONT_PORT |= (1<<CONT_TRI);
						else
							CONT_PORT &= ~(1<<CONT_TRI);
					}
					else if ((in_data>>1)==4) {		//8'b0000100z: RESET
						if (in_data & 1)
							CONT_PORT &= ~(1<<CONT_RESET);
						else
							CONT_PORT |= (1<<CONT_RESET);
					}
					else if ((in_data>>1)==7) {		//8'b0000111z: WAIT
						do_increment = (in_data & 1);
						state = S_WAITING;
					}
					else if ((in_data>>2)==4) {		//8'b000100zz: READ
						do_increment = (in_data & 1);
						DATA1_DDR = DATA2_DDR = 0x00; // set for input
						CONT_PORT &= ~(1<<CONT_OE);
						state = S_READING;
					}
					else if ((in_data>>1)==12) {	//8'b0001100z: WRITE
						do_increment = (in_data & 1);
						state = S_WDATA1;
					}
					else if ((in_data>>6)==1) {		//8'b01zzzzzz: DELAY
						cycle = (in_data<<2)>>2;
						state = S_DELAY;
					}
					else if ((in_data>>7)==1) {		//8'b1zzzzzzz: ADDR
						ADDR3_PORT = ((in_data<<1)>>1);
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
				if ((in_data = usb_serial_getchar()) != -1) {
					ADDR2_PORT = in_data;
					state = S_ADDR3;
				}
				break;

			case S_ADDR3:
				if ((in_data = usb_serial_getchar()) != -1) {
					ADDR1_PORT = in_data;
					state = S_IDLE;
				}
				break;

			case S_READING: //always read a whole sector (128KB)
				addr = buf_ix = 0;
				while (1) {
					buf[buf_ix++] = DATA2_PIN;
					buf[buf_ix++] = DATA1_PIN;
					addr_increment();
					if (buf_ix == 64) {
						usb_serial_write(buf, buf_ix);
						buf_ix = 0;
					}
					if (addr++ == 0xFFFF)
						break;
				}

				CONT_PORT |= (1<<CONT_OE);
				DATA1_DDR = DATA2_DDR = 0xFF;	// set for output
				state = S_IDLE;
				break;

			case S_WDATA1:
				if ((in_data = usb_serial_getchar()) != -1) {
					DATA2_PORT = in_data;
					state = S_WDATA2;
				}
				break;

			case S_WDATA2:
				if ((in_data = usb_serial_getchar()) != -1) {
					DATA1_PORT = in_data;
					state = S_WRITING;
					CONT_PORT &= ~(1<<CONT_WE);
				}
				break;

			case S_WRITING:
				CONT_PORT |= (1<<CONT_WE);
				if (do_increment)
					addr_increment();
				state = S_IDLE;
				break;

			case S_WAITING:
				if ((CONT_PIN & (1<<CONT_RYBY)) != 0) {
					if (do_increment)
						addr_increment();
					state = S_IDLE;
				}
				break;

			default:
				break;
			}
				
			if (tx_wr == 1)
				usb_serial_putchar(tx_data);
		}		
	}
}
