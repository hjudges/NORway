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
#include "delay_x.h"
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

// Define block/sector size for reading/writing
#define BSS_4		0x01000	//2Kwords = 4KB
#define BSS_8		0x02000	//4Kwords = 8KB
#define BSS_64		0x10000	//32Kwords = 64KB
#define BSS_128		0x20000	//64Kwords = 128KB
#define BSS_WORD	0x00002	//word = 2Bytes


void put_address(uint8_t address3, uint8_t address2, uint8_t address1) {
	//CONT_PORT |= (1<<CONT_CE); //HIGH
	ADDR3_PORT = address3;
	ADDR2_PORT = address2;
	ADDR1_PORT = address1;
	//CONT_PORT &= ~(1<<CONT_CE); //LOW
}

void put_data(uint8_t data2, uint8_t data1) {
	DATA2_PORT = data2;
	DATA1_PORT = data1;
	_delay_ns(100);
	CONT_PORT &= ~(1<<CONT_WE); //LOW
	CONT_PORT |= (1<<CONT_WE); //HIGH
}

uint8_t ADDR1, ADDR2, ADDR3;
void addr_increment(uint8_t lock_address)
{
	if (ADDR1 < 0xFF)
		ADDR1++;
	else {
		ADDR1 = 0;
		if (ADDR2 < 0xFF)
			ADDR2++;
		else {
			ADDR2 = 0;
			if (ADDR3 < 0x7F)
				ADDR3++;
			else
				ADDR3 = 0;
		}
	}

	if (lock_address == 0)
		return;
		
	//CONT_PORT |= (1<<CONT_CE); //HIGH

	ADDR3_PORT = ADDR3;
	ADDR2_PORT = ADDR2;
	ADDR1_PORT = ADDR1;

	//CONT_PORT &= ~(1<<CONT_CE); //LOW
}

uint8_t state_waiting1(uint8_t do_increment)
{
	//wait 200ns for RY/BY to become active
	_delay_ns(200);
	
	uint32_t cnt = 0xFFFFFF; //approx. 17secs
	while (cnt > 0) {
		if (CONT_PIN & (1<<CONT_RYBY)) {
			if (do_increment)
				addr_increment(1);
			return 1;
		}
		cnt--;
	}	
	return 0;
}

uint8_t state_waiting2(uint8_t do_increment)
{
	//wait 200ns for RY/BY to become active
	_delay_ns(200);
	
	while (1) {
		if (CONT_PIN & (1<<CONT_RYBY)) {
			if (do_increment)
				addr_increment(1);
			break;
		}
/*		else {
			//_delay_us(5);
			DATA1_DDR = DATA2_DDR = 0x00; // set for input
			CONT_PORT &= ~(1<<CONT_OE); //LOW
			_delay_ns(100); //better safe than sorry

			uint8_t status;
			status = DATA1_PIN;

			CONT_PORT |= (1<<CONT_OE); //HIGH
			DATA1_DDR = DATA2_DDR = 0xFF;	// set for output

			if (status & 0x20) { //time limits exceeded
				if (do_increment)
					addr_increment(1);
				break;
			}
		} */
	}	
	return S_IDLE;
}

uint8_t state_byte()
{
	return  ((CONT_PIN & (1<<CONT_TRI)) ? 0x20 : 0) | ((CONT_PIN & (1<<CONT_RESET)) ? 0x10 : 0) | ((CONT_PIN & (1<<CONT_RYBY)) ? 0x08 : 0) |
			((CONT_PIN & (1<<CONT_CE)) ? 0x04 : 0) | ((CONT_PIN & (1<<CONT_WE)) ? 0x02 : 0) | ((CONT_PIN & (1<<CONT_OE)) ? 0x01 : 0);
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
	
// pure serial receive takes 120secs for 16MB
void speedtest_receive()
{
	int16_t in_data;
	uint8_t buf_write[BSS_4];
	uint16_t i = 0;
	
	while (i < BSS_4) {	//receive buffer data
		if ((in_data = usb_serial_getchar()) != -1)
			buf_write[i++] = in_data;
		else {
			usb_serial_putchar('T');
			return;
		}
	}
	if (i != BSS_4) {	//if receiving timeout, prepare to send FAIL!
		usb_serial_putchar('R');
		return;
	}
	usb_serial_putchar('K');
}

// pure serial transmit takes 49secs for 16MB
void speedtest_send()
{
	uint8_t buf_read[64], buf_ix;
	uint32_t addr;
	
	addr = buf_ix = 0;
	while (1) {
		buf_read[buf_ix++] = buf_ix;
		buf_read[buf_ix++] = buf_ix;
		if (buf_ix == 64) {
			usb_serial_write(buf_read, buf_ix);
			buf_ix = 0;
		}
		if (addr++ == (BSS_128/2-1))
			break;
	}
}

uint8_t verify(const uint8_t *buffer, uint16_t len)
{
	DATA1_DDR = DATA2_DDR = 0x00; // set for input
	while (len)
	{
		CONT_PORT &= ~(1<<CONT_OE); //LOW
		_delay_ns(100); //better safe than sorry
					
		if ((*buffer++ != DATA2_PIN) || (*buffer++ != DATA1_PIN)) {
			CONT_PORT |= (1<<CONT_OE); //HIGH
			return 1;
		}
		
		CONT_PORT |= (1<<CONT_OE); //HIGH
		addr_increment(1);
		len -= 2;
	}
	DATA1_DDR = DATA2_DDR = 0xFF;	// set for output
	
	return 0;
}

void initports()
{
	DATA1_DDR = DATA2_DDR = 0xFF;	// set for output
	ADDR1_DDR = ADDR2_DDR = ADDR3_DDR = 0xFF; //address ports are always output
	ADDR1_PORT = ADDR2_PORT = ADDR3_PORT = 0;
	
	CONT_DDR = 0xFF; //all control ports are always output

	CONT_DDR &= ~(1<<CONT_RYBY); //except RY/BY# (input)
	CONT_PORT |= (1<<CONT_RYBY); //enable pull up
	
	CONT_PORT &= ~((1<<CONT_TRI) | (1<<CONT_CE)); //LOW
	CONT_PORT |= ((1<<CONT_WE) | (1<<CONT_OE) | (1<<CONT_RESET)); //HIGH

//	ADDR1_DDR = ADDR2_DDR = ADDR3_DDR = DATA1_DDR = DATA2_DDR = CONT_DDR = 0; //all ports are always input
//	ADDR1_PORT = ADDR2_PORT = ADDR3_PORT = DATA1_PORT = DATA2_PORT = CONT_PORT = 0; //disable pull ups for all ports

//	CONT_DDR |= (1<<CONT_TRI);
//	CONT_PORT &= ~(1<<CONT_TRI); //LOW
//	CONT_PORT |= (1<<CONT_TRI); //HIGH
}

void releaseports()
{
	ADDR1_DDR = ADDR2_DDR = ADDR3_DDR = DATA1_DDR = DATA2_DDR = CONT_DDR = 0; //all ports are always input
	ADDR1_PORT = ADDR2_PORT = ADDR3_PORT = DATA1_PORT = DATA2_PORT = CONT_PORT = 0; //disable pull ups for all ports

	//CONT_DDR |= (1<<CONT_TRI);
	//CONT_PORT |= (1<<CONT_TRI); //HIGH
}

int main(void)
{
	int16_t in_data;
	uint16_t addr, i;
	uint32_t bss;
	uint8_t vaddr1, vaddr2, vaddr3;
	uint8_t state, cycle, tx_data, tx_wr, buf_ix, do_increment, do_verify, offset_2nddie;
	uint8_t buf_read[64];
	uint8_t buf_write[BSS_4];

	// set for 8 MHz clock
	CPU_PRESCALE(1);
	// set for 16 MHz clock
	//CPU_PRESCALE(0);

	//disable JTAG
	MCUCR = (1<<JTD) | (1<<IVCE) | (0<<PUD);
	MCUCR = (1<<JTD) | (0<<IVSEL) | (0<<IVCE) | (0<<PUD);

	//set all i/o lines to input
	releaseports();

	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();
	while (!usb_configured()) /* wait */ ;

	//configure all i/o lines (and set tristate=low)
	initports();

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	state = S_IDLE;
	bss = BSS_128;
	cycle = tx_data = do_increment = do_verify = tx_wr = buf_ix = offset_2nddie = 0;
	
	ADDR1 = ADDR2 = ADDR3 = 0;

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
					else if (in_data == 4) {		//8'b00000100: BOOTLOADER
						bootloader();
					}
					else if (in_data == 5) {		//8'b00000101: ADDR_INCR
						addr_increment(1);
					}
					else if ((in_data>>1)==3) {		//8'b0000011z: TRISTATE
						if (in_data & 1)
							initports();
							//CONT_PORT &= ~(1<<CONT_TRI); //LOW
						else
							releaseports();
							//CONT_PORT |= (1<<CONT_TRI); //HIGH
					}
					else if ((in_data>>1)==4) {		//8'b0000100z: RESET
						if (in_data & 1)
							CONT_PORT &= ~(1<<CONT_RESET);
						else
							CONT_PORT |= (1<<CONT_RESET);
					}
					/* else if ((in_data>>1)==6) {		//8'b0000110z: INIT/RELEASE PORTS
						if (in_data & 1)
							initports();
						else
							releaseports();
					} */
					/* else if (in_data == 12) {		//8'b00001101: SPEEDTEST_READ
						speedtest_send();
					}
					else if (in_data == 13) {		//8'b00001100: SPEEDTEST_WRITE
						speedtest_receive();
					} */
					else if ((in_data>>1)==6) {		//8'b0000110z: VERIFY
						do_verify = (in_data & 1);
					}
					else if ((in_data>>1)==7) {		//8'b0000111z: WAIT
						do_increment = (in_data & 1);
						state = S_WAITING;
					}
					else if ((in_data>>3)==2) {		//8'b00010zzz: READ
						if (in_data == 0x10)
							bss = BSS_4;
						else if (in_data == 0x11)
							bss = BSS_8;
						else if (in_data == 0x12)
							bss = BSS_64;
						else if (in_data == 0x13)
							bss = BSS_128;
						else
							bss = BSS_WORD;

						DATA1_DDR = DATA2_DDR = 0x00; // set for input
						state = S_READING;
					}
					else if ((in_data>>1)==12) {	//8'b0001100z: WRITE
						CONT_PORT |= (1<<CONT_OE); //HIGH
						CONT_PORT &= ~((1<<CONT_WE) | (1<<CONT_CE)); //LOW
						do_increment = (in_data & 1);
						state = S_WDATA1;
					}
					else if ((in_data>>1)==0x0D) {	//8'b0001101z: WRITEWORD
						CONT_PORT |= (1<<CONT_OE); //HIGH
						CONT_PORT &= ~(1<<CONT_CE); //LOW
						offset_2nddie = (in_data & 1) ? 0x40 : 0; //A22=HIGH for Samsung K8Q2815
						do_increment = 0;
						state = S_WRITEWORD;
					}
					else if ((in_data>>1)==0x0E) {	//8'b0001110z: WRITEWORDUBM
						CONT_PORT |= (1<<CONT_OE); //HIGH
						CONT_PORT &= ~(1<<CONT_CE); //LOW
						offset_2nddie = (in_data & 1) ? 0x40 : 0; //A22=HIGH for Samsung K8Q2815
						do_increment = 0;
						state = S_WRITEWORDUBM;
					}
					else if ((in_data>>1)==0x0F) {	//8'b0001111z: WRITEWBP
						CONT_PORT |= (1<<CONT_OE); //HIGH
						CONT_PORT &= ~(1<<CONT_CE); //LOW
						offset_2nddie = do_increment = 0;
						state = S_WRITEWBP;
					}
					else if ((in_data>>6)==1) {		//8'b01zzzzzz: DELAY
						cycle = (in_data<<2)>>2;
						state = S_DELAY;
					}
					else if ((in_data>>7)==1) {		//8'b1zzzzzzz: ADDR
						//CONT_PORT |= (1<<CONT_CE); //HIGH
						ADDR3 = ((in_data<<1)>>1);
						ADDR3_PORT = ADDR3;
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
					ADDR2 = in_data;
					ADDR2_PORT = ADDR2;
					state = S_ADDR3;
				}
				break;

			case S_ADDR3:
				if ((in_data = usb_serial_getchar()) != -1) {
					ADDR1 = in_data;
					ADDR1_PORT = ADDR1;
					//CONT_PORT &= ~(1<<CONT_CE); //LOW
					state = S_IDLE;
				}
				break;

			case S_READING:
				if (bss == BSS_WORD) {
					CONT_PORT &= ~(1<<CONT_OE); //LOW
					_delay_ns(100); //better safe than sorry
					usb_serial_putchar(DATA2_PIN);
					usb_serial_putchar(DATA1_PIN);
					CONT_PORT |= (1<<CONT_OE); //HIGH
				}
				else {
					addr = buf_ix = 0;
					while (1) {
						CONT_PORT &= ~(1<<CONT_OE); //LOW
						_delay_ns(100); //better safe than sorry
						buf_read[buf_ix++] = DATA2_PIN;
						buf_read[buf_ix++] = DATA1_PIN;
						CONT_PORT |= (1<<CONT_OE); //HIGH
						addr_increment(1);
						if (buf_ix == 64) {
							usb_serial_write(buf_read, buf_ix);
							buf_ix = 0;
						}
						if (addr++ == (bss/2-1))
							break;
					}
				}
				
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
					_delay_ns(100);
					state = S_WRITING;
				}
				break;

			case S_WRITING:
				CONT_PORT |= (1<<CONT_WE); //HIGH
				if (do_increment)
					addr_increment(1);
				state = S_IDLE;
				break;

			case S_WRITEWORD: //"single word program mode"
				state = S_IDLE;

				for (i = 0; i < BSS_4; i++) {	//receive buffer data
					if ((in_data = usb_serial_getchar()) != -1)
						buf_write[i] = in_data;
					else
						break;
				}
				if (i < BSS_4) {	//if receiving timeout, prepare to send FAIL!
					tx_data = 'R';
					tx_wr = 1;
					break;			//and exit case
				}

				vaddr1 = ADDR1; vaddr2 = ADDR2; vaddr3 = ADDR3;

				for (i = 0; i < BSS_4; i += 2) {
					if ((buf_write[i] == 0xFF) && (buf_write[i+1] == 0xFF)) {
						addr_increment(0);
						continue;
					}

					put_address(offset_2nddie, 0x5, 0x55); put_data(0x0, 0xAA);
					put_address(offset_2nddie, 0x2, 0xAA); put_data(0x0, 0x55);
					put_address(offset_2nddie, 0x5, 0x55); put_data(0x0, 0xA0);
					put_address(ADDR3, ADDR2, ADDR1); put_data(buf_write[i], buf_write[i+1]);

					addr_increment(0);
					if (!state_waiting1(0))
						break;
				}		
				if (i < BSS_4) {	//if ack timeout, prepare to send FAIL!
					tx_data = 'T';
					tx_wr = 1;
					break;			//and exit case
				}

				if (do_verify == 1) {
					ADDR1 = vaddr1; ADDR2 = vaddr2; ADDR3 = vaddr3;
					put_address(vaddr3, vaddr2, vaddr1);
					
					if (verify(buf_write, BSS_4) == 1) {
						tx_data = 'V';
						tx_wr = 1;
						break;			//and exit case
					}
				}

				tx_data = 'K';		//ALL OK! prepare OK response
				tx_wr = 1;
				break;

			case S_WRITEWORDUBM: //"single word unlock bypass mode"
				state = S_IDLE;

				for (i = 0; i < BSS_4; i++) {	//receive buffer data
					if ((in_data = usb_serial_getchar()) != -1)
						buf_write[i] = in_data;
					else
						break;
				}

				if (i < BSS_4) {	//if receiving timeout, prepare to send FAIL!
					tx_data = 'R';
					tx_wr = 1;
					break;			//and exit case
				}
				
				vaddr1 = ADDR1; vaddr2 = ADDR2; vaddr3 = ADDR3;

				// enter unlock bypass mode
				put_address(offset_2nddie, 0x5, 0x55); put_data(0x0, 0xAA);
				put_address(offset_2nddie, 0x2, 0xAA); put_data(0x0, 0x55);
				put_address(offset_2nddie, 0x5, 0x55); put_data(0x0, 0x20);
				put_address(ADDR3, ADDR2, ADDR1);
				
				for (i = 0; i < BSS_4; i += 2) {
					if ((buf_write[i] == 0xFF) && (buf_write[i+1] == 0xFF)) {
						addr_increment(1);
						continue;
					}

					put_data(0x0, 0xA0);
					put_data(buf_write[i], buf_write[i+1]);

					//wait for RY/BY and increment address
					if (!state_waiting1(1))
						break;
				}		

				//exit unlock bypass mode
				put_data(0x0, 0x90);
				put_data(0x0, 0x0);

				if (i < BSS_4) {	//if ack timeout, prepare to send FAIL!
					tx_data = 'T';
					tx_wr = 1;
					break;			//and exit case
				}

				if (do_verify == 1) {
					ADDR1 = vaddr1; ADDR2 = vaddr2; ADDR3 = vaddr3;
					put_address(vaddr3, vaddr2, vaddr1);
					
					if (verify(buf_write, BSS_4) == 1) {
						tx_data = 'V';
						tx_wr = 1;
						break;			//and exit case
					}
				}

				tx_data = 'K';		//ALL OK! prepare OK response
				tx_wr = 1;
				break;

			case S_WRITEWBP: //"write buffer programming"
				state = S_IDLE;

				for (i = 0; i < BSS_4; i++) {	//receive buffer data
					if ((in_data = usb_serial_getchar()) != -1)
						buf_write[i] = in_data;
					else
						break;
				}
				if (i < BSS_4) {	//if receiving timeout, prepare to send FAIL!
					tx_data = 'R';
					tx_wr = 1;
					break;			//and exit case
				}
				
				uint8_t saddr1, saddr2, saddr3, k;
				vaddr1 = ADDR1; vaddr2 = ADDR2; vaddr3 = ADDR3;
				
				for (i = 0; i < BSS_4; i += 64) {
					saddr1 = ADDR1; saddr2 = ADDR2; saddr3 = ADDR3;
					
					// enter write buffer programming mode
					put_address(0, 0x5, 0x55); put_data(0x0, 0xAA);
					put_address(0, 0x2, 0xAA); put_data(0x0, 0x55);
					put_address(saddr3, saddr2, saddr1); put_data(0x0, 0x25);
					put_address(saddr3, saddr2, saddr1); put_data(0x0, 0x1F);

					//put_address(saddr3, saddr2, saddr1);
					for (k = 0; k < 64; k += 2) {
						put_data(buf_write[i+k], buf_write[i+k+1]);
						addr_increment(1);
					}

					put_address(saddr3, saddr2, saddr1); put_data(0x0, 0x29);

					//wait for RY/BY
					if (!state_waiting1(0))
						break;
				}		

				if (i < BSS_4) {	//if ack timeout, prepare to send FAIL!
					tx_data = 'T';
					tx_wr = 1;
					break;			//and exit case
				}
				
				if (do_verify == 1) {
					ADDR1 = vaddr1; ADDR2 = vaddr2; ADDR3 = vaddr3;
					put_address(vaddr3, vaddr2, vaddr1);
					
					if (verify(buf_write, BSS_4) == 1) {
						tx_data = 'V';
						tx_wr = 1;
						break;			//and exit case
					}
				}

				tx_data = 'K';		//ALL OK! prepare OK response
				tx_wr = 1;
				break;

			case S_WAITING:
				state = state_waiting2(do_increment);
				break;

			default:
				break;
			}
				
			if (tx_wr == 1) {
				usb_serial_putchar(tx_data);
			}				
		}		
	}
}
