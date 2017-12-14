/************************************************************************
SPIway.c (v0.40) - Teensy++ 2.0 SPI Flasher for PS4

Copyright (C) 2017	judges <judges@eEcho.com>

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include "delay_x.h"
#include "usb_serial.h"
#include "SPI_AVR8.h"

#define VERSION_MAJOR			0
#define VERSION_MINOR			40

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))

// Define commands
enum {
	CMD_PING1 = 0,
	CMD_PING2,
	CMD_BOOTLOADER,
	CMD_IO_LOCK,
	CMD_IO_RELEASE,
	CMD_PULLUPS_DISABLE,
	CMD_PULLUPS_ENABLE,
	CMD_SPI_ID,
	CMD_SPI_READBLOCK,
	CMD_SPI_WRITESECTOR,
	CMD_SPI_ERASEBLOCK,
	CMD_SPI_ERASECHIP,
	CMD_SPI_3BYTE_ADDRESS,
	CMD_SPI_4BYTE_ADDRESS,
	CMD_SPI_3BYTE_CMDS,
	CMD_SPI_4BYTE_CMDS,
} cmd_t;

//SPI commands (69)
#define SPI_COMMAND_3B_READ				0x03	//
#define SPI_COMMAND_3B_FASTREAD			0x0B	//
#define SPI_COMMAND_3B_2READ			0xBB	//
#define SPI_COMMAND_3B_DREAD			0x3B	//
#define SPI_COMMAND_3B_4READ_BOTTOM		0xEB	//
#define SPI_COMMAND_3B_4READ_TOP		0xEA	//
#define SPI_COMMAND_3B_QREAD			0x6B	//
#define SPI_COMMAND_3B_PAGEPROG			0x02	//
#define SPI_COMMAND_3B_4PAGEPROG		0x38	//
#define SPI_COMMAND_3B_SECTOR_ERASE		0x20	//
#define SPI_COMMAND_3B_BLOCK_ERASE_32K	0x52	//
#define SPI_COMMAND_3B_BLOCK_ERASE_64K	0xD8	//

#define SPI_COMMAND_4B_READ				0x13	//
#define SPI_COMMAND_4B_FASTREAD			0x0C	//
#define SPI_COMMAND_4B_2READ			0xBC	//
#define SPI_COMMAND_4B_DREAD			0x3C	//
#define SPI_COMMAND_4B_4READ			0xEC	//
#define SPI_COMMAND_4B_QREAD			0x6C	//
#define SPI_COMMAND_4B_PAGEPROG			0x12	//
#define SPI_COMMAND_4B_4PAGEPROG		0x3E	//
#define SPI_COMMAND_4B_SECTOR_ERASE		0x21	//
#define SPI_COMMAND_4B_BLOCK_ERASE_32K	0x5C	//
#define SPI_COMMAND_4B_BLOCK_ERASE_64K	0xDC	//

#define SPI_COMMAND_CHIP_ERASE			0x60	//
#define SPI_COMMAND_CHIP_ERASE2			0xC7	//
#define SPI_COMMAND_WREN				0x06	//
#define SPI_COMMAND_WRDI				0x04	//
#define SPI_COMMAND_RDSR				0x05	//
#define SPI_COMMAND_RDCR				0x15	//
#define SPI_COMMAND_WRSR				0x01	//
#define SPI_COMMAND_RDEAR				0xC8	//
#define SPI_COMMAND_WREAR				0xC5	//
#define SPI_COMMAND_WPSEL				0x68	//
#define SPI_COMMAND_EQIO				0x35	//
#define SPI_COMMAND_RSTQIO				0xF5	//
#define SPI_COMMAND_EN4B				0xB7	//
#define SPI_COMMAND_EX4B				0xE9	//
#define SPI_COMMAND_PGM_ERS_SUSPEND		0xB0	//
#define SPI_COMMAND_PGM_ERS_RESUME		0x30	//
#define SPI_COMMAND_DP					0xB9	//
#define SPI_COMMAND_RDP					0xAB	//
#define SPI_COMMAND_SBL					0xC0	//
#define SPI_COMMAND_RDFBR				0x16	//
#define SPI_COMMAND_WRFBR				0x17	//
#define SPI_COMMAND_ESFBR				0x18	//
#define SPI_COMMAND_RDID				0x9F	//
#define SPI_COMMAND_RES					0xAB	//
#define SPI_COMMAND_REMS				0x90	//
#define SPI_COMMAND_QPIID				0xAF	//
#define SPI_COMMAND_RDSFDP				0x5A	//
#define SPI_COMMAND_ENSO				0xB1	//
#define SPI_COMMAND_EXSO				0xC1	//
#define SPI_COMMAND_RDSCUR				0x2B	//
#define SPI_COMMAND_WRSCUR				0x2F	//
#define SPI_COMMAND_GBLK				0x7E	//
#define SPI_COMMAND_GBULK				0x98	//
#define SPI_COMMAND_WRLR				0x2C	//
#define SPI_COMMAND_RDLR				0x2D	//
#define SPI_COMMAND_WRPASS				0x28	//
#define SPI_COMMAND_RDPASS				0x27	//
#define SPI_COMMAND_PASSULK				0x29	//
#define SPI_COMMAND_WRSPB				0xE3	//
#define SPI_COMMAND_ESSPB				0xE4	//
#define SPI_COMMAND_RDSPB				0xE2	//
#define SPI_COMMAND_SPBLK				0xA6	//
#define SPI_COMMAND_RDSPBLK				0xA7	//
#define SPI_COMMAND_WRDPB				0xE1	//
#define SPI_COMMAND_RDDPB				0xE0	//
#define SPI_COMMAND_NOP					0x00	//
#define SPI_COMMAND_RSTEN				0x66	//
#define SPI_COMMAND_RST					0x99	//

#define SPI_STATUS_WIP 				0b00000001 // write in progress bit set
#define SPI_STATUS_WEL	 			0b00000010 // write enable bit set
#define SPI_STATUS_BP0	 			0b00000100
#define SPI_STATUS_BP1			 	0b00001000
#define SPI_STATUS_BP2			 	0b00010000
#define SPI_STATUS_BP3			 	0b00100000
#define SPI_STATUS_QE			 	0b01000000 // quad enable bit set
#define SPI_STATUS_SRWD			 	0b10000000 // status register write disable set

#define SPI_SECURITY_OTP			0b00000001 // factory lock bit set
#define SPI_SECURITY_LDSO 			0b00000010 // lock-down bit set (cannot program/erase otp)
#define SPI_SECURITY_PSB 			0b00000100 // program suspended bit set
#define SPI_SECURITY_ESB		 	0b00001000 // erase suspended bit set
#define SPI_SECURITY_RESERVED	 	0b00010000
#define SPI_SECURITY_P_FAIL		 	0b00100000 // program operation failed bit set
#define SPI_SECURITY_E_FAIL		 	0b01000000 // erase operation failed bit set
#define SPI_SECURITY_WPSEL		 	0b10000000 // status register write disable set

#define BUF_SIZE_RW		4096
#define BUF_SIZE_ADDR	4

uint32_t 	SPI_BLOCK_SIZE = 0x10000; //64KB block size
uint8_t		SPI_ADDRESS_LENGTH = 4;
uint8_t		SPI_USE_3B_CMDS = 0;
uint8_t		IO_PULLUPS = 0xFF;
uint8_t		buf_rw[BUF_SIZE_RW];
uint8_t		buf_addr[BUF_SIZE_ADDR];

#define SPI_IO_PORT		PORTF
#define SPI_IO_PIN		PINF
#define SPI_IO_DDR		DDRF
#define SPI_IO_0		0b00000001		// 0: SIO0
#define SPI_IO_1		0b00000010		// 1: SIO1
#define SPI_IO_2		0b00000100		// 2: SIO2
#define SPI_IO_3		0b00001000		// 3: SIO3
#define SPI_IO_MASK		0b00001111		//

#define SPI_IO_SI		0b00000001		// 0: SI
#define SPI_IO_SO		0b00000010		// 1: SO
#define SPI_IO_WP		0b00000100		// 2: WP
#define SPI_IO_SO_WP_IO3	0b00001110		// 2: WP

#define SPI_CONT_PORT	PORTC
#define SPI_CONT_PIN	PINC
#define SPI_CONT_DDR	DDRC
#define SPI_CONT_CS		0b00000001		// 0: Chip Select
#define SPI_CONT_SCLK	0b00000010		// 1: Clock Input
#define SPI_CONT_RESET	0b00000100		// 2: Reset

#define HWSPI_PORT		PORTB
#define HWSPI_PIN		PINB
#define HWSPI_DDR		DDRB
#define HWSPI_IO_CS		0b00000001		// 0: CS# (Chip select)
#define HWSPI_IO_SCLK	0b00000010		// 1: SCLK (Clock)
#define HWSPI_IO_MOSI	0b00000100		// 2: SI (Master out / Slave in)
#define HWSPI_IO_MISO	0b00001000		// 3: SO (Master in / Slave out)
#define HWSPI_IO_WP		0b00010000		// 4: WP#/SIO2
#define HWSPI_IO_SIO3	0b00100000		// 5: SIO3
#define HWSPI_IO_RESET	0b01000000		// 6: RESET#

#define HWSPI_CS_HIGH		HWSPI_PORT |= HWSPI_IO_CS
#define HWSPI_CS_LOW		HWSPI_PORT &= ~(HWSPI_IO_CS)
#define HWSPI_RESET_HIGH	HWSPI_PORT |= HWSPI_IO_RESET
#define HWSPI_RESET_LOW		HWSPI_PORT &= ~(HWSPI_IO_RESET)
#define HWSPI_WP_HIGH		HWSPI_PORT |= HWSPI_IO_WP
#define HWSPI_WP_LOW		HWSPI_PORT &= ~(HWSPI_IO_WP)
#define HWSPI_SIO3_HIGH		HWSPI_PORT |= HWSPI_IO_SIO3
#define HWSPI_SIO3_LOW		HWSPI_PORT &= ~(HWSPI_IO_SIO3)



#define SPI_CS_HIGH			SPI_CONT_PORT |= SPI_CONT_CS
#define SPI_CS_LOW			SPI_CONT_PORT &= ~(SPI_CONT_CS)
#define SPI_TOGGLE_SCLK		SPI_CONT_PORT |= SPI_CONT_SCLK; SPI_CONT_PORT &= ~(SPI_CONT_SCLK)

#define SPI_IO_READ(_data_)	SPI_TOGGLE_SCLK; (_data_) = ((SPI_IO_PIN & SPI_IO_SO)<<6); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<5); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<4); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<3); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<2); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)<<1); SPI_TOGGLE_SCLK; (_data_) |= (SPI_IO_PIN & SPI_IO_SO); SPI_TOGGLE_SCLK; (_data_) |= ((SPI_IO_PIN & SPI_IO_SO)>>1)
//#define SPI_IO_SET(_data_)	SPI_IO_PORT = (((_data_)>>7) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>6) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>5) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>4) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>3) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>2) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>1) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>0) & ~0xFE) | SPI_IO_3; SPI_TOGGLE_SCLK
#define SPI_IO_SET(_data_)	SPI_IO_PORT = (((_data_)>>7) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>6) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>5) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>4) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>3) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>2) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>1) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK; SPI_IO_PORT = (((_data_)>>0) & ~0xFE) | SPI_IO_SO_WP_IO3; SPI_TOGGLE_SCLK
#define SPI_COMMAND(_cmd_)	SPI_IO_SET(_cmd_)
#define QPI_IO_READ(_data_)	(_data_) = 0; SPI_TOGGLE_SCLK; (_data_) = (SPI_IO_PIN & SPI_IO_MASK)<<4; SPI_TOGGLE_SCLK; (_data_) |= (SPI_IO_PIN & SPI_IO_MASK)
#define QPI_IO_SET(_data_)	SPI_IO_PORT = (_data_)>>4; SPI_TOGGLE_SCLK; SPI_IO_PORT = (_data_); SPI_TOGGLE_SCLK
#define QPI_COMMAND(_cmd_)	QPI_IO_SET(_cmd_)

#define SPI_IO_INPUT		SPI_IO_PORT |= SPI_IO_SO //; SPI_IO_DDR = 0x00; SPI_IO_PORT = IO_PULLUPS //0=disable, 0xFF=enable
#define SPI_IO_OUTPUT		SPI_IO_DDR = 0xFF
//#define SPI_BUSY_WAIT		while (1) { if (!(spi_status() & SPI_STATUS_WIP)) { break; } }
#define SPI_BUSY_WAIT		HWSPI_CS_LOW; SPI_SendByte(SPI_COMMAND_RDSR); while (SPI_ReceiveByte() & SPI_STATUS_WIP); HWSPI_CS_HIGH
#define SPI_WREN			while ((spi_status() & SPI_STATUS_WEL) == 0) { cli(); SPI_CS_LOW; SPI_COMMAND(SPI_COMMAND_WREN); SPI_CS_HIGH; sei(); }
//#define HWSPI_WREN			while ((hwspi_status() & SPI_STATUS_WEL) == 0) { cli(); HWSPI_CS_LOW; SPI_SendByte(SPI_COMMAND_WREN); HWSPI_CS_HIGH; sei(); }
#define HWSPI_WREN			do { HWSPI_CS_LOW; SPI_SendByte(SPI_COMMAND_WREN); HWSPI_CS_HIGH; } while (!(hwspi_status() & SPI_STATUS_WEL))

void hwspi_init()
{
	HWSPI_DDR |= (HWSPI_IO_SIO3 | HWSPI_IO_RESET | HWSPI_IO_WP);
	
	HWSPI_RESET_HIGH;
	HWSPI_WP_LOW;
	HWSPI_SIO3_HIGH;
}

void spi_enable()
{
	
	SPI_CONT_DDR = 0xFF; 			// all control ports - output
	//SPI_CONT_DDR &= ~SPI_CONT_SCLK;
	SPI_CONT_PORT = 0; //low
	SPI_CS_HIGH;
	SPI_CONT_PORT |= SPI_CONT_RESET; //high
	
	SPI_IO_DDR = 0xFF;
	SPI_IO_DDR &= ~(SPI_IO_SO) ;
	SPI_IO_PORT = 0;
	//SPI_IO_PORT |= (SPI_IO_3 | SPI_IO_WP);
	//SPI_IO_PORT |= (SPI_IO_3);
	SPI_IO_PORT |= SPI_IO_SO_WP_IO3;
}

void releaseports()
{
	DDRA = 0; DDRB = 0; DDRC = 0; DDRD = 0; DDRE = 0; DDRF = 0;
	PORTA = 0; PORTB = 0; PORTC = 0; PORTD = 0; PORTE = 0; PORTF = 0;
}

uint8_t wait_ryby()
{
	/* Should be done within 3 milliseconds for all commands. */
	volatile uint32_t timeout = 0x200000; //approx. 3secs

	while (timeout > 0) {
		if ( SPI_CONT_PIN & SPI_CONT_CS ) {
			return 1;
		}
		--timeout;
	}

	return 0;
}

void spi_reset()
{
	spi_enable();
	SPI_COMMAND(SPI_COMMAND_RSTEN);
	SPI_COMMAND(SPI_COMMAND_RST);
	wait_ryby();
}

uint8_t maker_code;
uint8_t device_code0;
//uint8_t device_code1;
uint8_t hwspi_read_id()
{
	HWSPI_CS_LOW;

	SPI_SendByte(SPI_COMMAND_REMS);
	SPI_SendByte(0);
	SPI_SendByte(0);
	SPI_SendByte(0);
	
	//maker_code = 0xC2;
	//device_code0 = 0x18;
	maker_code = SPI_ReceiveByte();
	device_code0 = SPI_ReceiveByte();

	HWSPI_CS_HIGH;

	return 1;
}

uint8_t spi_read_id()
{
	return hwspi_read_id();
	//spi_enable();
	
	//SPI_CS_LOW;
	//SPI_COMMAND(SPI_COMMAND_EX4B);
	//SPI_CS_HIGH;

	SPI_CS_LOW;
	SPI_COMMAND(SPI_COMMAND_REMS);
	SPI_IO_SET(0);
	SPI_IO_SET(0);
	SPI_IO_SET(0);

	//SPI_IO_INPUT;
	SPI_IO_READ(maker_code);
	SPI_IO_READ(device_code0);
	//SPI_IO_READ(device_code1);
	SPI_CS_HIGH;
	
	//maker_code = 0xC2;
	//device_code = 0x18;
	
	return 1;
}

uint8_t hwspi_read_block() {
	uint16_t i;
	uint8_t sreg;
	
	if (SPI_USE_3B_CMDS && SPI_ADDRESS_LENGTH == 4) {
		HWSPI_CS_LOW;
		SPI_SendByte(SPI_COMMAND_EN4B);
		HWSPI_CS_HIGH;
	}

	/* Save global interrupt flag and disable interrupts */
	sreg = SREG;
	cli();

	HWSPI_CS_LOW;

	if (SPI_USE_3B_CMDS || SPI_ADDRESS_LENGTH == 3)
		SPI_SendByte(SPI_COMMAND_3B_READ);
	else
		SPI_SendByte(SPI_COMMAND_4B_READ);
	
	/* address */
	if (SPI_ADDRESS_LENGTH == 4) SPI_SendByte(buf_addr[0]);
	SPI_SendByte(buf_addr[1]);
	SPI_SendByte(buf_addr[2]);
	SPI_SendByte(buf_addr[3]);

	for (uint8_t k = 0; k < SPI_BLOCK_SIZE / BUF_SIZE_RW; ++k) {
		for (i = 0; i < BUF_SIZE_RW; ++i) {
			buf_rw[i] = SPI_ReceiveByte();
		}
		usb_serial_write(buf_rw, BUF_SIZE_RW);
	}

	uint16_t rest = SPI_BLOCK_SIZE - ((SPI_BLOCK_SIZE / BUF_SIZE_RW) * BUF_SIZE_RW);
	for (i = 0; i < rest; ++i) {
		buf_rw[i] = SPI_ReceiveByte();
	}
	
	HWSPI_CS_HIGH;

	usb_serial_write(buf_rw, rest);

	/* Restore global interrupt flag */
	SREG = sreg;

	return 1;
}

uint8_t spi_read_block() {
	return hwspi_read_block();
	
	uint16_t i;
	uint8_t sreg;
	
	/* Save global interrupt flag and disable interrupts */
	sreg = SREG;
	cli();
	
	//spi_enable();
	
	/* read command */
	//SPI_CS_LOW;
	//SPI_COMMAND(SPI_COMMAND_EN4B);
	//SPI_CS_HIGH;

	SPI_CS_LOW;
	SPI_COMMAND(SPI_COMMAND_4B_READ);

	/* address */
	SPI_IO_SET(buf_addr[0]);
	SPI_IO_SET(buf_addr[1]);
	SPI_IO_SET(buf_addr[2]);
	SPI_IO_SET(buf_addr[3]);

	//SPI_IO_INPUT;
	
	/* wait for the nand to read this page to the internal page register */
	//wait_ryby();
	
	for (uint8_t k = 0; k < SPI_BLOCK_SIZE / BUF_SIZE_RW; ++k) {
		for (i = 0; i < BUF_SIZE_RW; ++i) {
			SPI_IO_READ(buf_rw[i]);
		}
		usb_serial_write(buf_rw, BUF_SIZE_RW);
	}

	uint16_t rest = SPI_BLOCK_SIZE - ((SPI_BLOCK_SIZE / BUF_SIZE_RW) * BUF_SIZE_RW);
	for (i = 0; i < rest; ++i) {
		SPI_IO_READ(buf_rw[i]);
	}
	SPI_CS_HIGH;
	usb_serial_write(buf_rw, rest);

	/* Restore global interrupt flag */
	SREG = sreg;

	return 1;
}

uint8_t hwspi_status() {
	uint8_t status;
	
	HWSPI_CS_LOW;
	SPI_SendByte(SPI_COMMAND_RDSR);
	status = SPI_ReceiveByte();
	HWSPI_CS_HIGH;
	
	return status;
}

uint8_t spi_status() {
	return hwspi_status();
	
	uint8_t status;
	
	SPI_CS_LOW;
	SPI_COMMAND(SPI_COMMAND_RDSR);
	SPI_IO_READ(status);
	SPI_CS_HIGH;

	return status;
}

uint8_t hwspi_security() {
	uint8_t sec;
	
	HWSPI_CS_LOW;
	SPI_SendByte(SPI_COMMAND_RDSCUR);
	sec = SPI_ReceiveByte();
	HWSPI_CS_HIGH;
	
	return sec;
}

uint8_t spi_security() {
	return hwspi_security();
	
	uint8_t sec;
	
	SPI_CS_LOW;
	SPI_COMMAND(SPI_COMMAND_RDSCUR);
	SPI_IO_READ(sec);
	SPI_CS_HIGH;

	return sec;
}

int8_t hwspi_write_block() {
	uint8_t i;
	uint16_t k;
	int16_t in_data;
	int8_t ret = 1;

	if (SPI_USE_3B_CMDS && SPI_ADDRESS_LENGTH == 4) {
		HWSPI_CS_LOW;
		SPI_SendByte(SPI_COMMAND_EN4B);
		HWSPI_CS_HIGH;
	}

	HWSPI_WP_HIGH;
	
	for (i = 0; i < BUF_SIZE_RW/256; ++i) {
		// set write enable bit
		HWSPI_WREN;
	
		/* Serial Data Input command */
		HWSPI_CS_LOW;
		
		if (SPI_USE_3B_CMDS || SPI_ADDRESS_LENGTH == 3)
			SPI_SendByte(SPI_COMMAND_3B_PAGEPROG);
		else
			SPI_SendByte(SPI_COMMAND_4B_PAGEPROG);

		/* address */
		if (SPI_ADDRESS_LENGTH == 4) SPI_SendByte(buf_addr[0]);
		SPI_SendByte(buf_addr[1]);
		SPI_SendByte(buf_addr[2] | i);
		SPI_SendByte(buf_addr[3]);

		for (k = 0; k < 256; ++k) {
			if ((in_data = usb_serial_getchar()) != -1) {
				SPI_SendByte(in_data);
			}
			else {
				ret = -1;
				break;
			}
		}
		
		HWSPI_CS_HIGH;
		
		// wait for the internal controller to finish the program command
		SPI_BUSY_WAIT;
		
		if (ret == -1) break;
		
		if ((maker_code == 0xC2) && (device_code0 == 0x18)) {
			if (hwspi_security() & SPI_SECURITY_P_FAIL) {
				ret = 0;
			}
		}
	}
	
	HWSPI_WP_LOW;

	return ret;
}

int8_t spi_write_block() {
	return hwspi_write_block();
	
	uint16_t i;
	uint16_t k;
	int16_t in_data;
	int8_t ret = 1;
	
	//spi_enable();


	for (i = 0; i < SPI_BLOCK_SIZE/64; ++i) {
		// set write enable bit
		SPI_CS_LOW;
		SPI_COMMAND(SPI_COMMAND_WREN);
		SPI_CS_HIGH;
	
		/* Serial Data Input command */
		SPI_CS_LOW;
		SPI_COMMAND(SPI_COMMAND_4B_PAGEPROG);

		/* address */
		//buf_addr[2] = i;
		SPI_IO_SET(buf_addr[0]);
		SPI_IO_SET(buf_addr[1]);
		SPI_IO_SET(i);
		SPI_IO_SET(buf_addr[3]);

		for (k = 0; k < 256; ++k) {
			if ((in_data = usb_serial_getchar()) != -1) {
				SPI_IO_SET(in_data);
			}
			else {
				break;
			}
		}
		SPI_CS_HIGH;
		/* wait for the internal controller to finish the program command 
			TBD - up to 200us */
		SPI_BUSY_WAIT;
		if (spi_security() & SPI_SECURITY_P_FAIL) {
			ret = 0;
		}
	}

	/* Page Program confirm command */
	//SPI_COMMAND(SPI_COMMAND_PAGEPROG2);

	if (i < SPI_BLOCK_SIZE/64) { // timeout
		return -1;		// and exit
	}

	return ret; //!(spi_security() & SPI_SECURITY_P_FAIL);
}

int8_t hwspi_erase_chip() {
	HWSPI_WP_HIGH;

	// set write enable bit
	HWSPI_WREN;

	// block erase setup command
	HWSPI_CS_LOW;
	SPI_SendByte(SPI_COMMAND_CHIP_ERASE);
	HWSPI_CS_HIGH;

	// wait for the internal controller to finish the erase command
	SPI_BUSY_WAIT;

	HWSPI_WP_LOW;

	if ((hwspi_status() & (SPI_STATUS_BP0 | SPI_STATUS_BP1 | SPI_STATUS_BP2 | SPI_STATUS_BP3 | SPI_STATUS_SRWD))) {
		return -1;
	}

	if ((maker_code == 0xC2) && (device_code0 == 0x18)) {
		if (hwspi_security() & (SPI_SECURITY_E_FAIL | SPI_SECURITY_WPSEL)) return 0;
	}
		
	return 1;
}

int8_t spi_erase_chip() {
	return hwspi_erase_chip();
	
	//spi_enable();

	// set write enable bit
	//SPI_CS_LOW;
	//SPI_COMMAND(SPI_COMMAND_WREN);
	//SPI_CS_HIGH;
	SPI_WREN;
	/* block erase setup command */
	SPI_CS_LOW;
	SPI_COMMAND(SPI_COMMAND_CHIP_ERASE);
	SPI_CS_HIGH;
	_delay_ns(125);
	/* wait for the internal controller to finish the erase command 
		TBD - up to 2ms */
	SPI_BUSY_WAIT;

	if ((spi_status() & (SPI_STATUS_BP0 | SPI_STATUS_BP1 | SPI_STATUS_BP2 | SPI_STATUS_BP3 | SPI_STATUS_SRWD))) {
		return -1;
	}
		
	if (spi_security() & (SPI_SECURITY_E_FAIL | SPI_SECURITY_WPSEL)) return 0;
	
	return 1;
}

int8_t hwspi_erase_block() {
	if (SPI_USE_3B_CMDS && SPI_ADDRESS_LENGTH == 4) {
		HWSPI_CS_LOW;
		SPI_SendByte(SPI_COMMAND_EN4B);
		HWSPI_CS_HIGH;
	}

	HWSPI_WP_HIGH;

	// set write enable bit
	HWSPI_WREN;

	/* block erase setup command */
	HWSPI_CS_LOW;
	
	if (SPI_USE_3B_CMDS || SPI_ADDRESS_LENGTH == 3)
		SPI_SendByte(SPI_COMMAND_3B_BLOCK_ERASE_64K);
	else
		SPI_SendByte(SPI_COMMAND_4B_BLOCK_ERASE_64K);
		
	/* block address */
	if (SPI_ADDRESS_LENGTH == 4) SPI_SendByte(buf_addr[0]);
	SPI_SendByte(buf_addr[1]);
	SPI_SendByte(buf_addr[2]);
	SPI_SendByte(buf_addr[3]);
	
	HWSPI_CS_HIGH;
	
	// wait for the internal controller to finish the erase command
	SPI_BUSY_WAIT;

	HWSPI_WP_LOW;

	if ((hwspi_status() & (SPI_STATUS_BP0 | SPI_STATUS_BP1 | SPI_STATUS_BP2 | SPI_STATUS_BP3 | SPI_STATUS_SRWD))) {
		return -1;
	}
	
	if ((maker_code == 0xC2) && (device_code0 == 0x18)) {
		if (hwspi_security() & (SPI_SECURITY_E_FAIL | SPI_SECURITY_WPSEL)) return 0;
	}
		
	return 1;
}

int8_t spi_erase_block() {
	return hwspi_erase_block();
	
	//spi_enable();

	// set write enable bit
	//SPI_CS_LOW;
	//SPI_COMMAND(SPI_COMMAND_WREN);
	//SPI_CS_HIGH;
	SPI_WREN;
	/* block erase setup command */
	SPI_CS_LOW;
	SPI_COMMAND(SPI_COMMAND_4B_BLOCK_ERASE_64K);

	/* block address */
	SPI_IO_SET(buf_addr[0]);
	SPI_IO_SET(buf_addr[1]);
	SPI_IO_SET(buf_addr[2]);
	SPI_IO_SET(buf_addr[3]);
	SPI_CS_HIGH;
	
	/* wait for the internal controller to finish the erase command 
		TBD - up to 2ms */
	SPI_BUSY_WAIT;

	if ((spi_status() & (SPI_STATUS_BP0 | SPI_STATUS_BP1 | SPI_STATUS_BP2 | SPI_STATUS_BP3 | SPI_STATUS_SRWD))) {
		return -1;
	}
		
	if (spi_security() & (SPI_SECURITY_E_FAIL | SPI_SECURITY_WPSEL)) return 0;
	
	return 1;
}

void handle_read_block() {
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
	spi_read_block();
}

void handle_write_block() {
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

	res = spi_write_block();
	if (res > 0)
		usb_serial_putchar('K');
	else if (res < 0)
		usb_serial_putchar('R');
	else
		usb_serial_putchar('V');
}

void handle_erase_chip() {
	int8_t res;

	res = spi_erase_chip();
	if (res > 0)
		usb_serial_putchar('K');
	else if (res < 0)
		usb_serial_putchar('P');
	else
		usb_serial_putchar('V');
}

void handle_erase_block() {
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
	
	res = spi_erase_block();
	if (res > 0)
		usb_serial_putchar('K');
	else if (res < 0)
		usb_serial_putchar('P');
	else
		usb_serial_putchar('V');
}

void handle_read_id() {
	if (spi_read_id()) {
		usb_serial_putchar(maker_code);
		usb_serial_putchar(device_code0);
		//usb_serial_putchar(device_code1);
	}
	else {
		usb_serial_putchar(0); //maker_code
		usb_serial_putchar(0); //device_code0
		//usb_serial_putchar(0); //device_code1
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
	int16_t command = -1;
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

	//Init SPI
	SPI_Init(SPI_SPEED_FCPU_DIV_2 | SPI_ORDER_MSB_FIRST | SPI_SCK_LEAD_RISING | SPI_SAMPLE_LEADING | SPI_MODE_MASTER);
	hwspi_init();
	
	// Initialize the USB, and then wait for the host to set configuration.
	// If the Teensy is powered without a PC connected to the USB port,
	// this will wait forever.
	usb_init();

	while (!usb_configured()) /* wait */ ;

	// Wait an extra second for the PC's operating system to load drivers
	// and do whatever it does to actually be ready for input
	_delay_ms(1000);

	while (1) {
		// discard anything that was received prior.  Sometimes the
		// operating system or other software will send a modem
		// "AT command", which can still be buffered.
		usb_serial_flush_input();

		while (usb_configured()) { // is user still connected?
			command = usb_serial_getchar();
			if (command == -1) continue;

			switch (command) {
			case CMD_PING1:
				usb_serial_putchar(VERSION_MAJOR);
				break;
				
			case CMD_PING2:
				freemem = freeRam();
				usb_serial_putchar(VERSION_MINOR);
				usb_serial_putchar((freemem >> 8) & 0xFF);
				usb_serial_putchar(freemem & 0xFF);
				break;
				
			case CMD_BOOTLOADER:
				bootloader();
				break;
				
			case CMD_IO_LOCK:
				break;
				
			case CMD_IO_RELEASE:
				break;
				
			case CMD_PULLUPS_DISABLE:
				IO_PULLUPS = 0;
				break;
				
			case CMD_PULLUPS_ENABLE:
				IO_PULLUPS = 0xFF;
				break;
				
			case CMD_SPI_ID:
				handle_read_id();
				break;
				
			case CMD_SPI_READBLOCK:
				handle_read_block();
				break;
				
			case CMD_SPI_WRITESECTOR:
				handle_write_block();
				break;
				
			case CMD_SPI_ERASEBLOCK:
				handle_erase_block();
				break;
				
			case CMD_SPI_ERASECHIP:
				handle_erase_chip();
				break;
				
			case CMD_SPI_3BYTE_ADDRESS:
				SPI_ADDRESS_LENGTH = 3;
				break;
			
			case CMD_SPI_4BYTE_ADDRESS:
				SPI_ADDRESS_LENGTH = 4;
				break;
			
			case CMD_SPI_3BYTE_CMDS:
				SPI_USE_3B_CMDS = 1;
				break;
			
			case CMD_SPI_4BYTE_CMDS:
				SPI_USE_3B_CMDS = 0;
				break;
			
			default:
				break;
			}
		}		
	}
}
