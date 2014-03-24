# *************************************************************************
# SPIway.py - Teensy++ 2.0 SPI flasher for PS4
#
# Copyright (C) 2013 judges@eEcho.com
#
# This code is licensed to you under the terms of the GNU GPL, version 2;
# see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
# *************************************************************************

import serial, time, datetime, sys, struct

class TeensySerialError(Exception):
	pass

class TeensySerial(object):
	BUFSIZE = 32768

	def __init__(self, port):
		self.ser = serial.Serial(port, 9600, timeout=300, rtscts=False, dsrdtr=False, xonxoff=False, writeTimeout=120)
		if self.ser is None:
			raise TeensySerialError("could not open serial %s")%port
		self.ser.flushInput()
		self.ser.flushOutput()
		self.obuf = ""

	def write(self, s):
		if isinstance(s,int):
			s = chr(s)
		elif isinstance(s,tuple) or isinstance(s,list):
			s = ''.join([chr(c) for c in s])
		self.obuf += s
		while len(self.obuf) > self.BUFSIZE:
			self.ser.write(self.obuf[:self.BUFSIZE])
			self.obuf = self.obuf[self.BUFSIZE:]

	def flush(self):
		if len(self.obuf):
			self.ser.write(self.obuf)
			self.ser.flush()
			self.obuf = ""

	def read(self, size):
		self.flush()
		data = self.ser.read(size)
		return data

	def readbyte(self):
		return ord(self.read(1))

	def close(self):
		print
		print "Closing serial device..."
		if self.ser is None:
			print "Device already closed."
		else:
			self.ser.close()
			print "Done."

class SPIError(Exception):
	pass

class SPIFlasher(TeensySerial):
	VERSION_MAJOR = 0
	VERSION_MINOR = 0
	SPI_DISABLE_PULLUPS = 0
	MF_ID = 0
	DEVICE_ID = 0
	SPI_SECTOR_SIZE = 0
	SPI_TOTAL_SECTORS = 0
	SPI_BLOCK_COUNT = 0
	SPI_SECTORS_PER_BLOCK = 0
	SPI_BLOCK_SIZE = 0
	SPI_ADDRESS_LENGTH = 0

	# Teensy commands
	CMD_PING1 = 0
	CMD_PING2 = 1
	CMD_BOOTLOADER = 2
	CMD_IO_LOCK = 3
	CMD_IO_RELEASE = 4
	CMD_PULLUPS_DISABLE = 5
	CMD_PULLUPS_ENABLE = 6
	CMD_SPI_ID = 7
	CMD_SPI_READBLOCK = 8
	CMD_SPI_WRITESECTOR = 9
	CMD_SPI_ERASEBLOCK = 10
	CMD_SPI_ERASECHIP = 11
	CMD_SPI_3BYTE_ADDRESS = 12
	CMD_SPI_4BYTE_ADDRESS = 13
	
	def __init__(self, port, ver_major, ver_minor):
		if port:
			TeensySerial.__init__(self, port)
		self.SPI_DISABLE_PULLUPS = 0
		self.VERSION_MAJOR = ver_major
		self.VERSION_MINOR = ver_minor

	def ping(self):
		self.write(self.CMD_PING1)
		self.write(self.CMD_PING2)
		ver_major = self.readbyte()
		ver_minor = self.readbyte()
		freeram = (self.readbyte() << 8) | self.readbyte()
		if (ver_major != self.VERSION_MAJOR) or (ver_minor != self.VERSION_MINOR):
			print "Ping failed (expected v%d.%02d, got v%d.%02d)"%(self.VERSION_MAJOR, self.VERSION_MINOR, ver_major, ver_minor)
			self.close()
			sys.exit(1)

		return freeram

	def readid(self):
		if (self.SPI_DISABLE_PULLUPS == 0):
			self.write(self.CMD_PULLUPS_ENABLE)
		else:
			self.write(self.CMD_PULLUPS_DISABLE)

		self.write(self.CMD_SPI_ID)

		spi_info = self.read(2)
		
#		print "Raw ID data: 0x%02x 0x%02x"%(ord(spi_info[0]), ord(spi_info[1]))

		self.MF_ID = ord(spi_info[0])
		self.DEVICE_ID = ord(spi_info[1])

	def printstate(self):
		print "SPI information"
		print "---------------"
		self.readid()

		if self.MF_ID == 0xC2:
			print "Chip manufacturer: Macronix (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0x18:
				print "Chip type:         MX25L25635F (0x%02x)"%self.DEVICE_ID
				self.SPI_BLOCK_COUNT = 512
				self.SPI_SECTORS_PER_BLOCK = 16
				self.SPI_SECTOR_SIZE = 0x1000
				self.SPI_TOTAL_SECTORS = self.SPI_SECTORS_PER_BLOCK * self.SPI_BLOCK_COUNT
				self.SPI_BLOCK_SIZE = self.SPI_SECTORS_PER_BLOCK * self.SPI_SECTOR_SIZE
				self.SPI_ADDRESS_LENGTH = 4

			elif self.DEVICE_ID == 0x10:
				print "Chip type:         MX25L1006E (0x%02x)"%self.DEVICE_ID
				self.SPI_BLOCK_COUNT = 2
				self.SPI_SECTORS_PER_BLOCK = 16
				self.SPI_SECTOR_SIZE = 0x1000
				self.SPI_TOTAL_SECTORS = self.SPI_SECTORS_PER_BLOCK * self.SPI_BLOCK_COUNT
				self.SPI_BLOCK_SIZE = self.SPI_SECTORS_PER_BLOCK * self.SPI_SECTOR_SIZE
				self.SPI_ADDRESS_LENGTH = 3

			else:
				print "Chip type:         unknown (0x%02x)"%self.DEVICE_ID
				self.close()
				sys.exit(1)

		elif self.MF_ID == 0xEF:
			print "Chip manufacturer: Winbond (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0x10:
				print "Chip type:         W25X10CL (0x%02x)"%self.DEVICE_ID
				self.SPI_BLOCK_COUNT = 2
				self.SPI_SECTORS_PER_BLOCK = 16
				self.SPI_SECTOR_SIZE = 0x1000
				self.SPI_TOTAL_SECTORS = self.SPI_SECTORS_PER_BLOCK * self.SPI_BLOCK_COUNT
				self.SPI_BLOCK_SIZE = self.SPI_SECTORS_PER_BLOCK * self.SPI_SECTOR_SIZE
				self.SPI_ADDRESS_LENGTH = 3
			elif self.DEVICE_ID == 0x13:
				print "Chip type:         W25Q80BV (0x%13x)"%self.DEVICE_ID
				self.SPI_BLOCK_COUNT = 16
				self.SPI_SECTORS_PER_BLOCK = 16
				self.SPI_SECTOR_SIZE = 0x1000
				self.SPI_TOTAL_SECTORS = self.SPI_SECTORS_PER_BLOCK * self.SPI_BLOCK_COUNT
				self.SPI_BLOCK_SIZE = self.SPI_SECTORS_PER_BLOCK * self.SPI_SECTOR_SIZE
				self.SPI_ADDRESS_LENGTH = 3

			else:
				print "Chip type:         unknown (0x%02x)"%self.DEVICE_ID
				self.close()
				sys.exit(1)
		else:
			print "Chip manufacturer: unknown (0x%02x)"%self.MF_ID
			print "Chip type:         unknown (0x%02x)"%self.DEVICE_ID
			self.close()
			sys.exit(1)

		print
		if (self.SPI_BLOCK_SIZE * self.SPI_BLOCK_COUNT / 1024) <= 8192:
			print "Chip size:         %d KB"%(self.SPI_BLOCK_SIZE * self.SPI_BLOCK_COUNT / 1024)
		else:
			print "Chip size:         %d MB"%(self.SPI_BLOCK_SIZE * self.SPI_BLOCK_COUNT / 1024 / 1024)

		print "Sector size:       %d bytes"%(self.SPI_SECTOR_SIZE)
		print "Block size:        %d bytes"%(self.SPI_BLOCK_SIZE)
		print "Sectors per block: %d"%(self.SPI_SECTORS_PER_BLOCK)
		print "Number of blocks:  %d"%(self.SPI_BLOCK_COUNT)
		print "Number of sectors: %d"%(self.SPI_TOTAL_SECTORS)

	def bootloader(self):
		self.write(self.CMD_BOOTLOADER)
		self.flush()

	def read_result(self):
		# read status byte
		res = self.readbyte()
		
		# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout, 'V' = Verification error
		error_msg = ""
		
		if (res != 75): #'K'
			if (res == 84): #'T'
				error_msg = "RY/BY timeout error while writing!"
			elif (res == 82): #'R'
				self.close()
				raise SPIError("Teensy receive buffer timeout! Disconnect and reconnect Teensy!")
			elif (res == 86): #'V'
				error_msg = "Verification error!"
			elif (res == 80): #'P'
				error_msg = "Device is write-protected!"
			else:
				self.close()
				raise SPIError("Received unknown error! (Got 0x%02x)"%res)

			print error_msg
			return 0

		return 1

	def erase_chip(self):
		self.write(self.CMD_SPI_ERASECHIP)

		if self.read_result() == 0:
			print "Error erasing chip!"
			return 0

		return 1

	def erase_block(self, block):
		if self.SPI_ADDRESS_LENGTH == 3:
			self.write(self.CMD_SPI_3BYTE_ADDRESS)
		else:
			self.write(self.CMD_SPI_4BYTE_ADDRESS)

		self.write(self.CMD_SPI_ERASEBLOCK)

		# set address (msb first)
		address = block * self.SPI_BLOCK_SIZE
		self.write((address >> 24) & 0xFF)
		self.write((address >> 16) & 0xFF)
		self.write((address >> 8) & 0xFF)
		self.write(address & 0xFF)

		if self.read_result() == 0:
			print "Block %d - error erasing block"%(block)
			return 0

		return 1

	def readblock(self, block):
		if self.SPI_ADDRESS_LENGTH == 3:
			self.write(self.CMD_SPI_3BYTE_ADDRESS)
		else:
			self.write(self.CMD_SPI_4BYTE_ADDRESS)

		self.write(self.CMD_SPI_READBLOCK)

		# set address (msb first)
		address = block * self.SPI_BLOCK_SIZE
		self.write((address >> 24) & 0xFF)
		self.write((address >> 16) & 0xFF)
		self.write((address >> 8) & 0xFF)
		self.write(address & 0xFF)
		
		if self.read_result() == 0:
			return "error"
		
		data = self.read(self.SPI_BLOCK_SIZE)
		return data
		
	def dump(self, filename, block_offset, nblocks):
		fo = open(filename,"wb")

		if nblocks == 0:
			nblocks = self.SPI_BLOCK_COUNT

		if nblocks > self.SPI_BLOCK_COUNT:
			nblocks = self.SPI_BLOCK_COUNT
		
		for block in range(block_offset, (block_offset+nblocks), 1):
			data = self.readblock(block)
			fo.write(data)
			print "\r%d KB / %d KB"%((block-block_offset+1)*self.SPI_BLOCK_SIZE/1024, nblocks*self.SPI_BLOCK_SIZE/1024),
			sys.stdout.flush()

		return

	def writesector(self, data, sector):
		if len(data) != self.SPI_SECTOR_SIZE:
			print "Incorrent data size %d"%(len(data))
			
		if self.SPI_ADDRESS_LENGTH == 3:
			self.write(self.CMD_SPI_3BYTE_ADDRESS)
		else:
			self.write(self.CMD_SPI_4BYTE_ADDRESS)

		self.write(self.CMD_SPI_WRITESECTOR)

		# set address (msb first)
		address = sector * self.SPI_SECTOR_SIZE
		self.write((address >> 24) & 0xFF)
		self.write((address >> 16) & 0xFF)
		self.write((address >> 8) & 0xFF)
		self.write(address & 0xFF)

		self.write(data)
		
		if self.read_result() == 0:
			return 0
		
		return 1

	def program_block(self, data, pgblock, verify):
		datasize = len(data)
		if datasize != self.SPI_BLOCK_SIZE:
			print "Incorrect length %d != %d!"%(datasize, self.SPI_BLOCK_SIZE)
			return -1
		
		sectornr = 0
		while sectornr < self.SPI_SECTORS_PER_BLOCK:
			real_sectornr = (pgblock * self.SPI_SECTORS_PER_BLOCK) + sectornr
			if sectornr == 0:
				self.erase_block(pgblock)

			self.writesector(data[sectornr*self.SPI_SECTOR_SIZE:(sectornr+1)*self.SPI_SECTOR_SIZE], real_sectornr)
				
			sectornr += 1

		# verification
		if verify == 1:
			if data != self.readblock(pgblock):
				print
				print "Error! Block verification failed (block=%d)."%(pgblock)
				return  -1
				
		return 0

	def program(self, data, verify, block_offset, nblocks):
		datasize = len(data)

		if nblocks == 0:
			nblocks = self.SPI_BLOCK_COUNT - block_offset
			
		# validate that the data is a multiplication of self.SPI_BLOCK_SIZE
		if datasize % self.SPI_BLOCK_SIZE:
			print "Error: expecting file size to be a multiplication of block size: %d"%(self.SPI_BLOCK_SIZE)
			return -1

		# validate that the the user didn't want to read from incorrect place in the file
		if block_offset + nblocks > datasize/self.SPI_BLOCK_SIZE:
			print "Error: file is %d bytes long and last block is at %d!"%(datasize, (block_offset + nblocks + 1) * self.SPI_BLOCK_SIZE)
			return -1
		
		# validate that the the user didn't want to write to incorrect place on the chip
		if block_offset + nblocks > self.SPI_BLOCK_COUNT:
			print "Error: chip has %d blocks. Writing outside the chip's capacity!"%(self.SPI_BLOCK_COUNT, block_offset + nblocks + 1)
			return -1
		
		block = 0

		print "Writing %d blocks to device (starting at offset %d)..."%(nblocks, block_offset)
		
		while block < nblocks:
			pgblock = block+block_offset
			self.program_block(data[pgblock*self.SPI_BLOCK_SIZE:(pgblock+1)*self.SPI_BLOCK_SIZE], pgblock, verify)
			print "\r%d KB / %d KB"%(((block+1)*self.SPI_BLOCK_SIZE)/1024, (nblocks*self.SPI_BLOCK_SIZE)/1024),
			sys.stdout.flush()

			block += 1

		print
		

if __name__ == "__main__":
	VERSION_MAJOR = 0
	VERSION_MINOR = 30

	print "SPIway v%d.%02d - Teensy++ 2.0 SPI Flasher for PS4"%(VERSION_MAJOR, VERSION_MINOR)
	print "Copyright (C) 2013 judges@eEcho.com"
	print

	if len(sys.argv) == 1:
		print "Usage:"
		print "SPIway.py Serial-Port Command"
		print
		print "  Serial-Port  Name of serial port to open (eg. COM1, COM2, /dev/ttyACM0, etc)"
		print "  Commands:"
		print "  *  info"
		print "     Displays chip information"
		print "  *  dump Filename [Offset] [Length]"
		print "     Dumps to Filename at [Offset] and [Length] "
		print "  *  vwrite/write Filename [Offset] [Length]"
		print "     Flashes (v=verify) Filename at [Offset] and [Length]"
#		print "  *  vdiffwrite/diffwrite Filename Diff-file"
#		print "     Flashes (v=verify) Filename using a Diff-file"
		print "  *  erasechip"
		print "     Erases the entire chip"
		print "  *  bootloader"
		print "     Enters Teensy's bootloader mode (for Teensy reprogramming)"
		print
		print "     Notes: 1) All offsets and lengths are in decimal (number of blocks)"
#		print "            2) The Diff-file is a file which lists all the changed"
#		print "               offsets of a dump file. This will increase flashing"
#		print "               time dramatically."
		print
		print "Examples:"
		print "  SPIway.py COM1 info"
		print "  SPIway.py COM1 dump d:\myflash.bin"
		print "  SPIway.py COM1 dump d:\myflash.bin 10 20"
		print "  SPIway.py COM1 write d:\myflash.bin"
		print "  SPIway.py COM3 write d:\myflash.bin 0 20"
		print "  SPIway.py COM3 vwrite d:\myflash.bin"
		print "  SPIway.py COM3 vwrite d:\myflash.bin 10 20"
#		print "  SPIway.py COM4 diffwrite d:\myflash.bin d:\myflash_diff.txt"
#		print "  SPIway.py COM3 vdiffwrite d:\myflash.bin d:\myflash_diff.txt"
		print "  SPIway.py COM3 erasechip"
		print "  SPIway.py COM1 bootloader"
		sys.exit(0)

	n = SPIFlasher(sys.argv[1], VERSION_MAJOR, VERSION_MINOR)
	print "Pinging Teensy..."
	freeram = n.ping()
	print "Available memory: %d bytes"%(freeram)
	print
	
	tStart = time.time()
	if len(sys.argv) in (4,5,6) and sys.argv[2] == "dump":
		n.printstate()
		print
		print "Dumping...",
		sys.stdout.flush()
		print
		
		block_offset=0
		nblocks=0

		if len(sys.argv) == 5:
			block_offset=int(sys.argv[4])
		elif len(sys.argv) == 6:
			block_offset=int(sys.argv[4])
			nblocks=int(sys.argv[5])

		n.dump(sys.argv[3], block_offset, nblocks)
		
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
			
	if len(sys.argv) == 3 and sys.argv[2] == "info":
		n.printstate()
#		print
			
	elif len(sys.argv) in (4,5,6) and (sys.argv[2] == "write" or sys.argv[2] == "vwrite"):
		n.printstate()
		print
		
		print "Writing...",
		sys.stdout.flush()

		print
		
		data = open(sys.argv[3],"rb").read()

		block_offset=0
		nblocks=0
		verify=0

		if (sys.argv[2] == "vwrite"):
			verify=1
		
		if len(sys.argv) == 5:
			block_offset=int(sys.argv[4])
		elif len(sys.argv) == 6:
			block_offset=int(sys.argv[4])
			nblocks=int(sys.argv[5])

		n.program(data, verify, block_offset, nblocks)
			
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		
#	elif len(sys.argv) == 5 and (sys.argv[2] == "diffwrite" or sys.argv[2] == "vdiffwrite"):
#		n.printstate()
#		print
#		print "Writing using diff file ..."
#		sys.stdout.flush()
#		print
#		
#		data = open(sys.argv[3],"rb").read()
#		diff_data = open(sys.argv[4],"rb").readlines()
#
#		block_offset=0
#		nblocks=0
#		verify=0
#		nlines=len(diff_data)
#		cur_line=0
#
#		if (sys.argv[2] == "vdiffwrite"):
#			verify=1
#		
#		for line in diff_data:
#			addr=int(line[2:], 16)
#			if addr % n.SPI_BLOCK_SIZE:
#				print "Error: incorrect address for block addr=%x. addresses must be on a per-block boundary"%(addr)
#				sys.exit(0)
#
#			block_offset=addr/n.SPI_BLOCK_SIZE
#			print "Programming offset %x block %x (%d/%d)"%(addr, block_offset, cur_line+1, nlines)
#			n.program(data, verify, block_offset, 1)
#			cur_line += 1
#			
#		print
#		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		
	elif len(sys.argv) == 3 and sys.argv[2] == "erasechip":
		n.printstate()
		print
		print "Erasing chip..."
		n.erase_chip()
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))

	elif len(sys.argv) == 3 and sys.argv[2] == "bootloader":
		print
		print "Entering Teensy's bootloader mode... Goodbye!"
		n.bootloader()
		sys.exit(0)

	n.ping()
