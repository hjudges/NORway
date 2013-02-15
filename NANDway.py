#!/usr/bin/python
# *************************************************************************
# Teensy++ 2.0 modifications by Effleurage
#  NANDway.py
#
# Teensy++ 2.0 modifications by judges@eEcho.com
# *************************************************************************
#  noralizer.py - NOR flasher for PS3
#
# Copyright (C) 2010-2011  Hector Martin "marcan" <hector@marcansoft.com>
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

class NANDError(Exception):
	pass

class NANDFlasher(TeensySerial):
	MF_ID = 0xEC
	DEVICE_ID = 0xF1
	NAND_PAGE_SZ = 2048
	NAND_RAS = 64 # Redundent Area Size
	NAND_PAGE_SZ_PLUS_RAS = 2112
	NAND_NPAGES = 1024*64
	NAND_NBLOCKS = 1024
	NAND_PAGES_PER_BLOCK = 64
	NAND_BLOCK_SZ = 2048*64
	NAND_BLOCK_SZ_PLUS_RAS = 2112*64

	def __init__(self, port, nand_id):
		if port:
			TeensySerial.__init__(self, port)
		self.NAND_ID=nand_id

	def ping(self):
		self.write(0x02)
		self.write(0x03)
		val = self.readbyte()
		if val != 0x43:
			self.close()
			if val == 0x42:
				raise NANDError("Ping failed (NORWay hex? use Norway.py instead.)"%val)
			else:
				raise NANDError("Ping failed (expected 0x42, got 0x%02x)"%val)
		val = self.readbyte()
		if val != 0xbe:
			self.close()
			if val == 0xbd:
				raise NANDError("Ping failed (NORWay hex? use Norway.py instead.)"%val)
			else:
				raise NANDError("Ping failed (expected 0xbd, got 0x%02x)"%val)

	def readid(self):
		if (self.NAND_ID==1):
			self.write(22)
		else:
			self.write("\x0C")
		
		self.MF_ID=self.readbyte()
		self.DEVICE_ID=self.readbyte()
		self.readbyte()
		
		info=self.readbyte()

		self.readbyte()
		
		print "chip %x, device_id %x, %x"%(self.MF_ID, self.DEVICE_ID, info)

		if (self.MF_ID == 0xEC): 
			if (info & 3) == 0:
				self.NAND_PAGE_SZ = 1024*1
			if (info & 3) == 1:
				self.NAND_PAGE_SZ = (1024*2)
			
			if ((info>>4) & 3) == 0:
				self.NAND_BLOCK_SZ = 64*1024
			if ((info>>4) & 3) == 1:
				self.NAND_BLOCK_SZ = 128*1024
			if ((info>>4) & 3) == 0:
				self.NAND_BLOCK_SZ = 256*1024
			
			if ((info>>2) & 1) == 0:
				self.NAND_RAS = 8
			if ((info>>2) & 1) == 1:
				self.NAND_RAS = 16
			
			self.NAND_PAGE_SZ_PLUS_RAS = self.NAND_PAGE_SZ + (self.NAND_PAGE_SZ/512)*self.NAND_RAS
			self.NAND_PAGES_PER_BLOCK = self.NAND_BLOCK_SZ / self.NAND_PAGE_SZ
			self.NAND_BLOCK_SZ_PLUS_RAS = self.NAND_PAGES_PER_BLOCK * self.NAND_PAGE_SZ_PLUS_RAS
			self.NAND_NPAGES = 65536
			self.NAND_NBLOCKS = 1024		
			
		elif (self.MF_ID == 0xAD): 
			self.NAND_PAGE_SZ = 512
			self.NAND_PAGES_PER_BLOCK = 32
			self.NAND_BLOCK_SZ = self.NAND_PAGES_PER_BLOCK * self.NAND_PAGE_SZ
			self.NAND_RAS = 16
			self.NAND_PAGE_SZ_PLUS_RAS = self.NAND_PAGE_SZ + self.NAND_RAS
			self.NAND_NBLOCKS = 1024
			self.NAND_NPAGES = self.NAND_PAGES_PER_BLOCK * self.NAND_NBLOCKS
			
		
		if (self.MF_ID == 0):
			print
			print "Unknown chip manufacturer! Exiting..."
			self.close()
			sys.exit(1)
		if (self.DEVICE_ID == 0):
			print
			print "Unknown device id! Exiting..."
			self.close()
			sys.exit(1)
			
	def printstate(self):
		
		self.readid()

		if self.MF_ID == 0xEC:
			print "NAND chip manufacturer: Samsung (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0xA1:
				print "NAND chip type: K9F1G08R0A (0x%02x)"%self.DEVICE_ID
			if self.DEVICE_ID == 0xF1:
				print "NAND chip type: K9F1G08U0A (0x%02x)"%self.DEVICE_ID
		elif self.MF_ID == 0xAD:
			print "NAND chip manufacturer: Hynix (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0x73:
				print "NAND chip type: HY27US08281A (0x%02x)"%self.DEVICE_ID
		else:
			print "NAND chip manufacturer: unknown (0x%02x)"%self.MF_ID
			print "NAND chip type: unknown (0x%06x)"%self.DEVICE_ID
			self.MF_ID = 0
			self.DEVICE_ID = 0
			return

		print "Page size: %d"%(self.NAND_PAGE_SZ)
		print "Block size: %d"%(self.NAND_BLOCK_SZ)
		print "Block plus RAS size: %d"%(self.NAND_BLOCK_SZ_PLUS_RAS)
		print "Page RAS size: %d"%(self.NAND_PAGE_SZ_PLUS_RAS)
		print "Pages Per Block: %d"%(self.NAND_PAGES_PER_BLOCK)
		print "Total Blocks: %d"%(self.NAND_NBLOCKS)
		print "Total Pages: %d"%(self.NAND_NPAGES)
		
		print

	def bootloader(self):
		self.write("\x04")
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
				raise NANDError("Teensy receive buffer timeout! Disconnect and reconnect Teensy!")
			elif (res == 86): #'V'
				error_msg = "Verification error!"
			elif (res == 80): #'P'
				error_msg = "Device is write-ptotected!"
			else:
				self.close()
				raise NANDError("Received unknown error! (Got 0x%02x)"%res)

			print error_msg
			return 0

		return 1

	def erase_block(self, pagenr):
		if (self.NAND_ID==1):
			self.write(27)
		else:
			self.write(17)

		pgblock = pagenr / self.NAND_PAGES_PER_BLOCK
		
		# row (page number) address
		self.write(pagenr & 0xFF)
		self.write((pagenr >> 8) & 0xFF)

		if self.read_result() == 0:
			print "Block %x - error erasing block"%(pgblock)
			return 0

		return 1

		
	def validate_page(self, data, pagenr):
		spare = data[self.NAND_PAGE_SZ:]
		pgblock = pagenr / self.NAND_PAGES_PER_BLOCK
		pgoff = pagenr % self.NAND_PAGES_PER_BLOCK
		
		nand_bl = (ord(spare[1])<<8) | ord(spare[0])
		if (self.MF_ID == 0xAD) and (nand_bl != 0xFFFF) and (nand_bl != pgblock) and (pgoff == 0):
			print
			if (pgblock < 0x3D0):
				return 0
		#	else:
		#		print "Found Bad block remapping: %x to %x\n"%(nand_bl, pgblock),
		#		sys.stdout.flush()
		elif (self.MF_ID == 0xEC) and (pgoff==0):
			if ord(spare[0]) == 0:
				return 0
			
			
		return 1


	def readpage(self, page):

		if (self.NAND_ID==1):
			self.write(23)
		else:
			self.write(13)

		# address
		self.write(0x0)
		self.write(0x0)
		self.write(page & 0xFF)
		self.write((page >> 8) & 0xFF)
		
		if self.read_result() == 0:
			return "error"
		
		data = self.read(self.NAND_PAGE_SZ_PLUS_RAS)
		return data
		
		
	def writepage(self, data, pagenr):
		if len(data) != self.NAND_PAGE_SZ_PLUS_RAS:
			print "Incorrent data size %d"%(len(data))
			
		pgblock = pagenr / self.NAND_PAGES_PER_BLOCK
		pgoff = pagenr % self.NAND_PAGES_PER_BLOCK
		
		if (self.NAND_ID==1):
			self.write(26)
		else:
			self.write(16)

		# address
		self.write(0x0)
		self.write(0x0)
		self.write(pagenr & 0xFF)
		self.write((pagenr >> 8) & 0xFF)

		self.write(data)
		
		if self.read_result() == 0:
			return 0
		
		return 1

	def dump(self, filename, block_offset, nblocks):
		fo = open(filename,"wb")

		if nblocks == 0:
			nblocks = self.NAND_NBLOCKS

		if nblocks > self.NAND_NBLOCKS:
			nblocks = self.NAND_NBLOCKS
		
		for page in range(block_offset*self.NAND_PAGES_PER_BLOCK, (block_offset+nblocks)*self.NAND_PAGES_PER_BLOCK, 1):
			data = self.readpage(page)
			page_valid = self.validate_page(data, page)
			if page_valid == 0:
				print
				print "Found invalid bad block %x"%(page / self.NAND_PAGES_PER_BLOCK)
				
			
			fo.write(data)
			print "\r%d KB / %d KB"%((page-(block_offset*self.NAND_PAGES_PER_BLOCK)+1)*self.NAND_PAGE_SZ_PLUS_RAS/1024, nblocks*self.NAND_BLOCK_SZ_PLUS_RAS/1024),
			sys.stdout.flush()

		return

	def program_block(self, data, pgblock, verify):

		pagenr = 0
		
		datasize = len(data)
		if datasize != self.NAND_BLOCK_SZ_PLUS_RAS:
			print "Incorrect length %d != %d"%(datasize, self.NAND_BLOCK_SZ_PLUS_RAS)
			return -1
		
		while pagenr < self.NAND_PAGES_PER_BLOCK:
			real_pagenr = (pgblock * self.NAND_PAGES_PER_BLOCK) + pagenr
			page_valid = self.validate_page(data[pagenr*self.NAND_PAGE_SZ_PLUS_RAS:(pagenr+1)*self.NAND_PAGE_SZ_PLUS_RAS], real_pagenr)

			if pagenr == 0:
				if page_valid == 0:
					print
					print "Block 0x%x - Not valid. skipping..."%(pgblock)
					return -1
				else:
					self.erase_block(real_pagenr)
					
			self.writepage(data[pagenr*self.NAND_PAGE_SZ_PLUS_RAS:(pagenr+1)*self.NAND_PAGE_SZ_PLUS_RAS], real_pagenr)
				
			pagenr += 1


		# verification
		if verify == 1:
			pagenr = 0;
			while pagenr < self.NAND_PAGES_PER_BLOCK:
				real_pagenr = (pgblock * self.NAND_PAGES_PER_BLOCK) + pagenr
				if data[pagenr*self.NAND_PAGE_SZ_PLUS_RAS:(pagenr+1)*self.NAND_PAGE_SZ_PLUS_RAS] != self.readpage(real_pagenr):
					print
					print "Error! Block verification failed. block=0x%x page=%d"%(pgblock, real_pagenr)
					return  -1
					
				pagenr += 1
				
			
		return 0

	def program(self, data, verify, block_offset, nblocks):
		datasize = len(data)

		if nblocks == 0:
			nblocks = self.NAND_NBLOCKS - block_offset
			
		# validate that the data is a multiplication of self.NAND_BLOCK_SZ_PLUS_RAS
		if datasize % self.NAND_BLOCK_SZ_PLUS_RAS:
			print "Error: expecting file size to be a multiplication of block+ras size: %d"%(self.NAND_BLOCK_SZ_PLUS_RAS)
			return -1

		# validate that the the user didn't want to read from incorrect place in the file
		if block_offset + nblocks > datasize/self.NAND_BLOCK_SZ_PLUS_RAS:
			print "Error: file is %x bytes long and last block is at %x"%(datasize, (block_offset + nblocks + 1) * self.NAND_BLOCK_SZ_PLUS_RAS)
			return -1
		
		# validate that the the user didn't want to write to incorrect place in the NAND
		if block_offset + nblocks > self.NAND_NBLOCKS:
			print "Error: nand has %x blocks. writing outside the nand's capacity"%(self.NAND_NBLOCKS, block_offset + nblocks + 1)
			return -1
		
		block = 0

		print "Writing %x blocks to device (starting at offset %x)..."%(nblocks, block_offset)
		
		while block < nblocks:
			pgblock = block+block_offset
			self.program_block(data[pgblock*self.NAND_BLOCK_SZ_PLUS_RAS:(pgblock+1)*self.NAND_BLOCK_SZ_PLUS_RAS], pgblock, verify)
			print "\r%d KB / %d KB"%(((block+1)*self.NAND_BLOCK_SZ_PLUS_RAS)/1024, (nblocks*self.NAND_BLOCK_SZ_PLUS_RAS)/1024),
			sys.stdout.flush()

			block += 1

		print
		
def validate_page_ex(is_xbox, data, pagenr, page_sz, pages_per_block):
	spare = data[page_sz:]
	pgblock = pagenr / pages_per_block
	pgoff = pagenr % pages_per_block
	
	nand_bl = (ord(spare[1])<<8) | ord(spare[0])
	if is_xbox and (nand_bl != 0xFFFF) and (nand_bl != pgblock) and (pgoff == 0):
		print
		if (pgblock < 0x3D0):
			return 0
	elif (is_xbox == 0) and (pgoff==0):
		if ord(spare[0]) == 0:
			return 0
		
	return 1

def ps3_validate_block(block_data, page_plus_ras_sz, page_sz):
	spare1 = block_data[page_sz:page_plus_ras_sz]
	spare2 = block_data[page_plus_ras_sz+page_sz:page_plus_ras_sz*2]
	
	if ord(spare1[0]) == 0 or ord(spare2[0]) == 0:
		return 0
		
	return 1
		

if __name__ == "__main__":
	print "NANDWay v0.2 beta - Teensy++ 2.0 NAND flasher for PS3 (and Xbox 360)"
	print "(Orignal NORWay.py by judges <judges@eEcho.com>)"
	print "(Orignal noralizer.py by Hector Martin \"marcan\" <hector@marcansoft.com>)"
	print

	if len(sys.argv) == 1:
		print "Usage:"
		print "NANDWay.py Serial-Port 0/1 Command"
		print
		print "  Serial-Port  Name of serial port to open (eg. COM1, COM2, /dev/ttyACM0, etc)"
		print "  0/1  NAND id number: 0-NAND0, 1-NAND1"
		print "  Commands:"
		print "  *  dump Filename [Offset] [Length]"
		print "     dumps to Filename at [Offset] and [Length] "
		print "  *  vwrite/write Filename [Offset] [Length]"
		print "     Flashes (v=verify) Filename at [Offset] and [Length]"
		print "  *  vdiffwrite/diffwrite  Filename Diff-file"
		print "     Flashes (v=verify) Filename using a Diff-file"
		print "  *  release"
		print "     Releases TRISTATE, so that the PS3 can boot"
		print "  *  bootloader"
		print "     Enters Teensy's bootloader mode (for Teensy reprogramming)"
		print "     Notes: 1) All offsets and lengths are in hex"
		print "            2) The Diff-file is a file which lists all the changed"
		print "               offsets of a dump file. This should increase flashing"
		print "               time dramatically."
		print
		print "Examples:"
		print
		print "  NANDWay.py COM1"
		print "  NANDWay.py COM1 0 dump d:\myflash.bin"
		print "  NANDWay.py COM1 1 dump d:\myflash.bin 3D a0"
		print "  NANDWay.py COM1 0 write d:\myflash.bin"
		print "  NANDWay.py COM3 1 write d:\myflash.bin"
		print "  NANDWay.py COM3 1 vwrite d:\myflash.bin 8D A0000"
		print "  NANDWay.py COM4 0 diffwrite d:\myflash.bin d:\myflash_diff"
		print "  NANDWay.py COM3 1 vdiffwrite d:\myflash.bin d:\myflash_diff"
		print "  NANDWay.py COM1 0 release"
		sys.exit(0)


	if (len(sys.argv) == 3) and (sys.argv[1] == "badblocks"):

		tStart = time.time()

		data = open(sys.argv[2],"rb").read()

		datasize = len(data)
		page_sz = 2048
		page_plus_ras_sz = 2112
		nblocks = 1024
		pages_per_block = 64
		block = 0
		block_plus_ras_sz=page_plus_ras_sz*pages_per_block
		block_offset=0
		
		tStart = time.time()
		
		while block < nblocks:
			pgblock = block+block_offset

			block_data=data[pgblock*(block_plus_ras_sz):(pgblock+1)*(block_plus_ras_sz)]
			block_valid = ps3_validate_block(block_data, page_plus_ras_sz, page_sz)
			if block_valid == 0:
				print
				print "Invalid block: %X"%(pgblock)
				
			print "\r%d KB / %d KB"%(((block+1)*(block_plus_ras_sz))/1024, (nblocks*(block_plus_ras_sz))/1024),
			sys.stdout.flush()

			block += 1

		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		sys.exit(0)
		
	if (len(sys.argv) == 5) and (sys.argv[1] == "remapbadblocks"):

		tStart = time.time()

		data = open(sys.argv[2],"rb").read()
		data_tgt = open(sys.argv[3],"rb").read()
		fo = open(sys.argv[4],"wb")

		datasize = len(data)
		page_sz = 2048
		page_plus_ras_sz = 2112
		nblocks = 1024
		pages_per_block = 64
		block = 0
		block_offset=0
		block_plus_ras_sz=page_plus_ras_sz*pages_per_block
		tgt_pgblock=0
		
		tStart = time.time()
		
		while block < nblocks:
			pgblock = block+block_offset

			block_data=data[pgblock*(block_plus_ras_sz):(pgblock+1)*(block_plus_ras_sz)]
			block_valid = ps3_validate_block(block_data, page_plus_ras_sz, page_sz)
			if block_valid == 0:
				print
				print "Invalid Src block: %X"%(pgblock)
			else:
				tgt_block_is_valid = 0
				while tgt_block_is_valid == 0:
					block_data_tgt=data_tgt[tgt_pgblock*(block_plus_ras_sz):(tgt_pgblock+1)*(block_plus_ras_sz)]
					tgt_block_is_valid=ps3_validate_block(block_data_tgt, page_plus_ras_sz, page_sz)
					if tgt_block_is_valid==0:
						print
						print "Invalid Tgt block: %X. Remapping to next index..."%(tgt_pgblock)
					#	zero_data = bytearray()
					#	for v in range(0, block_plus_ras_sz):
					#		zero_data.append('\0')
						fo.write(block_data_tgt)
					tgt_pgblock += 1
				
				fo.write(block_data)
				
			print "\r%d KB / %d KB"%(((block+1)*(block_plus_ras_sz))/1024, (nblocks*(block_plus_ras_sz))/1024),
			sys.stdout.flush()

			block += 1
			# 99627264 / 135168
			# 99618816 - 2e1

		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		sys.exit(0)

	
	
	n = NANDFlasher(sys.argv[1], int(sys.argv[2], 10))
	print "Pinging Nand%d..."%(int(sys.argv[2], 10))
	n.ping()
	
	tStart = time.time()
	if len(sys.argv) in (5,6,7) and sys.argv[3] == "dump":
		n.printstate()
		print
		print "Dumping...",
		sys.stdout.flush()
		print
		
		block_offset=0
		nblocks=0

		if len(sys.argv) == 6:
			block_offset=int(sys.argv[5],16)
		elif len(sys.argv) == 7:
			block_offset=int(sys.argv[5],16)
			nblocks=int(sys.argv[6],16)

		n.dump(sys.argv[4], block_offset, nblocks)
		
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
			
	if len(sys.argv) == 4 and sys.argv[3] == "info":
		n.printstate()
		print
			
	elif len(sys.argv) in (5,6,7) and (sys.argv[3] == "write" or sys.argv[3] == "vwrite"):
		n.printstate()
		print
		
		print "Writing...",
		sys.stdout.flush()

		print
		
		data = open(sys.argv[4],"rb").read()

		block_offset=0
		nblocks=0
		verify=0

		if (sys.argv[3] == "vwrite"):
			verify=1
		
		if len(sys.argv) == 6:
			block_offset=int(sys.argv[5],16)
		elif len(sys.argv) == 7:
			block_offset=int(sys.argv[5],16)
			nblocks=int(sys.argv[6],16)

		n.program(data, verify, block_offset, nblocks)
			
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		
	elif len(sys.argv) == 6 and (sys.argv[3] == "diffwrite" or sys.argv[3] == "vdiffwrite"):
		n.printstate()
		print
		
		print "Writing using diff file ..."
		
		sys.stdout.flush()

		print
		
		data = open(sys.argv[4],"rb").read()
		diff_data = open(sys.argv[5],"rb").readlines()

		block_offset=0
		nblocks=0
		verify=0
		nlines=len(diff_data)
		cur_line=0

		if (sys.argv[3] == "vdiffwrite"):
			verify=1
		
		for line in diff_data:
			addr=int(line[2:], 16)
			if addr % n.NAND_BLOCK_SZ_PLUS_RAS:
				print "Error: incorrect address for block addr=%x. addresses must be on a per-block boundary"%(addr)
				sys.exit(0)

			block_offset=addr/n.NAND_BLOCK_SZ_PLUS_RAS
			print "Programming offset %x block %x (%d/%d)"%(addr, block_offset, cur_line+1, nlines)
			n.program(data, verify, block_offset, 1)
			cur_line += 1
			
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		
	elif len(sys.argv) == 4 and sys.argv[3] == "release":
		print
		#n.trist = 0
	#	n.setports(0)
		print "NAND Released"
	elif len(sys.argv) == 4 and sys.argv[3] == "bootloader":
		print
		print "Entering Teensy's bootloader mode... Goodbye!"
		n.bootloader()
	#	n.closedevice()
		sys.exit(0)

	n.ping()
	#n.closedevice()
