#!/usr/bin/python
# *************************************************************************
#  NORway.py v0.5 beta
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
		self.ser = serial.Serial(port, 9600, timeout=300, rtscts=0, dsrdtr=1, xonxoff=0, writeTimeout=120)
		if self.ser is None:
			raise TeensySerialError("could not open serial %s")%port
		self.ser.setDTR()
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
			self.obuf = ""
		self.ser.flush()

	def read(self, size):
		self.flush()
		data = self.ser.read(size)
		return data

	def readbyte(self):
		return ord(self.read(1))

class NORError(Exception):
	pass

STATUS_TRIST_N = 0x20
STATUS_RESET_N = 0x10
STATUS_READY = 0x08
STATUS_CE_N = 0x04
STATUS_WE_N = 0x02
STATUS_OE_N = 0x01

class NORFlasher(TeensySerial):
	MF_ID = 0
	DEVICE_ID = 0
	DEVICE_PROTECTED = 0
	RETRY_COUNT = 20

	def __init__(self, port):
		TeensySerial.__init__(self, port)

	def ping(self):
		self.write(0x02)
		self.write(0x03)
		val = self.readbyte()
		if val != 0x42:
			raise NORError("Ping failed (expected 42, got %02x)"%val)
		val = self.readbyte()
		if val != 0xbd:
			raise NORError("Ping failed (expected bd, got %02x)"%val)

	def state(self):
		self.write(0x01)
		return self.readbyte()

	def manufacturer_id(self):
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x90)
		self.addr(0x0)
		self.delay(10)
		self.write("\x14")
		self.readbyte()
		val = self.readbyte()
		self.reset = 1
		self.udelay(40)
		self.reset = 0
		self.udelay(40)
		self.ping()
		return val

	def device_id(self):
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x90)

		self.addr(0x1)
		self.delay(10)
		self.write("\x14")
		self.readbyte()
		val = self.readbyte()

		self.addr(0x0e)
		self.delay(10)
		self.write("\x14")
		self.readbyte()
		val = (val << 8) | self.readbyte()

		self.addr(0x0f)
		self.delay(10)
		self.write("\x14")
		self.readbyte()
		val = (val << 8) | self.readbyte()

		self.reset = 1
		self.udelay(40)
		self.reset = 0
		self.udelay(40)
		self.ping()
		return val

	def checkprotection(self):
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0xc0)

		print
		print "Checking sector protection..."
		for offset in range(0, 0x800000, 0x10000):
			self.addr(offset)
			self.delay(10)
			self.write("\x14")
			self.readbyte()
			val = self.readbyte()
			if ((val & 1) == 0):
				print "Sector at 0x%06x is protected!"%offset
				self.DEVICE_PROTECTED = 1

		if (self.DEVICE_PROTECTED == 0):
			print "No protected sectors found!"

		self.writeat(0x0, 0x90)
		self.writeat(0x0, 0x0)
		self.delay(10)

		self.reset = 1
		self.udelay(40)
		self.reset = 0
		self.udelay(40)
		self.ping()

	def checkchip(self):
		if (self.MF_ID == 0):
			print
			print "Unknown chip manufacturer! Exiting..."
			sys.exit(1)
		if (self.DEVICE_ID == 0):
			print
			print "Unknown device id! Exiting..."
			sys.exit(1)
		if (self.DEVICE_PROTECTED == 1):
			print
			print "Device has protected sectors! Command not supported! Exiting..."
			sys.exit(1)

	def printstate(self):
		state = self.state()
		self.MF_ID = self.manufacturer_id()
		self.DEVICE_ID = self.device_id()

		if self.MF_ID == 0x01:
			print "NOR chip manufacturer: Spansion (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0x7e2101:
				print "NOR chip type: S29GL128 (0x%06x)"%self.DEVICE_ID
			else:
				print "NOR chip type: unknown (0x%06x)"%self.DEVICE_ID
				self.DEVICE_ID = 0
		elif self.MF_ID == 0xEC:
			print "NOR chip manufacturer: Samsung (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0x7e6660:
				print "NOR chip type: K8P2716UZC (0x%06x)"%self.DEVICE_ID
			elif self.DEVICE_ID == 0x7e0601:
				print "NOR chip type: K8Q2815UQB (0x%06x)"%self.DEVICE_ID
			else:
				print "NOR chip type: unknown (0x%06x)"%self.DEVICE_ID
				self.DEVICE_ID = 0
		elif self.MF_ID == 0xC2:
			print "NOR chip manufacturer: Macronix (0x%02x)"%self.MF_ID
			if self.DEVICE_ID == 0x7e2101:
				print "NOR chip type: MX29GL128 (0x%06x)"%self.DEVICE_ID
			else:
				print "NOR chip type: unknown (0x%06x)"%self.DEVICE_ID
				self.DEVICE_ID = 0
		else:
			print "NOR chip manufacturer: unknown (0x%02x)"%self.MF_ID
			print "NOR chip type: unknown (0x%06x)"%self.DEVICE_ID
			self.MF_ID = 0
			self.DEVICE_ID = 0
		
		if (self.MF_ID != 0xEC) and (self.MF_ID != 0x0) and (self.DEVICE_ID != 0x7e0601) and (self.DEVICE_ID != 0):
			self.checkprotection()

		print
		print "{0:15} {1}".format("STATUS_TRIST_N:", bool(state & STATUS_TRIST_N))
		print "{0:15} {1}".format("STATUS_RESET_N:", bool(state & STATUS_RESET_N))
		print "{0:15} {1}".format("STATUS_READY:", bool(state & STATUS_READY))
		print "{0:15} {1}".format("STATUS_CE_N:", bool(state & STATUS_CE_N))
		print "{0:15} {1}".format("STATUS_WE_N:", bool(state & STATUS_WE_N))
		print "{0:15} {1}".format("STATUS_OE_N:", bool(state & STATUS_OE_N))

	def _s_trist(self, v):
		self.write(0x06 | bool(v))
	def _g_trist(self):
		return not (self.state() & STATUS_TRIST_N)
	trist = property(_g_trist, _s_trist)

	def _s_reset(self, v):
		self.write(0x08 | bool(v))
	def _g_reset(self):
		return not (self.state() & STATUS_RESET_N)
	reset = property(_g_reset, _s_reset)

	# (0x0c | 0)=releaseports
	# (0x0c | 1)=initports
	def setports(self, v):
		self.write(0x0c | bool(v))

	def addr(self, v):
		assert 0 <= v <= 0x7FFFFF
		self.write((0x80 | (v >> 16), (v >> 8) & 0xff, v & 0xff))

	def wait(self, inc=False):
		self.write(0x0e | bool(inc))

	def writeword(self, data, inc=False):
		self.write((0x18 | bool(inc), (data>>8) & 0xff, data & 0xff))

	def writeat(self, addr, data, inc=False):
		self.addr(addr)
		self.writeword(data, inc)

	def bootloader(self):
		self.write("\x04")
		self.flush()

	def readsector(self, off, blocksize=0x20000):
		assert (off & (blocksize/2-1)) == 0
		self.addr(off)

		if (blocksize == 0x1000):
			self.write("\x10")
		elif (blocksize == 0x2000):
			self.write("\x11")
		elif (blocksize == 0x10000):
			self.write("\x12")
		elif (blocksize == 0x20000):
			self.write("\x13")

		d = self.read(blocksize)
		return d

	def erasesector(self, off):
		offset_2nddie = 0
		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601):
			offset_2nddie = off & 0x400000

		self.writeat(0x555 + offset_2nddie, 0xaa)
		self.writeat(0x2aa + offset_2nddie, 0x55)
		self.writeat(0x555 + offset_2nddie, 0x80)
		self.writeat(0x555 + offset_2nddie, 0xaa)
		self.writeat(0x2aa + offset_2nddie, 0x55)
		self.writeat(off, 0x30)
		self.delay(10)
		self.wait()
		self.ping()

	def erasechip(self):
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x80)
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x10)
		self.delay(10)
		self.wait()
		self.ping()

		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601):
			self.writeat(0x555+0x400000, 0xaa)
			self.writeat(0x2aa+0x400000, 0x55)
			self.writeat(0x555+0x400000, 0x80)
			self.writeat(0x555+0x400000, 0xaa)
			self.writeat(0x2aa+0x400000, 0x55)
			self.writeat(0x555+0x400000, 0x10)
			self.delay(10)
			self.wait()
			self.ping()

	def programline(self, off, data):
		assert off&0x1f == 0
		if isinstance(data, str):
			data = struct.unpack(">%dH"%(len(data)/2), data)
		assert len(data) <= 32
		saddr = off & ~0x1f
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(saddr, 0x25)
		self.writeat(saddr, len(data)-1)
		self.addr(off)
		for d in data:
			self.writeword(d, True)
		self.writeat(saddr, 0x29)
		self.wait()

	def programword(self, off, data):
		if isinstance(data, str):
			data = struct.unpack(">%dH"%(len(data)/2), data)
		assert len(data) <= 2

		offset_2nddie = 0
		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601):
			offset_2nddie = addr & 0x400000

		self.writeat(0x555 + offset_2nddie, 0xaa)
		self.writeat(0x2aa + offset_2nddie, 0x55)
		self.writeat(0x555 + offset_2nddie, 0xa0)
		self.wait()
		self.writeat(off, data[0])
		self.wait()

	def program(self, addr, data, wordmode=False, ubm=False):
		assert len(data) == 0x20000
		assert (addr & 0xffff) == 0

		use2nddie = 0
		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601) and (addr & 0x400000):
			use2nddie = 1

		if (wordmode == True):
			# 4KB blocks
			for block in range(0,0x20000,0x1000):
				odata = self.readsector(addr+(block/2), 0x1000)
				if (odata == data[block:block+0x1000]):
					continue

				retries = self.RETRY_COUNT
				while retries != 0:
					self.addr(addr+(block/2))
					self.write(0x1A | use2nddie)
					self.write(data[block:block+0x1000])

					# read write status byte
					res = self.readbyte()
					# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout
					if (res != 75):
						self.reset = 1
						self.udelay(40)
						self.reset = 0
						self.udelay(40)
						self.ping()

					retries -= 1

					rdata = self.readsector(addr+(block/2), 0x1000)
					if (rdata == data[block:block+0x1000]):
						break
					else:
						print "(%d. Retry)"%(self.RETRY_COUNT-retries)

				if retries == 0:
					raise NORError("Verification failed")
		elif (ubm == True):
			# 4KB blocks
			for block in range(0,0x20000,0x1000):
				odata = self.readsector(addr+(block/2), 0x1000)
				if (odata == data[block:block+0x1000]):
					continue

				retries = self.RETRY_COUNT
				while retries != 0:
					# enter unlock bypass mode
					self.addr(addr+(block/2))
					self.write(0x1C | use2nddie)
					self.write(data[block:block+0x1000])

					# read write status byte
					res = self.readbyte()
					# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout
					if (res != 75):
						self.reset = 1
						self.udelay(40)
						self.reset = 0
						self.udelay(40)
						self.ping()

					retries -= 1

					rdata = self.readsector(addr+(block/2), 0x1000)
					if (rdata == data[block:block+0x1000]):
						break
					else:
						print "(%d. Retry)"%(self.RETRY_COUNT-retries)

				if retries == 0:
					raise NORError("Verification failed")	
		else:
			odata = self.readsector(addr)
			if odata == data:
				return
				
			if odata != "\xff"*0x20000:
				self.erasesector(addr)

			# 4KB blocks
			for block in range(0,0x20000,0x1000):
				odata = self.readsector(addr+(block/2), 0x1000)
				if (odata == data[block:block+0x1000]):
					continue

				retries = self.RETRY_COUNT
				while retries != 0:
					self.addr(addr+(block/2))
					self.write("\x1E")
					self.write(data[block:block+0x1000])

					# read write status byte
					res = self.readbyte()
					# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout
					if (res != 75):
						self.reset = 1
						self.udelay(40)
						self.reset = 0
						self.udelay(40)
						self.ping()

					retries -= 1

					rdata = self.readsector(addr+(block/2), 0x1000)
					if (rdata == data[block:block+0x1000]):
						break
					else:
						print "(%d. Retry)"%(self.RETRY_COUNT-retries)

				if retries == 0:
					raise NORError("Verification failed")

	def writerange(self, addr, data, wordmode=False, ubm=False):
		if len(data) == 0:
			return

		datasize = len(data)
		start = addr

		print "Writing..."
		while len(data) >= 0x20000:
			print "\r%d KB / %d KB"%((addr-start)/1024, datasize/1024),
			sys.stdout.flush()
			self.program(addr/2, data[:0x20000], wordmode, ubm)
			addr += 0x20000
			data = data[0x20000:]
		print "\r%d KB / %d KB"%((addr-start)/1024, datasize/1024),
		sys.stdout.flush()
		print

	def speedtest_read(self):
		self.write(0x0C)
		d = self.read(0x20000)
		return d

	def speedtest_write_data(self, addr, data):
		assert len(data) == 0x20000
		assert (addr & 0xffff) == 0

		# 4KB blocks
		for block in range(0,0x20000,0x1000):
			self.write(0x0D)
			self.write(data[block:block+0x1000])

			# read write status byte
			res = self.readbyte()
			# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout
			if (res != 75):
				print "Error: %c"%res

	def speedtest_write(self, addr, data):
		if len(data) == 0:
			return

		datasize = len(data)
		start = addr

		while len(data) >= 0x20000:
			print "\r%d KB / %d KB"%((addr-start)/1024, datasize/1024),
			sys.stdout.flush()
			self.speedtest_write_data(addr/2, data[:0x20000])
			addr += 0x20000
			data = data[0x20000:]
		print "\r%d KB / %d KB"%((addr-start)/1024, datasize/1024),
		sys.stdout.flush()
		print

	def delay(self, v):
		while v > 0x41:
			self.write(0x7f)
			v -= 0x41
		if v <= 0:
			return
		elif v == 1:
			self.write(0x00)
		else:
			self.write(0x40 | (v-2))

	def udelay(self, v):
		self.delay(v * 60)

if __name__ == "__main__":
	print "NORway.py v0.5 beta - Teensy++ 2.0 NOR flasher for PS3 (judges@eEcho.com)"
	print "(Orignal noralizer.py by Hector Martin \"marcan\" <hector@marcansoft.com>)"
	print

	if len(sys.argv) == 1:
		print "Usage:"
		print "%s serialport [command] [filename] [address]"%sys.argv[0]
		print
		print "  serialport  Name of serial port to open (eg. COM1, COM2, /dev/ttyACM0, etc)"
		print "  command  dump          Reads entire NOR to [filename]"
		print "           erase         Erases one sector (128KB) at [address]"
		print "           erasechip     Erases entire NOR"
		print "           write         Flashes (read-erase-modify-write-verify) [filename]"
		print "                         at [address] to NOR (buffered programming mode)"
		print "           writeword     Flashes (write-verify) [filename]"
		print "                         at [address] to NOR (word programming mode)"
		print "           writewordubm  Flashes (write-verify) [filename]"
		print "                         at [address] to NOR (word prgrmming/unlock bypass mode)"
		print "           release       Releases NOR interface, so the PS3 can boot"
		print "           bootloader    Enters Teensy's bootloader mode"
		print "  filename Filename for [dump|write|writeword|writewordubm]"
		print "  address  Address for [erase|write|writeword|writewordubm]"
		print "           Default is 0x0, address must be aligned (multiple of 0x20000)"
		print
		print "Examples:"
		print "  %s COM1"%sys.argv[0]
		print "  %s COM1 dump d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 erase 0x20000"%sys.argv[0]
		print "  %s COM1 write d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 write d:\myflash.bin 0xA0000"%sys.argv[0]
		print "  %s COM1 writeword d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 writewordubm d:\myflash.bin 0x40000"%sys.argv[0]
		print "  %s COM1 release"%sys.argv[0]
		sys.exit(0)
	
	n = NORFlasher(sys.argv[1])
	print "Pinging..."
	n.ping()

	n.setports(1)
	n.reset = 0
	print "Set SB to tristate"
	print
	n.trist = 1
	n.printstate()
	print
	print "Resetting NOR..."
	n.reset = 1
	n.udelay(40)
	n.reset = 0
	n.udelay(40)
	n.ping()
	print "Ready."

	tStart = time.time()
	if len(sys.argv) == 4 and sys.argv[2] == "dump":
		BLOCK = 0x10000
		print
		print "Dumping NOR..."
		fo = open(sys.argv[3],"wb")
		for offset in range(0, 0x800000, BLOCK):
			fo.write(n.readsector(offset))
			print "\r%d KB / 16384 KB"%((offset+BLOCK)/512),
			sys.stdout.flush()
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 4 and sys.argv[2] == "erase":
		n.checkchip()
		print
		addr = int(sys.argv[3], 16)
		if addr & 0x1:
			print "Address must be even!"
			sys.exit(1)
		print "Erasing sector/block at address %06x..."%addr,
		sys.stdout.flush()
		n.erasesector(addr/2)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 3 and sys.argv[2] == "erasechip":
		n.checkchip()
		print
		print "Erasing chip, might take a while... (1-3 minutes)"
		n.erasechip()
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and sys.argv[2] == "write":
		n.checkchip()
		print
		if (n.MF_ID == 0xEC) and (n.DEVICE_ID == 0x7e0601):
			print "Command not supported for Samsung K8Q2815UQB"
			sys.exit(1)
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		n.writerange(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	#elif len(sys.argv) == 4 and sys.argv[2] == "speedtest_read":
	#	BLOCK = 0x10000
	#	print
	#	print "Measuring read performance..."
	#	fo = open(sys.argv[3],"wb")
	#	for offset in range(0, 0x800000, BLOCK):
	#		fo.write(n.speedtest_read())
	#		print "\r%d KB / 16384 KB"%((offset+BLOCK)/512),
	#		sys.stdout.flush()
	#	print
	#	print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	#elif len(sys.argv) in (4,5) and sys.argv[2] == "speedtest_write":
	#	print
	#	print "Measuring write performance..."
	#	data = open(sys.argv[3],"rb").read()
	#	addr = 0
	#	if len(sys.argv) == 5:
	#		addr = int(sys.argv[4],16)
	#	n.speedtest_write(addr, data)
	#	print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and sys.argv[2] == "writeword":
		n.checkchip()
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		n.writerange(addr, data, True)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and sys.argv[2] == "writewordubm":
		n.checkchip()
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		n.writerange(addr, data, False, True)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 3 and sys.argv[2] == "release":
		print
		n.trist = 0
		n.setports(0)
		print "NOR Released"
	elif len(sys.argv) == 3 and sys.argv[2] == "bootloader":
		print
		print "Entering Teensy's bootloader mode... Goodbye!"
		n.bootloader()
		sys.exit(0)

	n.ping()
