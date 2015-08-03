#!/usr/bin/python
# *************************************************************************
#  NORway.py v0.7
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
			self.close()
			raise NORError("Ping failed (expected 0x42, got 0x%02x)"%val)
		val = self.readbyte()
		if val != 0xbd:
			self.close()
			raise NORError("Ping failed (expected 0xbd, got 0x%02x)"%val)

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
			self.close()
			sys.exit(1)
		if (self.DEVICE_ID == 0):
			print
			print "Unknown device id! Exiting..."
			self.close()
			sys.exit(1)
		if (self.DEVICE_PROTECTED == 1):
			print
			print "Device has protected sectors! Command not supported! Exiting..."
			self.close()
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
		#print "{0:15} {1}".format("STATUS_TRIST_N:", bool(state & STATUS_TRIST_N))
		#print "{0:15} {1}".format("STATUS_RESET_N:", bool(state & STATUS_RESET_N))
		#print "{0:15} {1}".format("STATUS_READY:", bool(state & STATUS_READY))
		#print "{0:15} {1}".format("STATUS_CE_N:", bool(state & STATUS_CE_N))
		#print "{0:15} {1}".format("STATUS_WE_N:", bool(state & STATUS_WE_N))
		#print "{0:15} {1}".format("STATUS_OE_N:", bool(state & STATUS_OE_N))
		print "{0:15} {1}".format("STATUS_TRIST_N:", "HIGH" if (state & STATUS_TRIST_N) else "LOW")
		print "{0:15} {1}".format("STATUS_RESET_N:", "HIGH" if (state & STATUS_RESET_N) else "LOW")
		print "{0:15} {1}".format("STATUS_READY:", "HIGH" if (state & STATUS_READY) else "LOW")
		print "{0:15} {1}".format("STATUS_CE_N:", "HIGH" if (state & STATUS_CE_N) else "LOW")
		print "{0:15} {1}".format("STATUS_WE_N:", "HIGH" if (state & STATUS_WE_N) else "LOW")
		print "{0:15} {1}".format("STATUS_OE_N:", "HIGH" if (state & STATUS_OE_N) else "LOW")

	#def _s_trist(self, v):
	#	self.write(0x06 | bool(v))
	#def _g_trist(self):
	#	return not (self.state() & STATUS_TRIST_N)
	#trist = property(_g_trist, _s_trist)

	def _s_reset(self, v):
		self.write(0x08 | bool(v))
	def _g_reset(self):
		return not (self.state() & STATUS_RESET_N)
	reset = property(_g_reset, _s_reset)

	# (0x0c | 0)=releaseports
	# (0x0c | 1)=initports
	def setports(self, v):
		self.write(0x06 | bool(v))

	def addr(self, v):
		assert 0 <= v <= 0x7FFFFF
		self.write((0x80 | (v >> 16), (v >> 8) & 0xff, v & 0xff))

	def wait(self, inc):
		self.write(0x0e | bool(inc))

	def writeword(self, data, inc):
		self.write((0x18 | bool(inc), (data>>8) & 0xff, data & 0xff))

	def writeat(self, addr, data):
		self.addr(addr)
		self.writeword(data, False)

	def bootloader(self):
		self.write("\x04")
		self.flush()

	def readsector(self, off, blocksize):
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
		self.wait(False)
		self.ping()

	def erasechip(self):
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x80)
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x10)
		self.delay(10)
		self.wait(False)
		self.ping()

		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601):
			self.writeat(0x555+0x400000, 0xaa)
			self.writeat(0x2aa+0x400000, 0x55)
			self.writeat(0x555+0x400000, 0x80)
			self.writeat(0x555+0x400000, 0xaa)
			self.writeat(0x2aa+0x400000, 0x55)
			self.writeat(0x555+0x400000, 0x10)
			self.delay(10)
			self.wait(False)
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
		self.wait(False)

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
		self.wait(False)
		self.writeat(off, data[0])
		self.wait(False)

	def program(self, addr, data, wordmode, ubm, noverify):
		ssize = self.getsectorsize(addr*2)
		assert len(data) == ssize
		#assert (addr & 0xffff) == 0
		assert (addr & (ssize/2-1)) == 0

		use2nddie = 0
		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601) and (addr & 0x400000):
			use2nddie = 1

		if (wordmode == True):
			if (noverify == False):
				self.write("\x0D")
			else:
				self.write("\x0C")

			odata = self.readsector(addr, ssize)
			if odata == data:
				return
				
			if odata != "\xff"*ssize:
				self.erasesector(addr)

			# 4KB blocks
			for block in range(0,ssize,0x1000):
				retries = self.RETRY_COUNT
				while retries != 0:
					self.addr(addr+(block/2))
					self.write(0x1A | use2nddie)
					self.write(data[block:block+0x1000])

					# read write status byte
					res = self.readbyte()
					# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout, 'V' = Verification error
					error_msg = ""
					if (res != 75): #'K'
						if (res == 84): #'T'
							error_msg = "RY/BY timeout error while writing!"
						elif (res == 82): #'R'
							self.close()
							raise NORError("Teensy receive buffer timeout! Disconnect and reconnect Teensy!")
						elif (res == 86): #'V'
							error_msg = "Verification error!"
						else:
							self.close()
							raise NORError("Received unknown error! (Got 0x%02x)"%val)

						self.reset = 1
						self.udelay(40)
						self.reset = 0
						self.udelay(40)
						self.ping()

					retries -= 1

					if (res == 86):
						print "(%d. Retry) %s"%(self.RETRY_COUNT-retries, error_msg)
					else:
						break

				if retries == 0:
					self.close()
					raise NORError("Verification failed")
		elif (ubm == True):
			if (noverify == False):
				self.write("\x0D")
			else:
				self.write("\x0C")

			odata = self.readsector(addr, ssize)
			if odata == data:
				return
				
			if odata != "\xff"*ssize:
				self.erasesector(addr)

			# 4KB blocks
			for block in range(0,ssize,0x1000):
				retries = self.RETRY_COUNT
				while retries != 0:
					# enter unlock bypass mode
					self.addr(addr+(block/2))
					self.write(0x1C | use2nddie)
					self.write(data[block:block+0x1000])

					# read write status byte
					res = self.readbyte()
					# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout, 'V' = Verification error
					error_msg = ""
					if (res != 75): #'K'
						if (res == 84): #'T'
							error_msg = "RY/BY timeout error while writing!"
						elif (res == 82): #'R'
							self.close()
							raise NORError("Teensy receive buffer timeout! Disconnect and reconnect Teensy!")
						elif (res == 86): #'V'
							error_msg = "Verification error!"
						else:
							self.close()
							raise NORError("Received unknown error! (Got 0x%02x)"%val)

						self.reset = 1
						self.udelay(40)
						self.reset = 0
						self.udelay(40)
						self.ping()

					retries -= 1

					if (res == 86):
						print "(%d. Retry) %s"%(self.RETRY_COUNT-retries, error_msg)
					else:
						break

				if retries == 0:
					self.close()
					raise NORError("Verification failed")	
		else:
			odata = self.readsector(addr, ssize)
			if odata == data:
				return
				
			if odata != "\xff"*ssize:
				self.erasesector(addr)

			# 4KB blocks
			for block in range(0,ssize,0x1000):
				retries = self.RETRY_COUNT
				while retries != 0:
					self.addr(addr+(block/2))
					self.write("\x1E")
					self.write(data[block:block+0x1000])

					# read write status byte
					res = self.readbyte()
					# 'K' = okay, 'T' = timeout error when writing, 'R' = Teensy receive buffer timeout, 'V' = Verification error
					error_msg = ""
					if (res != 75): #'K'
						if (res == 84): #'T'
							error_msg = "RY/BY timeout error while writing!"
						elif (res == 82): #'R'
							self.close()
							raise NORError("Teensy receive buffer timeout! Disconnect and reconnect Teensy!")
						elif (res == 86): #'V'
							error_msg = "Verification error!"
						else:
							self.close()
							raise NORError("Received unknown error! (Got 0x%02x)"%val)

						self.reset = 1
						self.udelay(40)
						self.reset = 0
						self.udelay(40)
						self.ping()

					retries -= 1

					if (res == 86):
						print "(%d. Retry) %s"%(self.RETRY_COUNT-retries, error_msg)
					else:
						break

				if retries == 0:
					self.close()
					raise NORError("Verification failed")

	def writerange(self, addr, data, wordmode, ubm, noverify):
		if len(data) == 0:
			return

		datasize = len(data)
		start = addr

		if (noverify == False):
			self.write("\x0D")
		else:
			self.write("\x0C")

		print "Writing..."
		ssize = self.getsectorsize(addr)
		while len(data) >= ssize:
			print "\r%d KB / %d KB"%((addr-start)/1024, datasize/1024),
			sys.stdout.flush()
			self.program(addr/2, data[:ssize], wordmode, ubm, noverify)
			addr += ssize
			data = data[ssize:]
			ssize = self.getsectorsize(addr)
			
		print "\r%d KB / %d KB"%((addr-start)/1024, datasize/1024),
		sys.stdout.flush()
		print

	def verify(self, addr, data):
		BLOCK = 0x10000
		start = addr
		print
		print "Verifying..."
		for offset in range(0, len(data)/2, BLOCK):
			addr = start+offset*2
			nordata = self.readsector(addr/2, 0x20000)
			print "\r%d KB / %d KB"%((offset+BLOCK)/512, len(data)/1024),
			sys.stdout.flush()
			if (nordata == data[offset*2:offset*2+BLOCK*2]):
				continue
			else:
				print
				print "Verification failed! Please repeat command [%s]!"%self.getargs()
				self.close()
				sys.exit(1)
		print
	
	def getsectorsize(self, addr):
		s8kb = 0x2000
		s64kb = 0x10000
		s128kb = 0x20000

		#sector/block sizes for Samsung K8Q2815UQB
		if (self.MF_ID == 0xEC) and (self.DEVICE_ID == 0x7e0601):
			if (addr < 0x8000*2):
				return s8kb
			elif (addr >= 0x8000*2) and (addr < 0x3f8000*2):
				return s64kb
			elif (addr >= 0x3f8000*2) and (addr < 0x408000*2):
				return s8kb
			elif (addr >= 0x408000*2) and (addr < 0x7f8000*2):
				return s64kb
			else:
				return s8kb
		
		#sector/block size for all other NORs
		return s128kb

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

	def closedevice(self):
		self.close()
		
	def getargs(self):
		args = ""
		for arg in sys.argv:
			if " " in arg:
				args = args + '"' + arg + '"' + " "
			else:
				args = args + arg + " "
		if len(args) > 0:
			args = args[0:len(args)-1]
		return args

if __name__ == "__main__":
	print "NORway.py v0.7 - Teensy++ 2.0 NOR flasher for PS3 (judges@eEcho.com)"
	print "(Original noralizer.py by Hector Martin \"marcan\" <hector@marcansoft.com>)"
	print

	if len(sys.argv) == 1:
		print "Usage:"
		print "%s serialport [command] [filename] [address]"%sys.argv[0]
		print
		print "  serialport  Name of serial port to open (eg. COM1, COM2, /dev/ttyACM0, etc)"
		print "  command  dump          Reads entire NOR to [filename]"
		print "           erase         Erases one sector/block (128KB/64KB/8KB) at [address]"
		print "           erasechip     Erases entire NOR"
		print "           write         Flashes (read-erase-modify-write) [filename]"
		print "                         at [address] to NOR (buffered programming mode)"
		print "           writeword     Flashes (read-erase-modify-write) [filename]"
		print "                         at [address] to NOR (word programming mode)"
		print "           writewordubm  Flashes (read-erase-modify-write) [filename]"
		print "                         at [address] to NOR (word prgrmmng/unlock bypass mode)"
		print "           vwrite        Flashes (read-erase-modify-write-verify) [filename]"
		print "                         at [address] to NOR (buffered programming mode)"
		print "           vwriteword    Flashes (read-erase-modify-write-verify) [filename]"
		print "                         at [address] to NOR (word programming mode)"
		print "           vwritewordubm Flashes (read-erase-modify-write-verify) [filename]"
		print "                         at [address] to NOR (word prgrmmng/unlock bypass mode)"
		print "           verify        Verifies NOR content with [filename] at [address]"
		print "           release       Releases NOR interface, so the PS3 can boot"
		print "           bootloader    Enters Teensy's bootloader mode"
		print "  filename Filename for [dump|(v)write|(v)writeword|(v)writewordubm|verify]"
		print "  address  Address for [erase|(v)write|(v)writeword|(v)writewordubm|verify]"
		print "           Default is 0x0, address must be aligned (multiple of 0x20000)"
		print
		print "Examples:"
		print "  %s COM1"%sys.argv[0]
		print "  %s COM1 dump d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 erase 0x20000"%sys.argv[0]
		print "  %s COM1 erasechip"%sys.argv[0]
		print "  %s COM1 write d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 write d:\myflash.bin 0xA0000"%sys.argv[0]
		print "  %s COM1 writeword d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 writewordubm d:\myflash.bin 0x40000"%sys.argv[0]
		print "  %s COM1 verify d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 release"%sys.argv[0]
		sys.exit(0)
	
	n = NORFlasher(sys.argv[1])
	print "Pinging..."
	n.ping()

	print "Set SB to tristate"
	print
	n.setports(1)
	n.reset = 0
	#n.trist = 1
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
			fo.write(n.readsector(offset, 0x20000))
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
			n.closedevice()
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
	elif len(sys.argv) in (4,5) and (sys.argv[2] == "write" or sys.argv[2] == "vwrite"):
		n.checkchip()
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		if (n.MF_ID == 0xEC) and (n.DEVICE_ID == 0x7e0601):
			print "Buffered programming mode not supported for Samsung K8Q2815UQB!"
			print "Programming in unlock bypass mode (writewordubm)..."
			if sys.argv[2] == "write":
				n.writerange(addr, data, False, True, True)
			else:
				n.writerange(addr, data, False, True, False)
		else:
			if sys.argv[2] == "write":
				n.writerange(addr, data, False, False, True)
			else:
				n.writerange(addr, data, False, False, False)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		tStart = time.time()
		n.verify(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and (sys.argv[2] == "writeword" or sys.argv[2] == "vwriteword"):
		n.checkchip()
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		if sys.argv[2] == "writeword":
			n.writerange(addr, data, True, False, True)
		else:
			n.writerange(addr, data, True, False, False)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		tStart = time.time()
		n.verify(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and (sys.argv[2] == "writewordubm" or sys.argv[2] == "vwritewordubm"):
		n.checkchip()
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		if sys.argv[2] == "writewordubm":
			n.writerange(addr, data, False, True, True)
		else:
			n.writerange(addr, data, False, True, False)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
		tStart = time.time()
		n.verify(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and sys.argv[2] == "verify":
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		n.verify(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 3 and sys.argv[2] == "release":
		print
		#n.trist = 0
		n.setports(0)
		print "NOR Released"
	elif len(sys.argv) == 3 and sys.argv[2] == "bootloader":
		print
		print "Entering Teensy's bootloader mode... Goodbye!"
		n.bootloader()
		n.closedevice()
		sys.exit(0)
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

	n.ping()
	n.closedevice()
