#!/usr/bin/python
# *************************************************************************
#  NORway.py v0.3
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
		self.ser = serial.Serial(port, 9600, timeout=5, rtscts=0, dsrdtr=1, xonxoff=0)
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

STATUS_DRIVE = 0x40
STATUS_TRIST_N = 0x20
STATUS_RESET_N = 0x10
STATUS_READY = 0x08
STATUS_CE_N = 0x04
STATUS_WE_N = 0x02
STATUS_OE_N = 0x01

class NORFlasher(TeensySerial):
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

	def printstate(self):
		state = self.state()
		print "{0:15} {1}".format("STATUS_DRIVE:", bool(state & STATUS_DRIVE))
		print "{0:15} {1}".format("STATUS_TRIST_N:", bool(state & STATUS_TRIST_N))
		print "{0:15} {1}".format("STATUS_RESET_N:", bool(state & STATUS_RESET_N))
		print "{0:15} {1}".format("STATUS_READY:", bool(state & STATUS_READY))
		print "{0:15} {1}".format("STATUS_CE_N:", bool(state & STATUS_CE_N))
		print "{0:15} {1}".format("STATUS_WE_N:", bool(state & STATUS_WE_N))
		print "{0:15} {1}".format("STATUS_OE_N:", bool(state & STATUS_OE_N))

	def _s_drive(self, v):
		self.write(0x04 | bool(v))
	def _g_drive(self):
		return bool(self.state() & STATUS_DRIVE)
	drive = property(_g_drive, _s_drive)

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

	def readsector(self, off):
		BLOCK = 0x10000
		assert (off & (BLOCK-1)) == 0
		self.addr(off)
		self.write("\x13")
		d = self.read(2*BLOCK)
		return d

	def erasesector(self, off):
		assert off&0xFFFF == 0
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(0x555, 0x80)
		self.writeat(0x555, 0xaa)
		self.writeat(0x2aa, 0x55)
		self.writeat(off, 0x30)
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

	def writesector(self, addr, data):
		assert len(data) == 0x20000
		assert (addr & 0xffff) == 0
		odata = self.readsector(addr)
		if odata == data:
			return
		self.erasesector(addr)
		for off in range(0,0x20000,0x40):
			d = data[off:off+0x40]
			if d != "\xff"*0x40:
				self.programline(addr+(off/2), d)
		rdata = self.readsector(addr)
		if rdata != data:
			raise NORError("Verification failed")

	def rmwsector(self, addr, data):
		offset = addr & 0x1ffff
		endaddr = offset + len(data)
		assert endaddr <= 0x20000
		secaddr = (addr & ~0x1ffff)/2
		odata = self.readsector(secaddr)
		wdata = odata[:offset] + data + odata[endaddr:]
		if odata != wdata:
			self.writesector(secaddr, wdata)

	def writerange(self, addr, data):
		if len(data) == 0:
			return
		first_sector = addr & ~0x1ffff
		last_sector = (addr + len(data) - 1) & ~0x1ffff

		offset = addr & 0x1ffff

		sec_count = (last_sector + 0x20000 - first_sector)/0x20000
		done = 0
		if offset != 0:
			print "\rSector %06x (%d/%d) [F]..."%(first_sector, done+1, sec_count),
			sys.stdout.flush()
			self.rmwsector(addr, data[:0x20000-offset])
			data = data[0x20000-offset:]
			done += 1
			addr += 0x20000-offset

		while len(data) >= 0x20000:
			print "\rSector %06x (%d/%d) [M]..."%(addr, done+1, sec_count),
			sys.stdout.flush()
			self.writesector(addr/2, data[:0x20000])
			done += 1
			addr += 0x20000
			data = data[0x20000:]

		if len(data) != 0:
			print "\rSector %06x (%d/%d) [L]..."%(addr, done+1, sec_count),
			sys.stdout.flush()
			self.rmwsector(addr, data)
			done += 1

		assert done == sec_count
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
	if len(sys.argv) == 1:
		print "NORway.py v0.3 - Teensy++ 2.0 NOR flasher for PS3 (judges@eEcho.com)"
		print "(Orignal noralizer.py by Hector Martin \"marcan\" <hector@marcansoft.com>)"
		print
		print "Usage:"
		print "%s serialport [command] [filename] [address]"%sys.argv[0]
		print
		print "  serialport  Name of serial port to open (eg. COM1, COM2, /dev/ttyACM0, etc)"
		print "  command     dump       Reads entire NOR to [filename]"
		print "              erase      Erases one sector (128KB) at [address]"
		print "              write      Flashes (read-erase-modify-write-verify) [filename]"
		print "                         at [address] to NOR"
		print "              writeimg   Same as write, but prepend a 16-byte length header"
		print "                         [address] is required"
		print "              program    Flashes (erase-write-verify) [filename]"
		print "                         at [address] to NOR"
		print "              release    Releases NOR interface, so the PS3 can boot"
		print "  filename    Filename for [dump|write|writeimg|program]"
		print "  address     Address for [erase|write|writeimg|program]"
		print "              Default is 0x0, address must be aligned (multiple of 0x20000)"
		print
		print "Examples:"
		print "  %s COM1"%sys.argv[0]
		print "  %s COM1 dump d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 erase 0x20000"%sys.argv[0]
		print "  %s COM1 write d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 write d:\myflash.bin 0xA0000"%sys.argv[0]
		print "  %s COM1 program d:\myflash.bin"%sys.argv[0]
		print "  %s COM1 program d:\myflash.bin 0x40000"%sys.argv[0]
		print "  %s COM1 release"%sys.argv[0]
		sys.exit(0)
	
	n = NORFlasher(sys.argv[1])
	print "Pinging..."
	n.ping()

	n.drive = 0
	n.reset = 0
	print "Set SB to tristate"
	print
	n.trist = 1
	n.printstate()
	n.drive = 1
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
			print "\r%d KB"%((offset+BLOCK)/512),
			sys.stdout.flush()
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 4 and sys.argv[2] == "erase":
		print
		addr = int(sys.argv[3], 16)
		if addr & 0x1ffff:
			print "Address must be aligned!"
			sys.exit(1)
		assert addr&1 == 0
		print "Erasing sector %06x..."%addr,
		sys.stdout.flush()
		n.erasesector(addr/2)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and sys.argv[2] == "write":
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
		n.writerange(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 5 and sys.argv[2] == "writeimg":
		print
		data = open(sys.argv[3],"rb").read()
		addr = int(sys.argv[4],16)
		data = struct.pack(">12xI", len(data)) + data
		n.writerange(addr, data)
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) in (4,5) and sys.argv[2] == "program":
		print
		data = open(sys.argv[3],"rb").read()
		addr = 0
		if len(sys.argv) == 5:
			addr = int(sys.argv[4],16)
			if addr & 0x1ffff:
				print "Address must be aligned!"
				sys.exit(1)
			addr /= 2
		sectors = (len(data)+0x1ffff) / 0x20000
		if (sectors*0x20000) != len(data):
			left = len(data)%0x20000
			print "NOTE: padding file with 0x%x FF bytes to complete the sector"%(0x20000 - left)
		if len(data) & 1: # pad to 16 bits
			data += "\xff"

		for sec in range(sectors):
			secaddr = addr + sec * 0x10000
			print "\rSector %06x (%d/%d) E"%(secaddr, sec+1, sectors),
			sys.stdout.flush()
			n.erasesector(secaddr)

			print "\rSector %06x (%d/%d) P"%(secaddr, sec+1, sectors),
			sys.stdout.flush()
			d = data[sec*0x20000:sec*0x20000+0x20000]
			for off in range(0,len(d),0x40):
				n.programline(secaddr+(off/2), d[off:off+0x40])
			print "\rSector %06x (%d/%d) V"%(secaddr, sec+1, sectors),
			sys.stdout.flush()
			dv = n.readsector(secaddr)[:len(d)]
			if d != dv:
				print
				print "Verification failed!"
				sys.exit(1)
		print
		print "Done. [%s]"%(datetime.timedelta(seconds=time.time() - tStart))
	elif len(sys.argv) == 3 and sys.argv[2] == "release":
		print
		n.drive = 0
		n.trist = 0
		print "NOR Released"

	n.ping()
