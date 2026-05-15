#!/usr/bin/python
# *************************************************************************
#  NORway.py v0.9
#
# Teensy++ 2.0 modifications by judges@eEcho.com
# Python 3 porting by MikeM64
# *************************************************************************
#  noralizer.py - NOR flasher for PS3
#
# Copyright (C) 2010-2011  Hector Martin "marcan" <hector@marcansoft.com>
#
# This code is licensed to you under the terms of the GNU GPL, version 2;
# see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
# *************************************************************************

import serial

class TeensySerialError(Exception):
	pass

class TeensySerial(object):
	BUFSIZE = 32768

	def __init__(self, port):
		self.ser = serial.Serial(port, 9600, timeout=300, rtscts=False, dsrdtr=False, xonxoff=False, write_timeout=120)
		if self.ser is None:
			raise TeensySerialError("could not open serial %s")%port
		self.ser.flushInput()
		self.ser.flushOutput()
		self.obuf = b""

	def write(self, s):
		if isinstance(s,int):
			# This will fail if s in an integer greater than 0xFF
			s = s.to_bytes()
		elif isinstance(s, (tuple, list)):
			s = bytes(s)

		self.obuf += s
		while len(self.obuf) > self.BUFSIZE:
			self.ser.write(self.obuf[:self.BUFSIZE])
			self.obuf = self.obuf[self.BUFSIZE:]

	def flush(self):
		if len(self.obuf):
			self.ser.write(self.obuf)
			self.ser.flush()
			self.obuf = b""

	def read(self, size):
		self.flush()
		data = self.ser.read(size)
		return data

	def readbyte(self):
		data = self.read(1)
		if data:
			return data[0]
		return 0

	def close(self):
		print("")
		print("Closing serial device...")
		if self.ser is None:
			print("Device already closed.")
		else:
			self.ser.close()
			print("Done.")

