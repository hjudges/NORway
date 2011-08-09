NORway v0.3

This project has been ported for the Teensy++ 2.0 from the original NOR flasher tool
("noralizer" by Hector Martin "marcan" <hector@marcansoft.com>) that was used to flash
AsbestOS onto the demo PS3 at 27C3, and for experimentation.

Contents:
	\teensy\NORway.avrsln		- Source code for the Teensy++ 2.0 (AVR Studio 5.0)
	\teensy\default\NORway.hex	- Compiled hex-file for the Teensy++ 2.0 (AT90USB1286)
	\hwinstall\install-*.jpg	- Some pics of how I've mounted the Teensy (CECH-2504A/B)
	\hwinstall\nor_testpoints.png	- Connection diagram of the NOR testpoints (CECH-2504A/B)
	\hwinstall\teensy.jpg		- Teensy connection points (see chart below)
	\serial_install.exe		- Teensy serial drivers for Windows
	\norpatch.exe			- Small tool to verify content of NOR dump (recognizes any OFW),
					  replace ros0/ros1 with new core os (.NET 2.0 required)
	\NORway.py			- PC Python client
	\README.txt			- This file

Hardware connections (see "\hwinstall\nor_testpoints.png" + "\hwinstall\teensy.jpg"):
	PS3		Teensy
	A0-7		PF0-7
	A8-15		PA0-7
	A16-22		PB0-6
	D0-7		PD0-7
	D8-15		PC0-7
	CE#		PE0
	OE#		PE1
	RESET#		PE4
	WE#		PE5
	RY/BY#		PE6
	TRISTATE#	PE7
	GND		GND

	The Teensy requires a 3.3V voltage regulator! 5V trace has to be cut and 3V pads have to be shorted!
	Please refer to https://www.pjrc.com/teensy/3volt.html

	DON'T CONNECT THE VCC SOLDER PADS TO ANYTHING!

	Connection diagrams for other boards can be found at:
	http://ps3devwiki.com/index.php?title=Hardware_flashing

Prerequisites for Windows:
	Python 2.7.2 (http://www.python.org/ftp/python/2.7.2/python-2.7.2.msi)
	pyserial 2.5 (http://pypi.python.org/packages/any/p/pyserial/pyserial-2.5.win32.exe)

Usage:
	Install Python + pyserial (see "Prerequisites for Windows"). Install drivers. Connect Teensy
	directly to your PC (not an USB hub, might cause trouble). Flash Teensy with "\teensy\default\NORway.hex"
	and reset it after completion. Start Windows' "Device Manager", expand the "Ports (COM & LPT)" node,
	you should see something like "USB Serial (Communication Class, Abstract Control Model) (COM4)". This
	tells you that "COM4" (or whatever shows up) is the COM port used by Teensy.
	
	At the command prompt enter "NORway.py" to display help.

	Procedure to dump your flash:
	- PS3 is turned off
	- At the command prompt enter:
	  NORway.py <your com port>
	- Turn on your PS3, it shouldn't boot
	- At the command prompt enter:
	  NORway.py <your com port> dump flash.bin
	- When dumping is finished enter:
	  NORway.py <your com port> release
	- Power off your PS3 (disconnect Teensy if PS3 doesn't boot when restarted)

	Dumping takes 45 seconds. A file "flash.bin" with a size of exactly 16 MB (16.777.216 bytes)
	has been created in your current directory.
	
	Writing a complete flash using the "write" command takes about 8.5 minutes.

	IMPORTANT:
	Before you flash your PS3 with anything new, make sure that the content of your dump is correct
	(use the supplied tool "norpatch.exe", it's able to recognize any OFW core os).
	And keep your original dump in a safe place. You might need it.


It's tested on Win7 x86/x64, but should also work just fine on any other x86/x64 Windows >=XP. Basically
it should also work on Linux, but I didn't test. If you're running Linux, you probably know what to do. ;-)
Serial drivers are not required for Linux.

Thanks to "marcan" for the original implementation.

 -- judges (judges@eEcho.com)
