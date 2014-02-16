NANDway v0.64 - Teensy++ 2.0 NAND flasher

Disclaimer
----------

WARNING: Use this software at your own risk. The author accepts no
responsibility for the consequences of your use of this software.


This project has been ported for the Teensy++ 2.0 from the original NOR flasher tool
("noralizer" by Hector Martin "marcan" <hector@marcansoft.com>) that was used to flash
AsbestOS onto the demo PS3 at 27C3, and for experimentation.

Contents:
	\teensyNOR\NORway.avrsln		- Source code for the Teensy++ 2.0 (AVR Studio 5.0)
	\teensyNOR\default\NORway.hex	- Compiled hex-file for the Teensy++ 2.0 (AT90USB1286)
	\teensyNAND\NANDway.atsln		- Source code for the Teensy++ 2.0 (AVR Studio 6.0)
	\teensyNAND\default\NANDway.hex	- Compiled hex-file for the Teensy++ 2.0 (AT90USB1286)
	\hwinstall\install-*.jpg	- Some pics of how I've mounted the Teensy (CECH-2504A/B)
	\hwinstall\nor_testpoints.png	- Connection diagram of the NOR testpoints (CECH-2504A/B)
	\hwinstall\teensy.jpg		- Teensy connection points (see chart below)
	\serial_install.exe		- Teensy serial drivers for Windows
	\norpatch.exe			- Small tool to verify content of NOR dump (recognizes any OFW),
					  replace ros0/ros1 with new core os (.NET 2.0 required)
	\NORway.py			- NOR PC Python client
	\NANDway.py			- NAND PC Python client
	\changes.txt			- Version history
	\perf-test.txt			- Performance tests for the different writing modes
	\README.txt			- This file

NOR Hardware connections (see "\hwinstall\nor_testpoints.png" + "\hwinstall\teensy.jpg"):
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


NAND Hardware connections:
		NAND		Teensy - nand0  	Teensy - nand1
	*	IO0-7		PF0-7			PC0-7		
	7	RY/BY#		PB6			PD6
	8	RE#		PB1			PD1
	9	CE#		PB0			PD0
	12	Vcc		3.3v - Vcc		3.3v - Vcc
	13	GND		GND			GND
	16	CLE		PB2			PD2
	17	ALE		PB3			PD3
	18	WE#		PB5			PD5
	19	WP#		PB4			PD4

	Tristate:
		PS3		Teensy
		TRISTATE	PB7/PD7

--------------------------------------------
	Powering option 1 (voltage regulator, Teensy powered by USB):
	Install the 3.3V voltage regulator available at pjrc.com! 5V trace has to be cut and 3V pads have to be shorted!
	Please refer to https://www.pjrc.com/teensy/3volt.html

	DON'T CONNECT THE VCC SOLDER PADS TO ANYTHING!
--------------------------------------------

--------------------------------------------
	Powering option 2 (external power, Teensy powered by console):
	Connect Teensy's VCC solder pad to PS3's 3.3V supply (see connection diagram). 5V trace has to be cut and
	3V pads have to be shorted!
	Please refer to https://www.pjrc.com/teensy/3volt.html

	DON'T INSTALL VOLTAGE REGULATOR!
--------------------------------------------

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

	-Norway.py
	
		* Flash Teensy with "\teensyNOR\default\NORway.hex"
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

		Dumping takes about 1 minute. A file "flash.bin" with a size of exactly 16 MB (16.777.216 bytes)
		has been created in your current directory.
		
		Writing a complete flash using the "write" command takes about 4:30 minutes.

	-NANDway.py

		* Flash Teensy with "\teensyNAND\default\NANDway.hex"
		At the command prompt enter "NANDway.py" to display help.

		first make sure that you are able to read the NAND's info. do this by using 
		the info command.
		

		get information:
		NANDway.py COMx NAND_ID info 

		dump:
		NANDway.py COMx NAND_ID dump filename 
		
		write:
		NANDway.py COMx NAND_ID write filename 

		NAND_ID is the id number of the NAND. it can be either 0 for NAND0 or 1 for 
		NAND1 on your motherboard.
	
		Bad blocks (currently only PS3 is supported):

		- Locate and print bad blocks in an input NAND dump file:
		NANDWay.py badblocks Filename

		- Remap the bad blocks of an input file to a new NAND given the new nand's dump (that is used to locate the bad blocks on the new NAND)
		NANDWay.py remapbadblocks InputFile NewNand OutputFile

IMPORTANT:
Before you flash your PS3 with anything new, make sure that the content of your dump is correct
(use the supplied tool "norpatch.exe", it's able to recognize any OFW core os).
And keep your original dump in a safe place. You might need it.

Support thread:
http://www.ps3hax.net/showthread.php?t=25408

More information available at (thanks to eussNL and all contributors for maintaining this great wiki!):
http://ps3devwiki.com/wiki/Teensy_2.0%2B%2B
http://ps3devwiki.com/wiki/Hardware_flashing

It's tested on Win7 x86/x64, but should also work just fine on any other x86/x64 Windows >=XP. Basically
it should also work on Linux and OSX, but I didn't test. If you're running Linux/OSX, you probably know what to do. ;-)
Serial drivers are not required for Linux/OSX.

Thanks to "marcan" for the original implementation.
Thanks to "CrackMyDay" for code optimizations and his Samsung tutorial!
	check this out if you run into troubles (last resort):
	http://www.ps3hax.net/showthread.php?p=284070#post284070
Thanks to everyone else for contributing and beta testing!

 -- judges (judges@eEcho.com)
