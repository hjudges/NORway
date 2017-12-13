SPIway v0.40 - Teensy++ 2.0 SPI Flasher

Disclaimer
----------
WARNING: Use this software at your own risk. The author accepts no
responsibility for the consequences of your use of this software.

Contents:
	\SPIway\Release\SPIway.hex	- Compiled hex-file for the Teensy++ 2.0 (AT90USB1286)
	\SPIway_Installation\SPIway.jpg	- Teensy connection points
	\serial_install.exe		- Teensy serial drivers for Windows
	\SPIway.py			- SPI PC Python client
	\SPIway_Changes.txt		- Version history
	\SPIway_README.txt		- This file

--------------------------------------------
	Powering option 1 (voltage regulator, Teensy powered by USB):
	Install the 3.3V voltage regulator available at pjrc.com! 5V trace has to be cut and 3V pads have to be shorted!
	Please refer to https://www.pjrc.com/teensy/3volt.html

	Only off-board flashing has been tested (desoldered chip).
	You should power the chip with Teensy.
	Ideally you connect a decoupling capacitor (0.1uF) between GND and Vcc as close to the chip as possible!
	Keep the wires short (< 10cm) if you don't use a cap!
--------------------------------------------

Prerequisites for Windows:
	Python 2.7.2 (http://www.python.org/ftp/python/2.7.2/python-2.7.2.msi)
	pyserial 2.5 (http://pypi.python.org/packages/any/p/pyserial/pyserial-2.5.win32.exe)

Usage:
	Install Python + pyserial (see "Prerequisites for Windows"). Install drivers. Connect Teensy
	directly to your PC (not an USB hub, might cause trouble). Flash Teensy with "\SPIway\Release\SPIway.hex"
	and reset it after completion. Start Windows' "Device Manager", expand the "Ports (COM & LPT)" node,
	you should see something like "USB Serial (Communication Class, Abstract Control Model) (COM4)". This
	tells you that "COM4" (or whatever shows up) is the COM port used by Teensy.

	-SPIway.py

		* Flash Teensy with "\SPIway\Release\SPIway.hex"
		At the command prompt enter "SPIway.py" to display help.

		first make sure that you are able to read the SPI chip info. Do this by using
		the info command.

		get information:
		SPIway.py COMx info

		dump:
		SPIway.py COMx dump filename

		write:
		SPIway.py COMx write filename

		write with verify:
		SPIway.py COMx vwrite filename

		erase entire chip:
		SPIway.py COMx erasechip

Support thread:
http://www.ps3hax.net/showthread.php?t=25408

More information available at (thanks to eussNL and all contributors for maintaining this great wiki!):
http://www.psdevwiki.com/ps4/SPIway

It's tested on Win7 x86/x64, but should also work just fine on any other x86/x64 Windows >=XP. Works as well on
Linux and OS X. Serial drivers are not required for Linux/OS X.

 -- judges (judges@eEcho.com)
