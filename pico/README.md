# NORway for the RPi Pico RP2350B
This is a port of NORway (and eventually NANDway Dual NAND/SPIway) for the RPi Pico.

# Build Dependencies
## Debian/Ubuntu
`sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib`

## Mac OS
`brew install arm-eabi-none-gcc cmake`

# Build Instructions
1. `cd pico`
2. `cmake -S . -B build`
3. `cd build`
4. `make -j8`

# Flashing Instructions
- Hold the BOOT button while plugging the pico's USB cable in
- Drag `NORway_rp2350b.uf2` to the pico's USB volume

# Changes from Teensy
## Performance
Writing to flash was tested after performing a chip erase before each test.

| Action | Command | Time |
| ------ | ------- | ---- |
| Read NOR | `dump` | 22s |
| Erase Chip | `erasechip` | 55s |
| Write - Buffered Programming Mode | `write` | 1m45s |
| Write - Unlock Bypass Mode | `writewordubm` | 1m45s |
| Write - Word mode | `writeword` | 4m30s |
| Verify | `verify` | 22s |

## GPIO Pins
- State of the GPIO pins is not accurately reflected in the output of the python script and will show `LOW` for all of them

# Hardware Compatibility
| Pico Chip           | GPIO Pins | NORway | NANDway Signal Booster | NANDway Dual NAND | SPIway |
| ------------------- | --------- | ------ | ------- | ------ | --------- |
| Pico 1              | 26        | No     | **Yes** | No     | **Yes**   |
| Pico 2                                                                  |
| &nbsp;&nbsp;RP235xA | 26        | No     | **Yes** | No     | **Yes**   |
| &nbsp;&nbsp;RP235xB | 48        | **Yes**| **Yes** | **Yes**| **Yes**   |

## Board Options
### Tested
- https://github.com/jvanderberg/RP2350B-Dev-Board
  This is what was used for development and testing. Form factor is similar to the Teensy++ 2.0 where it can fit in a PS3 Slim

### Untested
- https://www.solder.party/docs/rp2350-stamp-xl/
- https://www.olimex.com/Products/RaspberryPi/PICO/PICO2-XXL/

Let me know if you try these boards and I'll update the README.

# Pin Mapping
## NANDway
### Signal Booster Edition
| RPi Pin | NANDway SBE Signal |
| ------- | ------------------ |
| GPIO0   | WPal               |
| GPIO1   | RYBY               |
| GPIO2   | ALE                |
| GPIO3   | ALE                |
| GPIO4   | ALE                |
| GPIO5   | ALE                |
| GPIO6   | RE                 |
| GPIO7   | RE                 |
| GPIO8   | RE                 |
| GPIO9   | RE                 |
| GPIO10  | CLE                |
| GPIO11  | CLE                |
| GPIO12  | CLE                |
| GPIO13  | CLE                |
| GPIO14  | IO0                |
| GPIO15  | IO1                |
| GPIO16  | IO2                |
| GPIO17  | IO3                |
| GPIO18  | IO4                |
| GPIO19  | IO5                |
| GPIO20  | IO6                |
| GPIO21  | IO7                |
| GPIO22  | WE                 |
| GPIO23  | WE                 |
| GPIO24  | WE                 |
| GPIO25  | WE                 |


## NORway
Any RP235xB chip/board should be usable as long as all the GPIO pins are broken out. It **MUST** be a "B" revision chip as only that one has enough GPIO pins.

| RPi Pin | NORway Signal |
| ------- | ------------- |
| GPIO0   | A0            |
| GPIO1   | A1            |
| GPIO2   | A2            |
| GPIO3   | A3            |
| GPIO4   | A4            |
| GPIO5   | A5            |
| GPIO6   | A6            |
| GPIO7   | A7            |
| GPIO8   | A8            |
| GPIO9   | A9            |
| GPIO10  | A10           |
| GPIO11  | A11           |
| GPIO12  | A12           |
| GPIO13  | A13           |
| GPIO14  | A14           |
| GPIO15  | A15           |
| GPIO16  | A16           |
| GPIO17  | A17           |
| GPIO18  | A18           |
| GPIO19  | A19           |
| GPIO20  | A20           |
| GPIO21  | A21           |
| GPIO22  | A22           |
| GPIO23  | D0            |
| GPIO24  | D1            |
| GPIO25  | D2            |
| GPIO26  | D3            |
| GPIO27  | D4            |
| GPIO28  | D5            |
| GPIO29  | D6            |
| GPIO30  | D7            |
| GPIO31  | D8            |
| GPIO32  | D9            |
| GPIO33  | D10	          |
| GPIO34  | D11           |
| GPIO35  | D12           |
| GPIO36  | D13           |
| GPIO37  | D14           |
| GPIO38  | D15           |
| GPIO39  | RY/BY#        |
| GPIO40  | TRI#          |
| GPIO41  | CE#           |
| GPIO42  | WE#           |
| GPIO43  | OE#           |
| GPIO44  | RESET#        |


# TODOs
- Improve build to create a universal binary for Pico 1 and 2 (NANDway SBE and SPIway)
- Reflect GPIO status properly for the control lines
- Finish up porting NANDway and SPIway

# Debugging
Using the RPi debug probe and openocd: `sudo openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"`
- If `openocd` can't find `rp2350.cfg`, you'll need to build your own copy from the latest source code

Then you can use GDB to connect to the device:
1. `arm-none-eabi-gdb NORway_rp2350b.elf`
2. `target remote :3333`
3. `monitor reset init`
