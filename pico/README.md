# NORway for the RPi RP2350B
Coming soon...

# Build Dependencies
## Debian/Ubuntu
`sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib`

# Build Instructions
1. `cmake -S . -B build`
2. `cd build`
3. `make -j8`

# Hardware Compatibility

| Pico Chip | GPIO Pins | NORway | NANDway Signal Booster | NANDway Dual NAND | SPIway |
| --------- | --------- | ------ | ------- | ------ | --------- |
| Pico 1    | 26        | No     | **Yes** | No     | **Yes**   |
| Pico 2                                                        |
| &nbsp;&nbsp;RP235xA | 26        | No     | **Yes** | No     | **Yes**   |
| &nbsp;&nbsp;RP235xB | 48        | **Yes**| **Yes** | **Yes**| **Yes**   |

# TODOs
- Improve build to create a universal binary for Pico 1 and 2 (NANDway SBE and SPIway)
