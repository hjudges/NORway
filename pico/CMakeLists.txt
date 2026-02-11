cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from GIT
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(flashway_rp2350b)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_compile_options(-Werror -Wall)

# Common Utilities
add_library(flashway_common
    common/bootloader.c
    common/delay.c
    common/mem_utils.c
    common/usb_serial.c
)
include_directories(common)
target_link_libraries(flashway_common pico_stdlib)

pico_enable_stdio_usb(flashway_common 1)
pico_enable_stdio_uart(flashway_common 0)

##############################

# NANDway Signal Booster Build
add_executable(NANDway_SB_rp2350b
    NANDway/main.c
)

target_compile_definitions(NANDway_SB_rp2350b PRIVATE BUILD_VERSION=2)
include_directories(common)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(NANDway_SB_rp2350b pico_stdlib)
target_link_libraries(NANDway_SB_rp2350b flashway_common)

# Setup SDK features
pico_enable_stdio_usb(NANDway_SB_rp2350b 1)
pico_enable_stdio_uart(NANDway_SB_rp2350b 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(NANDway_SB_rp2350b)

##############################

# NORway Build
add_executable(NORway_rp2350b
    NORway/main.c
)

include_directories(common)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(NORway_rp2350b pico_stdlib)
target_link_libraries(NORway_rp2350b flashway_common)

# Setup SDK features
pico_enable_stdio_usb(NORway_rp2350b 1)
pico_enable_stdio_uart(NORway_rp2350b 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(NORway_rp2350b)

##############################
