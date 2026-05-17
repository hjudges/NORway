# Common Utilities
add_library(flashway_common STATIC
    bootloader.c
    delay.c
    mem_utils.c
    usb_serial.c
)

target_link_libraries(flashway_common pico_stdlib)

pico_enable_stdio_usb(flashway_common 1)
pico_enable_stdio_uart(flashway_common 0)
