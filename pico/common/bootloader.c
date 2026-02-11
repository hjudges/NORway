#include "bootloader.h"

#include "pico/bootrom.h"

void __attribute__((noreturn)) enter_bootloader(void)
{
    reset_usb_boot(0, 0);
}
