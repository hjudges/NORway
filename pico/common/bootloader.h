#ifndef __BOOTLOADER_H__
#define __BOOTLOADER_H__

/*
 * Restarts the pico in bootloader mode
 */
void __attribute__((noreturn)) enter_bootloader(void);

#endif /* __BOOTLOADER_H__ */
