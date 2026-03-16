/*
 * Copyright (c) 2026 MikeM64
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef __PICO2B_BOARD_H__
#define __PICO2B_BOARD_H__

/*
 * This is a simple header for the RP2350B dev boards.
 */
pico_board_cmake_set(PICO_PLATFORM, rp2350)
#define RASPBERRYPI_PICO2B
#define PICO_RP2350B 1

#define PICO_BOOT_STAGE2_CHOOSE_W25Q080 1

#ifndef PICO_FLASH_SPI_CLKDIV
#define PICO_FLASH_SPI_CLKDIV 2
#endif /* PICO_FLASH_SPI_CLKDIV */

pico_board_cmake_set_default(PICO_FLASH_SIZE_BYTES, (16 * 1024 * 1024))
#ifndef PICO_FLASH_SIZE_BYTES
#define PICO_FLASH_SIZE_BYTES (16 * 1024 * 1024)
#endif /* PICO_FLASH_SIZE_BYTES */

pico_board_cmake_set_default(PICO_RP2350_A2_SUPPORTED, 1)
#ifndef PICO_RP2350_A2_SUPPORTED
#define PICO_RP2350_A2_SUPPORTED 1
#endif /* PICO_RP2350_A2_SUPPORTED */

#endif /* __PICO2B_BOARD_H__ */
