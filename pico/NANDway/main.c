/************************************************************************
NANDway.c (v0.64) - Teensy++ 2.0 NAND Flasher for PS3/Xbox/Wii

Copyright (C) 2013  Effleurage
                    judges <judges@eEcho.com>
Copyright (C) 2026  MikeM64 - RPi Pico Ports

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/

#include <stdio.h>
#include "pico/stdlib.h"

#include "bootloader.h"
#include "delay.h"
#include "mem_utils.h"
#include "usb_serial.h"

#include "hardware/sync.h"

#define VERSION_MAJOR           0
#define VERSION_MINOR           65

#define BUILD_DUAL_NAND         1
#define BUILD_SIGNAL_BOOSTER    2

#if ((BUILD_VERSION != BUILD_DUAL_NAND) && (BUILD_VERSION != BUILD_SIGNAL_BOOSTER))
    #error BUILD_VERSION must be defined!
#endif

// Define commands
enum {
    CMD_PING1 = 0,
    CMD_PING2,
    CMD_BOOTLOADER,
    CMD_IO_LOCK,
    CMD_IO_RELEASE,
    CMD_PULLUPS_DISABLE,
    CMD_PULLUPS_ENABLE,
    CMD_NAND0_ID,
    CMD_NAND0_READPAGE,
    CMD_NAND0_WRITEPAGE,
    CMD_NAND0_ERASEBLOCK,
    CMD_NAND1_ID,
    CMD_NAND1_READPAGE,
    CMD_NAND1_WRITEPAGE,
    CMD_NAND1_ERASEBLOCK,
} cmd_t;


/*! \brief NAND flash read page command start. */
#define NAND_COMMAND_READ1              0x00
/*! \brief NAND flash read page command end. */
#define NAND_COMMAND_READ2              0x30
/*! \brief NAND flash read ID command. */
#define NAND_COMMAND_READID             0x90
/*! \brief NAND flash reset command. */
#define NAND_COMMAND_RESET              0xFF
/*! \brief NAND flash program page command start. */
#define NAND_COMMAND_PAGEPROG1          0x80
/*! \brief NAND flash program page command end. */
#define NAND_COMMAND_PAGEPROG2          0x10
/*! \brief NAND flash erase block command start. */
#define NAND_COMMAND_ERASE1             0x60
/*! \brief NAND flash erase block command end. */
#define NAND_COMMAND_ERASE2             0xD0
/*! \brief NAND flash read status command. */
#define NAND_COMMAND_STATUS             0x70
/*! \brief NAND flash random program page command start. */
#define NAND_COMMAND_RANDOM_PAGEPROG    0x85
/*! \brief NAND flash random read page command start. */
#define NAND_COMMAND_RANDOM_READ1       0x05
/*! \brief NAND flash random read page command end. */
#define NAND_COMMAND_RANDOM_READ2       0xE0

#define NAND_STATUS_FAIL            (1<<0) /* HIGH - FAIL,  LOW - PASS */
#define NAND_STATUS_IDLE            (1<<5) /* HIGH - IDLE,  LOW - ACTIVE */
#define NAND_STATUS_READY           (1<<6) /* HIGH - READY, LOW - BUSY */
#define NAND_STATUS_NOT_PROTECTED   (1<<7) /* HIGH - NOT,   LOW - PROTECTED */

#define BUF_SIZE_RW     4320
#define BUF_SIZE_ADDR   3

uint16_t    PAGE_PLUS_RAS_SZ = 0; /* page size + Redundant Area Size */
bool        IO_PULLUPS = true;

/*! \brief NAND flash information about maker, device, size and timing.
*/
typedef struct _nand_info {
    uint8_t raw_data[5];

    /*! \brief Out of bounce layout information. */
    uint16_t    oob_size;

    /*! \brief Maker code. */
    uint8_t     maker_code;
    /*! \brief Device code. */
    uint8_t     device_code;

    /*! \brief Page size in bytes. */
    uint32_t    page_size;
    /*! \brief Number of positions to shift when converting an offset in
     *         a block to page. Used when calculating NAND flash address.
     */
    //uint32_t  page_shift;
    /*! \brief Number of pages per block. */
    uint32_t    pages_per_block;
    /*! \brief Block size in bytes. */
    uint32_t    block_size;
    /*! \brief Number of positions to shift when converting block number
     *         to NAND flash address.
     */
    //uint32_t  block_shift;
    /*! \brief Number of blocks. */
    uint32_t    num_blocks;
    /*! \brief NAND flash I/O bus width in bits. */
    uint8_t     bus_width;

    /*! \brief NAND flash number of planes. */
    uint8_t     num_planes;
    /*! \brief NAND flash plane size. */
    uint32_t    plane_size;
} nand_info;


#if BUILD_VERSION == BUILD_DUAL_NAND
    #error Dual NAND is not yet supported!
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
    #define NAND0_CONTROL_WPal_PIN  (0)
    #define NAND0_CONTROL_RYBY_PIN  (1)

    #define NAND0_CONTROL_PIN_SHIFT (0)
    #define NAND0_CONTROL_PIN_MASK  (0x03ull << NAND0_CONTROL_PIN_SHIFT)
    #define NAND0_CONTROL_PIN_COUNT (2)

    #define CONTROL_ALE_PIN_SHIFT   (2)
    #define CONTROL_ALE_PIN_MASK    (0x0Full << CONTROL_ALE_PIN_SHIFT)
    #define CONTROL_ALE_PIN_COUNT   (4)

    #define CONTROL_RE_PIN_SHIFT    (6)
    #define CONTROL_RE_PIN_MASK     (0x0Full << CONTROL_RE_PIN_SHIFT)
    #define CONTROL_RE_PIN_COUNT    (4)

    #define CONTROL_WE_PIN_SHIFT    (20)
    #define CONTROL_WE_PIN_MASK     (0x0Full << CONTROL_WE_PIN_SHIFT)
    #define CONTROL_WE_PIN_COUNT    (4)

    #define CONTROL_CLE_PIN_SHIFT   (10)
    #define CONTROL_CLE_PIN_MASK    (0x0Full << CONTROL_CLE_PIN_SHIFT)
    #define CONTROL_CLE_PIN_COUNT   (4)

    #define NAND0_IO_PIN_SHIFT      (14)
    #define NAND0_IO_PIN_MASK       (0xFFull << NAND0_IO_PIN_SHIFT)
    #define NAND0_IO_PIN_COUNT      (8)

    #define ALL_PIN_MASK (CONTROL_ALE_PIN_MASK | \
                            CONTROL_RE_PIN_MASK | \
                            CONTROL_WE_PIN_MASK | \
                            CONTROL_CLE_PIN_MASK | \
                            NAND0_IO_PIN_MASK | \
                            NAND0_CONTROL_PIN_MASK)

#endif /* BUILD_VERSION */


typedef struct _nand_port {
    nand_info info;

    uint64_t    io_port_pin_mask;
    uint64_t    control_pins_mask;
    uint64_t    control_ale_pin_mask;
    uint64_t    control_re_pin_mask;
    uint64_t    control_we_pin_mask;
    uint64_t    control_cle_pin_mask;

    uint8_t     io_port_pin_shift;
    uint8_t     control_pins_shift;
    uint8_t     control_ale_pin_shift;
    uint8_t     control_re_pin_shift;
    uint8_t     control_we_pin_shift;
    uint8_t     control_cle_pin_shift;

    uint8_t     io_port_pin_count;
    uint8_t     control_pins_count;
    uint8_t     control_ale_pin_count;
    uint8_t     control_re_pin_count;
    uint8_t     control_we_pin_count;
    uint8_t     control_cle_pin_count;
} nand_port;


#if BUILD_VERSION == BUILD_DUAL_NAND
    #error Dual NAND is not yet supported!
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
    nand_port nand0 = {
        .io_port_pin_mask = NAND0_IO_PIN_MASK,
        .io_port_pin_shift = NAND0_IO_PIN_SHIFT,
        .io_port_pin_count = NAND0_IO_PIN_COUNT,

        .control_pins_mask = NAND0_CONTROL_PIN_MASK,
        .control_pins_shift = NAND0_CONTROL_PIN_SHIFT,
        .control_pins_count = NAND0_CONTROL_PIN_COUNT,

        .control_ale_pin_mask = CONTROL_ALE_PIN_MASK,
        .control_ale_pin_shift = CONTROL_ALE_PIN_SHIFT,
        .control_ale_pin_count = CONTROL_ALE_PIN_COUNT,

        .control_re_pin_mask = CONTROL_RE_PIN_MASK,
        .control_re_pin_shift = CONTROL_RE_PIN_SHIFT,
        .control_re_pin_count = CONTROL_RE_PIN_COUNT,

        .control_we_pin_mask = CONTROL_WE_PIN_MASK,
        .control_we_pin_shift = CONTROL_WE_PIN_SHIFT,
        .control_we_pin_count = CONTROL_WE_PIN_COUNT,

        .control_cle_pin_mask = CONTROL_CLE_PIN_MASK,
        .control_cle_pin_shift = CONTROL_CLE_PIN_SHIFT,
        .control_cle_pin_count = CONTROL_CLE_PIN_COUNT,
    };
#endif


/*
 * Enable pullups on the NAND IO pins
 */
void nand_io_pullups_enable(void)
{
    uint32_t i;

    for (i = NAND0_IO_PIN_SHIFT; i < 8; i++) {
        gpio_set_pulls(i, true /* up */, false /* down */);
    }

    IO_PULLUPS = true;
}


/*
 * Disable pullups on the NAND IO pins
 */
void nand_io_pullups_disable(void)
{
    uint32_t i;

    for (i = NAND0_IO_PIN_SHIFT; i < 8; i++) {
        gpio_set_pulls(i, false /* up */, false /* down */);
    }

    IO_PULLUPS = false;
}


/*
 * Reset pins to a known good default state
 */
void init_pins(void)
{
    /* Assign all pins to SW I/O */
    gpio_set_function_masked64(ALL_PIN_MASK, GPIO_FUNC_SIO);

    /* NAND IO Pins are set to output by default */
    gpio_set_dir_out_masked64(NAND0_IO_PIN_MASK);
}


/*
 * Release all pins to enable regular booting
 */
void release_pins(void)
{
    gpio_set_dir_in_masked64(ALL_PIN_MASK);

    nand_io_pullups_disable();
}


void nand_io_output(nand_port *nandp)
{
    gpio_set_dir_out_masked64(nandp->io_port_pin_mask);
}


void nand_control_output(nand_port *nandp)
{
    gpio_set_dir_out_masked64(nandp->control_pins_mask);
}


void nand_we_pins_pullup(nand_port *nandp)
{
    uint32_t i;

    for (i = nandp->control_we_pin_shift;
         i < nandp->control_we_pin_shift + nandp->control_we_pin_count;
         i++) {
        gpio_set_pulls(i, true /* up */, false /* down */);
    }
}


void nand_re_pins_pullup(nand_port *nandp)
{
    uint32_t i;

    for (i = nandp->control_re_pin_shift;
         i < nandp->control_re_pin_shift + nandp->control_re_pin_count;
         i++) {
        gpio_set_pulls(i, true /* up */, false /* down */);
    }
}


void nand_ale_pins_pullup(nand_port *nandp)
{
    uint32_t i;

    for (i = nandp->control_ale_pin_shift;
         i < nandp->control_ale_pin_shift + nandp->control_ale_pin_count;
         i++) {
        gpio_set_pulls(i, true /* up */, false /* down */);
    }
}


void nand_cle_pins_pullup(nand_port *nandp)
{
    uint32_t i;

    for (i = nandp->control_cle_pin_shift;
         i < nandp->control_cle_pin_shift + nandp->control_cle_pin_count;
         i++) {
        gpio_set_pulls(i, true /* up */, false /* down */);
    }
}


void ryby_pin_input(void)
{
    gpio_set_dir_in_masked64((1ull << NAND0_CONTROL_RYBY_PIN));
}


void ryby_pin_pullup_enable(void)
{
    gpio_set_pulls(NAND0_CONTROL_RYBY_PIN, true /* up */, false /* down */);
}


void wpal_pin_pullup_enable(void)
{
    gpio_set_pulls(NAND0_CONTROL_WPal_PIN, true /* up */, false /* down */);
}


void nand_ale_high(nand_port *nandp)
{
    gpio_put_masked64(
        nandp->control_ale_pin_mask,
        (uint64_t)0xFF << nandp->control_ale_pin_shift);
}


void nand_ale_low(nand_port *nandp)
{
    gpio_put_masked64(
        nandp->control_ale_pin_mask,
        0);
}


void nand_re_low(nand_port *nandp)
{
    gpio_put_masked64(
        nandp->control_re_pin_mask,
        0);
}


void nand_re_high(nand_port *nandp)
{
    gpio_put_masked64(
        nandp->control_re_pin_mask,
        (uint64_t)0xFF << nandp->control_re_pin_shift);
}


void nand_io_set(nand_port *nandp, char data)
{
    gpio_put_masked64(
        nandp->io_port_pin_mask,
        (uint64_t)data << nandp->io_port_pin_shift);
}


void nand_io_input(nand_port *nandp)
{
    gpio_set_dir_in_masked64(nandp->io_port_pin_mask);

    if (IO_PULLUPS) {
        nand_io_pullups_enable();
    } else {
        nand_io_pullups_disable();
    }
}


void nand_io_read(nand_port *nandp, char *data_out)
{
    uint64_t all_gpio_pins;

    nand_re_low(nandp);

    DELAY_100_NS();

    all_gpio_pins = gpio_get_all64();
    *data_out = (char)(all_gpio_pins >> nandp->io_port_pin_shift);

    nand_re_high(nandp);
}


void nand_enable(nand_port *nandp)
{
#if BUILD_VERSION == BUILD_DUAL_NAND
    *(nandp->cont_ddr) = 0xFF;          // all control ports - output
    *(nandp->cont_ddr) &= ~NAND_CONT_RYBY; /* ready / busy - input */

    *(nandp->cont_port) =   NAND_CONT_WEal |
                            NAND_CONT_REal |
                            NAND_CONT_WPal |
                            NAND_CONT_RYBY; /* input - pull up */
#elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
    nand_control_output(nandp);

    ryby_pin_input();

    /* input - pull up */
    wpal_pin_pullup_enable();
    ryby_pin_pullup_enable();

    /* Set WE to output + pullup */
    gpio_set_dir_out_masked64(CONTROL_WE_PIN_MASK);
    nand_we_pins_pullup(nandp);

    /* Set RE to output + pullup */
    gpio_set_dir_out_masked64(CONTROL_RE_PIN_MASK);
    nand_we_pins_pullup(nandp);

    /* Set ALE to output + pullup */
    gpio_set_dir_out_masked64(CONTROL_ALE_PIN_MASK);
    nand_ale_pins_pullup(nandp);

    /* Set CLE to output + pullup */
    gpio_set_dir_out_masked64(CONTROL_CLE_PIN_MASK);
    nand_cle_pins_pullup(nandp);
#endif

    nand_io_output(nandp); // io set as output
}


void nand_toggle_we(nand_port *nandp)
{
    gpio_put_masked64(
        nandp->control_we_pin_mask,
        0);
    gpio_put_masked64(
        nandp->control_we_pin_mask,
        (uint64_t)0xFF << nandp->control_we_pin_shift);
}


void nand_command(nand_port *nandp, uint8_t command)
{
    gpio_put_masked64(
        nandp->io_port_pin_mask,
        (uint64_t)command << nandp->io_port_pin_shift);
    gpio_put_masked64(
        nandp->control_cle_pin_mask,
        (uint64_t)0xFF << nandp->control_cle_pin_shift);

    nand_toggle_we(nandp);

    gpio_put_masked64(
        nandp->control_cle_pin_mask,
        0);
}


int wait_ryby(nand_port *nandp)
{
    /* Should be done within 3 milliseconds for all commands. */
    volatile uint32_t timeout = 0x2000000; //approx. 3secs

    while (timeout > 0) {
        if ( gpio_get(NAND0_CONTROL_RYBY_PIN) ) {
            return 1;
        }
        --timeout;
    }

    return 0;
}


void nand_reset(nand_port *nandp)
{
    nand_enable(nandp);

    nand_command(nandp, NAND_COMMAND_RESET);

    wait_ryby(nandp);
}


uint8_t nand_read_id(nand_port *nandp)
{
    uint32_t spare_size;
    char maker_code;
    char device_code;
    char chip_data;
    char size_data;
    char plane_data;
    char plane_size;
    uint8_t block_size;

    nand_enable(nandp);

    nand_command(nandp, NAND_COMMAND_READID);

    /* follow by address - 0 */
    nand_ale_high(nandp);
    nand_io_set(nandp, 0x00);
    nand_ale_low(nandp);

    nand_io_input(nandp);
    nand_io_read(nandp, &maker_code);
    nand_io_read(nandp, &device_code);
    nand_io_read(nandp, &chip_data);
    nand_io_read(nandp, &size_data);
    nand_io_read(nandp, &plane_data);

    //Samsung K9F1G08R0A
    //maker_code = 0xec;
    //device_code = 0xf1;
    //chip_data = 0xa0;
    //size_data = 0x15;
    //plane_data = 0x40;

    //Samsung K9T1G08U0M
    //maker_code = 0xec;
    //device_code = 0x79;
    //chip_data = 0xa5;
    //size_data = 0xc0;
    //plane_data = 0x00; /??

    //Samsung K9F2G08U0M
    //maker_code = 0xec;
    //device_code = 0xda;
    //chip_data = 0xa0; //??
    //size_data = 0x15;
    //plane_data = 0x50;

    //Hynix H27UBG8T2A
    //maker_code = 0xad;
    //device_code = 0xd7;
    //chip_data = 0x94;
    //size_data = 0x9a;
    //plane_data = 0x74;

    nandp->info.raw_data[0] = maker_code;
    nandp->info.raw_data[1] = device_code;
    nandp->info.raw_data[2] = chip_data;
    nandp->info.raw_data[3] = size_data;
    nandp->info.raw_data[4] = plane_data;

    nandp->info.maker_code = maker_code;
    nandp->info.device_code = device_code;

    if ((maker_code == 0xAD) && (device_code == 0xD7)) { // Hynix H27UBG8T2A
        /* Fill the NAND structure parameters */
        nandp->info.page_size  = 0x800 << (size_data & 0x03);
        //nandp->info.page_shift = ctz(nandp->info.page_size);
        nandp->info.bus_width = 8;
        nandp->info.num_planes = (1 << ((plane_data >> 2) & 0x03));
        nandp->info.plane_size = 2048UL * 1024UL * 1024UL;

        block_size = (size_data & 0xB0);
        /* Store the plane size in bytes. */
        switch (block_size) {
            case 0x0:
                nandp->info.block_size = 128UL * 1024UL;
            break;
            case 0x10:
                nandp->info.block_size = 256UL * 1024UL;
                break;
            case 0x20:
                nandp->info.block_size = 512UL * 1024UL;
                break;
            case 0x30:
                nandp->info.block_size = 768UL * 1024UL;
                break;
            case 0x80:
                nandp->info.block_size = 1024UL * 1024UL;
                break;
            case 0x90:
                nandp->info.block_size = 2048UL * 1024UL;
                break;
            default:
                return 0;
        }

        spare_size = (size_data >> 2) & 0x03;
        switch (spare_size) {
            case 0x0:
                nandp->info.oob_size = 128;
                break;
            case 0x1:
                nandp->info.oob_size = 224;
                break;
            case 0x2:
                nandp->info.oob_size = 448;
                break;
            default:
                return 0;
        }
    }
    else if ((maker_code == 0xAD) && (device_code == 0x73)) { // Hynix HY27US08281A
        nandp->info.page_size  = 512;
        nandp->info.block_size = 32UL * nandp->info.page_size;
        nandp->info.num_planes = 1;
        nandp->info.oob_size = 16;
        nandp->info.plane_size = 1UL << 24;
        nandp->info.bus_width = 8;
    }
    else if ((maker_code == 0xEC) && (device_code == 0x79)) { // Samsung K9T1G08U0M
        nandp->info.page_size  = 512;
        nandp->info.block_size = 32UL * nandp->info.page_size;
        nandp->info.num_planes = 4;
        nandp->info.oob_size = 16;
        nandp->info.plane_size = 1UL << 25;
        nandp->info.bus_width = 8;
    }
    else {
        if ((maker_code == 0xEC) && (device_code == 0xF1)) // Samsung K9F1G08U0A
            plane_data = 0x40;
        else if ((maker_code == 0xEC) && (device_code == 0xA1)) // Samsung K9F1G08R0A
            plane_data = 0x40;
        else if ((maker_code == 0xEC) && (device_code == 0xDA)) // Samsung K9F2G08U0M
            plane_data = 0x50;

        /* Fill the NAND structure parameters */
        nandp->info.page_size  = 0x400 << (size_data & 0x03);
        //nandp->info.page_shift = ctz(nandp->info.page_size);
        nandp->info.block_size = (64UL * 1024UL) << ((size_data & 0x30) >> 4);

        nandp->info.num_planes = (1 << ((plane_data >> 2) & 0x03));

        spare_size = (8 << ((size_data & 0x04) >> 2)) * (nandp->info.page_size / 512);
        switch (spare_size) {
            case 16:
            case 64:
                nandp->info.oob_size = spare_size;
                break;
            default:
                return 0;
        }

        plane_size = (plane_data >> 4) & 0x07;
        /* Store the plane size in bytes. */
        switch (plane_size) {
            case 0x0:
                nandp->info.plane_size = 1UL << 23;
                break;
            case 0x1:
                nandp->info.plane_size = 1UL << 24;
                break;
            case 0x2:
                nandp->info.plane_size = 1UL << 25;
                break;
            case 0x3:
                nandp->info.plane_size = 1UL << 26;
                break;
            case 0x4:
                nandp->info.plane_size = 1UL << 27;
                break;
            case 0x5:
                nandp->info.plane_size = 1UL << 28;
                break;
            case 0x6:
                nandp->info.plane_size = 1UL << 29;
                break;
            case 0x7:
                nandp->info.plane_size = 1UL << 30;
                break;
            default:
                return 0;
        }

        if ((size_data & 0x40) == 0) {
            nandp->info.bus_width = 8;
        } else {
            nandp->info.bus_width = 16;
        }
    }

    nandp->info.num_blocks = (uint32_t)nandp->info.num_planes * (nandp->info.plane_size / nandp->info.block_size);
    nandp->info.pages_per_block = nandp->info.block_size / nandp->info.page_size;
    //nandp->info.block_shift = ctz(nandp->info.pages_per_block);

    PAGE_PLUS_RAS_SZ = nandp->info.page_size + nandp->info.oob_size;

    return 1;
}


void handle_read_id(nand_port *nand) {
    nand_reset(nand);
    if (nand_read_id(nand)) {
        //25 bytes
        usb_serial_putchar(nand->info.raw_data[0]); //maker_code
        usb_serial_putchar(nand->info.raw_data[1]); //device_code
        usb_serial_putchar(nand->info.raw_data[2]);
        usb_serial_putchar(nand->info.raw_data[3]);
        usb_serial_putchar(nand->info.raw_data[4]);
        usb_serial_putchar((nand->info.page_size >> 24) & 0xFF);
        usb_serial_putchar((nand->info.page_size >> 16) & 0xFF);
        usb_serial_putchar((nand->info.page_size >> 8) & 0xFF);
        usb_serial_putchar(nand->info.page_size & 0xFF);
        usb_serial_putchar((nand->info.oob_size >> 8) & 0xFF);
        usb_serial_putchar(nand->info.oob_size & 0xFF);
        usb_serial_putchar(nand->info.bus_width);
        usb_serial_putchar((nand->info.block_size >> 24) & 0xFF);
        usb_serial_putchar((nand->info.block_size >> 16) & 0xFF);
        usb_serial_putchar((nand->info.block_size >> 8) & 0xFF);
        usb_serial_putchar(nand->info.block_size & 0xFF);
        usb_serial_putchar((nand->info.num_blocks >> 24) & 0xFF);
        usb_serial_putchar((nand->info.num_blocks >> 16) & 0xFF);
        usb_serial_putchar((nand->info.num_blocks >> 8) & 0xFF);
        usb_serial_putchar(nand->info.num_blocks & 0xFF);
        usb_serial_putchar(nand->info.num_planes);
        usb_serial_putchar((nand->info.plane_size >> 24) & 0xFF);
        usb_serial_putchar((nand->info.plane_size >> 16) & 0xFF);
        usb_serial_putchar((nand->info.plane_size >> 8) & 0xFF);
        usb_serial_putchar(nand->info.plane_size & 0xFF);
    }
    else {
        usb_serial_putchar(nand->info.raw_data[0]); //maker_code
        usb_serial_putchar(nand->info.raw_data[1]); //device_code
        usb_serial_putchar(nand->info.raw_data[2]);
        usb_serial_putchar(nand->info.raw_data[3]);
        usb_serial_putchar(nand->info.raw_data[4]);
        for (uint8_t i = 0; i < 20; ++i) {
            usb_serial_putchar(0);
        }
    }
}


void nand_read_page(nand_port *nandp, char *buf_addr) {
    size_t      i;
    char        buf_rw[BUF_SIZE_RW];
    uint32_t    saved_interrupts;

    nand_enable(nandp);

    /* read command */
    nand_command(nandp, NAND_COMMAND_READ1);

    /* address */
    nand_ale_high(nandp);
    if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73)) {
        nand_io_set(nandp, 0);
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
    }
    else if ((nandp->info.maker_code == 0xEC) && (nandp->info.device_code == 0x79)) { // Samsung K9T1G08U0M
        nand_io_set(nandp, 0);
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
        nand_io_set(nandp, buf_addr[2]);
    }
    else {
        nand_io_set(nandp, 0);
        nand_io_set(nandp, 0);
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
        nand_io_set(nandp, buf_addr[2]);
    }
    nand_ale_low(nandp);

    if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73))
        DELAY_100_NS();
    else if ((nandp->info.maker_code == 0xEC) && (nandp->info.device_code == 0x79)) // Samsung K9T1G08U0M
        DELAY_100_NS();
    else
        nand_command(nandp, NAND_COMMAND_READ2);

    nand_io_input(nandp);

    saved_interrupts = save_and_disable_interrupts();

    /* wait for the nand to read this page to the internal page register */
    wait_ryby(nandp);

    for (uint8_t k = 0; k < PAGE_PLUS_RAS_SZ / BUF_SIZE_RW; ++k) {
        for (i = 0; i < BUF_SIZE_RW; ++i) {
            nand_io_read(nandp, &buf_rw[i]);
        }
        usb_serial_write(buf_rw, BUF_SIZE_RW);
    }

    uint16_t rest = PAGE_PLUS_RAS_SZ - ((PAGE_PLUS_RAS_SZ / BUF_SIZE_RW) * BUF_SIZE_RW);
    for (i = 0; i < rest; ++i) {
        nand_io_read(nandp, &buf_rw[i]);
    }
    usb_serial_write(buf_rw, rest);

    restore_interrupts(saved_interrupts);
}


void handle_read_page(nand_port *nand) {
    size_t  i = 0;
    int     rc = 0;
    char    buf_addr[BUF_SIZE_ADDR];

    while (i < sizeof(buf_addr) && rc != PICO_ERROR_TIMEOUT) {
        rc = usb_serial_getbuf(&buf_addr[i], 128);
        if (rc == PICO_ERROR_TIMEOUT) {
            usb_serial_putchar('T');
        }

        i += rc;
    }

    if (i < sizeof(buf_addr)) {
        usb_serial_putchar('R');
    } else {
        usb_serial_putchar('K');
        nand_read_page(nand, buf_addr);
    }
}


uint8_t nand_status(nand_port *nandp) {
    char status;

    nand_command(nandp, NAND_COMMAND_STATUS);

    nand_io_input(nandp);

    nand_io_read(nandp, &status);

    nand_io_output(nandp);

    return status;
}


void nand_busy_wait(nand_port *nandp)
{
    uint8_t status;

    do {
        sleep_us(5);
        status = nand_status(nandp);
    } while (!(status & NAND_STATUS_READY));
}


int nand_write_page(nand_port *nandp, char *buf_addr) {
    uint16_t i;
    int16_t in_data;

    nand_enable(nandp);

    /* Serial Data Input command */
    /* CLE - high & CE# - low */
    nand_command(nandp, NAND_COMMAND_PAGEPROG1);

    /* address */
    /* ALE on & CLE off */
    nand_ale_high(nandp);
    if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73)) {
        nand_io_set(nandp, 0);
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
    }
    else if ((nandp->info.maker_code == 0xEC) && (nandp->info.device_code == 0x79)) { // Samsung K9T1G08U0M
        nand_io_set(nandp, 0);
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
        nand_io_set(nandp, buf_addr[2]);
    }
    else {
        nand_io_set(nandp, 0);
        nand_io_set(nandp, 0);
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
        nand_io_set(nandp, buf_addr[2]);
    }
    nand_ale_low(nandp);

    /*
     * TODO: This might be a good target for future optimization
     */
    for (i = 0; i < PAGE_PLUS_RAS_SZ; i++) {
        if ((in_data = usb_serial_getchar()) != -1) {
            nand_io_set(nandp, in_data);
        }
        else {
            break;
        }
    }

    /* Page Program confirm command */
    nand_command(nandp, NAND_COMMAND_PAGEPROG2);

    if (i < PAGE_PLUS_RAS_SZ) { // timeout
        return -1;      // and exit
    }

    /* wait for the internal controller to finish the program command
        TBD - up to 200us */
    nand_busy_wait(nandp);

    return !(nand_status(nandp) & NAND_STATUS_FAIL);
}


void handle_write_page(nand_port *nand) {
    size_t  i = 0;
    int     rc = 0;
    char    buf_addr[BUF_SIZE_ADDR];

    while (i < sizeof(buf_addr) && rc != PICO_ERROR_TIMEOUT) {
        rc = usb_serial_getbuf(&buf_addr[i], 128);
        if (rc == PICO_ERROR_TIMEOUT) {
            usb_serial_putchar('T');
        }

        i += rc;
    }

    if (i < sizeof(buf_addr)) {
        usb_serial_putchar('R');
    } else {
        rc = nand_write_page(nand, buf_addr);
        if (rc > 0) {
            usb_serial_putchar('K');
        } else if (rc < 0) {
            usb_serial_putchar('R');
        } else {
            usb_serial_putchar('V');
        }
    }
}


int nand_erase_block(nand_port *nandp, char *buf_addr) {
    nand_enable(nandp);

    /* block erase setup command */
    nand_command(nandp, NAND_COMMAND_ERASE1);

    /* block address */
    /* ALE on & CLE off */
    nand_ale_high(nandp);
    if ((nandp->info.maker_code == 0xAD) && (nandp->info.device_code == 0x73)) {
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
    }
    else {
        nand_io_set(nandp, buf_addr[0]);
        nand_io_set(nandp, buf_addr[1]);
        nand_io_set(nandp, buf_addr[2]);
    }
    nand_ale_low(nandp);

    /* block erase confirm command */
    nand_command(nandp, NAND_COMMAND_ERASE2);

    if (!(nand_status(nandp) & NAND_STATUS_NOT_PROTECTED)) {
        return -1;
    }

    /* wait for the internal controller to finish the erase command
        TBD - up to 2ms */
    nand_busy_wait(nandp);

    return !(nand_status(nandp) & NAND_STATUS_FAIL);
}


void handle_erase_block(nand_port *nand) {
    size_t  i = 0;
    int     rc = 0;
    char    buf_addr[BUF_SIZE_ADDR];

    while (i < sizeof(buf_addr) && rc != PICO_ERROR_TIMEOUT) {
        rc = usb_serial_getbuf(&buf_addr[i], 128);
        if (rc == PICO_ERROR_TIMEOUT) {
            usb_serial_putchar('T');
        }

        i += rc;
    }

    if (i < sizeof(buf_addr)) {
        usb_serial_putchar('R');
    } else {
        rc = nand_erase_block(nand, buf_addr);
        if (rc > 0) {
            usb_serial_putchar('K');
        } else if (rc < 0) {
            usb_serial_putchar('P');
        } else {
            usb_serial_putchar('V');
        }
    }
}



void run_commands(void)
{
    int                 rc;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        switch (rc) {
        case CMD_PING1:
            usb_serial_putchar(VERSION_MAJOR);
            break;
        case CMD_PING2:
            uint16_t freeram = get_free_heap();
            usb_serial_putchar(VERSION_MINOR);
            usb_serial_putchar((freeram >> 8) & 0xFF);
            usb_serial_putchar(freeram & 0xFF);
            break;
        case CMD_BOOTLOADER:
            enter_bootloader();
            break;
        case CMD_IO_LOCK:
            init_pins();
            break;
        case CMD_IO_RELEASE:
            release_pins();
            break;
        case CMD_PULLUPS_DISABLE:
            nand_io_pullups_disable();
            break;
        case CMD_PULLUPS_ENABLE:
            nand_io_pullups_enable();
            break;
        case CMD_NAND0_ID:
            usb_serial_putchar('Y');
            handle_read_id(&nand0);
            #if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
                release_pins();
            #endif
            break;
        case CMD_NAND0_READPAGE:
            handle_read_page(&nand0);
            #if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
                release_pins();
            #endif
            break;
        case CMD_NAND0_WRITEPAGE:
            handle_write_page(&nand0);
            #if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
                release_pins();
            #endif
            break;
        case CMD_NAND0_ERASEBLOCK:
            handle_erase_block(&nand0);
            #if BUILD_VERSION == BUILD_SIGNAL_BOOSTER
                release_pins();
            #endif
            break;
        case CMD_NAND1_ID:
            #if BUILD_VERSION == BUILD_DUAL_NAND
                usb_serial_putchar('Y');
                handle_read_id(&nand1);
            #elif BUILD_VERSION == BUILD_SIGNAL_BOOSTER
                usb_serial_putchar('N');
            #endif
            break;
        case CMD_NAND1_READPAGE:
            #if BUILD_VERSION == BUILD_DUAL_NAND
                handle_read_page(&nand1);
            #endif
            break;
        case CMD_NAND1_WRITEPAGE:
            #if BUILD_VERSION == BUILD_DUAL_NAND
                handle_write_page(&nand1);
            #endif
            break;
        case CMD_NAND1_ERASEBLOCK:
            #if BUILD_VERSION == BUILD_DUAL_NAND
                handle_erase_block(&nand1);
            #endif
            break;
        }
    }
}


int main(void)
{
    stdio_init_all();
    systick_timer_init();

    release_pins();
    wait_for_usb_serial_connection();
    init_pins();

    while(1) {
        usb_serial_flush();

        while (usb_serial_connected()) {
            run_commands();
        }
    }

    return 0;
}
