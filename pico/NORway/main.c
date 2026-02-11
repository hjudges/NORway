/************************************************************************
norflash.v - NOR flasher for PS3

Copyright (C) 2010-2011, 2026

Hector Martin "marcan" <hector@marcansoft.com>
NORway.c (v0.8) - Teensy++ 2.0 port by judges@eEcho.com
RP2350B port by MikeM64

This code is licensed to you under the terms of the GNU GPL, version 2;
see file COPYING or http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
*************************************************************************/


#include <stdio.h>
#include "pico/stdlib.h"

#include "delay.h"
#include "bootloader.h"
#include "usb_serial.h"

#include "hardware/gpio.h"

/* GPIO 0 through 22 are reserved for address lines */
#define ADDRESS_PIN_MASK    (0x7FFFFFull)

/* GPIO 23 though 39 are reserved for data lines */
#define DATA_PIN_SHIFT      (23)
#define DATA_PIN_MASK       (0x3FFFC00000ull)

/* GPIO 40 through 45 are for control lines */
#define RYBY_PIN            (40)
#define TRISTATE_PIN        (41)
#define CE_PIN              (42)
#define WE_PIN              (43)
#define OE_PIN              (44)
#define RESET_PIN           (45)

#define RYBY_PIN_MASK       (1ull << RYBY_PIN)
#define TRISTATE_PIN_MASK   (1ull << TRISTATE_PIN)
#define CE_PIN_MASK         (1ull << CE_PIN)
#define WE_PIN_MASK         (1ull << WE_PIN)
#define OE_PIN_MASK         (1ull << OE_PIN)
#define RESET_PIN_MASK      (1ull << RESET_PIN)

#define CONTROL_PIN_MASK    (RYBY_PIN_MASK | \
                             TRISTATE_PIN_MASK | \
                             CE_PIN_MASK | \
                             WE_PIN_MASK | \
                             OE_PIN_MASK | \
                             RESET_PIN_MASK)

#define ALL_PIN_MASK        (ADDRESS_PIN_MASK | \
                             DATA_PIN_MASK | \
                             CONTROL_PIN_MASK)

#define NUM_PINS            (46)


enum fsm_states_e {
    S_IDLE = 0,
    S_READING_BSS_4,
    S_READING_BSS_8,
    S_READING_BSS_64,
    S_READING_BSS_128,
    S_READING_BSS_WORD,
    S_ADDR2,
    S_ADDR3,
    S_WRITE,
    S_WRITE_INCREMENT,
    S_WAIT,
    S_WAIT_INCREMENT,
    S_WRITEWORD,
    S_WRITEWORD_2NDDIE,
    S_WRITEWORD_UBM,
    S_WRITEWORD_UBM_2NDDIE,
    S_WRITEWORD_WBP,
    S_WRITEWORD_WBP_2NDDIE,
};

enum external_commands_e {
    CMD_NOP = 0,
    CMD_READSTATE,
    CMD_PING1,
    CMD_PING2,
    CMD_BOOTLOADER,
    CMD_ADDR_INCREMENT,
    CMD_RELEASE_PORTS,
    CMD_INIT_PORTS,
    CMD_RESET_DISABLE,
    CMD_RESET_ENABLE,
    CMD_SPEEDTEST_READ,
    CMD_SPEEDTEST_WRITE,
    CMD_VERIFY_DISABLE,
    CMD_VERIFY_ENABLE,
    CMD_WAIT,
    CMD_WAIT_INCREMENT,
    CMD_READ_BSS_4 = 0x10,
    CMD_READ_BSS_8 = 0x11,
    CMD_READ_BSS_64 = 0x12,
    CMD_READ_BSS_128 = 0x13,
    CMD_READ_BSS_WORD = 0x14,
    CMD_WRITE = 0x18,
    CMD_WRITE_INCREMENT = 0x19,
    CMD_WRITEWORD = 0x1a,
    CMD_WRITEWORD_2NDDIE = 0x1b,
    CMD_WRITEWORD_UBM = 0x1c,
    CMD_WRITEWORD_UBM_2NDDIE = 0x1d,
    CMD_WRITEWORD_WBP = 0x1e,
    CMD_WRITEWORD_WBP_2NDDIE = 0x1f,
};

// Define block/sector size for reading/writing
#define BSS_4       0x01000 //2Kwords = 4KB
#define BSS_8       0x02000 //4Kwords = 8KB
#define BSS_64      0x10000 //32Kwords = 64KB
#define BSS_128     0x20000 //64Kwords = 128KB
#define BSS_WORD    0x00002 //word = 2Bytes


/* Global configuration */
static bool s_write_verification_enabled = false;


/*
 * Reset pins to a known good default state
 */
void init_pins(void)
{
    /* Assign all pins to SW I/O */
    gpio_set_function_masked64(ALL_PIN_MASK, GPIO_FUNC_SIO);

    /* Address pins are always output and default to 0 */
    gpio_set_dir_out_masked64(ADDRESS_PIN_MASK);
    gpio_clr_mask64(ADDRESS_PIN_MASK);

    /* Data Pins start as output and are not touched */
    gpio_set_dir_out_masked64(DATA_PIN_MASK);

    /* Control pins except RYBY are output */
    gpio_set_dir_out_masked64(CONTROL_PIN_MASK & (~RYBY_PIN_MASK));
    gpio_set_dir_in_masked64(RYBY_PIN_MASK);
}


/*
 * Release all pins to enable regular booting
 */
void release_pins(void)
{
    uint32_t i;

    gpio_set_dir_in_masked64(ALL_PIN_MASK);

    /* Disable pull up/down resistors on the pins */
    for (i = 0; i < NUM_PINS; i++) {
        gpio_set_pulls(i, false, false);
    }
}


uint16_t get_data_pins(void)
{
    uint64_t all_gpio_pins = gpio_get_all64();

    return (uint16_t)(all_gpio_pins >> DATA_PIN_SHIFT);
}


void put_data_u16(uint16_t data_to_output)
{
    gpio_put_masked64(
        DATA_PIN_MASK,
        (uint64_t)data_to_output << DATA_PIN_SHIFT);
}


void set_data_pins_input(void)
{
    gpio_set_dir_in_masked64(DATA_PIN_MASK);
}


void set_data_pins_output(void)
{
    gpio_set_dir_out_masked64(DATA_PIN_MASK);
}


/*
 * To simplify porting the teensy code, the address is updated
 * byte-by-byte per the existing protocol.
 */
static uint32_t s_address = 0x0;
void update_address3(uint8_t addr3)
{
    s_address = (addr3 << 16) | (s_address & 0xffff);
}


void update_address2(uint8_t addr2)
{
    s_address = (s_address & 0xff00ff) | (addr2 << 8);
}


void update_address1(uint8_t addr1)
{
    s_address = (s_address & 0xffff00) | (addr1);
}


/*
 * Put the current address on the bus
 */
void update_address_pins(void)
{
    gpio_put_masked(
        ADDRESS_PIN_MASK,
        (s_address & ADDRESS_PIN_MASK)
    );
}


void put_address_u32(uint32_t address_to_output)
{
    gpio_put_masked(
        ADDRESS_PIN_MASK,
        (address_to_output & ADDRESS_PIN_MASK));
}


void address_increment(void)
{
    s_address++;
}


void address_increment_and_update_pins(void)
{
    s_address++;
    update_address_pins();
}


/* Helpers for single pins */
void CE_LOW(void)
{
    gpio_clr_mask64(CE_PIN_MASK);
}


void CE_HIGH(void)
{
    gpio_set_mask64(CE_PIN_MASK);
}


void OE_LOW(void)
{
    gpio_clr_mask64(OE_PIN_MASK);
}


void OE_HIGH(void)
{
    gpio_set_mask64(OE_PIN_MASK);
}


void RESET_LOW(void)
{
    gpio_clr_mask64(RESET_PIN_MASK);
}


void RESET_HIGH(void)
{
    gpio_set_mask64(RESET_PIN_MASK);
}


void WE_LOW(void)
{
    gpio_clr_mask64(WE_PIN_MASK);
}


void WE_HIGH(void)
{
    gpio_set_mask64(WE_PIN_MASK);
}


bool get_RYBY(void)
{
    return gpio_get(RYBY_PIN);
}


uint8_t state_byte(void)
{
    uint8_t state_byte = 0;

    if (gpio_get(TRISTATE_PIN)) {
        state_byte |= 0x20;
    }

    if (gpio_get(RESET_PIN)) {
        state_byte |= 0x10;
    }

    if (gpio_get(RYBY_PIN)) {
        state_byte |= 0x08;
    }

    if (gpio_get(CE_PIN)) {
        state_byte |= 0x04;
    }

    if (gpio_get(WE_PIN)) {
        state_byte |= 0x02;
    }

    if (gpio_get(OE_PIN)) {
        state_byte |= 0x01;
    }

    return (0);
}


void speedtest_send()
{
    char buf_read[64];
    uint8_t buf_ix;
    uint32_t addr;

    addr = buf_ix = 0;
    while (1) {
        buf_read[buf_ix] = buf_ix;
        buf_ix++;
        buf_read[buf_ix] = buf_ix;
        buf_ix++;
        if (buf_ix == 64) {
            usb_serial_write(buf_read, buf_ix);
            buf_ix = 0;
        }
        if (addr++ == (BSS_128/2-1))
            break;
    }
}


void speedtest_receive()
{
    int rc;
    char buf_write[BSS_4];
    int i = 0;

    while (i < BSS_4) {
        rc = usb_serial_getbuf(&buf_write[i], 128);
        if (rc == PICO_ERROR_TIMEOUT) {
            usb_serial_putchar('T');
            return;
        }

        i += rc;
    }

    if (i != BSS_4) {
        usb_serial_putchar('R');
    } else {
        usb_serial_putchar('K');
    }
}


enum fsm_states_e run_idle_state(void)
{
    int rc;
    enum fsm_states_e next_state = S_IDLE;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        switch (rc) {
        case CMD_NOP:
            break;
        case CMD_READSTATE:
            usb_serial_putchar(state_byte());
            break;
        case CMD_PING1:
            usb_serial_putchar(0x42);
            break;
        case CMD_PING2:
            usb_serial_putchar(0xbd);
            break;
        case CMD_BOOTLOADER:
            enter_bootloader();
            break;
        case CMD_ADDR_INCREMENT:
            address_increment_and_update_pins();
            break;
        case CMD_RELEASE_PORTS:
            release_pins();
            break;
        case CMD_INIT_PORTS:
            init_pins();
            break;
        case CMD_RESET_DISABLE:
            RESET_HIGH();
            break;
        case CMD_RESET_ENABLE:
            RESET_LOW();
            break;
        case CMD_VERIFY_ENABLE:
            s_write_verification_enabled = true;
            break;
        case CMD_VERIFY_DISABLE:
            s_write_verification_enabled = false;
            break;
        case CMD_SPEEDTEST_READ:
            speedtest_send();
            break;
        case CMD_SPEEDTEST_WRITE:
            speedtest_receive();
            break;
        case CMD_WAIT:
            next_state = S_WAIT;
            break;
        case CMD_WAIT_INCREMENT:
            next_state = S_WAIT_INCREMENT;
            break;
        case CMD_READ_BSS_4:
            next_state = S_READING_BSS_4;
            break;
        case CMD_READ_BSS_8:
            next_state = S_READING_BSS_8;
            break;
        case CMD_READ_BSS_64:
            next_state = S_READING_BSS_64;
            break;
        case CMD_READ_BSS_128:
            next_state = S_READING_BSS_128;
            break;
        case CMD_READ_BSS_WORD:
            next_state = S_READING_BSS_WORD;
            break;
        case CMD_WRITE:
            OE_HIGH();
            CE_LOW();
            WE_LOW();
            next_state = S_WRITE;
            break;
        case CMD_WRITE_INCREMENT:
            OE_HIGH();
            CE_LOW();
            WE_LOW();
            next_state = S_WRITE_INCREMENT;
            break;
        case CMD_WRITEWORD:
            OE_HIGH();
            CE_LOW();
            next_state = S_WRITEWORD;
            break;
        case CMD_WRITEWORD_2NDDIE:
            OE_HIGH();
            CE_LOW();
            next_state = S_WRITEWORD_2NDDIE;
            break;
        case CMD_WRITEWORD_UBM:
            OE_HIGH();
            CE_LOW();
            next_state = S_WRITEWORD_WBP;
            break;
        case CMD_WRITEWORD_UBM_2NDDIE:
            OE_HIGH();
            CE_LOW();
            next_state = S_WRITEWORD_WBP_2NDDIE;
            break;
        default:
            /*
             * This is for commands that pack an argument into
             * the command byte itself.
             */
            if ((rc >> 7) == 1) {
                /*
                 * Address - Receive address byte 3
                 */
                update_address3((rc << 1) >> 1);
                next_state = S_ADDR2;
            } else if ((rc >> 6) == 1) {
                /*
                 * Delay - The teensy implementation would loop
                 * through the main FSM loop for as many iterations
                 * as sent. For the pico, 1 "cycle" == 1us.
                 */
                sleep_us(rc & 0x3f);
            }
            break;
        }
    }

    return (next_state);
}


enum fsm_states_e run_reading_state(enum fsm_states_e current_state)
{
    uint16_t data_pins;

    set_data_pins_input();

    if (current_state == S_READING_BSS_WORD) {
        OE_LOW();
        DELAY_100_NS();
        data_pins = get_data_pins();
        OE_HIGH();

        usb_serial_putchar((char)(data_pins & 0xff00 >> 8));
        usb_serial_putchar((char)(data_pins & 0xff));
    } else {
        uint32_t bss_size;
        char     buf_read[64];
        uint8_t  buf_ix = 0;
        uint32_t address = 0;

        if (current_state == S_READING_BSS_4) {
            bss_size = BSS_4;
        } else if (current_state == S_READING_BSS_8) {
            bss_size = BSS_8;
        } else if (current_state == S_READING_BSS_64) {
            bss_size = BSS_64;
        } else if (current_state == S_READING_BSS_128) {
            bss_size = BSS_128;
        } else {
            bss_size = BSS_WORD;
        }

        do {
            OE_LOW();
            DELAY_100_NS();

            data_pins = get_data_pins();
            buf_read[buf_ix++] = data_pins & 0xff00 >> 8;
            buf_read[buf_ix++] = data_pins & 0xff;
            OE_HIGH();

            address_increment_and_update_pins();
            if (buf_ix == 64) {
                usb_serial_write(buf_read, sizeof(buf_read));
                buf_ix = 0;
            }
        } while (address++ != (bss_size/2-1));
    }

    set_data_pins_output();

    return (S_IDLE);
}


enum fsm_states_e run_addr2_state(enum fsm_states_e current_state)
{
    int rc;
    enum fsm_states_e next_state = S_ADDR3;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        update_address2(rc);
    } else {
        next_state = S_ADDR2;
    }

    return (next_state);
}


enum fsm_states_e run_addr3_state(enum fsm_states_e current_state)
{
    int rc;
    enum fsm_states_e next_state = S_IDLE;

    rc = usb_serial_getchar();

    if (rc != PICO_ERROR_TIMEOUT) {
        update_address1(rc);
    } else {
        next_state = S_ADDR3;
    }

    return (next_state);
}


enum fsm_states_e run_write_state(enum fsm_states_e current_state)
{
    enum fsm_states_e next_state = S_IDLE;
    int rc;
    uint16_t data_word = 0;

    do {
    rc = usb_serial_getchar();
        if (rc != PICO_ERROR_TIMEOUT) {
            data_word |= ((uint8_t)rc) << 8;
        }
    } while (rc == PICO_ERROR_TIMEOUT);

    do {
    rc = usb_serial_getchar();
        if (rc != PICO_ERROR_TIMEOUT) {
            data_word |= ((uint8_t)rc);
        }
    } while (rc == PICO_ERROR_TIMEOUT);

    DELAY_100_NS();
    WE_HIGH();

    if (current_state == S_WRITE_INCREMENT) {
        address_increment_and_update_pins();
    }

    return (next_state);
}


enum fsm_states_e run_wait_state(enum fsm_states_e current_state)
{
    enum fsm_states_e next_state = S_IDLE;

    /* Wait 200ns for RYBY to become active */
    DELAY_200_NS();

    do {
        /* Busy wait for RYBY if it's not ready yet */
    } while (!get_RYBY());

    if (current_state == S_WAIT_INCREMENT) {
        address_increment_and_update_pins();
    }

    return (next_state);
}


bool wait_for_ryby_during_write(void)
{
    DELAY_200_NS();

    uint32_t cnt = 0xFFFFFF; /* ~17s on teensy, hopefully long enough on pico */
    do {
        cnt--;
    } while(!get_RYBY() || cnt > 0);

    return (cnt == 0);
}


bool verify(char *buf_to_verify, size_t buf_len)
{
    bool        data_mismatch = false;
    uint16_t    data_word;

    set_data_pins_input();

    while (buf_len && !data_mismatch) {
        OE_LOW();
        DELAY_100_NS();

        data_word = get_data_pins();

        if ((*buf_to_verify++ != (data_word & 0xff00) >> 8) ||
            (*buf_to_verify++ != (data_word & 0xff))) {
            data_mismatch = true;
            OE_HIGH();
        } else {
            OE_HIGH();
            address_increment_and_update_pins();
            buf_len -= 2;
        }
    }

    set_data_pins_output();

    return (data_mismatch);
}


size_t program_buffer_writeword(
    char *buf_write,
    size_t buf_len,
    uint32_t offset_2nddie)
{
    bool    write_timeout = false;
    size_t  i;

    for (i = 0; i < buf_len && !write_timeout; i += 2) {
        /*
         * Erased flash defaults to 0xFFFF, skip those writes
         */
        if (buf_write[i] == 0xFF && buf_write[i+1] == 0xFF) {
            address_increment();
        } else {
            put_address_u32(offset_2nddie | 0x0555);
            put_data_u16(0x00AA);

            put_address_u32(offset_2nddie | 0x02AA);
            put_data_u16(0x0055);

            put_address_u32(offset_2nddie | 0x0555);
            put_data_u16(0x00A0);

            put_address_u32(s_address);
            put_data_u16((buf_write[i] << 8) | buf_write[i+1]);

            address_increment();
            write_timeout = wait_for_ryby_during_write();
        }
    }

    return (i);
}


size_t program_buffer_writeword_ubm(
    char *buf_write,
    size_t buf_len,
    uint32_t offset_2nddie)
{
    bool    write_timeout = false;
    size_t  i;

    /*
     * Enter unlock bypass mode
     */
    put_address_u32(offset_2nddie | 0x0555);
    put_data_u16(0x00AA);

    put_address_u32(offset_2nddie | 0x02AA);
    put_data_u16(0x0055);

    put_address_u32(offset_2nddie | 0x0555);
    put_data_u16(0x0020);

    put_address_u32(s_address);

    /*
     * Write data
     */
    for (i = 0; i < buf_len && !write_timeout; i += 2) {
        /*
         * Erased flash defaults to 0xFFFF, skip those writes
         */
        if (buf_write[i] == 0xFF && buf_write[i+1] == 0xFF) {
            address_increment();
        } else {
            put_data_u16(0x00A0);
            put_data_u16((buf_write[i] << 8) | buf_write[i+1]);

            write_timeout = wait_for_ryby_during_write();
            if (!write_timeout) {
                address_increment_and_update_pins();
            }
        }
    }

    /*
     * Exit unlock bypass mode
     */
    put_data_u16(0x0090);
    put_data_u16(0x0000);

    return (i);
}


size_t program_buffer_writeword_wbp(
    char *buf_write,
    size_t buf_len,
    uint32_t offset_2nddie)
{
    bool        write_timeout = false;
    size_t      i, k;
    uint32_t    buf_start_address;

    /*
     * Write data
     */
    for (i = 0; i < buf_len && !write_timeout; i += 64) {
        buf_start_address = s_address;

        /*
         * Enter write buffer programming mode
         */
        put_address_u32(0x0555);
        put_data_u16(0x00AA);

        put_address_u32(0x02AA);
        put_data_u16(0x0055);

        put_address_u32(buf_start_address);
        put_data_u16(0x0025);

        put_address_u32(buf_start_address);
        put_data_u16(0x001F);

        /*
         * Write to the write buffer
         */
        for (k = 0; k < 64; k += 2) {
            put_data_u16((buf_write[i+k] << 8) | buf_write[i+k+1]);
            address_increment_and_update_pins();
        }

        put_address_u32(buf_start_address);
        put_data_u16(0x0029);

        write_timeout = wait_for_ryby_during_write();
    }

    return (i);
}


enum fsm_states_e run_write_buffer_state(enum fsm_states_e current_state)
{
    enum fsm_states_e   next_state = S_IDLE;
    int                 rc = 0;
    char                buf_write[BSS_4];
    size_t              i = 0;
    uint32_t            offset_2nddie;

    if (current_state == S_WRITEWORD_2NDDIE ||
        current_state == S_WRITEWORD_UBM_2NDDIE) {
        offset_2nddie = 0x400000;
    } else {
        offset_2nddie = 0x0;
    }

    while (i < sizeof(buf_write) && rc != PICO_ERROR_TIMEOUT) {
        rc = usb_serial_getbuf(&buf_write[i], 128);
        if (rc == PICO_ERROR_TIMEOUT) {
            usb_serial_putchar('T');
        }

        i += rc;
    }

    if (i < sizeof(buf_write)) {
        usb_serial_putchar('R');
    } else {
        uint32_t verify_starting_addr = s_address;

        if (current_state == S_WRITEWORD ||
            current_state == S_WRITEWORD_2NDDIE) {
            i = program_buffer_writeword(
                buf_write,
                sizeof(buf_write),
                offset_2nddie);
        } else if (current_state == S_WRITEWORD_UBM ||
            current_state == S_WRITEWORD_UBM_2NDDIE) {
            i = program_buffer_writeword_ubm(
                buf_write,
                sizeof(buf_write),
                offset_2nddie);
        } else if (current_state == S_WRITEWORD_WBP ||
            current_state == S_WRITEWORD_WBP_2NDDIE) {
            i = program_buffer_writeword_wbp(
                buf_write,
                sizeof(buf_write),
                offset_2nddie);
        }

        if (i < sizeof(buf_write)) {
            usb_serial_putchar('T');
        } else if (s_write_verification_enabled) {
            s_address = verify_starting_addr;
            put_address_u32(s_address);

            if (verify(buf_write, sizeof(buf_write))) {
                usb_serial_putchar('V');
            } else {
                usb_serial_putchar('K');
            }
        } else {
            usb_serial_putchar('K');
        }
    }

    return (next_state);
}


enum fsm_states_e run_norway_state_machine(enum fsm_states_e current_state)
{
    enum fsm_states_e next_state = S_IDLE;

    switch (current_state) {
    case S_IDLE:
        next_state = run_idle_state();
        break;
    case S_READING_BSS_4:
    case S_READING_BSS_8:
    case S_READING_BSS_64:
    case S_READING_BSS_128:
    case S_READING_BSS_WORD:
        next_state = run_reading_state(current_state);
        break;
    case S_ADDR2:
        next_state = run_addr2_state(current_state);
        break;
    case S_ADDR3:
        next_state = run_addr3_state(current_state);
        break;
    case S_WRITE:
    case S_WRITE_INCREMENT:
        next_state = run_write_state(current_state);
        break;
    case S_WAIT:
    case S_WAIT_INCREMENT:
        next_state = run_wait_state(current_state);
        break;
    case S_WRITEWORD:
    case S_WRITEWORD_2NDDIE:
    case S_WRITEWORD_UBM:
    case S_WRITEWORD_UBM_2NDDIE:
    case S_WRITEWORD_WBP:
    case S_WRITEWORD_WBP_2NDDIE:
        next_state = run_write_buffer_state(current_state);
        break;
    }

    return (next_state);
}


int main(void)
{
    enum fsm_states_e current_fsm_state = S_IDLE;

    stdio_init_all();
    systick_timer_init();

    /* Don't lock the flash bus unless we're connected to a computer */
    release_pins();
    wait_for_usb_serial_connection();
    init_pins();

    while (1) {
        usb_serial_flush();

        while (usb_serial_connected()) {
            current_fsm_state = run_norway_state_machine(current_fsm_state);
        }
    }

    return 0;
}
