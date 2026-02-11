#include "usb_serial.h"

#include <stdio.h>

#include "tusb.h"


bool usb_serial_connected(void)
{
	return tud_cdc_connected();
}


void wait_for_usb_serial_connection(void)
{
    while(!usb_serial_connected()) {
    }
}


void usb_serial_putchar(char c)
{
    (void)stdio_putchar_raw(c);
}


int usb_serial_getchar(void)
{
    int rc;
    absolute_time_t end_time;
    char input_buf;

    end_time = delayed_by_us(get_absolute_time(), 100000);
    rc = stdio_get_until(&input_buf, sizeof(input_buf), end_time);

    if (rc != PICO_ERROR_TIMEOUT) {
        rc = input_buf;
    }

    return (rc);
}


size_t usb_serial_write(char *buf, size_t len)
{
    return stdio_put_string(
        buf, len,
        false /* newline */,
        false /* cr_translation */);
}


int usb_serial_getbuf(char *input_buf, size_t input_buf_len)
{
    int rc;
    absolute_time_t end_time;

    end_time = delayed_by_us(get_absolute_time(), 100000);
    rc = stdio_get_until(input_buf, input_buf_len, end_time);

    return (rc);
}


void usb_serial_flush(void)
{
    int rc;
    absolute_time_t end_time;

    /* Flush all output */
    stdio_flush();

    /* Flush any waiting input data */
    char flush_buf[8];

    /* Wait up to 10us since the last data to consider the
     * input buffer flushed */
    do {
        end_time = delayed_by_us(get_absolute_time(), 10);
        rc = stdio_get_until(flush_buf, sizeof(flush_buf), end_time);
    } while (rc != PICO_ERROR_TIMEOUT);
}
