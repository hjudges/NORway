#ifndef __USB_SERIAL_H__
#define __USB_SERIAL_H__

#include "pico/stdlib.h"


/**
 * @return true if USB Serial is connected, false otherwise
 */
bool usb_serial_connected(void);


/**
 * Waits indefinitely until USB Serial connectivity is established
 * with the host.
 */
void wait_for_usb_serial_connection(void);


/**
 * Sends a single character to the host.
 */
void usb_serial_putchar(char c);


/**
 * Waits for a character to be received by the serial port.
 * Will wait up to 100ms for a character to be received -
 * needed because the python script on the host is much slower
 * to send data than the pico can receive it.
 *
 * @return < 0 on error, or the character received
 */
int usb_serial_getchar(void);


/**
 * Writes len characters from buf to the host.
 *
 * @return < 0 on error, or the number of characters transmitted
 */
size_t usb_serial_write(char *buf, size_t len);


/**
 * Similar to usb_serial_getchar, this waits up to 100ms
 * to receive input_buf_len characters and stores them in input_buf.
 *
 * @return < 0 on error, or the number of characters received
 */
int usb_serial_getbuf(char *input_buf, size_t input_buf_len);


/**
 * Flushes both RX and TX buffers for serial IO. Waits up to 10us
 * for the last data RX to consider it flushed.
 */
void usb_serial_flush(void);


#endif /* __USB_SERIAL_H__ */
