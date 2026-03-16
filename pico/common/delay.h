#ifndef __DELAY_H__
#define __DELAY_H__

#include "pico/stdlib.h"

/* 6.66ns per tick -> 6 ticks for 40ns delay */
#define DELAY_40_NS()  (systick_delay(6))
/* 6.66ns per tick -> 15 ticks for 100ns delay */
#define DELAY_100_NS()  (systick_delay(15))
/* 6.66ns per tick -> 25 ticks for 200ns delay */
#define DELAY_200_NS()  (systick_delay(30))

/**
 * Initializes the systick timer
 */
void systick_timer_init(void);

/**
 * Delays the CPU for the number of ticks specified.
 * The default CPU speed is 150MHz (6.66ns/clock cycle).
 */
void systick_delay(uint16_t ticks);

#endif /* __DELAY_H__ */