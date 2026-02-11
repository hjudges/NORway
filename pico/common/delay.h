#ifndef __DELAY_H__
#define __DELAY_H__

#include "pico/stdlib.h"

/* 8ns per tick -> 13 ticks for 104ns delay */
#define DELAY_100_NS()  (systick_delay(13))
/* 8ns per tick -> 25 ticks for 200ns delay */
#define DELAY_200_NS()  (systick_delay(25))

/**
 * Initializes the systick timer
 */
void systick_timer_init(void);

/**
 * Delays the CPU for the number of ticks specified.
 * The default CPU speed is 125MHz (8ns/clock cycle).
 */
void systick_delay(uint16_t ticks);

#endif /* __DELAY_H__ */