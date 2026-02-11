#include "delay.h"

#include "hardware/structs/systick.h"

void systick_timer_init(void)
{
    // the clock is set to 125 MHz => 1 tick == 8 ns
    // init SysTick timer
    systick_hw->csr = 0x05; // enable systick at 1 cycle resolution (8ns)
    systick_hw->rvr = 0xffff; // 16 bit
}


void systick_delay(uint16_t ticks)
{
    uint16_t start = systick_hw->cvr;
    while((uint16_t)(start-systick_hw->cvr) < ticks);
}
