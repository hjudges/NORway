#include "mem_utils.h"

#include <malloc.h>


static uint32_t get_total_heap(void)
{
   extern char __StackLimit, __bss_end__;

   return &__StackLimit  - &__bss_end__;
}


uint16_t get_free_heap(void)
{
   struct mallinfo m = mallinfo();

   return (uint16_t)((get_total_heap() - m.uordblks) >> 16);
}
