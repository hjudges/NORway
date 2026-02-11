#ifndef __MEM_UTILS_H__
#define __MEM_UTILS_H__

#include "pico/stdlib.h"

/**
 * Returns the number of kb (rounded down) free in the heap
 */
uint16_t get_free_heap(void);


#endif /* __MEM_UTILS_H__ */
