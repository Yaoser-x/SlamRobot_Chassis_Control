#ifndef HOST_MAIN_H
#define HOST_MAIN_H

#include <stdint.h>

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t primask);

#endif
