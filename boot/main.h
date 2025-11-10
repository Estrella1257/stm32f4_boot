#ifndef __BL_MAIN_H
#define __BL_MAIN_H

#include <stdint.h>

void bl_delay_init(void);
void bl_delay_ms(uint32_t ms);
uint32_t bl_now(void);


#endif /* __BL_MAIN_H */