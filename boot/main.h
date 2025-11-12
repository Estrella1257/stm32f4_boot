#ifndef __BL_MAIN_H
#define __BL_MAIN_H

#include <stdint.h>

#define TRY_CALL(f, ...) \
if (f != NULL)           \
    f(__VA_ARGS__)

#define CHECK_RET(x)    \
do {                    \
    if (!(x)) {	        \
        return;         \
    }                   \
} while (0)

#define CHECK_RETX(x,r) \
do {                    \
    if (!(x)) {	        \
        return (r);     \
    }                   \
} while (0)

#define CHECK_GO(x,e)   \
do {                    \
    if (!(x)) {	        \
        goto e;         \
    }                   \
} while (0)

void bl_delay_init(void);
void bl_delay_ms(uint32_t ms);
uint32_t bl_now(void);


#endif /* __BL_MAIN_H */