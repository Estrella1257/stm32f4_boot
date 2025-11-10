#ifndef __BL_UART_H
#define __BL_UART_H

#include <stdint.h>

typedef void (*bl_uart_recv_cb_t)(uint8_t *data, uint32_t len);

void bl_uart_init(void);
void bl_uart_deinit(void);
void bl_uart_write(uint8_t *data, uint32_t len);
void bl_uart_recv_register(bl_uart_recv_cb_t callback);


#endif /* __BL_UART_H */
