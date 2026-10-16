#ifndef SYSTEM_UART_H
#define SYSTEM_UART_H

#include <stdint.h>

/* 注意：uart_port_* 前缀避免与 HPM SDK 的 uart_init/uart_write 重名 */
void uart_port_init(void);
void uart_port_write(uint8_t *buf, uint16_t len);
void uart_port_write_nonblock(uint8_t *buf, uint16_t len);
void uart_port_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len));

#endif
