#ifndef __BSP_UART_H
#define	__BSP_UART_H
#include "hal_data.h"
#include <stdio.h>

void uart_init(void);
void uart_write(uint8_t* buf, uint16_t len);                        /* 串口发送，阻塞 */
void uart_write_nonblock(uint8_t* buf, uint16_t len);               /* 串口发送，非阻塞 */
void uart_set_rxcall(void(*rxcall)(uint8_t* buf, uint16_t len));    /* 串口接收回调设置 */

#endif
