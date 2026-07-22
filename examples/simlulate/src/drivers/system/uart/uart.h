#ifndef SYSTEM_UART_H
#define	SYSTEM_UART_H

#include <stdint.h>

#define  COM_NAME           "\\\\.\\COM2"                           /* 打开的虚拟串口端子 */

void uart_init(void);
void uart_write(uint8_t* buf, uint16_t len);                        /* 串口发送 */
void uart_read(uint8_t* buf, uint16_t *len);                        /* 串口读取 */
void uart_set_rxcall(void(*rxcall)(uint8_t* buf, uint16_t len));    /* 设置串口接收回调 */

#endif
