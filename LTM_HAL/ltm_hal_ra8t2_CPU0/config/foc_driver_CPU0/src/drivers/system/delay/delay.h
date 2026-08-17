#ifndef SYSTEM_DELAY_H
#define SYSTEM_DELAY_H

#include <stdint.h>

void delay_init(void); 	           /* 延时初始化 */
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);

#endif