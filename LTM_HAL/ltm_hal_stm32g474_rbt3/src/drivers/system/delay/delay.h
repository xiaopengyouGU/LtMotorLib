#ifndef SYSTEM_DELAY_H
#define SYSTEM_DELAY_H

#include <stdint.h>

void delay_init(void);
void delay_ms(uint16_t ms);
void delay_us(uint32_t us);

#endif
