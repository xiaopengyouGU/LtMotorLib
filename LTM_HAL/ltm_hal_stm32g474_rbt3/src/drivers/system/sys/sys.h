#ifndef SYSTEM_SYS_H
#define SYSTEM_SYS_H

#include <stdint.h>

void system_init(void);
uint64_t system_get_tick(void);
uint64_t system_get_ms(void);
uint64_t system_get_us(void);
void system_set_callback(void (*callback)(void));   /* 1ms 中断回调 */
void system_reset(void);

#endif
