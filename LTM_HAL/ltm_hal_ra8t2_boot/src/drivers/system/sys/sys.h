#ifndef SYSTEM_SYS_H
#define SYSTEM_SYS_H

#include "hal_data.h"      
#include <stdint.h>

/* 系统初始化函数 */
void system_init(void);
uint64_t system_get_tick();         /* 获取当前系统时基：ms */
uint64_t system_get_ms();           /* 获取系统运行时间：ms */
uint64_t system_get_us();           /* 获取系统运行时间：us */
uint64_t system_get_ns();           /* 获取系统运行时间：ns */
void system_set_callback(void(*callback)(void));    /* 设置系统回调函数：1ms */

#endif