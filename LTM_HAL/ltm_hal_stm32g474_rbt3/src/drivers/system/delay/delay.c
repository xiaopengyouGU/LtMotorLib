/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 延时（DWT 周期计数 + 系统时基）
 */
#include "system/delay/delay.h"
#include "system/sys/sys.h"
#include "stm32g4xx_hal.h"

#define SYS_TICKS_PER_US    170U            /* 170MHz：每微秒计数 */

void delay_init(void)
{
    /* DWT 使能（幂等，system_init 也会调用） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * SYS_TICKS_PER_US;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { ; }   /* 无需担心重装载，uint32_t 可自动回绕 */
}

void delay_ms(uint16_t ms)
{
    /* 基于系统时基死等，只比较低 32 位差值（ms 级不会回绕） */
    volatile uint64_t start = system_get_tick();
    while ((uint32_t)(system_get_tick() - start) < (uint32_t)ms) { ; }
}
