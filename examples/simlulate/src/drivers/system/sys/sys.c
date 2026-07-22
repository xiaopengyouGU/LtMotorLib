#include "system/sys/sys.h"
#include <errno.h>
/* 系统时基：单位为 ms 
*  必须加 volatile 否则可能被编译器优化掉（高频中断调用）
*/
volatile static uint64_t sys_tick_ms = 0;       /* 系统运行时基，单位：ms */
static uint32_t g_fac_us        = 0;            /* 每微秒所需的 SysTick 计数值 */
static float    g_fac_us_inv    = 0;            /* us 换算系数，避免除法 */ 
static float    g_fac_ns_inv    = 0;            /* ns 换算系数，避免除法 */

static void (*sys_callback)(void) = NULL;       /* 系统中断回调函数：1ms */

// /**
// * @brief  SysTick的中断服务函数
// * @param  无
// * @retval 无
// */
// extern void SysTick_Handler(void); //需要先extern声明一下避免编译器警告
// void SysTick_Handler(void)
// {   /* 时基递增，不用担心溢出 */
//     sys_tick_ms++;     
//     if(sys_callback) sys_callback();  /* 定时调用系统中断回调 */
// }

// void system_init(void)
// {
//     SystemInit();                               /* 手动调用系统初始化函数，配置所有外设 */
//     R_IOPORT_Open(&IOPORT_CFG_CTRL, &IOPORT_CFG_NAME);

//     uint32_t period = SystemCoreClock / 1000;   /* Systick 中断：1ms */
//     SysTick_Config(period);                     /* 1ms 定时中断 */  
//     g_fac_us = SystemCoreClock / 1000000;         /* 每微秒所需的 SysTick 计数值 */
//     g_fac_us_inv  = 1.0f   / g_fac_us;            /* 获取 us 换算系数，避免后续的除法运算 */
//     g_fac_ns_inv  = 1000.f / g_fac_us;            /* 获取 ns 换算系数，避免后续的除法运算 */
//     // 不需要额外操作了，系统配置已经自动调用了
// }

// uint64_t system_get_tick()
// {
//     return sys_tick_ms;
// }

// uint64_t system_get_ms()
// {
//     return sys_tick_ms;
// }

// uint64_t system_get_us()                       /* 该函数主要用于系统时间补偿：us */
// {   
//     /* 一般仅在高优先级中断中调用，不用担心被 SysTick 打断 */
//     uint32_t val    = SysTick->VAL;            /* SysTick 是向下计数的，一个系统时钟周期减 1 */
//     uint32_t reload = SysTick->LOAD;           /* 重装载值，对应 1ms */
//     uint32_t elapsed_tick = reload - val;      /* 已经走过的 tick 数 */
//     uint64_t ms     = sys_tick_ms;             /* 系统时基：ms */ 
    
//     /* 遇到极端情况：初次读取时，SysTick->VAL 恰好由 0 重装载到 LOAD，即 val = LOAD */
//     if(val == reload){                    
//         ms++;                                  /* 此时补偿 1ms, 因为 elapsed_tick 极小 */
//     }      
    
//     return  ms * 1000 + elapsed_tick * g_fac_us_inv; /* 获取系统当前的 us 值 */
// }

// uint64_t system_get_ns()                       /* 该函数主要用于系统时间补偿：ns */
// {   
//     /* 一般仅在高优先级中断中调用，不用担心被 SysTick 打断 */
//     uint32_t val    = SysTick->VAL;            /* SysTick 是向下计数的，一个系统时钟周期减 1 */
//     uint32_t reload = SysTick->LOAD;           /* 重装载值，对应 1ms */
//     uint32_t elapsed_tick = reload - val;      /* 已经走过的 tick 数 */
//     uint64_t ms     = sys_tick_ms;             /* 系统时基：ms */ 
    
//     /* 遇到极端情况：初次读取时，SysTick->VAL 恰好由 0 重装载到 LOAD，即 val = LOAD */
//     if(val == reload){                    
//         ms++;                                  /* 此时补偿 1ms, 因为 elapsed_tick = 0 */
//     }      
    
//     return  ms * 1000000 + elapsed_tick * g_fac_ns_inv; /* 获取系统当前的 us 值 */
// }

void system_set_callback(void(*callback)(void))      /* 设置系统中断回调：1ms */
{
    if(!callback)       return;         /* 判空 */
    sys_callback = callback;
}

