#include "system/delay/delay.h"
#include "system/sys/sys.h"

static uint32_t g_fac_us = 0;           /* 每微秒所需的 SysTick 计数值 */


/**
 * @brief  延时函数初始化
 * @param  hclk_mhz: HCLK 时钟频率，单位 MHz
 */
void delay_init(void)
{ 
    g_fac_us = SystemCoreClock / 1000000;       /* 获取 1us 需要的 tick 数 */
}

/**
 * @brief  微秒级延时（阻塞式，高精度）
 * @param  nus: 延时的微秒数
 */
void delay_us(uint32_t nus)
{
	R_BSP_SoftwareDelay(nus, BSP_DELAY_UNITS_MICROSECONDS);
}

/**
 * @brief  毫秒级延时（阻塞式）
 * @param  nms: 延时的毫秒数
 */
void delay_ms(uint16_t nms)
{
	R_BSP_SoftwareDelay(nms, BSP_DELAY_UNITS_MILLISECONDS);
}
