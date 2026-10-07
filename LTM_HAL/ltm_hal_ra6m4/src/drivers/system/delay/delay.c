#include "system/delay/delay.h"
#include "system/sys/sys.h"

void delay_init(void)
{ 
    /* 直接用 FSP 的延时函数 */
}

void delay_us(uint32_t nus)
{
	R_BSP_SoftwareDelay(nus, BSP_DELAY_UNITS_MICROSECONDS);
}

void delay_ms(uint16_t nms)
{
	R_BSP_SoftwareDelay(nms, BSP_DELAY_UNITS_MILLISECONDS);
}
