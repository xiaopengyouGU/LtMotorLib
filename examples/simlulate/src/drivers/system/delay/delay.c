#include "system/delay/delay.h"
#include "system/sys/sys.h"

#include <windows.h>

/* @brief  延时函数初始化 */
void delay_init(void)
{ 
    
}

void delay_us(uint32_t nus)
{
	for(int i = nus * 1000; i > 0; i--);
}

void delay_ms(uint16_t nms)
{
	Sleep(nms);
}
