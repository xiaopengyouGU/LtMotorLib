#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "hal_data.h"

#define FOC_COUNT_PERIOD         5000                    /* 定时器一个周期计数值（中心对齐模式下为半个周期）*/
#define FOC_ENABLE_PORT_PIN      BSP_IO_PORT_08_PIN_02   /* 预驱使能引脚 */         
/*---------------------- API ----------------------*/
/* PWM 初始化与占空比设置（TIM1） */
void pwm_init(void);
void pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC); /* 三相占空比（Q15：32768 = 100%）*/
void pwm_start(void);
void pwm_stop(void);

#endif