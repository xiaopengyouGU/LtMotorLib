#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "hal_data.h"

#define FOC_POS_DIVIDER          3                       /* 位置环分频系数：3kHz / 3 = 1kHz */
#define FOC_COUNT_PERIOD         5000                    /* 定时器一个周期计数值（中心对齐模式下为半个周期）*/
/*---------------------- API ----------------------*/
/* PWM 初始化与占空比设置（TIM1） */
void foc_pwm_init(void);
void foc_pwm_set_duty(float dutyA, float dutyB, float dutyC); /* 设置三相占空比, duty: 0.0~1.0 */
void foc_pwm_start(void);
void foc_pwm_stop(void);

/* 辅助定时器初始化与回调设置 */
void foc_aux_timer_init(void);
void foc_set_speed_callback(void (*callback)(void));     /* 速度环回调（3kHz） */
void foc_set_pos_callback(void (*callback)(void));       /* 位置环回调（1kHz） */

#endif