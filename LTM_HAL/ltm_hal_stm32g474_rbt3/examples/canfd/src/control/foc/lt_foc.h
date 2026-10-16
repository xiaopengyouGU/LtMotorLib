#ifndef LT_FOC_H
#define LT_FOC_H

#include <stdint.h>
/*------------------------- FOC 算法 ---------------------------------*/

/* 电压调制类型 */
typedef enum {
    FOC_TYPE_SPWM = 0,      /* 标准 SPWM（无零序注入）*/
    FOC_TYPE_SPWM_1,        /* SPWM + 最小值注入 */
    FOC_TYPE_SPWM_2,        /* SPWM + 均值注入 */
    FOC_TYPE_SVPWM,         /* SVPWM（扇区+过调制）*/
	FOC_TYPE_TRAPZ,			/* 六步换相法 （同步整流版）*/
} foc_type_t;

void lt_foc_init(uint8_t type);
void lt_foc_set(uint8_t type);
void lt_foc_process(float  Vd, float Vq, float angle_el);	/* Vd 和 Vq 的范围：[-1, 1]，标准SVPWM实现 */
void lt_foc_process2(float Vd, float Vq, float angle_el);	/* 其余电压调制方法（不包括SVPWM） */
void lt_foc_process3(float Vq, uint8_t hallA, uint8_t hallB, uint8_t hallC); /* 六步换向，配Hall传感器（或过零点信号） */
void lt_foc_get_dutys(float *dutyA, float *dutyB, float *dutyC);    /* 获取计算后的三相占空比 */

/*------------------------- FOC 算法 ---------------------------------*/

#endif