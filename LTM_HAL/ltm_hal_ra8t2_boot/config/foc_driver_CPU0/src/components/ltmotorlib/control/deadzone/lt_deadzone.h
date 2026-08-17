#ifndef LT_DEADZONE_H
#define LT_DEADZONE_H

#include <stdint.h>

/*------------------------- 电流极性法死区补偿 ---------------------------------*/

void lt_deadzone_init(float dead_duty, float Ith, float alpha); /* 死区补偿初始化 */
void lt_deadzone_set(float dead_duty, float Ith, float alpha);  /* 设置死区对应占空比（[0-1]），和电流阈值（A） */
void lt_deadzone_compensate(float Ia, float Ib, float Ic); /* 启动死区补偿 */
void lt_deadzone_get(float *dutyA, float *dutyB, float *dutyC); /* 得到补偿后的PWM输出占空比 */

/*------------------------- 电流极性法死区补偿 ---------------------------------*/

#endif