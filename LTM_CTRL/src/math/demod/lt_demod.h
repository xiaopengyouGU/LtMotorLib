#ifndef LT_DEMOD_H
#define LT_DEMOD_H

#include <stdint.h>

/* 相干解调算法模块，测量信号时最好去除直流偏置 
 * count 为整数圈周期时，可自动消除直流偏置量
 */
void lt_demod_init(uint16_t count);             /* count：单次采集的数据点数 */
void lt_demod_reset(void);                      /* 重置累加器（每个频率点开始时调用） */
void lt_demod_add(float ref_sin, float ref_cos, float meas_signal);  /* ref : 参考正余弦信号（-1~1），meas 测量信号（同频）*/
uint8_t lt_demod_is_done(void);                 /* 本次数据采集是否完毕，0：未完毕，1：已完毕 */
void lt_demod_solve(void);                      /* 解算幅值和相位（累加足够点数后调用） */
void lt_demod_get(float *amp, float *phase_deg);/* 获取结果 */

#endif