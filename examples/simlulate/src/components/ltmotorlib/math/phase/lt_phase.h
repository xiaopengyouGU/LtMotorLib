#ifndef LT_PHASE_H
#define LT_PHASE_H

#include <stdint.h>

/* 过零点相位计算 ：° ，len：数据长度，m： 注入信号频率/基频 */
float lt_phase_calculate(const float* buf, uint16_t len, uint16_t m);

#endif