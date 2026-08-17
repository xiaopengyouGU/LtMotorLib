#ifndef LT_NOTCH_H
#define LT_NOTCH_H

#include <stdint.h>

/* 陷波滤波器模块，用于抑制机械共振，电流环专用，支持三个抑制点 
 * 一般 1~2 个齿轮谐振 + 1 个联轴器谐振 */

void lt_notch_init(float ts_s);                              /* 采样周期 (s) */
void lt_notch_set(uint8_t level, float fc_Hz, float Q, float depth_dB);
/* level: 级数 (0~2), fc: 中心频率(Hz), Q: 品质因数(5~20), depth: 陷波深度(dB, 负值) */
void lt_notch_reset(void);                                   /* 清空状态，不改变系数 */
float lt_notch_process(float x);                             /* 执行滤波，返回 y */

#endif