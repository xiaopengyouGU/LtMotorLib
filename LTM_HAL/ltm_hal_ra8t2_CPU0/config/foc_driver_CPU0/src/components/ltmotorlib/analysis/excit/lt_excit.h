#ifndef LT_EXCIT_H
#define LT_EXCIT_H

#include <stdint.h>

void lt_excit_init(float dt, float offset);     /* 初始化，dt(s), offset：固定偏置量 */
/* 启动各类型波形 */
void lt_excit_start_step(float amplitude);      /* 阶跃信号 */
void lt_excit_start_square(float amplitude, float freq_hz); /* 方波信号 */ 
void lt_excit_start_triangle(float v_peak, float accel);    /* 三角波信号 */
void lt_excit_start_sine(float amplitude, float freq_hz, float phase_rad);  /* 正弦波信号 */
void lt_excit_stop(void);                       /* 停止输出（回到 offset） */
/* type = 0:获取当前激励值, type = 1: 去除偏置后激励，2:获取正交激励（cos）*/
float lt_excit_get(uint8_t type);               
float lt_excit_update(void);                    /* 更新并返回当前激励信号 */

#endif