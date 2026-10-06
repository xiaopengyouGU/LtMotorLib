#ifndef LT_INTERP_H
#define LT_INTERP_H

#include <stdint.h>

/* 一维线性插值，边界 clamp */
float lt_interp_linear(float x, const float *x_table, const float *y_table, uint16_t n);  
float lt_interp_bilinear(float x, float y,
                         const float *x_table, const float *y_table,
                         const float *z_table,        /* Z_table: 二维表格，输入：&z_table[0][0] */
                         uint16_t nx, uint16_t ny);   /* 二维插值，行优先，边界 clamp */
void lt_interp_hermite3(float t, float p0, float v0, float p1, float v1,  /* t：归一化输入时间 */
                        float *p, float *v);          /* 三次 Hermite 插值，t ∈ [0,1]，输出位置（p）和速度（v）*/
#endif