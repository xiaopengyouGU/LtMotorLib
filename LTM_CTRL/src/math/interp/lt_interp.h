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
/* 三次 Hermite 插值（纯定点：Q24 位置/斜率，Q16 段内归一化时间）
 *   t_q16    段内归一化时间：0 = 起点，65536 = 终点
 *   p0,p1    端点位置 Q24（1.0 = 1 圈）
 *   v0,v1    端点斜率 Q24 = dp/dt（对 t∈[0,1] 的导数）
 *            物理速度 v[圈/s] × 段长 T[s] 就是它，两者本来就都是 Q24，不用换算
 *   p_q24    段内位置 Q24；v_q24 段内斜率 Q24（要物理速度再 × 1/T）
 *            两个输出都可给空，给空就跳过对应输出（计算不分路）
 * 起终点位置与斜率严格命中，段间一阶连续；与 Q24 位置/速度规划器同一套标度。
 */
void lt_interp_hermite3_q24(int32_t t_q16, int32_t p0_q24, int32_t v0_q24,
                            int32_t p1_q24, int32_t v1_q24,
                            int32_t *p_q24, int32_t *v_q24);
#endif