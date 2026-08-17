#include "math/interp/lt_interp.h"
#include "math/basic/lt_math.h"

/* 二分法快速定位插值权重与下标 */
static float _find_weight_idx(float value, uint16_t *idx, const float *table, uint16_t n);

/* 一维线性插值法 */
float lt_interp_linear(float x, const float *x_table, const float *y_table, uint16_t n)
{
    uint16_t idx;
    float t = _find_weight_idx(x, &idx, x_table, n);
    return y_table[idx] + (y_table[idx + 1] - y_table[idx]) * t;
}

/* 二维线性插值法 */
float lt_interp_bilinear(float x, float y,
                         const float *x_table, const float *y_table,
                         const float *z_table,
                         uint16_t nx, uint16_t ny)
{
    uint16_t ix, iy;
    float tx = _find_weight_idx(x, &ix, x_table, nx);
    float ty = _find_weight_idx(y, &ix, y_table, ny);

    float z00 = z_table[ix * ny + iy];
    float z10 = z_table[(ix + 1) * ny + iy];
    float z01 = z_table[ix * ny + iy + 1];
    float z11 = z_table[(ix + 1) * ny + iy + 1];

    float z0 = z00 + (z10 - z00) * tx;      /* 底边 */
    float z1 = z01 + (z11 - z01) * tx;      /* 顶边 */
    return z0 + (z1 - z0) * ty;             /* 纵（Y）向插值 */
}

/* 三次 Hermite 插值，归一化时间 t ∈ [0,1]，输出位置（p）和速度（v）
 * 差值多项式：p(t) = h00(t)·p0 + h10(t)·v0 + h01(t)·p1 + h11(t)·v1
 * 生成的平滑插值曲线在起点和终点同时满足位置和速度要求，
 * 常用于伺服 CSP和CSV 模式 
 */
void lt_interp_hermite3(float t, float p0, float v0, float p1, float v1,
                        float *p, float *v)
{
    float t2 = t * t;
    float t3 = t2 * t;

    /* 四个基函数 */
    float h00 =  2.0f * t3 - 3.0f * t2 + 1.0f;
    float h10 =         t3 - 2.0f * t2 + t;
    float h01 = -2.0f * t3 + 3.0f * t2;
    float h11 =         t3 -        t2;

    /* 基函数的导数 */
    float h00d =  6.0f * t2 - 6.0f * t;
    float h10d =  3.0f * t2 - 4.0f * t + 1.0f;
    float h01d = -6.0f * t2 + 6.0f * t;
    float h11d =  3.0f * t2 - 2.0f * t;

    if(p) *p = h00 * p0 + h10 * v0 + h01 * p1 + h11 * v1;
    if(v) *v = h00d * p0 + h10d * v0 + h01d * p1 + h11d * v1;
}

/***********************************************************************************/
/* 二分法快速定位插值权重与下标 */
static float _find_weight_idx(float value, uint16_t *idx, const float *table, uint16_t n)
{
    if (value <= table[0]) {
        *idx = 0;
        return 0.0f;
    }
    if (value >= table[n - 1]) {
        *idx = n - 2;
        return 1.0f;
    }

    uint16_t lo = 0, hi = n - 1;
    while (hi > lo) {
        uint16_t mid = (lo + hi) >> 1;      /* 移位性能高于 /2 */
        if (value < table[mid]) hi = mid;
        else lo = mid;
    }
    *idx = lo;
    
    return (value - table[lo]) / (table[lo + 1] - table[lo]);
}
