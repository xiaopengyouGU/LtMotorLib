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
    float ty = _find_weight_idx(y, &iy, y_table, ny);

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
 * 注：这里的 v0/v1 是对 t 的导数（斜率），不是物理速度；浮点版留作 PC 侧对拍基准。
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

/* 三次 Hermite 插值（纯定点：Q24 位置/斜率，Q16 段内归一化时间）
 *
 *   a = 3·(p1−p0) − 2·v0 − v1
 *   b = −2·(p1−p0) + v0 + v1
 *   p(t) = p0 + t·(v0 + t·(a + t·b))
 *   v(t) = v0 + t·(2a + 3b·t)
 *
 * Horner 展开：b·t 只乘一次（Q40），位置/斜率两路共用，每级右移 16 回到 Q24；
 * 位置再用 2 次乘加、斜率 1 次。
 * 系数与中间量全程 int64，所以 p0/p1 打满 ±128 圈、斜率顶到 Q24 上限也不会溢出；
 * 结果按 int32（= Q24 的 ±128 圈）饱和，不做 UB 的隐式回绕。
 * p_q24 / v_q24 都可以给空：计算不分路，给空就跳过对应输出。
 * 截断误差每级 ≤ 2⁻²⁴ 圈，三级合计 < 2e-7 圈（10000 线编码器上 0.0017 count，实测）。
 * 另一项误差来自 t 本身的 Q16 量化：|dp/dt|×2⁻¹⁷ 圈；真实工况 |dp/dt|≈1 圈/t 时约 0.08 count。
 * 斜率给得过大时三次曲线会过冲（Hermite 固有），调用方靠段长/速度限幅约束 |v|·T 即可。
 */
void lt_interp_hermite3_q24(int32_t t_q16, int32_t p0_q24, int32_t v0_q24,
                            int32_t p1_q24, int32_t v1_q24,
                            int32_t *p_q24, int32_t *v_q24)
{
    int64_t dp = (int64_t)p1_q24 - (int64_t)p0_q24;
    int64_t a  = 3 * dp - 2 * (int64_t)v0_q24 - v1_q24;   /* 2*v0 别在 int32 里乘 */
    int64_t b  = -2 * dp + v0_q24 + v1_q24;
    int64_t bt = b * t_q16;                             /* Q40：两路共用，只乘一次 */
    int64_t h;

    /* p0 + t·(v0 + t·(a + t·b)) */
    h = a + (bt >> 16);
    h = v0_q24 + ((h * t_q16) >> 16);
    h = (int64_t)p0_q24 + ((h * t_q16) >> 16);
    if (p_q24)  *p_q24 = (int32_t)lt_clamp_i64(h, (int64_t)INT32_MAX, (int64_t)INT32_MIN);

    /* v0 + t·(2a + 3b·t) */
    h = 2 * a + ((3 * bt) >> 16);
    h = v0_q24 + ((h * t_q16) >> 16);
    if (v_q24)  *v_q24 = (int32_t)lt_clamp_i64(h, (int64_t)INT32_MAX, (int64_t)INT32_MIN);
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
        uint16_t mid = (uint16_t)((lo + hi) >> 1);  /* 移位性能高于 /2 */
        if (value < table[mid]) hi = mid;
        else lo = mid;
    }
    *idx = lo;
    
    return (value - table[lo]) / (table[lo + 1] - table[lo]);
}
