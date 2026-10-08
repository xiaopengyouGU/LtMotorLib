/*
 * 聚合公共头（自动生成自各模块头，勿手改；内部实现仍用模块头）
 */
#ifndef LT_MATH_H__
#define LT_MATH_H__

#include <stdint.h>
/*---------------------- 浮点数学函数 ----------------------*/

float lt_sin(float the);                /* 浮点正弦（查表 + 线性插值，输入 0~2π） */
float lt_cos(float the);                /* 浮点余弦（输入 0~2π） */
void  lt_sin_cos(float *s, float *c, float the);
float lt_atan2(float y, float x);       /* 浮点反正切（atan2） */
float lt_mean(const float *data, uint16_t n);     /* 均值 */
float lt_std(const float *data, uint16_t n);      /* 标准差（样本标准差，除以 n-1） */
void  lt_mean_std(const float *data, uint16_t n, float *mean, float *std);/* 均值 + 标准差 */
float lt_correlation_coeff(const float *x, const float *y, uint16_t n);   /* 相关系数 r = cov(x,y) / (std_x * std_y) */
float lt_normalize_quick(float value, float range);     /* 快速归一化，[0, range) */

/* macro math functions */
#define LOW_PASS_FILTER(X_k,Y_k_1,a) 	((a)*(X_k) + (1.0f - (a))*(Y_k_1))
#define HIGH_PASS_FILTER(X_k,Y_k_1,a)	((1.0f - (a))*(X_k) + (a)*(Y_k_1))
#define CONSTRAINS(x,up,down)			((x) < (down) ? (down) : ((x) > (up) ? (up) : (x)))


/* macro values */
#define _SQRT_3						1.7320508f
#define _SQRT_3_2					0.8660254f		/* sqrt(3)/2 */
#define _SQRT_3_3					0.5773503f		/* sqrt(3)/3 */
#define _SQRT_3_INV					0.5773503f	    /* 1/sqrt(3) */
#define _PI							3.1415926f
#define _2_PI						6.2831853f
#define _PI_2						1.5707963f
#define _PI_3						1.0471976f
#define _PI_6						0.5235988f
#define _2_DIV_3					0.6666667f
#define _PI_3_INV					0.9549297f		/* 1/(pi/3)*/
#define _2_PI_INV  					0.1591549f		/* 1/(2*pi) */


/* 内联优化，加快执行速度 */
static inline float lt_normalize(float angle)		/* 角度归一化[0, 2π)，快速操作 */
{   /* 利用浮点数的整数部分快速截断 */
    if (angle >= 0 && angle < _2_PI) return angle;
    
    /* 方法：角度 / (2*PI)，取小数部分，再乘回 2*PI */
    float div = angle * _2_PI_INV;   	/* 乘以 1/(2*PI) */
    int ipart = (int)div;              	/* 取整数部分 */
    float fpart = div - (float)ipart;  	/* 取小数部分（0~1）*/
    
    if (fpart < 0) fpart += 1.0f;     	/* 处理负数 */
    return fpart * _2_PI;
}

static inline float lt_sqrt(float x)	/* 快速开方, 约 0.175% 最大相对误差 */
{   /* 边界检查，默认输入值是正数 */
    if(x <= 0.0f)       return 0.0f;

    union { float f; uint32_t u; } v;	/* 位表示互转：union 不触碰严格别名 */
    float x2, y;
    const float threehalfs = 1.5f;
    
    x2 = x * 0.5f;
    y  = x;
    v.f = y;                    	/* 浮点数的位表示 */
    v.u = 0x5f3759dfu - (v.u >> 1);	/* 魔法数字：初始猜测 */
    y  = v.f;
    y  = y * (threehalfs - (x2 * y * y));   /* 1次牛顿迭代：~0.15% */

    return y * x;  					/* 乘以 x 得到 √x */
}

static inline float lt_minf(float a, float b) { return a < b ? a : b; }
static inline float lt_maxf(float a, float b) { return a > b ? a : b; }
static inline float lt_absf(float value)      { return (value >= 0.0f) ? value : -value; }

/* 饱和：down ≤ v ≤ up（调用方保证 up ≥ down）。取 up = −down 即对称夹限 */
static inline int32_t lt_clamp_i32(int32_t v, int32_t up, int32_t down)
{
    if (v > up)   return up;
    if (v < down) return down;
    return v;
}

/* 64 位版：乘 / 移位出来的大中间量，先夹回目标量程再降位 */
static inline int64_t lt_clamp_i64(int64_t v, int64_t up, int64_t down)
{
    if (v > up)   return up;
    if (v < down) return down;
    return v;
}

/* 逐位整数平方根：sqrt(v)，v < 2^32（16 次迭代，无除法，冷路径用）*/
uint32_t lt_sqrt_u32(uint32_t v);


/* 相干解调算法模块，测量信号时最好去除直流偏置 
 * count 为整数圈周期时，可自动消除直流偏置量
 */
void lt_demod_init(uint16_t count);             /* count：单次采集的数据点数 */
void lt_demod_reset(void);                      /* 重置累加器（每个频率点开始时调用） */
void lt_demod_add(float ref_sin, float ref_cos, float meas_signal);  /* ref : 参考正余弦信号（-1~1），meas 测量信号（同频）*/
uint8_t lt_demod_is_done(void);                 /* 本次数据采集是否完毕，0：未完毕，1：已完毕 */
void lt_demod_solve(void);                      /* 解算幅值和相位（累加足够点数后调用） */
void lt_demod_get(float *amp, float *phase_deg);/* 获取结果 */

/* 一维线性插值，边界 clamp */
float lt_interp_linear(float x, const float *x_table, const float *y_table, uint16_t n);  
float lt_interp_bilinear(float x, float y,
                         const float *x_table, const float *y_table,
                         const float *z_table,        /* Z_table: 二维表格，输入：&z_table[0][0] */
                         uint16_t nx, uint16_t ny);   /* 二维插值，行优先，边界 clamp */
void lt_interp_hermite3(float t, float p0, float v0, float p1, float v1,  /* t：归一化输入时间 */
                        float *p, float *v);          /* 三次 Hermite 插值，t ∈ [0,1]，输出位置（p）和速度（v）*/
void lt_interp_hermite3_q24(int32_t t_q16, int32_t p0_q24, int32_t v0_q24,   /* t_q16: 0~65536 */
                            int32_t p1_q24, int32_t v1_q24,               /* p/v: Q24 位置/斜率 */
                            int32_t *p_q24, int32_t *v_q24);              /* 输出可空 */

/* 最小二乘线性回归：y = a0 + a1*x1 + a2*x2 + ... + an*xn
 *   x    : 自变量数组，大小为 m x n，行优先存储（x[样本][变量]）
 *   y    : 因变量数组，大小为 m
 *   n    : 自变量个数（1 ~ 3）
 *   m    : 样本点数（必须 >= n + 1）
 *   coeff: 输出回归系数，大小为 n+1（coeff[0] 为常数项 a0）
 *   返回 1：成功，0：失败（样本不足或矩阵奇异）
 */
uint8_t lt_lsq_solve(const float *x, const float *y, uint8_t n, uint16_t m, float *coeff);


#endif /* LT_MATH_H__ */
