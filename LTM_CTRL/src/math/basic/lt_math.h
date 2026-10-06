#ifndef LT_MATH_H
#define LT_MATH_H

#include <stdint.h>
/*---------------------- 浮点数学函数 ----------------------*/

float lt_sin(float the);                /* 浮点正弦（查表 + 线性插值，输入 0~2π） */
float lt_cos(float the);                /* 浮点余弦（输入 0~2π）*/
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

static inline uint16_t lt_normalize_u16(float angle)  /* 浮点数快速归一化到 [0,65535] ==>对应浮点数（0~2*pi) */
{
    if (angle >= 0 && angle < _2_PI) return (uint16_t)(angle * 10430.378f);
    /* 方法：角度 / (2*PI)，取小数部分，再乘回 2*PI */
    float div = angle * _2_PI_INV;   	/* 乘以 1/(2*PI) */
    int ipart = (int)div;              	/* 取整数部分 */
    float fpart = div - (float)ipart;  	/* 取小数部分（0~1）*/
    
    if (fpart < 0) fpart += 1.0f;     	/* 处理负数 */
    return (uint16_t)(fpart * 65536.0f);
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

#endif