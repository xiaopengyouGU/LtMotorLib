#ifndef LT_MATH_H
#define LT_MATH_H

#include <stdint.h>
#include <stdio.h>
/*---------------------- 浮点数学函数 ----------------------*/

float lt_sin(float the);                /* 浮点正弦（查表 + 线性插值，输入 0~2π） */
float lt_cos(float the);                /* 浮点余弦（输入 0~2π） */
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

/*---------------------- 简易浮点转字符串 ----------------------*/
/* 仅用整数格式化（%ld），不依赖标准库浮点 printf，
   链接时无需 -u _printf_float，可省数 KB 代码。固定 3 位小数。 */
#define LT_FTOAT3(buf, val)                                         \
    do {                                                            \
        float _v = (val);                                           \
        long  _ip = (long)_v;                                       \
        long  _fp = (long)(((_v < 0.0f ? -_v : _v) - (_ip < 0 ? -_ip : _ip)) \
                           * 1000.0f + 0.5f);                       \
        if (_fp >= 1000) { _fp = 0; _ip += (_v < 0 ? -1 : 1); }     \
        if (_ip < 0) sprintf((buf), "-%ld.%03ld", -_ip, _fp);       \
        else         sprintf((buf), "%ld.%03ld", _ip, _fp);         \
    } while (0)


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

	long i;
    float x2, y;
    const float threehalfs = 1.5f;
    
    x2 = x * 0.5f;
    y  = x;
    i  = *(long *)&y;           	/* 浮点数的位表示 */
    i  = 0x5f3759df - (i >> 1); 	/* 魔法数字：初始猜测 */
    y  = *(float *)&i;
    y  = y * (threehalfs - (x2 * y * y));   /* 1次牛顿迭代：~0.15% */

    return y * x;  					/* 乘以 x 得到 √x */
}

static inline float lt_minf(float a, float b)
{
    return a < b ? a : b;
}

static inline float lt_maxf(float a, float b)
{
    return a > b ? a : b;
}

static inline float lt_absf(float value){
    return (value >= 0.0f) ? value : -value;
}

#endif
