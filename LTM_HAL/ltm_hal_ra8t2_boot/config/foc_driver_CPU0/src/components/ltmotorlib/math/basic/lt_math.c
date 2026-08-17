/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-8-27      Lvtou        The first version
 * 2026-7-25      Lvtou        添加均值和标准差计算实现
 */
#include "math/basic/lt_math.h"

/*---------------------- Q15 正弦表（原始） ----------------------*/
static const uint16_t sine_array_q15[65] = {
    0,    804,  1608, 2411, 3212, 4011, 4808, 5602,
    6393, 7180, 7962, 8740, 9512, 10279,11039,11793,
    12540,13279,14010,14733,15447,16151,16846,17531,
    18205,18868,19520,20160,20788,21403,22006,22595,
    23170,23732,24279,24812,25330,25833,26320,26791,
    27246,27684,28106,28511,28899,29269,29622,29957,
    30274,30572,30853,31114,31357,31581,31786,31972,
    32138,32286,32413,32522,32610,32679,32729,32758,
    32768
};

/*---------------------- 浮点正弦（查表） ----------------------*/
#define Q15_INV                 3.0517578e-5f   /* 1.0f / 32768.0f */
#define ANGLE_TO_INDEX_SCALE    10430.378f      /* 65536.0f / (2*PI) ，用于角度转查表索引 */

float lt_sin(float the)
{
    int32_t t1, t2;
    unsigned int i = (unsigned int)(the * ANGLE_TO_INDEX_SCALE);
    int frac = i & 0xff;
    i = (i >> 8) & 0xff;

    if (i < 64) {
        t1 = (int32_t)sine_array_q15[i];
        t2 = (int32_t)sine_array_q15[i+1];
    } else if (i < 128) {
        t1 = (int32_t)sine_array_q15[128 - i];
        t2 = (int32_t)sine_array_q15[127 - i];
    } else if (i < 192) {
        t1 = -(int32_t)sine_array_q15[-128 + i];
        t2 = -(int32_t)sine_array_q15[-127 + i];
    } else {
        t1 = -(int32_t)sine_array_q15[256 - i];
        t2 = -(int32_t)sine_array_q15[255 - i];
    }
    return Q15_INV * (t1 + (((t2 - t1) * frac) >> 8));
}

float lt_cos(float the)
{
    float the_sin = the + _PI_2;
    the_sin = (the_sin > _2_PI) ? the_sin - _2_PI : the_sin;
    return lt_sin(the_sin);
}

/*---------------------- 浮点反正切 ----------------------*/
float lt_atan2(float y, float x)
{
    float abs_y = lt_absf(y);
    float abs_x = lt_absf(x);
    float a = lt_minf(abs_x, abs_y) / (lt_maxf(abs_x, abs_y) + 1e-10f); /* 防止除零 */
    float s = a * a;
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    if (abs_y > abs_x) r = 1.57079637f - r;
    if (x < 0.0f) r = 3.14159274f - r;
    if (y < 0.0f) r = -r;
	
	//将 [-π, π] 映射到 [0, 2π)
    if (r < 0.0f) r += _2_PI;
    return r;
}


/*----------------------均值与标准差计算-------------------*/
float lt_mean(const float *data, uint16_t n)
{
    if (!data || n == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) sum += data[i];
    return sum / (float)n;
}

float lt_std(const float *data, uint16_t n)
{
    if (!data || n < 2) return 0.0f;
    float mean = lt_mean(data, n);
    float sq_sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float diff = data[i] - mean;
        sq_sum += diff * diff;
    }
    return lt_sqrt(sq_sum / (float)(n - 1));
}

void lt_mean_std(const float *data, uint16_t n, float *mean, float *std)
{
    if (!data || n == 0) {
        if (mean) *mean = 0.0f;
        if (std)  *std  = 0.0f;
        return;
    }
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) sum += data[i];
    float mu = sum / (float)n;
    float sq_sum = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        float diff = data[i] - mu;
        sq_sum += diff * diff;
    }
    if (mean) *mean = mu;
    if (std)  *std  = (n > 1) ? lt_sqrt(sq_sum / (float)(n - 1)) : 0.0f;
}

/* 相关系数 r = cov(x,y) / (std_x * std_y) */
float lt_correlation_coeff(const float *x, const float *y, uint16_t n)
{
    if (n < 2) return 0.0f;
    
    float mean_x, std_x, mean_y, std_y;
    lt_mean_std(x, n, &mean_x, &std_x);
    lt_mean_std(y, n, &mean_y, &std_y);
    
    if (std_x < 1e-12f || std_y < 1e-12f) return 0.0f;
    
    float cov = 0.0f;
    for (uint16_t i = 0; i < n; i++) {
        cov += (x[i] - mean_x) * (y[i] - mean_y);
    }
    cov /= (float)(n - 1);
    
    return cov / (std_x * std_y);
}

float lt_normalize_quick(float value, float range)  /* 快速归一化，[0, range) */
{ 
    if(range <= 0.0f)               return 0.0f;
    if(value >= 0 && value < range) return value;
    /* 利用浮点数的整数部分快速截断 */
    /* 方法：value / range，取小数部分，再乘回 range */
    float div = value / range;   	    /* 乘以 1/range */
    int ipart = (int)div;              	/* 取整数部分 */
    float fpart = div - (float)ipart;  	/* 取小数部分（0~1）*/
    
    if (fpart < 0) fpart += 1.0f;     	/* 处理负数 */
    return fpart * range;
}