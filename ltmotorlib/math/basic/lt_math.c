/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-8-27      Lvtou        The first version
 */
#include "math/basic/lt_math.h"
#include <math.h>

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
