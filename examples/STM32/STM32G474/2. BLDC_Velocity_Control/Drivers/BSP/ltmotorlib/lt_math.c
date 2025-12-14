/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-8-27      Lvtou        the first version
 */
#include "ltmotorlib.h"

/* lookup table trigonometric function implementations are mainly referred to simpleFOC project's foc_utils.h and foc_utils.cpp 
* simpleFOC project's address: https://www.simplefoc.com , thanks for the project contributors!!!
*/

static uint16_t sine_array[65] = {0,804,1608,2411,3212,4011,4808,5602,6393,7180,7962,8740,9512,10279,11039,11793,12540,13279,14010,14733,15447,16151,16846,17531,18205,18868,19520,20160,20788,21403,22006,22595,23170,23732,24279,24812,25330,25833,26320,26791,27246,27684,28106,28511,28899,29269,29622,29957,30274,30572,30853,31114,31357,31581,31786,31972,32138,32286,32413,32522,32610,32679,32729,32758,32768};
/* input angle must be 0~2PI */
float lt_sin(float the)
{
	/* resulting precision compared to stdlib sine is 0.00006480 (RMS difference in range -PI,PI for 3217 steps) */
	int32_t t1, t2;
	unsigned int i = (unsigned int)(the * (64*4*256.0f/_2_PI));
	int frac = i & 0xff;
	i = (i >> 8) & 0xff;
	if (i < 64) 
	{
		t1 = (int32_t)sine_array[i]; t2 = (int32_t)sine_array[i+1];
	}
	else if(i < 128)
	{
		t1 = (int32_t)sine_array[128 - i]; t2 = (int32_t)sine_array[127 - i];
	}
	else if(i < 192)
	{
		t1 = -(int32_t)sine_array[-128 + i]; t2 = -(int32_t)sine_array[-127 + i];
	}
	else 
	{
		t1 = -(int32_t)sine_array[256 - i]; t2 = -(int32_t)sine_array[255 - i];
	}
	return (1.0f/32768.0f) * (t1 + (((t2 - t1) * frac) >> 8));
}
	
/* input angle must be 0~2PI
* cost time:  ~56us (int array)
* precision: +-0.005
*/
float lt_cos(float the)
{
	float the_sin = the + _PI_2;
	the_sin = the_sin > _2_PI ? the_sin - _2_PI : the_sin;
	return lt_sin(the_sin);
}

float lt_atan2(float y, float x)
{
	float abs_y = fabsf(y);
    float abs_x = fabsf(x);
    /* inject FLT_MIN in denominator to avoid division by zero */
    float a = fminf(abs_x, abs_y) / (fmaxf(abs_x, abs_y));
    /* s := a * a */
    float s = a * a;
    /* r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a */
    float r =	((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    /* if |y| > |x| then r := 1.57079637 - r */
    if (abs_y > abs_x) r = 1.57079637f - r;
    /* if x < 0 then r := 3.14159274 - r */
    if (x < 0.0f) r = 3.14159274f - r;
    /* if y < 0 then r := -r */
    if (y < 0.0f) r = -r;

    return r;
}

float lt_normalize(float angle_el)
{ 
	float _r = fmodf(angle_el, _2_PI);
	return _r + (_r < 0 ? _2_PI : 0);
}




