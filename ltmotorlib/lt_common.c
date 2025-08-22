/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "math.h"


float _constrains(float val, float up_limit, float down_limit)
{
	if(up_limit <= down_limit) return val;
	
	if(val > up_limit)
	{
		val = up_limit;
	}
	else if(val < down_limit)
	{
		val = down_limit;
	}
	
	return val;
}

float _constrains_dead_region(float val,float up_limit, float down_limit)
{
	if(up_limit <= down_limit) return val;
	
	if(val > up_limit)
	{
		val = up_limit;
	}
	else if(val < down_limit)
	{
		val = 0;
	}
	
	return val;
}

rt_uint8_t _get_rotation_dir(float input)
{
	if(input == 0)
	{
		return 0;
	}
	else if(input > 0) 	/* forward rotate */
	{
		return ROT_FORWARD;
	}
	else   				/* reversal rotate */
	{
		return ROT_REVERSAL;
	}
	
}

int _abs(int i)
{
	if(i < 0) i = -i;
	return i;
}

float _absf(float i)
{
	if(i < 0) i = -i;
	return i;
}

float _max(float a, float b)
{
	if(a >= b) return a;
	else return b;
}

float _min(float a, float b)
{
	if(a < b) return a;
	else return b;
}

float _normalize_angle(float angle_el)
{
	int n = _abs(angle_el/(2*PI));
	if(angle_el <= 0)
	{
		angle_el += (n+1)*2*PI;
	}
	else
	{
		angle_el -= n * 2 *PI;
	}
	angle_el = 2*PI - angle_el;			/* CW --> CCW, electrical angle is in CCW(+), but mechanical angle is in CW(+) */
	return angle_el;
}

void _clark_trans(float fa, float fb, float fc, float*f_alpha, float*f_beta)
{
	/* Clark transform */
	*f_alpha = 2.0f/3 * (fa - 0.5f*fb - 0.5f*fc);
	*f_beta = SQRT_3/3 * (fb - fc);
}

void _park_trans(float f_alpha, float f_beta, float angle, float*fd, float*fq)
{
	/* Park transform */
	float _c = cosf(angle);
	float _s = sinf(angle);
	*fd = _c*f_alpha + _s*f_beta; 
	*fq = -_s*f_alpha + _c*f_beta;
}

void _clark_inv_trans(float f_alpha, float f_beta, float*fa, float*fb, float*fc)
{
	/* Clark inverse transform */
	*fa = f_alpha;
	*fb = -0.5f*f_alpha + SQRT_3/2*f_beta;
	*fc = -0.5f*f_alpha - SQRT_3/2*f_beta;
}

void _park_inv_trans(float fd, float fq, float angle, float*f_alpha, float*f_beta)
{
	/* Park inverse transform */
	float _c = cosf(angle);
	float _s = sinf(angle);
	*f_alpha = _c*fd - _s*fq;
	*f_beta  = _s*fd + _c*fq;
}

