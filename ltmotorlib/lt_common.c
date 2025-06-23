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


rt_uint8_t _get_quard(int x_pos, int y_pos, rt_uint8_t dir)
{
	if(x_pos > 0)
	{
		if(y_pos > 0)	return 0;			/* 1th quadrant */
		else if(y_pos < 0)	return 3;		/* 4th quadrant */
		else		/* in + X axis */
		{
			if(dir == DIR_CCW) return 0;	/* 1th quadrant */
			else			   return 3;	/* 4th quadrant */
		}
	}
	else if(x_pos < 0)
	{
		if(y_pos > 0)	return 1;			/* 2th quadrant */
		else if(y_pos < 0)	return 2;		/* 3th quadrant */
		else		/* in - X axis */
		{
			if(dir == DIR_CCW) return 2;	/* 3th quadrant */
			else			   return 1;	/* 2th quadrant */
		}
	}
	else
	{				/* in + Y axis*/
		if(y_pos > 0)
		{
			if(dir == DIR_CCW) return 1;	/* 2th quadrant */
			else			   return 0;	/* 1th quadrant */
		}
		else
		{			/* in - Y axis*/
			if(dir == DIR_CCW) return 3;	/* 4th quadrant */
			else			   return 2;	/* 3th quadrant */
		}
	}
}

/* check whether start pos and end pos in the same quarent and circle */
rt_uint8_t _check_circular_pos(int x_start, int y_start, int x_end, int y_end)
{
	int tmp = sqrtf(x_start*x_start + y_start*y_start);/* get radius */
	float ref = tmp/200;
	tmp -= sqrtf(x_end*x_end + y_end*y_end);			
	/* if radius bias are lower than %0.5, we regard that two points are in the same circle */
	if(tmp >= ref || tmp <= -ref) return 0;	
	
	return 1;
}


int _abs(int i)
{
	if(i < 0) i = -i;
	return i;
}

int _abs_plus_1(int i,rt_uint8_t dir)
{
	if(i < 0)
	{
		return i - 1;
	}
	else if (i < 0)
	{
		return i + 1;
	}
	else
	{
		if(dir > 0)
			return i + 1;
		else
			return i - 1;
	}
}

int _abs_sub_1(int i)
{
	if(i < 0)
	{
		return i + 1;
	}
	else
	{
		return i - 1;
	}	
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

rt_uint8_t _get_center(int x_start, int y_start, int x_end, int y_end,rt_uint16_t r, rt_uint8_t dir, float*x_center, float* y_center)
{
	/* get center of a circle */
	float x_mid = (x_start + x_end)/2.0f;
	float y_mid = (y_start + y_end)/2.0f;
	float dist = (x_end - x_start) * (x_end - x_start);
	float h,vec_x,vec_y;
	dist += (y_end - y_start) * (y_end - y_start);
	dist = sqrtf(dist);
	
	if (dist > 2*r) return 0;		/* radius is too small */
	/* calculate normal vector */
	vec_x = -1.0f * (y_end - y_start)/dist;
	vec_y = 1.0f*(x_end - x_start)/dist;
    
    /* select center based on dir--> clockwise, counter-clockwise; */
    h = sqrtf(r*r - (dist/2)*(dist/2));
    if (dir == DIR_CCW)
	{
		*x_center = x_mid + h * vec_x;
		*y_center = y_mid + h * vec_y;
	}
	else
	{
		*x_center = x_mid - h * vec_x;
		*y_center = y_mid - h * vec_y;
	}
	return 1;
}

rt_uint8_t _check_end(int x_pos, int y_pos, int x_target, int y_target,rt_uint8_t exact)
{
	rt_uint32_t dist = _abs(x_pos - x_target) + _abs(y_pos - y_target);
	if(exact == 1)
	{
		if(dist == 0 ) return 1;
	}
	else
	{
		if(dist <= 2) return 1;			/* reach target position */
	}
	
	return 0;
}

rt_int32_t _get_count(float curr, float last,rt_int32_t count, rt_uint8_t volt)
{
	float dist = curr - last;
	if (dist == 0) return count;
	
	if(volt)			/* value increase counter-clockwise */
	{
		if(dist < - PI)
		{
			count = count - 1;		/* finish a rotation */
		}
		else if(dist >= PI)	/* dist >= PI */
		{
			count = count + 1;
		}
	}
	else				/* value increase clockwise */
	{
		if (dist < - PI)
		{
			count = count + 1;
		}
		else if(dist >= PI)	/* dist >= PI */
		{
			count = count - 1;
		}
	}
	return count;
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

