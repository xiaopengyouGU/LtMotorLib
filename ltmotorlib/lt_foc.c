/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "math.h"

static rt_int8_t _6_table[6][3] = { {1,-1,0},
									{1,0,-1},
									{0,1,-1},
									{-1,1,0},
									{-1,0,1},
									{0,-1,1}
};

static float _normalize_angle(float angle_el);
lt_foc_t lt_foc_create()
{
	lt_foc_t foc;
	foc = rt_malloc(sizeof(struct lt_foc_object));
	if(foc == RT_NULL) return RT_NULL;
	rt_memset(foc,0,sizeof(struct lt_foc_object));
	return foc;
}

rt_err_t lt_foc_delete(lt_foc_t foc)
{
	RT_ASSERT(foc != RT_NULL);
	rt_free(foc);
	return RT_EOK;
}

void lt_foc_set_maxval(lt_foc_t foc,float max_val)
{
	RT_ASSERT(foc != RT_NULL);
	foc->max_val = _absf(max_val);
}

void lt_foc_set_type(lt_foc_t foc,rt_uint8_t type)
{
	RT_ASSERT(foc != RT_NULL);
	foc->type = type;
}

void lt_foc_process(lt_foc_t foc,float fd, float fq, float angle_el,float*duty_A, float*duty_B, float*duty_C)
{
	RT_ASSERT(foc != RT_NULL);
	if(foc->max_val == 0) return;
	/* check angle_el */
	float f_alpha, f_beta;
	float _c,_s;
	float fa,fb,fc;
	float center = foc->max_val/2;		
	float u_min, u_max,tmp;					
	rt_uint8_t i;
	fq = _constrains(fq,center,-center);	/* avoid output saturation */
	fd = _constrains(fd,center,-center);
	
	switch(foc->type)
	{
		case FOC_TYPE_6_TRAPZOID:
		{
			fq = -fq;
			i = (_normalize_angle(angle_el + PI/6) /(PI/3)); 
			
			if(i >= 6) i %= 6;
			fa = fq * _6_table[i][0];
			fb = fq * _6_table[i][1];
			fc = fq * _6_table[i][2];
			
			break;
		}
		case FOC_TYPE_SPWM:
		case FOC_TYPE_SVPWM:			   /* we use center modulation */
		{
			angle_el = _normalize_angle(angle_el);
			/* Park inverse transform */
			_c = cosf(angle_el);
			_s = sinf(angle_el);
			f_alpha = _c*fd - _s*fq;
			f_beta  = _s*fd + _c*fq;
			/* Clark inverse transform */
			fa = f_alpha;
			fb = -0.5f*f_alpha + SQRT_3/2*f_beta;
			fc = -0.5f*f_alpha - SQRT_3/2*f_beta;
			if(foc->type == FOC_TYPE_SVPWM)
			{
				u_min = _min(fa,_min(fb,fc));
				u_max = _max(fa,_max(fb,fc));
				center -= (u_max + u_min)/2;
			}
			break;
		}
		default:break;
	}
	
	fa += center;
	fb += center;
	fc += center;
	/* return duty cycle */
	*duty_A = fa/foc->max_val;
	*duty_B = fb/foc->max_val;
	*duty_C = fc/foc->max_val;
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

//void _clark_trans(float fa, float fb, float fc, float*f_alpha, float*f_beta)
//{
//	/* Clark transform */
//	*f_alpha = 2.0f/3 * (fa - 0.5f*fb - 0.5f*fc);
//	*f_beta = SQRT_3/3 * (fb - fc);
//}

//void _park_trans(float f_alpha, float f_beta, float angle, float*fd, float*fq)
//{
//	/* Park transform */
//	float _c = cosf(angle);
//	float _s = sinf(angle);
//	*fd = _c*f_alpha + _s*f_beta; 
//	*fq = -_s*f_alpha + _c*f_beta;
//}

//void _clark_inv_trans(float f_alpha, float f_beta, float*fa, float*fb, float*fc)
//{
//	/* Clark inverse transform */
//	*fa = f_alpha;
//	*fb = -0.5f*f_alpha + SQRT_3/2*f_beta;
//	*fc = -0.5f*f_alpha - SQRT_3/2*f_beta;
//}

//void _park_inv_trans(float fd, float fq, float angle, float*f_alpha, float*f_beta)
//{
//	/* Park inverse transform */
//	float _c = cosf(angle);
//	float _s = sinf(angle);
//	*f_alpha = _c*fd - _s*fq;
//	*f_beta  = _s*fd + _c*fq;
//}


