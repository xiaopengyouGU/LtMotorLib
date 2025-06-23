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

void lt_foc_process(lt_foc_t foc,float fd, float fq, float angle_el)
{
	RT_ASSERT(foc != RT_NULL);
	/* check angle_el */
	float f_alpha, f_beta;
	float center = foc->max_val/2;		
	float u_min, u_max,tmp;					
	rt_uint8_t i;
	tmp = angle_el;
	angle_el = _normalize_angle(angle_el);
	fq = _constrains(fq,center,-center);	/* avoid output saturation */
	fd = _constrains(fd,center,-center);
	
	switch(foc->type)
	{
		case FOC_TYPE_6_TRAPZOID:
		{
			if(fq < 0)
			{
				angle_el = _normalize_angle(tmp + PI);
			}
			else
			{
				fq = -fq;
			}
			i = (angle_el+PI/6)/(PI/3);
			if(i >= 6 || i == 0) i = 6;
			i = i - 1;
			foc->fa = fq * _6_table[i][0];
			foc->fb = fq * _6_table[i][1];
			foc->fc = fq * _6_table[i][2];
			
			break;
		}
		case FOC_TYPE_SPWM:
		case FOC_TYPE_SVPWM:			   /* we use center modulation */
		{
			/* Park inverse transform */
			_park_inv_trans(fd,fq,angle_el,&f_alpha,&f_beta);
			/* Clark inverse transform */
			_clark_inv_trans(f_alpha,f_beta,&(foc->fa),&(foc->fb),&(foc->fc));
			if(foc->type == FOC_TYPE_SVPWM)
			{
				u_min = _min(foc->fa,_min(foc->fb,foc->fc));
				u_max = _max(foc->fa,_max(foc->fb,foc->fc));
				center -= (u_max + u_min)/2;
			}
			break;
		}
		default:break;
	}
	
	foc->fa += center;
	foc->fb += center;
	foc->fc += center;
	
}

void lt_foc_map_duty(lt_foc_t foc,float* dutyA, float* dutyB,float* dutyC)
{
	RT_ASSERT(foc != RT_NULL);
	if(foc->max_val == 0) 
	{
		*dutyA = 0;
		*dutyB = 0;
		*dutyC = 0;
	}
	else
	{
		*dutyA = foc->fa/foc->max_val;
		*dutyB = foc->fb/foc->max_val;
		*dutyC = foc->fc/foc->max_val;
	}
}
