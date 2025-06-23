/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "math.h"
/* in many case, acceleration part and deceleration part are symmetric, so we don't need to create a full velocity table */
static void _trapzoid_accel(lt_curve_t curve,int step,float acc,float dec, float speed)
{
	rt_uint16_t i,j,flag;
	rt_uint16_t dec_start, dec_step;
	float* T_nums;
	if(acc == dec) flag = 1;									/* in this case, we only save a part of the full velocity table to save memory */
	/* get min period ang initial period */
	rt_uint32_t T0 = 0.69*sqrtf(1000 * 2/ acc) * 1000.0f;	    /* initial period, t = 1000*sqrt(2*S/(acc*k0)), unit: us, multiply error coefficiency */
	/* get accel step and max accel step */
	rt_uint16_t acc_step = (dec * step)/(acc + dec);
	rt_uint16_t acc_max_step = (speed * speed)/(2 * acc) /1000.0f;			/* S = v^2/(2*accel) * k0 */
	
	if(acc_max_step == 0) acc_max_step = 1;
	if(acc_step <= acc_max_step)					/* stepper can't reach desired speed, so accel --> decel */
	{ 
		dec_step = step - acc_step;
	}
	else											/* accel --> constant --> decel */
	{
		acc_step = acc_max_step;					/* let acc_step = acc_max_step */
		dec_step = 1.0f*(acc_max_step * acc)/dec;
	}

	if(dec_step == 0)								/* in this case, stepper must decelerate */
	{
		dec_step = 1;				
	}
	dec_start = step - dec_step;
	
	if(flag)										/* only need to create a part of velocity table */
	{	
		T_nums = rt_malloc((acc_step)* sizeof(float));
	}
	else											/* create full table */
	{
		T_nums = rt_malloc((step)* sizeof(float));
		curve->flag = 1;
	}
	RT_ASSERT(T_nums != RT_NULL);					/* check res */
	
	T_nums[0] = T0;		
	for(i = 1; i < acc_step - 1; i++)				/* accel, modify last acc value */
	{
		T_nums[i] = T_nums[i-1] - 2.0f*T_nums[i-1]/(4*i + 1);
	}
	
	T_nums[0] = T0/0.69;							/* consider acc_step == 1, T_nums[0] would be redefined! */
	if(acc_step >= acc_max_step)					/* can reach desired speed */
	{
		T_nums[acc_step - 1] = 1000000.0f/speed;
	}
	else
	{	
		if(i >= 1)									/* avoid index output */
		T_nums[i] = T_nums[i-1] - 2.0f*T_nums[i-1]/(4*i + 1);
	}
	
	/* create a full velocity table or not */
	if(!flag)										/* create full table */
	{
		for(i = i; i < dec_start; i++)				/* const */
		{
			T_nums[i] = T_nums[i-1];
		}
		for(i = i; i < step; i++)					/* decel */
		{
			j = step - i;
			T_nums[i] = T_nums[i-1] + 2.0f*T_nums[i-1]/(4*j - 1);
		}
	}
	
	curve->T_nums = T_nums;
	curve->acc_step = acc_step;
	curve->dec_start = dec_start;
	curve->step = step;
}

static void _s_curve_accel(lt_curve_t curve,int step, rt_uint16_t acc_step,float freq_min, float freq_max,float flexible)
{
	rt_uint16_t dec_step = acc_step;
	if(acc_step + dec_step > step) return;	/* unvalid input */
	
	float delt_f = freq_max - freq_min;
	float num,den,freq;
	rt_uint16_t i;
	rt_uint16_t dec_start = step - dec_step;
	
	float *T_nums = rt_malloc((acc_step)* sizeof(float));   /* create a part of velocity table list, unit: us */
	RT_ASSERT(T_nums != RT_NULL);							/* check res */
	
	for(i = 0; i < acc_step; i++)								
	{
		num = flexible * (i - acc_step/2)/(acc_step/2);
		den = 1.0f + expf(-num);
		freq = freq_min + delt_f/den;
		T_nums[i] = 1000000.0f/freq;						/* get period, unit: us */
	}
	/* we don't need to create a full velocity table, since we know the rule */
	curve->T_nums = T_nums;
	curve->acc_step = acc_step;
	curve->dec_start = dec_start;
	curve->step = step;
}

static void _5_section_accel(lt_curve_t curve,int step,rt_uint16_t acc_t,float vo,float vt)
{
	rt_uint8_t is_inc;				/* 1: increment, 0: decrement */
	rt_uint16_t dec_start;
	if(vo == vt) return;
	if(vo < vt) 
	{
		is_inc = 1;
	}
	else							/* exchange vo and vt */
	{
		float tmp = vo;			
		vo = vt;
		vt = tmp;
	}
	
	acc_t = acc_t/2;			 	/* inc_acc time is equal to dec_acc time, unit: ms */
	float J = (vt - vo)/(acc_t * acc_t);										/* Vm = 0.5*J*t^2 , Vm = Vt/2, but delt_V = Vt - Vo */
	rt_uint16_t inc_step = (vo * acc_t + 1.0f/6 * J * acc_t * acc_t * acc_t)/1000.0f;		/* acc inc step */
	rt_uint16_t dec_step = ((vt + vo) * acc_t)/1000.0f - inc_step;							/* acc dec step */
	rt_uint16_t acc_step = inc_step + dec_step;
	
	if(step < 2*acc_step)			/* can't reach target speed given step */
	{
		return;
	}
	
	float *T_nums = rt_malloc((acc_step)* sizeof(float));         		/* create a speed  period list, unit: us */
	RT_ASSERT(T_nums != RT_NULL);								  	    /* check res */
	/* check step and record important index */
	dec_start = step - acc_step;
	
	/* get first step speed */
	float T0 = powf(1000.0f * 6.0f/J,1.0f/3);							/* unit: ms, multiply 1000: coffeciency  */
	float sum_t = T0;													/* total time */
	float delt_v = 0.5f * J * powf(sum_t,2);
	float Ti, vel = vo + delt_v;
	rt_uint16_t i;
	T_nums[0] = 1000.0f/vel;											/* unit: ms*/																			   		
	/* create velocity table */
	for( i = 1; i < acc_step; i++)										
	{
		Ti = T_nums[i-1];											
		sum_t += Ti;
		if(i < inc_step)
		{
			delt_v = 0.5f * J * powf(sum_t,2);
			vel = delt_v + vo;
			if(i == inc_step - 1)	/* in last inc step, time isn't equal to acc_t exactly, take this into dec_acc part */
				sum_t = fabs(sum_t - acc_t);		
		}
		else
		{
			delt_v = 0.5f * J * powf(fabs(acc_t - sum_t),2);
			vel = vt - delt_v;
		}
		T_nums[i] = 1000.0f/vel;
	}
	for(i = 0; i < acc_step; i++)
	{
		T_nums[i] *= 1000.0f;											/* ms --> us */
	}
	
	if(!is_inc)															/* descend */
	{
		for(i = 0; i < acc_step/2; i++)									/* change sequence */
		{
			vel = T_nums[i];
			T_nums[i] = T_nums[acc_step - 1 - i];
			T_nums[acc_step - 1 -i] = vel;
		}
	}
	/* we don't need to create a full velocity table, since we know the rule */
	curve->T_nums = T_nums;
	curve->acc_step = acc_step;
	curve->dec_start = dec_start;
	curve->step = step;
}

lt_curve_t lt_curve_create(void)
{
	lt_curve_t _curve = rt_malloc(sizeof(struct lt_curve_generator_object));
	if(_curve != RT_NULL)
	{
		rt_memset(_curve,0,sizeof(struct lt_curve_generator_object));
	}
	return _curve;
}


rt_err_t lt_curve_set(lt_curve_t curve,struct lt_curve_config* config)
{
	RT_ASSERT(curve != RT_NULL);
	RT_ASSERT(config != RT_NULL);
	/* create a velocity table */
	config->step = _abs(config->step);
	rt_uint16_t step = config->step;
	if(step == 0) return RT_ERROR;								/* unvalid input */
	if(config->initial < 0 || config->target <= 0) return RT_ERROR;
	if(config->acc < 0 || config->dec < 0 || config->flexible < 0) return RT_ERROR;
	/* user forget to release memory */
	if(curve->T_nums != RT_NULL)
	{
		rt_free(curve->T_nums);
		curve->T_nums = RT_NULL;
	}
	
	switch(config->type)
	{
		case CURVE_TYPE_TRAPZOID:
		{
			_trapzoid_accel(curve,step,config->acc,config->dec,config->target);
			break;
		}
		case CURVE_TYPE_S_CURVE:
		{
			_s_curve_accel(curve,step,config->acc,config->initial,config->target,config->flexible);
			break;
		}
		case CURVE_TYPE_5_SECTION:
		{
			_5_section_accel(curve,step,config->acc,config->initial,config->target);
			break;
		}
		default:break;
	}
	curve->curr_step = 0;
	/* check whether generate accel curve successfully */
	if(curve->T_nums == RT_NULL) return RT_ERROR;
	
	return RT_EOK;
}

rt_uint32_t lt_curve_process(lt_curve_t curve)
{
	RT_ASSERT(curve != RT_NULL);
	RT_ASSERT(curve->T_nums != RT_NULL);
	rt_uint32_t time;
	rt_uint16_t curr_step = curve->curr_step;
	if(curr_step >= curve->step) return 0;			/* finish accleration */
	
	if(!curve->flag)								/* part velocity table */
	{
		if(curr_step < curve->acc_step)				/* accel part */
		{
			time = curve->T_nums[curr_step];
		}
		else if(curr_step < curve->dec_start)		/* const speed part */
		{
			time = curve->T_nums[curve->acc_step - 1];
		}
		else 										/* decel part */
		{
			time = curve->T_nums[curve->step - 1 - curr_step];
		}
	}
	else											/* full velocity table */
	{
		time = curve->T_nums[curr_step];		
	}
	
	curve->curr_step++;
	return time;
}

rt_err_t lt_curve_restart(lt_curve_t curve)				/* use same velocity table */
{
	RT_ASSERT(curve != RT_NULL);
	curve->curr_step = 0;
	return RT_EOK;
}

rt_err_t lt_curve_release(lt_curve_t curve)
{
	RT_ASSERT(curve != RT_NULL);
	if(curve->T_nums != RT_NULL)		/* release memory */
	{
		rt_free(curve->T_nums);
		curve->T_nums = RT_NULL;
	}
	return RT_NULL;
}

rt_err_t lt_curve_delete(lt_curve_t curve)
{
	RT_ASSERT(curve != RT_NULL);
	rt_free(curve);						/* release memory */
	return RT_EOK;
}	

