/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "math.h"
#define _TF 0.0005		/* unit: s */

static float _current_read(lt_current_t current,rt_int8_t channel,float bias);
static void _current_get(lt_current_t current, float*Ia, float*Ib, float*Ic);

lt_current_t lt_current_create(float resistor, float amp_gain, rt_uint8_t bit_num)
{
	if(resistor <= 0 || amp_gain == 0 || bit_num == 0) return RT_NULL;
	lt_current_t _current;
	lt_filter_t _filter_d = lt_filter_create(_TF,0);
	lt_filter_t _filter_q = lt_filter_create(_TF,0);
	_current = rt_malloc(sizeof(struct lt_current_sense_object));
	if(_current == RT_NULL) return RT_NULL;
	if(_filter_d == RT_NULL || _filter_q == RT_NULL) return RT_NULL;
	rt_memset(_current,0,sizeof(struct lt_current_sense_object));
	
	_current->resistor = resistor;
	_current->amp_gain = amp_gain;
	_current->bit_num = bit_num;
	_current->lpf_d = _filter_d;
	_current->lpf_q = _filter_q;
	return _current;
}

rt_err_t lt_current_set_adc(lt_current_t current, char*adc_name,rt_int8_t channel_A, rt_int8_t channel_B, rt_int8_t channel_C)
{
	RT_ASSERT(current != RT_NULL);
	rt_adc_device_t adc = (rt_adc_device_t)rt_device_find(adc_name);
	if(adc == RT_NULL) return RT_ERROR;
	
	current->adc = adc;
	current->channel_A = channel_A;
	current->channel_B = channel_B;
	current->channel_C = channel_C;
	
	return RT_EOK;
}

rt_err_t lt_current_calibrate(lt_current_t current)
{
	RT_ASSERT(current != RT_NULL);
	RT_ASSERT(current->adc != RT_NULL);
	if(current->channel_A < 0 && current->channel_B < 0 && current->channel_C < 0) return RT_ERROR;
	float bias_a, bias_b, bias_c;
	rt_int8_t channel_A = current->channel_A, channel_B = current->channel_B, channel_C = current->channel_C;
	rt_uint8_t i;
	rt_adc_device_t adc = current->adc;
	rt_adc_enable(adc,channel_A);
	rt_adc_enable(adc,channel_B);
	

	for(i = 0; i < 100; i++)		/* read 100 times to get zeros bias */
	{
		bias_a += rt_adc_read(adc,channel_A);
		bias_b += rt_adc_read(adc,channel_B);
	}
	bias_a /= 100;
	bias_b /= 100;
		
	if(current->channel_C > 0)		/* we use three measure resistors */
	{
		rt_adc_enable(adc,channel_C);
		for(i = 0; i < 100; i++)
		{
			bias_c += rt_adc_read(adc,channel_C);
		}
		bias_c /= 100;
	}
	
	current->bias_a = bias_a;
	current->bias_b = bias_b;
	current->bias_c = bias_c;
	
	return RT_EOK;
}

rt_err_t lt_current_get(lt_current_t current, float*Ia, float*Ib, float*Ic)
{
	RT_ASSERT(current != RT_NULL);
	RT_ASSERT(current->adc != RT_NULL);
	if(current->channel_A < 0 && current->channel_B < 0 && current->channel_C < 0) return RT_ERROR;
	_current_get(current,Ia,Ib,Ic);
	return RT_EOK;
}

rt_err_t lt_current_get_ab(lt_current_t current, float angle, float * _I_alpha, float * _I_beta)
{
	RT_ASSERT(current != RT_NULL);
	RT_ASSERT(current->adc != RT_NULL);
	if(current->channel_A < 0 && current->channel_B < 0 && current->channel_C < 0) return RT_ERROR;
	float Ia, Ib, Ic;
	
	_current_get(current,&Ia,&Ib,&Ic);
	_clark_trans(Ia,Ib,Ic,_I_alpha,_I_beta);
	return RT_EOK;
}

rt_err_t lt_current_get_dq(lt_current_t current, float angle,float dt, float * _Id, float * _Iq)
{
	RT_ASSERT(current != RT_NULL);
	RT_ASSERT(current->adc != RT_NULL);
	if(current->channel_A < 0 && current->channel_B < 0 && current->channel_C < 0) return RT_ERROR;
	float Ia, Ib, Ic;
	float I_alpha, I_beta;
	float Id,Iq;
	
	_current_get(current,&Ia,&Ib,&Ic);
	/* Clark transform */
	_clark_trans(Ia,Ib,Ic,&I_alpha,&I_beta);
	/* Park transform ,check angle */
	_park_trans(I_alpha,I_beta,angle,&Id,&Iq);
	/* low pass filter process */
	if(dt >= 0)							/* sample time, unit: s */
	{
		lt_filter_set_dt(current->lpf_d,dt);
		lt_filter_set_dt(current->lpf_q,dt);
		Id = lt_filter_process(current->lpf_d,Id);
		Iq = lt_filter_process(current->lpf_q,Iq);
	}
	*_Id = Id;
	*_Iq = Iq;
	
	return RT_EOK;
}

rt_err_t lt_current_get_info(lt_current_t current, float angle, struct lt_current_info* info)
{
	RT_ASSERT(current != RT_NULL);
	RT_ASSERT(current->adc != RT_NULL);
	RT_ASSERT(info != RT_NULL);
	if(current->channel_A < 0 && current->channel_B < 0 && current->channel_C < 0) return RT_ERROR;
	float Ia, Ib, Ic;
	float I_alpha, I_beta;
	float Id,Iq;
	angle = _normalize_angle(angle);
	_current_get(current,&Ia,&Ib,&Ic);
	/* Clark transform */
	_clark_trans(Ia,Ib,Ic,&I_alpha,&I_beta);
	/* Park transform ,check angle */
	_park_trans(I_alpha,I_beta,angle,&Id,&Iq);
	
	/* get result */
	info->Ia = Ia;
	info->Ib = Ib;
	info->Ic = Ic;
	info->I_alpha = I_alpha;
	info->I_beta = I_beta;
	info->Id = Id;
	info->Iq = Iq;
	
	return RT_EOK;
}

rt_err_t lt_current_delete(lt_current_t current)
{
	RT_ASSERT(current != RT_NULL);
	if(current->adc != RT_NULL)
	{
		rt_adc_disable(current->adc,current->channel_A);
		rt_adc_disable(current->adc,current->channel_B);
		rt_adc_disable(current->adc,current->channel_C);
		current->adc = RT_NULL;
	}
	if(current->lpf_d != RT_NULL)
	{
		lt_filter_delete(current->lpf_d);
		current->lpf_d = RT_NULL;
	}
	if(current->lpf_q != RT_NULL)
	{
		lt_filter_delete(current->lpf_q);
		current->lpf_q = RT_NULL;
	}
	
	rt_free(current);
	
	return RT_EOK;
}

static float _current_read(lt_current_t current,rt_int8_t channel,float bias)
{
	rt_adc_enable(current->adc,channel);
	rt_uint8_t i;
	float value;
	
	for(i = 0; i < 10; i++)									/* read 10 times and get average value*/
	{
		value += rt_adc_read(current->adc,channel);
	}
	
	value = value/10.0f - bias;								/* subtract bias */
	value = value * 3.3f/(1 << current->bit_num);			/* transform to voltage */
	value = value / current->resistor / current->amp_gain;	/* get current value */
	
	return value;
}

static void _current_get(lt_current_t current, float*Ia, float*Ib, float*Ic)
{
	float ia,ib,ic;
	
	ia = _current_read(current,current->channel_A,current->bias_a);
	ib = _current_read(current,current->channel_B,current->bias_b);
	if(current->channel_C > 0)		/* we use three measure resistors */
	{
		ic = _current_read(current,current->channel_B,current->bias_c);
	}
	else							/* we only use two measure resistors */
	{
		ic = ia + ib;
	}
	
	/* here we guarentee the measure direction is matched with real direction */
	*Ia = -ia;
	*Ib = -ib;
	*Ic = ic;
}
