/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        The first version
 * 2025-9-7		  Lvtou        The second version
 * 2025-11-19  	  Lvtou		   Only reserve contents of frame level, and some variables
 * 2025-11-30	  Lvtou		   Reimplement current device, improve effeciency, filter will be processed aotomatically
 */
#include "ltmotorlib.h"
#define CURRENT_FILTER_PARAM 		0.1f		

static void _current_get(lt_current_t current, float*Ia, float*Ib, float*Ic);

lt_current_t lt_current_create(const char* dev_name)
{
	lt_current_t _current = (lt_current_t)lt_malloc(sizeof(struct lt_current_sense));
	
	if(_current == NULL) return NULL;
	memset(_current,0,sizeof(struct lt_current_sense));
	/* copy device name */
	strcpy(_current->name, dev_name);
	_current->flag = DEVICE_FLAG_UNINIT;

	return _current;
}

void lt_current_set(lt_current_t current, struct lt_current_config * config)
{
	/* debug */
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(current);
	LT_CHECK_NULL(config);
	LT_CHECK_NULL(config->ops);
	LT_CHECK_NULL(config->ops->calibrate);
	LT_CHECK_NULL(config->ops->start);
	LT_CHECK_NULL(config->ops->stop);
#endif
	if(config->name != NULL)
	{
		strcpy(current->name,config->name);		/* set device name */
	}
	
	current->val_I_gain = 3.3f / (1 << config->bit_num) / config->resistor /config->amp_gain;
	current->val_bus_gain = 3.3f / (1 << config->bit_num) / config->vbus_gain;
	current->get_temp = config->get_temp;
	current->type = config->type;
	current->flag |= DEVICE_FLAG_INIT;
	current->ops = config->ops;					/* check ops successfully */
	current->flag |= DEVICE_FLAG_CHECKED;
}

void lt_current_get(lt_current_t current, float*Ia, float*Ib, float*Ic)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	/* return currents */
	float Ia_t,Ib_t,Ic_t;
	_current_get(current,&Ia_t,&Ib_t,&Ic_t);
	/* low pass filter process */
	*Ia = LOW_PASS_FILTER(Ia_t,*Ia,CURRENT_FILTER_PARAM);
	*Ib = LOW_PASS_FILTER(Ib_t,*Ib,CURRENT_FILTER_PARAM);
	*Ic = LOW_PASS_FILTER(Ic_t,*Ic,CURRENT_FILTER_PARAM);
}

void lt_current_get_bus(lt_current_t current, uint8_t* pulses,float input,float*I_bus)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
	LT_CHECK_NULL(pulses);
#endif
	float Ia, Ib, Ic;
	float Ip;					/* phase current */
	uint8_t val = (input > 0) ? 1 : 0;	/*	input > 0 : forward rotation, else : reversal rotation */
	
	_current_get(current,&Ia,&Ib,&Ic);
	/* check conducting phase */
	if(pulses[0] == pulses[1])	/* phase A high impedance */
	{
		if(pulses[2] == val)	Ip = Ib;	/* B+C- */
		else					Ip = Ic; 	/* B-C+ */
	}
	else if(pulses[2] == pulses[3])/* phase B high impedance */
	{
		if(pulses[0] == val)	Ip = Ia;	/* A+C- */
		else					Ip = Ic;	/* A-C+ */
	}
	else						 /* phase C high impedance */
	{
		if(pulses[0] == val)	Ip = Ia;	/* A+B- */
		else					Ip = Ib; 	/* A-B+ */
	}
	
	/* low pass filter process */
	*I_bus = LOW_PASS_FILTER(Ip,*I_bus,CURRENT_FILTER_PARAM);
	/* save bus current */
	current->I_bus_async = *I_bus;
}

void lt_current_get_ab(lt_current_t current, float * I_alpha, float * I_beta)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	float Ia, Ib, Ic;
	
	_current_get(current,&Ia,&Ib,&Ic);
	/* Clark transform */
	float Ia_t = _2_DIV_3 * (Ia- 0.5f*Ib- 0.5f*Ic);
	float Ib_t = _SQRT_3_3 * (Ib - Ic);
	/* low pass filter */
	*I_alpha = LOW_PASS_FILTER(Ia_t,*I_alpha,CURRENT_FILTER_PARAM);;
	*I_beta = LOW_PASS_FILTER(Ib_t,*I_beta,CURRENT_FILTER_PARAM);
}

void lt_current_get_dq(lt_current_t current, float angle_el,float * Id, float * Iq)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	float Ia, Ib, Ic;
	float I_alpha, I_beta;
	float Id_t, Iq_t;
	
	_current_get(current,&Ia,&Ib,&Ic);
	/* Clark transform */
	I_alpha = _2_DIV_3 * (Ia- 0.5f*Ib- 0.5f*Ic);
	I_beta = _SQRT_3_3 * (Ib - Ic);
	/* Park transform */
	float _c = lt_cos(angle_el);
	float _s = lt_sin(angle_el);
	Id_t = _c*I_alpha + _s*I_beta; 
	Iq_t = -_s*I_alpha + _c*I_beta;
	/* low pass filter */
	*Id = LOW_PASS_FILTER(Id_t,*Id,CURRENT_FILTER_PARAM);;
	*Iq = LOW_PASS_FILTER(Iq_t,*Iq,CURRENT_FILTER_PARAM);
	/* save d and q current */
	current->Id_async = *Id;
	current->Iq_async = *Iq;
}

void lt_current_get_vt(lt_current_t current, float * volt, float * temp)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	if (current->val_bus_gain != 0)
		*volt = current->vals[3] * current->val_bus_gain;
	if (current->get_temp != NULL)
		*temp = current->get_temp(current->vals[4]);
}
/* asynchronous read functions */
void lt_current_get_bus_async(lt_current_t current, float*I_bus)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	*I_bus = current->I_bus_async;
}

void lt_current_get_dq_async(lt_current_t current, float*Id, float*Iq)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	*Id = current->Id_async;
	*Iq = current->Iq_async;
}

uint32_t lt_current_calibrate(lt_current_t current)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	if(current->flag & DEVICE_FLAG_CALIB) return LT_EOK;
	else if(!(current->flag & DEVICE_FLAG_RUN))		/* if current device not RUN : STOP or INIT */
	{
		if(current->flag & DEVICE_FLAG_CALIBING) return LT_EBUSY;
		current->flag |= DEVICE_FLAG_CALIBING;
		current->flag = CLEAR_BITS(current->flag,DEVICE_FLAG_STOP);
		current->ops->calibrate(current);
		return LT_EBUSY;
	}
	else	return LT_EBUSY;
}

void lt_current_start(lt_current_t current)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	if(current->flag & DEVICE_FLAG_CALIB)	/* calibrate successfully */
	{
		if(!(current->flag & DEVICE_FLAG_RUN))	/* not start */
		{
			current->ops->start(current);		/* start current sense */
			current->flag |= DEVICE_FLAG_RUN;
			current->flag = CLEAR_BITS(current->flag, DEVICE_FLAG_STOP);
		}
	}
	else return;
}

void lt_current_stop(lt_current_t current)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	current->ops->stop(current);
	current->flag |= DEVICE_FLAG_STOP;
	current->flag = CLEAR_BITS(current->flag,DEVICE_FLAG_RUN);
}

void lt_current_delete(lt_current_t current)
{
#ifdef LT_DEBUG_ENABLE	
	LT_CHECK_CURRENT(current);
#endif
	current->ops->stop(current);
	current->flag = DEVICE_FLAG_UNINIT;
	lt_free(current);
}


static void _current_get(lt_current_t current, float*Ia, float*Ib, float*Ic)
{
	/* when this function is called, ADC values are already read by DMA, we only need to process datas */
	/* values transform, current positive direction is reversed !!! */
	float val_I_gain = current->val_I_gain;
	float Ia_t = -(current->vals[0] - current->bias[0]) * val_I_gain;
	float Ib_t = -(current->vals[1] - current->bias[1]) * val_I_gain;
	float Ic_t;
	
	if(current->type == CURRENT_TYPE_3)
	{
		Ic_t = -(current->vals[2] - current->bias[2]) * val_I_gain;
	}
	else
	{
		Ic_t = -(Ia_t + Ib_t);
	}
	
	/* output values */
	*Ia = Ia_t;
	*Ib = Ib_t;
	*Ic = Ic_t;
}
