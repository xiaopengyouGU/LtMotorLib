/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

static rt_err_t	_motor_bldc_output(lt_bldc_t, float input);
static rt_err_t	_motor_bldc_output_angle(lt_bldc_t,float angle);
static void _motor_bldc_config(lt_bldc_t bldc,struct lt_bldc_config* config);
static rt_err_t _motor_bldc_output_torque(lt_bldc_t bldc, float input);
static rt_err_t _motor_bldc_info(lt_bldc_t bldc,int cmd ,struct lt_bldc_info* info);
static rt_uint8_t _bldc_check_end(lt_bldc_t bldc,float angle);	
static void _bldc_output(lt_bldc_t bldc, float input);

static lt_motor_t _motor_bldc_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_bldc_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_bldc_object));
	if(_motor == RT_NULL) return RT_NULL;
	rt_memset(_motor,0,sizeof(struct lt_motor_bldc_object));
	rt_strcpy(_motor->parent.name,name);
	
	_motor->parent.reduction_ratio = reduction_ration;
	_motor->parent.type = type;
	_motor->parent.ops = &_motor_bldc_ops;		/* set operators */
	
	/* create foc object */
	_motor->foc = lt_foc_create();
	
	return (lt_motor_t)_motor;
}

static rt_err_t _motor_bldc_control(lt_motor_t motor, int cmd, void*arg)
{
	RT_ASSERT(motor != RT_NULL);
	lt_bldc_t bldc = (lt_bldc_t)motor;
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT:
		{
			float input = *(float *)arg;
			return _motor_bldc_output(bldc,input);
		}
		case BLDC_CTRL_GET_ELECTRIC_INFO:
		case BLDC_CTRL_GET_MECHANICAL_INFO:
		case BLDC_CTRL_GET_MOTOR_INFO:
		{
			struct lt_bldc_info* info = (struct lt_bldc_info* )arg;
			return _motor_bldc_info(bldc,cmd,info);
		}		
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			float angle = *(float *)arg;
			return _motor_bldc_output_angle(bldc,angle);
		}
		case BLDC_CTRL_CONFIG:
		{
			struct lt_bldc_config* config = (struct lt_bldc_config*)arg;
			_motor_bldc_config(bldc,config);
			break;
		}
		case BLDC_CTRL_OUTPUT_TORQUE:
		{
			float input = *(float*)arg;
			return _motor_bldc_output_torque(bldc,input);
		}
		case BLDC_CTRL_OUTPUT_NO_TIMER:
		{
			bldc->no_timer = 1;
			break;
		}
		default:break;
	}
	
	return RT_EOK;
}


static rt_err_t _motor_bldc_delete(lt_motor_t motor)
{
	lt_bldc_t bldc = (lt_bldc_t)motor;
	
	if(bldc->foc != RT_NULL)
	{
		lt_foc_delete(bldc->foc);
		bldc->foc = RT_NULL;
	}
	rt_free(bldc);
	
	return RT_EOK;
}

struct lt_motor_ops _motor_bldc_ops = {
										_motor_bldc_create,
										_motor_bldc_control,
										_motor_bldc_delete,
									 };

static void _bldc_open_loop_timeout(lt_timer_t timer)
{
	lt_bldc_t bldc = (lt_bldc_t)timer->user_data;
	lt_driver_t driver = bldc->parent.driver;
	lt_sensor_t sensor = bldc->parent.sensor;
	lt_foc_t foc = bldc->foc;
	rt_uint8_t res = 0;
	float angle = lt_sensor_get_angle(sensor);					/* get mechanical angle */;
	float Uq = bldc->target_vel/bldc->KV;						/* get Uq */
	float duty_A, duty_B, duty_C;
	
	if(bldc->flag & FLAG_BLDC_OPEN_POS)							/* open loop output angle */
	{
		res = _bldc_check_end(bldc,angle);
	}
	if(res)														/* reach target pos, stop output*/ 
	{
		lt_timer_disable(timer,TIMER_TYPE_HW);
		lt_driver_disable(driver);
		if(bldc->parent.callback)
		{
			bldc->parent.callback(bldc->parent.call_param);	   /* call callback function */
		}
		bldc->parent.status = MOTOR_STATUS_STOP;
		return;
	}
	
	lt_foc_process(foc,0,Uq,angle*bldc->poles);			    /* transform to electric angle */
	lt_foc_map_duty(foc,&duty_A,&duty_B,&duty_C);
	lt_driver_3pwm_output(driver,BLDC_OUTPUT_PERIOD,duty_A,duty_B,duty_C);		/* frequency: 10kHz <==> 100us */
}

static rt_err_t _motor_bldc_output(lt_bldc_t bldc, float input)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return RT_ERROR;				/* not configured !!! */
	/* clear open pos flags at first */
	bldc->flag = CLEAR_BITS(bldc->flag,FLAG_BLDC_OPEN_POS);
	_bldc_output(bldc,input);
	return RT_EOK;
}

static rt_err_t _motor_bldc_output_angle(lt_bldc_t bldc,float angle)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return RT_ERROR;				/* not configured !!! */
	/* set flag at first */
	bldc->flag |= FLAG_BLDC_OPEN_POS;
	float curr_pos = lt_sensor_get_angle(bldc->parent.sensor);
	rt_int8_t dir;		

	/* compare current pos and disired pos */
	angle = angle * PI/180.0f;										/* degree to rad */
	bldc->target_pos = angle;										/* record target */
	if(curr_pos > angle)
	{
		bldc->flag |= FLAG_BLDC_OPEN_POS_BIAS;						/* 1: initial angle > target */
		dir = -1;													/* reversal rotation */
	}
	else if(curr_pos < angle)
	{
		bldc->flag = CLEAR_BITS(bldc->flag,FLAG_BLDC_OPEN_POS_BIAS);	/* initial angle < target */
		dir = 1;													/* forward rotation */
	}
	else
	{
		return RT_EOK;
	}
	
	_bldc_output(bldc,dir*BLDC_OPEN_SPEED);
	return RT_EOK;
}

static void _motor_bldc_config(lt_bldc_t bldc,struct lt_bldc_config* config)
{
	if(config->KV <= 0 || config->poles == 0 || config->max_volt == 0 ) return;
	
	bldc->inductance = config->inductance;
	bldc->max_volt = _absf(config->max_volt);
	bldc->poles = config->poles;
	bldc->resistance = config->resistance;
	bldc->KV = config->KV;
	bldc->current = config->current;
	bldc->flag = FLAG_BLDC_CONFIG;				/* set config! */
	
	lt_foc_set_maxval(bldc->foc,config->max_volt);
	lt_foc_set_type(bldc->foc,config->foc_type);
}

static rt_err_t _motor_bldc_info(lt_bldc_t bldc,int cmd ,struct lt_bldc_info* info)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return RT_ERROR;				/* not configured !!! */
	bldc->flag |= FLAG_BLDC_GET_INFO;									/* set flag */
	lt_current_t current = bldc->current;
	lt_sensor_t sensor = bldc->parent.sensor;
	float angle_el = lt_sensor_get_angle(sensor)*bldc->poles;
	info->pos = angle_el/bldc->poles;	
	
	if(cmd == BLDC_CTRL_GET_ELECTRIC_INFO || cmd == BLDC_CTRL_GET_MOTOR_INFO)
	{
		struct lt_current_info current_info;
		lt_current_get_info(current,angle_el,&current_info);
		info->angle_el = _normalize_angle(angle_el)*180.0f/PI;
		info->Ia = current_info.Ia;
		info->Ib = current_info.Ib;
		info->Ic = current_info.Ic;
		info->Id = current_info.Id;
		info->Iq = current_info.Iq;		
	}
	if(cmd == BLDC_CTRL_GET_MECHANICAL_INFO || cmd == BLDC_CTRL_GET_MOTOR_INFO)
	{
		if(info->dt != 0)
		{
			info->vel = (info->pos - bldc->target_pos)/info->dt;			/* get velocity, no filter */
		}
	}
	bldc->target_pos = info->pos;										    /* record this pos */
	
	return RT_EOK;
}


void _bldc_torque_timeout(lt_timer_t timer)
{
	lt_bldc_t bldc = (lt_bldc_t)timer->user_data;
	lt_driver_t driver = bldc->parent.driver;
	lt_pid_t pid = bldc->parent.pid_current;
	lt_foc_t foc = bldc->foc;
	float angle_el = lt_sensor_get_angle(bldc->parent.sensor) * bldc->poles;
	float Id,Iq;
	float control_u;
	float duty_A, duty_B, duty_C;
	
	lt_current_get_dq(bldc->current,angle_el,pid->dt*1000000,&Id,&Iq);	/* unit: s --> us */
	control_u = PID_CURR_CONST * lt_pid_control(pid,Iq);
	lt_foc_process(foc,0,control_u,angle_el);
	lt_foc_map_duty(foc,&duty_A,&duty_B,&duty_C);
	lt_driver_3pwm_output(driver,BLDC_OUTPUT_PERIOD,duty_A,duty_B,duty_C);		/* frequency: 10kHz <==> 100us */	
}

static rt_err_t _motor_bldc_output_torque(lt_bldc_t bldc, float input)
{
	if(!(bldc->flag & FLAG_BLDC_CONFIG)) return RT_ERROR;				/* not configured !!! */
	if(bldc->current == RT_NULL || bldc->parent.pid_current == RT_NULL) return RT_ERROR;
	
	lt_timer_t timer = bldc->parent.timer;
	float Iq = input*bldc->KV/8.27;										/* get desired Iq, T[N.m] = 8.27*Iq[A]/KV */
	Iq = _constrains(Iq,BLDC_CURRENT_LIMIT, -BLDC_CURRENT_LIMIT);
	
	/* disable output at first */
	lt_driver_disable(bldc->parent.driver);
	lt_timer_disable(timer,TIMER_TYPE_HW);
	lt_pid_set_target(bldc->parent.pid_current,Iq);							/* set target Iq */
	return lt_timer_period_call(timer,BLDC_PERIOD,_bldc_torque_timeout,bldc,TIMER_TYPE_HW);
}

static rt_uint8_t _bldc_check_end(lt_bldc_t bldc,float angle)
{
	rt_uint8_t res = 0;
	if(bldc->flag & FLAG_BLDC_OPEN_POS_BIAS)				/* 1: initial angle > target */
	{
		if(angle <= bldc->target_pos)
		{
			bldc->flag = FLAG_BLDC_CONFIG;					/* clear other flags */
			res = 1;
		}
	}
	else													/*  initial angle < target */
	{
		if(angle >= bldc->target_pos)
		{
			bldc->flag = FLAG_BLDC_CONFIG;					/* clear other flags */
			res = 1;
		}
	}
	
	return res;
}

static void _bldc_output(lt_bldc_t bldc, float input)
{
	float max_speed = bldc->max_volt*bldc->KV/2;	
	lt_driver_t driver = bldc->parent.driver;
	lt_timer_t timer = bldc->parent.timer;
	lt_sensor_t sensor = bldc->parent.sensor;
	lt_foc_t foc = bldc->foc;
	float duty_A,duty_B,duty_C;
	float angle_el;
	/* check input */
	input = _constrains(input,max_speed,-max_speed);
	if(_absf(input) < 5) input = 0;
	
	if(input == 0) 
	{
		lt_driver_disable(driver);
		bldc->parent.status = MOTOR_STATUS_STOP;	/* change motor status */
		bldc->flag = CLEAR_BITS(bldc->flag,FLAG_BLDC_GET_INFO);					/* clear this flag */
		if(!bldc->no_timer) lt_timer_disable(timer,TIMER_TYPE_HW);
		return;
	}
	else
	{
		bldc->parent.status = MOTOR_STATUS_RUN;
	}

	if(bldc->no_timer)					/* user is responsible for controlling bldc open output */
	{
		input /= bldc->KV;
		if(bldc->flag & FLAG_BLDC_GET_INFO)											/* already read current pos */
		{
			angle_el = bldc->target_pos * bldc->poles;
			bldc->flag = CLEAR_BITS(bldc->flag,FLAG_BLDC_GET_INFO);					/* clear this flag */
		}
		else
		{
			angle_el = lt_sensor_get_angle(sensor) * bldc->poles;
		}
		lt_foc_process(foc,0,input,angle_el);			    				/* transform to electric angle */
		lt_foc_map_duty(foc,&duty_A,&duty_B,&duty_C);
		lt_driver_3pwm_output(driver,BLDC_OUTPUT_PERIOD,duty_A,duty_B,duty_C);		/* frequency: 10kHz <==> 100us */
	}
	else								/* use timer to control bldc open output */
	{	
		lt_timer_disable(timer,TIMER_TYPE_HW);
		bldc->target_vel = input;
		lt_timer_period_call(timer,BLDC_PERIOD,_bldc_open_loop_timeout,bldc,TIMER_TYPE_HW);	/* start timer */
	}
}
