/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "math.h"

#define INTERP_TIMER_TYPE	TIMER_TYPE_SOFT
/* use software timer to help output given number pulse, this function is called once */
static void _timer_output_angle(lt_timer_t timer)
{
	lt_stepper_t stepper = (lt_stepper_t)(timer->user_data);
	lt_driver_disable(stepper->parent.driver);		/* disable output */
	lt_timer_disable(timer,TIMER_TYPE_SOFT);		/* use software timer */
	stepper->parent.status = MOTOR_STATUS_STOP;
	if(stepper->parent.callback)					/* call done callback function */
	{
		stepper->parent.callback(stepper->parent.call_param);
	}
}

static rt_err_t _motor_stepper_output_angle(lt_stepper_t stepper,float angle);
static rt_err_t _motor_stepper_output(lt_stepper_t stepper, float input);
static void _motor_stepper_config(lt_stepper_t,struct lt_stepper_config* config);
static rt_err_t _motor_stepper_accelerate(lt_stepper_t stepper,struct lt_curve_config* config);
static rt_err_t _motor_stepper_interp(lt_stepper_t stepper,struct lt_interp_config* config);

static lt_motor_t _motor_stepper_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_stepper_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_stepper_object));
	if(_motor == RT_NULL) return RT_NULL;
	
	rt_memset(_motor,0,sizeof(struct lt_motor_stepper_object));
	rt_strcpy(_motor->parent.name,name);
	
	_motor->parent.reduction_ratio = reduction_ration;
	_motor->parent.type = type;
	_motor->parent.ops = &_motor_stepper_ops;/* set operators */
	/* create curve and interp */
	_motor->curve = lt_curve_create();
	_motor->interp = lt_interp_create();
	
	return (lt_motor_t)_motor;				 /* type transform */
}

static rt_err_t _motor_stepper_control(lt_motor_t motor, int cmd, void*arg)
{
	RT_ASSERT(motor != RT_NULL);
	lt_stepper_t stepper = (lt_stepper_t)motor;			/* type transform */
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT:
		{
			float input = *(float *)arg;
			return _motor_stepper_output(stepper,input);
		}
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			if(motor->timer == RT_NULL) return RT_ERROR;
			float angle = *(float *)arg;
			return _motor_stepper_output_angle(stepper,angle);
		}
		case STEPPER_CTRL_CONFIG:		/* config stepper parameters */
		{
			struct lt_stepper_config*  config = (struct lt_stepper_config*)arg;
			_motor_stepper_config(stepper,config);
			break;
		}
		case STEPPER_CTRL_ACCELERATE:
		{
			if(motor->timer == RT_NULL) return RT_ERROR;
			struct lt_curve_config* config = (struct lt_curve_config*)arg;
			return _motor_stepper_accelerate(stepper,config);
		}
		case STEPPER_CTRL_INTERPOLATION:
		{
			struct lt_interp_config* config = (struct lt_interp_config*)arg;
			return _motor_stepper_interp(stepper,config);
		}
		default:break;
	}
	
	return RT_EOK;
		
}

static rt_err_t _motor_stepper_delete(lt_motor_t motor)
{
	lt_stepper_t stepper = (lt_stepper_t)motor;
	
	if(stepper->curve != RT_NULL)
	{
		lt_curve_delete(stepper->curve);
		stepper->curve = RT_NULL;
	}
	if(stepper->interp != RT_NULL)
	{
		lt_interp_delete(stepper->interp);
		stepper->interp = RT_NULL;
	}
	
	rt_free(stepper);
	return RT_EOK;
}

struct lt_motor_ops _motor_stepper_ops = {
										_motor_stepper_create,
										_motor_stepper_control,
										_motor_stepper_delete,
									 };

static rt_err_t _motor_stepper_output(lt_stepper_t stepper, float input)
{
	/* rotate at a constant speed */
	/* speed is proportional to frequency */
	if(!(stepper->flag & FLAG_STEPPER_CONFIG))	return RT_ERROR;							/* not configured */
	rt_uint8_t dir = _get_rotation_dir(input);									/* get rotation direction  */
	input = _absf(input);
	lt_driver_t driver = stepper->parent.driver;
	/* check input */
	input = _constrains_dead_region(input,STEPPER_MAX_SPEED,STEPPER_MIN_SPEED);
	
	if(input == 0)
	{
		lt_driver_disable(driver);												/* disable output */
		stepper->parent.status = MOTOR_STATUS_STOP;
	}
	else
	{
		input = STEPPER_MAX_PERIOD/input;										/* transform */
		if(input > STEPPER_MAX_PERIOD) input = STEPPER_MAX_PERIOD;			    /* slowest rotation */
		lt_driver_set_output(driver,input,STEPPER_DUTY_CYCLE);				
		lt_driver_enable(driver,dir);											/* enable output */
		stepper->parent.status = MOTOR_STATUS_RUN;
	}
	return RT_EOK;
}

static rt_err_t _motor_stepper_output_angle(lt_stepper_t stepper, float angle)
{	/* in open loop case, we just use software timer to save hardware timer */
	if(!(stepper->flag & FLAG_STEPPER_CONFIG))	return RT_ERROR;							/* not configured */
	rt_uint8_t dir = _get_rotation_dir(angle);			/* get rotation direction */
	angle = _absf(angle);
	lt_driver_t driver = stepper->parent.driver;
	lt_timer_t timer = stepper->parent.timer;
	lt_driver_disable(driver);							/* disable driver at first */
	
	if(angle == 0)
	{
		stepper->parent.status = MOTOR_STATUS_STOP;
		return RT_EOK;
	}
	/* get number of square pulse */
	rt_uint32_t n = angle/(stepper->angle/stepper->subdivide);		
	rt_uint32_t period = stepper->period*1000;									/* unit: us */		
	/* config software timer and start  */
	lt_timer_period_call(timer,n*period,_timer_output_angle,stepper,TIMER_TYPE_SOFT);
	lt_driver_set_output(driver,period,STEPPER_DUTY_CYCLE);
	lt_driver_enable(driver,dir);
	stepper->parent.status = MOTOR_STATUS_RUN;	/* output */
	
	return RT_EOK;
}

static void _motor_stepper_config(lt_stepper_t stepper,struct lt_stepper_config* config)
{
	stepper->angle = config->stepper_angle;
	stepper->period = config->period;
	stepper->subdivide = config->subdivide;
	stepper->flag |= FLAG_STEPPER_CONFIG;										/* finish config */
	
	if(stepper->subdivide == 0)
		stepper->subdivide = 1;	
}

void _hw_timer_callback_accel(lt_timer_t timer)
{
	lt_stepper_t stepper = (lt_stepper_t)(timer->user_data);
	lt_driver_t  driver = stepper->parent.driver;
	lt_curve_t curve = stepper->curve;
	/* disable output at first */
	lt_driver_disable(driver);
	rt_uint32_t period = lt_curve_process(curve);				/* get pulse period unit: us */
	
	if(period == 0)												/* finish all work, stop timer! */
	{
		lt_timer_disable(timer,TIMER_TYPE_HW);					/* disable hwtimer */
		stepper->parent.status = MOTOR_STATUS_STOP;
		if(stepper->parent.callback)							/* call done callback function */
		{
			stepper->parent.callback(stepper->parent.call_param);
		}
		return;
	}
	
	if(!(stepper->flag & FLAG_STEPPER_S_CURVE))					/* S_CURVE acceleration don't change timer period */
	{
		lt_timer_set(timer,period,0,TIMER_TYPE_HW);				/* default mode */
		lt_timer_enable(timer,TIMER_TYPE_HW);							
	}
	/* config driver */
	lt_driver_set_output(driver,period,STEPPER_DUTY_CYCLE);
	lt_driver_enable(driver,0);		
	
}

static rt_err_t _motor_stepper_accelerate(lt_stepper_t stepper,struct lt_curve_config* config)
{
	if(!(stepper->flag & FLAG_STEPPER_CONFIG)) return RT_ERROR;		/* stepper is not configured */
	RT_ASSERT(config != RT_NULL);
	/* clear stepper flag */
	stepper->flag = FLAG_STEPPER_CONFIG;
	lt_driver_t driver = stepper->parent.driver;
	lt_timer_t timer = stepper->parent.timer;
	lt_curve_t curve = stepper->curve; 
	rt_uint8_t dir = _get_rotation_dir((float)config->step);	
	rt_err_t res;
	rt_uint32_t T0;
	
	lt_driver_disable(driver);			    /* disable pwm output at first */
	lt_curve_release(curve);										/* release memory */
	/* check accleration type and create curve */
	switch(config->type)
	{
		case CURVE_TYPE_TRAPZOID:
			stepper->flag |= FLAG_STEPPER_TRAPZOID;
			break;
		case CURVE_TYPE_S_CURVE:
			stepper->flag |= FLAG_STEPPER_S_CURVE;
			break;
		case CURVE_TYPE_5_SECTION:
			stepper->flag |= FLAG_STEPPER_5_SECTION;
			break;
		default:return RT_ERROR;							/* unknown type */
	}
	/* set curve */
	res = lt_curve_set(curve,config);
	if(res != RT_EOK) return res;							/* create curve failed*/
	
	T0 = lt_curve_process(curve);							/* get first period */
	/* config hw_timer and start  */
	lt_timer_period_call(timer,T0,_hw_timer_callback_accel,stepper,TIMER_TYPE_HW);
	lt_driver_set_output(driver,T0,STEPPER_DUTY_CYCLE);
	lt_driver_enable(driver,dir);
	stepper->parent.status = MOTOR_STATUS_ACCELERATE;
	
	return RT_EOK;
}

void _timer_callback_interp(lt_timer_t timer)
{/* in interp case, software timer is also used to save hardware timer */
	lt_stepper_t stepper = (lt_stepper_t)(timer->user_data);
	lt_motor_t y_motor = (lt_motor_t)(stepper->parent.user_data);
	lt_driver_t x_driver = stepper->parent.driver;
	lt_driver_t y_driver = y_motor->driver;
	lt_interp_t interp = stepper->interp;
	rt_uint8_t dir, curr_axis;
	/* disable output at first */
	lt_driver_disable(x_driver);
	lt_driver_disable(y_driver);
	
	/* get current move axis and move direction */
	curr_axis = lt_interp_process(interp,&dir);
	if(curr_axis == X_MOTOR_MOVE)
	{
		lt_driver_enable(x_driver,dir);
	}
	else if(curr_axis == Y_MOTOR_MOVE)
	{
		lt_driver_enable(y_driver,dir);
	}
	else	/* finish interpolation, close timer and call motor callback function */
	{
		lt_timer_disable(timer,INTERP_TIMER_TYPE);				/* disable interp timer */
		stepper->parent.status = MOTOR_STATUS_STOP;
		y_motor->status = MOTOR_STATUS_STOP;
		if(stepper->parent.callback)							/* call done callback function */
		{
			stepper->parent.callback(stepper->parent.call_param);
		}
		if(y_motor->callback)
		{
			y_motor->callback(y_motor->call_param);
		}
		return;
	}
}

static rt_err_t _motor_stepper_interp(lt_stepper_t stepper,struct lt_interp_config* config)
{
	if(!(stepper->flag & FLAG_STEPPER_CONFIG)) return RT_ERROR;		/* stepper is not configured */
	RT_ASSERT(config != RT_NULL);
	lt_interp_t interp = stepper->interp;
	/* x motor is this stepper */
	lt_motor_t y_motor = (lt_motor_t)config->y_motor;
	lt_timer_t timer = stepper->parent.timer;
	/* x motor is this stepper */
	lt_driver_t x_driver = stepper->parent.driver;
	lt_driver_t y_driver = y_motor->driver; 
	/* x motor is this stepper */
	rt_uint32_t period = stepper->period * 1000;			/* unit: us */
	rt_err_t res;
	
	/* disable pwm output at first */
	lt_driver_disable(x_driver);			   
	lt_driver_disable(y_driver);
	/* set interp */
	res = lt_interp_set(interp,config);
	if(res != RT_EOK) return res;							/* set interp failed */
	
	/* set driver */
	lt_driver_set_output(x_driver,period,0.5);
	lt_driver_set_output(y_driver,period,0.5);
	/* config timer and start  */
	lt_timer_period_call(timer,period,_timer_callback_interp,stepper,INTERP_TIMER_TYPE);

	
	/* here we use user_data variation to transport object! */
	stepper->parent.user_data = y_motor;
	stepper->parent.status = MOTOR_STATUS_INTERP;
	y_motor->status = MOTOR_STATUS_INTERP;
	
	return RT_EOK;
}
