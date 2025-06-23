/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

static void _motor_dc_output(lt_motor_t, float input);
static void _motor_dc_output_angle(lt_motor_t,float angle);

	
static lt_motor_t _motor_dc_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type)
{
	lt_motor_t _motor;
	_motor = rt_malloc(sizeof(struct lt_motor_object));
	if(_motor == RT_NULL) return RT_NULL;
	rt_memset(_motor,0,sizeof(struct lt_motor_object));
	rt_strcpy(_motor->name,name);
	
	_motor->reduction_ratio = reduction_ration;
	_motor->type = type;
	_motor->ops = &_motor_dc_ops;		/* set operators */
	return _motor;
}

static rt_err_t _motor_dc_control(lt_motor_t motor, int cmd, void*arg)
{
	RT_ASSERT(motor != RT_NULL);
	
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT:
		{
			float input = *(float *)arg;
			_motor_dc_output(motor,input);
			break;
		}
		case MOTOR_CTRL_OUTPUT_ANGLE:
		{
			float angle = *(float *)arg;
			_motor_dc_output_angle(motor,angle);
			break;
		}
		default:break;
	}
	
	return RT_EOK;
}


static rt_err_t _motor_dc_delete(lt_motor_t motor)
{
	rt_free(motor);
	return RT_EOK;
}

struct lt_motor_ops _motor_dc_ops = {
										_motor_dc_create,
										_motor_dc_control,
										_motor_dc_delete,
									 };

static void _motor_dc_output(lt_motor_t motor, float input)
{
	float duty_cycle;	
	rt_uint8_t dir = _get_rotation_dir(input);	/* get rotation direction */
	input = _absf(input);
	/* check input boundary with dead region */
	input = _constrains_dead_region(input,MOTOR_MAX_SPEED,MOTOR_MIN_SPEED);
	
	if(input == 0) 
	{
		lt_driver_disable(motor->driver);	
		motor->status = MOTOR_STATUS_STOP;	/* change motor status */
	}
	else
	{
		duty_cycle = input/MOTOR_MAX_SPEED;
		lt_driver_set_output(motor->driver,MOTOR_OUTPUT_PERIOD,duty_cycle);
		lt_driver_enable(motor->driver,dir);/* enable output */
		motor->status = MOTOR_STATUS_RUN;	/* change motor status */
	}
}

static void _motor_dc_output_angle(lt_motor_t motor,float angle)
{
	/* only close loop can output target angle */
	/* the code bellow is specified for steering motor, a close-loop type motor */
	if(angle <= 0)
	{
		lt_driver_disable(motor->driver);	
		motor->status = MOTOR_STATUS_STOP;
	}
	else if(angle >= MOTOR_MAX_ANGLE)
	{
		angle = MOTOR_MAX_ANGLE;
	}
	/* transform angle to pulse */
	angle = MOTOR_ANGLE_BASE + (angle/MOTOR_MAX_ANGLE)*MOTOR_ANGLE_PERIOD/10;		    /* correspond to 0.5ms - 2.5ms */
	angle /= MOTOR_ANGLE_PERIOD;														/* get duty_cycle */
	lt_driver_set_output(motor->driver,MOTOR_ANGLE_PERIOD,angle);
	lt_driver_enable(motor->driver,0);	/* enable output */
	motor->status = MOTOR_STATUS_RUN;	 			/* change motor status */
}
