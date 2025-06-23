/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

extern struct lt_motor_manager_object * _manager;
static rt_err_t _output_pid(lt_motor_t motor,float target);
static rt_err_t _disable_motor(lt_motor_t motor);

lt_motor_t lt_motor_create(char* name, rt_uint8_t reduction_ration, rt_uint8_t type)
{
	lt_motor_t _motor;
	if(reduction_ration == 0) reduction_ration = 1;
	
	/* check whether the motor is in the manager */
	_motor = lt_manager_get_motor(name);
	if(_motor != RT_NULL)	return RT_NULL;		/* motor is already in the mananger */
	
	switch(type)
	{
		case MOTOR_TYPE_DC:
		{
			_motor = _motor_dc_ops.create(name,reduction_ration,type);
			break;
		}
		case MOTOR_TYPE_BLDC:
		{
			_motor = _motor_bldc_ops.create(name,reduction_ration,type);
			break;
		}
		case MOTOR_TYPE_STEPPER:
		{
			_motor = _motor_stepper_ops.create(name,reduction_ration,type);
		}
	}
	lt_manager_add_motor(_motor);		/* add motor to manager */

	return _motor;
}

rt_err_t lt_motor_set_driver(lt_motor_t motor, lt_driver_t driver)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(driver != RT_NULL);
	motor->driver = driver;
	return RT_EOK;
}

rt_err_t lt_motor_set_callback(lt_motor_t motor, rt_err_t (*callback)(void*), void*call_param)
{
	RT_ASSERT(motor != RT_NULL);
	motor->callback = callback;
	motor->call_param = call_param;
	return RT_EOK;
}

rt_err_t lt_motor_set_sensor(lt_motor_t motor, lt_sensor_t sensor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(sensor != RT_NULL);
	lt_sensor_calibrate(sensor);		/* calibrate sensor before we use it */
	motor->sensor = sensor;
	
	return RT_EOK;
}

rt_err_t lt_motor_set_pid(lt_motor_t motor,lt_pid_t pid,rt_uint8_t pid_type)
{
	RT_ASSERT(motor != RT_NULL);
	if(pid_type == PID_TYPE_VEL)
	{
		motor->pid_vel = pid;
	}
	else if(pid_type == PID_TYPE_POS)
	{
		motor->pid_pos = pid;
	}
	else if(pid_type == PID_TYPE_CURRENT)
	{
		motor->pid_current = pid;
	}
	
	return RT_EOK;
}

rt_err_t lt_motor_set_timer(lt_motor_t motor, lt_timer_t timer)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(timer != RT_NULL);
	motor->timer = timer;
	
	return RT_EOK;
}

float lt_motor_get_velocity(lt_motor_t motor, rt_uint32_t measure_time_ms)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->sensor != RT_NULL);
	return lt_sensor_get_velocity(motor->sensor,measure_time_ms*1000)/motor->reduction_ratio;	/* ms --> us, unit: rad/s */		
}

float lt_motor_get_position(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->sensor != RT_NULL);
	return lt_sensor_get_angle(motor->sensor)/motor->reduction_ratio;	/* unit: rad */
}

rt_err_t lt_motor_get_info(lt_motor_t motor, struct lt_motor_info* info)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(info != RT_NULL);
	rt_strcpy(info->name,motor->name);
	info->status = motor->status;
	info->type = motor->type;
	if(motor->sensor == RT_NULL)
	{
		info->position = 0;
		return RT_ERROR;
	}
	else
	{
		info->position = lt_sensor_get_angle(motor->sensor)/motor->reduction_ratio;
		return RT_EOK;
	}
}

rt_err_t lt_motor_control(lt_motor_t motor, int cmd, void* arg)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->control != RT_NULL);
	/* in order to fast output function call */
	if(cmd == MOTOR_CTRL_OUTPUT)
	{
		return motor->ops->control(motor,cmd,arg);
	}
	
	switch(cmd)
	{
		case MOTOR_CTRL_OUTPUT_PID:
		{
			float* velocity = (float*)arg;
			motor->pid_type = PID_TYPE_VEL;
			return _output_pid(motor,*velocity);
		}
		case MOTOR_CTRL_OUTPUT_ANGLE_PID:
		{
			float* position = (float*)arg;
			motor->pid_type = PID_TYPE_POS;
			return _output_pid(motor,*position);
		}
		case MOTOR_CTRL_GET_STATUS:
		{
			rt_uint8_t* status = (rt_uint8_t *)arg;
			*status = motor->status;
			break;
		}
		case MOTOR_CTRL_GET_POSITION:
		{
			float* res = (float*)arg;
			*res = lt_sensor_get_angle(motor->sensor)/motor->reduction_ratio;
			break;
		}
		case MOTOR_CTRL_GET_VELOCITY:
		{
			float* res = (float*)arg;
			*res = lt_sensor_get_velocity(motor->sensor,*res*1000)/motor->reduction_ratio;	/* ms --> us, unit: rad/s */	
			break;
		}
		case MOTOR_CTRL_DISABLE_PID:
		{
			motor->pid_type = 0;
			_disable_motor(motor);
			break;
		}
		default:
		{
			return motor->ops->control(motor,cmd,arg);
		}
	}
	
	return RT_EOK;
}

rt_err_t lt_motor_enable(lt_motor_t motor,rt_uint8_t dir)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->driver != RT_NULL);
	motor->status = MOTOR_STATUS_RUN;
	return lt_driver_enable(motor->driver,dir);
}

rt_err_t lt_motor_disable(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	return _disable_motor(motor);
}

rt_err_t lt_motor_delete(lt_motor_t motor)
{
	RT_ASSERT(motor != RT_NULL);
	RT_ASSERT(motor->ops != RT_NULL);
	RT_ASSERT(motor->ops->_delete != RT_NULL);
	_disable_motor(motor);
	lt_manager_delete_motor(motor);				/* move motor from the manager */
	
	return motor->ops->_delete(motor);
}

static rt_err_t _disable_motor(lt_motor_t motor)
{
	motor->status = MOTOR_STATUS_STOP;
	if(motor->timer != RT_NULL)
	{
		lt_timer_disable(motor->timer,TIMER_TYPE_HW);
		lt_timer_disable(motor->timer,TIMER_TYPE_SOFT);
	}
	if(motor->driver != RT_NULL)
	{
		lt_driver_disable(motor->driver);
	}
	
	return RT_EOK;
}

static void _output_pid_timeout(lt_timer_t timer)
{
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	/* check pid type*/
	lt_pid_t pid;
	float feedback, control_u;
	if(motor->pid_type == PID_TYPE_VEL)
	{
		pid = motor->pid_vel;
		feedback = lt_sensor_get_velocity(motor->sensor,pid->dt*1000000) * 9.55 / motor->reduction_ratio;	/* get rpm, unit: s --> us */
		control_u = PID_VEL_CONST * lt_pid_control(pid,feedback) ;				/* multiply a coefficiency */
	}
	else if(motor->pid_type == PID_TYPE_POS)
	{
		pid = motor->pid_pos;
		feedback = lt_sensor_get_angle(motor->sensor) * 180.0f/PI/ motor->reduction_ratio;				/* get angle */
		control_u = PID_POS_CONST * lt_pid_control(pid,feedback);				/* multiply a coefficiency */
	}
	else return;
	motor->ops->control(motor,MOTOR_CTRL_OUTPUT,&control_u);					/* finally output */
}
/* simple pid interface */
static rt_err_t _output_pid(lt_motor_t motor,float target)
{
	/* check needed part*/
	if(motor->sensor == RT_NULL || motor->driver == RT_NULL || motor->timer == RT_NULL) return RT_ERROR;
	lt_pid_t pid;
	lt_timer_t timer = motor->timer;
	rt_err_t res;
	
	if(motor->pid_type == PID_TYPE_VEL)
	{
		pid = motor->pid_vel;
	}
	else
	{
		pid = motor->pid_pos;
	}
	if(pid == RT_NULL) return RT_ERROR;
	
	lt_pid_set_target(pid,target);
	/* config hw_timer and start  */
	res = lt_timer_period_call(timer,pid->dt*1000000,_output_pid_timeout,motor,TIMER_TYPE_HW);	/* unit: s --> us */
	motor->status = MOTOR_STATUS_RUN;
	
	return res;
}


char * _status[4] = {"STOP","RUN","ACCELERATE","INTERP"};
char* _type[4] = {"UNKNOWN","DC","BLDC","STEPPER"};
