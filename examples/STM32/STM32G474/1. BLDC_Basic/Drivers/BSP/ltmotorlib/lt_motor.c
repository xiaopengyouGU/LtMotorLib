/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-25  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"

lt_motor_t lt_motor_create(const char* name)
{
	lt_motor_t _motor = (lt_motor_t)lt_malloc(sizeof(struct lt_motor_object));
	if(_motor == NULL) return NULL;
	memset(_motor,0,sizeof(struct lt_motor_object));
	/* copy device name */
	_motor->V_target = 0;
	strcpy(_motor->name, name);
	_motor->flag = DEVICE_FLAG_UNINIT;

	return _motor;
}

uint32_t lt_motor_check(lt_motor_t motor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(motor);
	LT_CHECK_NULL(motor->sensor);
	LT_CHECK_NULL(motor->driver);
	LT_CHECK_NULL(motor->current);
#endif
	
	lt_sensor_t sensor = motor->sensor;
	lt_current_t current = motor->current;
	lt_driver_t driver = motor->driver;
	/* start motor checking */
	/* calibrating position sensor and current sensor */
	uint32_t res, res2;
	uint8_t count = 0;
	while(1)
	{
		count++;
		res = lt_current_calibrate(current);
		res2 = lt_sensor_calibrate(sensor);
		if(res == LT_EOK && res2 == LT_EOK)		break;
		if(count >= 20)
		{
#ifdef LT_DEBUG_ENABLE
			LT_DEBUG(count < 20, "check motor failed!!! ");
#endif
			return LT_ERROR;			/* check failed !!! */
		}
		lt_delay_ms(5);
	}
	/* start all hardware drivers */
	lt_driver_start(driver);
	lt_current_start(current);
	lt_sensor_start(sensor);
	
	motor->flag |= MOTOR_FLAG_CHECKED;
	return LT_EOK;
}

void lt_motor_start(lt_motor_t motor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	if(!(motor->flag & MOTOR_FLAG_RUN))				/* motor is not started */
	{
		lt_driver_start(motor->driver);			/* stop motor */
		motor->flag |= MOTOR_FLAG_RUN;
		motor->flag = CLEAR_BITS(motor->flag,MOTOR_FLAG_STOP);
	}
}

void lt_motor_stop(lt_motor_t motor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	lt_driver_stop(motor->driver);			/* stop motor */
	motor->flag |= MOTOR_FLAG_STOP;
	motor->flag = CLEAR_BITS(motor->flag,MOTOR_FLAG_RUN);
}

void lt_motor_set(lt_motor_t motor, struct lt_motor_config* config)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(motor);
	LT_CHECK_NULL(config);
#endif
	if(config->name != NULL)
	{
		strcpy(motor->name,config->name);		/* set device name */
	}
	/* set device */
	motor->current = config->current;
	motor->driver = config->driver;
	motor->sensor = config->sensor;
	/* set pid object and controller */
	motor->vel_pid = config->vel_pid;
	motor->pos_pid = config->pos_pid;
	motor->curr_pid_d = config->curr_pid_d;
	motor->curr_pid_q = config->curr_pid_q;
	/* set other */
	motor->observer  = config->observer;
	motor->ctrl = config->ctrl;
	motor->type = config->type;
	/* set motor parameters */
	motor->param.curr_rated = config->param.curr_rated;
	motor->param.n_rated = config->param.n_rated;
	motor->param.Ke = config->param.Ke;
	motor->param.Kt = config->param.Kt;
	motor->param.Ls = config->param.Ls;
	motor->param.R = config->param.R;
	motor->param.pn = config->param.pn;
	motor->param.Vdc = config->param.Vdc;
	if(config->param.KV == 0)
	{
		motor->param.KV = 1000.0f / (config->param.Ke) / 2.0f;
	}
	else motor->param.KV = config->param.KV;
	
	motor->flag |= DEVICE_FLAG_INIT;
}

void lt_motor_set_pos(lt_motor_t motor, float target_pos)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	lt_pid_t pid = motor->pos_pid;
	if(motor->flag & MOTOR_FLAG_CLOSE_LOOP)
	{
		if(pid != NULL) lt_pid_set_target(pid,target_pos);
	}
	motor->info.target_pos = target_pos;
}

void lt_motor_set_vel(lt_motor_t motor, float target_vel)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	lt_pid_t pid = motor->vel_pid;
	if(motor->flag & MOTOR_FLAG_CLOSE_LOOP)
	{
		if(pid != NULL) lt_pid_set_target(pid,target_vel);
	}
	else	/* open loop output  */
	{
		float Vdc = motor->param.Vdc;
		float V_target = target_vel/motor->param.KV;
		motor->V_target = CONSTRAINS(V_target,Vdc,-Vdc);
	}
	
	motor->info.target_vel = target_vel;
}

void lt_motor_set_curr(lt_motor_t motor, float target_curr)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	lt_pid_t pid = motor->curr_pid_q;
	if(motor->flag & MOTOR_FLAG_CLOSE_LOOP)
	{
		if(pid != NULL) lt_pid_set_target(pid,target_curr);
	}
	motor->info.target_curr = target_curr;
}

void lt_motor_delete(lt_motor_t motor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(motor);
#endif
	if((motor->flag & MOTOR_FLAG_CHECKED))
	{
		lt_driver_stop(motor->driver);			/* stop motor */
		lt_current_stop(motor->current);
		lt_sensor_stop(motor->sensor);
		motor->flag = MOTOR_FLAG_UNINIT;
	}
	
	lt_free(motor);
}

float lt_motor_get_vtar(lt_motor_t motor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	return motor->V_target;
}

void lt_motor_monitor(lt_motor_t motor)		/* monitor key parameters */
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	/* current */
	lt_current_t current = motor->current;
	float I_bus,Id,Iq;
	lt_current_get_bus_async(current,&I_bus);
	lt_current_get_dq_async(current,&Id, &Iq);
	/* temperature and bus voltage */
	float temp, volt;
	lt_current_get_vt(current, &volt, &temp);
	/* position and velocity */
	float pos,vel;
	lt_sensor_get_async(motor->sensor,&pos,&vel);
#ifdef LT_COMMUTE_ENABLE
	
#else
	/* printf monitor information */
	printf("Idc = %.1f mA \r\n",I_bus*1000.0f);
	printf("Temp = %.1f degree, Vbus = %.1f V \r\n",temp,volt);
	printf("\r\n");
#endif
	/* save motor values */
	motor->info.pos = pos;
	motor->info.vel = vel;
	motor->info.Id = Id;
	motor->info.Iq = Iq;
	motor->info.I_bus = I_bus;
	motor->info.temp = temp;
	motor->info.vbus = volt;
	motor->info.flag = motor->flag;
}

lt_info_t lt_motor_get(lt_motor_t motor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_MOTOR(motor);
#endif
	return &(motor->info);
}
