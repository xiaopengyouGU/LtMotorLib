/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-27  	  Lvtou		   The first version
 * 2025-12-14	  Lvtou		   Velocity loop control
 * 2025-12-20	  Lvtou		   Modity velocity and current loop control, add breaked protection
 * 2025-12-20	  Lvtou		   Add over current protection	
 */
#include "ltmotorlib.h"

extern lt_current_t current;
extern lt_sensor_t sensor;
extern lt_driver_t driver;

static lt_trape_t trape;
static lt_pid_t pid_vel;
static lt_pid_t pid_curr;

static uint8_t hall_signal;
static uint8_t *pulse;
static float duty;
static float input;
static float vel;
static float pos;
static float I_bus;
static struct lt_motor_object motor_obj;
lt_motor_t motor = &motor_obj;

//static void _vf_control_startup(void);
//static void _current_loop(void);
//static void _velocity_loop(void);
//static void _locked_protection(void);
//static void _over_current_protection(void);

void drv_motor_init(void)
{
	if(trape != NULL || pid_vel != NULL) return;				/* avoid repeated initialization */
	/* Six-Step commutation part */
	trape = lt_trape_create(24,TRAPZOID_MODE_DEFAULT);			/* DC bus voltage */
	pid_vel = lt_pid_create(0.004,0.020,0.0,1);					/* sample period : 1ms */
	pid_curr = lt_pid_create(0.2,25,0.0,0.0625);				/* sample period : 0.0625ms = 62.5us ==> 16k */
	LT_CHECK_NULL(trape);
	LT_CHECK_NULL(pid_vel);
	LT_CHECK_NULL(pid_curr);
	lt_pid_set_limits(pid_vel,9.5,9.5);
	lt_pid_set_limits(pid_curr,24*0.95,24*0.95);
	/* configure motor */
	struct lt_motor_config config;
	config.name = "bldc";
	/* config device at first */
	config.current = current;
	config.sensor = sensor;
	config.driver = driver;
	/* config control algorithm */
	config.vel_pid = pid_vel;
	config.curr_pid_q = pid_curr;
	config.ctrl = trape;
	config.type = MOTOR_TYPE_BLDC;
	/* config motor parameters */
	config.param.pn = 4;			/* pole pairs */
	config.param.Ke = 4.3;			/* back emf constant, unit : V/krpm */
	config.param.Vdc = 24;			/* unit : votage */
	config.param.Ls = 0.0285;		/* inductance, unit : mH */
	config.param.R = 0.51;			/* resitance, unit : ohm */
	config.param.n_rated = 3000;	/* rated speed, unit : rpm */
	config.param.curr_rated = 4;	/* rated current, unit : A */
	config.param.KV = 0;			/* KV value, if unkonwn, set 0 */
	/* set motor finally */
	lt_motor_set(motor,&config);	
}

void ltm_motor_run(uint8_t loop_flag)
{
	if(motor->flag & MOTOR_FLAG_RUN)			/* */
	{
		if(loop_flag == LOOP_FLAG_CURRENT)
		{
			float dir = lt_motor_get_vtar(motor);
			if(dir == 0)		return;			/* motor stop */
			
			if(vel == 0)						/* preposition */
			{
				input = 1.5;					/* 1.5/24 ~= 12.5% duty cycle */
			}
			else 
			{
				/* in startup stage,  100 <= vel <= 670 */
				if(dir > 0)						/* forward rotation */
				{
					input = vel/4200.0f * 24.0f;
					lt_sensor_set_dir(sensor,DIR_CCW);
				}
				else							/* reversal rotation */
				{
					lt_sensor_set_dir(sensor,DIR_CW);
					input = -vel/4200.0f * 24.0f;
				}
			}
			
			/* we use software position sensor in this example, see drv_sensor.c for details */
			lt_sensor_get2(sensor,&hall_signal,&vel);
			pulse = lt_trape_process(trape,hall_signal,input,&duty);
			lt_driver_output2(driver,pulse,duty);
			
		}
	}
}

///* The implementaion of ltm_motor_run function */
//void ltm_motor_run(uint8_t loop_flag)
//{	
//	if(motor->flag & MOTOR_FLAG_RUN)			/* */
//	{
//		if(loop_flag == LOOP_FLAG_CURRENT)
//		{
//			_current_loop();
//		}
//		else if(loop_flag == LOOP_FLAG_VELOCITY)
//		{
//			_velocity_loop();
//		}
//		else
//		{
//			
//		}			
//	}
//	else if(motor->flag & MOTOR_FLAG_LOCKED)
//	{
//		_locked_protection();
//	}
//}	

//#define MAX_CURRENT 4
//static void _current_loop(void)
//{
//	static uint8_t flag = 0;
//	static uint16_t count2 = 0;
//	/* we used last pulse and input to get bus current 
//	 * since mechanical time const is so large compared with electrical one */
//	if(flag != 0)
//	{
//		//lt_current_get_bus(current,pulse,vel,&I_bus);				/* get dc current */
//		lt_current_get_bus(current,pulse,input,&I_bus);
//		input = lt_pid_process(pid_curr,I_bus);
//	}
//	else 
//	{	
//		/* at first time, pulse is NULL, so we don't get I_bus!!! */
//		flag = 1;
//	}	
//	/* over current protection */
//	if(fabsf(I_bus) > MAX_CURRENT)		count2++;
//	else								count2 = 0;
//	if(count2 == 500)													/* we assume the motor is over-current, about 30ms */
//	{
//		count2 = 0;
//		motor->flag = CLEAR_BITS(motor->flag, MOTOR_FLAG_RUN);
//		motor->flag |= MOTOR_FLAG_OVER_CURRENT;
//		_over_current_protection();
//		return;
//	}
//	
//	
//	if(vel > 0 && input < 0)
//	{
//		lt_sensor_set_dir(sensor,DIR_CCW);
//		input = 0;
//		flag = 2;
//	}
//	else if(vel < 0 && input > 0)
//	{
//		lt_sensor_set_dir(sensor,DIR_CW);
//		input = 0;
//		flag = 2;
//	}
//	else if(vel == 0)
//	{
//		if(input >= 0) 	lt_sensor_set_dir(sensor,DIR_CCW);
//		else			lt_sensor_set_dir(sensor,DIR_CW);
//			
//		if(flag == 2)
//		{
//			lt_pid_reset(pid_curr);
//			flag = 1;
//			input = 0;
//		}
//	}

//	lt_sensor_get2(sensor,&hall_signal,&vel);			
//	pulse = lt_trape_process(trape,hall_signal,input,&duty);		/* get output pulse sequence */
//	
//	lt_driver_output2(driver,pulse,duty);							/* PWM output */
//}

//#define MIN_SPEED	20
//static void _velocity_loop(void)
//{
//	static uint8_t count = 0;
//	if(fabsf(vel) < MIN_SPEED)		count++;
//	else							count = 0;
//	
//	if(count == 60)													/* we assume the motor is locked, 60ms */
//	{
//		count = 0;
//		motor->flag = CLEAR_BITS(motor->flag, MOTOR_FLAG_RUN);
//		motor->flag |= MOTOR_FLAG_LOCKED;
//		return;
//	}
//		
//	float target_curr = lt_pid_process(pid_vel,vel);
//	lt_pid_set_target(pid_curr,target_curr);
//}

//static void _locked_protection(void)
//{
//	vel = 0;
//	I_bus = 0;
//	lt_pid_reset(pid_vel);
//	lt_pid_reset(pid_curr);
//	lt_driver_output2(driver,pulse,0);						/* no PWM output */
//	
//	motor->flag = CLEAR_BITS(motor->flag, MOTOR_FLAG_LOCKED);
//	motor->flag |= MOTOR_FLAG_RUN;
//}

//static void _over_current_protection(void)
//{
//	vel = 0;
//	I_bus = 0;
//	lt_pid_reset(pid_vel);
//	lt_pid_reset(pid_curr);
//	lt_driver_output2(driver,pulse,0);						/* no PWM output */
//	
//	motor->flag = CLEAR_BITS(motor->flag, MOTOR_FLAG_OVER_CURRENT);
//	motor->flag |= MOTOR_FLAG_RUN;
//}
