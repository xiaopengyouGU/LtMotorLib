/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-27  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"

extern lt_current_t current;
extern lt_sensor_t sensor;
extern lt_driver_t driver;
lt_trape_t trape;
uint8_t hall_signal;
uint8_t *pulse;
float duty;
float input;
float vel;
float pos;
float I_bus;
lt_pid_t pid_vel;
//lt_pid_t pid_curr;
static struct lt_motor_object motor_obj;
lt_motor_t motor = &motor_obj;


void drv_motor_init(void)
{
	if(trape != NULL || pid_vel != NULL) return;				/* avoid repeated initialization */
	/* Six-Step commutation part */
	trape = lt_trape_create(24,TRAPZOID_MODE_DEFAULT);			/* DC bus voltage */
	pid_vel = lt_pid_create(0.1,2.0,0.0,10.0);
	lt_pid_set_target(pid_vel,1000);
	LT_CHECK_NULL(trape);
	LT_CHECK_NULL(pid_vel);
	/* configure motor */
	struct lt_motor_config config;
	config.name = "bldc";
	/* config device at first */
	config.current = current;
	config.sensor = sensor;
	config.driver = driver;
	/* config control algorithm */
	config.vel_pid = pid_vel;
	config.ctrl = trape;
	config.type = MOTOR_TYPE_BLDC;
	/* config motor parameters */
	config.param.pn = 4;			/* pole pairs */
	config.param.Ke = 4.3;			/* back emf constant, unit : V/krpm */
	config.param.Vdc = 24;			/* unit : votage */
	config.param.Ls = 0.59;			/* inductance, unit : mH */
	config.param.R = 1.02;			/* resitance, unit : ohm */
	config.param.n_rated = 3000;	/* rated speed, unit : rpm */
	config.param.curr_rated = 4;	/* rated current, unit : A */
	config.param.KV = 0;			/* KV value, if unkonwn, set 0 */
	/* set motor finally */
	lt_motor_set(motor,&config);	
}

/* The implementaion of ltm_motor_run function */
void ltm_motor_run(void)
{	
	if(motor->flag & MOTOR_FLAG_RUN)
	{
		input = lt_motor_get_vtar(motor);		
		if(input < 0)
		{
			lt_sensor_set_dir(sensor,DIR_CW);
		}
		else
		{
			lt_sensor_set_dir(sensor,DIR_CCW);
		}
		
		lt_sensor_get2(sensor,&hall_signal,&vel);			
		pulse = lt_trape_process(trape,hall_signal,input,&duty);	/* get output pulse sequence */
		lt_current_get_bus(current,pulse,input,&I_bus);				/* get dc current */
		
		lt_driver_output2(driver,pulse,duty);						/* PWM output */
	}
	
}	

