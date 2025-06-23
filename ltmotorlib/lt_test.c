/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "protocol.h"
#include "stdlib.h"

/* when communicating with upper computer, FINSH is forbidden */
/* basic command, eg: MOTOR_CTRL_OUTPUT	works on all motor
*  in fact, MOTOR_CTRL_OUTPUT = STEPPER_CTRL_OUPUT = BLDC_CTRL_OUTPUT
*/
rt_align(RT_ALIGN_SIZE)

/* dc motor config */
static lt_motor_t motor_dc;
static lt_driver_t driver_dc;
static lt_sensor_t sensor_dc;
static lt_timer_t timer_dc;

static lt_pid_t pid_pos_dc;		/* position pid */
static lt_pid_t pid_vel_dc;		/* velocity pid */

rt_err_t dc_callback(void *parameter)
{
	return rt_kprintf("motor_dc finished a output! \n");
}

rt_err_t test_motor_dc_config(void)
{
	/* create motor, driver, sensor, get pin number */
	rt_base_t forward = rt_pin_get("PG.6");
	rt_base_t reversal = rt_pin_get("PG.7");
	
	motor_dc = lt_motor_create("motor_dc",34,MOTOR_TYPE_DC);
	driver_dc = lt_driver_create(DRIVER_TYPE_DC);
	sensor_dc = lt_sensor_create(ENCODER_NAME_1,4*13,SENSOR_TYPE_ENCODER);
	timer_dc = lt_timer_create("dc_timer",TIMER_NAME_1,0);				
	pid_vel_dc = lt_pid_create(5.6,14.0,0.02,50);						/* sample time: 50ms, unit: ms */
	pid_pos_dc = lt_pid_create(8.0,0,0,50);							
	/* timer is not needed in all case, so as sensor */
	/* check res */
	if(motor_dc == RT_NULL || driver_dc == RT_NULL || sensor_dc == RT_NULL) return RT_ERROR;
	if(timer_dc == RT_NULL || pid_vel_dc == RT_NULL || pid_pos_dc == RT_NULL) return RT_ERROR;	

	/* config driver, here we use default config */
	lt_driver_set_pwm(driver_dc,PWM_NAME_1,2,0);						/* set pwm and channel */
	lt_driver_set_pins(driver_dc,forward, reversal,0);					/* set pins */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	lt_sensor_calibrate(sensor_dc);										/* calibrate sensor */
	
	/* config motor */
	lt_motor_set_driver(motor_dc,driver_dc);
	lt_motor_set_sensor(motor_dc,sensor_dc);
	lt_motor_set_timer(motor_dc,timer_dc);
	lt_motor_set_pid(motor_dc,pid_vel_dc,PID_TYPE_VEL);							/* Ltmotorlib provided simple pid interface for velocity and position pid */
	lt_motor_set_pid(motor_dc,pid_pos_dc,PID_TYPE_POS);
	lt_motor_set_callback(motor_dc,dc_callback,RT_NULL);						/* when finished a output, motor object will call callback function */
	
	return RT_EOK;
}

/* x stepper config */
static lt_motor_t x_stepper;
static lt_driver_t x_driver; 
static lt_sensor_t x_encoder;
static lt_timer_t x_timer;
static lt_pid_t pid_vel_x;
static lt_pid_t pid_pos_x;
/* when finish output, this function would be called */
rt_err_t x_callback(void *parameter)
{
	return rt_kprintf("x_stepper finished a output! \n");
}

rt_err_t test_stepper_x_config(void)
{
	/* create motor, driver, sensor, get pin number */
	rt_base_t forward = rt_pin_get("PD.13");
	rt_base_t enable = rt_pin_get("PD.12");
	struct lt_stepper_config config;
	
	x_stepper = lt_motor_create("x_stepper",1,MOTOR_TYPE_STEPPER);
	x_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
	x_encoder = lt_sensor_create(ENCODER_NAME_2,4*600,SENSOR_TYPE_ENCODER);
	x_timer = lt_timer_create("x_timer",TIMER_NAME_2,0);				
	pid_vel_x = lt_pid_create(28.2,40,0.01,100);							   /* sample time: 100ms, unit: ms */
	pid_pos_x = lt_pid_create(4,0,0,100);
	/* sample time: 50ms, unit: ms */
	/* timer is not needed in all case, so as sensor */
	/* check res */
	if(x_stepper == RT_NULL || x_driver == RT_NULL || x_encoder == RT_NULL) return RT_ERROR;
	if(x_timer == RT_NULL || pid_vel_x == RT_NULL  || pid_pos_x == RT_NULL) return RT_ERROR;	
	
	/* config driver, here we use default config */
	lt_driver_set_pwm(x_driver,PWM_NAME_1,3,0);							/* set pwm and channel */
	lt_driver_set_pins(x_driver,forward,0,enable);						/* set pins */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	lt_sensor_calibrate(x_encoder);										/* calibrate sensor */
	
	/* config motor */
	lt_motor_set_driver(x_stepper,x_driver);
	lt_motor_set_sensor(x_stepper,x_encoder);
	lt_motor_set_timer(x_stepper,x_timer);
	lt_motor_set_pid(x_stepper,pid_vel_x,PID_TYPE_VEL);					/* Ltmotorlib provided simple pid interface for velocity and position pid */
	lt_motor_set_pid(x_stepper,pid_pos_x,PID_TYPE_POS);
	lt_motor_set_callback(x_stepper,x_callback,RT_NULL);						/* when finished a output, motor object will call callback function */
	
	/* stepper part config */
	config.period = 20;													/* 20ms unit: ms */
	config.stepper_angle = 1.8;											/* unit: degree */
	config.subdivide = 2;					
	return lt_motor_control(x_stepper,STEPPER_CTRL_CONFIG,&config);
}

/* y stepper config:
*  y_stepper is open loop output, so we don't configure position sensor and pid
*  therefore accleration function and pid output function are forbiddened;
*/
static lt_motor_t y_stepper;
static lt_driver_t y_driver; 
static lt_timer_t y_timer;
/* when finish output, this function would be called */
rt_err_t y_callback(void *parameter)
{
	return rt_kprintf("y_stepper finished a output! \n");
}

rt_err_t test_stepper_y_config(void)
{
	/* create motor, driver, sensor, get pin number */
	rt_base_t forward = rt_pin_get("PE.10");
	rt_base_t enable = rt_pin_get("PF.13");
	struct lt_stepper_config config;
	
	y_stepper = lt_motor_create("y_stepper",1,MOTOR_TYPE_STEPPER);
	y_driver = lt_driver_create(DRIVER_TYPE_STEPPER);
	y_timer = lt_timer_create("y_timer",TIMER_NAME_3,0);							/* we don't select hardware timer */			
	/* timer is not needed in all case, so as sensor */
	/* check res */
	if(y_stepper == RT_NULL || y_driver == RT_NULL || y_timer == RT_NULL) return RT_ERROR;

	/* config driver, here we use default config */
	lt_driver_set_pwm(y_driver,PWM_NAME_1,4,0);							/* set pwm and channel */
	lt_driver_set_pins(y_driver,forward,0,enable);						/* set pins */
	//lt_driver_set_output(driver,1000,0.5,0);							/* set pwm period and duty_cycle, unit: us */
	/* we use default stepper timer config */
	
	/* config motor */
	lt_motor_set_driver(y_stepper,y_driver);
	lt_motor_set_timer(y_stepper,y_timer);
	lt_motor_set_callback(y_stepper,y_callback,RT_NULL);						/* when finished a output, motor object will call callback function */
	
	/* stepper part config */
	config.period = 20;													/* 20ms unit: ms */
	config.stepper_angle = 1.8;											/* unit: degree */
	config.subdivide = 2;												
	return lt_motor_control(y_stepper,STEPPER_CTRL_CONFIG,&config);
}

/* bldc motor config */
static lt_motor_t x_bldc;
static lt_driver_t driver_bldc_x;
static lt_sensor_t sensor_bldc_x;
static lt_timer_t timer_bldc_x;
static lt_current_t curr_bldc_x;

static lt_pid_t pid_curr_bldc_x;	/* current pid */
static lt_pid_t pid_pos_bldc_x;		/* position pid */
static lt_pid_t pid_vel_bldc_x;		/* velocity pid */

rt_err_t bldc_x_callback(void *parameter)
{
	return rt_kprintf("x_bldc finished a output! \n");
}

rt_err_t test_bldc_x_config(void)
{
	struct lt_bldc_config config;
	rt_base_t enable = rt_pin_get("PB.15");
	
	x_bldc = lt_motor_create("x_bldc",1,MOTOR_TYPE_BLDC);
	driver_bldc_x = lt_driver_create(DRIVER_TYPE_BLDC);
	sensor_bldc_x = lt_sensor_create(I2C_NAME_1,1,SENSOR_TYPE_MAGNETIC);
	timer_bldc_x = lt_timer_create("blx_timer",TIMER_NAME_4,0);				
	curr_bldc_x = lt_current_create(0.01,50,12);								/* shunt resistor: 10 mOhm */
	pid_vel_bldc_x = lt_pid_create(1.96,1.6,0.01,5);							/* sample time: 20ms, unit: ms */
	pid_pos_bldc_x = lt_pid_create(10,0,0,15);		
	pid_curr_bldc_x = lt_pid_create(10,0,0,5);
	lt_pid_set_target(pid_curr_bldc_x,100);
	/* check res */
	if(x_bldc == RT_NULL || driver_bldc_x == RT_NULL || sensor_bldc_x == RT_NULL || curr_bldc_x == RT_NULL) return RT_ERROR;
	if(timer_bldc_x == RT_NULL || pid_vel_bldc_x == RT_NULL || pid_pos_bldc_x == RT_NULL || pid_curr_bldc_x == RT_NULL) return RT_ERROR;	

	/* config driver, here we use default config */
	lt_driver_set_pwm(driver_bldc_x,PWM_NAME_2,2,PWM_PHASE_A);				/* set three phases pwm and channels */
	lt_driver_set_pwm(driver_bldc_x,PWM_NAME_2,3,PWM_PHASE_B);				/* set three phases pwm and channels */
	lt_driver_set_pwm(driver_bldc_x,PWM_NAME_2,4,PWM_PHASE_C);				/* set three phases pwm and channels */
	lt_driver_set_pins(driver_bldc_x,0, 0,enable);							
	lt_sensor_calibrate(sensor_bldc_x);										/* calibrate sensor */
	
	/* config current sense */
	lt_current_set_adc(curr_bldc_x,ADC_NAME_1,6,11,-1);						/* set adc and channel, -1 means ignored */
	lt_current_calibrate(curr_bldc_x);										/* calibrate current sense */	
	
	/* config motor */
	lt_motor_set_driver(x_bldc,driver_bldc_x);
	lt_motor_set_sensor(x_bldc,sensor_bldc_x);
	lt_motor_set_timer(x_bldc,timer_bldc_x);
	lt_motor_set_pid(x_bldc,pid_vel_bldc_x,PID_TYPE_VEL);					/* Ltmotorlib provided simple pid interface for velocity and position pid */
	lt_motor_set_pid(x_bldc,pid_pos_bldc_x,PID_TYPE_POS);
	lt_motor_set_pid(x_bldc,pid_curr_bldc_x,PID_TYPE_CURRENT);
	lt_motor_set_callback(x_bldc,bldc_x_callback,RT_NULL);					/* when finished a output, motor object will call callback function */
	
	/* bldc part config */
	config.current = curr_bldc_x;
	config.foc_type = FOC_TYPE_DEFAULT;
	config.inductance = 0.00425;											/* unit: H */
	config.KV = 100;						
	config.resistance = 8;												    /* unit: Ohm */
	config.poles = 7;														/* pole pairs */
	config.max_volt = 12;													/* unit: Voltage */
	
	return lt_motor_control(x_bldc,BLDC_CTRL_CONFIG,&config);
}

/* bldc motor config */
static lt_motor_t y_bldc;
static lt_driver_t driver_bldc_y;
static lt_sensor_t sensor_bldc_y;
static lt_timer_t timer_bldc_y;
static lt_current_t curr_bldc_y;

static lt_pid_t pid_curr_bldc_y;	/* current pid */
static lt_pid_t pid_pos_bldc_y;		/* position pid */
static lt_pid_t pid_vel_bldc_y;		/* velocity pid */

rt_err_t bldc_y_callback(void *parameter)
{
	return rt_kprintf("y_bldc finished a output! \n");
}

rt_err_t test_bldc_y_config(void)
{
	struct lt_bldc_config config;
	rt_base_t enable = rt_pin_get("PF.12");
	
	y_bldc = lt_motor_create("y_bldc",1,MOTOR_TYPE_BLDC);
	driver_bldc_y = lt_driver_create(DRIVER_TYPE_BLDC);
	sensor_bldc_y = lt_sensor_create(I2C_NAME_2,1,SENSOR_TYPE_MAGNETIC);
	timer_bldc_y = lt_timer_create("bly_timer",TIMER_NAME_5,0);				
	curr_bldc_y = lt_current_create(0.01,50,12);							/* shunt resistor: 10 mOhm */
	pid_vel_bldc_y = lt_pid_create(1.96,1.6,0.01,10);						/* sample time: 20ms, unit: ms */
	pid_pos_bldc_y = lt_pid_create(10.1,23,0.01,10);		
	pid_curr_bldc_y = lt_pid_create(10,10,10,20);
	/* check res */
	if(y_bldc == RT_NULL || driver_bldc_y == RT_NULL || sensor_bldc_y == RT_NULL || curr_bldc_y == RT_NULL) return RT_ERROR;
	if(timer_bldc_y == RT_NULL || pid_vel_bldc_y == RT_NULL || pid_pos_bldc_y == RT_NULL || pid_curr_bldc_y == RT_NULL) return RT_ERROR;	

	/* config driver, here we use default config */
	lt_driver_set_pwm(driver_bldc_y,PWM_NAME_3,1,PWM_PHASE_A);				/* set three phases pwm and channels */
	lt_driver_set_pwm(driver_bldc_y,PWM_NAME_3,2,PWM_PHASE_B);				/* set three phases pwm and channels */
	lt_driver_set_pwm(driver_bldc_y,PWM_NAME_3,3,PWM_PHASE_C);				/* set three phases pwm and channels */
	lt_driver_set_pins(driver_bldc_y,0, 0,enable);								/* no need to set pins */
	lt_sensor_calibrate(sensor_bldc_y);										/* calibrate sensor */
	
	/* config current sense */
	lt_current_set_adc(curr_bldc_y,ADC_NAME_1,12,15,-1);					/* set adc and channel, -1 means ignored */
	lt_current_calibrate(curr_bldc_y);										/* calibrate current sense */	
	
	/* config motor */
	lt_motor_set_driver(y_bldc,driver_bldc_y);
	lt_motor_set_sensor(y_bldc,sensor_bldc_y);
	lt_motor_set_timer(y_bldc,timer_bldc_y);
	lt_motor_set_pid(y_bldc,pid_vel_bldc_y,PID_TYPE_VEL);					/* Ltmotorlib provided simple pid interface for velocity and position pid */
	lt_motor_set_pid(y_bldc,pid_pos_bldc_y,PID_TYPE_POS);
	lt_motor_set_pid(y_bldc,pid_curr_bldc_y,PID_TYPE_CURRENT);
	lt_motor_set_callback(y_bldc,bldc_y_callback,RT_NULL);					/* when finished a output, motor object will call callback function */
	
	/* bldc part config */
	config.current = curr_bldc_y;
	config.foc_type = FOC_TYPE_6_TRAPZOID;
	config.inductance = 0.00425;										    /* unit: H */
	config.KV = 100;						
	config.resistance = 8;												 	/* unit: Ohm */
	config.poles = 7;														/* pole pairs */
	config.max_volt = 12;													/* unit: Voltage */
	
	return lt_motor_control(y_bldc,BLDC_CTRL_CONFIG,&config);
}
/***************************************************************************************************/
static rt_thread_t _process;

void test_pid_simple(lt_timer_t timer)
{
	/* eg: KP = 0.26, KI = 0.9, KD = 0.02 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = motor->pid_vel;
	float read_val = lt_pid_get_control(pid);
	float curr_val = lt_pid_control(pid,read_val);
	int temp = curr_val;									/* transform data */
	static rt_uint32_t count = 0;
	if (count % TEST_PID_COMMUT_TIMES == 0)
	{				
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&temp,1);
	}
	count++;
}


void test_velocity_loop(lt_timer_t timer)
{
	/* eg: KP = 5.6, KI = 2.0, KD = 0.01 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = motor->pid_vel;
	float vel = lt_motor_get_velocity(motor,pid->dt*1000) * 9.55;	/* get rpm, s --> ms */
	float control_u = PID_VEL_CONST * lt_pid_control(pid,vel);		/* multiply a coefficiency, default: 0.15f */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);			/* output */
	/* send data to upper computer */
	static rt_uint32_t count = 0;
	if (count % TEST_PID_COMMUT_TIMES == 0)
	{
		int t_vel = 10*vel, t_control_u = 10*control_u;							
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_vel,1);
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
	}
	count++;
}

void test_position_loop(lt_timer_t timer)
{
	/* eg: KP = 8.0 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = motor->pid_pos;
	float position = lt_motor_get_position(motor)*180.0f/PI;		/* get motor angle, unit: degree */
	float control_u = PID_POS_CONST * lt_pid_control(pid,position);	/* multiply 0.15f as default  */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);
	int t_position = position,t_control_u = control_u;	    /* let control curve smoother */	
	/* send data to upper computer */
	static rt_uint32_t i;
	i++;
	if (i % TEST_PID_COMMUT_TIMES == 0)
	{
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
	}
}

void test_current_loop(lt_timer_t timer)
{
	/* eg: KP = 8.0 */
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = motor->pid_current;
	static struct lt_bldc_info info;
	lt_motor_control(motor,BLDC_CTRL_GET_ELECTRIC_INFO,&info);
	float Iq = info.Iq;		/* get q axis current, unit: A */
	float Ia = info.Ia, Ib = info.Ib;
	float control_u = PID_POS_CONST * lt_pid_control(pid,Iq);	/* multiply 0.15f as default  */
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);
	int t_Iq = Iq*1000,t_control_u = control_u;	    /* let control curve smoother */
	int t_Ia = Ia*1000, t_Ib = Ib*1000;
	/* send data to upper computer */
	static rt_uint32_t i;
	i++;
	if (i % TEST_PID_COMMUT_TIMES == 0)
	{
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_Iq,1);
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_control_u,1);
#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_Ia,1);
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH4,&t_Ib,1);
#endif
	}
}

void test_position_velocity_loop(lt_timer_t timer)
{
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid_vel = motor->pid_vel;
	lt_pid_t pid_pos = motor->pid_pos;
	/* we adjust vel_pid first */
	/* vel_pid: DC motor: Kp = 5.6, Ki = 2.0, Kd = 0.01 */
	/* pos_pid: DC motor: Kp = 10, Ki = 0, Kd = 0 */
	static rt_uint32_t count = 0;
	/* process position loop first, sample time 3T */
	if(count % 3 == 0)
	{
		float position = lt_motor_get_position(motor)*180.0f/PI;	/* unit: degree */
		float desired_vel = PID_POS_CONST * lt_pid_control(pid_pos,position);
		//float desired_vel = 0.0225 * lt_pid_control(pid_pos,position); /* for stepper pos_vel pid */
		lt_pid_set_target(pid_vel,desired_vel);
		if(count % (3*TEST_PID_COMMUT_TIMES) == 0)
		{
			int t_position = position;
			lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
		}
	}
	/* velocity loop follow, sample time T */
	float vel = lt_motor_get_velocity(motor,pid_vel->dt*1000)*9.55;	/* get rpm, s --> ms */
	float control_u = PID_VEL_CONST * lt_pid_control(pid_vel,vel);
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
	int t_speed = vel, t_control_u = control_u;
	
	if(count % TEST_PID_COMMUT_TIMES == 0)
	{
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_speed,1);
#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_control_u,1);
#endif
	}
	count++;
}

//void test_velocity_current_loop(lt_timer_t timer)
//{
//	lt_motor_t motor = (lt_motor_t)timer->user_data;
//	lt_pid_t pid_vel = motor->pid_vel;
//	lt_pid_t pid_current = motor->pid_current;
//	/* we adjust current pid first */
//	static rt_uint32_t count = 0;
//	static struct lt_bldc_info info;
//	/* process velocity loop first, sample time 3T */
//	if(count % 3 == 0)
//	{
//		float vel = lt_motor_get_velocity(motor,pid_vel->dt*1000)*9.55;	/* unit: rpm */
//		float desired_curr = PID_VEL_CONST * lt_pid_control(pid_vel,vel);
//		//float desired_vel = 0.0225 * lt_pid_control(pid_pos,position); /* for stepper pos_vel pid */
//		lt_pid_set_target(pid_current,desired_curr);
//		if(count % (3*TEST_PID_COMMUT_TIMES) == 0)
//		{
//			int t_vel = vel;
//			lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_vel,1);
//		}
//	}
//	/* current loop follow, sample time T */
//	lt_motor_control(motor,BLDC_CTRL_GET_ELECTRIC_INFO,&info);	/* unit: degree */
//	float Iq = info.Iq;
//	float control_u = PID_CURR_CONST * lt_pid_control(pid_current,Iq);
//	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
//	int t_current = Iq, t_control_u = control_u;
//	
//	if(count % TEST_PID_COMMUT_TIMES == 0)
//	{
//		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_current,1);
//#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
//		lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_control_u,1);
//#endif
//	}
//	count++;
//}

//void test_position_current_loop(lt_timer_t timer)
//{
//	lt_motor_t motor = (lt_motor_t)timer->user_data;
//	lt_pid_t pid_pos = motor->pid_pos;
//	lt_pid_t pid_current = motor->pid_current;
//	/* we adjust current pid first */
//	static rt_uint32_t count = 0;
//	static struct lt_bldc_info info;
//	/* process velocity loop first, sample time 3T */
//	if(count % 3 == 0)
//	{
//		float pos = lt_motor_get_position(motor)*180.0f/PI;	/* unit: rpm */
//		float desired_curr = PID_VEL_CONST * lt_pid_control(pid_pos,pos);
//		lt_pid_set_target(pid_current,desired_curr);
//		if(count % (3*TEST_PID_COMMUT_TIMES) == 0)
//		{
//			int t_pos = pos;
//			lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_pos,1);
//		}
//	}
//	/* current loop follow, sample time T */
//	lt_motor_control(motor,BLDC_CTRL_GET_ELECTRIC_INFO,&info);
//	float Iq = info.Iq;											/* unit: A */
//	float control_u = PID_CURR_CONST * lt_pid_control(pid_current,Iq);
//	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
//	int t_current = Iq, t_control_u = control_u;
//	
//	if(count % TEST_PID_COMMUT_TIMES == 0)
//	{
//		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_current,1);
//#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
//		lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_control_u,1);
//#endif
//	}
//	count++;
//}

//void test_position_velocity_current_loop(lt_timer_t timer)
//{
//	lt_motor_t motor = (lt_motor_t)timer->user_data;
//	lt_pid_t pid_pos = motor->pid_pos;
//	lt_pid_t pid_vel = motor->pid_vel;
//	lt_pid_t pid_current = motor->pid_current;
//	/* we adjust current pid first */
//	static rt_uint32_t count = 0;
//	static struct lt_bldc_info info;
//	/* process position loop first, sample time 9T */
//	if(count % 9 == 0)
//	{
//		float pos = lt_motor_get_position(motor)*180.0f/PI;	/* unit: rpm */
//		float desired_vel = PID_POS_CONST * lt_pid_control(pid_pos,pos);
//		lt_pid_set_target(pid_pos,desired_vel);
//		if(count % (9*TEST_PID_COMMUT_TIMES) == 0)
//		{
//			int t_pos = pos;
//			lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_pos,1);
//		}
//	}
//	/* velocity loop follow, sample time 3T */
//	if(count % 3 == 0)
//	{
//		float vel = lt_motor_get_velocity(motor,pid_vel->dt*1000)*9.55;	/* unit: rpm */
//		float desired_curr = PID_VEL_CONST * lt_pid_control(pid_vel,vel);
//		//float desired_vel = 0.0225 * lt_pid_control(pid_pos,position); /* for stepper pos_vel pid */
//		lt_pid_set_target(pid_current,desired_curr);
//		if(count % (3*TEST_PID_COMMUT_TIMES) == 0)
//		{
//			int t_vel = vel;
//			lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_vel,1);
//		}
//	}
//	/* current loop follow, sample time T*/
//	lt_motor_control(motor,BLDC_CTRL_GET_ELECTRIC_INFO,&info);
//	float Iq = info.Iq;/* unit: A */
//	float control_u = PID_CURR_CONST * lt_pid_control(pid_current,Iq);
//	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
//	int t_current = Iq, t_control_u = control_u;

//#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE	
//	if(count % TEST_PID_COMMUT_TIMES == 0)
//	{
//		lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_current,1);
//		lt_communicator_send(SEND_FACT_CMD,CURVES_CH4,&t_control_u,1);
//	}
//#endif
//	count++;
//}
/*************************************************************/
/* communicate with upper computer */
static void process_thread_entry(void* parameter)
{
	lt_timer_t timer = (lt_timer_t)parameter;	
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid = motor->pid_vel;;
	
#ifdef TEST_PID_POS	
	pid = motor->pid_pos;
#endif
#ifdef TEST_PID_CURR
	pid = motor->pid_current;
#endif
	
	rt_uint8_t cmd_type = CMD_NONE;     /* command type */
	struct lt_pid_info info;			/* pid info */
	
	while(1)
	{
		cmd_type = lt_communicator_receive(&info);
		switch (cmd_type)
		{
			case SET_PID_CMD:
			{
				lt_pid_set(pid,info.Kp, info.Ki, info.Kd);    /* set pid */
				break;
			}
			case SET_TARGET_CMD:
			{
				lt_pid_set_target(pid,info.target);    
				break;
			}
			case START_CMD:
			{
				/* in fact, we control motor in timer's timeout function */
				lt_timer_enable(timer,TEST_PID_TIMER_TYPE);
				break;
			}
			case STOP_CMD:
			{
				lt_motor_disable(motor);				/* disable motor and timer */
				lt_timer_disable(timer,TEST_PID_TIMER_TYPE);
				lt_communicator_send(SEND_STOP_CMD,CURVES_CH1,RT_NULL,0);
				break;
			}
			case RESET_CMD:
			{
				lt_pid_reset(pid);				 		 /* reset pid! */
				lt_motor_disable(motor);				 /* disable motor and timer */
				lt_timer_disable(timer,TEST_PID_TIMER_TYPE);
				lt_communicator_send(SEND_STOP_CMD,CURVES_CH1,RT_NULL,0);
				break;
			}
			case SET_PERIOD_CMD:
			{
				lt_pid_set_dt(pid,info.dt);
				lt_timer_set(timer,info.dt*1000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* ms --> us */
				break;
			}
			default:break;
		}
		rt_thread_mdelay(100);
  }

}

/* motor close loop test frame */
rt_err_t test_close_loop_pid(char*name)
{
	/* check parameter */
	lt_motor_t motor = lt_manager_get_motor(name);
	static struct lt_motor_info info;
	lt_motor_get_info(motor,&info);
	
	if(motor == RT_NULL) return RT_ERROR;
	if(info.type == MOTOR_TYPE_BLDC)		/* for bldc motor, we should disable output with timer */
	{
		lt_motor_control(motor,BLDC_CTRL_OUTPUT_NO_TIMER,RT_NULL);
	}
	
	lt_timer_t timer = motor->timer;
	lt_pid_t pid_vel = motor->pid_vel;
	lt_pid_t pid_pos = motor->pid_pos;
	lt_pid_t pid_current = motor->pid_current;
	if(timer == RT_NULL) return RT_ERROR;
	if(pid_vel == RT_NULL && pid_pos == RT_NULL &&pid_current == RT_NULL) return RT_ERROR;
	/* we already config motor and use user_data variable to transport data
	* software timer can be used if sample time is relatively large, such as 100ms */
	
	timer->user_data = motor;
	lt_timer_set_timeout(timer,TEST_PID_CALLBACK,TEST_PID_TIMER_TYPE);					/* set timer timeout function */
	lt_timer_set(timer,pid_vel->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* s --> us */	
#ifdef TEST_PID_POS
	lt_timer_set(timer,pid_pos->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* s --> us */	
#endif
#ifdef TEST_PID_CURR
	lt_timer_set(timer,pid_current->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);/* s --> us */	
#endif
	
#ifdef TEST_PID_POS_VEL	
	lt_timer_set(timer,pid_vel->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* s --> us */
	lt_pid_set_dt(pid_pos,pid_vel->dt*3000);							/* s --> ms */
#endif
#ifdef TEST_PID_POS_CURR
	lt_timer_set(timer,pid_current->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* s --> us */
	lt_pid_set_dt(pid_pos,pid_current->dt*3000);						/* s --> ms */
#endif
#ifdef TEST_PID_VEL_CURR
	lt_timer_set(timer,pid_current->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* s --> us */
	lt_pid_set_dt(pid_vel,pid_current->dt*3000);						/* s --> ms */
#endif
#ifdef TEST_PID_THREE_LOOP
	lt_timer_set(timer,pid_current->dt*1000000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* s --> us */
	lt_pid_set_dt(pid_vel,pid_current->dt*3000);						/* s --> ms */
	lt_pid_set_dt(pid_pos,pid_vel->dt*3000);							/* s --> ms */
#endif	

#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
/* create process thread */
	_process = rt_thread_create("_process",		/* create a process thread */
								process_thread_entry,
								timer,	
								2048,
								5,				
								20);
	if(_process == RT_NULL)
	{
		rt_kprintf("create process thread failed! \n");
	}
	rt_thread_startup(_process);
#endif
	
	return RT_EOK;
}


/****************************************************************************************/
#ifdef LT_USING_MOTOR_MSH_TEST
/* test functions for convenience and as examples */
void test_motor_output(lt_motor_t motor,float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&input);
	rt_kprintf("motor output: %.2f \n",input);
}

void test_motor_output_angle(lt_motor_t motor, float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT_ANGLE,&input);
	rt_kprintf("motor output angle %.2f degree \n",input);
}

void test_motor_output_pid(lt_motor_t motor, float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT_PID,&input);
	rt_kprintf("motor output pid : %.2f rpm \n",input);
}

void test_motor_output_angle_pid(lt_motor_t motor, float input)
{
	if(motor == RT_NULL) return;
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT_ANGLE_PID,&input);
	rt_kprintf("motor output angle pid : %.2f degree \n",input);
}

void test_motor_get_position(lt_motor_t motor)
{
	if(motor == RT_NULL) return;
	float pos = lt_motor_get_position(motor);
	rt_kprintf("motor position: %.2f rad, %.2f degree \n",pos,pos*180.0f/PI);
}

void test_motor_get_velocity(lt_motor_t motor)
{
	if(motor == RT_NULL) return;
	rt_tick_t tick = rt_tick_get();
	float vel;
	float pos = lt_motor_get_position(motor);
	rt_thread_delay(100);					/* delay 100ms */
	tick = rt_tick_get() - tick;			/* get measure time, unit: ms */
	pos = lt_motor_get_position(motor) - pos;
	vel = pos/tick * 1000;					
	rt_kprintf("motor velocity: %.2f rad/s, %.2f rpm , measure time: %d ms \n",vel,vel*9.55,tick);
}

void test_stepper_trapzoid(lt_motor_t motor, int step,float acc, float dec,float speed)
{
	if(motor == RT_NULL) return;	
	static struct lt_curve_config config; 		/* use static variable to avoid being distroyed */
	rt_err_t res;
	config.acc = acc;
	config.dec = dec;
	config.target = speed;
	config.step = step;
	config.type = CURVE_TYPE_TRAPZOID;
	res = lt_motor_control(motor,STEPPER_CTRL_ACCELERATE,&config);
	
	if(res == RT_EOK)
	{
		rt_kprintf("stepper trapzoid accel ==> step:%d,  accel:%.2f Hz/ms,  decel:%.2f Hz/ms,  speed:%.2f Hz \n",step,acc,dec,speed);
	}
	else
	{
		rt_kprintf("stepper config trapzoid failed! \n");
	}
}

void test_stepper_s_curve(lt_motor_t motor, int step, float acc_t,float freq_max, float freq_min, float flexible)
{
	if(motor == RT_NULL) return;	
	static struct lt_curve_config config; 		/* use static variable to avoid being distroyed */
	rt_err_t res;
	config.acc = acc_t;
	config.target = freq_max;
	config.initial = freq_min;
	config.flexible = flexible;
	config.step = step;
	config.type = CURVE_TYPE_S_CURVE;
	res = lt_motor_control(motor,STEPPER_CTRL_ACCELERATE,&config);
	
	if(res == RT_EOK)
	{
		rt_kprintf("stepper s_curve accel ==> step:%d,  acc_t:%.2f ms, freq_max:%.2f Hz,  freq_min:%.2f Hz,  flexible:%.2f \n",step,acc_t,freq_max,freq_min,flexible);
	}
	else
	{
		rt_kprintf("stepper config s_curve failed! \n");
	}
	
}

void test_stepper_5_section(lt_motor_t motor,int step,float acc_t, float speed)
{
	if(motor == RT_NULL) return;	
	static struct lt_curve_config config; 		/* use static variable to avoid being distroyed */
	rt_err_t res;
	config.acc = acc_t;
	config.initial = 0;
	config.target = speed;
	config.step = step;
	config.type = CURVE_TYPE_5_SECTION;
	res = lt_motor_control(motor,STEPPER_CTRL_ACCELERATE,&config);
	
	if(res == RT_EOK)
	{
		rt_kprintf("stepper 5_section accel ==> step:%d,  acc_t:%.2f ms, speed:%.2f Hz \n",step,acc_t,speed);
	}
	else
	{
		rt_kprintf("stepper config 5_section failed! \n");
	}
}

void test_stepper_line_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end)
{
	if(x_motor == RT_NULL) return;	
	if(y_motor == RT_NULL) return;	
	static struct lt_interp_config config;		/* use static variable to avoid being distroyed */
	rt_err_t res;
	config.x_start = x_start;
	config.y_start = y_start;
	config.x_end = x_end;
	config.y_end = y_end;
	config.radius = 0;							/* means line interp */
	config.y_motor = y_motor;
	res = lt_motor_control(x_motor,STEPPER_CTRL_INTERPOLATION,&config);
	
	if(res == RT_EOK)
	{
		rt_kprintf("stepper line interp ==> start:(%d, %d), end:(%d, %d) \n",x_start, y_start,x_end,y_end);
	}
	else
	{
		rt_kprintf("stepper config line interp failed! \n");
	}
}

void test_stepper_circular_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end,rt_uint16_t radius, rt_uint8_t dir)
{
	if(x_motor == RT_NULL) return;	
	if(y_motor == RT_NULL) return;	
	static struct lt_interp_config config;		/* use static variable to avoid being distroyed */
	rt_err_t res;
	char* direction;
	config.x_start = x_start;
	config.y_start = y_start;
	config.x_end = x_end;
	config.y_end = y_end;
	config.dir = dir;
	config.radius = radius;
	config.y_motor = y_motor;
	
	if(dir == DIR_CW)	direction = "CW";
	else				direction = "CCW";
	res = lt_motor_control(x_motor,STEPPER_CTRL_INTERPOLATION,&config);
	
	if(res == RT_EOK)
	{
		rt_kprintf("stepper circular interp ==> start:(%d, %d), end:(%d, %d), radius:%d	dir:%s \n",x_start,y_start,x_end,y_end,radius,direction);
	}
	else
	{
		rt_kprintf("stepper config circular interp failed! \n");
	}
}

void test_bldc_torque(lt_motor_t motor,float input)
{
	if(motor == RT_NULL) return;
	rt_err_t res = lt_motor_control(motor,BLDC_CTRL_OUTPUT_TORQUE,&input);
	
	if(res == RT_EOK)
	{
		rt_kprintf("bldc output torque:%.2f N.m \n",input);
	}
	else
	{
		rt_kprintf("bldc output torque failed! \n");
	}
}

void test_bldc_elec_info(lt_motor_t motor)
{
	if(motor == RT_NULL) return;
	static struct lt_bldc_info info;
	rt_err_t res = lt_motor_control(motor,BLDC_CTRL_GET_ELECTRIC_INFO,&info);
	
	if(res == RT_EOK)
	{
		rt_kprintf("bldc electric info: angle_el:%.2f degree, Ia:%.3f A, Ib:%.3f A, Ic:%.3f A, Id:%.3f A, Iq:%.3f A \n",info.angle_el,info.Ia,info.Ib,info.Ic,info.Id,info.Iq);
	}
	else
	{
		rt_kprintf("bldc get electric info failed! \n");
	}
}
/****************************************************************************************/
///* provided examples*/

//rt_align(RT_ALIGN_SIZE)

///* example 1 : stepper moves periodically and trapzoid acceleration */
//static rt_sem_t sem_ex_1;
//static rt_thread_t thread_ex_1;
//static lt_motor_t motor_ex_1;

//rt_err_t test_example_1_callback(void* parameter)
//{
//	return rt_sem_release(sem_ex_1);					/* when motor finish a output, release this semaphore */
//}

//static void test_example_1_thread_entry(void* parameter)
//{
//	rt_err_t res;
//	rt_uint32_t count = 0;
//	rt_uint8_t i;
//	float input;
//	
//	for(i = 0; i < 6; i++)
//	{
//		if (i % 2 == 0)
//		{
//			test_stepper_trapzoid(motor_ex_1,-1000,0.1,0.1,200);
//		}
//		else
//		{
//			test_stepper_trapzoid(motor_ex_1,1000,0.1,0.1,200);
//		}
//		res = rt_sem_take(sem_ex_1,RT_WAITING_FOREVER);	/* if the motor doesn't finish a output, suspend the thread */
//		if(res != RT_EOK) 
//		{
//			rt_kprintf("Error!!! \n");
//			break;
//		}
//		rt_thread_mdelay(3000);							/* delay 3s before */
//	}
//	
//	
//	while(1)
//	{
//		count++;
//		if(count % 2 == 0)							/* reversal rotation */
//		{
//			input = -2000;
//		}
//		else										/* forward rotation */
//		{
//			input = 2000;
//		}
//		rt_thread_mdelay(2000);						/* delay 2s before motor move */
//		lt_motor_control(motor_ex_1,MOTOR_CTRL_OUTPUT_ANGLE,&input);
//		
//		res = rt_sem_take(sem_ex_1,RT_WAITING_FOREVER);	/* if the motor doesn't finish a output, suspend the thread */
//		if(res != RT_EOK) 
//		{
//			rt_kprintf("Error!!! \n");
//			break;
//		}
//	}
//}

//rt_err_t test_example_1_config(void)
//{
//	motor_ex_1 = lt_manager_get_motor("y_stepper");
//	if(motor_ex_1 == RT_NULL) return RT_ERROR;
//	lt_motor_set_callback(motor_ex_1,test_example_1_callback,RT_NULL);
//	
//	sem_ex_1 = rt_sem_create("ex1_sem",0,RT_IPC_FLAG_PRIO);
//	thread_ex_1 = rt_thread_create("ex1_thread",test_example_1_thread_entry,RT_NULL,1024,15,20);
//	if(sem_ex_1 == RT_NULL || thread_ex_1 == RT_NULL) return RT_ERROR;
//	rt_thread_startup(thread_ex_1);
//	
//	return RT_EOK;
//}

///* example 2 : stepper interpolation */
//static rt_sem_t sem_ex_2;				/* semaphore for test */
//static rt_thread_t thread_ex_2;			/* stepper thread */
//static lt_motor_t motor_ex_x;
//static lt_motor_t motor_ex_y;
///* when finish output, this function would be called */
//rt_err_t test_example_2_callback(void *parameter)
//{
//	return rt_sem_release(sem_ex_2);
//}

//static void test_example_2_thread_entry(void *parameter)
//{
//	rt_uint8_t i;
//	float output = 360, res;
//	for (i = 0; i < 5; i++)			/* 5 forward rotations */
//	{
//		lt_motor_control(motor_ex_x,STEPPER_CTRL_OUTPUT_ANGLE,&output);
//		lt_motor_control(motor_ex_x,STEPPER_CTRL_OUTPUT_ANGLE,&output);
//		rt_sem_take(sem_ex_2,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(500);									/* delay */
//	}
//	output = -360;
//	for (i = 0; i < 5; i++)			/* 5 reversal rotations */
//	{
//		lt_motor_control(motor_ex_x,STEPPER_CTRL_OUTPUT_ANGLE,&output);
//		lt_motor_control(motor_ex_y,STEPPER_CTRL_OUTPUT_ANGLE,&output);
//		rt_sem_take(sem_ex_2,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(500);									/* delay */
//	}
//	
//	/* line interpolation path: /\ -->  ---
//								\/     |   |
//										---	
//	*/
//	rt_int32_t line_path[8][2] = {{500,800},{500,-800},{-500,-800},{-500,800},
//								  {0,1000},{1000,0},{0,-1000},{-1000,0}};
//	rt_int32_t circular_path[9][2] = {{707,0},{500,500},{0,707},{-500,500},{-707,0},{-500,-500},{0,-707},{500,-500},{0,707}};
//	/* draw a circle with radius = 707 steps */
//	/* line interp test */							  
//	for(i = 0; i < 8; i++)
//	{
//		test_stepper_line_interp(motor_ex_x,motor_ex_y,0,0,line_path[i][0],line_path[i][1]);
//		rt_sem_take(sem_ex_2,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(1000);									
//	}
//	/* circular interp test */
//	/* in this case, the circular should lie in one quadrant and interp direction should be reasonable 
//	*  eg: start:(50, 50), end:(71,0), direction should be CW since intep circular lies in 1th quadrant
//	*/
//	for(i = 0; i < 8; i++)
//	{
//		test_stepper_circular_interp(motor_ex_x,motor_ex_y,circular_path[i][0],circular_path[i][1],circular_path[i+1][0],circular_path[i+1][1],707,DIR_CCW);
//		rt_sem_take(sem_ex_2,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(1000);									
//	}
//	/* clock-wise */
//	for(i = 8; i > 1; i++)
//	{
//		test_stepper_circular_interp(motor_ex_x,motor_ex_y,circular_path[i][0],circular_path[i][1],circular_path[i+1][0],circular_path[i+1][1],707,DIR_CW);
//		rt_sem_take(sem_ex_2,RT_WAITING_FOREVER);				/* waiting output finish */
//		rt_thread_mdelay(1000);									
//	}
//	
//}

//rt_err_t test_example_2_config(void)
//{
//	motor_ex_x = lt_manager_get_motor("x_stepper");
//	motor_ex_y = lt_manager_get_motor("y_stepper");
//	if(motor_ex_x == RT_NULL || motor_ex_y) return RT_ERROR;
//	lt_motor_set_callback(motor_ex_x,test_example_2_callback,RT_NULL);
//	
//	sem_ex_2 = rt_sem_create("ex2_sem",0,RT_IPC_FLAG_PRIO);
//	thread_ex_2 = rt_thread_create("ex2_thread",test_example_2_thread_entry,RT_NULL,1024,15,20);
//	if(sem_ex_2 == RT_NULL || thread_ex_2 == RT_NULL) return RT_ERROR;
//	rt_thread_startup(thread_ex_2);
//	
//	return RT_EOK;
//}

///* example 3 : one motor follows another */



#endif


