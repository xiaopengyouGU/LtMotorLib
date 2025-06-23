/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#ifndef __LT_DEF_H__
#define __LT_DEF_H__

#define SQRT_3						1.73205
#define GET_BIT(x,pos)		(1 & (x >> pos) )
#define CLEAR_BITS(x,bits)	(x & (~bits))
#define LT_NAME_MAX					RT_NAME_MAX + 2

#define LT_USING_MOTOR_MSH_TEST
//#define COMMUNICATOR_TYPE_OSCILLOSCOPE	

//#define TEST_PID_POS
//#define TEST_PID_CURR
#define TEST_PID_TIMER_TYPE			TIMER_TYPE_HW
//#define TEST_PID_CALLBACK			test_pid_simple
#define TEST_PID_CALLBACK			test_velocity_loop
//#define TEST_PID_CALLBACK			test_position_loop
//#define TEST_PID_CALLBACK			test_current_loop
//#define TEST_PID_CALLBACK			test_position_velocity_loop
//#define TEST_PID_CALLBACK			test_velocity_current_loop
//#define TEST_PID_CALLBACK			test_position_current_loop
//#define TEST_PID_CALLBACK			test_position_velocity_current_loop

#define	TEST_PID_COMMUT_TIMES		1		/* for high speed sample, sample 10 times then send data to the upper computer */

#define PI							3.1415926
#define PID_INTEGRAL_LIMIT			6000
#define PID_OUTPUT_LIMIT			2000
#define PID_POS_CONST				0.15f
#define PID_VEL_CONST				0.15f
#define PID_CURR_CONST				0.15f
		
#define COMMUNICATOR_DEV_NAME	"uart1"//"dac1"	

#define ENCODER_NAME_1			"pulse1"
#define ENCODER_NAME_2  		"pulse4"	
#define ENCODER_NAME_3			"pulse4"
#define ENCODER_NAME_4			"pulse4"
#define ENCODER_NAME_5			"pulse4"

#define I2C_NAME_1				"i2c1"
#define I2C_NAME_2				"i2c2"
#define I2C_NAME_3				"i2c2"
#define I2C_NAME_4				"i2c2"
#define I2C_NAME_5				"i2c2"

#define PWM_NAME_1				"pwm2"
#define PWM_NAME_2				"pwm3"
#define PWM_NAME_3				"pwm8"
#define PWM_NAME_4				"pwm8"
#define PWM_NAME_5				"pwm8"

#define TIMER_NAME_1			"timer5"
#define TIMER_NAME_2			"timer7"
#define TIMER_NAME_3			"timer11"
#define TIMER_NAME_4			"timer13"
#define TIMER_NAME_5			"timer14"

#define ADC_NAME_1				"adc1"
#define ADC_NAME_2				"adc1"
#define ADC_NAME_3				"adc1"
#define ADC_NAME_4				"adc1"
#define ADC_NAME_5				"adc1"

/* DC motor definition parameter */
#define MOTOR_OUTPUT_PERIOD			100		    /* 0.1ms, unit: us , 10kHz */
#define MOTOR_MAX_SPEED				300			/* DC motor maximum speed */
#define MOTOR_MIN_SPEED				0.01		/* DC motor minimum speed, dead region */
#define MOTOR_MAX_ANGLE				180			/* for steering engine */
#define	MOTOR_ANGLE_PERIOD 			20000  		/* 20ms, unit: us */
#define MOTOR_ANGLE_BASE			500		    /* 0.5ms, unit: us */

/* stepper motor definition parameter */
#define STEPPER_MAX_SPEED			300			/* stepper motor maximum speed, unit: Hz */
#define STEPPER_MIN_SPEED			1			/* stepper motor minimum speed, dead region */
#define STEPPER_MAX_PERIOD			1000000	    /* 1000ms, unit: us */
#define STEPPER_DUTY_CYCLE			0.5			/* default duty cycle */

/* bldc motor definition parameter */
#define BLDC_OPEN_SPEED				100			/* BLDC motor open output angle  speed */
//#define BLDC_PERIOD					10000	/* BLDC measure period, 10ms, unit: us */
#define BLDC_PERIOD					2000
#define BLDC_OUTPUT_PERIOD			100			/* 100ms, unit: us, 10kHz */
#define BLDC_CURRENT_LIMIT			3.3/2		/* max current, unit: A */
	




#endif
