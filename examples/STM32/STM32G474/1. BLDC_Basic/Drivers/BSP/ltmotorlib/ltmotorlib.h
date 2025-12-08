/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        The first version
 * 2025-12-8	  Lvtou		   The second version
 */
#ifndef __LT_MOTOR_LIB_H__
#define __LT_MOTOR_LIB_H__

#include "lt_def.h"
#include "lt_debug.h"
#include "ltm_driver.h"
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/*********************************************************************************************************/
/* object part*/
void* lt_malloc(size_t);
void lt_free(void *);
/*********************************************************************************************************/
/* trape object */
#define TRAPZOID_MODE_ADAPTIVE		0x02
#define TRAPZOID_MODE_CENTER		0x01
#define TRAPZOID_MODE_DEFAULT		0x00
struct lt_trape_object
{
	float mag;				/* magnitude */	
	float dutyA;				
	float dutyB;
	float dutyC;
	uint8_t mode;			/* voltage modulation mode */
};
typedef struct lt_trape_object* lt_trape_t;

lt_trape_t lt_trape_create(float mag,uint8_t mode);
void lt_trape_set(lt_trape_t trape, float mag, uint8_t mode);
uint8_t* lt_trape_process(lt_trape_t trape,uint8_t hall_signal, float input, float* duty);
void lt_trape_process2(lt_trape_t trape,float angle_el, float input);
void lt_trape_get(lt_trape_t trape,float* dutyA, float* dutyB,float* dutyC);
void lt_trape_delete(lt_trape_t trape);
/*********************************************************************************************************/
/* foc object */
#define FOC_TYPE_DEFAULT		0x00
#define FOC_TYPE_SPWM			0x00
#define FOC_TYPE_SPWM_1			0x01
#define FOC_TYPE_SPWM_2			0x02
#define FOC_TYPE_SVPWM			0x03

#define FOC_MODE_ADAPTIVE		0x02
#define FOC_MODE_CENTER			0x01
#define FOC_MODE_DEFAULT		0x00

struct lt_foc_object
{
	float mag;				/* magnitude */		
	float dutyA;				
	float dutyB;
	float dutyC;
	uint8_t mode; 			/* voltage modulation mode */
	uint8_t type;			/* foc type  */
};
typedef struct lt_foc_object* lt_foc_t;

lt_foc_t lt_foc_create(float mag, uint8_t mode, uint8_t type);
void lt_foc_set(lt_foc_t foc, float mag, uint8_t mode, uint8_t type);
void lt_foc_process(lt_foc_t foc, float fd, float fq, float angle_el);
void lt_foc_get(lt_foc_t foc,float* dutyA, float* dutyB,float* dutyC);
void lt_foc_delete(lt_foc_t foc);
/*********************************************************************************************************/
/* define a pid object */
struct lt_pid_object
{
	float target;		
	float output;
	float err;			/* used for full pid */
	float err_prev;		/* used for increment pid */
	float err_prev2;	/* used for increment pid */
	float Kp;
	float Ki_ts;		/* Ki_ts = Ki * ts, ts is sample time, unit : s */
	float Kd_ts;		/* Kd_ts = Kd / ts, ts is sample time, unit : s */
	float integral;
	float int_limit;	/* integral limit */
	float output_limit;	/* output limit */	
	float ts;			/* sample time, unit : s */
};
typedef struct lt_pid_object* lt_pid_t;

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float ts_ms);
void lt_pid_delete(lt_pid_t pid);

void lt_pid_reset(lt_pid_t pid);
void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd);
void lt_pid_set_target(lt_pid_t pid, float target);
void lt_pid_set_ts(lt_pid_t pid, float ts_ms);
void lt_pid_set_limits(lt_pid_t pid,float int_limit, float output_limit);
float lt_pid_get(lt_pid_t pid);
float lt_pid_process(lt_pid_t pid,float curr_val);
float lt_pid_process2(lt_pid_t pid, float curr_val);		/* increment pid  */
/*******************************************************************************/
/* current sense object */
#define CURRENT_TYPE_2 0x00
#define CURRENT_TYPE_3 0x11

typedef struct lt_current_sense * lt_current_t;
struct lt_current_sense
{	
	char name[LT_NAME_MAX];
	uint16_t vals[5];		/* adc read values : a, b, c, Vbus, temp */
	float val_I_gain;		/* adc val to current coefficiency */
	float val_bus_gain;		/* adc val to vbus coefficiency */
	float bias[3];			/* channel bias : a, b, c */
	/* async values, used for asynchronous read */
	float I_bus_async;
	float Id_async;
	float Iq_async;
	
	float (*get_temp)(uint16_t);/* get temperature */
	uint8_t type;
	uint8_t flag;				/* calib flag */
	
	const struct lt_current_ops* ops;	/* operators */
	void* user_data;
};

struct lt_current_config
{
	const char* name;
	uint8_t bit_num;
	float resistor;
	float amp_gain;
	float vbus_gain;
	float (*get_temp)(uint16_t);
	struct lt_current_ops* ops;
	uint8_t type;
};

struct lt_current_ops
{
	void (*calibrate)(lt_current_t current);
	void (*start)(lt_current_t current);
	void (*stop)(lt_current_t current);
};

lt_current_t lt_current_create(const char* dev_name);
void lt_current_set(lt_current_t current, struct lt_current_config * config);
void lt_current_init(lt_current_t current);
void lt_current_start(lt_current_t current);
void lt_current_stop(lt_current_t current);
void lt_current_get(lt_current_t current, float*Ia, float*Ib, float*Ic);
void lt_current_get_vt(lt_current_t current, float*volt, float*temp);
void lt_current_get_ab(lt_current_t current,float*I_alpha, float*I_beta);
void lt_current_get_dq(lt_current_t current,float angle_el, float*Id, float*Iq);
void lt_current_get_bus(lt_current_t current, uint8_t* pulses,float input,float*I_bus);
/* asynchronous read functions */
void lt_current_get_bus_async(lt_current_t current, float*I_bus);
void lt_current_get_dq_async(lt_current_t current, float*Id, float*Iq);
uint32_t lt_current_calibrate(lt_current_t current);
void lt_current_delete(lt_current_t current);
/*******************************************************************************/
/* PWM driver object */
typedef struct lt_driver_object * lt_driver_t;
struct lt_driver_object
{	
	char name[LT_NAME_MAX];
	uint8_t flag;
	float band;
	const struct lt_driver_ops* ops;	/* operators */
	void* user_data;
};

struct lt_driver_config
{
	const char* name;
	float band;
	struct lt_driver_ops* ops;
};

struct lt_driver_ops
{
	void (*start)(lt_driver_t driver);
	void (*output)(lt_driver_t driver, float dutyA,float dutyB,float dutyC);
	void (*output2)(lt_driver_t driver, uint8_t* pulses, float duty);
	void (*stop)(lt_driver_t driver);
};

lt_driver_t lt_driver_create(const char* dev_name);
void lt_driver_set(lt_driver_t driver, struct lt_driver_config * config);
void lt_driver_start(lt_driver_t driver);
void lt_driver_stop(lt_driver_t driver);
void lt_driver_output(lt_driver_t driver, float dutyA,float dutyB,float dutyC);
void lt_driver_output2(lt_driver_t driver, uint8_t* pulses, float duty);
void lt_driver_delete(lt_driver_t driver);
/*******************************************************************************/
/* position sensor object */
#define SENSOR_TYPE_HALL 	0x01
#define SENSOR_TYPE_MAGNET 	0x02
#define SENSOR_TYPE_ENCODER	0x03

typedef struct lt_sensor_object * lt_sensor_t;
struct lt_sensor_object
{	
	char name[LT_NAME_MAX];
	float val_pos_gain;					/* values to position gain */
	float ts;							/* sample period, unit : s */
	float val_vel_gain;					/* values to velocity gain */
	/* async values */
	float pos_async;
	float vel_async;
	
	uint8_t flag;					
	uint8_t type;
	const struct lt_sensor_ops* ops;	/* operators */
	void* user_data;
};

struct lt_sensor_config
{
	const char* name;
	uint8_t bit_num;
	float revolutions;	
	float band;
	float ts;
	uint8_t type;
	struct lt_sensor_ops* ops;
};

struct lt_sensor_ops
{
	void (*start)(lt_sensor_t sensor);
	void (*calibrate)(lt_sensor_t sensor);
	void (*get)(lt_sensor_t sensor, void *value);
	void (*stop)(lt_sensor_t sensor);
};

lt_sensor_t lt_sensor_create(const char* dev_name);
void lt_sensor_set(lt_sensor_t sensor, struct lt_sensor_config * config);
void lt_sensor_start(lt_sensor_t sensor);
void lt_sensor_stop(lt_sensor_t sensor);
void lt_sensor_get(lt_sensor_t sensor, float* pos,float* vel);
void lt_sensor_get2(lt_sensor_t sensor, uint8_t* hall_signal);
/* asynchronous functions */
void lt_sensor_get_async(lt_sensor_t sensor,float* pos,float* vel);
uint32_t lt_sensor_calibrate(lt_sensor_t sensor);
void lt_sensor_delete(lt_sensor_t sensor);
/*******************************************************************************/
/*********************************************************************************************************/
/* motor object */
#define MOTOR_TYPE_DC	0x01
#define MOTOR_TYPE_BLDC 0x02
#define MOTOR_TYPE_PMSM	0x03
typedef struct lt_motor_object* lt_motor_t;

struct lt_motor_info
{
	/* target values */
	float target_pos;
	float target_vel;
	float target_curr;
	/* actual values */
	uint8_t hall_signal;		
	float pos;				/* unit : degree */
	float vel;				/* unit : rpm */
	float Id;				/* unit : A */
	float Iq;				/* unit : A */
	float I_bus;			/* unit : A */
	float temp;				/* temperature, unit: degree celsius */
	float vbus;				/* DC bus votage */
	float bemf[3];			/* Back EMF, A,B,C phase, unit : V */
	float te;				/* torque : N.m */
	uint8_t flag;			/* motor states flag */
};

struct lt_motor_param
{
	float Ls;				/* inductance, unit : mH */
	float R;				/* resitance, unit : ohm */
	float pn;				/* pole pairs */
	float Kt;				/* torque sensitive, unit : N.m/A */
	float Ke;				/* back emf constant, unit : V/krpm */
	float Vdc;				/* unit : votage */
	float n_rated;			/* rated speed, unit : rpm */
	float curr_rated;		/* rated current, unit : A */
	float KV;				/* KV value for open loop output, unit : rpm/V */
};

struct lt_motor_object
{
	char name[LT_NAME_MAX];
	/* device part */
	lt_sensor_t sensor;
	lt_driver_t driver;
	lt_current_t current;
	/* pid part */
	lt_pid_t pos_pid;
	lt_pid_t vel_pid;
	lt_pid_t curr_pid_d;
	lt_pid_t curr_pid_q;
	/* control algorithm */
	void* ctrl;						/* Six Step or FOC */ 
	void* observer;					/* Observer */
	/* motor parameters and infomation */
	struct lt_motor_param param;	/* motor physical parameters */
	struct lt_motor_info info;		/* motor information for monitoring */
	float V_target;					/* target voltage : only for open loop control !!! */
	
	uint8_t flag;					/* motor flag */
	uint8_t type;					/* motor type */
};

typedef struct lt_motor_info* lt_info_t;
struct lt_motor_config
{
	const char* name;
	lt_sensor_t sensor;
	lt_driver_t driver;
	lt_current_t current;
	lt_pid_t pos_pid;
	lt_pid_t vel_pid;
	lt_pid_t curr_pid_d;
	lt_pid_t curr_pid_q;
	
	void* ctrl;				/* Six Step or FOC */ 
	void* observer;			/* Observer */
	struct lt_motor_param param;
	
	uint8_t type;			/* motor type */
};

lt_motor_t lt_motor_create(const char* name);
void lt_motor_start(lt_motor_t motor);
void lt_motor_stop(lt_motor_t motor);
uint32_t lt_motor_check(lt_motor_t motor);
void lt_motor_delete(lt_motor_t motor);
void lt_motor_set(lt_motor_t motor, struct lt_motor_config* config);
void lt_motor_set_pos(lt_motor_t motor, float target_pos);
void lt_motor_set_vel(lt_motor_t motor, float target_vel);
void lt_motor_set_curr(lt_motor_t motor, float target_curr);
float lt_motor_get_vtar(lt_motor_t motor);		/* get V_target for open output */
lt_info_t lt_motor_get(lt_motor_t);
void lt_motor_monitor(lt_motor_t motor);		/* monitor key parameters */
/*******************************************************************************/
#ifdef LT_COMMUTE_ENABLE
/* communicator object */
/* this object is responsible for all communications, so it may be a little fat */  
/* */
#define  CMD_SEND_CURVES 				0x01
#define  CMD_SEND_TARGET				0x02
#define  CMD_SEND_PERIOD				0x03
#define  CMD_SEND_START					0x04
#define  CMD_SEND_STOP					0x05
#define  CMD_SEND_PID					0x06
/* used pid types */
#define  PID_TYPE_VEL					0x01
#define  PID_TYPE_POS					0x02
#define  PID_TYPE_CURR_D				0x03
#define  PID_TYPE_CURR_Q				0x04

struct lt_commut_object
{
	char name[LT_NAME_MAX];
	lt_motor_t motor;
	lt_pid_t pid;	
	int curves[5];						/* curves data */
	
	const struct lt_commut_ops *ops;	
	uint8_t flag;
};
typedef struct lt_commut_object* lt_commut_t;

struct lt_commut_ops
{
	void (*process)(lt_commut_t commut);
	void (*send)(lt_commut_t commut, uint8_t type);
};

struct lt_commut_config
{
	const char* name;
	struct lt_commut_ops *ops;
};
void lt_commut_print(const char *fmt, ...);
void lt_commut_set(struct lt_commut_config* config);
void lt_commut_process(void);
void lt_commut_set_motor(lt_motor_t, uint8_t pid_type);
void lt_commut_set_curve(uint8_t ch, int value);
void lt_commut_send(uint8_t cmd_type);
#endif
/*******************************************************************************/
/* ltmotorlib math function, including lookup table trigonometric functions and matrix operations */
float lt_sin(float the);
float lt_cos(float the);
float lt_atan2(float y, float x);
float lt_normalize(float angle_el);
/*******************************************************************************/
#endif
