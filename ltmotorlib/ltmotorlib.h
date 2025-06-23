/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#ifndef __LT_MOTOR_LIB_H__
#define __LT_MOTOR_LIB_H__
#include <rtthread.h>
#include <rtdevice.h>
#include "lt_def.h"
#include "protocol.h"
rt_align(RT_ALIGN_SIZE)
/* declare special motor operators */
extern char* _status[4];
extern char* _type[4];
extern struct lt_motor_ops _motor_dc_ops;
extern struct lt_motor_ops _motor_bldc_ops;
extern struct lt_motor_ops _motor_stepper_ops;

/************************* common functions ************************************/
float _constrains(float val, float up_limit, float down_limit);
float _constrains_dead_region(float val,float up_limit, float down_limit);
rt_uint8_t _get_rotation_dir(float input);			
rt_uint8_t _get_quard(int x_pos, int y_pos, rt_uint8_t dir);
/* check whether start pos and end pos in the same quarent and circle */
rt_uint8_t _check_circular_pos(int x_start, int y_start, int x_end, int y_end);
rt_uint8_t _get_center(int x_start, int y_start, int x_end, int y_end,rt_uint16_t r, rt_uint8_t dir, float*x_center, float* y_center);
rt_uint8_t _check_end(int x_pos, int y_pos, int x_target, int y_target,rt_uint8_t exact);
rt_int32_t _get_count(float curr, float last,rt_int32_t count, rt_uint8_t volt);
int _abs(int i);
int _abs_plus_1(int i,rt_uint8_t dir);
int _abs_sub_1(int i);
float _absf(float i);
float _max(float a, float b);
float _min(float a, float b);
float _normalize_angle(float angle_el);
void _clark_trans(float fa, float fb, float fc, float*f_alpha, float*f_beta);
void _park_trans(float f_alpha, float f_beta, float angle, float*fd, float*fq);
void _clark_inv_trans(float f_alpha, float f_beta, float*fa, float*fb, float*fc);
void _park_inv_trans(float fd, float fq, float angle, float*f_alpha, float*f_beta);
/************************* common functions ************************************/
/* foc object */
#define FOC_TYPE_DEFAULT		0x00
#define FOC_TYPE_SPWM			0x00
#define FOC_TYPE_SVPWM			0x01
#define FOC_TYPE_6_TRAPZOID		0x02

struct lt_foc_object
{
	float fa;				/* A phase */
	float fb;				/* B phase */
	float fc;				/* C phase */
	float max_val;			
	rt_uint8_t type;		/* foc type */
};
typedef struct lt_foc_object* lt_foc_t;

lt_foc_t lt_foc_create(void);
void lt_foc_set_maxval(lt_foc_t foc,float max_val);
void lt_foc_set_type(lt_foc_t foc,rt_uint8_t type);
void lt_foc_process(lt_foc_t foc, float fd, float fq, float angle_el);
void lt_foc_map_duty(lt_foc_t foc,float* dutyA, float* dutyB,float* dutyC);
rt_err_t lt_foc_delete(lt_foc_t foc);

/*******************************************************************************/
struct lt_filter_object			/* low pass filter object */
{
	float Tf;
	float dt;
	float val_prev;				/* last filter value */
};
typedef struct lt_filter_object * lt_filter_t;

lt_filter_t lt_filter_create(float Tf, float dt);
void lt_filter_set_tf(lt_filter_t, float Tf);
void lt_filter_set_dt(lt_filter_t, float dt);
void lt_filter_set(lt_filter_t, float Tf, float dt);
void lt_filter_reset(lt_filter_t);
float lt_filter_process(lt_filter_t,float value);
rt_err_t lt_filter_delete(lt_filter_t);

/*******************************************************************************/
/* curve generator */
#define CURVE_TYPE_TRAPZOID		0x01
#define CURVE_TYPE_S_CURVE		0x02
#define CURVE_TYPE_5_SECTION	0x03
struct lt_curve_config
{
	float initial;
	float target;
	float flexible;
	int step;
	float acc;
	float dec;
	rt_uint8_t type;
};
/* this curve generator provides a velocity table whose unit is us coresponding to Hz 
* user should release memory by hand! */
struct lt_curve_generator_object
{				
	rt_uint16_t step;
	rt_uint16_t acc_step;
	rt_uint16_t dec_start;
	rt_uint16_t curr_step;			/* current step */
	rt_uint8_t flag;				/* 0: part velocity table, 1: full velocity table */
	float* T_nums;                     
};
typedef struct lt_curve_generator_object* lt_curve_t;

lt_curve_t lt_curve_create(void);
rt_err_t lt_curve_set(lt_curve_t curve,struct lt_curve_config* config);
rt_uint32_t lt_curve_process(lt_curve_t curve);
rt_err_t lt_curve_release(lt_curve_t curve);
rt_err_t lt_curve_restart(lt_curve_t curve);				/* use same velocity table */
rt_err_t lt_curve_delete(lt_curve_t curve);

/*****************************************************************************************************/
/* interp object */
#define INTERP_TYPE_LINE		0x01
#define INTERP_TYPE_CURCULAR	0x02

#define DIR_CW					0x00
#define DIR_CCW					0x01
#define INTERP_STOP				0x00
#define X_MOTOR_MOVE			0x01
#define Y_MOTOR_MOVE			0x02

struct lt_interp_object
{
	int x_pos;					/* current pos */
	int y_pos;
	int x_end;					/* target position */
	int y_end;
	int deviation;				/* position deviation */
	rt_uint8_t dir;				/* interpolation direction, 0: clockwise, 1: counter-clockwise */
	rt_uint16_t num_pulse;		/* total pulse number */
	rt_uint8_t quadrant;		/* quadrant */
	rt_uint16_t radius;			/* 0: line interp, others: circular interp */
	rt_uint8_t exact;			/* in circular interp, 0: can't reach exact pos, 1: can reach exact pos */
};
typedef struct lt_interp_object* lt_interp_t;
/* this curve generator provides a velocity table whose unit is us coresponding to Hz 
*  user should release memeory by hand 
*/

struct lt_interp_config
{
	int x_start;
	int y_start;
	int x_end;
	int y_end;
	rt_uint16_t radius;			/* 0: line interp, others: circular interp */
	rt_uint8_t dir;				/* interpolation direction, 0: clockwise, 1: counter-clockwise */
	//void* x_motor;			
	void* y_motor;
};

lt_interp_t lt_interp_create(void);
rt_err_t lt_interp_set(lt_interp_t interp,struct lt_interp_config* config);
rt_uint8_t lt_interp_process(lt_interp_t interp,rt_uint8_t*dir);
rt_err_t lt_interp_delete(lt_interp_t curve);
/*********************************************************************************************************/
/* define pid object */
#define PID_TYPE_VEL		0x01
#define PID_TYPE_POS		0x02
#define PID_TYPE_CURRENT	0x03

struct lt_pid_object
{
	float target_val;		
	float control_u;
	float err;
	float err_prev;
	float err_last;
	float Kp,Ki,Kd;
	float integral;
	float dt;			/* sample time,unit: ms */
	float int_limit;	/* integral limit */
	float output_limit;	/* output limit */	
};
typedef struct lt_pid_object* lt_pid_t;
/* pid info for communicating with upper computer */
struct lt_pid_info 
{
	float Kp;
	float Ki;
	float Kd;
	float target;		/* target */
	float dt;			/* sample time, unit: ms */
};

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float dt);
rt_err_t lt_pid_delete(lt_pid_t pid);

void lt_pid_reset(lt_pid_t pid);
void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd);
void lt_pid_set_target(lt_pid_t pid, float target);
void lt_pid_set_dt(lt_pid_t pid, float dt);
void lt_pid_set_output_limit(lt_pid_t pid,float limit);
void lt_pid_set_int_limit(lt_pid_t pid,float limit);		/* set integral limit */

float lt_pid_get_control(lt_pid_t pid);
float lt_pid_control(lt_pid_t pid,float curr_val);
float lt_pid_incre_control(lt_pid_t pid, float curr_val);

/*******************************************************************************/
/* driver part */
#define DRIVER_TYPE_UNKNOWN		0x00
#define DRIVER_TYPE_DC			0x01
#define DRIVER_TYPE_BLDC		0x02
#define DRIVER_TYPE_STEPPER		0x03
/* rotation direction */
#define ROT_FORWARD				0x01
#define ROT_REVERSAL			0x02
#define ROT_DEFAULT				0x00


#define PWM_PHASE_DEFAULT	0x00
#define PWM_PHASE_A	  		0x01
#define PWM_PHASE_B	  		0x02
#define PWM_PHASE_C	  		0x03

struct lt_driver_object
{
	/* three phase PWM A,B,C */
	struct rt_device_pwm* pwm_A;
	struct rt_device_pwm* pwm_B;
	struct rt_device_pwm* pwm_C;
	rt_uint8_t pwm_channel_A;
	rt_uint8_t pwm_channel_B;
	rt_uint8_t pwm_channel_C;
	/* control pins */
	rt_base_t forward_pin;
	rt_base_t reversal_pin;
	rt_base_t enable_pin;
	/* three phase have same period and pulse */
	
	rt_uint8_t type;
	const struct lt_driver_ops *ops;		/* driver control operators */
	void *user_data;
};
typedef struct lt_driver_object* lt_driver_t;

struct lt_driver_ops
{
	rt_err_t (*set_pins)(lt_driver_t driver);
	rt_err_t (*enable)(lt_driver_t driver,rt_uint8_t dir);				
	rt_err_t (*disable)(lt_driver_t driver);
};

lt_driver_t lt_driver_create(rt_uint8_t type);
rt_err_t lt_driver_set_pins(lt_driver_t driver,rt_base_t forward_pin,rt_base_t reversal_pin,rt_base_t enable_pin);
rt_err_t lt_driver_set_pwm(lt_driver_t driver,char* pwm_name,rt_uint8_t pwm_channel,rt_uint8_t phase);
rt_err_t lt_driver_set_output(lt_driver_t driver,rt_uint32_t period,float duty_cycle);
rt_err_t lt_driver_3pwm_output(lt_driver_t driver,rt_uint32_t period,float dutyA,float dutyB, float dutyC);
rt_err_t lt_driver_enable(lt_driver_t driver,rt_uint8_t dir);
rt_err_t lt_driver_disable(lt_driver_t driver);
rt_err_t lt_driver_delete(lt_driver_t driver);

/*********************************************************/
/* sensor part */
#define SENSOR_TYPE_ENCODER			0x01
#define SENSOR_TYPE_MAGNETIC		0x02
#define SENSOR_TYPE_HALL			0x03
#define SENSOR_TYPE_UNKNOWN			0x00

struct lt_sensor_object
{
	rt_device_t dev;					
	rt_uint8_t type;
	rt_uint16_t resolution;
	int count;								/* current value  */
	float last;								/* last position */
	/* pins */
	
	lt_filter_t lpf;						/* low pass filter */
	const struct lt_sensor_ops *ops;		/* sensor control operators */
	void *user_data;
};
typedef struct lt_sensor_object* lt_sensor_t;

struct lt_sensor_ops
{
	lt_sensor_t (*create)(char* sensor_name, rt_uint16_t resolution, rt_uint8_t type);
	float (*get_angle)(lt_sensor_t sensor);
	float (*get_velocity)(lt_sensor_t sensor,rt_uint32_t measure_time_us );
	rt_err_t (*calibrate)(lt_sensor_t sensor);
};

lt_sensor_t lt_sensor_create(char* sensor_name, rt_uint16_t resolution, rt_uint8_t type);
float lt_sensor_get_angle(lt_sensor_t sensor);
float lt_sensor_get_velocity(lt_sensor_t sensor, rt_uint32_t measure_time_us);
rt_err_t lt_sensor_calibrate(lt_sensor_t sensor);
rt_err_t lt_sensor_delete(lt_sensor_t sensor);


/*******************************************************************************/
/* timer part */
#define TIMER_TYPE_HW				0x00
#define TIMER_TYPE_SOFT				0x01
#define TIMER_MODE_SINGLE_SHOT		0x01
#define TIMER_MODE_PERIODIC			0x02
typedef struct lt_timer_object * lt_timer_t;
struct lt_timer_object
{
	char hw_name[RT_NAME_MAX];		/* find timer from name */
	rt_device_t hw_timer;			/* hardware timer, high precision */
	rt_timer_t soft_timer;			/* software timer, no number limit */
	rt_uint32_t hw_period;			/* hardware timer period, unit: us */
	rt_hwtimer_mode_t hw_mode;		/* hardware timer mode */
	
	void(*soft_timeout)(lt_timer_t);/* hardware timer timeout function */
	void(*hw_timeout)(lt_timer_t);	/* software timer timeout function */	
	void* user_data;				/* user_data for data transport */
};

lt_timer_t lt_timer_create(char* soft_name,char* hw_name,rt_uint32_t freq);
rt_err_t lt_timer_set(lt_timer_t timer,rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type);
rt_err_t lt_timer_set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type);
rt_err_t lt_timer_period_call(lt_timer_t timer,rt_uint32_t period, void(*timeout)(lt_timer_t),void* user_data,rt_uint8_t type);
rt_err_t lt_timer_enable(lt_timer_t timer,rt_uint8_t type);
rt_err_t lt_timer_disable(lt_timer_t timer,rt_uint8_t type);
rt_err_t lt_timer_delete(lt_timer_t timer);

/*******************************************************************************/
/* current sense object */
struct lt_current_sense_object
{
	rt_adc_device_t adc;
	rt_int8_t channel_A;
	rt_int8_t channel_B;
	rt_int8_t channel_C;
	
	float resistor;
	float amp_gain;
	rt_uint8_t bit_num;
	float bias_a;			/* zero bias */
	float bias_b;
	float bias_c;
	lt_filter_t lpf_d;		/* low pass filter for d axis current */
	lt_filter_t lpf_q;      /* low pass filter for q axis current */
	
	
	void* user_data;
};
typedef struct lt_current_sense_object * lt_current_t;

struct lt_current_info
{
	float Ia;				/* three phase currents */
	float Ib;
	float Ic;
	float I_alpha;			/* static currents */
	float I_beta;			
	float Id;				/* synchronous currents */
	float Iq;
};


lt_current_t lt_current_create(float resistor, float amp_gain, rt_uint8_t bit_num);
rt_err_t lt_current_set_adc(lt_current_t, char* adc_name,rt_int8_t channel_A, rt_int8_t channel_B, rt_int8_t channel_C);
rt_err_t lt_current_calibrate(lt_current_t current);
rt_err_t lt_current_get(lt_current_t current, float*Ia, float*Ib, float*Ic);
rt_err_t lt_current_get_ab(lt_current_t current, float angel, float*I_alpha, float*I_beta);
rt_err_t lt_current_get_dq(lt_current_t current, float angle, float dt, float*Id, float*Iq);
rt_err_t lt_current_get_info(lt_current_t current, float angle, struct lt_current_info* info);
rt_err_t lt_current_delete(lt_current_t current);

/*******************************************************************************/
/* motor part!!! */
#define MOTOR_TYPE_DC		0x01
#define MOTOR_TYPE_BLDC		0x02
#define MOTOR_TYPE_STEPPER	0x03
#define MOTOR_TYPE_UNKNOWN	0x00

/* motor status */
#define MOTOR_STATUS_STOP				0x00
#define MOTOR_STATUS_RUN				0x01
#define MOTOR_STATUS_ACCELERATE			0x02
#define MOTOR_STATUS_INTERP				0x04

/* motor control command */
#define MOTOR_CTRL_OUTPUT				0x01
#define MOTOR_CTRL_OUTPUT_ANGLE			0x02
#define MOTOR_CTRL_GET_STATUS			0x03
#define MOTOR_CTRL_GET_VELOCITY			0x04
#define MOTOR_CTRL_GET_POSITION			0x05
#define MOTOR_CTRL_OUTPUT_PID			0x06
#define MOTOR_CTRL_OUTPUT_ANGLE_PID		0x07
#define MOTOR_CTRL_DISABLE_PID			0x08

struct lt_motor_ops;

/* define motor struct */
struct lt_motor_object{
	
	char  name[LT_NAME_MAX]; 
	rt_uint8_t reduction_ratio;
	lt_driver_t driver;					/* motor driver */
	lt_sensor_t sensor;					/* position sensor */
	lt_timer_t timer;					/* timer */
	lt_pid_t pid_vel;					/* pid object for simple closed loop output */
	lt_pid_t pid_pos;
	lt_pid_t pid_current;				/* current pid object */
	
	rt_uint8_t type;
	rt_uint8_t pid_type;				/* used pid type: pos/vel pid */
	rt_uint8_t status;					/* motor status */
	
	const struct lt_motor_ops *ops;		/* motor control operators */
	rt_err_t (*callback)(void*);		/* done callback function, used for accel and interp */
	void* call_param;					/* callback function param */
	
	void* user_data;
};
typedef struct lt_motor_object* lt_motor_t;

struct lt_motor_info
{
	char name[LT_NAME_MAX];
	rt_uint8_t type;
	rt_uint8_t status;
	float position;
	//float velocity;
};
/* motor control operator */
struct lt_motor_ops
{	
	lt_motor_t(*create)(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
	rt_err_t (*control)(lt_motor_t motor, int cmd,void* arg);
	rt_err_t(*_delete)(lt_motor_t motor);
};

lt_motor_t lt_motor_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
rt_err_t lt_motor_set_driver(lt_motor_t motor, lt_driver_t driver);
rt_err_t lt_motor_set_sensor(lt_motor_t motor, lt_sensor_t sensor);
rt_err_t lt_motor_set_callback(lt_motor_t, rt_err_t (*callback)(void*),void*call_param);
rt_err_t lt_motor_set_pid(lt_motor_t,lt_pid_t pid,rt_uint8_t pid_type);
rt_err_t lt_motor_set_timer(lt_motor_t motor, lt_timer_t timer);

float lt_motor_get_velocity(lt_motor_t, rt_uint32_t measure_time_ms);
float lt_motor_get_position(lt_motor_t);
rt_err_t lt_motor_get_info(lt_motor_t, struct lt_motor_info*);
rt_err_t lt_motor_control(lt_motor_t, int cmd, void* arg);
rt_err_t lt_motor_enable(lt_motor_t,rt_uint8_t dir);
rt_err_t lt_motor_disable(lt_motor_t);
rt_err_t lt_motor_delete(lt_motor_t);					/* delete a motor! */

/*******************************************************************************/
/* stepper motor part */

#define STEPPER_CTRL_OUTPUT					MOTOR_CTRL_OUTPUT
#define STEPPER_CTRL_OUTPUT_ANGLE			MOTOR_CTRL_OUTPUT_ANGLE
#define STEPPER_CTRL_GET_STATUS				MOTOR_CTRL_GET_STATUS
#define STEPPER_CTRL_GET_VELOCITY			MOTOR_CTRL_GET_VELOCITY
#define STEPPER_CTRL_GET_POSITION			MOTOR_CTRL_GET_POSITION
#define STEPPER_CTRL_OUTPUT_PID				MOTOR_CTRL_OUTPUT_PID
#define STEPPER_CTRL_OUTPUT_ANGLE_PID		MOTOR_CTRL_OUTPUT_ANGLE_PID
/* basic control commanda are the same */
#define STEPPER_CTRL_CONFIG					(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x10)
#define STEPPER_CTRL_ACCELERATE				(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x11)
#define STEPPER_CTRL_INTERPOLATION			(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x12)
/* stepper flag */
#define FLAG_STEPPER_CONFIG					0x01
#define FLAG_STEPPER_TRAPZOID				0x02
#define FLAG_STEPPER_S_CURVE				0x04
#define FLAG_STEPPER_5_SECTION				0x08


struct lt_motor_stepper_object
{
	struct lt_motor_object parent;
	/* config structures */
	rt_uint16_t period;			/* pulse period  ms */
	rt_uint16_t subdivide;		/* subdivide number */
	float angle;				/* stepper angle, unit: degree */
	lt_curve_t	curve;			/* accleration curve info */
	lt_interp_t interp;			/* interp object */

	rt_uint8_t flag;		/* config flag */
	
};
typedef struct lt_motor_stepper_object * lt_stepper_t;

struct lt_stepper_config
{
	rt_uint16_t period;			/* pulse period  ms */
	float stepper_angle;		/* unit: degree */
	rt_uint16_t subdivide;		/* subdivide number */
};
/*****************************************************************************************************/
/* bldc motor part */
#define BLDC_CTRL_OUTPUT					MOTOR_CTRL_OUTPUT
#define BLDC_CTRL_OUTPUT_ANGLE				MOTOR_CTRL_OUTPUT_ANGLE
#define BLDC_CTRL_GET_STATUS				MOTOR_CTRL_GET_STATUS
#define BLDC_CTRL_GET_VELOCITY				MOTOR_CTRL_GET_VELOCITY
#define BLDC_CTRL_GET_POSITION				MOTOR_CTRL_GET_POSITION
#define BLDC_CTRL_OUTPUT_PID				MOTOR_CTRL_OUTPUT_PID
#define BLDC_CTRL_OUTPUT_ANGLE_PID			MOTOR_CTRL_OUTPUT_ANGLE_PID
/* basic control commanda are the same */
#define BLDC_CTRL_CONFIG					(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x20)
#define BLDC_CTRL_OUTPUT_TORQUE				(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x21)	
#define BLDC_CTRL_GET_ELECTRIC_INFO			(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x22)
#define BLDC_CTRL_GET_MECHANICAL_INFO		(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x24)
#define BLDC_CTRL_GET_MOTOR_INFO			(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x25)
#define BLDC_CTRL_OUTPUT_NO_TIMER			(MOTOR_CTRL_OUTPUT_ANGLE_PID + 0x26)
/* bldc flag */
#define FLAG_BLDC_CONFIG					0x01
#define FLAG_BLDC_OPEN_POS					0x02	/* used for open loop output */					
#define FLAG_BLDC_OPEN_POS_BIAS				0x04	/* bit2: 1: curr angle > target, 0: curr angle < target */
#define FLAG_BLDC_TORQUE					0x08	/* constant torque output */
#define FLAG_BLDC_GET_INFO					0x10	/* already read current pos, no need to read again */

struct lt_motor_bldc_object
{
	struct lt_motor_object parent;

	rt_uint16_t poles;			/* pole pairs number */
	float resistance;			/* resistance, unit: ohm */
	float inductance;			/* inductance, unit: H */
	float KV;					/* KV value, unit: rpm/V */
	float max_volt;				/* max output voltage */
	lt_foc_t foc;				/* foc object */
	lt_current_t current;		/* current sense */
	
	float target_pos;			/* target position for open loop output */
	float target_vel;			/* target speed for open loop output */
	rt_uint8_t flag;			/* config flag */
	rt_uint8_t no_timer;		/* 1: user is responsible for controlling bldc open output, 0: converse */
	
};
typedef struct lt_motor_bldc_object * lt_bldc_t;

struct lt_bldc_config
{
	rt_uint16_t poles;			/* pole pairs number */
	float resistance;			/* resistance, unit: ohm */
	float inductance;			/* inductance, unit: H */
	float KV;					/* KV value, unit: rpm/V */
	float max_volt;				/* max output voltage */
	rt_uint8_t foc_type;		/* used foc type */
	lt_current_t current;		/* current sense */
};

struct lt_bldc_info
{
	float angle_el;
	float Ia;
	float Ib;
	float Ic;
	float Iq;
	float Id;
	float pos;
	float vel;
	float dt;
};

/*****************************************************************************************************/
/* motor mananger */
#define MANAGER_CTRL_SHOW_MOTOR			0x00
#define MANAGER_CTRL_SHOW_ALL_MOTORS	0x01

typedef struct lt_motor_node_object* lt_node_t;
struct lt_motor_node_object
{
	lt_node_t prev;
	lt_node_t next;
	lt_motor_t motor;
	char name[LT_NAME_MAX];
};

struct lt_motor_manager_object
{
	lt_node_t list;					/* motor list */
	struct lt_motor_info info;		/* motor information */
};
typedef struct lt_motor_manager_object* lt_manager_t;

/* when created a motor, it would be added to the manager, when deleting a motor, it would be removed from the manager */
/* there is only one motor manager */
int lt_manager_create(void);
rt_err_t lt_manager_add_motor(lt_motor_t motor);
rt_err_t lt_manager_delete_motor(lt_motor_t motor);
lt_motor_t lt_manager_get_motor(char* name);
rt_err_t lt_manager_delete(void);
/*****************************************************************************************************/
#ifdef LT_USING_MOTOR_MSH_TEST
/* test example functions */
void test_motor_output(lt_motor_t motor,float val);
void test_motor_output_angle(lt_motor_t motor, float val);
void test_motor_output_pid(lt_motor_t motor, float input);
void test_motor_output_angle_pid(lt_motor_t motor, float input);
void test_motor_get_position(lt_motor_t motor);
void test_motor_get_velocity(lt_motor_t motor);
void test_stepper_trapzoid(lt_motor_t motor, int step,float acc, float dec,float speed);
void test_stepper_s_curve(lt_motor_t motor, int step, float acc_t, float freq_max, float freq_min, float flexible);
void test_stepper_5_section(lt_motor_t motor,int step,float acc_t, float speed);
void test_stepper_line_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_pos, int y_pos,int x_end, int y_end);
void test_stepper_circular_interp(lt_motor_t x_motor, lt_motor_t y_motor, int x_start, int y_start, int x_end, int y_end, rt_uint16_t radius,rt_uint8_t dir);
void test_bldc_torque(lt_motor_t motor,float input);
void test_bldc_elec_info(lt_motor_t motor);
#endif
rt_err_t test_motor_dc_config(void);
rt_err_t test_stepper_x_config(void);
rt_err_t test_stepper_y_config(void);
rt_err_t test_bldc_x_config(void);
rt_err_t test_bldc_y_config(void);
rt_err_t test_close_loop_pid(char*name);

void test_pid_simple(lt_timer_t timer);
void test_velocity_loop(lt_timer_t timer);
void test_position_loop(lt_timer_t timer);
void test_position_velocity_loop(lt_timer_t timer);
/*****************************************************************************************************/
/* communicate with upper computer to operate motor pid control */
struct lt_communicator_object	
{
	rt_device_t dev;					/* process device */
	rt_thread_t thread;					/* receive thread */
	struct rt_semaphore rx_sem;    		/* semaphore for read */
	rt_uint8_t* buffer;					/* read buffer */
	rt_size_t buf_size;					/* buffer size */
	
	const struct lt_commun_ops* ops;	/* operators */
};
typedef struct lt_communicator_object* lt_commun_t;

struct lt_commun_ops
{
	void(*send)(lt_commun_t communicator,int cmd,rt_uint8_t channel,void*data,rt_uint8_t num);
	rt_uint8_t(*process)(lt_commun_t communicator,void* info);
	void(*data_recv)(lt_commun_t communicator,rt_uint8_t* data, rt_uint16_t length);
};


void lt_communicator_send(int cmd,rt_uint8_t channel,void*data,rt_uint8_t num);
void lt_communicator_set(char* serial,rt_size_t buf_size,struct lt_commun_ops* ops);
rt_uint8_t lt_communicator_receive(void*info);
rt_err_t lt_communicator_delete(void);
#endif
