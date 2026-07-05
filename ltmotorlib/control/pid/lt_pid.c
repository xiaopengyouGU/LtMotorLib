/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        The first version
 * 2025-10-21     Lvtou        Remove dependancies on RT-Thread 
 * 2025-11-29	  Lvtou		   Modity API and improve computation efficiency
 * 2025-12-15	  Lvtou		   Exchange the implementations of two types of pid 
 */
#include "control/pid/lt_pid.h"
#include "math/basic/lt_math.h"
#include <stdlib.h>
#include <string.h>

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

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float ts_ms)
{
	lt_pid_t pid;
	pid = (lt_pid_t)malloc(sizeof(struct lt_pid_object));
	if(pid == NULL) return NULL;
	memset(pid,0,sizeof(struct lt_pid_object));
	float ts = ts_ms / 1000.0f;						/* unit : s */
	pid->ts = ts;
	pid->Kp = Kp;
	pid->Ki_ts = Ki * ts ;
	pid->Kd_ts = Kd/ts;
	pid->int_limit = PID_INTEGRAL_LIMIT;			/* default integral limit */
	pid->output_limit = PID_OUTPUT_LIMIT;			/* default output limit */
	return pid;
}

void lt_pid_delete(lt_pid_t pid)
{
	if(pid == NULL)		return;						// 空指针检查
	free(pid);
}

void lt_pid_reset(lt_pid_t pid)
{
	if(pid == NULL)		return;						// 空指针检查
	pid->output = 0.0f;
	pid->err = 0.0f;
	pid->err_prev = 0.0f;
	pid->err_prev2 = 0.0f;
	pid->integral = 0.0f;
}

void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd)
{
	if(pid == NULL)		return;						// 空指针检查
	pid->Kp = Kp;
	pid->Ki_ts = Ki * pid->ts;
	pid->Kd_ts = Kd / pid->ts;
}
void lt_pid_set_target(lt_pid_t pid, float target)
{
	if(pid == NULL)		return;						// 空指针检查
	pid->target = target;
}

void lt_pid_set_ts(lt_pid_t pid, float ts_ms)
{
	if(pid == NULL)		return;						// 空指针检查

	float ts = ts_ms / 1000.0f;				/* unit : s */
	float Ki = pid->Ki_ts / pid->ts;		/* get original values */
	float Kd = pid->Kd_ts * pid->ts;	
	/* set values */
	pid->ts = ts;
	pid->Ki_ts = Ki * ts ;
	pid->Kd_ts = Kd/ts;
}

void lt_pid_set_limits(lt_pid_t pid,float int_limit, float output_limit)
{
	if(pid == NULL)		return;						// 空指针检查
	if(int_limit < 0) int_limit = -int_limit;
	if(output_limit < 0) output_limit = -output_limit;
	pid->int_limit = int_limit;
	pid->output_limit = output_limit;
}

float lt_pid_get(lt_pid_t pid)
{
	if(pid == NULL)		return 0;						// 空指针检查
	return pid->output;
}

float lt_pid_process(lt_pid_t pid,float curr_val)	/* increment pid */
{
	if(pid == NULL)		return 0;						// 空指针检查

	float err = pid->target - curr_val;
	float err_prev = pid->err_prev;
	float P,I,D;
	float output_limit = pid->output_limit;
	float output;
	/* start calculation */
	P = pid->Kp * (err - err_prev);
	I = pid->Ki_ts * err;
	D = pid->Kd_ts * (err - 2.0f*err_prev + pid->err_prev2);
	/* output limit */
	output = pid->output + P + I + D;
	output = CONSTRAINS(output, output_limit, -output_limit);
	
	/* refresh pid parameters */
	pid->err_prev = err;
	pid->err_prev2 = err_prev;
	pid->output = output;
	
	return output;
}

float lt_pid_process2(lt_pid_t pid, float curr_val)		/* position pid */
{
	if(pid == NULL)		return 0;						// 空指针检查

	float P,I,D;
	float err = pid->target - curr_val;
	float int_limit = pid->int_limit;
	float output_limit = pid->output_limit;
	float output;
	/* start calculating */
	P = pid->Kp * err;
	D = pid->Kd_ts * (err - pid->err);
	I = pid->integral + pid->Ki_ts * err;
	/* integral limit */
	if(I > int_limit) I = int_limit;
	else if(I < -int_limit) I = -int_limit;
	/* output limit */
	output = P + I + D;
	output = CONSTRAINS(output, output_limit, -output_limit);
	
	/* refresh pid parameters */
	pid->err = err;
	pid->integral = I;
	pid->output = output;
	
	return output;
}
