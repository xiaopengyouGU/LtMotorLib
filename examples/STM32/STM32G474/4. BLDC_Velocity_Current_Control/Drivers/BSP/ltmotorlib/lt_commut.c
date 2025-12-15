/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       	Notes
 * 2025-12-5      Lvtou			The first version
 */
#include "ltmotorlib.h"

static struct lt_commut_object _commut_object;
static lt_commut_t commut = &_commut_object;

void lt_commut_send(uint8_t data_type)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DEVICE(commut);
	LT_CHECK_MOTOR(commut->motor);
#endif
	commut->ops->send(commut,data_type);
}

void lt_commut_set_curve(uint8_t ch, int value)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DEVICE(commut);
	LT_CHECK_MOTOR(commut->motor);
#endif
	if(ch > 5 || ch < 1) return;
	commut->curves[ch-1] = value;
}

void lt_commut_set(struct lt_commut_config* config)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(commut);
	LT_CHECK_NULL(config);
	LT_CHECK_NULL(config->ops);
	LT_CHECK_NULL(config->ops->process);
	LT_CHECK_NULL(config->ops->send);
#endif
	memset(commut,0,sizeof(struct lt_commut_object));
	if(config->name != NULL)
	{
		strcpy(commut->name,config->name);
	}
	commut->ops =	config->ops;
	
	commut->flag |= DEVICE_FLAG_INIT;
	commut->flag |= DEVICE_FLAG_CHECKED;
}

void lt_commut_set_motor(lt_motor_t motor, uint8_t pid_type)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DEVICE(commut);
	LT_CHECK_MOTOR(motor);
#endif
	lt_pid_t pid;
	switch (pid_type)
	{
		case PID_TYPE_POS:
		{
			pid = motor->pos_pid;
			break;
		}
		case PID_TYPE_VEL:
		{
			pid = motor->vel_pid;
			break;
		}
		case PID_TYPE_CURR_D:
		{
			pid = motor->curr_pid_d;
			break;
		}
		case PID_TYPE_CURR_Q:
		{
			pid = motor->curr_pid_q;
			break;
		}
		default:break;
	}
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(pid);
#endif	
	commut->motor = motor;
	commut->pid = pid;
}

void lt_commut_process()
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DEVICE(commut);
	LT_CHECK_MOTOR(commut->motor);
#endif
	commut->ops->process(commut);
}
