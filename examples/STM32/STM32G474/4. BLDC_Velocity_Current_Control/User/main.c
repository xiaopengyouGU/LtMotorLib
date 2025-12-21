/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-8	  Lvtou		   The first version
 */
 
#include "ltmotorlib.h"

extern lt_motor_t motor;

void communicate(uint32_t count);

int main(void)
{
	ltm_driver_init();			/* init all hardwares and drivers */
	uint32_t count = 0;
	uint32_t res;
	uint8_t key;
	float target_vel = 0;
	/* calibrating position sensor and current sensor */
	lt_led_on(LED_NUM_0);			

	res = lt_motor_check(motor);	/* this function is blocking or non-blocking depending on the implementation of lt_delay_ms */
	if(res == LT_EOK)
	{
		lt_motor_start(motor);		/* don't forget this part */
		lt_commut_set_motor(motor,PID_TYPE_VEL);
		//lt_commut_set_motor(motor,PID_TYPE_CURR_Q);
		printf("电机检查完毕，硬件校准完毕!!! \r\n");
		printf("电机启动!!! \r\n");
		printf("按下KEY0 开始正转加速\r\n");
		printf("按下KEY1 开始反转加速\r\n");
		printf("按下KEY2 停止电机\r\n");
	}
	else
	{		
		printf("电机检查失败 !!! \r\n");
	}
	
	/* user interaction */
	while(1)
	{
		/* read key values */
		key = lt_key_scan(0);
		if(key == KEY_NUM_0)
		{							/* forward rotate */
			target_vel += 150;
			lt_motor_start(motor);
 		}
		else if(key == KEY_NUM_1)	/* reversal rotate */
		{
			target_vel -= 150;
			lt_motor_start(motor);
		}
		else if(key == KEY_NUM_2)	/* stop motor */
		{
			target_vel = 0;
			lt_motor_stop(motor);
			continue;
		}	
		/* constrains target velocity */	
		target_vel = CONSTRAINS(target_vel,1800,-1800);
		lt_motor_set_vel(motor,target_vel);
		
		/* communicate with the upper computer */
		communicate(count);
		lt_delay_ms(5);
		count++;
	}
		
}

float I_bus_t;
float vel_t;
void communicate(uint32_t count)
{
	/* process communication protocol */
	lt_commut_process();							/* this is the key function !!! */
	/* get motor information */
	lt_motor_monitor(motor);						/* we need monitor motor information at first */
	lt_info_t info = lt_motor_get(motor);			/* then get information */
	LT_CHECK_NULL(info);							/* if we can't get info, return !!! */
	
	vel_t = LOW_PASS_FILTER(info->vel,vel_t,0.234f);
	
	lt_commut_set_curve(1,(int)(info->target_vel));
	lt_commut_set_curve(2,(int)(vel_t));
	lt_commut_set_curve(3,(int)(info->Id * 1000));
	lt_commut_set_curve(4,(int)(info->Iq * 1000));
	lt_commut_set_curve(5,(int)(info->I_bus * 1000));
	/* send data to the upper computer */
	if(info->flag & MOTOR_FLAG_RUN)
	{
		lt_commut_send(CMD_SEND_CURVES);
	}
	if(count % 50 == 0)
	{
		lt_led_toggle(LED_NUM_0);
	}
	if(count % 200 == 0)
	{
		lt_led_toggle(LED_NUM_1);
		/* send pid parameters to the upper computer */
		if(info->flag & MOTOR_FLAG_RUN)
		{
			lt_commut_send(CMD_SEND_PID);
			lt_commut_send(CMD_SEND_PERIOD);
			lt_commut_send(CMD_SEND_TARGET);
		}
		//printf("电机运行中: V = %.0f rpm \r\n",target_vel);
		/* printf key information : Idc, Temperature, bus voltage */
		/* send data to the upper computer */
	}
}
