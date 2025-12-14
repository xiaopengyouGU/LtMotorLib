/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19  	  Lvtou		   The first version
 * 2025-12-09	  Lvtou		   Add hall velocity measure function and modify APIs
 */
#include "ltmotorlib.h"
lt_sensor_t sensor;
/* timer driver hardware implementation */
/* code below are taken from 正点原子 */
/* some modifications are added !!! */
/**
 ****************************************************************************************************
 * @file        bldc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief      	BLDC 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32G474开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20230801
 * 第一次发布
 *
 ****************************************************************************************************
 */
/******************************************************************************************/
/* 定时器配置句柄 定义 */
/**
  * @brief  霍尔传感器接口初始化
  * @param  无
  * @retval 无
  */
void hall_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    HALL1_U_GPIO_CLK_ENABLE();
    HALL1_V_GPIO_CLK_ENABLE();
    HALL1_W_GPIO_CLK_ENABLE();

    /* 霍尔通道 1 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH1_PIN;
    gpio_init_struct.Mode = GPIO_MODE_INPUT;
    gpio_init_struct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(HALL1_TIM_CH1_GPIO, &gpio_init_struct);

    /* 霍尔通道 2 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH2_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH2_GPIO, &gpio_init_struct);

    /* 霍尔通道 3 引脚初始化 */
    gpio_init_struct.Pin = HALL1_TIM_CH3_PIN;
    HAL_GPIO_Init(HALL1_TIM_CH3_GPIO, &gpio_init_struct);

}

/********************************************************************************************/
/* position sensor device operators' hardware implementation */
static struct lt_sensor_ops ops;
static uint32_t count = 0;
static uint8_t value_old = 0;							/* old value */
static float vel_old = 0;								/* old velocity */

void drv_sensor_init(void)
{	
	/* create current sense device */
	if(sensor != NULL) return;				/* avoid repeated initialization */
	sensor = lt_sensor_create("sensor");
	if(sensor == NULL) return;
	/* config current device */
	struct lt_sensor_config config;
	config.ops = &ops;
	config.type = SENSOR_TYPE_HALL;

	lt_sensor_set(sensor,&config);
}

#define HALL_VEL_GAINS		2.0f*136363.6f  /* (60.0f/((2.0f * 4.0f)*(55.0f * 1e-6f))) : 60 / Ts / 2 / pn / count */
#define HALL_VEL_MAX		4000.0f

static void _hall_start(lt_sensor_t sensor)
{
	if(sensor->flag & DEVICE_FLAG_CALIBING)	/* this hall sensor is firstly started, we need to init it */
	{
		hall_gpio_init();
		sensor->flag = CLEAR_BITS(sensor->flag,DEVICE_FLAG_CALIBING);
	}
}

static void _hall_calibrate(lt_sensor_t sensor)
{
	count = 0;
	value_old = 0;
	vel_old = 0;
	sensor->flag |= DEVICE_FLAG_CALIB;
}

static void _sensor_get(lt_sensor_t sensor, float* pos)
{

}

static void _hall_get2(lt_sensor_t sensor, uint8_t* signal, float* vel)
{
	uint8_t value = 0;
	uint8_t hall_signal = 0;

	if(HAL_GPIO_ReadPin(HALL1_TIM_CH1_GPIO, HALL1_TIM_CH1_PIN) != GPIO_PIN_RESET)  /* read U phase */
	{
		hall_signal |= 0x04;
	}
	if(HAL_GPIO_ReadPin(HALL1_TIM_CH2_GPIO, HALL1_TIM_CH2_PIN) != GPIO_PIN_RESET)  /* read V phase  */
	{
		hall_signal |= 0x02;
	}
	if(HAL_GPIO_ReadPin(HALL1_TIM_CH3_GPIO, HALL1_TIM_CH3_PIN) != GPIO_PIN_RESET)  /* read W phase */
	{
		hall_signal |= 0x01;
		value = 1;
	}

	/* get position and velocity */
	if(value_old == value)
	{
		count++;
		if(count > 40000)		/* we can assume that motor is stopped, t = 40000*55us =  2200ms = 2.2s  */
		{
			count = 0;
			vel_old = 0;
			return;			
		}
	}
	else					   /* measure time in edge */	
	{
		if(count != 0)
		{
			vel_old =  HALL_VEL_GAINS /((float)count);
			vel_old = (vel_old > HALL_VEL_MAX)? HALL_VEL_MAX : vel_old;
			/* get velocity */
			if(sensor->dir == DIR_CW) vel_old = -vel_old;
		}
		else	vel_old = 0;		/* error velocity, set zero velocity */
		
		value_old = value;		/* refresh old hall value */
		count = 0;
	}

	*vel = vel_old;
	*signal = hall_signal;		/* type transform */
}

static void _hall_stop(lt_sensor_t sensor)
{
	
}

static struct lt_sensor_ops ops = {	
										_hall_start,
										_hall_calibrate,
										_sensor_get,
										_hall_get2,
										_hall_stop,
};


