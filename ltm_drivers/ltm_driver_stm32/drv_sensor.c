/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19  	  Lvtou		   The first version
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
	sensor->flag |= DEVICE_FLAG_CALIB;
}

static void _hall_get(lt_sensor_t sensor, void* val)
{
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
	}
	
	uint8_t* res = (uint8_t *)val;		/* type transform */
	*res = hall_signal;
}

static void _hall_stop(lt_sensor_t sensor)
{
	
}

static struct lt_sensor_ops ops = {	
										_hall_start,
										_hall_calibrate,
										_hall_get,
										_hall_stop,
};


