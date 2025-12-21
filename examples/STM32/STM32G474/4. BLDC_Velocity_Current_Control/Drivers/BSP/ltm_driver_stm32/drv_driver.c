/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"

lt_driver_t driver;

/* PWM driver hardware implementation */
/* code below are taken from 正点原子 */
/* some modifications are added !!! */
/**
 ****************************************************************************************************
 * @file        bldc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       定时器 驱动代码
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
 extern TIM_HandleTypeDef g_atimx_handle;           /* 定时器x句柄 */

/********************************************************************************************/
/* driver device operators' hardware implementation */
static struct lt_driver_ops ops;

void drv_driver_init(void)
{
	/* create current sense device */
	if(driver != NULL) return;				/* avoid repeated initialization */
	driver = lt_driver_create("current");
	if(driver == NULL) return;
	/* config current device */
	struct lt_driver_config config;
	config.ops = &ops;
	config.name = NULL;
	
	lt_driver_set(driver,&config);
}

static void _driver_start(lt_driver_t driver)
{
	if(!(driver->flag & DEVICE_FLAG_CALIB))	/* hardware not init */
	{
		GPIO_InitTypeDef gpio_init_struct;
    
		SHUTDOWN_PIN_GPIO_CLK_ENABLE();
  
		gpio_init_struct.Pin = SHUTDOWN_PIN;
		gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
		gpio_init_struct.Pull = GPIO_NOPULL;
		gpio_init_struct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(SHUTDOWN_PIN_GPIO, &gpio_init_struct);    
		/* Init hardware */
		driver->flag |= DEVICE_FLAG_CALIB;		
	}
	/* start half bridges' IC output */
    HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_SET);
	/* start PWM */
    HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH1);
    HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH2);
    HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH3);
}

static void _driver_stop(lt_driver_t driver)
{
	uint32_t period = g_atimx_handle.Init.Period;
    /* stop half bridges' IC output */
    HAL_GPIO_WritePin(SHUTDOWN_PIN_GPIO,SHUTDOWN_PIN,GPIO_PIN_RESET);

    /* stop PWM */
    HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH1);
    HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH2);
    HAL_TIM_PWM_Stop(&g_atimx_handle, ATIM_TIMX_PWM_CH3);
	/* because we use center aligned PWM, and effective polarity is HIGH with PWM2, so duty would be transformed !!! */
    /* stop all bridges */
    __HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH1, period);	/* duty = 0 */
	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH2, period);	/* duty = 0 */
    __HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH3, period);	/* duty = 0 */
    HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT, M1_LOW_SIDE_U_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT, M1_LOW_SIDE_V_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT, M1_LOW_SIDE_W_PIN, GPIO_PIN_RESET);
}

static void _driver_output(lt_driver_t driver, float dutyA,float dutyB,float dutyC)
{
//	uint32_t period = g_atimx_handle.Init.Period;
//	/* if this mode is used, then complementary PWM output must be enable */
//	/* upper bridges */
//	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH1,(1.0f - dutyA) * period);
//	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH2,(1.0f - dutyB) * period);
//	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH3,(1.0f - dutyC) * period);
//	
}
static void _driver_output2(lt_driver_t driver, uint8_t* pulses, float duty)
{
	uint32_t period = g_atimx_handle.Init.Period;
	/* upper bridges */
	/* because we use center aligned PWM, and effective polarity is HIGH with PWM2, so duty would be transformed !!! */
	/* A+ */
	if(pulses[0] == 1) 	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH1,(1.0f - duty) * period);
	else				__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH1, period);
	/* B+ */
	if(pulses[2] == 1)	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH2,(1.0f - duty) * period);
	else				__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH2, period);
	/* C+ */
	if(pulses[4] == 1)	__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH3,(1.0f - duty) * period);
	else				__HAL_TIM_SET_COMPARE(&g_atimx_handle,ATIM_TIMX_PWM_CH3, period);
	
	/* lower bridges */
	/* A- */
	if(pulses[1] == 1)	HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_SET);
	else				HAL_GPIO_WritePin(M1_LOW_SIDE_U_PORT,M1_LOW_SIDE_U_PIN,GPIO_PIN_RESET);
	/* B- */
	if(pulses[3] == 1)	HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_SET);
	else				HAL_GPIO_WritePin(M1_LOW_SIDE_V_PORT,M1_LOW_SIDE_V_PIN,GPIO_PIN_RESET);
	/* C- */
	if(pulses[5] == 1)	HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_SET);
	else				HAL_GPIO_WritePin(M1_LOW_SIDE_W_PORT,M1_LOW_SIDE_W_PIN,GPIO_PIN_RESET);
}


static struct lt_driver_ops ops = {
											_driver_start,
											_driver_output,
											_driver_output2,
											_driver_stop,
};
