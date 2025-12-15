/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-25  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"

/* debug hardware implementation */
/* code below are taken from 正点原子 */
/**
 ****************************************************************************************************
 * @file        key.c led.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       按键输入 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台：正点原子 STM32G474开发板
 * 在线视频：www.yuanzige.com
 * 技术论坛：http://www.openedv.com/forum.php
 * 公司网址：www.alientek.com
 * 购买地址：zhengdianyuanzi.tmall.com
 *
 * 修改说明
 * V1.0 20230801
 * 第一次发布
 *
 ****************************************************************************************************
 */
 /**
 * @brief       初始化LED相关IO口, 并使能时钟
 * @param       无
 * @retval      无
 */
void led_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    LED0_GPIO_CLK_ENABLE();                                 /* LED0时钟使能 */
    LED1_GPIO_CLK_ENABLE();                                 /* LED1时钟使能 */

    gpio_init_struct.Pin = LED0_GPIO_PIN;                   /* LED0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(LED0_GPIO_PORT, &gpio_init_struct);       /* 初始化LED0引脚 */

    gpio_init_struct.Pin = LED1_GPIO_PIN;                   /* LED1引脚 */
    HAL_GPIO_Init(LED1_GPIO_PORT, &gpio_init_struct);       /* 初始化LED1引脚 */
	/* stop led */
	lt_led_off(LED_NUM_0);
	lt_led_off(LED_NUM_1);
}

 /**
 * @brief       按键初始化函数
 * @param       无
 * @retval      无
 */
void key_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;                          /* GPIO配置参数存储变量 */
    KEY0_GPIO_CLK_ENABLE();                                     /* KEY0时钟使能 */
    KEY1_GPIO_CLK_ENABLE();                                     /* KEY1时钟使能 */
    KEY2_GPIO_CLK_ENABLE();                                     /* KEY2时钟使能 */

    gpio_init_struct.Pin = KEY0_GPIO_PIN;                       /* KEY0引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY0_GPIO_PORT, &gpio_init_struct);           /* KEY0引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = KEY1_GPIO_PIN;                       /* KEY1引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY1_GPIO_PORT, &gpio_init_struct);           /* KEY1引脚模式设置,上拉输入 */

    gpio_init_struct.Pin = KEY2_GPIO_PIN;                       /* KEY2引脚 */
    gpio_init_struct.Mode = GPIO_MODE_INPUT;                    /* 输入 */
    gpio_init_struct.Pull = GPIO_PULLUP;                        /* 上拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
    HAL_GPIO_Init(KEY2_GPIO_PORT, &gpio_init_struct);           /* KEY2引脚模式设置,上拉输入 */
}

 

/********************************************************************************************/
/* communicator hardware implementation */
void drv_debug_init(void)
{
	led_init();
	key_init();
}


static uint8_t _key_read(uint8_t num)
{
	if(num == KEY_NUM_0)	return HAL_GPIO_ReadPin(KEY0_GPIO_PORT,KEY0_GPIO_PIN);
	if(num == KEY_NUM_1) 	return HAL_GPIO_ReadPin(KEY1_GPIO_PORT,KEY1_GPIO_PIN);
	if(num == KEY_NUM_2) 	return HAL_GPIO_ReadPin(KEY2_GPIO_PORT,KEY2_GPIO_PIN);
	return 1;
}	

uint8_t lt_key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;  /* 按键按松开标志 */
    uint8_t keyval = 0xFF;

    if (mode) key_up = 1;       /* 支持连按 */

    if (key_up && (_key_read(KEY_NUM_0) == 0 || _key_read(KEY_NUM_1) == 0 || _key_read(KEY_NUM_2) == 0))  /* 按键松开标志为1, 且有任意一个按键按下了 */
    {
        lt_delay_ms(10);           /* 去抖动 */
        key_up = 0;

        if (_key_read(KEY_NUM_0) == 0)  keyval = KEY_NUM_0;

        if (_key_read(KEY_NUM_1) == 0)  keyval = KEY_NUM_1;

        if (_key_read(KEY_NUM_2) == 0)  keyval = KEY_NUM_2;
    }
    else if (_key_read(KEY_NUM_0) == 1 && _key_read(KEY_NUM_1) == 1 && _key_read(KEY_NUM_2) == 1)         /* 没有任何按键按下, 标记按键松开 */
    {
        key_up = 1;
    }

    return keyval;              /* 返回键值 */
}

void lt_led_on(uint8_t num)
{
	if(num == LED_NUM_0)	HAL_GPIO_WritePin(LED0_GPIO_PORT,LED0_GPIO_PIN,GPIO_PIN_RESET);
	if(num == LED_NUM_1) 	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_GPIO_PIN,GPIO_PIN_RESET);
}

void lt_led_off(uint8_t num)
{
	if(num == LED_NUM_0)	HAL_GPIO_WritePin(LED0_GPIO_PORT,LED0_GPIO_PIN,GPIO_PIN_SET);
	if(num == LED_NUM_1) 	HAL_GPIO_WritePin(LED1_GPIO_PORT,LED1_GPIO_PIN,GPIO_PIN_SET);
}

void lt_led_toggle(uint8_t num)
{
	if(num == LED_NUM_0)	HAL_GPIO_TogglePin(LED0_GPIO_PORT,LED0_GPIO_PIN);
	if(num == LED_NUM_1) 	HAL_GPIO_TogglePin(LED1_GPIO_PORT,LED1_GPIO_PIN);
}


