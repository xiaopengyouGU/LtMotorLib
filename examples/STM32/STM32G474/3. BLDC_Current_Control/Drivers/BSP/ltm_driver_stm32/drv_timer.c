/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"

/* timer driver hardware implementation */
/* code below are taken from 正点原子 */
/* some modifications are added !!! */
/**
 ****************************************************************************************************
 * @file        bldc_tim.c
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
/******************************************************************************************/
/* 定时器配置句柄 定义 */

/* 高级定时器PWM */
TIM_MasterConfigTypeDef g_master_config;
TIM_HandleTypeDef g_atimx_handle;           /* 定时器x句柄 */
TIM_OC_InitTypeDef g_atimx_oc_chy_handle;   /* 定时器输出句柄 */
/******************************************************************************************/

/**
 * @brief       高级定时器TIMX PWM输出初始化函数
 * @note
 *              高级定时器的时钟来自APB2, 而PCLK2 = 170Mhz, 我们设置PPRE2不分频, 因此
 *              高级定时器时钟 = 170Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void atim_timx_oc_chy_init(uint16_t arr, uint16_t psc)
{
    ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* TIMX 时钟使能 */

    g_atimx_handle.Instance = ATIM_TIMX_PWM;                    /* 定时器x */
    g_atimx_handle.Init.Prescaler = psc;                        /* 定时器分频 */
    g_atimx_handle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;       /* PWM center aligned mode 1 */
    g_atimx_handle.Init.Period = arr;                           /* 自动重装载值 */
    g_atimx_handle.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;   /* 分频因子 */
    g_atimx_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; 		/*使能TIMx_ARR进行缓冲*/
    g_atimx_handle.Init.RepetitionCounter = 0;                  /* 开始时不计数*/
    HAL_TIM_PWM_Init(&g_atimx_handle);                          /* 初始化PWM */

    g_atimx_oc_chy_handle.OCMode = TIM_OCMODE_PWM2;             /* 模式选择PWM2 */
    g_atimx_oc_chy_handle.Pulse = 0;
    g_atimx_oc_chy_handle.OCPolarity = TIM_OCPOLARITY_HIGH;     /* with PWM2, when cnt < CCR, ouput 0; when cnt > CCR output 1 */
    g_atimx_oc_chy_handle.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    g_atimx_oc_chy_handle.OCFastMode = TIM_OCFAST_DISABLE;
    g_atimx_oc_chy_handle.OCIdleState = TIM_OCIDLESTATE_RESET;
    g_atimx_oc_chy_handle.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH1); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH2); /* 配置TIMx通道y */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_handle, &g_atimx_oc_chy_handle, ATIM_TIMX_PWM_CH3); /* 配置TIMx通道y */

	/*	config output trigger */
	g_master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;									/* update event trigger */
	g_master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	HAL_TIMEx_MasterConfigSynchronization(&g_atimx_handle,&g_master_config);
	/* Start PMW at first */
	HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH1);
    HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH2);
    HAL_TIM_PWM_Start(&g_atimx_handle, ATIM_TIMX_PWM_CH3);
    /* 启动高级定时器1以及开启中断 */
    HAL_TIM_Base_Start_IT(&g_atimx_handle);     
}


/**
 * @brief       定时器底层驱动，时钟使能，引脚配置
                此函数会被HAL_TIM_PWM_Init()调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == ATIM_TIMX_PWM)
    {
        GPIO_InitTypeDef gpio_init_struct;
        ATIM_TIMX_PWM_CHY_CLK_ENABLE();                             /* 定时器时钟使能 */
        
        /* 上桥臂的IO时钟使能 */
        ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE();                      
        ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE();                   
        ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE();                       
        
        /* 下桥臂的IO时钟使能 */
        M1_LOW_SIDE_U_GPIO_CLK_ENABLE();                         
        M1_LOW_SIDE_V_GPIO_CLK_ENABLE();                    
        M1_LOW_SIDE_W_GPIO_CLK_ENABLE();                            

        /* 下桥臂的IO初始化 */
        gpio_init_struct.Pin = M1_LOW_SIDE_U_PIN;
        gpio_init_struct.Pull = GPIO_NOPULL;
        gpio_init_struct.Speed = GPIO_SPEED_HIGH;
        gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;                /* 推挽输出模式 */
        HAL_GPIO_Init(M1_LOW_SIDE_U_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_V_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_V_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = M1_LOW_SIDE_W_PIN;
        HAL_GPIO_Init(M1_LOW_SIDE_W_PORT, &gpio_init_struct);


        /* 上桥臂即定时器IO初始化 */
        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH1_GPIO_PIN;          /* 通道y的GPIO口 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                    /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_NOPULL;                        /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;              /* 高速 */
        gpio_init_struct.Alternate = ATIM_TIMX_PWM_CHY_GPIO_AF;     /* 端口复用 */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH1_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH2_GPIO_PIN;          /* 通道y的CPIO口 */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH2_GPIO_PORT, &gpio_init_struct);

        gpio_init_struct.Pin = ATIM_TIMX_PWM_CH3_GPIO_PIN;          /* 通道y的CPIO口 */
        HAL_GPIO_Init(ATIM_TIMX_PWM_CH3_GPIO_PORT, &gpio_init_struct);
        
        HAL_NVIC_SetPriority(ATIM_TIMX_PWM_IRQn, 3, 1);				/* high priority : 1 !!! */
        HAL_NVIC_EnableIRQ(ATIM_TIMX_PWM_IRQn);

    }
}

/******************************************* 基本定时器初始化 **********************************************/
TIM_HandleTypeDef g_timx_handle;    /* 定时器句柄 */

/**
 * @brief       基本定时器TIMX定时中断初始化函数
 * @note
 *              基本定时器的时钟来自APB1, 而APB1为170M, 所以定时器时钟 = 170Mhz
 *              定时器溢出时间计算方法: Tout = ((arr + 1) * (psc + 1)) / Ft us.
 *              Ft=定时器工作频率,单位:Mhz
 *
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */
void btim_timx_int_init(uint16_t arr, uint16_t psc)
{
    g_timx_handle.Instance = BTIM_TIMX_INT;                     /* 通用定时器X */
    g_timx_handle.Init.Prescaler = psc;                         /* 设置预分频系数 */
    g_timx_handle.Init.CounterMode = TIM_COUNTERMODE_UP;        /* 递增计数模式 */
    g_timx_handle.Init.Period = arr;                            /* 自动装载值 */
    HAL_TIM_Base_Init(&g_timx_handle);

    HAL_TIM_Base_Start_IT(&g_timx_handle);                      /* 使能定时器x及其更新中断 */
}

/**
 * @brief       定时器底层驱动，开启时钟，设置中断优先级
                此函数会被HAL_TIM_Base_Init()函数调用
 * @param       htim:定时器句柄
 * @retval      无
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == BTIM_TIMX_INT)
    {
        BTIM_TIMX_INT_CLK_ENABLE();                             /* 使能TIM时钟 */
        HAL_NVIC_SetPriority(BTIM_TIMX_INT_IRQn, 3, 3);         /* lower priority compared with other priorities */
        HAL_NVIC_EnableIRQ(BTIM_TIMX_INT_IRQn);                 /* 开启ITM6中断 */
    }
}

/***********************************************定时器中断服务函数***********************************************/
/**
 * @brief       定时器TIMX中断服务函数
 * @param       无
 * @retval      无
 */
void BTIM_TIMX_INT_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_timx_handle);                         /* 定时器中断公共处理函数 */
}

/**
 * @brief       定时器中断服务函数
 * @param       无
 * @retval      无
 */
void ATIM_TIMX_PWM_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_atimx_handle);
}

/***********************************************定时器中断回调函数***********************************************/
/**
 * @brief       定时器中断回调
 * @param       htim:定时器句柄
 * @retval      无
 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint8_t cnt = 0;                                         /* 定时器时间记录 */
	
    if(htim->Instance == BTIM_TIMX_INT)                                     /* 1ms */
    {
		cnt++;
		ltm_motor_run(LOOP_FLAG_VELOCITY);
//		/* we can do safe protection in interruption, but we better do the same in RTOS threads */
//		/* safe protection ! */	
//		if((count % 500) == 0)						/* 500 ms : check temperature */
//		{
//			
//		}
//		if((count % 100) == 0)						/* 100 ms : Host computer communication */
//		{
//			
//		}
//		if((count % 20) == 0)						/* 20 ms : bus voltage detection */
//		{
//			
//		}
//		if((count % 10) == 0)						/* 10 ms : over current detection */
//		{
//			
//		}
		
    }
}


/********************************************************************************************/
/* timer hardware implementation */

void drv_timer_init(void)
{	
	/* because we use center aligned PWM, so fpwm = 1/(2*(arr+1)*(psc+1) = 18K */
	atim_timx_oc_chy_init((170000 / 36) - 1, 0);            /* 中断频率18K，约55us, 频率不能过慢或过快，否则都会导致电机堵转 */
	btim_timx_int_init(1000 - 1, 170 - 1);     				/* 基本定时器初始化，1ms中断一次 */
}

