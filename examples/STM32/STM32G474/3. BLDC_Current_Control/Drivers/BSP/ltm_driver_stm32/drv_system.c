/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-16  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"

/* code below are taken from 正点原子, some modifications are added!!! */
/*
 ****************************************************************************************************
 * @file        sys.c delay.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       系统初始化代码(包括时钟配置/中断管理/GPIO设置等)
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
 ****************************************************************************************************
 */

/**
 * @brief       时钟设置函数
 * @param       plln: 主PLL倍频系数(PLL倍频), 取值范围: 8~127.
 * @param       pllm: 主PLL和音频PLL预分频系数(进PLL之前的分频), 取值范围: 1~16.
 * @param       pllr: 主PLL的r分频系数(PLL之后的分频), 分频后作为系统时钟, 取值范围: 2, 4, 6, 8.(仅限这4个值)
 * @param       pllp: 主PLL的p分频系数(PLL之后的分频), 取值范围: 2~31.
 * @param       pllq: 主PLL的q分频系数(PLL之后的分频), 取值范围: 2, 4, 6, 8.(仅限这4个值)
 * @note
 *
 *              Fvco: VCO频率
 *              Fsys: 系统时钟频率, 也是主PLL的p分频输出时钟频率
 *              Fq:   主PLL的q分频输出时钟频率
 *              Fs:   主PLL输入时钟频率, 可以是HSI, HSE等.
 *              Fvco = Fs * (plln / pllm);
 *              Fsys = Fvco / pllr = Fs * (plln / (pllm * pllr));
 *              Fq   = Fvco / pllq = Fs * (plln / (pllm * pllq));
 *
 *              外部晶振为 8M的时候, 推荐值: plln = 85, pllm = 2, pllr = 2, pllp = 4, pllq = 8.
 *              得到:Fvco = 8 * (85 / 2) = 340Mhz
 *                   Fsys = pll_p_ck = 340 / 2 = 170Mhz
 *                   Fq   = pll_q_ck = 340 / 8 = 42.5Mhz
 *
 *              G474默认需要配置的频率如下:
 *              CPU频率(HCLK) = pll_p_ck = 170Mhz
 *              AHB1/2/3(rcc_hclk1/2/3) = 170Mhz
 *              APB1(rcc_pclk1) = pll_p_ck / 1 = 170Mhz
 *              APB1(rcc_pclk2) = pll_p_ck / 1 = 170Mhz
 *
 * @retval      错误代码: 0, 成功; 1, 错误;
 */
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllr,uint32_t pllp, uint32_t pllq)
{
    HAL_StatusTypeDef ret = HAL_OK;
    RCC_OscInitTypeDef rcc_osc_init = {0};
    RCC_ClkInitTypeDef rcc_clk_init = {0};

    __HAL_RCC_PWR_CLK_ENABLE();                                         /* 使能PWR时钟 */

    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);/* 设置调压器输出电压级别，以便在器件未以最大频率工作 */
    
    /* 使能HSE，并选择HSE作为PLL时钟源 */
    rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;        /* 时钟源为HSE */
    rcc_osc_init.HSEState = RCC_HSE_ON;                          /* 打开HSE */
    rcc_osc_init.PLL.PLLState = RCC_PLL_ON;                      /* 打开PLL */
    rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;              /* PLL时钟源选择HSE */
    rcc_osc_init.PLL.PLLN = plln;
    rcc_osc_init.PLL.PLLM = pllm;
    rcc_osc_init.PLL.PLLP = pllp;
    rcc_osc_init.PLL.PLLQ = pllq;
    rcc_osc_init.PLL.PLLR = pllr;
    ret = HAL_RCC_OscConfig(&rcc_osc_init);                      /* 初始化RCC */
    if(ret != HAL_OK)
    {
        return 1;                                                /* 时钟初始化失败，可以在这里加入自己的处理 */
    }

    /* 选中PLL作为系统时钟源并且配置HCLK,PCLK1和PCLK2 */
    rcc_clk_init.ClockType = ( RCC_CLOCKTYPE_SYSCLK \
                                    | RCC_CLOCKTYPE_HCLK \
                                    | RCC_CLOCKTYPE_PCLK1 \
                                    | RCC_CLOCKTYPE_PCLK2);

    rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;         /* 设置系统时钟时钟源为PLL */
    rcc_clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;                /* AHB分频系数为1 */
    rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV1;                 /* APB1分频系数为1 */
    rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV1;                 /* APB2分频系数为1 */
    ret = HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_4);   /* 同时设置FLASH延时周期为4WS，也就是5个CPU周期 */
    if(ret != HAL_OK)
    {
        return 1;                                                /* 时钟初始化失败 */
    }
    
    /* STM32G4x Z版本的器件支持预取功能 */
    if (HAL_GetREVID() == 0x1001)
    {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();                    /* 使能flash预取 */
    }
    return 0;
}
/*****************************************************************************************/
static uint32_t g_fac_us = 0;       /* us延时倍乘数 */

/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(HCLK),等于系统时钟主频，单位MHz
 * @retval    无
 */  
void delay_init(uint16_t sysclk)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);/* SYSTICK使用外部时钟源,频率为HCLK */
    g_fac_us = sysclk;                                  /* 不论是否使用OS,g_fac_us都需要使用 */
}

/**
 * @brief     延时nus
 * @note      无论是否使用OS, 都是用时钟摘取法来做us延时
 * @param     nus: 要延时的us数
 * @note      nus取值范围: 0 ~ (2^32 / fac_us) (fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOAD的值 */
    ticks = nus * g_fac_us;                 /* 需要的节拍数 */

    told = SysTick->VAL;                    /* 刚进入时的计数器值 */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* 这里注意一下SYSTICK是一个递减的计数器就可以了 */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks) 
            {
                break;                      /* 时间超过/等于要延迟的时间,则退出 */
            }
        }
    }
}

/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= (2^32 / fac_us / 1000))(fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));                   /* 普通方式延时 */
}

/**
 * @brief       HAL库内部函数用到的延时
 * @note        HAL库的延时默认用Systick，如果我们没有开Systick的中断会导致调用这个延时后无法退出
 * @param       Delay : 要延时的毫秒数
 * @retval      None
 */
void HAL_Delay(uint32_t Delay)
{
     delay_ms(Delay);
}

/*****************************************************************************************/
void ltm_driver_init(void)
{
	/* Init system at first !!! */
	drv_system_init();			
	/* then init device init */
	drv_commut_init();
	drv_current_init();
	drv_driver_init();
	drv_timer_init();
	drv_sensor_init();
	drv_debug_init();
	/* Init motor finally !!! */
	drv_motor_init();		
}

void drv_system_init(void)
{
	HAL_Init();                                 /* init HAL Library */
    sys_stm32_clock_init(85, 2, 2, 4, 8);       /* set system clock,170Mhz */
	delay_init(170);							/* delay init */
}

void lt_delay_us(uint32_t us)
{
	delay_us(us);
}

void lt_delay_ms(uint32_t ms)
{
	 delay_us((uint32_t)(ms * 1000));                   /* 普通方式延时 */
}
