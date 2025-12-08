/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-16  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"
#include "math.h"

lt_current_t current;

/* code below are taken from 正点原子 */
/* some modifications are added !!! */
/**
 ****************************************************************************************************
 * @file        adc.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       ADC 驱动代码
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
DMA_HandleTypeDef g_dma_adc_handle;                         /* 与ADC关联的DMA句柄 */
ADC_HandleTypeDef g_adc_handle;                             /* ADC句柄 */
uint16_t g_adc_dma_buf[ADC_SUM];                          	/* 存储ADC原始值 */
/**
 * @brief       计算ADC的平均值（滤波）
 * @param       * p ：代入ADC原始值
 * @param       * buf ：存放滤波后的ADC值
 * @param       ch_num ：采集的ADC通道数量
 * @param       len ：每个通道的采集次数
 * @note        此函数对电压、温度、电流对应的ADC值进行滤波
 * @retval      无
 */


/**
 * @brief       ADC初始化函数
 *   @note      本函数支持ADC1/ADC2任意通道, 但是不支持ADC3
 *              我们使用12位精度, ADC采样时钟=170/4 = 42.5M, 转换时间为: 采样周期 + 12.5个ADC周期
 *              设置最大采样周期: 12.5, 则转换时间 = 25 个ADC周期 < 1us
 * @param       无
 * @retval      无
 */
 
 
/**
 * @brief       设置ADC通道采样时间
 * @param       adcx : adc句柄指针,ADC_HandleTypeDef
 * @param       ch   : 通道号, ADC_CHANNEL_0~ADC_CHANNEL_17
 * @param       stime: 采样时间  0~7, 对应关系为:
 *   @arg       ADC_SAMPLETIME_2CYCLES_5,  2.5个ADC时钟周期        ADC_SAMPLETIME_6CYCLES_5, 6.5个ADC时钟周期
 *   @arg       ADC_SAMPLETIME_12CYCLES_5, 12.5个ADC时钟周期       ADC_SAMPLETIME_24CYCLES_5, 24.5个ADC时钟周期
 *   @arg       ADC_SAMPLETIME_47CYCLES_5, 47.5个ADC时钟周期       ADC_SAMPLETIME_92CYCLES_5, 92.5个ADC时钟周期
 *   @arg       ADC_SAMPLETIME_247CYCLES_5 , 247.5个ADC时钟周期    ADC_SAMPLETIME_640CYCLES_5,640.5个ADC时钟周期
 * @param       rank: 多通道采集时需要设置的采集编号,
                假设你定义channel1的rank=1，channel2的rank=2，
                那么对应你在DMA缓存空间的变量数组AdcDMA[0] 就是通道1的转换结果，AdcDMA[1]就是通道2的转换结果。 
                单通道设置为 ADC_REGULAR_RANK_1
 *   @arg       编号1~16：ADC_REGULAR_RANK_1~ADC_REGULAR_RANK_16
 * @retval      无
 */
void adc_channel_set(ADC_HandleTypeDef *adc_handle, uint32_t ch, uint32_t rank, uint32_t stime)
{
    /* 配置对应ADC通道 */
    ADC_ChannelConfTypeDef adc_channel;
    adc_channel.Channel = ch;                   /* 设置ADCX对通道ch */
    adc_channel.Rank = rank;                    /* 设置采样序列 */
    adc_channel.SamplingTime = stime;           /* 设置采样时间 */
    adc_channel.SingleDiff = ADC_SINGLE_ENDED;  /* 单端输入模式 */
    adc_channel.OffsetNumber = ADC_OFFSET_NONE;
    adc_channel.Offset = 0;
    HAL_ADC_ConfigChannel( adc_handle, &adc_channel );   
}
 
void adc_init(uint8_t flag)
{
	if(flag == DEVICE_FLAG_CALIBING)	/* start calibrating */
	{
		g_adc_handle.Init.ContinuousConvMode =	ENABLE;                         /* 使能连续转换 */
		g_adc_handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;                /* 软件触发 */
		g_adc_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE; /* 使用软件触发 */
	}
	else if(flag == DEVICE_FLAG_CALIB)	/* calirated successfully */
	{
		g_adc_handle.Init.ContinuousConvMode =	DISABLE;                         	/* 禁止连续转换 */
		g_adc_handle.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO;      		/* TIM1 trigger  */
		g_adc_handle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; 	/* update event trigger */
	}
	else 	return;
	
    g_adc_handle.Instance = ADC_ADCX;
    g_adc_handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;            /* 4分频，ADCCLK = SYSCLK/4 = 170/4 = 42.5Mhz */
    g_adc_handle.Init.Resolution = ADC_RESOLUTION_12B;                      /* 12位模式 */
    g_adc_handle.Init.GainCompensation = 0;                                 /* 不需要增益补偿 */
    g_adc_handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;                      /* 右对齐 */
    g_adc_handle.Init.ScanConvMode = ADC_SCAN_ENABLE;                       /* 扫描模式 */
    g_adc_handle.Init.EOCSelection = ADC_EOC_SEQ_CONV;                      /* 规则序列转换结束标志 */
    g_adc_handle.Init.LowPowerAutoWait = DISABLE;                           /* 禁用低功耗延迟模式 */
    g_adc_handle.Init.NbrOfConversion = ADC_CH_NUM;                         /* ADC_CH_NUM个转换在规则序列中 */
    g_adc_handle.Init.DiscontinuousConvMode = DISABLE;                      /* 禁止不连续采样模式 */
    g_adc_handle.Init.NbrOfDiscConversion = 0;                              /* 不连续采样通道数为0 */
    g_adc_handle.Init.DMAContinuousRequests = ENABLE;                       /* 开启DMA请求 */
    g_adc_handle.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;                   /* 数据溢出时覆盖ADC数据 */
    g_adc_handle.Init.OversamplingMode = DISABLE;                           /* 禁用过采样功能 */
    HAL_ADC_Init(&g_adc_handle);                                            /* 初始化 */
    
    HAL_ADCEx_Calibration_Start(&g_adc_handle, ADC_SINGLE_ENDED);           /* 以单端模式运行ADC校准 */
}

/**
 * @brief       ADC底层驱动，引脚配置，时钟使能
                此函数会被HAL_ADC_Init()调用
 * @param       hadc:ADC句柄
 * @retval      无
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    if(hadc->Instance == ADC_ADCX)
    {
        GPIO_InitTypeDef gpio_init_struct;
        RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
        
        /* 设置ADC的时钟源 */
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
        PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
        
        /* 使能ADCx时钟 */
        ADC_ADCX_CHY_CLK_ENABLE();    
        /* 开启GPIO时钟 */
        ADC_VBUS_CHX_GPIO_CLK_ENABLE(); 
        ADC_VTEMP_CHX_GPIO_CLK_ENABLE();     
        ADC_AMPU_CHX_GPIO_CLK_ENABLE();
        ADC_AMPV_CHX_GPIO_CLK_ENABLE();
        ADC_AMPW_CHX_GPIO_CLK_ENABLE();
        
        
        /* AD采集引脚模式设置,模拟输入 */
        gpio_init_struct.Pin = ADC_VBUS_CHX_GPIO_PIN;
        gpio_init_struct.Mode = GPIO_MODE_ANALOG;
        gpio_init_struct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(ADC_VBUS_CHX_GPIO_PORT, &gpio_init_struct);
        
        gpio_init_struct.Pin = ADC_VTEMP_CHX1_GPIO_PIN;
        HAL_GPIO_Init(ADC_VTEMP_CHX_GPIO_PORT, &gpio_init_struct);
        
        gpio_init_struct.Pin = ADC_AMPU_CHX_GPIO_PIN;
        HAL_GPIO_Init(ADC_AMPU_CHX_GPIO_PORT, &gpio_init_struct);
        
        gpio_init_struct.Pin = ADC_AMPV_CHX_GPIO_PIN;
        HAL_GPIO_Init(ADC_AMPV_CHX_GPIO_PORT, &gpio_init_struct);
        
        gpio_init_struct.Pin = ADC_AMPW_CHX_GPIO_PIN;
        HAL_GPIO_Init(ADC_AMPW_CHX_GPIO_PORT, &gpio_init_struct);
    }
}

/**
 * @brief       ADC DMA读取 初始化函数
 *   @note      本函数还是使用adc_init对ADC进行大部分配置,有差异的地方再单独配置
 * @param       par         : 外设地址
 * @param       mar         : 存储器地址
 * @retval      无
 */
void adc_dma_init(uint8_t flag)
{
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA配置 */
    g_dma_adc_handle.Instance = ADC_ADCX_DMASx;                             /* 设置DMA通道 */
    g_dma_adc_handle.Init.Request= ADC_ADCX_DMASx_REQUEST;                  /* 设置DMA请求 */
    g_dma_adc_handle.Init.Direction = DMA_PERIPH_TO_MEMORY;                 /* DIR = 1 ,  外设到存储器模式 */
    g_dma_adc_handle.Init.PeriphInc = DMA_PINC_DISABLE;                     /* 外设非增量模式 */
    g_dma_adc_handle.Init.MemInc = DMA_MINC_ENABLE;                         /* 存储器增量模式 */
    g_dma_adc_handle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;    /* 外设数据长度:16位 */
    g_dma_adc_handle.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;       /* 存储器数据长度:16位 */
    g_dma_adc_handle.Init.Mode = DMA_CIRCULAR;                              /* 外设流控模式 */
    g_dma_adc_handle.Init.Priority = DMA_PRIORITY_MEDIUM;                   /* 中等优先级 */
    HAL_DMA_Init(&g_dma_adc_handle);                                        /* 初始化DMA */
    
    adc_init(flag);                                                             /* 初始化ADC */

    /* 配置对应ADC通道 */
    adc_channel_set(&g_adc_handle, ADC_AMPU_CHX, ADC_REGULAR_RANK_1, ADC_SAMPLETIME_12CYCLES_5);
    adc_channel_set(&g_adc_handle, ADC_AMPV_CHX, ADC_REGULAR_RANK_2, ADC_SAMPLETIME_12CYCLES_5);
    adc_channel_set(&g_adc_handle, ADC_AMPW_CHX, ADC_REGULAR_RANK_3, ADC_SAMPLETIME_12CYCLES_5);
	adc_channel_set(&g_adc_handle, ADC_VBUS_CHX, ADC_REGULAR_RANK_4, ADC_SAMPLETIME_12CYCLES_5);
    adc_channel_set(&g_adc_handle, ADC_VTEMP_CHX,ADC_REGULAR_RANK_5, ADC_SAMPLETIME_12CYCLES_5);
    
    __HAL_LINKDMA(&g_adc_handle, DMA_Handle,g_dma_adc_handle);
    
    HAL_NVIC_SetPriority(ADC_ADCX_DMASx_IRQn, 2, 3);                        /* 设置DMA中断优先级为3，子优先级为3 */
    HAL_NVIC_EnableIRQ(ADC_ADCX_DMASx_IRQn);                                /* 使能DMA中断 */
}

/**
 * @brief       ADC DMA采集中断服务函数
 * @param       无
 * @retval      无
 */
void ADC_ADCX_DMASx_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&g_dma_adc_handle);
}

/**
 * @brief       ADC转换完成的回调函数
 * @param       无
 * @retval      无
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) 
    { 
		if(!(current->flag & DEVICE_FLAG_CALIB))					/* haven't calibrate */
		{
			HAL_ADC_Stop_DMA(&g_adc_handle);                      	/* Stop DMA at first */
			uint8_t i,j;
			for(i = 0; i < ADC_DMA_BUF_SIZE; i++)
			{
				for(j = 0; j < 3; j++)
				{
					current->bias[j] += g_adc_dma_buf[i*5 + j];
				}
			}
			/* get average values */
			for(j = 0; j < 3; j++)
			{
				current->bias[j] /= ADC_DMA_BUF_SIZE;
			}
			current->flag |= DEVICE_FLAG_CALIB;						/* finish current sense calibration */
		}
		else
		{
			ltm_motor_run();			/* start motor control*/
			/* trigger FOC calculation  thread */
		}
    }
}


/*
    Rt = Rp *exp(B*(1/T1-1/T2))
    Rt 是热敏电阻在T1温度下的阻值；
    Rp是热敏电阻在T2常温下的标称阻值；
    exp是e的n次方，e是自然常数，就是自然对数的底数，近似等于 2.7182818；
    B值是热敏电阻的重要参数，教程中用到的热敏电阻B值为3380；
    这里T1和T2指的是开尔文温度，T2是常温25℃，即(273.15+25)K
    T1就是所求的温度
*/

const float Rp = 10000.0f;                  /* 10K */
const float T2 = (273.15f + 25.0f);         /* T2 */
const float Bx = 3380.0f;                   /* B */
const float Ka = 273.15f;

/**
 * @brief       计算温度值
 * @param       para: 温度采集对应ADC通道的值（已滤波）
 * @note        计算温度分为两步：
                1.根据ADC采集到的值计算当前对应的Rt
                2.根据Rt计算对应的温度值
 * @retval      温度值
 */
float get_temp(uint16_t para)
{
    float Rt;
    float temp;
    Rt = 3.3f * 4700.0f / (para * 3.3f / 4096.0f) - 4700.0f;
    /* like this R=5000, T2=273.15+25,B=3470, RT=5000*EXP(3470*(1/T1-1/(273.15+25)) */
    temp = Rt / Rp;
    temp = log(temp);       /* ln(Rt/Rp) */
    temp /= Bx;             /* ln(Rt/Rp)/B */
    temp += (1.0f / T2);
    temp = 1.0f / (temp);
    temp -= Ka;
    return temp;
}
/********************************************************************************************/
/* current device operators' hardware implementation */
static struct lt_current_ops ops;

void drv_current_init(void)
{	
	/* create current sense device */
	if(current != NULL) return;			/* avoid repeated initialization */
	current = lt_current_create("current");
	if(current == NULL) return;
	/* config current device */
	struct lt_current_config config;
	config.amp_gain = 6;
	config.bit_num = 12;
	config.vbus_gain = 1.0f/25;
	config.get_temp = get_temp;
	config.resistor = 0.020;		/* 20 mOhm */
	config.ops = &ops;
	config.type = CURRENT_TYPE_3;
	
	lt_current_set(current,&config);
}


static void _adc_calibrate(lt_current_t current)
{
	adc_dma_init(DEVICE_FLAG_CALIBING);
	HAL_ADC_Start_DMA(&g_adc_handle, (uint32_t *)&g_adc_dma_buf, ADC_SUM); 		/* Start DMA conversion */
}

static void _adc_start(lt_current_t current)
{
	if(current->flag & DEVICE_FLAG_CALIBING)	/* first calibrate successfully, we need to init adc again */
	{
		adc_dma_init(DEVICE_FLAG_CALIB);
		current->flag = CLEAR_BITS(current->flag, DEVICE_FLAG_CALIBING);		/* clear clibing flag */
	}
	HAL_ADC_Start_DMA(&g_adc_handle, (uint32_t *)&(current->vals), ADC_CH_NUM); /* Start DMA conversion */
}

static void _adc_stop(lt_current_t current)
{
	HAL_ADC_Stop_DMA(&g_adc_handle);
}

static struct lt_current_ops ops = {	
										_adc_calibrate,
										_adc_start,
										_adc_stop,
};


