#ifndef __LTM_DRIVER_H__
#define __LTM_DRIVER_H__

#include "stm32g4xx.h"
#include "core_cm4.h"
#include "stm32g4xx_hal.h"

/******************************************************************************************/
void ltm_driver_init(void);							/* init all hardware drivers */
void ltm_motor_run(void);							/* motor operation function */
/* driver init functions */
void drv_system_init(void);							/* system init function */
void drv_current_init(void);						/* current sense driver init function */
void drv_timer_init(void);							/* hardware timer driver function */
void drv_driver_init(void);							/* PWM driver init function */
void drv_sensor_init(void);							/* position sensor driver init function */
void drv_commut_init(void);							/* communicator init function */
void drv_debug_init(void);							/* debug init : key and led */
void drv_motor_init(void);							/* this is final init function, and must be inited finally!!! */
/* delay functions */
void lt_delay_ms(uint32_t ms);
void lt_delay_ns(uint32_t us);
/* hardware specified delay functions */
void HAL_Delay(uint32_t Delay);            			/* delay function of HAL Library */
/* functions for debug */
void lt_led_on(uint8_t num);
void lt_led_off(uint8_t num);
void lt_led_toggle(uint8_t num);
uint8_t lt_key_scan(uint8_t mode);
/**********************************/

/* current sense driver hardware implementation */
/* code below are taken from 正点原子 */
/******************************************************************************************/
/* ADC及引脚 定义 */
#define ADC_VBUS_CHX_GPIO_PORT              GPIOA                                               /* 电源电压采集引脚 */
#define ADC_VBUS_CHX_GPIO_PIN               GPIO_PIN_3
#define ADC_VBUS_CHX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)

#define ADC_VTEMP_CHX_GPIO_PORT             GPIOC                                               /* 温度采集引脚 */
#define ADC_VTEMP_CHX1_GPIO_PIN             GPIO_PIN_0
#define ADC_VTEMP_CHX_GPIO_CLK_ENABLE()     do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define ADC_AMPU_CHX_GPIO_PORT              GPIOC                                               /* U相采集引脚 */
#define ADC_AMPU_CHX_GPIO_PIN               GPIO_PIN_1
#define ADC_AMPU_CHX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define ADC_AMPV_CHX_GPIO_PORT              GPIOC                                               /* V相采集引脚 */
#define ADC_AMPV_CHX_GPIO_PIN               GPIO_PIN_2
#define ADC_AMPV_CHX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)

#define ADC_AMPW_CHX_GPIO_PORT              GPIOC                                               /* W相采集引脚 */
#define ADC_AMPW_CHX_GPIO_PIN               GPIO_PIN_3
#define ADC_AMPW_CHX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)


#define ADC_ADCX                            ADC1 
#define ADC_VBUS_CHX                        ADC_CHANNEL_4                                       /* 电源电压采集ADC通道 */ 
#define ADC_VTEMP_CHX                       ADC_CHANNEL_6                                       /* 驱动板温度采集ADC通道 */ 
#define ADC_AMPU_CHX                        ADC_CHANNEL_7                                       /* U相电流采集ADC通道 */ 
#define ADC_AMPV_CHX                        ADC_CHANNEL_8                                       /* V相电流采集ADC通道 */ 
#define ADC_AMPW_CHX                        ADC_CHANNEL_9                                       /* W相电流采集ADC通道 */ 
#define ADC_ADCX_CHY_CLK_ENABLE()           do{ __HAL_RCC_ADC12_CLK_ENABLE(); }while(0)         /* ADC1 时钟使能 */

#define ADC_DMA_BUF_SIZE        50                                  /* 每个ADC通道的 DMA采集次数 */
#define ADC_CH_NUM              5                                   /* ADC 通道数量 */
#define ADC_SUM                 ADC_CH_NUM * ADC_DMA_BUF_SIZE       /* 总采集次数 */
/* ADC单通道/多通道 DMA采集 DMA通道相关定义
 * 注意: 这里我们的通道还是使用上面的定义.
 */
#define ADC_ADCX_DMASx                      DMA1_Channel1
#define ADC_ADCX_DMASx_REQUEST              DMA_REQUEST_ADC1        /* ADC1_DMA请求源 */
#define ADC_ADCX_DMASx_IRQn                 DMA1_Channel1_IRQn
#define ADC_ADCX_DMASx_IRQHandler           DMA1_Channel1_IRQHandler


extern uint16_t g_adc_dma_buf[ADC_SUM];                             /* ADC DMA 原始ADC数值 */
extern uint16_t g_adc_val[ADC_CH_NUM];                              /* 滤波后的ADC数值缓存数组  */
extern ADC_HandleTypeDef g_adc_handle;                              /* ADC句柄 */

/******************************************************************************************/

/* communicator hardware implementation */
/*******************************************************************************************************/
/* 引脚 和 串口 定义 
 * 默认是针对USART1的.
 * 注意: 通过修改这12个宏定义,可以支持USART1~UART7任意一个串口.
 */

#define USART_TX_GPIO_PORT              GPIOB
#define USART_TX_GPIO_PIN               GPIO_PIN_6
#define USART_TX_GPIO_AF                GPIO_AF7_USART1
#define USART_TX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* 发送引脚时钟使能 */

#define USART_RX_GPIO_PORT              GPIOB
#define USART_RX_GPIO_PIN               GPIO_PIN_7
#define USART_RX_GPIO_AF                GPIO_AF7_USART1
#define USART_RX_GPIO_CLK_ENABLE()      do{ __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)   /* 接收引脚时钟使能 */

#define USART_UX                        USART1
#define USART_UX_IRQn                   USART1_IRQn
#define USART_UX_IRQHandler             USART1_IRQHandler
#define USART_UX_CLK_ENABLE()           do{ __HAL_RCC_USART1_CLK_ENABLE(); }while(0)  /* USART1 时钟使能 */

/*******************************************************************************************************/

#define USART_REC_LEN   200                     /* 定义最大接收字节数 200 */
#define USART_EN_RX     1                       /* 使能（1）/禁止（0）串口1接收 */
#define RXBUFFERSIZE    1                       /* 缓存大小 */
/******************************************************************************************/

/* timer driver hardware implementation */
/******************************************************************************************/
/* 高级定时器 定义 */

/* TIM_CHx通道 上桥臂IO定义 */
#define ATIM_TIMX_PWM_CH1_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH1_GPIO_PIN             GPIO_PIN_8
#define ATIM_TIMX_PWM_CH1_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define ATIM_TIMX_PWM_CH2_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH2_GPIO_PIN             GPIO_PIN_9
#define ATIM_TIMX_PWM_CH2_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

#define ATIM_TIMX_PWM_CH3_GPIO_PORT            GPIOA
#define ATIM_TIMX_PWM_CH3_GPIO_PIN             GPIO_PIN_10
#define ATIM_TIMX_PWM_CH3_GPIO_CLK_ENABLE()    do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)     /* PA口时钟使能 */

/* 下桥臂IO定义 */
#define M1_LOW_SIDE_U_PORT                      GPIOB
#define M1_LOW_SIDE_U_PIN                       GPIO_PIN_13
#define M1_LOW_SIDE_U_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define M1_LOW_SIDE_V_PORT                      GPIOB
#define M1_LOW_SIDE_V_PIN                       GPIO_PIN_14
#define M1_LOW_SIDE_V_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

#define M1_LOW_SIDE_W_PORT                      GPIOB
#define M1_LOW_SIDE_W_PIN                       GPIO_PIN_15
#define M1_LOW_SIDE_W_GPIO_CLK_ENABLE()         do{  __HAL_RCC_GPIOB_CLK_ENABLE(); }while(0)    /* PB口时钟使能 */

/* 复用到TIM1 */
#define ATIM_TIMX_PWM_CHY_GPIO_AF              GPIO_AF6_TIM1

#define ATIM_TIMX_PWM                          TIM1
#define ATIM_TIMX_PWM_IRQn                     TIM1_UP_TIM16_IRQn
#define ATIM_TIMX_PWM_IRQHandler               TIM1_UP_TIM16_IRQHandler
#define ATIM_TIMX_PWM_CH1                      TIM_CHANNEL_1                                    /* 通道1 */
#define ATIM_TIMX_PWM_CH2                      TIM_CHANNEL_2                                    /* 通道2 */
#define ATIM_TIMX_PWM_CH3                      TIM_CHANNEL_3                                    /* 通道3 */
#define ATIM_TIMX_PWM_CHY_CLK_ENABLE()         do{ __HAL_RCC_TIM1_CLK_ENABLE(); }while(0)       /* TIM1 时钟使能 */


extern TIM_HandleTypeDef g_atimx_handle;                                                        /* 定时器x句柄 */

/******************************************************************************************/
/* 基本定时器 定义 */

/* TIMX 中断定义 
 * 默认是针对TIM6/TIM7
 * 注意: 通过修改这4个宏定义,可以支持TIM1~TIM8任意一个定时器.
 */
 
#define BTIM_TIMX_INT                       TIM6
#define BTIM_TIMX_INT_IRQn                  TIM6_DAC_IRQn
#define BTIM_TIMX_INT_IRQHandler            TIM6_DAC_IRQHandler
#define BTIM_TIMX_INT_CLK_ENABLE()          do{ __HAL_RCC_TIM6_CLK_ENABLE(); }while(0)          /* TIM6 时钟使能 */

/******************************************************************************************/

/* PWM driver hardware part implementation */
/***************************************** 半桥芯片的刹车引脚 ***************************************************/

#define SHUTDOWN_PIN                        GPIO_PIN_13                                                         /* PC13 */
#define SHUTDOWN_PIN_GPIO                   GPIOC
#define SHUTDOWN_PIN_GPIO_CLK_ENABLE()      do{  __HAL_RCC_GPIOC_CLK_ENABLE(); }while(0)                        /* PC口时钟使能 */
/***************************************** 霍尔传感器接口 *************************************************/

#define HALL1_TIM_CH1_PIN                   GPIO_PIN_0      /* U */
#define HALL1_TIM_CH1_GPIO                  GPIOA
#define HALL1_U_GPIO_CLK_ENABLE()           do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)    /* PA口时钟使能 */

#define HALL1_TIM_CH2_PIN                   GPIO_PIN_1      /* V */
#define HALL1_TIM_CH2_GPIO                  GPIOA
#define HALL1_V_GPIO_CLK_ENABLE()           do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)    /* PA口时钟使能 */

#define HALL1_TIM_CH3_PIN                   GPIO_PIN_2      /* W */
#define HALL1_TIM_CH3_GPIO                  GPIOA
#define HALL1_W_GPIO_CLK_ENABLE()           do{  __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)    /* PA口时钟使能 */

/*************************************************************************************************************/
/* hardware implementation for debug */
/******************************************************************************************/
/* 引脚 定义 */
#define LED0_GPIO_PORT                  GPIOE
#define LED0_GPIO_PIN                   GPIO_PIN_0
#define LED0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)             /* PE口时钟使能 */

#define LED1_GPIO_PORT                  GPIOE
#define LED1_GPIO_PIN                   GPIO_PIN_1
#define LED1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)             /* PE口时钟使能 */

#define LED_NUM_0						0x00
#define LED_NUM_1						0x01
/******************************************************************************************/
/******************************************************************************************/
/* 引脚 定义 */

#define KEY0_GPIO_PORT                  GPIOE
#define KEY0_GPIO_PIN                   GPIO_PIN_12
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define KEY1_GPIO_PORT                  GPIOE
#define KEY1_GPIO_PIN                   GPIO_PIN_13
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define KEY2_GPIO_PORT                  GPIOE
#define KEY2_GPIO_PIN                   GPIO_PIN_14
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define KEY_NUM_0						0x00
#define KEY_NUM_1						0x01
#define KEY_NUM_2						0x02
/******************************************************************************************/

#endif
