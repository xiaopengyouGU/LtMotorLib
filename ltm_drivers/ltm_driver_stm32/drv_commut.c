/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-12-7  	  Lvtou		   The first version
 */
#include "ltmotorlib.h"
/* communicator hardware implementation */
/* code below are taken from 正点原子 */
/**
 ****************************************************************************************************
 * @file        usart.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2023-08-01
 * @brief       串口初始化代码(一般是串口1)，支持printf
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
static void _recv_data(uint8_t * data);

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART1->ISR & 0X40) == 0);              /* 等待上一个字符发送完成 */

    USART1->TDR = (uint8_t)ch;                      /* 将要发送的字符 ch 写入到TDR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/
    
#if USART_EN_RX                                     /* 如果使能了接收 */

UART_HandleTypeDef g_uart1_handle;                  /* UART句柄 */


/**
 * @brief       串口X初始化函数
 * @param       baudrate: 波特率, 根据自己需要设置波特率值
 * @note        注意: 必须设置正确的时钟源, 否则串口波特率就会设置异常.
 *              这里的USART的时钟源在sys_stm32_clock_init()函数中已经设置过了.
 * @retval      无
 */
void usart_init(uint32_t baudrate)
{
    g_uart1_handle.Instance = USART_UX;                         /* USART1 */
    g_uart1_handle.Init.BaudRate = baudrate;                    /* 波特率 */
    g_uart1_handle.Init.WordLength = UART_WORDLENGTH_8B;        /* 字长为8位数据格式 */
    g_uart1_handle.Init.StopBits = UART_STOPBITS_1;             /* 一个停止位 */
    g_uart1_handle.Init.Parity = UART_PARITY_NONE;              /* 无奇偶校验位 */
    g_uart1_handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;        /* 无硬件流控 */
    g_uart1_handle.Init.Mode = UART_MODE_TX_RX;                 /* 收发模式 */
    HAL_UART_Init(&g_uart1_handle);                             /* HAL_UART_Init()会使能UART1 */
}

/**
 * @brief       UART底层初始化函数
 * @param       huart: UART句柄类型指针
 * @note        此函数会被HAL_UART_Init()调用
 *              完成时钟使能，引脚配置，中断配置
 * @retval      无
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init_struct;
    if(huart->Instance == USART_UX)                             /* 如果是串口1，进行串口1 MSP初始化 */
    {
        USART_UX_CLK_ENABLE();                                  /* USART1 时钟使能 */
        USART_TX_GPIO_CLK_ENABLE();                             /* 发送引脚时钟使能 */
        USART_RX_GPIO_CLK_ENABLE();                             /* 接收引脚时钟使能 */

        gpio_init_struct.Pin = USART_TX_GPIO_PIN;               /* TX引脚 */
        gpio_init_struct.Mode = GPIO_MODE_AF_PP;                /* 复用推挽输出 */
        gpio_init_struct.Pull = GPIO_PULLUP;                    /* 上拉 */
        gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
        gpio_init_struct.Alternate = USART_TX_GPIO_AF;          /* 复用为USART1 */
        HAL_GPIO_Init(USART_TX_GPIO_PORT, &gpio_init_struct);   /* 初始化发送引脚 */

        gpio_init_struct.Pin = USART_RX_GPIO_PIN;               /* RX引脚 */
        gpio_init_struct.Alternate = USART_RX_GPIO_AF;          /* 复用为USART1 */
        HAL_GPIO_Init(USART_RX_GPIO_PORT, &gpio_init_struct);   /* 初始化接收引脚 */
		__HAL_UART_DISABLE_IT(&g_uart1_handle, UART_IT_TC);     /* 关闭发送完成中断 */
#if USART_EN_RX
		 __HAL_UART_ENABLE_IT(&g_uart1_handle, UART_IT_RXNE);    /* 开启接收中断 */
        HAL_NVIC_EnableIRQ(USART_UX_IRQn);                      /* 使能USART1中断通道 */
        HAL_NVIC_SetPriority(USART_UX_IRQn, 3, 3);              /* 抢占优先级3，子优先级3 */
#endif
    }
}

/**
 * @brief       串口1中断服务函数
 * @param       无
 * @retval      无
 */
void USART_UX_IRQHandler(void)
{ 
    uint8_t data;

    if ((__HAL_UART_GET_FLAG(&g_uart1_handle, UART_FLAG_RXNE) != RESET)) /* 接收到数据 */
    {
        HAL_UART_Receive(&g_uart1_handle, &data, 1, 1000);
		_recv_data(&data);												/* receive data and save it to the recv_buf */
    } 
}

#endif
/********************************************************************************************/
/* communicator's implementation */
/* codes below are taken from 野火，some modifications are added */
/* 通道宏定义 */
#define CURVES_CH1      0x01
#define CURVES_CH2      0x02
#define CURVES_CH3      0x03
#define CURVES_CH4      0x04
#define CURVES_CH5      0x05

/* 指令(下位机 -> 上位机) */
#define SEND_TARGET_CMD      0x01     // 发送上位机通道的目标值
#define SEND_FACT_CMD        0x02     // 发送通道实际值
#define SEND_PID_CMD         0x03     // 发送 PID 值（同步上位机显示的值）
#define SEND_START_CMD       0x04     // 发送启动指令（同步上位机按钮状态）
#define SEND_STOP_CMD        0x05     // 发送停止指令（同步上位机按钮状态）
#define SEND_PERIOD_CMD      0x06     // 发送周期（同步上位机显示的值）

/* 指令(上位机 -> 下位机) */
#define SET_PID_CMD        	 0x10     // 设置 PID 值
#define SET_TARGET_CMD       0x11     // 设置目标值
#define START_CMD            0x12     // 启动指令
#define STOP_CMD             0x13     // 停止指令
#define RESET_CMD            0x14     // 复位指令
#define SET_PERIOD_CMD       0x15     // 设置周期

/* 空指令 */
#define CMD_NONE             0xFF     // 空指令


#define  DATA_SIZE		 23								/* 3 float (12 bytes) + frame header(10 bytes) + check sum (1 byte) */
#define  LOOP_REV_SIZE   128       						/* 环形缓冲区大小, 为单次接收最大数据的三倍  */
volatile uint8_t rev_num[LOOP_REV_SIZE];    			/* 存放接收数据的数组（环形缓冲区） */
volatile uint8_t rev_index = 0;                       	/* 地址偏移量 */
volatile uint8_t tmp_index = 0;							/* temporary index */
static void _send_data(lt_commut_t commut, uint8_t data_type);

static void _recv_data(uint8_t * data)
{
    rev_num[rev_index] = *(data);        				
	rev_index = (rev_index + 1) % LOOP_REV_SIZE;																				
}

/* 数据头结构体 */
typedef __packed struct
{
  uint32_t head;    // 包头
  uint8_t ch;       // 通道
  uint32_t len;     // 包长度
  uint8_t cmd;      // 命令
}packet_head_t;

typedef __packed struct
{
	float Kp;
	float Ki;
	float Kd;
}pid_param;

#define FRAME_HEADER     0x59485A53   // 帧头
/* 索引值宏定义 */
#define HEAD_INDEX_VAL       0x0u     // 包头索引值（4字节）
#define CHX_INDEX_VAL        0x4u     // 通道索引值（1字节）
#define LEN_INDEX_VAL        0x5u     // 包长索引值（4字节）
#define CMD_INDEX_VAL        0x9u     // 命令索引值（1字节)

#define COMPOUND_32BIT(data)        (((*(data-0) << 24) & 0xFF000000) |\
                                     ((*(data-1) << 16) & 0x00FF0000) |\
                                     ((*(data-2) <<  8) & 0x0000FF00) |\
                                     ((*(data-3) <<  0) & 0x000000FF))      // 合成为一个字,小端序

/**
  * @brief 计算校验和
  * @param ptr：需要计算的数据
  * @param len：需要计算的长度
  * @retval 校验和
  */
uint8_t check_sum(uint8_t init, uint8_t *ptr, uint8_t len )
{
  uint8_t sum = init;
  
  while(len--)
  {
    sum += *ptr;
    ptr++;
  }
  
  return sum;
}
/**********************************************************************************************/
/* inner functions */
#define HEAD_INDEX       0x00     // header  index（4 bytes）
#define CHX_INDEX        0x04     // channel index（1 byte）
#define LEN_INDEX        0x05     // length  index（4 bytes）
#define CMD_INDEX        0x09     // command index（1 bytes）
/* size : packet size; i : bias */
static uint8_t _get_byte(uint8_t size, uint8_t i)
{
	return rev_num[(tmp_index + LOOP_REV_SIZE - size + i) % LOOP_REV_SIZE];
}

static uint8_t _get_size(void)
{
    uint8_t i = 0;
	uint32_t header = 0;
	uint32_t length = 0;
	
    for (i = 0; i < DATA_SIZE - 3; i++)
    {
		header = 0;
		header |= ((uint32_t)_get_byte(DATA_SIZE,i)   << 0);
		header |= ((uint32_t)_get_byte(DATA_SIZE,i+1) << 8);
		header |= ((uint32_t)_get_byte(DATA_SIZE,i+2) << 16);
		header |= ((uint32_t)_get_byte(DATA_SIZE,i+3) << 24);
		
		if(header == FRAME_HEADER)	break;	/* check whether find the frame header! */
    }
	
	if(i == (DATA_SIZE - 3)) return 0;
	/* find frame header */
	/* get packet length */
	length |= ((uint32_t)_get_byte(DATA_SIZE,i+LEN_INDEX) << 0);
	length |= ((uint32_t)_get_byte(DATA_SIZE,i+LEN_INDEX+1) << 8);
	length |= ((uint32_t)_get_byte(DATA_SIZE,i+LEN_INDEX+2) << 16);
	length |= ((uint32_t)_get_byte(DATA_SIZE,i+LEN_INDEX+3) << 24);
	/* we get a valid frame */
	if(length < 11 ) return 0;
	
	return (uint8_t)length;
}

static void _send(uint8_t cmd,uint8_t channel,void*data,uint8_t num)
{
	uint8_t sum = 0;    // 校验和
	num *= 4;           // 一个参数 4 个字节
  
	static packet_head_t packet;
	  
	packet.head = FRAME_HEADER;    // 包头 0x59485A53
	packet.len  = 0x0B + num;      // 包长
	packet.ch   = channel;         // 设置通道
	packet.cmd  = cmd;             // 设置命令
	  
	sum = check_sum(0, (uint8_t *)&packet, sizeof(packet));       // 计算包头校验和
	sum = check_sum(sum, (uint8_t *)data, num);                   // 计算参数校验和
	/* send datas  */
	HAL_UART_Transmit(&g_uart1_handle, (uint8_t *)&packet, sizeof(packet), 0xFF);  	/* send frame header */
	HAL_UART_Transmit(&g_uart1_handle, (uint8_t *)data, num, 0xFF);   	 			/* send frame datas */
	HAL_UART_Transmit(&g_uart1_handle, (uint8_t *)&sum, sizeof(sum), 0xFF);   	 	/* send check sum */
}

void _send_data(lt_commut_t commut, uint8_t cmd_type)
{
	lt_pid_t pid = commut->pid;
	
	switch(cmd_type)
	{
		case CMD_SEND_CURVES:
		{
			for(uint8_t i = 0; i < 5; i++)
			{
				_send(SEND_FACT_CMD,i+1,&(commut->curves[i]),1);
			}
			break;
		}
		case CMD_SEND_START:
		{
			_send(SEND_START_CMD,CURVES_CH1,NULL,0);
			break;
		}
		case CMD_SEND_STOP:
		{
			_send(SEND_STOP_CMD,CURVES_CH1,NULL,0);
			break;
		}
		case CMD_SEND_TARGET:
		{
			int target = pid->target;
			_send(SEND_TARGET_CMD,CURVES_CH1,&target,1);
			break;
		}
		case CMD_SEND_PERIOD:
		{
			int period = pid->ts*1000;
			_send(SEND_PERIOD_CMD,CURVES_CH1,&period,1);
			break;
		}
		case CMD_SEND_PID:
		{
			pid_param param;
			param.Kp = pid->Kp;
			param.Ki = pid->Ki_ts/(pid->ts);
			param.Kd = pid->Kd_ts*(pid->ts);
			_send(SEND_PID_CMD,CURVES_CH1,&param,3);
		}
		default : break;
	}
}

void _process_data(lt_commut_t commut)
{
	if(tmp_index == rev_index) return;							 /* don't read datas */			
	tmp_index = rev_index;										 /* copy rev_index !!! */
	uint8_t size = _get_size();								 	 /* get valid frame size */
	if(size == 0) return;						 				 /* frame is unvalid */
	
    uint8_t num[DATA_SIZE];										 /* copy frame  */
	for(uint8_t i = 0; i < size; i++)
	{
		num[i] = _get_byte(size,i);
	}
    uint8_t cmd_type = num[CMD_INDEX];											
	lt_pid_t pid = commut->pid;
	lt_motor_t motor = commut->motor;
	
	switch(cmd_type)
	{
		case SET_PID_CMD:
		{
			uint32_t temp0 = COMPOUND_32BIT(&num[13]);
			uint32_t temp1 = COMPOUND_32BIT(&num[17]);
            uint32_t temp2 = COMPOUND_32BIT(&num[21]);
			float Kp = *(float *)&temp0;
			float Ki = *(float *)&temp1;
			float Kd = *(float *)&temp2;
			lt_pid_set(pid,Kp,Ki,Kd);    /* set pid */
			break;
		}
		case SET_TARGET_CMD:
		{
			int value = (int)COMPOUND_32BIT(&num[13]);
			lt_pid_set_target(pid,value);    /* set pid */		
			break;
		}
		case START_CMD:
		{
			lt_motor_start(motor);
			_send(SEND_START_CMD,CURVES_CH1,NULL,0);
			break;
		}
		case STOP_CMD:
		{
			lt_motor_stop(motor);
			_send(SEND_STOP_CMD,CURVES_CH1,NULL,0);
			break;
		}
		case RESET_CMD:
		{
			lt_pid_reset(pid);				 		 				/* reset pid! */
			lt_motor_stop(motor);									/* stop motor */
			_send(SEND_STOP_CMD,CURVES_CH1,NULL,0);
			break;
		}
		case SET_PERIOD_CMD:
		{
			int period = (int)COMPOUND_32BIT(&num[13]);
			lt_pid_set_ts(pid,period);
			break;
		}
		default:break;  
	}
}


/********************************************************************************************/
/* communicator hardware implementation */
static struct lt_commut_ops ops;

void drv_commut_init(void)
{
	usart_init(115200);
#ifdef LT_COMMUTE_ENABLE
	struct lt_commut_config config;
	config.name = "commut";
	config.ops = &ops;
	lt_commut_set(&config);
#endif
}

static struct lt_commut_ops ops = {
										_process_data,
										_send_data,
};

