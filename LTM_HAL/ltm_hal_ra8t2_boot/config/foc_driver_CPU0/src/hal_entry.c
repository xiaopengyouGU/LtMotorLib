#include "hal_data.h"

FSP_CPP_HEADER
void R_BSP_WarmStart(bsp_warm_start_event_t event);
FSP_CPP_FOOTER

#include <string.h>
#include <stdbool.h>
#include <math.h>
/* 系统层头文件 */
#include "system/uart/uart.h"
#include "system/sys/sys.h"
#include "system/delay/delay.h"

/* 外设层头文件 */
#include "bsp/led/led.h"
#include "bsp/tim/tim.h"
#include "bsp/adc/adc.h"
#include "bsp/encoder/encoder.h"
#include "bsp/canfd/canfd.h"

/* LTM 通讯协议 */
#include "protocol/ltm_commut.h"		

float   Vq_target = 0; 
uint8_t run_flag = 0;

/* 用户自定义通讯处理函数 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len);
void user_canfd_rxcall(void);				/* CAN-FD 接收回调 */

static uint8_t user_data[128] = {0};		/* 用户数据 */

void hal_entry(void)
{
   /* TODO: add your own code here */
	system_init();						/* 系统初始化 */
	delay_init();						/* 延时初始化 */
	uart_init();						/* 调试串口初始化 */
	canfd_init();
    led_init(); 						/* LED 初始化 */
	
	bool res = false;
	uint8_t  data_type;					/* 串口接收数据类型 */
	uint16_t data_len;					/* 串口接收数据长度 */
	uint32_t count = 0;
	uint8_t  tx_buf[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
	ltm_curves curves;					/* 曲线对象 */
    
	led_set(LED_STOP, 1);
	led_set(LED_RUN, 1);
	canfd_set_rxcall(user_canfd_rxcall);/* 设置 CAN-FD 接收回调函数 */
		
    ltm_commut_init();					/* 初始化 LTM_Monitor 协议对象 */
	ltm_commut_set_send(uart_write);	/* 设置 LTM 底层发送接口 */
	uart_set_rxcall(ltm_commut_recv);	/* 串口接收中断回调函数 绑定 LTM recv */
	
	ltm_commut_printf("Lvtou!!! \n");

	while(1)
    {
		res = ltm_commut_process(&data_type, user_data, &data_len);	/* 解析 LTM_Monitor 通讯协议 */
		if(res)	user_func(data_type, user_data, data_len);			/* 接收到数据后，进行用户自定义操作 */
				
		curves.size = 3;
		float angle = (count % 3000) * 6.283f / 3000.0f;
		curves.values[0] = sinf(angle);
		curves.values[1] = cosf(angle);
		curves.values[2] = angle;
		/* 批量发送曲线，效率更高 */
		if(run_flag)	ltm_commut_send_curves(&curves);
		
		if(count % 1000 == 0){
			canfd_send2(0x001, tx_buf, 8);		/* 2s 左右发送一个 CAN 帧 */
		}
		
		if(count % 3500 == 0){
			canfd_send2(0x002, tx_buf, 8);
			ltm_commut_printf("count = %d \n", count);
		}
		delay_ms(2);
		count++;
   }
    
#if BSP_TZ_SECURE_BUILD
    /* Enter non-secure code */
    R_BSP_NonSecureEnter();
#endif
}

/* 用户自定义接收例程 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len)
{
	switch(data_type)
	{
		case Data_Target:						//目标值 
		{
			float value = 0;
			memcpy(&value, buf, len);
			Vq_target = value * 0.01f;
			//修改目标值
    		ltm_commut_printf("Target : %.2f \n", value);
			break;
		}
		case Data_CMD_Set_PID:					//接收到PID参数
		{
			float pid[3] = {0};					//0 : Kp, 1 : Ki, 2 : Kd
			memcpy(&pid, buf, len);
    		ltm_commut_printf("Get PID params \n");
			break;
		}
		case Data_CMD_Set_Period:				//接收控制周期
		{
			float period = 0;
			memcpy(&period, buf, len);
			//修改控制周期
			ltm_commut_printf("Get period : %.2f ms\n", period);	//得强制类型转换，否则显示会有误
			break;
		}
		case Data_CMD_Text:						//接收到文本指令
		{										//回复 : 接收到的指令
			ltm_commut_send(data_type, buf, len);
			break;
		}
		case Data_CMD_Start:
		{
			ltm_commut_send(Data_CMD_Start, &data_type, 1);
			run_flag = 1;
			break;
		}
		case Data_CMD_Reset:
		{
			//重置PID
			run_flag = 0;
			Vq_target = 0;
			ltm_commut_send(Data_CMD_Stop, &data_type, 1);
			break;
		}
		case Data_CMD_Stop:
		{
			run_flag = 0;
			Vq_target = 0;
			ltm_commut_send(Data_CMD_Stop, &data_type, 1);
			break;
		}
		default: break;
	}
}

void user_canfd_rxcall(void){				/* CAN-FD 收发回显例程 */
	static uint8_t rx_buf[64] = {0};
	uint16_t id = 0;
	uint16_t len = 0;
	canfd_recv(&id, rx_buf, &len);
	canfd_send2(id, rx_buf, len); 
}
