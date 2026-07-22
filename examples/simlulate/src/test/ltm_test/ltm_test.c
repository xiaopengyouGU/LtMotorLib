#include <stdio.h>

#include "ltm_test/ltm_test.h"

#include "system/uart/uart.h"
#include "system/delay/delay.h"
#include "protocol/ltm_commut.h"

static uint8_t run_flag = 0;                /* 运行标志位 */
static uint8_t user_data[128] = {0};		/* 用户数据 */
/* 用户自定义通讯处理函数 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len);

int ltm_test(void)
{
    uint8_t len;
    uint16_t times = 0;
    
    uart_init();                            /* 串口初始化 */

	ltm_commut_init();						/* 初始化LTM_通讯协议 */
	ltm_commut_set_send(uart_write);		/* 设置底层发送接口 */
	uart_set_rxcall(ltm_commut_recv);		/* 设置串口接收回调函数 */	
    uart_write("Gododa", 6);
	uint8_t data_type;						/* 数据类型 */
	uint16_t data_len;						/* 数据长度 */
	bool res = false;						
	ltm_curves curves;						/* 动态数组发送，最多5条曲线 */
	curves.size = 3;						/* 发送三条曲线 */
	int count = 0;	
	
    while (1)
    {
		count++;
	    res = ltm_commut_process(&data_type, user_data, &data_len);	/* 解析 LTM_Monitor 通讯协议 */
	    if(res)	user_func(data_type, user_data, data_len);			/* 接收到数据后，进行用户自定义操作 */
	  
	    /* 演示发送动态曲线 */
	    float angle = (count % 400) / 400.0f * 6.28f;
	    curves.values[0] = sinf(angle);								/* 正弦值:      CH1 */
	    curves.values[1] = cosf(angle);								/* 余弦值:      CH2 */
	    curves.values[2] = angle;							        /* 角度值(RAD): CH3 */			
	    if(count % 200 == 0){
            ltm_commut_printf("count = %d \n", count);
            printf("count = %d \n", count);
        }
		if(run_flag){
            ltm_commut_send_curves(&curves);						 /* 批量发送曲线 */
        }
		delay_ms(40);
    }
}

/* 用户自定义通讯处理函数 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len)
{
	switch(data_type)
	{
		case Data_Target:						//目标值 
		{
			float value = 0;
			memcpy(&value, buf, len);
			//修改目标值
    		ltm_commut_printf("Target : %.2f \n", value);
			break;
		}
		case Data_CMD_Set_PID:					//接收到PID参数
		{
			float pid[3] = {0};					//0 : Kp, 1 : Ki, 2 : Kd
			memcpy(&pid, buf, len);
			//lt_pid_set(my_pid, pid[0], pid[1], pid[2]);
			//修改PID参数
    		ltm_commut_printf("Get PID params \n");
			break;
		}
		case Data_CMD_Set_Period:				//接收控制周期
		{
			float period = 0;
			memcpy(&period, buf, len);
			//lt_pid_set_ts(my_pid, period);
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
			ltm_commut_send(Data_Res_Start, &data_type, 1);
			run_flag = 1;
			break;
		}
		case Data_CMD_Reset:
		{
			//重置PID
			run_flag = 0;
			ltm_commut_send(Data_Res_Stop, &data_type, 1);
			//lt_pid_reset(my_pid);
			break;
		}
		case Data_CMD_Stop:
		{
			run_flag = 0;
			ltm_commut_send(Data_Res_Stop, &data_type, 1);
			break;
		}
		default: break;
	}
}