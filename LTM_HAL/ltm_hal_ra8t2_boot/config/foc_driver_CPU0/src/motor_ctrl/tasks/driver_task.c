#include "motor_ctrl/tasks/driver_task.h"

/* 系统层头文件 */
#include "system/uart/uart.h"
#include "system/sys/sys.h"
#include "system/delay/delay.h"

/* 外设层头文件 */
#include "bsp/led/led.h"
#include "bsp/tim/tim.h"
#include "bsp/adc/adc.h"
#include "bsp/encoder/encoder.h"

/* LTM 通讯协议配置 */
#include "protocol/ltm_commut.h"

/* 硬件驱动初始化完毕 */
void driver_task_init(void)
{
	system_init();						/* 系统初始化 */
	delay_init();						/* 延时初始化 */
	uart_init();						/* 调试串口初始化 */
    led_init(); 						/* LED 初始化 */
	
	foc_aux_timer_init();        		/* 速度环定时器启动 */
	foc_pwm_init();						/* PWM 输出启动 */
	foc_pwm_start();
	foc_pwm_set_duty(0.0f, 0.0f, 0.0f);	
	
	encoder_init();						/* 编码器启动 */
	// encoder_start_calibration();		/* 启动编码器 Z 相校准 */
	
	adc_init();							/* ADC 初始化 */
	adc_calibrate_zero(15000);			/* 零点校准 */

    ltm_commut_init();					/* 初始化 LTM_Monitor 协议对象 */
	ltm_commut_set_send(uart_write);	/* 设置 LTM 底层发送接口 */
	uart_set_rxcall(ltm_commut_recv);	/* 串口接收中断回调函数 绑定 LTM recv */
}

void driver_set_callback(driver_call_t type, void(*callback)(void))    /* 设置驱动回调函数 */
{
    if(!callback)           return;     /* 判空 */

    if(type == ADC_Scan_Callback){              /* 20kHz 回调 */
        adc_set_current_callback(callback);
    }else if(type == TIM_Speed_Callback){       /* 3kHz 回调 */
        foc_set_speed_callback(callback);
    }else if(type == TIM_Position_Callback){    /* 1kHz 回调 */
        foc_set_pos_callback(callback);
    }
}

void driver_enable(void)           /* FOC 驱动使能 */
{
    foc_pwm_start();
}

void driver_disable(void)          /* FOC 驱动失能 */
{
    foc_pwm_stop();
}