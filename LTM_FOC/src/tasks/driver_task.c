#include "tasks/driver_task.h"
#include "common/tasks_param_def.h"

/* 硬件驱动初始化完毕 */
void driver_task_init(void)
{
	ltm_hal_init();							/* 底层外设统一初始化（系统/延时/串口/CAN/LED/PWM/编码器/ADC） */
	
	ltm_delay_ms(50);						/* 等待电容充电 */
	ltm_pwm_start();
	ltm_pwm_set_dutys(0, 0, 0);	
	ltm_delay_ms(200);						/* 等待电流尖峰消除 */
	ltm_adc_calibrate_zero(5000);			/* 零点校准 */
	ltm_delay_ms(500);						/* 等待校准完毕 */
	ltm_pwm_set_dutys(0, 0, 0);				
	ltm_pwm_stop();							/* 关闭 PWM输出 */

	/* 编码器上电位置解算（多圈联合解圈）*/
	for (int i = 0; i < 60; i++) {
		ltm_enc_update();					/* 编码器上电位置解算 */
		ltm_delay_ms(1);
	}
}


void driver_set_callback(driver_call_t type, void(*callback)(void))    /* 设置驱动回调函数 */
{
    if (!callback) 	return;    		/* 判空 */

    if (type == ADC_Scan_Callback){         
        ltm_adc_set_callback(callback);
    } else {
		ltm_sys_set_callback(callback);	
	}
}

void driver_enable(void)           	/* FOC 驱动使能 */
{
    ltm_pwm_start();
}

void driver_disable(void)          	/* FOC 驱动失能 */
{
    ltm_pwm_stop();
}
