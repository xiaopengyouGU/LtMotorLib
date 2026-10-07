#ifndef DRIVER_TASK_H
#define DRIVER_TASK_H

/* 底层驱动任务实现 */
typedef enum{
    ADC_Scan_Callback = 0,         /* ADC 扫描完成回调：默认 20kHz */
    SysTick_Callback,              /* 系统时基回调：1000Hz */
}driver_call_t;

void driver_task_init(void);        /* 驱动任务初始化 */
void driver_set_callback(driver_call_t type, void(*callback)(void));    /* 设置驱动回调函数 */
void driver_enable(void);           /* FOC 驱动使能 */
void driver_disable(void);          /* FOC 驱动失能 */
#endif
