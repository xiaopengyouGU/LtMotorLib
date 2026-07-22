// #include "simulate/pmsm/pmsm.h"
// #include "simulate/pmsm/pmsm_thread.h"
// #include "math/basic/lt_math.h"
// #include <windows.h>
// #include <stdio.h>

// /*==============================================================================
//  * 静态变量
//  *==============================================================================*/
// static pmsm_t s_motor;                              /* 电机对象 */
// static HANDLE s_hThread = NULL;                     /* 电机线程句柄 */
// static HANDLE s_hStartEvent = NULL;                 /* 启动事件 */
// static HANDLE s_hDataReadyEvent = NULL;             /* 数据就绪事件 */
// static volatile uint8_t s_running = 0;              /* 运行标志 */

// /* 高精度计时器 */
// static LARGE_INTEGER s_freq;
// static LARGE_INTEGER s_last_time;

// /* 中断回调函数指针 */
// static void (*s_irq_current_callback)(void) = NULL;
// static void (*s_irq_speed_callback)(void) = NULL;
// static void (*s_irq_position_callback)(void) = NULL;

// /*==============================================================================
//  * 内部函数
//  *==============================================================================*/

// /* 获取时间差（微秒），并重置计时器 */
// static float get_elapsed_us(void)
// {
//     LARGE_INTEGER now;
//     QueryPerformanceCounter(&now);
//     float elapsed_us = (float)(now.QuadPart - s_last_time.QuadPart) * 1000000.0f / s_freq.QuadPart;
//     s_last_time = now;
//     return elapsed_us;
// }

// /* 虚拟电机线程主函数 */
// static DWORD WINAPI motor_thread_proc(LPVOID arg)
// {
//     (void)arg;
//     uint32_t steps;
//     float elapsed_us;
    
//     /* 初始化高精度计时器 */
//     QueryPerformanceFrequency(&s_freq);
//     QueryPerformanceCounter(&s_last_time);
    
//     while (s_running) {
//         /* 等待主线程发信号启动计算 */
//         WaitForSingleObject(s_hStartEvent, INFINITE);
        
//         /* 计算时间差，得到步数 */
//         elapsed_us = get_elapsed_us();
//         steps = (uint32_t)(elapsed_us / (s_motor.dt * 1000000.0f));
        
//         /* 限制最大步数，防止异常 */
//         if (steps > 1000000) {
//             steps = 1000000;
//         }
        
//         /* 推进模型 */
//         for (uint32_t i = 0; i < steps; i++) {
//             pmsm_step(&s_motor);
//         }
        
//         /* 通知主线程数据已更新 */
//         SetEvent(s_hDataReadyEvent);
//     }
    
//     return 0;
// }

// /*==============================================================================
//  * 公共接口
//  *==============================================================================*/

// /* 初始化虚拟电机线程 */
// int pmsm_thread_init(pmsm_param_t *param, float dt)
// {
//     /* 初始化电机模型 */
//     pmsm_init(&s_motor, param, dt);
    
//     /* 创建事件 */
//     s_hStartEvent = CreateEvent(NULL, FALSE, FALSE, NULL);      /* 自动复位事件 */
//     s_hDataReadyEvent = CreateEvent(NULL, FALSE, FALSE, NULL);  /* 自动复位事件 */
    
//     if (!s_hStartEvent || !s_hDataReadyEvent) {
//         printf("pmsm_thread_init: 创建事件失败\n");
//         return -1;
//     }
    
//     /* 启动线程 */
//     s_running = 1;
//     s_hThread = CreateThread(NULL, 2048, motor_thread_proc, NULL, 0, NULL);
    
//     if (!s_hThread) {
//         printf("pmsm_thread_init: 创建线程失败\n");
//         s_running = 0;
//         CloseHandle(s_hStartEvent);
//         CloseHandle(s_hDataReadyEvent);
//         return -1;
//     }
    
//     printf("pmsm_thread_init: 虚拟电机线程已启动, dt = %.3f us\n", dt * 1000000.0f);
//     return 0;
// }

// /* 反初始化 */
// void pmsm_thread_deinit(void)
// {
//     if (s_hThread) {
//         s_running = 0;
//         SetEvent(s_hStartEvent);                    /* 唤醒线程，让它退出 */
//         WaitForSingleObject(s_hThread, 1000);       /* 等待线程退出 */
//         CloseHandle(s_hThread);
//         s_hThread = NULL;
//     }
    
//     if (s_hStartEvent) {
//         CloseHandle(s_hStartEvent);
//         s_hStartEvent = NULL;
//     }
    
//     if (s_hDataReadyEvent) {
//         CloseHandle(s_hDataReadyEvent);
//         s_hDataReadyEvent = NULL;
//     }
    
//     printf("pmsm_thread_deinit: 虚拟电机线程已停止\n");
// }

// /* 触发虚拟电机计算（由delay_ms调用） */
// void pmsm_thread_trigger(void)
// {
//     if (s_running && s_hStartEvent) {
//         SetEvent(s_hStartEvent);
//     }
// }

// /* 等待电机计算完成（由delay_ms调用） */
// void pmsm_thread_wait(void)
// {
//     if (s_running && s_hDataReadyEvent) {
//         WaitForSingleObject(s_hDataReadyEvent, INFINITE);
//     }
// }

// /* 设置电流环中断回调 */
// void pmsm_set_current_irq(void (*callback)(void))
// {
//     if (!callback) return;
//     s_irq_current_callback = callback;
// }

// /* 设置速度环中断回调 */
// void pmsm_set_speed_irq(void (*callback)(void))
// {
//     if (!callback) return;
//     s_irq_speed_callback = callback;
// }

// /* 设置位置环中断回调 */
// void pmsm_set_position_irq(void (*callback)(void))
// {
//     if (!callback) return;
//     s_irq_position_callback = callback;
// }

// /*==============================================================================
//  * 对外获取接口（内联函数已在头文件实现）
//  *==============================================================================*/

// /* 获取三相电流 */
// void pmsm_thread_get_current(pmsm_current_t *current)
// {
//     pmsm_get_current(&s_motor, current);
// }

// /* 获取编码器计数值 */
// int32_t pmsm_thread_get_count(void)
// {
//     return pmsm_get_count(&s_motor);
// }

// /* 获取转子位置（弧度） */
// float pmsm_thread_get_position(void)
// {
//     return pmsm_get_position(&s_motor);
// }

// /* 获取转子速度（rad/s） */
// float pmsm_thread_get_speed(void)
// {
//     return pmsm_get_speed(&s_motor);
// }

// /* 获取电磁转矩（N·m） */
// float pmsm_thread_get_torque(void)
// {
//     return pmsm_get_torque(&s_motor);
// }

// /* 获取D轴电流 */
// float pmsm_thread_get_id(void)
// {
//     return pmsm_get_id(&s_motor);
// }

// /* 获取Q轴电流 */
// float pmsm_thread_get_iq(void)
// {
//     return pmsm_get_iq(&s_motor);
// }

// /* 设置三相电压 */
// void pmsm_thread_set_voltage(float va, float vb, float vc)
// {
//     pmsm_set_voltage(&s_motor, va, vb, vc);
// }

// /* 设置负载转矩 */
// void pmsm_thread_set_load(float torque)
// {
//     pmsm_set_load(&s_motor, torque);
// }

// /* 获取当前仿真步数 */
// uint64_t pmsm_thread_get_step_count(void)
// {
//     return pmsm_get_step_count(&s_motor);
// }

// /* 获取当前仿真物理时间（秒） */
// float pmsm_thread_get_time(void)
// {
//     return pmsm_get_time(&s_motor);
// }