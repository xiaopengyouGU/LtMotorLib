// #ifndef PMSM_THREAD_H
// #define PMSM_THREAD_H

// #include "simulate/pmsm/pmsm.h"
// #include <stdint.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// /*==============================================================================
//  * 初始化/反初始化
//  *==============================================================================*/
// int  pmsm_thread_init(pmsm_param_t *param, float dt);
// void pmsm_thread_deinit(void);

// /*==============================================================================
//  * 触发与等待（由delay_ms调用）
//  *==============================================================================*/
// void pmsm_thread_trigger(void);    /* 触发电机计算 */
// void pmsm_thread_wait(void);       /* 等待计算完成 */

// /*==============================================================================
//  * 中断回调注册
//  *==============================================================================*/
// void pmsm_set_current_irq(void (*callback)(void));
// void pmsm_set_speed_irq(void (*callback)(void));
// void pmsm_set_position_irq(void (*callback)(void));

// /*==============================================================================
//  * 状态获取
//  *==============================================================================*/
// void     pmsm_thread_get_current(pmsm_current_t *current);
// int32_t  pmsm_thread_get_count(void);
// float    pmsm_thread_get_position(void);
// float    pmsm_thread_get_speed(void);
// float    pmsm_thread_get_torque(void);
// float    pmsm_thread_get_id(void);
// float    pmsm_thread_get_iq(void);
// uint64_t pmsm_thread_get_step_count(void);
// float    pmsm_thread_get_time(void);

// /*==============================================================================
//  * 控制输入
//  *==============================================================================*/
// void pmsm_thread_set_voltage(float va, float vb, float vc);
// void pmsm_thread_set_load(float torque);

// #ifdef __cplusplus
// }
// #endif

// #endif // PMSM_THREAD_H