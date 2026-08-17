#ifndef MOTOR_FSM_H
#define MOTOR_FSM_H
/**
 * @file    motor_fsm.h
 * @brief   电机有限状态机接口
 */

#include "motor_ctrl/common/lt_motor_types.h"          /* 包含公共类型定义 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * 状态机驱动事件定义
 * ============================================================ */
typedef enum {
    Event_None = 0,          /* 无事件 */
    Event_Init_Done,         /* 初始化完毕 */
    Event_Enable,            /* 使能事件 */
    Event_Disable,           /* 失能事件 */
    Event_Run,               /* 运行事件 */
    Event_Stop,              /* 停机事件 */
    Event_Fault,             /* 故障事件 */
    Event_Fault_Clear,       /* 故障清除事件 */
} lt_event_t;
/* ============================================================
 * 状态机 API
 * ============================================================ */
void lt_fsm_init(void);                 /* 初始状态机 */
void lt_fsm_update(lt_event_t event);   /* 更新状态机 */
lt_motor_state_t lt_fsm_get(void);      /* 获取当前状态 */

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_FSM_H */
