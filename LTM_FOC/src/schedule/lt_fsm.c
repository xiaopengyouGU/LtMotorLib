#include "schedule/lt_fsm.h"

/* 当前状态（静态变量，文件域私有） */
static volatile tasks_state_t s_state = State_Init;

void lt_fsm_init(void)                      /* 状态机初始化 */
{
    s_state = State_Init;
}

void lt_fsm_update(lt_event_t event)         /* 状态机更新，事件触发 */
{
    /* 遇错误事件，直接进入 Error 状态 */
    if(event == Event_Fault){
        s_state = State_Error;
        return;
    }
    
    switch (s_state)
    {
        case State_Init:                    /* State_Init —— 初始化状态 */
            if(event == Event_Init_Done) {
                s_state = State_Idle;       /* 初始化完成 → 进入空闲 */
            }
            break;

        case State_Idle:                    /* State_IDLE —— 空闲状态 */
            if(event == Event_Enable) {
                s_state = State_Enable;     /* 收到使能 → 进入使能状态 */
            }
            break;

        case State_Enable:                  /* State_Enable —— 使能状态（等待运行指令）*/
            if(event == Event_Run) {
                s_state = State_Running;    /* 收到运行 → 进入运行状态 */
            }else if(event == Event_Disable) {
                s_state = State_Idle;       /* 收到失能 → 回到空闲状态 */
            }
            break;

        case State_Running:                 /* State_Running —— 运行状态（执行三环控制）*/
            if(event == Event_Stop) {
                s_state = State_Stop;       /* 收到停止 → 进入停机状态 */
            }else if(event == Event_Disable) {
                s_state = State_Idle;       /* 收到失能 → 回到空闲状态 */
            }
            break;

        case State_Stop:                    /* State_Stop —— 停机状态（减速停止，主电保持使能）*/
            if(event == Event_Run) {
                s_state = State_Running;    /* 收到运行 → 恢复运行状态 */
            }else if(event == Event_Disable) {
                s_state = State_Idle;       /* 收到失能 → 回到空闲状态 */
            }
            break;
        case State_Error:                   /* State_Error —— 故障状态（PWM封锁，等待故障清除） */
            if(event == Event_Fault_Clear) {
                s_state = State_Idle;       /* 故障清除 → 回到空闲状态 */
            }
            break;
        default:
            break;
    }
}

tasks_state_t lt_fsm_get(void)           /* 获取当前状态 */
{
    return s_state;
}
