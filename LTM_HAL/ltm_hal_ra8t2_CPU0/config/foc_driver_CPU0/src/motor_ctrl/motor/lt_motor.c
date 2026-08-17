#include "motor_ctrl/motor/lt_motor_api.h"
#include "motor_ctrl/schedule/lt_fsm.h"
#include "motor_ctrl/tasks/driver_task.h"
#include "motor_ctrl/tasks/control_tasks.h"
// #include "motor_ctrl/tasks/ident_task.h"

static int _motor_disable(void);    /* 电机失能 */

void lt_motor_init(void)
{
    lt_fsm_init();              /* 状态机初始化 */
    driver_task_init();         /* 底层驱动初始化 */
    control_tasks_init();       /* 三环控制任务初始化 */
    /* 三环任务回调绑定 */
    driver_set_callback(ADC_Scan_Callback, current_loop_task);      /* 20kHz 回调 */
    driver_set_callback(TIM_Speed_Callback, speed_loop_task);       /* 3kHz  回调 */
    driver_set_callback(TIM_Position_Callback, position_loop_task); /* 1kHz  回调 */
    /* 电机初始化完毕后，内部会自动进入 IDLE 状态，无需手动操作 */
}

int lt_motor_enable(void)           /* 电机使能 */
{
    driver_enable();                /* 驱动使能 */
    lt_fsm_update(Event_Enable);    /* 状态机更新*/
    return 0;                       /* 操作成功 */
}

int lt_motor_disable(void)          /* 电机失能 */
{
    return _motor_disable();
}

// /* 返回值：0：辨识中，-1：非运行时，1：辨识完毕 */
// int lt_motor_identify(lt_ident_mode_t mode)   /* 电机辨识 */
// {
//     lt_motor_state_t state = lt_fsm_get();    /* 获取电机状态 */
//     if(state != State_Running)      return -1;/* 非运行时，直接退出 */
//     if(ident_task_is_done()){
//         ident_task_stop();                    /* 辨识完毕后，任务停止 */
//         return  _motor_disable();             /* 辨识完毕，失能电机 */
//     }
//     /* 启动辨识任务，内部自动维护辨识状态机 */
//     ident_task_start(mode);
//     return 0;
// }

int lt_motor_start(void)            /* 电机启动 */
{
    lt_motor_state_t state = lt_fsm_get();     /* 获取电机状态 */
    if(state == State_Stop){        /* 停止状态下，判断 */
        lt_motor_info_t info;
        control_tasks_get(&info);   /* 获取电机信息 */
        if(info.speed > 3.0f || info.speed <-3.0f)  return -1; /* 只有在速度接近0时，才能重新启动 */
    }
    lt_fsm_update(Event_Run);       /* 状态机更新 */
    return 1;
}

int lt_motor_stop(void)             /* 电机停机 */
{
    lt_fsm_update(Event_Stop);
    return 1;                       /* 操作成功 */
}

int lt_motor_set(lt_motor_mode_t mode, float target)    /* 设置电机模式 */
{
    lt_motor_state_t state = lt_fsm_get();              /* 获取电机状态 */
    lt_motor_info_t info;
    control_tasks_get(&info);                           /* 获取电机信息 */
    if(info.mode != mode){                             
        if(state != State_Idle)         return 0;       /* 只有 IDLE 状态下，才能进行状态切换！*/
    }
    /* 没有发生模式切换，则直接修改目标值即可 */
    control_tasks_set(mode, target);
    return 1;                                           /* 1：操作成功， 0：操作失败 */
}

void lt_motor_get_info(lt_motor_info_t *info)
{
    if(!info)           return;     /* 判空 */
    control_tasks_get(info);        /* 获取电机信息 */
}

/***************************************************************************/
static int _motor_disable(void)     /* 电机失能接口 */
{
    driver_disable();               /* 驱动失能 */
    lt_fsm_update(Event_Disable);   /* 状态机更新*/
    return 1;                       /* 操作成功 */
}

