#include "motor/lt_motor.h"
#include "schedule/lt_fsm.h"
#include "tasks/driver_task.h"
#include "tasks/control_tasks.h"
#include "common/tasks_param_def.h"

/* 对称范围判断：超出 ±max 为真 */
#define LT_OVER_RANGE(v, max)   ((v) > (max) || (v) < -(max))
/* 超范围直接返回 LT_ERR_OVER_RANGE */
#define LT_REQUIRE_RANGE(v, max) \
    do { if (LT_OVER_RANGE(v, max)) return LT_ERR_OVER_RANGE; } while (0)

/* 应用单位 → 控制域（各环定点格式）：整条链路只有这一处换算
 *   开环     %（1.0 pu = 母线/√3）→ Vq Q15
 *   力矩     q 轴电流 A（峰值）    → iq Q15
 *   速度/斜坡 RPM                 → 速度 Q24（1.0 = 1 圈/s）
 *   位置     角度°                → 位置 Q24（1.0 = 1 圈）
 * 超范围返回 LT_ERR_OVER_RANGE，正常返回 LT_OK */
static lt_err_t _target_encode(lt_motor_mode_t mode, float v, int32_t *out)
{
    switch (mode) {
        case Mode_Open_Loop:
            LT_REQUIRE_RANGE(v, 100.0f);
            *out = PCT_TO_Q15(v);                       /* % → Vq Q15 */
            break;
        case Mode_Torque:
        case Mode_Ramp_Torque:
            LT_REQUIRE_RANGE(v, TARGET_TORQUE_MAX_A);
            *out = A_TO_IQ15(v);                        /* A（峰值）→ iq Q15，与 info.Iq 同口径 */
            break;
        case Mode_Speed:
        case Mode_Ramp_Speed:
            LT_REQUIRE_RANGE(v, TARGET_SPEED_MAX_RPM);
            *out = RPM_TO_Q24(v);                       /* RPM → 速度 Q24 */
            break;
        case Mode_Position:
            LT_REQUIRE_RANGE(v, TARGET_POS_MAX_DEG);
            *out = DEG_TO_Q24(v);                       /* ° → 位置 Q24 */
            break;
        default:    return LT_ERR_STATE;
    }
    return LT_OK;
}

static lt_err_t _motor_disable(void)   /* 电机失能 */
{
    driver_disable();                  /* 驱动失能 */
    lt_fsm_update(Event_Disable);      /* 状态机更新 */
    return LT_OK;
}

/* 目标值反编码（与 _target_encode 对称）*/
static float _target_decode(tasks_mode_t mode, int32_t v)
{
    switch (mode) {
        case Mode_Open_Loop:    return Q15_TO_PCT(v);
        case Mode_Torque:
        case Mode_Ramp_Torque:  return Q15_TO_A(v);
        case Mode_Speed:
        case Mode_Ramp_Speed:   return Q24_TO_RPM(v);
        case Mode_Position:     return Q24_TO_DEG(v);
        default:                return 0.0f;
    }
}

lt_err_t lt_motor_init(void)
{
    lt_fsm_init();                      /* 状态机初始化 */
    driver_task_init();                 /* 底层驱动初始化 */
    control_tasks_init();               /* 三环控制任务初始化 */
        control_tasks_zero_find();      /* 上电电角度零点（内部：calib 吸死 → 存进控制层）*/
    /* 零点结果上报接口待定（作者后续改动），这里先只做标定 */
    lt_fsm_update(Event_Init_Done);
    /* 三环任务回调绑定 */
    driver_set_callback(ADC_Scan_Callback, control_tasks_run);   /* 单路径三环级联分频 */
    /* 电机初始化完毕后，内部会自动进入 IDLE 状态，无需手动操作 */
    return LT_OK;
}

lt_err_t lt_motor_enable(void)          /* 电机使能 */
{
    /* 仅 IDLE 允许使能 */
    if (lt_fsm_get() != State_Idle)     return LT_ERR_STATE;   
    driver_enable();                    /* 驱动使能 */
    lt_fsm_update(Event_Enable);        /* 状态机更新 */
    return LT_OK;
}

lt_err_t lt_motor_disable(void)          /* 电机失能 */
{
    return _motor_disable();
}

lt_err_t lt_motor_run(void)              /* 电机启动 */
{
    tasks_state_t state = lt_fsm_get();
    if (state == State_Error)   return LT_ERR_STATE;   /* 错误态直接拒绝 */
    if (state == State_Stop) {          /* 停机状态下：只有速度接近 0 才能重新启动 */
        lt_motor_info_t info;
        lt_motor_get_info(&info);
        if (LT_OVER_RANGE(info.speed_pll, STOP_DONE))   return LT_ERR_STATE;
    } 
    /* 位置模式：Start 时做初次规划（尚未 Running，与规划器无竞态）*/
    if (control_tasks_get_mode() == Mode_Position) {
        control_tasks_plan_start();
    }
    lt_fsm_update(Event_Run);

    return LT_OK;
}

lt_err_t lt_motor_stop(uint8_t estop)    /* 电机停机：0 受控（25pu/s），1 急停（100pu/s）*/
{
    if (lt_fsm_get() == State_Running) {
        control_tasks_stop(estop);       /* 异步：返回时还在减速，看 info.state 判定停完 */
    }
    return lt_fsm_get() == State_Error ? LT_ERR_STATE : LT_OK;
}

lt_err_t lt_motor_set(lt_motor_mode_t mode, float target)
{
    /* IDLE 或 STOP（受控停机已停稳、占空比已清零）都允许切换运行模式 */
    if (mode != control_tasks_get_mode() && lt_fsm_get() != State_Idle && lt_fsm_get() != State_Stop)  return LT_ERR_STATE;

    /* 位置模式额外要求：当前位置也在范围内 */
    if (mode == Mode_Position) {
        lt_motor_info_t info;
        lt_motor_get_info(&info);
        LT_REQUIRE_RANGE(info.pos, TARGET_POS_MAX_DEG);
    }

    int32_t enc;
    lt_err_t err = _target_encode(mode, target, &enc);   /* 内含范围检查 + 换算 */
    if (err != LT_OK)  return err;

    control_tasks_set(mode, enc);                        /* 入口换算：float → 控制域 */

    return LT_OK;
}

/* 应用层取信息：先拿内部整数快照，再用预定义宏折成应用单位 */
void lt_motor_get_info(lt_motor_info_t *info)
{
    if (!info) return;
    tasks_info_t task;
    control_tasks_get(&task);

    info->pos         = CNT_TO_DEG(task.pos);
    info->speed       = CPS_TO_RPM(task.speed);
    info->speed_pll   = CPS_TO_RPM(task.speed_pll);
    info->vbus        = Q15_TO_V(task.vbus);
    info->motor_temp  = TENTH_TO_C(task.motor_temp);
    info->driver_temp = TENTH_TO_C(task.driver_temp);
    info->te          = 0.0f;                   /* 转矩上报待接（要力矩常数）*/
    /* 电机电流 */
    info->Ia = Q15_TO_A(task.Ia);
    info->Ib = Q15_TO_A(task.Ib);
    info->Ic = Q15_TO_A(task.Ic);
    info->Id = Q15_TO_A(task.Id);
    info->Iq = Q15_TO_A(task.Iq);

    info->mode   = (lt_motor_mode_t)task.mode;
    info->state  = (lt_motor_state_t)task.state;
    info->err    = task.err;                     /* 保护错误码：LT_OK = 正常 */
    info->target = _target_decode(task.mode, task.target);
}