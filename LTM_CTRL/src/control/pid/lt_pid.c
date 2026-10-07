/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * 2026-08-31  Lvtou   增量式 PI 接口
 * 2026-9-25   Lvtou   全定点化：增益与调用周期全整数
 * 2026-9-26   Lvtou   增益 Q15.15，ki = Ki·ts；去 malloc，静态实例池
 * 2026-9-27   Lvtou   init 声明信号格式；增益 Q15.15
 * 2026-10-07  Lvtou   去掉 Q15/Q24 分支：误差与增量全 64 位，内部不再区分信号格式
 */
/* lt_pid —— 增量式定点 PID（静态实例池）
 *
 * 信号：Q15（32767 = 1.0）或 Q24（2^24 = 1.0），模块不区分 —— Q24 的误差最坏到
 *       ±2^31（target 与 curr 取反号），int32 装不下，所以 err/err2 存 int64。
 * 增益：Q15.15（32768 = 1.0），内部存每拍系数 ki = Ki/freq、kd = Kd*freq。
 * 输出：累加器 int64（信号单位 << PID_ACC_SH），单位与输入一致。
 */
#include "control/pid/lt_pid.h"
#include "math/basic/lt_math.h"
#include <string.h>

#define PID_ACC_SH          15          /* 累加器小数位（与 Q15.15 增益对齐）*/
#define PID_OUT_LIM_DEF     (1 << 24)   /* 默认输出限幅（满量程），set_limits 会覆盖 */
#define PID_D2_LIMIT        (255 << 9)  /* D 项二阶差分夹幅（信号单位，位置环口径）*/
#define PID_GAIN_LIM        0x7FFFFFFFLL/* 增益存储上限 */

typedef struct{
    int32_t  target;            /* 目标值（信号单位）*/

    /* ---- 增益，全部 Q15.15 ---- */
    int32_t  kp;                /* Kp */
    int32_t  ki;                /* Ki·ts（每拍）*/
    int32_t  kd;                /* Kd·freq（每拍）*/

    /* ---- 输出累加器 ---- */
    int64_t  out_acc;           /* 输出（信号单位）× 2^PID_ACC_SH */
    int32_t  out_max;           /* 输出上限（信号单位）*/
    int32_t  out_min;           /* 输出下限（信号单位）*/

    int64_t  err;               /* 上一拍偏差（int64）*/
    int64_t  err2;              /* 上上拍偏差（int64）*/
    uint32_t freq;              /* 调用频率（Hz）*/
} lt_pid_object;

/* 私有：内部一律用指针书写，对外仍是 uint8_t idx */
typedef lt_pid_object *lt_pid_t;

/* ---- 实例池：无堆，idx 直接映射 ---- */
static lt_pid_object pid_pool[LT_PID_MAX_INSTANCES];

static lt_pid_object *_pid(uint8_t idx)
{
    if (idx < LT_PID_MAX_INSTANCES) {
        return &pid_pool[idx];
    }
    return 0;
}

uint8_t lt_pid_init(uint8_t idx, uint32_t freq)
{
    lt_pid_t pid;
    if (idx >= LT_PID_MAX_INSTANCES)    return 0;   /* 索引越界 */

    pid = &pid_pool[idx];
    memset(pid, 0, sizeof(lt_pid_object));
    pid->freq    = freq ? freq : 1;
    pid->out_max =  PID_OUT_LIM_DEF;
    pid->out_min = -PID_OUT_LIM_DEF;
    return 1;
}

void lt_pid_reset(uint8_t idx)
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return;
    pid->out_acc = 0;
    pid->err     = 0;
    pid->err2    = 0;
}

void lt_pid_set(uint8_t idx, int32_t Kp, int32_t Ki, int32_t Kd)
{
    lt_pid_t pid = _pid(idx);
    if (!pid || !pid->freq) return;         /* 未初始化：freq == 0 */

    int64_t f  = (int64_t)pid->freq;
    int64_t ki = ((int64_t)Ki + (f >> 1)) / f;      /* Ki·ts */
    int64_t kd = (int64_t)Kd * f;                   /* Kd/ts */

    ki = lt_clamp_i64(ki, PID_GAIN_LIM, -PID_GAIN_LIM);
    kd = lt_clamp_i64(kd, PID_GAIN_LIM, -PID_GAIN_LIM);

    pid->kp = Kp;
    pid->ki = (int32_t)ki;
    pid->kd = (int32_t)kd;
}

void lt_pid_set_target(uint8_t idx, int32_t target)
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return;
    pid->target = target;
}

void lt_pid_set_limits(uint8_t idx, int32_t out_max, int32_t out_min)
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return;

    if (out_max < out_min) {                /* 上下限写反了也认 */
        int32_t t = out_max;
        out_max = out_min;
        out_min = t;
    }
    pid->out_max = out_max;                 /* 信号单位 */
    pid->out_min = out_min;
}

int32_t lt_pid_get(uint8_t idx)
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return 0;
    return (int32_t)(pid->out_acc >> PID_ACC_SH);
}

/* 输出累加 + 限幅：在累加器尺度做，避免小增量被截成 0 */
static inline int32_t _out_commit(lt_pid_t pid, int64_t d)
{
    int64_t hi  = (int64_t)pid->out_max << PID_ACC_SH;
    int64_t lo  = (int64_t)pid->out_min << PID_ACC_SH;
    int64_t out = pid->out_acc + d;
    if (out > hi)      out = hi;
    else if (out < lo) out = lo;
    pid->out_acc = out;
    return (int32_t)(out >> PID_ACC_SH);
}

static inline int64_t _d2_clamp(int64_t d2)
{
    if (d2 >  PID_D2_LIMIT) return  PID_D2_LIMIT;
    if (d2 < -PID_D2_LIMIT) return -PID_D2_LIMIT;
    return d2;
}

int32_t lt_pi_update(uint8_t idx, int32_t curr_val)     /* 增量式 PI */
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return 0;

    int64_t err = (int64_t)pid->target - curr_val;                  /* 全 64 位 */
    int64_t d   = (int64_t)pid->kp * (err - pid->err) + (int64_t)pid->ki * err;
    pid->err = err;
    return _out_commit(pid, d);
}

int32_t lt_pd_update(uint8_t idx, int32_t curr_val)     /* 增量式 PD */
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return 0;

    int64_t err = (int64_t)pid->target - curr_val;
    int64_t d2  = _d2_clamp(err - 2 * pid->err + pid->err2);
    int64_t d   = (int64_t)pid->kp * (err - pid->err) + (int64_t)pid->kd * d2;
    pid->err2 = pid->err;
    pid->err  = err;
    return _out_commit(pid, d);
}

int32_t lt_pid_update(uint8_t idx, int32_t curr_val)    /* 增量式 PID */
{
    lt_pid_t pid = _pid(idx);
    if (!pid)   return 0;

    int64_t err = (int64_t)pid->target - curr_val;
    int64_t d2  = _d2_clamp(err - 2 * pid->err + pid->err2);
    int64_t d   = (int64_t)pid->kp * (err - pid->err) + (int64_t)pid->ki * err
                + (int64_t)pid->kd * d2;
    pid->err2 = pid->err;
    pid->err  = err;
    return _out_commit(pid, d);
}
