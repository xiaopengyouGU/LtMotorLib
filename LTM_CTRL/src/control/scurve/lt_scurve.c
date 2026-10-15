/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author      Notes
 * 2026-4-21      Lvtou       5-segment S-curve、traped-curve、triangle-curve
 * 2026-9-24      Lvtou       全整数化：整数 + 残差累加，热路径无除法，落点残差在规划阶段吸收
 * 2026-9-26      Lvtou       去 malloc：idx 索引 + 静态实例池（默认 2 个规划器）
 * 2026-10-14     Lvtou       改在线重规划：目标随时可改，每拍只解下一拍，不再预解算段
 */
#include "control/scurve/lt_scurve.h"
#include <string.h>

/*---------------------- 在线规划器对象 ----------------------*/
struct lt_scurve_object {
    /* ---- 约束 ---- */
    int32_t  v_max;             /* 速度上限（Unit/s，正数）*/
    int32_t  a_acc;             /* 加速上限（Unit/s²）*/
    int32_t  a_dec;             /* 减速上限（Unit/s²）*/
    int32_t  j_step;            /* 每周期加速度增量（Unit/s²，S 形；0 = T 形）*/
    uint16_t Ts;                /* 更新周期（ms）*/
    /* ---- 指令状态 ---- */
    int32_t  target;            /* 目标位置（Unit），随时可改 */
    int32_t  pos;               /* 位置指令（Unit）*/
    int32_t  vel;               /* 速度指令（Unit/s，带符号）*/
    int32_t  acc;               /* 当前加速度（Unit/s²，带符号）*/
    int32_t  pos_res;           /* 位置残差（1/1000 Unit）*/
    int32_t  vel_res;           /* 速度残差（1/1000 Unit/s）*/
    volatile uint8_t ready;     /* start 执行中挡住中断侧步进 */
};

/* 私有：内部一律用指针书写，对外仍是 uint8_t idx */
typedef struct lt_scurve_object *lt_scurve_t;

/* ---- 实例池：无堆，idx 直接映射 ---- */
static struct lt_scurve_object scurve_pool[LT_SCURVE_MAX_INSTANCES];

static struct lt_scurve_object *_sc(uint8_t idx)
{
    if (idx < LT_SCURVE_MAX_INSTANCES) {
        return &scurve_pool[idx];
    }
    return 0;
}

/*==============================================================================
 * 除以 1000 的倒数乘法表示（除法 → 乘 + 移位）
 *   q = (x * 1048) >> 20：k = floor(2^20/1000) 略小于真值，商最多偏小 1，用残差比较修正
 *   入口 64 位：acc·Ts、vel·Ts 都在 int64 里算，Unit 取 count 或 Q24 都不会顶出
 *==============================================================================*/
#define LT_DIV1000_K   1048u
#define LT_DIV1000_S   20

static inline int32_t _lt_div1000(uint64_t x, uint32_t *rem)
{
    uint32_t q = (uint32_t)((x * LT_DIV1000_K) >> LT_DIV1000_S);
    uint32_t r = (uint32_t)(x - (uint64_t)q * 1000u);
    if (r >= 1000u) { r -= 1000u; q++; }        /* 修正倒数截断误差 */
    *rem = r;
    return (int32_t)q;
}

/* 带符号的 x/1000：商与残差同号，残差接着累加，不丢小数 */
static inline int32_t _div1000_i64(int64_t x, int32_t *rem)
{
    uint32_t r;
    if (x >= 0) {
        int32_t q = _lt_div1000((uint64_t)x, &r);
        *rem = (int32_t)r;
        return q;
    }
    int32_t q = -_lt_div1000((uint64_t)(-x), &r);
    *rem = -(int32_t)r;
    return q;
}

/*==============================================================================
 * 对外接口
 *==============================================================================*/
void lt_scurve_reset(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (s) memset(s, 0, sizeof(*s));
}

/* 建状态：位置/速度初值由 config 给，之后全由 update 自己走 */
void lt_scurve_start(uint8_t idx, lt_scurve_config_t *cfg)
{
    lt_scurve_t s = _sc(idx);
    if (!s || !cfg) return;
    s->ready = 0;                                   /* 未就绪：中断侧 update 只返回旧位置 */

    int32_t  v_max = (cfg->v_max < 0) ? -cfg->v_max : cfg->v_max;
    uint16_t Ta    = cfg->acct_ms < 20 ? 20 : cfg->acct_ms;
    uint16_t Td    = cfg->dect_ms < 20 ? 20 : cfg->dect_ms;

    s->v_max = v_max;
    s->a_acc = (int32_t)((int64_t)v_max * 1000 / Ta);      /* Unit/s² */
    s->a_dec = (int32_t)((int64_t)v_max * 1000 / Td);
    if (s->a_acc < 1) s->a_acc = 1;
    if (s->a_dec < 1) s->a_dec = 1;
    s->Ts    = cfg->period_ms ? cfg->period_ms : 1;        /* 下限 1 ms */

    /* S 形：用 acct_ms 的一半把加速度从 0 拉满（与段式 S 形同口径）*/
    s->j_step = 0;
    if (cfg->type != 0) {
        int32_t Tj = Ta >> 1;
        if (Tj > 0) {
            int64_t J = (int64_t)s->a_acc * 1000 / Tj;     /* Unit/s³ */
            s->j_step = (int32_t)(J * s->Ts / 1000);       /* 每周期增量 */
            if (s->j_step < 1) s->j_step = 1;
        }
    }

    s->target  = cfg->target_pos;
    s->pos     = cfg->start_pos;
    s->vel     = cfg->v_start;
    if (s->vel >  s->v_max) s->vel =  s->v_max;
    if (s->vel < -s->v_max) s->vel = -s->v_max;
    s->acc     = 0;
    s->pos_res = 0;
    s->vel_res = 0;

    s->ready = 1;                                   /* 建状态完成，恢复步进 */
}

/* 在线改目标：只写一个 int32，中断侧下一拍就按新目标解，不用停机也不用重算曲线 */
void lt_scurve_set_target(uint8_t idx, int32_t target)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return;
    s->target = target;
}

/*==============================================================================
 * 每周期推一拍
 *   判据只有一条：剩余距离 d 和刹停距离 v²/(2·a_dec) 比大小。d 与 v 异号
 *   （目标落在背后）必然落进"要刹"，刹到 0 之后下一拍自然反向加速，
 *   不需要显式写"先停再走"。
 *==============================================================================*/
int32_t lt_scurve_update(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return 0;
    if (!s->ready) return s->pos;                   /* start 执行中：返回切换前位置，不跳变 */

    int32_t Ts = s->Ts;
    int32_t v  = s->vel;
    int64_t d  = (int64_t)s->target - s->pos;       /* 剩余距离（带符号）*/

    /* 1) 刹车判断：目标在背后、或剩余距离已经不够停 */
    uint8_t brake;
    if (v == 0) {
        brake = 0;
    } else if (d == 0 || ((v > 0) != (d > 0))) {
        brake = 1;
    } else {
        int64_t v2 = (int64_t)v * v;
        int64_t ds = 2 * (int64_t)s->a_dec * (d > 0 ? d : -d);
        brake = (v2 >= ds);
    }

    /* 2) 期望加速度 */
    int32_t a_cmd;
    if (brake)                          a_cmd = (v > 0) ? -s->a_dec : s->a_dec;
    else if (d == 0)                    a_cmd = 0;                  /* 已到目标：保持 */
    else if (d > 0 && v >=  s->v_max)   a_cmd = 0;                  /* 正向巡航 */
    else if (d < 0 && v <= -s->v_max)   a_cmd = 0;                  /* 反向巡航 */
    else                                a_cmd = (d > 0) ? s->a_acc : -s->a_acc;

    /* 3) S 形限 jerk：加速度朝期望值按 j_step 渐变；T 形直接给 */
    int32_t a = s->acc;
    if (s->j_step > 0) {
        if      (a < a_cmd) { a += s->j_step; if (a > a_cmd) a = a_cmd; }
        else if (a > a_cmd) { a -= s->j_step; if (a < a_cmd) a = a_cmd; }
    } else {
        a = a_cmd;
    }

    /* 4) 积分速度：v += a·Ts/1000 */
    int32_t vres  = s->vel_res;
    int32_t v_pre = v;
    v += _div1000_i64((int64_t)a * Ts + vres, &vres);
    s->vel_res = vres;
    /* 刹车不过零：过零就停在 0，下一拍判据再决定要不要反向 */
    if (brake && ((v_pre > 0 && v < 0) || (v_pre < 0 && v > 0))) {
        v = 0;
        s->vel_res = 0;
    }
    if (v >  s->v_max) { v =  s->v_max; s->vel_res = 0; }
    if (v < -s->v_max) { v = -s->v_max; s->vel_res = 0; }

    /* 5) 积分位置：pos += v·Ts/1000（带 1/1000 残差，落点能收到 1 Unit 以内）*/
    int32_t pres = s->pos_res;
    s->pos += _div1000_i64((int64_t)v * Ts + pres, &pres);
    s->pos_res = pres;

    s->vel = v;
    s->acc = a;
    return s->pos;
}

int32_t lt_scurve_get_vel(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    return s ? s->vel : 0;
}

uint8_t lt_scurve_is_done(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return 1;
    return (s->vel == 0 && s->pos == s->target) ? 1 : 0;
}

/* 曲线停止：就地停住。目标改成当前位置，速度/加速度清零（到位判据随即成立）*/
void lt_scurve_stop(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return;
    s->target  = s->pos;
    s->vel     = 0;
    s->acc     = 0;
    s->vel_res = 0;
    s->pos_res = 0;
}
