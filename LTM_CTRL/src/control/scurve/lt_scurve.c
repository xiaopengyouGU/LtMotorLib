/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-4-21      Lvtou        5-segment S-curve、traped-curve、triangle-curve
 * 2026-9-24      Lvtou        全整数化：整数 + 残差累加，热路径无除法，落点残差在规划阶段吸收
 * 2026-9-24      Lvtou        规划器去重：plan 结构体 + _plan_begin/_plan_commit
 * 2026-9-26      Lvtou        去 malloc：idx 索引 + 静态实例池（默认 2 个规划器）
 */
#include "control/scurve/lt_scurve.h"
#include <string.h>

/*---------------------- 5段S型曲线规划结果 ----------------------*/
typedef struct {
    int32_t Tsec[5];            /* 5段时长 (ms)，用不到的段填 0 */
    int32_t v_limit;            /* 速度上限 (Unit/s) */
    int64_t a_acc;              /* 加速段加速度 (Unit/s²) */
    int64_t a_dec;              /* 减速段加速度 (Unit/s²) */
    int64_t acc0;               /* 起步加速度：T形/三角形 = a_acc，S形 = 0 */
    int64_t j_step;             /* 加加速步进（S形） */
    int64_t j_step_dec;         /* 减加速步进（S形） */
    int32_t Nd;                 /* 减速段拍数，用于落点吸收；0 = 不做吸收 */
} lt_scurve_plan_t;

/*---------------------- 5段S型曲线对象 ----------------------*/
struct lt_scurve_object {
    /* ---- 规划结果 ---- */
    lt_scurve_plan_t plan;
    /* ---- 运行时状态 ---- */
    int32_t pos;                /* 当前位置 (Unit) */
    int32_t pos_res;            /* 位置残差 (1/1000 Unit) */
    int32_t target;             /* 目标位置 (Unit) */
    int32_t vel;                /* 当前速度 (Unit/s) */
    int32_t vel_res;            /* 速度残差 (1/1000 Unit/s) */
    int32_t elapsed;            /* 当前段已用时间 (ms) */
    int32_t pos_hold;           /* 规划切换前的旧位置 */
    int32_t vel_trim;           /* 减速段速度微调 (Unit/s) */
    int64_t acc;                /* 当前加速度 (Unit/s²) */
    uint16_t Ts;                /* 更新周期 (ms) */
    int8_t dir;                 /* 方向：1 正向，-1 反向 */
    uint8_t phase;              /* 当前阶段：0~4，5 表示完成 */
    volatile uint8_t ready;     /* 规划就绪标志 */
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

/* 5个阶段的加加速度符号：+1 加加速，-1 减加速，0 匀速 */
static const int8_t J_sign[5] = {1, -1, 0, -1, 1};

static inline int32_t _abs32(int32_t x) { return (x < 0) ? -x : x; }
static inline int64_t _abs64(int64_t x) { return (x < 0) ? -x : x; }

/*==============================================================================
 * 整数平方根（逐位法，仅 start 冷路径使用：无浮点、无库调用）
 *==============================================================================*/
static int64_t _lt_isqrt(int64_t x)
{
    int64_t r = 0, bit;
    if (x <= 0) return 0;
    for (bit = (int64_t)1 << 62; bit > x; bit >>= 2) { }
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return r;
}

/*==============================================================================
 * 除以 1000 的倒数乘法表示（除法 → 乘 + 移位）
 *   q = (x * 1048) >> 20：k = floor(2^20/1000) 略小于真值，商最多偏小 1，用残差比较修正
 *   入口 64 位：acc·Ts、vel·Ts 都在 int64 里算，Unit 取 count 或 Q24 都不会再顶出
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

/*==============================================================================
 * 落点残差吸收：计算本次规划实际距离，把差额摊到减速段速度上
 *   精确离散位移（µUnit，与逐拍累加完全一致，速度/位置都带 1/1000 残差）：
 *     加速段  Σ(v0 + Aa·U·k/1000)·U/1000 = Na·v0·U·1000 + Aa·U²·Na(Na+1)/2
 *     匀速段  Tv·vp·U·1000
 *     减速段  Nd·vp·U·1000 - Ad·U²·Nd(Nd+1)/2
 *   减速段速度整体 +delta (Unit/s) ⇒ 位移 +delta·Nd·U·1000 (µUnit)，delta 取整后落点误差 ≤ 1 Unit
 *   Nd = 0（S 形暂不吸收）时 delta = 0
 *==============================================================================*/
static void _lt_trim_calc(lt_scurve_t s, int64_t need_u,
                          int32_t Na, int32_t Tv, int32_t Nd)
{
    int64_t denom = (int64_t)Nd * s->Ts * 1000;
    if (denom <= 0) { 
        s->vel_trim = 0; 
        return; 
    }

    uint32_t U  = s->Ts;
    int64_t  U2 = (int64_t)U * U;
    int64_t  v0 = s->vel;
    int64_t  Aa = s->plan.a_acc;
    int64_t  Ad = s->plan.a_dec;
    int64_t  vp = v0 + Aa * U * Na / 1000;                  /* 加速段结束速度 */

    int64_t plan_u  = (int64_t)Na * v0 * U * 1000 + Aa * U2 * Na * (Na + 1) / 2;
    plan_u += vp * U * Tv * 1000;                           /* 匀速段 */
    plan_u += (int64_t)Nd * vp * U * 1000 - Ad * U2 * Nd * (Nd + 1) / 2;   /* 减速段 */

    s->vel_trim = (int32_t)((need_u - plan_u) / denom);
}

/*==============================================================================
 * 规划公共部分
 *==============================================================================*/
/* 规划前：位置/目标/方向/周期/速度状态复位；返回钳位后的 Ta/Td 与总行程 */
static void _plan_begin(lt_scurve_t s, lt_scurve_config_t *cfg,
                        uint16_t *Ta, uint16_t *Td, int64_t *total)
{
    *Ta = cfg->acct_ms < 20 ? 20 : cfg->acct_ms;
    *Td = cfg->dect_ms < 20 ? 20 : cfg->dect_ms;
    int32_t hold = s->pos;                             /* 清空前先存旧位置 */
    memset(s, 0, sizeof(struct lt_scurve_object));     /* 清空结构体 */
    s->pos_hold = hold;                                /* 未就绪窗口内 update 返回它 */
    s->pos     = cfg->start_pos;
    s->target  = cfg->target_pos;
    int64_t d  = (int64_t)s->target - s->pos;
    s->dir     = (d >= 0) ? 1 : -1;
    s->Ts      = cfg->period_ms ? cfg->period_ms : 1;   /* 下限 1 ms：Ts = 0 会让段计时停住 */
    s->vel     = cfg->v_start;
    *total = (d >= 0) ? d : -d;
}

/* 规划后：把 plan 装进对象，并算好落点微调 */
static void _plan_commit(lt_scurve_t s, const lt_scurve_plan_t *p, int64_t total)
{
    memcpy(&(s->plan), p, sizeof(lt_scurve_plan_t));
    s->acc = p->acc0;            /* 起步加速度：不装进去就永远不走车 */
    int32_t U  = (s->Ts > 0) ? s->Ts : 1;                       /* 毫秒 -> 拍数 */
    int32_t Tv = (p->Tsec[1] > 0) ? (p->Tsec[1] / U) : 1;       /* Tsec[1]=0 时 phase1 仍走 1 拍 */
    _lt_trim_calc(s, total * 1000000, p->Tsec[0] / U, Tv, p->Nd / U);
}

/*==============================================================================
 * 三种规划：三角形（短距离降级）/ 梯形 / S形 —— 各自只算参数，最后交给 _plan_commit
 *==============================================================================*/
static void _lt_scurve_start_triangle(lt_scurve_t s, lt_scurve_config_t *cfg,
                                      int64_t a_acc, int64_t a_dec, int64_t total)
{
    /* 解 total = (v_peak²-v_start²)/(2a_acc) + (v_peak²-v_stop²)/(2a_dec)
     *   => v_peak² = H·total + (v_start²·a_dec + v_stop²·a_acc)/(a_acc+a_dec)
     *      H = 2·a_acc·a_dec/(a_acc+a_dec)（对称时即 a）
     *   a 上到 2^31 以上时 2·a_acc·a_dec 会顶出 64 位，改成先折 Q16 权重：
     *      w = a_acc/(a_acc+a_dec)，H = 2·a_dec·w，T2 用同一组权重，全程不出大乘积 */
    int64_t v_start = cfg->v_start;
    int64_t v_stop  = cfg->v_stop;
    if (a_acc < 1) a_acc = 1;      /* v_max == v_start 时 a 会解出 0，这里兜底为 1 */
    if (a_dec < 1) a_dec = 1;
    int64_t sum_a   = a_acc + a_dec;
    int64_t w_acc   = (a_acc * 65536 + sum_a / 2) / sum_a;  /* Q16 权重 a_acc/(a_acc+a_dec) */
    int64_t w_dec   = 65536 - w_acc;
    int64_t H       = (2 * a_dec * w_acc) >> 16;            /* ≈ 2·a_acc·a_dec/(a_acc+a_dec) */
    int64_t T2      = ((v_start * v_start) >> 16) * w_dec
                    + ((v_stop  * v_stop ) >> 16) * w_acc;  /* 起步/停止速度的加权平均 */
    int64_t vmax2   = (int64_t)cfg->v_max * cfg->v_max;
    int32_t v_peak;
    if (total <= 0) {
        v_peak = (int32_t)v_start;                          /* 没有位移，原地不动 */
    } else if (vmax2 <= T2 || H > (vmax2 - T2) / total) {
        v_peak = cfg->v_max;                                /* 短距离会把 v_peak 顶过上限，直接取上限 */
    } else {
        v_peak = (int32_t)_lt_isqrt(H * total + T2);
    }
    if (v_peak > cfg->v_max)   v_peak = cfg->v_max;         /* 不超过速度上限 */
    if (v_peak < v_start)      v_peak = (int32_t)v_start;

    int32_t Ta_act = (int32_t)((int64_t)(v_peak - v_start) * 1000 / a_acc);
    int32_t Td_act = (int32_t)((int64_t)(v_peak - v_stop)  * 1000 / a_dec);
    if (Ta_act < 1) Ta_act = 1;
    if (Td_act < 1) Td_act = 1;

    lt_scurve_plan_t p = {0};                            /* 只有加速、减速两段 */
    p.Tsec[0] = Ta_act;  
    p.Tsec[2] = Td_act;
    p.v_limit = v_peak;  
    p.a_acc   = a_acc;  
    p.a_dec   = a_dec;  
    p.acc0    = a_acc;
    p.Nd      = Td_act;
    _plan_commit(s, &p, total);
}

static void _lt_scurve_start_trape(lt_scurve_t s, lt_scurve_config_t *cfg)
{
    uint16_t Ta, Td;
    int64_t  total;
    _plan_begin(s, cfg, &Ta, &Td, &total);

    /* 梯形：a = Δv / T，用平均速度估位移 */
    int32_t v_max = cfg->v_max;
    int64_t a_acc = _abs64((int64_t)(v_max - cfg->v_start) * 1000 / Ta);
    int64_t a_dec = _abs64((int64_t)(v_max - cfg->v_stop)  * 1000 / Td);
    int64_t s_acc = (int64_t)(cfg->v_start + v_max) * Ta / 2000;
    int64_t s_dec = (int64_t)(v_max + cfg->v_stop)  * Td / 2000;

    if (s_acc + s_dec > total || v_max == 0) {
        _lt_scurve_start_triangle(s, cfg, a_acc, a_dec, total);   /* 短距离降级 */
        return;
    }

    lt_scurve_plan_t p = {0};                            /* 匀加速、匀速、匀减速 */
    p.Tsec[0] = Ta;  
    p.Tsec[1] = (int32_t)((total - s_acc - s_dec) * 1000 / _abs32(v_max));  
    p.Tsec[2] = Td;
    p.v_limit = v_max;  
    p.a_acc   = a_acc;  
    p.a_dec   = a_dec;  
    p.acc0    = a_acc;
    p.Nd      = Td;
    _plan_commit(s, &p, total);
}

static void _lt_scurve_start_s(lt_scurve_t s, lt_scurve_config_t *cfg)
{
    uint16_t Ta, Td;
    int64_t  total;
    _plan_begin(s, cfg, &Ta, &Td, &total);

    /* S形：Tj = Ta/2 内把加速度从 0 加到 a，再回到 0（jerk J）；减速段同理 */
    int32_t v_max = cfg->v_max;
    int32_t Tj  = Ta >> 1;
    int32_t Tjd = Td >> 1;
    int64_t a_acc = (Tj  > 0) ? _abs64((int64_t)(v_max - cfg->v_start) * 1000 / Tj)  : 0;
    int64_t a_dec = (Tjd > 0) ? _abs64((int64_t)(v_max - cfg->v_stop)  * 1000 / Tjd) : 0;
    int64_t J     = (Tj  > 0) ? (a_acc * 1000 / Tj)  : 0;
    int64_t Jd    = (Tjd > 0) ? (a_dec * 1000 / Tjd) : 0;

    int64_t s_acc = (int64_t)cfg->v_start * Ta / 1000 + (int64_t)a_acc * Tj  * Tj  / 1000000;
    int64_t s_dec = (int64_t)v_max * Td / 1000 - (int64_t)a_dec * Tjd * Tjd / 1000000;

    if (s_acc + s_dec > total || v_max == 0) {
        _lt_scurve_start_triangle(s, cfg, a_acc, a_dec, total);   /* 短距离降级 */
        return;
    }

    lt_scurve_plan_t p = {0};               /* 加加速、减加速、匀速、加减速、减减速 */
    p.Tsec[0] = Tj;  
    p.Tsec[1] = Tj;
    p.Tsec[2] = (int32_t)((total - s_acc - s_dec) * 1000 / _abs32(v_max));
    p.Tsec[3] = Tjd; 
    p.Tsec[4] = Tjd;
    p.v_limit = v_max;  
    p.a_acc   = a_acc;  
    p.a_dec   = a_dec;  
    p.acc0    = 0;
    p.j_step     = J  * s->Ts / 1000;
    p.j_step_dec = Jd * s->Ts / 1000;
    p.Nd = 0;                               /* S 形暂不做落点吸收 */
    _plan_commit(s, &p, total);
}

/*==============================================================================
 * 对外接口
 *==============================================================================*/
void lt_scurve_reset(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (s) memset(s, 0, sizeof(*s));
}

void lt_scurve_start(uint8_t idx, lt_scurve_config_t *cfg)    /* 规划启动 */
{
    lt_scurve_t s = _sc(idx);
    if (!s || !cfg) return;
    s->ready = 0;               /* 置未就绪：中断内 update 返回原值，避免读到半更新状态 */

    if (cfg->type == 0) {
        _lt_scurve_start_trape(s, cfg);
    } else {
        _lt_scurve_start_s(s, cfg);
    }
    s->ready = 1;               /* 规划完成，恢复步进 */
}

/* 每周期更新，返回当前位置指令值（Unit）：全整数、热路径无除法 */
int32_t lt_scurve_update(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return 0;
    if (!s->ready) return s->pos_hold;      /* 规划未就绪（start 执行中）：返回切换前位置，不步进不跳变 */
    lt_scurve_plan_t *p = &s->plan;
    uint8_t phase  = s->phase;              /* 当前所处规划阶段 */
    int64_t j_step = p->j_step;             /* 每周期的加速度增量 */    
    /* 梯形完成判断：phase>=3 且 j_step==0 */
    if (j_step == 0 && phase >= 3) return s->target;
    /* S形完成判断： phase>=5 */
    if (phase >= 5) return s->target;

    int32_t Ts    = s->Ts;
    int64_t step  = (phase <= 2) ? j_step   : p->j_step_dec;
    int64_t limit = (phase <= 2) ? p->a_acc : p->a_dec;

    /* 根据当前阶段更新加速度 */
    int64_t acc = s->acc;                   /* 当前加速度 Unit/s^2 */
    if (J_sign[phase] != 0)
        acc += step * J_sign[phase];

    /* 限幅加速度：T 形（j_step==0）减速段在 phase2，加速度为负，必须允许 ±limit；
     * S 形（j_step!=0）减速段在 phase3~4，phase0~2 加速度非负 */
    if (j_step == 0) {
        if (acc >  limit)       acc =  limit;
        if (acc < -limit)       acc = -limit;
    } else if (phase <= 2) {
        if (acc > limit)        acc =  limit;
        else if (acc < 0)       acc =  0;
    } else {
        if (acc > 0)            acc =  0;
        else if (acc < -limit)  acc = -limit;
    }

    /* 更新速度：vel += acc·Ts/1000（倒数乘法，无除法），残差保证不丢小数 */
    int64_t dv  = (int64_t)acc * Ts + s->vel_res;
    int32_t vel = s->vel;
    uint32_t rem;
    int32_t qv = (dv >= 0)  ?  _lt_div1000((uint64_t)dv,    &rem)
                            : -_lt_div1000((uint64_t)(-dv), &rem);
    vel += qv;
    s->vel_res = (dv >= 0) ? (int32_t)rem : -(int32_t)rem;
    if (vel > p->v_limit) { vel = p->v_limit; s->vel_res = 0; }
    else if (vel < 0)     { vel = 0;          s->vel_res = 0; }

    /* 更新位置：pos += dir·vel·Ts/1000（vel >= 0，同样用倒数乘法）*/
    int64_t dp = (int64_t)vel * Ts + s->pos_res;
    int32_t qp = _lt_div1000((uint64_t)dp, &rem);
    s->pos    += s->dir * qp;
    s->pos_res = (int32_t)rem;

    /* 段切换 */
    s->elapsed += Ts;                       /* 段计时用毫秒：Ts != 1 时段长不会被拉长 Ts 倍 */
    if (s->elapsed >= p->Tsec[phase]) {
        s->elapsed = 0;
        phase++;
        if (j_step == 0) {
            /* 梯形：进入匀速段清加速度；进入减速段置负加速度 + 落点微调 */
            if (phase == 1)     acc = 0;
            else if (phase == 2) {
                acc  = -p->a_dec;
                vel +=  s->vel_trim;
            }
        } else if (phase == 3) {
            /* S形：进入减速段（phase3+4）加落点微调（暂为 0）*/
            vel += s->vel_trim;
        }
    }
    /* 最后更新当前阶段、速度和加速度 */
    s->phase = phase;
    s->vel   = vel;
    s->acc   = acc;

    return s->pos;
}

uint8_t lt_scurve_is_done(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return 1;
    if (s->plan.j_step == 0 && s->phase >= 3) return 1;
    if (s->phase >= 5) return 1;        /* 此时显然 j_step != 0 */
    return 0;
}

/* 曲线停止：就地停住
 * 完成判定返回的是 target，所以把 target 改成当前位置
 * = 位置保持不动（不再像以前那样跳到原目标） */
void lt_scurve_stop(uint8_t idx)
{
    lt_scurve_t s = _sc(idx);
    if (!s) return;
    s->target  = s->pos;
    s->phase   = 5;
    s->vel     = 0;
    s->vel_res = 0;
    s->acc     = 0;
}

