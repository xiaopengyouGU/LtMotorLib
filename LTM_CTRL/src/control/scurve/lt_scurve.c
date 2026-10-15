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
    int32_t  ak;                /* 当前加速度：以 jerk 步长为单位的整数拍数（T 形时就是加速度本身）*/
    int32_t  ak_acc;            /* +a_acc 对应的拍数 */
    int32_t  ak_dec;            /* a_dec 对应的拍数 */
    int64_t  brake_k;           /* 刹车判据 |v| 项系数（start 预算）*/
    int64_t  brake_b;           /* 刹车判据 |a| 项系数（start 预算，T 形为 0）*/
    int64_t  brake_a2;          /* 刹车判据右侧系数 = 2·a_dec（start 预算）*/
    int32_t  cap_v;             /* 终点捕获：速度门限 = 一个 jerk 斜坡能拉出的速度 */
    int32_t  cap_d;             /* 终点捕获：距离门限 = 一个 jerk 斜坡走过的距离 */
    int64_t  brk_c;             /* 判据常数项 a_dec²·Tj²/12（收尾段走过的距离）*/
    int64_t  ramp_v;            /* 收尾判据系数 = j_step·Ts/2000 */
    int64_t  kk3;               /* 翻转段曲率项 = ak_dec·j_step²·Ts²/3e6 */
    /* ---- 指令状态 ---- */
    int32_t  target;            /* 目标位置（Unit），随时可改 */
    int32_t  pos;               /* 位置指令（Unit）*/
    int32_t  vel;               /* 速度指令（Unit/s，带符号）*/
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

    /* S 形：加速度用“每拍一个 jerk 步”的整数拍数记；T 形 j_step = 0，
     * 拍数直接就是加速度（a = ak）*/
    s->j_step = 0;
    s->ak     = 0;
    s->ak_acc = s->a_acc;
    s->ak_dec = s->a_dec;
    if (cfg->type != 0) {
        /* jerk 段取配置值；没给才退回半个斜坡时间（老口径）*/
        int32_t Tj = cfg->jerk_ms ? (int32_t)cfg->jerk_ms : (Ta >> 1);
        if (Tj > 0) {
            int64_t J = (int64_t)s->a_acc * 1000 / Tj;      /* Unit/s³ */
            s->j_step = (int32_t)(J * s->Ts / 1000);        /* 每拍加速度增量 */
            if (s->j_step < 1) s->j_step = 1;
            s->ak_acc = s->a_acc / s->j_step;               /* 拉满 a_acc 的拍数 */
            s->ak_dec = s->a_dec / s->j_step;               /* 拉满 a_dec 的拍数 */
            if (s->ak_acc < 1) s->ak_acc = 1;
            if (s->ak_dec < 1) s->ak_dec = 1;
        }
    }

    /* 刹车判据的定值（start 一次算好，热路径无除法）：
     *   brake_k = a_dec·Tj          brake_b = 2a_dec·Ts/1000
     *   kk3     = ak_dec·j_step²·Ts²/3e6（翻向段的曲率项）
     *   brk_c   = (a_dec·Tj)²/12（收尾段走过的距离）
     *   ramp_v  = j_step·Ts/2000（收尾判据：|v| ≤ ak²·ramp_v 才开始把 a 收 0）
     * T 形（j_step = 0）不给这些值，判据退化成经典的 v² ≥ 2a_dec·|d| */
    s->brake_a2 = 2 * (int64_t)s->a_dec;
    s->brake_k  = 0;
    s->brake_b  = 0;
    s->cap_v    = 0;                /* T 形判据本身精确，不需要捕获 */
    s->cap_d    = 0;
    s->ramp_v   = 0;
    s->kk3      = 0;
    s->brk_c    = 0;
    if (s->j_step > 0) {
        int64_t Tj_ms = (int64_t)s->a_dec * s->Ts / s->j_step;   /* 减速 jerk 段时长（ms）*/
        s->brake_k = Tj_ms * s->a_dec / 1000;
        s->brake_b = s->brake_a2 * s->Ts / 1000;
        s->cap_v    = (int32_t)(s->brake_k / 128);                 /* a_dec·Tj/128 */
        s->cap_d    = (int32_t)((int64_t)s->cap_v * Tj_ms / 500);   /* cap_v·Tj·2 */
        s->ramp_v   = (int64_t)s->j_step * s->Ts / 2000;
        s->kk3      = (int64_t)s->ak_dec * s->j_step * s->j_step * s->Ts * s->Ts / 3000000;
        s->brk_c    = (s->brake_k * s->brake_k) / 12;       /* 2a_dec·(a_dec·Tj²/24) */
    }
    s->target  = cfg->target_pos;
    s->pos     = cfg->start_pos;
    s->vel     = cfg->v_start;
    if (s->vel >  s->v_max) s->vel =  s->v_max;
    if (s->vel < -s->v_max) s->vel = -s->v_max;
    s->ak      = 0;
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
    int32_t ak = s->ak;             /* 当前加速度（拍数）*/
    int64_t d  = (int64_t)s->target - s->pos;       /* 剩余距离（带符号）*/

    /* 终点捕获：进了一个 jerk 斜坡就能跨过的窗口就直接贴住目标。
     * 1kHz 离散 + jerk 量化没法让曲线严格收敛到 0，不收就在目标附近留下小极限环 */
    if (s->cap_v > 0 && d <= s->cap_d && d >= -s->cap_d &&
        v <= s->cap_v && v >= -s->cap_v) {
        s->pos     = s->target;
        s->vel     = 0;
        s->vel_res = 0;
        s->pos_res = 0;
        s->ak      = 0;
        return s->pos;
    }

    /* 1) 刹车判据：算出“以当前 (v, ak) 停到目标还要多少距离”。先把加速度从
     * |ak| 翻到 -a_dec（k 拍；这段里速度先涨后落，比“速度不变”多走
     * kk3·k²·(2|ak|-ak_dec)——漏掉它就是大 jerk 下几千 count 的过冲），
     * 再按 -a_dec 匀减速到收尾速度，最后是 jerk 收尾段。
     * 两边同乘 2a_dec，系数均在 start 预算，热路径只剩乘法 */
    int64_t av  = (v > 0) ? v : -v;
    int64_t kak = (ak > 0) ? ak : -ak;
    int64_t k   = kak + s->ak_dec;                        /* 加速度翻转段拍数 */
    int64_t va  = av + (kak - s->ak_dec) * k * s->ramp_v;  /* 翻转结束时的速度 */
    int64_t lhs = va * va
                + av * k * s->brake_b
                + 2 * av * s->brake_b  /* 补两拍行程：判据用的是拍初状态，加上离散量化 */
                + s->kk3 * k * k * (2 * kak - s->ak_dec)
                + s->brk_c;
    int64_t rhs = (d > 0 ? d : -d) * s->brake_a2;

    uint8_t brake;
    if (v == 0) {
        brake = 0;
    } else if (d == 0 || ((v > 0) != (d > 0))) {
        brake = 1;
    } else {
        brake = (lhs >= rhs);
    }

    /* 2) 期望加速度（拍数域）*/
    int32_t ak_cmd;
    if (brake) {
        /* 收尾判据：剩下的 |ak| 拍把加速度收到 0，恰好能把速度收掉
         * Σa·Ts/1000 = ak²·j_step·Ts/2000。|v| 还大就继续满制动，不大就开始 ramp；
         * 边收边算、自校正，不依赖进收尾时的速度多准 */
        int64_t v_ramp = kak * kak * s->ramp_v;
        ak_cmd = (av > v_ramp) ? ((v > 0) ? -s->ak_dec : s->ak_dec) : 0;
    }
    else if (d == 0)                    ak_cmd = 0;                  /* 已到目标：保持 */
    else if (d > 0 && v >=  s->v_max)   ak_cmd = 0;                  /* 正向巡航 */
    else if (d < 0 && v <= -s->v_max)   ak_cmd = 0;                  /* 反向巡航 */
    else                                ak_cmd = (d > 0) ? s->ak_acc : -s->ak_acc;

    /* 3) S 形限 jerk：加速度朝期望值按 j_step 渐变；T 形直接给 */
    if (s->j_step > 0) {
        if      (ak < ak_cmd) ak++;                  /* S 形：每拍朝期望值挪一步 */
        else if (ak > ak_cmd) ak--;
    } else {
        ak = ak_cmd;                                /* T 形：直接跟期望值 */
    }
    int64_t a_i64 = (s->j_step > 0) ? (int64_t)ak * s->j_step : (int64_t)ak;

    /* 4) 积分速度：v += a·Ts/1000 */
    int32_t vres  = s->vel_res;
    int32_t v_pre = v;
    v += _div1000_i64(a_i64 * Ts + vres, &vres);
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
    s->ak  = ak;
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
    s->ak      = 0;
    s->vel_res = 0;
    s->pos_res = 0;
}
