/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author      Notes
 * 2026-07-02     Lvtou       自适应 M法 测速实现
 * 2026-07-20     Lvtou       锁相环（PLL）测试模块实现
 * 2026-08-12     Lvtou       修改测速模块实现和API接口
 * 2026-08-31     Lvtou       将归一化替换为单圈回绕，性能更优
 * 2026-9-24      Lvtou       全整数化：内部 counts/tick（Q16）+ 对外 count/s，热路径无浮点
 *
 * 内部用 counts/tick（Q16），对外 count/s：位置增量直接就是速度、M 法窗口的整数拍换算
 * 退化成 1/cycle，量程也只有 10^2~10^4；用 count/s 时 3000RPM/16384CPR 就是 819200，
 * int32 里放不下 16 位小数。
 *
 * 环路（每拍，dt = 1/freq）：pos_est += speed_est + Kp·dt·err；speed_est += Ki·dt·err。
 * 令 a = Kp·dt、g = Ki·dt，特征方程 λ² − (2−a−g)·λ + (1−a) = 0；连续近似下
 * ζ = Kp/(2√(Ki·freq))、f_n = √(Ki·freq)/2π、整定时间 ≈ 4/(2πζf_n)。
 * a ≤ 0.2 时离散极点偏离公式 5% 以内；a = 0.66（4kHz 调 f_n=300Hz）时已是 389Hz/ζ0.88，
 * 公式失效——f_n 的上限被这个 a 卡住，所以得把调用频率拉上去才能保证大带宽。
 *
 * 输出低通极点取 ratio·f_n（ratio 见 win_config），alpha = ratio·√(Ki/freq)，由
 * _alpha_from_ki 在 set 里算好。 */

#include <string.h>
#include <stdint.h>
#include "control/speed/lt_speed.h"
#include "math/basic/lt_math.h"

#define     ABS(x)                          ((x) > 0 ? (x) : -(x))

#define     PLL_MIN_SPEED_RPM               1       /* 锁相环允许的最低速度（RPM），只作用于输出 */
#define     PLL_STOP_SHIFT                  3       /* 停机判定死区 = 最低速度 >> 3 = 1/8 RPM（由 pll_min_q16 移位得到，不占变量）*/
#define     PLL_STOP_TIME_DEN               4       /* 停机判定时间 0.25 s = 1/4 s */
#define     M_MIN_SPEED_RPM                 4       /* M 法可用下限（RPM）：各档按自己的窗口拍数折算成最少脉冲数 */
/* 位置估计的小数位上限（Q12）：init 按 reso 往下调，保证 reso<<pos_frac 不溢出 int32 */
#define     POS_FRAC_MAX                    12

/* ---- 数值常量 ---- */
#define     INT32_TOP_U                     0x7FFFFFFFU /* int32 上界（无符号写法，供移位比较）*/
#define     POS_ERR_LIMIT                   32767       /* 位置误差限幅：保证 Kp·dt×err 不溢出 int32 */
#define     RECIP_Q16(cyc)                  (65536 / (cyc))  /* 1/cyc 的 Q16，cyc 取 2 的幂 */
#define     ALPHA_RATIO_SCALE               1000U       /* alpha_ratio 的标度 */
#define     ALPHA_Q16_SHIFT                 16          /* alpha 定点标度 Q16，1.0 = 65536 */
#define     ALPHA_G_Q16_MAX                 0xFFFFU     /* g=Ki/freq 的 Q16 上限，再大 alpha 也饱和 */
#define     ALPHA_Q16_MAX                   32768U      /* alpha 上限 = 0.5，极点留在奈奎斯特内 */
#define     ALPHA_NUM_MAX                   15          /* num/2^shift 的分子上限 */
#define     ALPHA_SHIFT_MAX                 8           /* 2^shift 的 shift 上限 */

/* 模块运行状态（cfg->state）*/
#define     LT_SPEED_ST_UNINIT              0           /* 未初始化 */
#define     LT_SPEED_ST_WAIT_SYNC           1           /* 已初始化，等首拍建立基准 */
#define     LT_SPEED_ST_RUN                 2           /* 正常运行 */
/* 速度的小数位（Q16）：Ki 项最小增量 Ki·dt² ≈ 4e-5 counts/tick，需要 16 位小数才能保证精度 */
#define     SPEED_FRAC                        16

/* ====== 自适应M法 测速状态机 ====== */
typedef enum {
    Window_Cycle_16 = 0,
    Window_Cycle_8,
    Window_Cycle_4,
    Window_Cycle_2,
    Window_Cycle_1,
    Window_Count
}lt_speed_state_t;

/* 速度分档表：阈值按 RPM 写，init 时换算成 counts/tick(Q16)；alpha_ratio 与阈值是两个估计器
 * 共用的速度区间定义，cycle 只有 M 法用。窗口拍数取 2 的幂，所以 1/cycle 在 init 里是常量。
 * 升档阈值按该档窗口累计约 30 个脉冲定，降档取上一档升档的 0.9 倍留滞环 */
typedef struct {
    uint16_t speed_up;                  /* 升档阈值 (RPM) */
    uint16_t speed_down;                /* 降档阈值 (RPM) */
    uint16_t alpha_ratio;               /* 输出低通极点 / f_n，x1000（0.88~2.65，低速档滤得重）*/
    uint8_t  cycle;                     /* 窗口拍数 */
}lt_speed_win_t;

/* ---- 按档位预算好的表：init 按 reso/freq 算，set 里再补 alpha ---- */
typedef struct{
    int32_t  recip_q16[Window_Count];   /* 1/cycle（Q16）：counts → counts/tick */
    int32_t  up_q16[Window_Count];      /* 升档阈值（Q16 counts/tick）*/
    int32_t  down_q16[Window_Count];    /* 降档阈值（Q16 counts/tick）*/
    uint8_t  alpha_num[Window_Count];   /* 输出低通 num/2^shift：由 Ki/freq 自适应 */
    uint8_t  alpha_shift[Window_Count];
#if LT_SPEED_USE_ADAP_M
    uint16_t min_delt[Window_Count];    /* 该档窗口内的最少脉冲数，低于它算极低速 */
#endif
}lt_speed_tab_t;

/* ---- 由 init 按 reso/freq 预先算好的标量 ---- */
typedef struct{
    int32_t  reso;                      /* 编码器分辨率 */
    int32_t  half_reso;                 /* 一半分辨率（半圈法补偿用）*/
    int32_t  reso_q;                    /* reso << pos_frac */
    uint8_t  pos_frac;                  /* 实际使用的位置小数位（按 reso 自适应）*/
    int32_t  pll_min_q16;               /* PLL 输出最低速度（Q16）*/
    uint32_t freq;                      /* 调用频率（Hz）*/
    uint16_t stop_ticks;                /* 判定停机所需拍数 */
    uint8_t  state;                     /* 模块状态 */
}lt_speed_cfg_t;

/* ---- 锁相环状态 ---- */
typedef struct{
    int32_t  pos_est;                   /* 估计位置（count，Q pos_frac）*/
    int32_t  speed_est;                 /* 锁相环估计速度（counts/tick，Q16）*/
    int32_t  speed_pll;                 /* 锁相环输出（counts/tick，Q16，已滤波）*/
    int32_t  kp_dt_q16;                 /* Kp×dt（Q16）*/
    int32_t  ki_dt_q16;                 /* Ki×dt（Q16）每拍速度增量 / 每个 count 误差 */
    uint16_t stop_cnt;                  /* 输出持续为 0 的拍数 */
}lt_speed_pll_t;

/* ---- 自适应 M 法状态 ---- */
typedef struct{
    int32_t  pos_last;                  /* 上一次 M 法触发时的位置（count）*/
    int32_t  speed;                     /* 自适应M法输出（counts/tick，Q16，已滤波）*/
    int32_t  speed_last;                /* 上一个滤波输出（counts/tick，Q16）*/
    uint8_t  cycle_cnt;                 /* 当前节拍数 */
    uint8_t  cycle_target;              /* 目标节拍数 */
    lt_speed_state_t state;             /* 自适应M法测速状态机 */
}lt_speed_m_t;

/* ---- 汇总对象：常量 + 档位表 + PLL + M法 ---- */
typedef struct{
    lt_speed_cfg_t cfg;
    lt_speed_tab_t tab;
    lt_speed_pll_t pll;
#if LT_SPEED_USE_ADAP_M
    lt_speed_m_t   m;
#endif
}lt_speed_obj;

static const lt_speed_win_t win_config[Window_Count] = {
    {  31,    8,  884, 16 },   /* 16 拍 =  8ms  , 升档 31 RPM,  极点 = 0.88 f_n */
    {  62,   28,  884,  8 },   /*  8 拍 =  4ms  , 升档 62 RPM,  极点 = 0.88 f_n */
    { 125,   56, 1768,  4 },   /*  4 拍 =  2ms  , 升档 125 RPM, 极点 = 1.77 f_n */
    { 250,  112, 1768,  2 },   /*  2 拍 =  1ms  , 升档 250 RPM, 极点 = 1.77 f_n */
    { 999,  225, 2652,  1 }    /*  1 拍 =  0.5ms, 顶档（升档阈值不再使用）, 极点 = 2.65 f_n */
};

static lt_speed_obj speed_obj;  /* 唯一的测速对象 */

static lt_speed_cfg_t *  cfg = &speed_obj.cfg;
static lt_speed_tab_t *  tab = &speed_obj.tab;
static lt_speed_pll_t *  pll = &speed_obj.pll;
#if LT_SPEED_USE_ADAP_M
static lt_speed_m_t   *  m   = &speed_obj.m;
#endif

/**********************************************************************************************************/
/*                                          静态小工具                                                    */
/**********************************************************************************************************/

/* RPM → 内部 counts/tick(Q16)：RPM/60×reso/freq×2^16 */
static int32_t _rpm_to_tick_q16(uint32_t rpm, int32_t reso, uint32_t freq)
{
    if (freq == 0) return 0;
    return (int32_t)(((int64_t)rpm * reso << SPEED_FRAC) / (60 * (int64_t)freq));
}

/* 按速度反查分档（只取降档阈值，滞环由 M 法状态机负责），两个估计器共用同一套区间 */
#if LT_SPEED_USE_ADAP_M
/* M 法下限 RPM 折算成该档窗口内的最少脉冲数：RPM/60×reso×cycle/freq，向上取整 */
static uint16_t _min_delt_ticks(int32_t reso, uint32_t freq, uint32_t cycle)
{
    uint64_t den = 60ull * freq;
    uint64_t v;

    if (!den) return 0;
    v = ((uint64_t)M_MIN_SPEED_RPM * (uint64_t)reso * cycle + den - 1) / den;
    return (uint16_t)((v > 0xFFFFu) ? 0xFFFFu : v);
}
#endif

static inline uint8_t _band_for_speed(const lt_speed_tab_t *t, int32_t v)
{
    if (v < t->down_q16[Window_Cycle_2]) {
        if (v < t->down_q16[Window_Cycle_8])  return Window_Cycle_16;
        if (v < t->down_q16[Window_Cycle_4])  return Window_Cycle_8;
        return Window_Cycle_4;
    }
    if (v < t->down_q16[Window_Cycle_1])      return Window_Cycle_2;
    return Window_Cycle_1;
}

/* 一阶低通：y += (x − y)·num/2^shift，只用移位 */
static inline int32_t _lpf_q16(int32_t x, int32_t y, uint8_t num, uint8_t shift)
{
    int32_t d = (x - y) >> shift;
    return y + d * (int32_t)num;
}

#if LT_SPEED_USE_ADAP_M
/* M 法测速状态机：按 |speed| 在分档之间升/降 1 档（带滞环）*/
static void _speed_update_state(void)
{
    lt_speed_state_t curr_state = m->state;
    lt_speed_state_t new_state  = m->state;     /* 默认保持本档，别写成 Window_Cycle_16 */
    int32_t speed_abs           = ABS(m->speed);

    if (speed_abs > tab->up_q16[curr_state] && curr_state != Window_Cycle_1) {
        new_state = curr_state + 1;
    } else if (speed_abs < tab->down_q16[curr_state] && curr_state != Window_Cycle_16) {
        new_state = curr_state - 1;
    }

    if (new_state != curr_state) {
        m->state        = new_state;
        m->cycle_cnt    = 0;
        m->cycle_target = win_config[new_state].cycle;
    }
}
#endif

/* 输出低通系数按带宽自适应：alpha = ratio·√(Ki/freq) = ratio·√g。只跟 g 有关、
 * 与 freq 无关，换调用频率时滤波器的墙钟时间常数自动不变。量化成 num/2^shift */
static void _alpha_from_ki(uint32_t ki)
{
    uint32_t g_q16, sq, b;

    if (!cfg->freq || !ki) return;
    g_q16 = (uint32_t)(((uint64_t)ki << ALPHA_Q16_SHIFT) / cfg->freq);   /* g = Ki/freq，Q16 */
    if (g_q16 > ALPHA_G_Q16_MAX) g_q16 = ALPHA_G_Q16_MAX;                /* alpha 早就饱和了 */
    sq = lt_sqrt_u32(g_q16 << ALPHA_Q16_SHIFT);                          /* sqrt(g)，Q16 */

    for (b = 0; b < Window_Count; b++) {
        uint32_t best_e = 0xFFFFFFFFu;
        uint32_t want = (uint32_t)(((uint64_t)sq * win_config[b].alpha_ratio) / ALPHA_RATIO_SCALE);
        int n_best = 1, s_best = 0, n, sh;

        if (want > ALPHA_Q16_MAX) want = ALPHA_Q16_MAX;
        for (sh = 0; sh <= ALPHA_SHIFT_MAX; sh++) {
            n = (int)((want * (1u << sh) + (1u << (ALPHA_Q16_SHIFT - 1))) >> ALPHA_Q16_SHIFT);
            if (n < 1) n = 1;
            if (n > ALPHA_NUM_MAX) continue;
            uint32_t got = ((uint32_t)n << ALPHA_Q16_SHIFT) >> sh;
            uint32_t e = (got > want) ? (got - want) : (want - got);
            if (e < best_e) { 
                best_e = e; 
                n_best = n; 
                s_best = sh; 
            }
        }
        tab->alpha_num[b]   = (uint8_t)n_best;
        tab->alpha_shift[b] = (uint8_t)s_best;
    }
}

/**********************************************************************************************************/
/*                                          对外接口                                                      */
/**********************************************************************************************************/
void lt_speed_init(uint32_t reso, uint32_t freq)
{
    memset(&speed_obj, 0, sizeof(speed_obj));
    if (!reso || !freq)     return;

    cfg->reso       = (int32_t)reso;
    cfg->half_reso  = (int32_t)(reso >> 1);
    /* 位置小数位按 reso 自适应：保证 reso<<pos_frac 不溢出 int32
     * reso <= 2^19 → 12；2^21 → 9；2^23 → 7 */
    uint8_t pf = POS_FRAC_MAX;
    while (pf > 0 && (uint32_t)reso > (INT32_TOP_U >> pf)) pf--;
    cfg->pos_frac   = pf;
    cfg->reso_q     = (int32_t)(reso << pf);
    cfg->freq       = freq;

    /* 窗口拍数是 2 的幂，1/cycle 写成除常数表达式，编译器折成移位 */
#if LT_SPEED_USE_ADAP_M
    tab->recip_q16[Window_Cycle_16] = RECIP_Q16(16);
    tab->recip_q16[Window_Cycle_8]  = RECIP_Q16(8);
    tab->recip_q16[Window_Cycle_4]  = RECIP_Q16(4);
    tab->recip_q16[Window_Cycle_2]  = RECIP_Q16(2);
    tab->recip_q16[Window_Cycle_1]  = RECIP_Q16(1);
#endif

    for (uint8_t i = 0; i < Window_Count; i++) {
#if LT_SPEED_USE_ADAP_M
        tab->up_q16[i]    = _rpm_to_tick_q16(win_config[i].speed_up,   reso, freq);
        tab->min_delt[i]  = _min_delt_ticks((int32_t)reso, freq, win_config[i].cycle);
#endif
        tab->down_q16[i]  = _rpm_to_tick_q16(win_config[i].speed_down, reso, freq);
    }
    cfg->pll_min_q16 = _rpm_to_tick_q16(PLL_MIN_SPEED_RPM, reso, freq);
    cfg->stop_ticks  = (uint16_t)(freq / PLL_STOP_TIME_DEN);                    /* 0.25 s 对应的拍数 */

#if LT_SPEED_USE_ADAP_M
    m->cycle_target = win_config[Window_Cycle_16].cycle;    /* 从最长窗口（16 拍）开始 */
    m->state        = Window_Cycle_16;
#endif
    /* 占位：紧跟的 lt_speed_set 会覆盖，这里只防漏调 set 时滤波器冻住 */
    for (uint8_t b = 0; b < Window_Count; b++) {
        tab->alpha_num[b]   = 1;
        tab->alpha_shift[b] = 4;
    }
    cfg->state = LT_SPEED_ST_WAIT_SYNC;
}

void lt_speed_set(uint32_t pll_kp, uint32_t pll_ki)
{
    if (!cfg->state)     return;
    /* Kp_dt = Kp/freq（Q16）；Ki_dt = Ki/freq（Q16）—— 与浮点版 pll_kp*dt、pll_ki*dt*dt 等价 */
    pll->kp_dt_q16 = (int32_t)(((int64_t)pll_kp << SPEED_FRAC) / cfg->freq);
    pll->ki_dt_q16 = (int32_t)(((int64_t)pll_ki << SPEED_FRAC) / cfg->freq);
    _alpha_from_ki(pll_ki);                             /* 输出低通随带宽走 */
}

void lt_speed_update(uint32_t pos_count)
{
    if (!cfg->state)    return;
    if (cfg->state == LT_SPEED_ST_WAIT_SYNC) {
#if LT_SPEED_USE_ADAP_M
        m->pos_last  = pos_count;
#endif
        pll->pos_est = (int32_t)pos_count << cfg->pos_frac;
        cfg->state   = LT_SPEED_ST_RUN;
        return;                                         /* 首拍只同步，不参与测速 */
    }

    int32_t reso       = cfg->reso;
    int32_t half_reso  = cfg->half_reso;

#if LT_SPEED_USE_ADAP_M
    int32_t speed_last = m->speed_last;                 /* 上次 M 法触发时算出的速度 counts/tick */
    uint8_t band       = m->state;                      /* M 法自己的档位（带滞环）*/

    /* ---- 自适应 M 法 ---- */
    /* pos_delt 是“距离上一次 M 法触发”的位置增量，不是上一拍的增量，
     * 因为 m->pos_last 只在 run_adap 那一拍才更新 */
    int32_t pos_delt = pos_count - m->pos_last;
    if (pos_delt > half_reso) {                         /* 出现编码器跨圈：反向补偿 */
        pos_delt -= reso;
    } else if (pos_delt < -half_reso) {                 /* 出现编码器跨圈：正向补偿 */
        pos_delt += reso;
    }

    uint8_t run_adap = 0;                               /* 本拍是否到达 M 法窗口 */
    m->cycle_cnt++;
    if (m->cycle_cnt >= m->cycle_target) {
        m->cycle_cnt = 0;
        run_adap = 1;
    }
    /* 窗口未到时不写 m->speed：它本来就等于 speed_last，重写是多余的 */

    if (run_adap) {
        int32_t speed_raw;
        if (pos_delt < (int32_t)tab->min_delt[m->state]
            && pos_delt > -(int32_t)tab->min_delt[m->state]) {
            speed_raw = 0;                                              /* 脉冲太少：极低速 */
        } else {
            speed_raw = pos_delt * tab->recip_q16[m->state];            /* counts → counts/tick（Q16）*/
        }
        int32_t speed = _lpf_q16(speed_raw, speed_last,
                                 tab->alpha_num[band], tab->alpha_shift[band]);
        m->speed_last = speed;
        m->pos_last   = pos_count;
        m->speed      = speed;
        _speed_update_state();
    }
#endif

    /* ---- PLL 测速 ---- */
    uint8_t pos_frac  = cfg->pos_frac;
    int32_t reso_q    = cfg->reso_q; 
    int32_t speed_est = pll->speed_est;
    int32_t pos_est   = pll->pos_est + (speed_est >> (SPEED_FRAC - pos_frac));  /* counts/tick → Q pos_frac */
    int32_t err       = (int32_t)pos_count - (pos_est >> pos_frac);           /* 位置误差（count）*/
    if (err > half_reso) {                              /* 半圈法补偿：中高频下单次补偿即成立 */
        err -= reso;
    } else if (err < -half_reso) {
        err += reso;
    }

    /* 半圈法补偿必须在限幅之前：reso > 2^16 时回绕误差可达 ±reso/2（>32767），先限幅会把补偿判据打掉，导致大分辨率下环路失控 */
    if (err > POS_ERR_LIMIT)       err = POS_ERR_LIMIT;                 /* 限幅：保证增益乘积不溢出 int32 */
    else if (err < -POS_ERR_LIMIT) err = -POS_ERR_LIMIT;

    pos_est   += (pll->kp_dt_q16 * err) >> (SPEED_FRAC - pos_frac);
    if (pos_est >= reso_q)     pos_est -= reso_q;       /* 归一化：单次回绕即可，避免除法 */
    else if (pos_est < 0)      pos_est += reso_q;
    speed_est += pll->ki_dt_q16 * err;

    /* 最低速度死区只作用于输出，积分器持续积分 */
    int32_t speed_pll = pll->speed_pll;
    int32_t speed_out = (ABS(speed_est) < cfg->pll_min_q16) ? 0 : speed_est;
    uint8_t pband     = _band_for_speed(tab, ABS(speed_pll));  /* PLL 自己定档 */
    speed_pll = _lpf_q16(speed_out, speed_pll,
                              tab->alpha_num[pband], tab->alpha_shift[pband]);

    /* 停机防漂移：输出持续为 0 超过 0.25 s → 清积分器 */
    if (ABS(speed_pll) < (cfg->pll_min_q16 >> PLL_STOP_SHIFT)) {
        if (pll->stop_cnt < cfg->stop_ticks)    pll->stop_cnt++;
        else { 
            speed_est = 0; 
            pll->stop_cnt = 0; 
        }
    } else {
        pll->stop_cnt = 0;
    }

    pll->pos_est   = pos_est;
    pll->speed_est = speed_est;
    pll->speed_pll = speed_pll;
}

/* 输出：count/s（int32），应用层需要 RPM 时自己按 60/reso 折算 */
void lt_speed_get(int32_t *adap_speed, int32_t *pll_speed)
{
    if (cfg->state != LT_SPEED_ST_RUN) {
        if (adap_speed)     *adap_speed = 0;
        if (pll_speed)      *pll_speed  = 0;
        return;
    }
    /* counts/tick(Q16) → count/s：×freq >> 16 */
#if LT_SPEED_USE_ADAP_M
    if (adap_speed)     *adap_speed = (int32_t)(((int64_t)m->speed       * cfg->freq) >> SPEED_FRAC);
#else
    if (adap_speed)     *adap_speed = 0;                /* M 法已裁剪，只输出 PLL */
#endif
    if (pll_speed)      *pll_speed  = (int32_t)(((int64_t)pll->speed_pll * cfg->freq) >> SPEED_FRAC);
}
