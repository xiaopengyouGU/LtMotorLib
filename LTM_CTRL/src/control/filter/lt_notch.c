/* 陷波滤波器：三级级联双二阶（DF2T），全定点实现（模块内没有任何浮点）
 *   I/O  ：Q15 信号
 *   系数 ：b0/b2/a2 用 int16 Q15；a1/b1 拆成 int8 整数部分 + int32 Q16 小数
 *          （陷波器 a1 能到 ±2，Q15 装不下；拆开之后小数部分有完整 16 位精度）
 *   状态 ：int32（DF2T 状态实测 ≤0.9 倍满量程，但中间项要 32 位）
 *   热路径：零浮点、零除法、零库调用
 *   设计路径：定点 sin/cos（多项式）+ 定点 10^(d/20)（连乘），只在 set 里跑一次
 */
#include "control/filter/lt_notch.h"
#include <string.h>

/* 一般 1~2 个齿轮谐振 + 1 个联轴器谐振 */
#define MAX_STAGES  3u          /* 支持三级抑制 */
#define NOTCH_Q     15          /* b0/b2/a2 的 Q 格式 */
#define NOTCH_QA    16          /* a1/b1 小数部分的 Q 格式 */
#define ONE24       (1 << 24)

/* 定点 sin/cos 的多项式系数（Q24）：u 为 [0, pi/4] 的圈数（0~0.125）
 *   sin(2*pi*u) = u*(C1 - u2*(C3 - u2*(C5 - u2*(C7 - u2*C9))))
 *   cos(2*pi*u) = 1 - u2*(D2 - u2*(D4 - u2*(D6 - u2*D8)))
 * 折叠到半象限后截断误差 <1e-8，比 float 的 sinf/tanf 还准
 */
#define SC_C1   105414357L
#define SC_C3   693598668L
#define SC_C5  1369108894L
#define SC_C7  1286910778L
#define SC_C9   705627793L
#define SC_D2   331168970L
#define SC_D4  1089502240L
#define SC_D6  1433727481L
#define SC_D8  1010737361L

/* 10^(-1/20) 的 Q15（每次 -1dB 乘一次）*/
#define DEPTH_1DB_Q15   29205

typedef struct {
    int32_t a1f;                /* a1/b1 的小数部分，Q16 */
    int32_t s1;                 /* Q15 信号域的 DF2T 状态 */
    int32_t s2;
    int16_t b0;                 /* Q15 */
    int16_t b2;                 /* Q15 */
    int16_t a2;                 /* Q15 */
    int8_t  a1n;                /* a1/b1 的整数部分（陷波器为 -2/-1/0）*/
    uint8_t enable;             /* 该级是否启用 */
} stage_t;

typedef struct {
    uint32_t freq;              /* 调用频率（Hz）*/
    stage_t stages[MAX_STAGES];
} lt_notch_obj;

static lt_notch_obj notch_obj;

/* Q24 角度（圈数×2^24）→ sin/cos（Q24）*/
static void notch_sincos(uint32_t the, int32_t *sv, int32_t *cv)
{
    int32_t quad = (int32_t)(the >> 22);            /* 象限 0~3 */
    int32_t rem  = (int32_t)(the & 0x3FFFFFu);      /* 象限内角度 */
    int64_t u, u2, p, q;
    int32_t su, cu, t, sw = 0;

    if (rem > 0x200000) { rem = 0x400000 - rem; sw = 1; }   /* 折到 [0, pi/4] */
    u  = (int64_t)rem;                               /* Q24 圈数：rem 就是 0~0.25 圈的 Q24 */
    u2 = (u * u) >> 24;

    p = SC_C7 - ((u2 * SC_C9) >> 24);
    p = SC_C5 - ((u2 * p)     >> 24);
    p = SC_C3 - ((u2 * p)     >> 24);
    p = SC_C1 - ((u2 * p)     >> 24);
    su = (int32_t)((u * p) >> 24);

    q = SC_D6 - ((u2 * SC_D8) >> 24);
    q = SC_D4 - ((u2 * q)     >> 24);
    q = SC_D2 - ((u2 * q)     >> 24);
    cu = (int32_t)(ONE24 - ((u2 * q) >> 24));

    if (sw) { t = su; su = cu; cu = t; }             /* 折过角：sin/cos 互换 */

    switch (quad) {
    case 0:  *sv =  su; *cv =  cu; break;
    case 1:  *sv =  cu; *cv = -su; break;
    case 2:  *sv = -su; *cv = -cu; break;
    default: *sv = -cu; *cv =  su; break;
    }
}

/* 10^(depth_dB/20) 的 Q15（负 dB），≤ -60dB 直接按 0 处理（理想陷波）*/
static uint32_t notch_ratio_q15(int32_t depth_dB)
{
    uint32_t r = 32768;
    int32_t n = -depth_dB;

    if (n <= 0) return 32768;
    if (n >= 60) return 0;
    while (n-- > 0) r = (r * DEPTH_1DB_Q15) >> 15;
    return r;
}

static int16_t notch_q15(int64_t v)
{
    int32_t t = (int32_t)((v + 256) >> 9);           /* Q24 → Q15，四舍五入 */

    if (t >  32767) t =  32767;                     /* a2 逼近 1 时按 32767 截，极点仍在单位圆内 */
    if (t < -32768) t = -32768;
    return (int16_t)t;
}

/*
 * 单级系数设计：双线性变换 + 频率预畸（公式与浮点版完全一致，只是全程定点）
 * 连续域: H(s) = (s^2 + 2*xi_z*wn*s + wn^2) / (s^2 + 2*xi_p*wn*s + wn^2)
 *         xi_p = 1/(2Q), xi_z = xi_p * 10^(depth/20)
 * 双线性变化 s = (2/T) * (1 - z⁻¹)/(1 + z⁻¹) ==>
 * H(z) = (b2*z^-2 + b1*z^-1 + b0)/(a2*z^-2 + a1*z^-1 + a0)
 * wa/K = tan(pi*fc/fs)，K² 在分子分母里约掉，所以不用算 K
 */
static void stage_design(stage_t *st, uint32_t fc_Hz, uint32_t Q, int32_t depth_dB, uint32_t fs)
{
    uint32_t the, half;
    int32_t  s, c, t_q16;
    int64_t  xp, xz, t24, t2, d, nb0, nb1, nb2, na2;
    int64_t  b0f, b1f, b2f, a2f;
    int32_t  ai, af;

    st->s1 = 0;
    st->s2 = 0;

    if (fc_Hz == 0u || Q == 0u || 2u * fc_Hz >= fs) {   /* 超过 Nyquist 频率，直接返回 */
        st->enable = 0;
        st->b0 = 32767; st->b2 = 0; st->a2 = 0;
        st->a1n = 0; st->a1f = 0;
        return;
    }

    /* 半角 pi*fc/fs 的 sin/cos，tan = sin/cos（Q16）*/
    the  = (uint32_t)((((uint64_t)fc_Hz) << 24) / fs);
    half = the >> 1;
    notch_sincos(half, &s, &c);
    if ((c >> 8) == 0) {                                /* 太靠近 Nyquist，tan 发散 */
        st->enable = 0;
        st->b0 = 32767; st->b2 = 0; st->a2 = 0;
        st->a1n = 0; st->a1f = 0;
        return;
    }
    t_q16 = (int32_t)((((int64_t)(s >> 8)) << 16) / (c >> 8));

    /* xi_p = 1/(2Q)（Q24），xi_z = xi_p * 10^(depth/20) */
    xp = ONE24 / (2 * (int64_t)Q);
    xz = (xp * (int64_t)notch_ratio_q15(depth_dB)) >> 15;

    /* 开始双线性变换，并进行系数归一化（a0归一化到1）处理 */
    t24 = (int64_t)t_q16 << 8;                          /* Q24 */
    t2  = (t24 * t24) >> 24;
    d   =  t2 + (((2 * xp) * t24) >> 24) + ONE24;       /* 分母 */
    nb0 =  t2 + (((2 * xz) * t24) >> 24) + ONE24;
    nb1 =  2 * (t2 - ONE24);
    nb2 =  t2 - (((2 * xz) * t24) >> 24) + ONE24;
    na2 =  t2 - (((2 * xp) * t24) >> 24) + ONE24;

    b0f = (nb0 << 24) / d;                              /* Q24 */
    b1f = (nb1 << 24) / d;
    b2f = (nb2 << 24) / d;
    a2f = (na2 << 24) / d;

    st->b0 = notch_q15(b0f);
    st->b2 = notch_q15(b2f);
    st->a2 = notch_q15(a2f);

    /* 陷波器零极点同角度，b1 恒等于 a1；拆成整数 + Q16 小数 */
    ai = (int32_t)(b1f >> 24);
    af = (int32_t)(b1f - ((int64_t)ai << 24));
    if (af > (1 << 23)) { ai++; af -= ONE24; }          /* 折到 [-0.5, 0.5] */
    st->a1n = (int8_t)ai;
    st->a1f = (af + 128) >> 8;                          /* Q24 → Q16，四舍五入 */

    st->enable = 1;
}

void lt_notch_init(uint32_t freq)
{
    lt_notch_obj *notch = &notch_obj;
    memset(notch, 0, sizeof(lt_notch_obj));
    notch->freq = freq ? freq : 20000u;
}

void lt_notch_set(uint8_t level, uint32_t fc_Hz, uint32_t Q, int32_t depth_dB)
{
    lt_notch_obj *notch = &notch_obj;
    if (level >= MAX_STAGES) return;
    stage_design(&notch->stages[level], fc_Hz, Q, depth_dB, notch->freq);
}

void lt_notch_reset(void)
{
    lt_notch_obj *notch = &notch_obj;
    for (uint32_t i = 0; i < MAX_STAGES; i++) {
        notch->stages[i].s1 = 0;
        notch->stages[i].s2 = 0;
    }
}

int32_t lt_notch_update(int32_t x)
{
    stage_t *st = notch_obj.stages;  /* 指针调用，局部变量进寄存器，性能更好 */
    int32_t y = x;                   /* 上一级的输出 */

    for (uint32_t i = 0; i < MAX_STAGES; i++, st++) {
        int32_t yi;
        if (!st->enable) continue;

        /* DF2T: y[n] = b0*x[n] + s1[n]
         *       s1[n+1] = b1*x[n] - a1*y[n] + s2[n]，用旧的 s2
         *       s2[n+1] = b2*x[n] - a2*y[n]
         * a1/b1 项 = a1n * 值 + ((a1f * 值) >> 16)
         * 每项单独移位再相减，避免两个乘积相加把 int32 顶出去；
         * 级间不做饱和（阶跃时输出能到 1.11 倍满量程，夹了会削顶）。
         */
        yi = ((int32_t)st->b0 * y >> NOTCH_Q) + st->s1;
        st->s1 = ((int32_t)st->a1n * y)  + (st->a1f * y >> NOTCH_QA)
               - ((int32_t)st->a1n * yi) - (st->a1f * yi >> NOTCH_QA) + st->s2;
        st->s2 = ((int32_t)st->b2 * y  >> NOTCH_Q)
               - ((int32_t)st->a2 * yi >> NOTCH_Q);
        y = yi;
    }

    if (y >  32767) y =  32767;      /* 出口按 Q15 饱和 */
    if (y < -32768) y = -32768;
    return y;
}
