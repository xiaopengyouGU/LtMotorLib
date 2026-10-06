/* lt_pdob —— 速度环 DOB+P ＋ 电流环无差拍 DPCC ＋ ESO（全程定点，内部标幺）
 *
 * 标幺基准取硬件物理上限（参考 InstaSPIN FOC）：
 * I_B = i_base_mA（ADC 峰值电流）、
 * V_B = 3·v_base_mV/√3（相电压基准：ADC 满量程母线 × 3 再 / √3）、
 * ω_B = 2π·freq_max（最高电频率）。
 * ψ_pu = ψ·ω_B/V_B，标幺量与机型解耦：换电机只重填 config。
 * 被控对象（τ = ω_B·t）：L_pu·di/dτ = u − R_pu·i − we×ψ_pu − we×L_pu·i，无差拍解
 *     u = Ldt·i_ref − (Ldt−R_pu)·i_fb − we×L·i − we·ψ_pu − f̂ ,  Ldt = L_pu/dt_pu
 * ESO 用估计值推进、实测只经 eps = i − î 校正：î 超前一拍，正好抵消无差拍延时。
 * 内部电压是 pu(V_B)；对外 Vd/Vq 折算回"1.0 pu = 实测母线/√3" 后进 lt_foc_update。
 */
#include "lt_pdob.h"
#include "math/basic/lt_math.h"
#include <string.h>

#define Q_SIG    15                 /* 信号：电流/电压/we/ψ_pu/R_pu/L_pu… 全部 ≤ 1 pu */
#define Q_ST     24                 /* ESO 状态累加器 */
#define Q_CO     24                 /* 系数：>1 的 Ldt/ldr 用 Q24（范围 ±128，精度 6e-8）*/
#define Q15_ONE  32767              /* 1.0 pu：Q15 能表示的上限（饱和用）*/
#define SIG_FS   (1 << Q_SIG)       /* 32768：Q15 满量程定标因子（乘除换算用，≠Q15_ONE）*/
#define Q_WEK    16                 /* we_k 的 Q 格式：we_pu = speed·we_k >> Q_WEK */
#define SH_ST    (Q_ST - Q_SIG)     /* Q24 状态 ↔ Q15 信号（9）*/
#define SH_MUL   (2 * Q_SIG - Q_ST) /* Q15×Q15 → Q24 的右移（6），状态增量用 */
#define Q_SPD    24                 /* 速度环内部：速度与增益一律 pu Q24 */
#define SH_SPD   (Q_SPD - Q_SIG)    /* pu Q24 ↔ 对外 Q15 iq（9）*/

/* 状态（Q24）↔ 信号（Q15）*/
static inline int32_t st2sig(int32_t x) { return x >> SH_ST; }
static inline int32_t sig2st(int32_t x) { return x << SH_ST; }
#define MUL_TOP  2147467263LL       /* 0x7FFF7FFF：≈2^31，给四舍五入留余量 */
#define EPS_LIM  65000              /* ESO 观测残差限幅（Q15） */
#define G_LIM    16384              /* ESO 增益上限：带宽 ≈ 1/(6dt) */

/*---------------- 速度环 DOB+P --------------------------------------------
 * 被控对象：w' = C·(iq + d)，d 是折成等效 iq 的负载扰动。
 *   dw  = w - w_prev                实测加速度（w = lt_speed 的 pll_speed）
 *   err = dw - C·(iq + d)           模型加速度与实测之差
 *   d  += K·err/C                   扰动估计，K = wb·dt
 *   iq_ref = kp·(ref - w) - d       控制律（积分作用由观测器提供）
 *
 * 内部全在 pu 域，信号一律 Q24：
 *   速度基准 = speed_max（1.0 = speed_max count/s），电流基准 = 1.0 pu iq。
 *   |w|、|ref| ≤ 1，|ref-w| ≤ 2，|d| ≤ d_limit，故所有乘积与累加的上界固定，
 *   不随 reso / 极对数 / 频率漂移：中间量一律 64 位，限幅也在 64 位域做。
 * -------------------------------------------------------------------------*/
typedef struct {                 /* 速度环 DOB+P 子对象 */
    uint32_t freq;               /* 速度环调用频率 (Hz) */
    int32_t  base;               /* 速度基准：1.0 pu 对应的 count/s（= speed_max）*/
    int32_t  k_q;                /* count/s → pu Q24：(speed·k_q) >> k_sh */
    uint8_t  k_sh;
    int32_t  ref_q24;            /* 速度目标 (pu Q24) */
    int32_t  kp_q24;             /* 比例增益：iq(pu) / 速度(pu) */
    int32_t  out_lim_q24;        /* 输出限幅 (iq pu Q24) */
    int32_t  d_max_q24;          /* 扰动限幅 (iq pu Q24) */
    int32_t  c_q24;              /* C：每拍、每 pu iq 产生的 pu 加速度 */
    int32_t  kd_q24;             /* K/C */
    uint8_t  en;                 /* 1: 扰动补偿使能 */
    int32_t  fw_ki;              /* 弱磁积分增益（Q15，≤32767），0 = 关闭；
                                  * 输入 v_excess 是电压平方量纲（∝1/V_B²），
                                  * 标定值与 V_B² 成反比，见 lt_pdob_fw_set */
    int32_t  we_on;              /* 弱磁启用阈值（we Q15）：反电势 0.85pu 对应的 we */
    int32_t  id_fw_q15;          /* 弱磁输出：id 指令（Q15，≤ 0）*/
    /* ---- 观测器状态 ---- */
    int32_t  d_q24;              /* 扰动估计（等效 iq, pu Q24）*/
    int32_t  w_prev_q24;         /* 上一拍测速 (pu Q24) */
} pdob_dob_t;

/* 配置期常量（init / curr_set / 母线变化时算好，热路径只读）*/
typedef struct {
    int32_t r_pu;      /* R_pu = Rs·I_B/V_B */
    int32_t l_pu;      /* L_pu = Ls·ω_B·I_B/V_B */
    int32_t psi_pu;    /* ψ_pu = ψ·ω_B/V_B，Q24（弱磁时 > 1，Q15 只能到 2.0）*/
    int32_t ldt;       /* 无差拍增益 L_pu/dt_pu = Ls·I_B/(V_B·dt)，Q24（可 >1）*/
    int32_t ldr;       /* Ldt − R_pu（一步无差拍的反馈系数），Q24 */
    int32_t dtl;       /* dt_pu/L_pu = V_B·dt/(Ls·I_B)，Q15 */
    int32_t wb_rad;    /* ω_B = 2π·freq_max，rad/s（频率基准）*/
    int32_t dt_pu;     /* dt·ω_B */
    int32_t g1, g2;    /* ESO 增益 3·wb·dt、3·(wb·dt)²·L_pu，Q15 */
    int32_t eso_wb;    /* 上次设置的 ESO 带宽（rad/s），用于判断要不要复位估算器 */
    int32_t we_k;      /* speed → we_pu 的乘法系数 2^31/ω_Bc，Q16 */
    int32_t speed_max;   /* 基速 ω_Bc：此处 we_pu = 1，即反电势 = V_B，count/s */
    int32_t i_max;     /* 输入电流上限：Ldt·i、Ldr·i、L·i、R·i 乘积 < 2^31 */
    int32_t arg_max;   /* ESO 电压残差上限：dtl·arg < 2^31 */
    int32_t v_act;     /* 实测母线，Q15 标幺（32767 = ADC 满量程对应母线）*/
    int32_t v_lim;     /* 输出电压限幅，Q15（≤1.0 pu） */
    int32_t v_lim_act; /* 按实测母线折算后的内部限幅 */
    int32_t lim_inv;   /* 64·2^15/lim_act：查表索引的比例系数 */
    int32_t out_k;     /* V_B/(v_act/√3)，Q15：内部 pu(V_B) → 对外 pu */
    uint8_t eso_on;    /* 1: ESO 使能 */
} pdob_prm_t;

/* 运行态 */
typedef struct {
    int32_t id_ref, iq_ref;                 /* 电流指令，Q15 pu */
    int32_t ud_ref, uq_ref;                 /* Ldt·i_ref，指令变化时算一次 */
    int32_t iq_out;                         /* 对外返回的 iq_ref */
    int32_t we;                             /* we/ω_B，速度环算好缓存 */
    int32_t id_hat, iq_hat;                 /* 电流估计（Q24 pu，超前一拍） */
    int32_t d_hat, q_hat;                   /* 扰动电压估计（Q24 pu） */
    int32_t d_dist, q_dist;                 /* 本拍扰动补偿（Q15 pu） */
    int32_t ud_prev, uq_prev, ud_prev2, uq_prev2;   /* 上一拍/上上拍电压 */
    int32_t v_excess;                       /* 上一拍电压余量（Q15，>0 = 超出限幅圆）*/
    uint8_t manual, inited;                 /* manual: 纯电流环模式；inited: 估计器已对齐 */
} pdob_st_t;

static pdob_prm_t prm_mem;
static pdob_st_t  st_mem;
static pdob_prm_t *prm = &prm_mem;
static pdob_st_t  *st  = &st_mem;
static pdob_dob_t  dob_mem;
static pdob_dob_t *dob = &dob_mem;      /* 速度环 DOB+P 的子对象 */

/* Q15 × Q15 → 右移 sh 位（带四舍五入）：sh = Q_SIG 得 Q15，
 * sh = 2·Q_SIG − Q_ST 得 Q24（ESO 积分项）。乘积 32 位装得下。*/
static int32_t mul_q15(int32_t a, int32_t b, int32_t sh)
{
    return (a * b + (1 << (sh - 1))) >> sh;
}

/* Q24 系数 × Q15 信号 → Q15：乘积要 39 位，用 64 位中间量 */
static int32_t mul_q24(int32_t co, int32_t x)
{
    return (int32_t)(((int64_t)co * x + (1 << (Q_CO - 1))) >> Q_CO);
}

/* 无差拍前馈项 Ldt·i_ref：指令变化时算一次，热路径直接用 */
static void ref_update(void)
{
    st->ud_ref = mul_q24(prm->ldt, st->id_ref);
    st->uq_ref = mul_q24(prm->ldt, st->iq_ref);
}

#define DOB_D_MAX        30000          /* 扰动限幅上限（Q15 iq，≈0.92 pu）*/
#define FW_ON_Q15        27853          /* 弱磁启用门槛：反电势 = we·ψ_pu 达到 0.85 pu */

/* count/s → pu Q24（1.0 = base）；输入先夹到基准，保证 |w| ≤ 1 */
static inline int32_t speed_cps2pu(int32_t speed)
{
    return (int32_t)(((int64_t)lt_clamp_i32(speed,  dob->base, -dob->base) * dob->k_q) >> dob->k_sh);
}

/* 速度环配置（init 调用一次）*/
static void speed_init(int32_t acc_per_iq, uint32_t freq, int32_t speed_max, int32_t d_limit_iq)
{
    int32_t acc  = (acc_per_iq > 0) ? acc_per_iq : 1;   /* 0 会让后面的除法踩雷 */
    int32_t base = (speed_max  > 0) ? speed_max  : 1;
    int32_t s;

    memset(dob, 0, sizeof(pdob_dob_t));         /* dob 独立于 prm/st，自己清零 */
    dob->freq = freq ? freq : 1u;
    dob->base = base;

    /* 输出 / 扰动限幅：对外 Q15 iq → 内部 Q24 iq */
    dob->out_lim_q24 = (int32_t)prm->i_max << SH_SPD;
    dob->d_max_q24   = (int32_t)(d_limit_iq > DOB_D_MAX ? DOB_D_MAX : d_limit_iq) << SH_SPD;

    /* count/s → pu Q24：k_q = 2^(Q_SPD+s)/base，s 取到 k_q 不顶 int32 */
    s = 30;
    while (s > 0 && ((int64_t)1 << (Q_SPD + s)) / base >= INT32_MAX) s--;
    dob->k_q  = (int32_t)(((((int64_t)1 << (Q_SPD + s))) + (base >> 1)) / base);
    dob->k_sh = (uint8_t)s;

    /* C = 每拍、每 pu iq 产生的 pu 加速度（Q24）：
     *   1 pu iq = 32767 LSB，1 pu 速度 = base count/s ⇒ C = acc·32767/(freq·base)
     * 分两步算，避免 acc 直接左移 39 位把 int64 顶掉。*/
    dob->c_q24 = (int32_t)(((((int64_t)acc << Q_SIG) / dob->freq) << Q_SPD) / base);
    if (dob->c_q24 < 1) dob->c_q24 = 1;
}

static void speed_set(int32_t kp, int32_t wb, int32_t out_limit)
{
    int64_t kd, kpd;

    /* 对外 kp 口径不变（Q15 量纲），内部折算到 pu Q24。kpd 用 64 位算，
     * 超出 Q24 可表示范围就封顶（同 lt_pid：增益格式由调用方定，库内部调整）。
     * kp24 = kp·base/(freq·64) */
    kpd = lt_clamp_i64((int64_t)kp * dob->base / ((int64_t)dob->freq * 64),
                    INT32_MAX, -INT32_MAX);
    dob->kp_q24 = (int32_t)kpd;

    /* 输出限幅：iq 是 1.0 pu 信号，上界就是满量程 */
    if (out_limit > 0) dob->out_lim_q24 = lt_clamp_i32(out_limit,  Q15_ONE, -Q15_ONE) << SH_SPD;

    if (wb <= 0) {                          /* 关闭扰动补偿：只剩纯 P 环 */
        dob->en         = 0;
        dob->kd_q24     = 0;
        dob->d_q24      = 0;
        dob->w_prev_q24 = 0;
        return;
    }
    if (wb > (int32_t)dob->freq) wb = (int32_t)dob->freq;   /* K = wb·dt ≤ 1 */

    /* kd = K/C（Q24）：C 很小时封顶，热路径乘积仍在 64 位内 */
    kd = ((int64_t)wb << Q_SPD) / dob->freq * ((int64_t)1 << Q_SPD) / dob->c_q24;
    dob->kd_q24 = (kd > INT32_MAX) ? INT32_MAX : (int32_t)kd;
    dob->en = 1;
}

static void fw_step(void);

/* 速度环一拍：速度（count/s）+ 实测 iq（Q15 pu）→ iq 指令（Q15 pu）*/
static int32_t speed_update(int32_t speed, int32_t iq)
{
    int32_t w_q24 = speed_cps2pu(speed);
    int64_t t;

    iq = lt_clamp_i32(iq,  prm->i_max, -prm->i_max);

    if (dob->en) {
        int32_t dw    = w_q24 - dob->w_prev_q24;                  /* 实测加速度 */
        int32_t acc   = (iq << SH_SPD) + dob->d_q24;              /* 控制 + 扰动 */
        int32_t model = (int32_t)(((int64_t)dob->c_q24 * acc) >> Q_SPD);
        int32_t err   = dw - model;

        t = (int64_t)dob->d_q24 + (((int64_t)dob->kd_q24 * err) >> Q_SPD);
        t = lt_clamp_i64(t, dob->d_max_q24, -dob->d_max_q24);        /* 64 位域夹限 */
        dob->d_q24 = (int32_t)t;
    }
    dob->w_prev_q24 = w_q24;

    /* 控制律：iq_ref = kp·(ref − w) − d */
    t = (((int64_t)dob->kp_q24 * (dob->ref_q24 - w_q24)) >> Q_SPD) - dob->d_q24;
    int32_t lim = dob->out_lim_q24;
    if (dob->fw_ki) {   /* 弱磁占用的电流从 iq 预算里扣掉 */
        lim -= (int32_t)((dob->id_fw_q15 < 0 ? -(int32_t)dob->id_fw_q15
                                                : dob->id_fw_q15) << SH_SPD);
        if (lim < 0) lim = 0;
    }
    t = lt_clamp_i64(t, lim, -lim);
    
    return (int32_t)(t >> SH_SPD);
}

/* t(r) = 0.995/√(1+r²)，r = k/64，Q15：圆形限幅的修正系数。
 * 统一乘 0.995 是为了抵消最近邻查表的正误差 —— 查表结果只会略保守，不会越圆。*/
#define LIM_IDX_N  64               /* 圆限幅查表点数：r = idx/LIM_IDX_N 覆盖 [0,1] */
static const int16_t lim_rsqrt[LIM_IDX_N + 1] = {
    32603, 32599, 32587, 32567, 32540, 32504, 32461, 32410,
    32351, 32285, 32212, 32132, 32045, 31951, 31850, 31743,
    31630, 31510, 31385, 31255, 31119, 30978, 30832, 30682,
    30527, 30368, 30206, 30039, 29870, 29697, 29521, 29342,
    29161, 28978, 28792, 28605, 28416, 28226, 28034, 27841,
    27647, 27453, 27258, 27062, 26866, 26670, 26474, 26278,
    26083, 25887, 25692, 25498, 25304, 25111, 24918, 24727,
    24536, 24347, 24159, 23971, 23785, 23600, 23417, 23235,
    23054,
};
/* 电压矢量限幅：全在局部量里算，输出指针只读一次、写一次。
 * 只有 (|ud|+|uq|) > limit 才进；进来先判圆内（圆内不动），
 * 真圆外才按最大分量归一、再乘查表系数 t = 1/√(1+r²)（r = min/limit）：
 * 归一后最大分量恰为 limit，所以索引只需 mul_q15(sml, lim_inv)（bus_set 预算好），
 * 热路径没有开方、没有除法，全程 Q15。*/
static void limit_vec(int32_t *Ud, int32_t *Uq, int32_t limit)
{
    int32_t ud  = *Ud;
    int32_t uq  = *Uq;
    int32_t ud_abs = ud < 0 ? -ud : ud;
    int32_t uq_abs = uq < 0 ? -uq : uq;
    int32_t sml, t, idx;

    if (limit <= 0 || (ud_abs + uq_abs) <= limit) return;    /* L1 内 ⇒ 圆内：最常见的一条比较 */

    if (ud_abs < limit && uq_abs < limit) {                  /* 两分量都没越限，先判圆 */
        if (ud_abs * ud_abs + uq_abs * uq_abs <= limit * limit) return;   /* 圆内：不动（平方不溢出）*/
    }

    t = (int32_t)(((uint32_t)limit << Q_SIG) / (uint32_t)(ud_abs > uq_abs ? ud_abs : uq_abs));  /* 最大分量 → limit */
    ud = (ud * t) >> Q_SIG;
    uq = (uq * t) >> Q_SIG;

    ud_abs = ud < 0 ? -ud : ud;                      /* 归一后 max == limit */
    uq_abs = uq < 0 ? -uq : uq;
    sml = ud_abs > uq_abs ? uq_abs : ud_abs;
    if (sml == 0) { *Ud = ud; *Uq = uq; return; }    /* 纯轴向量：已在圆上 */

    idx = mul_q15(sml, prm->lim_inv, Q_SIG);         /* = 64·sml/limit */
    if (idx > LIM_IDX_N) idx = LIM_IDX_N;
    t = lim_rsqrt[idx];
    *Ud = (ud * t) >> Q_SIG;
    *Uq = (uq * t) >> Q_SIG;
}

/* 母线折算：入参就是母线 Q15 标幺（32767 = ADC 满量程对应的 v_base_mV），
 * 即调制上限相对相电压基准 V_B 的比值，不需要再除一次——调用方从 ADC 直接归一化即可。
 * 内部电压是 pu(V_B)，对外折算回 1.0 pu = 实测母线/√3。*/
__attribute__((noinline))
static void bus_set(int32_t v_bus)
{
    int32_t r = v_bus;

    if (r <= 0) return;                                      /* 未测到母线：保持原折算 */
    if (r > (8 * SIG_FS)) r = 8 * SIG_FS;                          /* 超过 8 倍量程按 8 倍处理 */
    prm->v_act = r;
    prm->out_k = (int32_t)(((int64_t)SIG_FS * SIG_FS) / r);          /* 内部 pu → 对外 pu */
    prm->v_lim_act = (r < Q15_ONE) ? mul_q15(prm->v_lim, r, Q_SIG) : prm->v_lim;
    if (prm->v_lim_act < 1) prm->v_lim_act = 1;              /* 母线塌了也要走限幅 */
    /* 弱磁启用阈值：反电势 we·ψ_pu 触及 0.85 倍当前限幅（母线变了自动跟随）；
     * 结果 >1.0pu 说明这条母线下用不到弱磁，夹到 SIG_FS 即永不启用 */
    {
        int64_t wo = ((int64_t)mul_q15(FW_ON_Q15, prm->v_lim_act, Q_SIG) << (Q_CO - Q_SIG))
                     / (prm->psi_pu > 0 ? prm->psi_pu : 1);
        dob->we_on = (wo > SIG_FS) ? SIG_FS : (int32_t)wo;
    }
    prm->lim_inv = (int32_t)((((int64_t)LIM_IDX_N * SIG_FS) + (prm->v_lim_act >> 1))
                             / prm->v_lim_act);              /* 索引比例 64/limit */
}

/* ---- 配置期单位换算常数（只在 init/curr_set/母线变化时用，热路径不碰）---- */
#define SQRT3_X1000  1732           /* √3×1000 */
#define VB_RATIO     3              /* V_B = VB_RATIO·母线/√3：1.0pu 覆盖 3 倍满量程母线 */
#define TAU_E6       6283185LL      /* 2π×1e6：ω_B = 2π·f_max */
#define E6           1000000LL
#define TAU_Q16      411775LL       /* 2π×2^16：count/s ↔ we_pu 换算 */
#define TAU_ONE      205887LL       /* 2π×32767：速度环 acc 的单位换算常数（2π·Q15_ONE）*/

/* ================================ 配置 ================================ */
/* 上电 / 换电机调一次：按物理量重算全部标幺系数 */
void lt_pdob_init(const lt_pdob_config_t *c)
{
    int64_t wb, dtq, wbc, k;
    int32_t reso  = c->reso          ? c->reso        : 4096u;
    int32_t pp    = c->pp            ? (int32_t)c->pp : 2;
    int32_t fc    = c->freq_curr      ? (int32_t)c->freq_curr : 1;
    int32_t ib    = c->i_base_mA > 0 ? c->i_base_mA   : 10000;
    int32_t psi   = c->phi_uWb   > 0 ? c->phi_uWb     : 1;        /* ψ、Ls 是基准分母 */
    int32_t ls    = c->Ls_uH     > 0 ? c->Ls_uH       : 1;
    int32_t vbase = c->v_base_mV > 0 ? c->v_base_mV   : 72000;    /* 默认 72V 母线满量程 */
    int32_t fmax  = c->freq_max      ? (int32_t)c->freq_max : 1;  /* 频率基准：最高电频率 Hz */

    memset(&st_mem, 0, sizeof(st_mem));
    memset(&prm_mem, 0, sizeof(prm_mem));

    int32_t v_ph_mV = (int32_t)(((int64_t)vbase * (1000 * VB_RATIO) + (SQRT3_X1000 / 2)) / SQRT3_X1000);  /* V_B = 3·最大母线/√3 */
    wb              = ((int64_t)TAU_E6 * fmax + E6 / 2) / E6;     /* ω_B = 2π·f_max */
    if (wb < 1) wb  = 1;
    if (wb > (INT32_MAX / 2)) wb = (INT32_MAX / 2);
    dtq = (wb * SIG_FS + (fc >> 1)) / fc;                        /* dt·ω_B（+fc/2 是四舍五入）*/
    prm->wb_rad = (int32_t)wb;
    prm->dt_pu  = (int32_t)dtq;

    /* 标幺电机参数：R_pu = Rs·I_B/V_B、L_pu = Ls·ω_B·I_B/V_B、ψ_pu = ψ·ω_B/V_B */
    prm->r_pu = (int32_t)(((int64_t)c->Rs_mohm * ib * SIG_FS)
                          / ((int64_t)v_ph_mV  * 1000));
    prm->l_pu = (int32_t)(((int64_t)ls  * ib   * wb * SIG_FS)
                          / ((int64_t)v_ph_mV  * 1000000LL));
    prm->psi_pu = (int32_t)(((int64_t)psi * wb << Q_CO)
                            / ((int64_t)v_ph_mV * 1000));        /* Q24 */
    prm->ldt = (int32_t)(((int64_t)prm->l_pu << Q_CO) / dtq);    /* Ldt，Q24 */
    prm->ldr = prm->ldt - (prm->r_pu << (Q_CO - Q_SIG));         /* Ldt − R_pu，Q24 */
    prm->dtl = (int32_t)((dtq * SIG_FS + (prm->l_pu >> 1)) / prm->l_pu);   /* dt_pu/L_pu（带四舍五入）*/

    /* 夹限：arg（ESO 电压残差）仍走 32 位乘，保留溢出夹限；
     * 电流上限就是物理满量程 1 pu（信号全 ≤1 pu，Ldt 项已用 64 位乘）*/
    prm->arg_max = (int32_t)(MUL_TOP / (prm->dtl > 0 ? prm->dtl : 1));
    prm->i_max   = Q15_ONE;

    /* 速度换算：ω_Bc = we_pu 为 1 时的 count/s（基速），we_pu = speed·32768/ω_Bc */
    wbc = ((int64_t)wb * reso * (SIG_FS * 2)) / (TAU_Q16 * pp);
    if (wbc < 1) wbc = 1;
    k = (((int64_t)SIG_FS << Q_WEK) + (wbc >> 1)) / wbc;                /* +wbc/2 是四舍五入 */
    prm->we_k    = (int32_t)k;
    prm->speed_max = (int32_t)((MUL_TOP - SIG_FS) / k);            /* 扣掉 Q16 四舍五入位 */
    if (prm->speed_max < 1) prm->speed_max = 1;

    /* 弱磁启用阈值随母线走，在 bus_set() 里算：0.85·v_lim_act/ψ_pu */
    
    prm->g1 = 0;
    prm->g2 = 0;
    prm->eso_on = 0;
    prm->v_lim = Q15_ONE / VB_RATIO;  /* 1.0pu(V_B) = VB_RATIO 倍线性边界 */
    prm->out_k = Q15_ONE;
    bus_set(Q15_ONE);                               /* 上电按满量程母线初始化折算 */

    /* 速度环模型增益（Kt 按幅值不变 dq 约定 Te = 1.5·pp·ψ·iq）：
     *   Kt  = 1.5·pp·ψ                     [µN·m/A]
     *   acc = Kt·I_B/J·(reso/2π)/32767     [count/s² per Q15 iq]
     * 单位换算：µN·m/A × mA / nN·m·s² 的 1e-6·1e-3/1e-9 恰好相消，
     * 故 acc = Kt·I_B·reso/(J·2π·2^15)，常数 205887 = 2π·32767。*/
    int64_t kt = 3LL * pp * psi / 2;                     /* µN·m/A */
    int64_t jj = c->J_nNm_s2 > 0 ? c->J_nNm_s2 : 1;        /* nN·m·s² */
    int32_t acc = (int32_t)((kt * ib * (int64_t)reso / jj + TAU_ONE / 2) / TAU_ONE);

    speed_init(acc, c->freq_speed, c->speed_max, c->d_limit_iq);
    bus_set(prm->v_act);            /* speed_init 清了 dob，母线折算要在其后重算 */
    lt_pdob_reset();
}

void lt_pdob_speed_set(int32_t kp, int32_t wb, int32_t out_limit)
{
    /* kp 对外 Q15；wb=0 退化为纯 P 环。内部折算到 pu Q24 在 speed_set 里做 */
    speed_set(kp, wb, out_limit);
}

/* 弱磁积分增益（Q15），0 = 关闭。
 * 口径：v_excess = (|v|² − 限幅²)，|v| 与限幅都是 pu(V_B)，故同一物理工况下
 * v_excess ∝ 1/V_B²，fw_ki 的标定值也就与 V_B² 成反比：换硬件（改 v_base_mV /
 * VB_RATIO）时按 (V_B_old/V_B_new)² 反比折算；同一硬件调一次即可
 * （弱磁环增益本身还随转速 we 变，最终以台架为准）。*/
void lt_pdob_fw_set(int32_t ki)
{
    dob->fw_ki     = lt_clamp_i32(ki, Q15_ONE, 0);        /* 夹到 [0, 1.0pu] */
    dob->id_fw_q15 = 0;
}

void lt_pdob_curr_set(int32_t eso_wb, int32_t out_limit)    /* eso_wb=0 关 ESO */
{
    if (eso_wb > 0) {
        int64_t wp = ((int64_t)eso_wb << Q_SIG) / (prm->wb_rad > 0 ? prm->wb_rad : 1);
        prm->eso_on = 1;                                   /* wb_pu = wb/ω_B */
        prm->g1 = (int32_t)(3 * wp * prm->dt_pu / SIG_FS);
        prm->g2 = (int32_t)(3 * wp * wp / SIG_FS * prm->dt_pu / SIG_FS
                            * prm->l_pu / SIG_FS);
    } else {
        prm->eso_on = 0;
        prm->g1 = 0;
        prm->g2 = 0;
    }
    if (prm->g1 > G_LIM) prm->g1 = G_LIM;                  /* 增益上限：带宽 ≈ 1/(6dt) */
    if (prm->g2 > G_LIM) prm->g2 = G_LIM;
    if (out_limit > 0) prm->v_lim = lt_clamp_i32(out_limit / VB_RATIO,  Q15_ONE / VB_RATIO, -Q15_ONE / VB_RATIO);

    if (eso_wb != prm->eso_wb) {        /* 带宽/开关变了：估算状态作废 */
        st->d_hat  = 0;                 /* 下一拍以实测电流重新对齐（inited=0）*/
        st->q_hat  = 0;
        st->d_dist = 0;
        st->q_dist = 0;
        st->inited = 0;
        prm->eso_wb = eso_wb;
    }

    bus_set(prm->v_act);
}

void lt_pdob_set_speed_ref(int32_t speed_ref)   /* 速度指令（count/s），退出纯电流环模式 */
{
    dob->ref_q24 = speed_cps2pu(speed_ref);
    st->manual = 0;
}

void lt_pdob_set_curr_ref(int32_t id_ref, int32_t iq_ref)   /* 手动给定 id/iq（Q15），纯电流环模式 */
{
    st->id_ref = lt_clamp_i32(id_ref,  prm->i_max, -prm->i_max);
    st->iq_ref = lt_clamp_i32(iq_ref,  prm->i_max, -prm->i_max);
    ref_update();
    st->manual = 1;
}

int32_t lt_pdob_speed_update(int32_t speed, int32_t iq)
{
    st->we = mul_q15(lt_clamp_i32(speed,  prm->speed_max, -prm->speed_max), prm->we_k, Q_WEK);
    if (!st->manual) {
        st->iq_out = speed_update(speed, iq);     /* 内部已按 out_lim 限幅（默认 1 pu）*/
        st->iq_ref = st->iq_out;
        fw_step();                                /* 弱磁：可能改写 st->id_ref */
        ref_update();
    }
    return st->iq_out;
}

/* =============================== 热路径 =============================== */
/* 电压基准下的反电势 we·ψ_pu（ψ_pu 是 Q24 系数）*/
static inline int32_t we_psi_of(int32_t we) { return mul_q24(prm->psi_pu, we); }

/* 弱磁一拍（速度拍调用）：电压余量积分，把 id 拉到电压限幅圆上。
 * 反电势 we·ψ_pu 达到 0.85 pu 才启用；低速严格为 0，一点铜损不加。
 * id_fw ≤ 0，夹到 -(i_max - |iq|)（L1 保守），iq 侧再由 speed_update 让位。*/
static void fw_step(void)
{
    int32_t id_fw_q15 = dob->id_fw_q15;            /* 弱磁输出 */
    if (dob->fw_ki == 0 || st->manual) return;
    if (st->we < dob->we_on && id_fw_q15 == 0) return;   /* 退出时让它自然跑回 0 */

    /* fw_ki ≤ 32767、v_excess ≤ 32767，32 位乘积装得下 */
    id_fw_q15 -= (dob->fw_ki * st->v_excess) >> Q_SIG;
    int32_t lo = -(prm->i_max);                   /* 总电流预算；iq 让位在 speed_update 里做 */
    if (id_fw_q15 > 0)  id_fw_q15 = 0;
    if (id_fw_q15 < lo) id_fw_q15 = lo;
    st->id_ref = id_fw_q15;
    dob->id_fw_q15 = id_fw_q15;
}

/* ESO 一拍：ud/uq 为上一拍实际电压，id/iq 为本拍实测，we 为电角速度 */
static void eso_step(int32_t ud, int32_t uq, int32_t id, int32_t iq, int32_t we)
{
    int32_t id_hat, iq_hat, d_hat, q_hat;   /* 估计值（Q24），只在出入口碰 st-> */
    int32_t id_sig, iq_sig;                 /* 同一拍的 Q15 换算：只做一次 */
    int32_t eps_d, eps_q, li_d, li_q, arg_d, arg_q, we_psi;

    if (!prm->eso_on) {
        st->d_dist = 0;
        st->q_dist = 0;
        return;
    }
    id_hat = st->id_hat;
    iq_hat = st->iq_hat;
    d_hat  = st->d_hat;
    q_hat  = st->q_hat;
    if (!st->inited) {                      /* 首次用实测对齐，避免起步冲击 */
        id_hat = sig2st(id);
        iq_hat = sig2st(iq);
        st->inited = 1;
    }
    /* 旋转后的 Q24 状态在残差、L·i、R·i 里各用一遍，先一次换到 Q15 信号域 */
    id_sig = st2sig(id_hat);
    iq_sig = st2sig(iq_hat);

    eps_d = lt_clamp_i32(id - id_sig,  EPS_LIM, -EPS_LIM);
    eps_q = lt_clamp_i32(iq - iq_sig,  EPS_LIM, -EPS_LIM);

    li_d   = mul_q15(prm->l_pu, id_sig, Q_SIG);   /* L·i */
    li_q   = mul_q15(prm->l_pu, iq_sig, Q_SIG);
    we_psi = we_psi_of(we);

    /* 电压残差：u − R·i − we×L·i − we·ψ + 已有扰动估计 */
    arg_d = lt_clamp_i32(ud - mul_q15(prm->r_pu, id_sig, Q_SIG)
                         + mul_q15(li_q, we, Q_SIG) + st2sig(d_hat),  prm->arg_max, -prm->arg_max);
    arg_q = lt_clamp_i32(uq - mul_q15(prm->r_pu, iq_sig, Q_SIG)
                         - mul_q15(li_d, we, Q_SIG) - we_psi + st2sig(q_hat),  prm->arg_max, -prm->arg_max);

    /* 状态推进（Q24）：î += dtl·arg + g1·eps，d̂ += g2·eps */
    id_hat = lt_clamp_i32(id_hat + mul_q15(prm->dtl, arg_d, SH_MUL)
                              + mul_q15(prm->g1,  eps_d, SH_MUL),  sig2st(prm->i_max), -sig2st(prm->i_max));
    iq_hat = lt_clamp_i32(iq_hat + mul_q15(prm->dtl, arg_q, SH_MUL)
                              + mul_q15(prm->g1,  eps_q, SH_MUL),  sig2st(prm->i_max), -sig2st(prm->i_max));
    d_hat  = lt_clamp_i32(d_hat  + mul_q15(prm->g2,  eps_d, SH_MUL),  sig2st(Q15_ONE), -sig2st(Q15_ONE));
    q_hat  = lt_clamp_i32(q_hat  + mul_q15(prm->g2,  eps_q, SH_MUL),  sig2st(Q15_ONE), -sig2st(Q15_ONE));

    st->id_hat = id_hat;
    st->iq_hat = iq_hat;
    st->d_hat  = d_hat;
    st->q_hat  = q_hat;
    st->d_dist = st2sig(d_hat);     /* 注意：此处 d̂/q̂ 已推进过，与残差里那次不是同一个值，不能合并 */
    st->q_dist = st2sig(q_hat);
}

/* 电流环一拍：实测 id/iq/母线（Q15）→ Vd/Vq（Q15，1.0 pu = 实测母线/√3）*/
void lt_pdob_curr_update(int32_t id, int32_t iq, int32_t vbus, int32_t *vd, int32_t *uq)
{
    int32_t id_fb, iq_fb;
    int32_t we = st->we;                    /* 本拍电角速度，速度环里已算好 */

    id = lt_clamp_i32(id,  prm->i_max, -prm->i_max);
    iq = lt_clamp_i32(iq,  prm->i_max, -prm->i_max);
    if (vbus > 0 && vbus != prm->v_act) bus_set(vbus);   /* 母线变了才重算折算系数 */

    eso_step((st->ud_prev + st->ud_prev2) >> 1, (st->uq_prev + st->uq_prev2) >> 1,
             id, iq, we);                   /* 电压取两拍平均，抑制奈奎斯特翻转 */

    if (prm->eso_on) {                      /* 无差拍作用在超前一拍的估计上 */
        id_fb = lt_clamp_i32(st2sig(st->id_hat),  prm->i_max, -prm->i_max);
        iq_fb = lt_clamp_i32(st2sig(st->iq_hat),  prm->i_max, -prm->i_max);
    } else {
        id_fb = id;
        iq_fb = iq;
    }

    int32_t li_d   = mul_q15(prm->l_pu, id_fb, Q_SIG);      /* L·i，dq 交叉耦合项用 */
    int32_t li_q   = mul_q15(prm->l_pu, iq_fb, Q_SIG);
    int32_t we_psi = we_psi_of(we);                         /* we·ψ_pu */

    /* 无差拍：u = Ldt·i_ref − (Ldt−R_pu)·i_fb − we×L·i − we·ψ − f̂ */
    int32_t v_d = st->ud_ref - mul_q24(prm->ldr, id_fb) - mul_q15(li_q, we, Q_SIG) - st->d_dist;
    int32_t v_q = st->uq_ref - mul_q24(prm->ldr, iq_fb) + mul_q15(li_d, we, Q_SIG)
                + we_psi - st->q_dist;

    if (dob->fw_ki) {   /* 弱磁驱动：用限幅前的需求算 L2 圆余量（L1 菱形比圆大，最多多弱磁 √2 倍）*/
        int32_t aa = v_d < 0 ? -v_d : v_d, bb = v_q < 0 ? -v_q : v_q, limv = prm->v_lim_act;
        if (aa > Q15_ONE) aa = Q15_ONE;         /* 需求可远超 1pu，先夹再平方，否则 int32 乘积溢出 */
        if (bb > Q15_ONE) bb = Q15_ONE;
        st->v_excess = ((aa * aa + bb * bb) - limv * limv) >> Q_SIG;   /* ≤ 2·ONE² < 2^31 */
    }
    limit_vec(&v_d, &v_q, prm->v_lim_act);          /* 限幅在内部 pu（V_B）上做 */

    st->ud_prev2 = st->ud_prev;                     /* ESO 下一拍用真实电压（两拍平均）*/
    st->uq_prev2 = st->uq_prev;
    st->ud_prev  = v_d;
    st->uq_prev  = v_q;

    /* pu(V_B) → pu(实测母线/√3)：×VB_RATIO；v_lim ≤ Q15_ONE/3 时上界恰为 32767 */
    *vd = lt_clamp_i32(VB_RATIO * mul_q15(v_d, prm->out_k, Q_SIG),  Q15_ONE, -Q15_ONE);
    *uq = lt_clamp_i32(VB_RATIO * mul_q15(v_q, prm->out_k, Q_SIG),  Q15_ONE, -Q15_ONE);
}

/* 清运行状态（估计器、电压历史、we、manual）；配置保留 */
void lt_pdob_reset(void)
{
    memset(&st_mem, 0, sizeof(st_mem));
    dob->d_q24      = 0;                    /* 速度环观测器状态 */
    dob->w_prev_q24 = 0;
    dob->id_fw_q15  = 0;                    /* 弱磁输出清零 */
    st->v_excess    = 0;
}
