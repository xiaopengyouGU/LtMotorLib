/* lt_deadzone —— 电流极性法死区补偿（全定点 Q15 标幺）
 *
 * 原理：死区使实际输出电压相对指令偏移，偏移方向由相电流极性决定。
 *   每相补偿 = 极性 × 死区占空比；三相共模不产生差模电压，先剔除；
 *   再对差模分量做一阶低通（α = 1/2^shift），抑制极性翻转时的抖动。
 *
 * 标度：电流、占空比一律 Q15（32767 = 1.0 pu）
 * 热路径：整数乘 + 移位，无浮点；唯一的 /3 是编译期常量，编成乘加
 *
 * 两个参数的经验值（仿真实测）：
 *   ith   取几倍电流采样 LSB 即可（≈1e-4 pu）。取到额定的 1%~3% 会在轻载下
 *         推迟极性翻转，5/7 次补偿相位错开，畸变反而比不补偿更大
 *   alpha 越接近 1 越好（默认 1/2）。
 */
#include "control/deadzone/lt_deadzone.h"
#include <string.h>

typedef struct {
    int32_t comp[3];        /* 死区补偿占空比（差模、已滤波），Q15 */
    int8_t  dir[3];         /* 电流方向与 PWM 正占空比的关系：+1 同向，-1 反向 */
    int32_t dead_q15;       /* 死区对应占空比，Q15 */
    int32_t ith_q15;        /* 极性判别阈值，Q15 */
    int32_t gain;           /* 线性过渡斜率 dead/ith，Q15 定标（init 里算好，热路径不做除法）*/
    uint8_t alpha_sh;       /* 滤波系数 α = 1/2^alpha_sh */
    uint8_t valid;          /* 1：已初始化 */
} lt_deadzone_obj;

static lt_deadzone_obj dz_obj;
#define dz (&dz_obj)

#define DEAD_Q15_DEF    393     /* 默认死区占空比 1.2% */
#define ITH_Q15_DEF     8       /* 默认阈值 ≈0.00024 pu（33A 基准下 ≈8mA，约半个采样 LSB）
                                 * 取大会在轻载下推迟极性翻转，反而把畸变做大 */
#define ALPHA_SH_DEF    1       /* 默认 α = 1/2：6 步波要跟得上，越慢 5/7 次相位差越大 */
#define DEAD_Q15_MAX    1638    /* 死区上限 5% */

void lt_deadzone_init(int32_t dead_duty_q15, int32_t ith_q15, uint8_t alpha_shift)
{
    memset(dz, 0, sizeof(lt_deadzone_obj));
    memset(dz->dir, 1, 3);              /* 默认三相电流与 PWM 正占空比同向 */

    if (dead_duty_q15 <= 0 || dead_duty_q15 > DEAD_Q15_MAX) dead_duty_q15 = DEAD_Q15_DEF;
    if (ith_q15 < 0) ith_q15 = -ith_q15;
    if (ith_q15 == 0)      ith_q15 = ITH_Q15_DEF;
    if (ith_q15 > 32767)   ith_q15 = 32767;
    if (alpha_shift < 1)   alpha_shift = 1;
    if (alpha_shift > 8)   alpha_shift = 8;

    dz->dead_q15 = dead_duty_q15;
    dz->ith_q15  = ith_q15;
    /* 阈值内补偿 = c·dead/ith，把 dead/ith 预算成 Q15 系数，热路径只剩乘和移位。
     * 乘积有界：|c| ≤ ith ⇒ |c·gain| ≤ ith·dead·32768/ith = dead·32768，int32 装得下 */
    dz->gain     = (int32_t)(((int64_t)dead_duty_q15 << 15) / ith_q15);
    dz->alpha_sh = alpha_shift;
    dz->valid    = 1;
}

void lt_deadzone_set(int8_t dirA, int8_t dirB, int8_t dirC)
{
    dz->dir[0] = (dirA >= 0) ? 1 : -1;
    dz->dir[1] = (dirB >= 0) ? 1 : -1;
    dz->dir[2] = (dirC >= 0) ? 1 : -1;
}

/* 每电流环拍调用一次：三相电流（Q15）进，Q15 占空比就地叠加补偿 */
void lt_deadzone_compensate(int32_t ia, int32_t ib, int32_t ic, int32_t dutys[3])
{
    int32_t curr[3] = { ia, ib, ic }, raw[3];
    int32_t sum, common, half, sh;
    int i;

    if (!dz->valid || !dutys) return;

    /* ---- 1. 原始补偿：阈值外给满量，阈值内线性过渡 ----
     * 硬 sign + 滞环在电流过零时把补偿量整块翻转 2·dead，靠后面那级低通慢慢跟，
     * 跟随期间补偿方向是错的——相电流上那团毛刺就是这么来的（实测残差峰值由
     * 206mA 涨到 344mA，位置正好偏移一个低通时间常数 ~200us）。线性过渡在过零
     * 处补偿量本来就是 0，翻不翻都一样，不需要低通来擦 */
    sum = 0;
    for (i = 0; i < 3; i++) {
        int32_t c = curr[i];
        int32_t r;
        if      (c >=  dz->ith_q15) r =  dz->dead_q15;
        else if (c <= -dz->ith_q15) r = -dz->dead_q15;
        else                        r = (c * dz->gain) >> 15;
        raw[i] = (dz->dir[i] < 0) ? -r : r;
        sum   += raw[i];
    }

    /* ---- 2. 剔除共模（共模不产生差模电压）---- */
    common = sum / 3;

    /* ---- 3. 差模分量一阶低通，并就地叠加到占空比 ---- */
    sh   = dz->alpha_sh;
    half = 1 << (sh - 1);
    for (i = 0; i < 3; i++) {
        dz->comp[i] += ((raw[i] - common - dz->comp[i]) + half) >> sh;
        dutys[i]    += dz->comp[i];
    }
}
