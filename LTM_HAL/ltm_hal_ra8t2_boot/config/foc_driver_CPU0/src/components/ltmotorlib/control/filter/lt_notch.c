// lt_notch.c

#include "control/filter/lt_notch.h"
#include "math/basic/lt_math.h"
#include <math.h>
#include <string.h>

/* 一般 1~2 个齿轮谐振 + 1 个联轴器谐振 */
#define MAX_STAGES  3u          /* 支持三级抑制 */

typedef struct {
    float b0, b1, b2;           /* 分子系数 (a0 归一化为 1) */
    float a1, a2;               /* 分母系数 (a0=1) */
    float s1, s2;               /* DF2T 延迟状态，实现状态压缩（2个变量即可） */
    uint8_t enable;             /* 该级是否启用 */
} stage_t;

typedef struct {
    float ts_s;                 /* 采样周期（s）*/
    stage_t stages[MAX_STAGES];
} lt_notch_obj;

static lt_notch_obj notch_obj;
static lt_notch_obj *notch = &notch_obj;

/*
 * 单级系数设计：双线性变换 + 频率预畸
 * 连续域: H(s) = (s^2 + 2*xi_z*wn*s + wn^2) / (s^2 + 2*xi_p*wn*s + wn^2)
 *         xi_p = 1/(2Q), xi_z = xi_p * 10^(depth/20)
 * 双线性变化 s = (2/T) * (1 - z⁻¹)/(1 + z⁻¹) ==> 
 * H(z) = (b2*z^-2 + b1*z^-1 + b0)/(a2*z^-2 + a1*z^-1 + a0)
 */
static void stage_design(stage_t *st, float fc_Hz, float Q, float depth_dB, float ts_s)
{
    st->s1 = 0.0f;
    st->s2 = 0.0f;

    if (fc_Hz <= 0.0f || Q <= 0.0f || fc_Hz > (0.5f/ts_s)){ /* 超过 Nyquist频率，直接返回 */
        st->enable = 0;         
        st->b0 = 1.0f; st->b1 = 0.0f; st->b2 = 0.0f;
        st->a1 = 0.0f; st->a2 = 0.0f;
        return;
    }

    float xi_p = 1.0f / (2.0f * Q);
    float xi_z = (depth_dB <= -60.0f) ? 0.0f : xi_p * powf(10.0f, depth_dB / 20.0f);
    /* 双线性变换会导致频率偏移（连续域到离散域时）*/
    float wn = _2_PI * fc_Hz;
    float K  = 2.0f / ts_s;
    float wa = K * tanf(0.5f * wn * ts_s);    /* 预畸公式：把数字频率 wn 映射到模拟频率 wa */
    float wa2 = wa * wa;                      /* 在连续域上，针对 wa 设计滤波器，双线性变换后，陷波器刚好作用到 wn */
    float K2  = K * K;

    /* 开始双线性变换，并进行系数归一化（a0归一化到1）处理 */
    float inv_a0 = 1.0f / (wa2 + 2.0f * xi_p * wa * K + K2);
    st->b0 = (wa2 + 2.0f * xi_z * wa * K + K2) * inv_a0;
    st->b1 = (2.0f * (wa2 - K2)) * inv_a0;
    st->b2 = (wa2 - 2.0f * xi_z * wa * K + K2) * inv_a0;
    st->a1 = (2.0f * (wa2 - K2)) * inv_a0;
    st->a2 = (wa2 - 2.0f * xi_p * wa * K + K2) * inv_a0;

    st->enable = 1;
}

void lt_notch_init(float ts_s)
{
    memset(notch, 0, sizeof(lt_notch_obj));
    notch->ts_s = (ts_s > 1e-9f) ? ts_s : 40e-6f;
}

void lt_notch_set(uint8_t level, float fc_Hz, float Q, float depth_dB)
{
    if (level >= MAX_STAGES) return;
    stage_design(&notch->stages[level], fc_Hz, Q, depth_dB, notch->ts_s);
}

void lt_notch_reset(void)
{
    for (uint32_t i = 0; i < MAX_STAGES; i++) {
        notch->stages[i].s1 = 0.0f;
        notch->stages[i].s2 = 0.0f;
    }
}

float lt_notch_process(float x)
{
    float y = x;                /* 上一级的输出 */
    for (uint32_t i = 0; i < MAX_STAGES; i++) {
        stage_t *st = &notch->stages[i];
        if (!st->enable) continue;

        /* DF2T: y[n] = b0*x[n] + s1[n] （实现了状态压缩，只需保存2个变量）
                      = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*[yn-2] */
        /*       s1[n+1] = b1*x[n] - a1*y[n] + s2[n]，用旧的 s2 */
        /*       s2[n+1] = b2*x[n] - a2*y[n] */
        float y_i = st->b0 * y + st->s1;
        st->s1 = st->b1 * y - st->a1 * y_i + st->s2;
        st->s2 = st->b2 * y - st->a2 * y_i;
        y = y_i;
    }
    return y;
}