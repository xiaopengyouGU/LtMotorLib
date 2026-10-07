/*
 * SPDX-License-Identifier: MIT
 * lt_speed 主机侧压测（PC，无硬件依赖）
 *
 * 配置：25 kHz 采样、18 位单圈编码器，与 lt_speed_init(262144, 25000) 对应。
 * 默认编译（LT_SPEED_USE_ADAP_M=0）只校验 PLL 通道，并检查接口约定
 * "adap_speed 恒为 0"；用 -DLT_SPEED_USE_ADAP_M=1 编译时两个通道都校验。
 *
 * 运行：
 *   python script.py -t                      构建并执行本压测
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "ltm_ctrl/lt_control.h"

/* ==================== 目标配置 ==================== */
#define ENCODER_CPR      262144u            /* 18 位单圈编码器 */
#define SAMPLE_HZ        25000u             /* 测速调用频率 Hz */
#define DT_S             (1.0 / (double)SAMPLE_HZ)
#define PLL_KP           1500u              /* 锁相环比例增益 1/s */
#define PLL_KI           50u                /* 锁相环积分增益 1/s² */
#define RPM_MAX          3000.0             /* 用例扫描上限 */
#define RPM_RANGE_LIMIT  (RPM_MAX * 1.4)    /* 输出越界判据 */

/* ==================== 判定阈值 ==================== */
#define MEAN_TOL_MIN     1.0                /* 均值误差下限 RPM */
#define MEAN_TOL_REL     0.02               /* 均值误差相对目标 2% */
#define PEAK_TOL_MIN     2.0                /* 单点误差下限 RPM */
#define PEAK_TOL_REL     0.05               /* 单点误差相对目标 5% */
#define SPIKE_REL        1.3                /* 超速尖峰：1.3 倍目标 */
#define SPIKE_ABS        50.0               /* 或 +50 RPM */
#define PLL_RAMP_ALLOW   12.0               /* PLL 斜坡固有跟踪滞后 RPM */
#define M_FILTER_WINDOW  18.0               /* M 法最长测速窗口（拍） */
#define M_FILTER_ALPHA   0.12               /* M 法输出滤波系数 */
#define RAMP_SLEW        6000.0             /* 斜坡加速度 RPM/s */

/* ADAP_M 关闭时，接口约定 adap_speed 恒为 0 */
#define ADAP_ACTIVE      (LT_SPEED_USE_ADAP_M != 0)

/* ==================== 确定性随机数（LCG） ==================== */
static uint32_t rng_state = 0x12345678u;

static uint32_t rng_next(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return rng_state;
}

/* ==================== 墙钟 ==================== */
static double now_sec(void)
{
#ifdef _WIN32
    static LARGE_INTEGER freq;
    static int ready = 0;
    LARGE_INTEGER cnt;
    if (!ready) { QueryPerformanceFrequency(&freq); ready = 1; }
    QueryPerformanceCounter(&cnt);
    return (double)cnt.QuadPart / (double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1e-9 * (double)ts.tv_nsec;
#endif
}

/* ==================== 编码器模拟（单圈计数契约） ==================== */
static double wrap_count(double v)
{
    v = fmod(v, (double)ENCODER_CPR);
    if (v < 0.0) v += (double)ENCODER_CPR;
    return v;
}

typedef struct {
    double pos;                             /* [0, ENCODER_CPR) */
} encoder_t;

static void encoder_init(encoder_t *enc, double pos0)
{
    enc->pos = wrap_count(pos0);
}

/* 按 rpm 推进一拍，noise 为计数噪声，返回单圈计数 */
static uint32_t encoder_step(encoder_t *enc, double rpm, double noise)
{
    enc->pos = wrap_count(enc->pos + rpm * ((double)ENCODER_CPR / 60.0) * DT_S);
    return (uint32_t)wrap_count(enc->pos + noise);
}

static void speed_start(void)
{
    lt_speed_init(ENCODER_CPR, SAMPLE_HZ);
    lt_speed_set(PLL_KP, PLL_KI);
}

/* lt_speed_get() 输出 count/s，本压测统一换算成 RPM 判定 */
static double cps_to_rpm(double cps)
{
    return cps * 60.0 / (double)ENCODER_CPR;
}

/* ==================== 指标统计 ==================== */
typedef struct {
    double adap_mean;                       /* 均值误差，后 1/2 窗口 */
    double adap_peak;                       /* 单点最大误差 */
    double adap_max_abs;                    /* 输出最大绝对值（尖峰监视） */
    double pll_mean;
    double pll_peak;
    double pll_max_abs;
    int64_t bad;                            /* 越界样本数 */
} result_t;

static double mean_tol(double target)
{
    const double t = fabs(target);
    return (t > 1.0) ? t * MEAN_TOL_REL : MEAN_TOL_MIN;
}

static double peak_tol(double target)
{
    const double t = fabs(target);
    return (t > 2.0) ? t * PEAK_TOL_REL : PEAK_TOL_MIN;
}

/* 单通道是否满足稳态判据 */
static int channel_ok(double mean_err, double peak_err, double target)
{
    return (fabs(mean_err) <= mean_tol(target)) && (peak_err <= peak_tol(target));
}

/* adap 通道判据：ADAP_M 关闭时要求恒为 0，否则与 PLL 同判据 */
static int adap_ok(const result_t *r, double target)
{
    return ADAP_ACTIVE ? channel_ok(r->adap_mean, r->adap_peak, target)
                       : (r->adap_max_abs == 0.0);
}

typedef double (*profile_fn)(double t, double param);

/* 以 pos0 起步，跑 seconds 秒；前 warmup 秒不计入统计。
 * noise > 0 时每拍叠加 ±noise 计数的随机抖动 */
static void run_profile(double pos0, double seconds, double warmup,
                        profile_fn fn, double param, double noise, result_t *r)
{
    encoder_t enc;
    encoder_init(&enc, pos0);
    speed_start();

    const int64_t total = (int64_t)(seconds * SAMPLE_HZ);
    const int64_t warm  = (int64_t)(warmup * SAMPLE_HZ);
    double sum_adap = 0.0, sum_pll = 0.0;
    int64_t n = 0;
    int32_t adap = 0, pll = 0;

    for (int64_t i = 0; i < total; i++) {
        const double rpm = fn((double)i / SAMPLE_HZ, param);
        const double jitter = (noise > 0.0)
            ? (double)(int)(rng_next() % (uint32_t)(2.0 * noise + 1.0)) - noise
            : 0.0;
        lt_speed_update(encoder_step(&enc, rpm, jitter));
        lt_speed_get(&adap, &pll);
        const double adap_rpm = cps_to_rpm((double)adap);
        const double pll_rpm  = cps_to_rpm((double)pll);

        const double a = fabs(adap_rpm), p = fabs(pll_rpm);
        if (a > RPM_RANGE_LIMIT || p > RPM_RANGE_LIMIT) r->bad++;
        if (a > r->adap_max_abs) r->adap_max_abs = a;
        if (p > r->pll_max_abs)  r->pll_max_abs  = p;

        if (i >= warm) {
            const double ea = adap_rpm - rpm, ep = pll_rpm - rpm;
            sum_adap += ea;
            sum_pll  += ep;
            if (fabs(ea) > r->adap_peak) r->adap_peak = fabs(ea);
            if (fabs(ep) > r->pll_peak)  r->pll_peak  = fabs(ep);
            n++;
        }
    }
    if (n) {
        r->adap_mean = sum_adap / (double)n;
        r->pll_mean  = sum_pll / (double)n;
    }
}

/* ==================== 转速曲线 ==================== */
static double prof_const(double t, double rpm)
{
    (void)t;
    return rpm;
}

static double prof_ramp(double t, double unused)
{
    (void)unused;
    if (t < 1.0) return  3000.0 * t;                        /* 加速到 3000 */
    if (t < 1.4) return  3000.0;                            /* 保持 */
    if (t < 2.4) return  3000.0 - RAMP_SLEW * (t - 1.4);    /* 反向加速过零 */
    if (t < 2.8) return -3000.0;
    if (t < 3.3) return -3000.0 + RAMP_SLEW * (t - 2.8);
    return 0.0;
}

static double prof_step(double t, double unused)
{
    (void)unused;
    if (t < 0.5) return  100.0;
    if (t < 1.0) return  500.0;
    if (t < 1.5) return   80.0;
    if (t < 2.0) return  -80.0;
    return 0.0;
}

static double prof_reversal(double t, double unused)
{
    (void)unused;
    if (t < 0.5) return  300.0;
    if (t < 0.8) return  300.0 - 2000.0 * (t - 0.5);
    if (t < 1.1) return -300.0;
    if (t < 1.4) return -300.0 + 2000.0 * (t - 1.1);
    if (t < 1.7) return  300.0;
    return 0.0;
}

static double prof_soak(double t, double unused)
{
    (void)unused;
    const double x = fmod(t, 6.0);
    if (x < 0.5) return 0.0;
    if (x < 1.5) return 1200.0 * (x - 0.5);
    if (x < 2.0) return 1200.0;
    if (x < 2.5) return 1200.0 - 4800.0 * (x - 2.0);
    if (x < 3.0) return -1200.0;
    if (x < 3.5) return -1200.0 + 2700.0 * (x - 3.0);
    if (x < 4.0) return 150.0;
    if (x < 4.5) return 150.0 + 4700.0 * (x - 4.0);
    if (x < 5.0) return 2500.0;
    if (x < 5.5) return 2500.0 - 5000.0 * (x - 5.0);
    return 0.0;
}

/* ==================== 1. 接口保护 ==================== */
static int test_guards(void)
{
    int32_t adap = -1, pll = -1;
    int ok = 1;

    lt_speed_update(123);                       /* 未初始化就调用 */
    lt_speed_get(&adap, &pll);
    ok &= (adap == 0 && pll == 0);

    lt_speed_set(100, 100);                     /* 未初始化设置增益，应无副作用 */
    lt_speed_init(0, SAMPLE_HZ);                /* 非法分辨率 */
    lt_speed_update(0);
    lt_speed_get(&adap, &pll);
    ok &= (adap == 0 && pll == 0);

    lt_speed_init(ENCODER_CPR, 0);              /* 非法频率 */
    lt_speed_update(0);
    lt_speed_get(&adap, &pll);
    ok &= (adap == 0 && pll == 0);

    lt_speed_init(ENCODER_CPR, SAMPLE_HZ);      /* 正常初始化，首次更新前 */
    lt_speed_get(&adap, &pll);
    ok &= (adap == 0 && pll == 0);

    lt_speed_update(100);                       /* 首拍只做同步，输出仍为 0 */
    lt_speed_get(&adap, &pll);
    ok &= (adap == 0 && pll == 0);

    printf("\n== 1. 接口保护 ==\n");
    printf("非法初始化 / 初始化前调用均安全: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

/* ==================== 2. 冷启动同步 ==================== */
static int test_cold_start(void)
{
    static const double pos0s[] = { 0.0, 100000.0, 250000.0 };
    int ok = 1;

    printf("\n== 2. 冷启动同步（不应出现假速度尖峰）==\n");
    for (unsigned k = 0; k < sizeof(pos0s) / sizeof(pos0s[0]); k++) {
        encoder_t enc;
        encoder_init(&enc, pos0s[k]);
        speed_start();

        double max_adap = 0.0, max_pll = 0.0;
        int32_t adap = 0, pll = 0;
        for (int i = 0; i < (int)(0.2 * SAMPLE_HZ); i++) {
            lt_speed_update(encoder_step(&enc, 0.0, 0.0));
            lt_speed_get(&adap, &pll);
            const double a = fabs(cps_to_rpm((double)adap));
            const double p = fabs(cps_to_rpm((double)pll));
            if (a > max_adap) max_adap = a;
            if (p > max_pll)  max_pll  = p;
        }
        const int pass = (max_adap <= 1e-4 && max_pll <= 1e-4);
        ok &= pass;
        printf("静止于 %7.0f: |adap|max=%8.4f |PLL|max=%8.4f  %s\n",
               pos0s[k], max_adap, max_pll, pass ? "PASS" : "FAIL");
    }

    /* 带速启动：前 3 拍必须输出 0，之后收敛 */
    {
        encoder_t enc;
        encoder_init(&enc, 100000.0);
        speed_start();

        int32_t adap = 0, pll = 0;
        int zero_first = 1;
        for (int i = 0; i < 3; i++) {
            lt_speed_update(encoder_step(&enc, 30.0, 0.0));
            lt_speed_get(&adap, &pll);
            if (adap != 0 || pll != 0) zero_first = 0;
        }
        double max_err = 0.0;
        for (int i = 3; i < (int)(2.0 * SAMPLE_HZ); i++) {
            lt_speed_update(encoder_step(&enc, 30.0, 0.0));
            lt_speed_get(&adap, &pll);
            if ((double)i >= 0.5 * SAMPLE_HZ) {
                const double e = fabs(cps_to_rpm((double)pll) - 30.0);
                if (e > max_err) max_err = e;
            }
        }
        const int pass = zero_first && (max_err <= peak_tol(30.0));
        ok &= pass;
        printf("30 RPM 带速启动: 前 3 拍为 0=%s, 稳定后 PLL 最大误差=%.3f  %s\n",
               zero_first ? "是" : "否", max_err, pass ? "PASS" : "FAIL");
    }
    return ok;
}

/* ==================== 3. 恒速精度扫描 ==================== */
static int test_constant(void)
{
    static const double speeds[] = { 5, 10, 15, 20, 30, 50, 80, 85, 90, 95,
                                     100, 120, 200, 300, 600, 1200, 3000 };
    const int nsp = (int)(sizeof(speeds) / sizeof(speeds[0]));
    int pass = 0;

    printf("\n== 3. 恒速扫描（每点 2 s，统计后 1.5 s）==\n");
    printf("%9s | %11s %11s | 判定\n", "RPM", "PLL 均值误差", "PLL 最大误差");
    for (int dir = 0; dir < 2; dir++) {
        for (int k = 0; k < nsp; k++) {
            const double rpm = (dir == 0) ? speeds[k] : -speeds[k];
            result_t r;
            memset(&r, 0, sizeof(r));
            run_profile(0.0, 2.0, 0.5, prof_const, rpm, 0.0, &r);

            const int ok_ch = adap_ok(&r, rpm) &&
                              channel_ok(r.pll_mean, r.pll_peak, rpm);
            const int ok_spike = (r.pll_max_abs <= fabs(rpm) * SPIKE_REL + SPIKE_ABS) &&
                                 (r.adap_max_abs <= fabs(rpm) * SPIKE_REL + SPIKE_ABS);
            const int ok = ok_ch && ok_spike && (r.bad == 0);

            printf("%+9.0f | %11.3f %11.3f | %s%s%s\n", rpm,
                   r.pll_mean, r.pll_peak, ok ? "PASS" : "FAIL",
                   ok_ch ? "" : "(误差)", ok_spike ? "" : "(尖峰)");
            if (ok) pass++;
        }
    }
    printf("恒速: %d/%d 点通过\n", pass, 2 * nsp);
    return (pass == 2 * nsp);
}

/* ==================== 4. 加减速斜坡 ==================== */
static int test_ramp(void)
{
    const double duration = 3.5, warmup = 0.1;
    const int64_t total = (int64_t)(duration * SAMPLE_HZ);
    const int64_t warm  = (int64_t)(warmup  * SAMPLE_HZ);

    /* M 法最坏滤波滞后：最长窗口 18 拍、alpha=0.12，叠加窗口自身的时间滞后 */
    const double m_lag = 1.15 * ((1.0 - M_FILTER_ALPHA) / M_FILTER_ALPHA *
                                 (M_FILTER_WINDOW / SAMPLE_HZ) * RAMP_SLEW +
                                 (M_FILTER_WINDOW / SAMPLE_HZ) * RPM_MAX);
    const double peak_allow = fmax(5.0, 0.08 * RPM_MAX);

    encoder_t enc;
    encoder_init(&enc, 0.0);
    speed_start();

    FILE *trace = fopen("trace_ramp.csv", "w");
    if (trace) fprintf(trace, "t_s,true_rpm,adap_rpm,pll_rpm\n");

    double max_adap = 0.0, max_pll = 0.0;
    int64_t oob_adap = 0, oob_pll = 0, bad = 0;
    int32_t adap = 0, pll = 0;

    for (int64_t i = 0; i < total; i++) {
        const double t = (double)i / SAMPLE_HZ;
        const double rpm = prof_ramp(t, 0.0);
        lt_speed_update(encoder_step(&enc, rpm, 0.0));
        lt_speed_get(&adap, &pll);
        if (fabs(cps_to_rpm((double)adap)) > RPM_RANGE_LIMIT ||
            fabs(cps_to_rpm((double)pll))  > RPM_RANGE_LIMIT) bad++;

        const double ea = fabs(cps_to_rpm((double)adap) - rpm);
        const double ep = fabs(cps_to_rpm((double)pll)  - rpm);
        const double band_adap = fmax(2.0, fmax(0.05 * fabs(rpm), m_lag));
        const double band_pll  = fmax(2.0, fmax(0.05 * fmax(fabs(rpm), 10.0), PLL_RAMP_ALLOW));

        if (i >= warm) {
            if (ea > max_adap) max_adap = ea;
            if (ep > max_pll)  max_pll  = ep;
            if (ea > band_adap) oob_adap++;
            if (ep > band_pll)  oob_pll++;
        }
        if (trace && (i % 50 == 0))
            fprintf(trace, "%.6f,%.3f,%.6f,%.6f\n", t, rpm,
                    cps_to_rpm((double)adap), cps_to_rpm((double)pll));
    }
    if (trace) fclose(trace);

    const double n = (double)(total - warm);
    const double frac_adap = (double)oob_adap / n;
    const double frac_pll  = (double)oob_pll  / n;
    const int ok_adap = (max_adap <= peak_allow) && (frac_adap <= 0.01);
    const int ok_pll  = (max_pll  <= peak_allow) && (frac_pll  <= 0.01);
    const int ok = (ADAP_ACTIVE ? ok_adap : 1) && ok_pll && (bad == 0);

    printf("\n== 4. 斜坡 0 -> 3000 -> -3000 -> 0（3.5 s）==\n");
    printf("M 法滤波滞后允许 = %.1f RPM（18 拍/alpha=0.12 @ 6000 RPM/s）\n", m_lag);
    if (ADAP_ACTIVE)
        printf("M   : 最大误差=%8.3f RPM  越带=%5.2f%%  %s\n",
               max_adap, frac_adap * 100.0, ok_adap ? "PASS" : "FAIL");
    printf("PLL : 最大误差=%8.3f RPM  越带=%5.2f%%  %s\n",
           max_pll, frac_pll * 100.0, ok_pll ? "PASS" : "FAIL");
    printf("越界样本=%lld ；波形 -> trace_ramp.csv\n", (long long)bad);
    return ok;
}

/* ==================== 5. 阶跃响应 ==================== */
static double settle_time(const double *samples, int64_t i0, int64_t i1,
                          double target, double band, int64_t hold)
{
    int64_t inband = 0;
    for (int64_t i = i0; i <= i1; i++) {
        if (fabs((double)samples[i] - target) <= band) {
            if (++inband >= hold) return (double)(i - inband + 1) / SAMPLE_HZ;
        } else {
            inband = 0;
        }
    }
    return -1.0;
}

static double overshoot(const double *samples, int64_t i0, int64_t i1, double target)
{
    double worst = 0.0;
    for (int64_t i = i0; i <= i1; i++) {
        const double e = fabs((double)samples[i] - target);
        if (e > worst) worst = e;
    }
    return worst;
}

static int test_steps(void)
{
    static const double step_t[] = { 0.5, 1.0, 1.5, 2.0 };
    static const double step_v[] = { 500.0, 80.0, -80.0, 0.0 };
    const double duration = 2.5;
    const int64_t total = (int64_t)(duration * SAMPLE_HZ);
    const int64_t hold  = (int64_t)(0.05 * SAMPLE_HZ);      /* 判定稳定需连续 50 ms */

    double *adap_buf = malloc((size_t)total * sizeof(double));
    double *pll_buf  = malloc((size_t)total * sizeof(double));
    if (!adap_buf || !pll_buf) {
        printf("\n== 5. 阶跃响应 ==\n内存不足\n");
        free(adap_buf); free(pll_buf);
        return 0;
    }

    encoder_t enc;
    encoder_init(&enc, 0.0);
    speed_start();

    for (int64_t i = 0; i < total; i++) {
        int32_t adap = 0, pll = 0;
        lt_speed_update(encoder_step(&enc, prof_step((double)i / SAMPLE_HZ, 0.0), 0.0));
        lt_speed_get(&adap, &pll);
        adap_buf[i] = cps_to_rpm((double)adap);
        pll_buf[i]  = cps_to_rpm((double)pll);
    }

    int pass = 0;
    printf("\n== 5. 阶跃响应（带宽 max(2, 5%%)，连续 50 ms 判稳定）==\n");
    for (unsigned k = 0; k < 4; k++) {
        const double t_step = step_t[k], target = step_v[k];
        const double prev = (k == 0) ? 100.0 : step_v[k - 1];
        const int64_t i0   = (int64_t)(t_step * SAMPLE_HZ);
        const int64_t i1   = (int64_t)((t_step + 0.45) * SAMPLE_HZ);
        const double band  = fmax(2.0, 0.05 * fabs(target));
        const double os_limit = 1.3 * fabs(target - prev) + 2.0;

        const double st_pll = settle_time(pll_buf, i0, i1, target, band, hold);
        const double os_pll = overshoot(pll_buf, i0, i0 + (int64_t)(0.1 * SAMPLE_HZ), target);
        const int ok_pll = (st_pll >= 0.0) && (st_pll - t_step <= 0.2) && (os_pll <= os_limit);

        int ok_adap = 1;
        double st_adap = -1.0, os_adap = 0.0;
        if (ADAP_ACTIVE) {
            st_adap = settle_time(adap_buf, i0, i1, target, band, hold);
            os_adap = overshoot(adap_buf, i0, i0 + (int64_t)(0.1 * SAMPLE_HZ), target);
            ok_adap = (st_adap >= 0.0) && (st_adap - t_step <= 0.2) && (os_adap <= os_limit);
        }

        printf("阶跃 %+5.0f RPM: PLL 稳定=%6.1f ms 超调=%6.1f %s", target,
               st_pll < 0 ? -1.0 : (st_pll - t_step) * 1000.0, os_pll, ok_pll ? "PASS" : "FAIL");
        if (ADAP_ACTIVE)
            printf(" | M 稳定=%6.1f ms 超调=%6.1f %s",
                   st_adap < 0 ? -1.0 : (st_adap - t_step) * 1000.0, os_adap,
                   ok_adap ? "PASS" : "FAIL");
        printf(" | %s\n", (ok_pll && ok_adap) ? "PASS" : "FAIL");
        if (ok_pll && ok_adap) pass++;
    }
    free(adap_buf);
    free(pll_buf);
    printf("阶跃: %d/4 通过\n", pass);
    return (pass == 4);
}

/* ==================== 6. 过零换向 ==================== */
static int test_reversal(void)
{
    const int64_t total = (int64_t)(2.0 * SAMPLE_HZ);

    encoder_t enc;
    encoder_init(&enc, 0.0);
    speed_start();

    double sum_neg = 0.0, sum_pos = 0.0, sum_zero = 0.0;
    int64_t n_neg = 0, n_pos = 0, n_zero = 0;
    double zero_at[2] = { -1.0, -1.0 };          /* 两次真实过零时刻 0.65 / 1.25 s */
    double deadband_max[2] = { 0.0, 0.0 };
    int64_t deadband_run[2] = { 0, 0 };
    int64_t bad = 0;
    int32_t pll = 0;

    for (int64_t i = 0; i < total; i++) {
        const double t = (double)i / SAMPLE_HZ;
        lt_speed_update(encoder_step(&enc, prof_reversal(t, 0.0), 0.0));
        lt_speed_get(NULL, &pll);
        const double pll_rpm = cps_to_rpm((double)pll);
        if (fabs(pll_rpm) > RPM_RANGE_LIMIT) bad++;

        if (t >= 0.95 && t <= 1.05) { sum_neg += pll_rpm; n_neg++; }
        if (t >= 1.55 && t <= 1.65) { sum_pos += pll_rpm; n_pos++; }
        if (t >= 1.90 && t <= 2.00) { sum_zero += pll_rpm; n_zero++; }

        /* 过零后首次进入 ±1 RPM 的时刻 */
        for (int k = 0; k < 2; k++) {
            const double lo = (k == 0) ? 0.55 : 1.15;
            if (t >= lo && t <= lo + 0.45 && zero_at[k] < 0.0 && fabs(pll_rpm) <= 1.0)
                zero_at[k] = t;
        }
        /* 过零附近的输出死区时长（钳位异常监视） */
        for (int k = 0; k < 2; k++) {
            const double lo = (k == 0) ? 0.55 : 1.15;
            if (t >= lo && t <= lo + 0.45) {
                if (fabs(pll_rpm) <= 1.0) {
                    deadband_run[k]++;
                } else {
                    if ((double)deadband_run[k] > deadband_max[k])
                        deadband_max[k] = (double)deadband_run[k];
                    deadband_run[k] = 0;
                }
            }
        }
    }

    const double err_neg  = fabs(sum_neg / (double)n_neg + 300.0);
    const double err_pos  = fabs(sum_pos / (double)n_pos - 300.0);
    const double err_zero = fabs(sum_zero / (double)n_zero);
    const double deadband_ms[2] = { deadband_max[0] / SAMPLE_HZ * 1000.0,
                                    deadband_max[1] / SAMPLE_HZ * 1000.0 };

    const int ok = (err_neg <= 30.0) && (err_pos <= 30.0) && (err_zero <= 5.0) &&
                   (deadband_ms[0] <= 10.0) && (deadband_ms[1] <= 10.0) && (bad == 0);

    printf("\n== 6. 换向 +300 -> -300 -> +300 -> 0 ==\n");
    printf("-300 段误差=%6.2f | +300 段误差=%6.2f | 归零误差=%6.2f RPM\n",
           err_neg, err_pos, err_zero);
    printf("过零进入 ±1 RPM 时刻: 第一处 %.3f s, 第二处 %.3f s\n", zero_at[0], zero_at[1]);
    printf("过零输出死区: %.1f ms / %.1f ms（限 10 ms）\n", deadband_ms[0], deadband_ms[1]);
    printf("换向: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

/* ==================== 7. 单圈跨零 ==================== */
static int test_wrap(void)
{
    static const int dirs[] = { 1, -1 };
    int ok = 1;

    printf("\n== 7. 单圈跨零（600 RPM 越过计数 0）==\n");
    for (unsigned d = 0; d < 2; d++) {
        const double rpm = 600.0 * dirs[d];
        const double pos0 = (dirs[d] > 0) ? (ENCODER_CPR - 5.0) : 5.0;

        result_t r;
        memset(&r, 0, sizeof(r));
        run_profile(pos0, 1.5, 0.5, prof_const, rpm, 0.0, &r);

        const int pass = adap_ok(&r, rpm) && channel_ok(r.pll_mean, r.pll_peak, rpm) && (r.bad == 0);
        ok &= pass;
        printf("方向 %+d: PLL 均值=%7.3f 最大=%7.3f  %s\n",
               dirs[d], r.pll_mean, r.pll_peak, pass ? "PASS" : "FAIL");
    }
    return ok;
}

/* ==================== 8. 位置抖动 ==================== */
static int test_jitter(void)
{
    result_t r;
    memset(&r, 0, sizeof(r));
    rng_state = 0xABCDEF01u;

    /* 100 RPM 上叠加 ±2 计数抖动，3 s */
    run_profile(0.0, 3.0, 0.5, prof_const, 100.0, 2.0, &r);

    const int ok = (fabs(r.pll_mean) <= 5.0) && (r.pll_peak <= 15.0) &&
                   (r.pll_max_abs <= 160.0) && (r.adap_max_abs <= 160.0) &&
                   (r.bad == 0);
    printf("\n== 8. 位置抖动（100 RPM ±2 计数，3 s）==\n");
    printf("PLL 均值=%6.2f 最大=%6.2f 峰值=%6.2f  %s\n",
           r.pll_mean, r.pll_peak, r.pll_max_abs, ok ? "PASS" : "FAIL");
    return ok;
}

/* ==================== 9. 单次读数毛刺 ==================== */
static int test_glitch(void)
{
    const int64_t glitch_at = (int64_t)(1.0 * SAMPLE_HZ);
    const int64_t total = (int64_t)(3.0 * SAMPLE_HZ);
    const int64_t hold  = (int64_t)(0.05 * SAMPLE_HZ);

    encoder_t enc;
    encoder_init(&enc, 0.0);
    speed_start();

    double max_pll = 0.0, recover = -1.0;
    int64_t inband = 0, bad = 0;
    int32_t pll = 0;

    for (int64_t i = 0; i < total; i++) {
        int32_t c = (int32_t)encoder_step(&enc, 100.0, 0.0);
        if (i == glitch_at) c = (int32_t)(((int64_t)c + 5000) % (int64_t)ENCODER_CPR);
        lt_speed_update((uint32_t)c);
        lt_speed_get(NULL, &pll);
        const double pll_rpm = cps_to_rpm((double)pll);
        if (fabs(pll_rpm) > RPM_RANGE_LIMIT) bad++;

        if (i > glitch_at) {
            const double p = fabs(pll_rpm);
            if (p > max_pll) max_pll = p;
            if (recover < 0.0) {
                if (p <= 110.0) {
                    if (++inband >= hold)
                        recover = (double)(i - inband + 1) / SAMPLE_HZ - 1.0;
                } else {
                    inband = 0;
                }
            }
        }
    }

    const int ok = (max_pll <= RPM_RANGE_LIMIT) && (recover >= 0.0) &&
                   (recover <= 0.2) && (bad == 0);
    printf("\n== 9. 单次 +5000 计数毛刺（t=1.0 s，100 RPM）==\n");
    printf("毛刺后 |PLL|max=%8.1f 恢复=%6.1f ms  %s\n",
           max_pll, recover < 0 ? -1.0 : recover * 1000.0, ok ? "PASS" : "FAIL");
    return ok;
}

/* ==================== 10. 极低速（参考） ==================== */
static int test_ultra_low(void)
{
    static const double rpms[] = { 0.0, 0.5, 4.0 };
    int ok = 1;

    printf("\n== 10. 极低速 / 静止（参考项）==\n");
    for (unsigned k = 0; k < sizeof(rpms) / sizeof(rpms[0]); k++) {
        result_t r;
        memset(&r, 0, sizeof(r));
        run_profile(0.0, 2.0, 1.0, prof_const, rpms[k], 0.0, &r);

        const int pass = (r.bad == 0);
        ok &= pass;
        printf("%+5.1f RPM: PLL 均值=%8.3f 峰值=%8.3f  %s\n",
               rpms[k], r.pll_mean, r.pll_max_abs, pass ? "ok" : "FAIL");
    }
    printf("说明：M 法在窗口分辨率以下无输出（约 1 RPM @25 kHz/18 位）；\n");
    printf("      PLL 输出死区为最低速度档的 1/8，积分器不受影响。\n");
    return ok;
}

/* ==================== 11. PLL 输出死区扫描（参考） ==================== */
static double pll_steady(double rpm, uint32_t ki)
{
    encoder_t enc;
    encoder_init(&enc, 0.0);
    lt_speed_init(ENCODER_CPR, SAMPLE_HZ);
    lt_speed_set(PLL_KP, ki);

    const int64_t total = (int64_t)(1.5 * SAMPLE_HZ);
    const int64_t warm  = (int64_t)(0.5 * SAMPLE_HZ);
    double sum = 0.0;
    int64_t n = 0;
    int32_t pll = 0;

    for (int64_t i = 0; i < total; i++) {
        lt_speed_update(encoder_step(&enc, rpm, 0.0));
        lt_speed_get(NULL, &pll);
        if (i >= warm) { sum += (double)pll; n++; }
    }
    return cps_to_rpm(sum / (double)n);
}

static int test_pll_floor(void)
{
    static const double scan_ki50[] = { 1, 2, 3, 4, 5, 10, 20, 50, 100 };
    static const double scan_ki5[]  = { 4, 10, 50, 100 };

    printf("\n== 11. PLL 输出死区扫描（参考项）==\n");
    printf("ki=50 : ");
    for (unsigned k = 0; k < sizeof(scan_ki50) / sizeof(scan_ki50[0]); k++)
        printf("%.0f->%6.2f  ", scan_ki50[k], pll_steady(scan_ki50[k], 50u));
    printf("\nki=5  : ");
    for (unsigned k = 0; k < sizeof(scan_ki5) / sizeof(scan_ki5[0]); k++)
        printf("%.0f->%6.2f  ", scan_ki5[k], pll_steady(scan_ki5[k], 5u));
    printf("\n");
    return 1;
}

/* ==================== 12. 长时浸泡 / 确定性 / 耗时 ==================== */
static int test_soak(void)
{
    const int64_t calls = 10000000LL;               /* 400 s 仿真时长 @25 kHz */
    const int64_t chunk = 1000;

    int32_t *input = malloc((size_t)calls * sizeof(int32_t));
    if (!input) {
        printf("\n== 12. 长时浸泡 ==\n内存不足\n");
        return 0;
    }

    encoder_t enc;
    encoder_init(&enc, 0.0);
    rng_state = 0xDEADBEEFu;
    for (int64_t i = 0; i < calls; i++) {
        const double noise = (double)(int)(rng_next() % 5u) - 2.0;
        input[i] = (int32_t)encoder_step(&enc, prof_soak((double)i / SAMPLE_HZ, 0.0), noise);
    }

    uint64_t hash[2] = { 0, 0 };
    int64_t bad[2] = { 0, 0 };
    double wall[2] = { 0.0, 0.0 };
    double worst_chunk = 0.0;

    for (int run = 0; run < 2; run++) {
        speed_start();
        int32_t adap = 0, pll = 0;
        uint32_t ua = 0, up = 0;
        const double t0 = now_sec();
        double chunk_start = t0;

        for (int64_t i = 0; i < calls; i++) {
            if ((i % chunk) == 0) chunk_start = now_sec();
            lt_speed_update((uint32_t)input[i]);
            lt_speed_get(&adap, &pll);
            if (fabs(cps_to_rpm((double)adap)) > RPM_RANGE_LIMIT ||
                fabs(cps_to_rpm((double)pll))  > RPM_RANGE_LIMIT) bad[run]++;
            memcpy(&ua, &adap, sizeof(ua));
            memcpy(&up, &pll, sizeof(up));
            hash[run] = hash[run] * 1099511628211ULL ^ ua;
            hash[run] = hash[run] * 1099511628211ULL ^ up;
            if (run == 0 && ((i + 1) % chunk) == 0) {
                const double d = now_sec() - chunk_start;
                if (d > worst_chunk) worst_chunk = d;
            }
        }
        wall[run] = now_sec() - t0;
    }
    free(input);

    const int deterministic = (hash[0] == hash[1]) && (bad[0] == bad[1]);
    const int ok = (bad[0] == 0) && deterministic;

    printf("\n== 12. 长时浸泡：%lld 次调用（%.0f s 仿真）==\n",
           (long long)calls, (double)calls / SAMPLE_HZ);
    printf("轮次耗时 %.3f s / %.3f s，平均 %.1f ns/次，最差 %lld 次分块 %.1f ns/次\n",
           wall[0], wall[1], wall[0] / (double)calls * 1e9,
           (long long)chunk, worst_chunk / (double)chunk * 1e9);
    printf("越界样本 run1=%lld run2=%lld；校验和一致=%s\n",
           (long long)bad[0], (long long)bad[1], deterministic ? "是" : "否");
    printf("浸泡: %s\n", ok ? "PASS" : "FAIL");
    return ok;
}

/* ==================== 用例表 ==================== */
typedef struct {
    const char *name;
    int (*run)(void);
    int informational;                              /* 参考项：只打印，不计入结论 */
} suite_t;

static const suite_t SUITES[] = {
    { "接口保护",       test_guards,     0 },
    { "冷启动同步",     test_cold_start, 0 },
    { "恒速精度扫描",   test_constant,   0 },
    { "加减速斜坡",     test_ramp,       0 },
    { "阶跃响应",       test_steps,      0 },
    { "过零换向",       test_reversal,   0 },
    { "单圈跨零",       test_wrap,       0 },
    { "位置抖动",       test_jitter,     0 },
    { "读数毛刺",       test_glitch,     0 },
    { "极低速",         test_ultra_low,  1 },
    { "PLL 死区扫描",   test_pll_floor,  1 },
    { "长时浸泡",       test_soak,       0 },
};

int main(void)
{
    const int suite_count = (int)(sizeof(SUITES) / sizeof(SUITES[0]));
    int passed = 0;

    printf("lt_speed 压测：采样 %.0f Hz，CPR = %u（18 位），PLL kp=%u ki=%u，"
           "M 法通道 %s\n",
           (double)SAMPLE_HZ, (unsigned)ENCODER_CPR, (unsigned)PLL_KP, (unsigned)PLL_KI,
           ADAP_ACTIVE ? "开启" : "关闭（仅校验 adap 恒为 0）");

    for (int i = 0; i < suite_count; i++) {
        const int ok = SUITES[i].run();
        if (ok || SUITES[i].informational) passed++;
    }

    printf("\n============================================\n");
    printf("汇总: %d/%d 组通过\n", passed, suite_count);
    return (passed == suite_count) ? 0 : 1;
}