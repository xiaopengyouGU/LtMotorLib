/*
 * SPDX-License-Identifier: MIT
 * LTM_CTRL 闭环仿真示例（PC，无硬件依赖）
 *
 * 与固件控制任务同构：25 kHz 电流环（测速 + FOC），速度环 5 kHz 分频调用。
 * 控制链全程整数：测速输出 count/s，PID 环内 Q24 标幺，FOC 输入输出 Q15。
 *
 * 闭环路径：
 *   lt_pid(速度环) -> iq(mA) -> lt_foc -> 三相占空比
 *   被控对象 J·dω/dt = Kt·Iq - B·ω -> 单圈编码器计数 -> lt_speed 测速 -> 回到速度环
 */
#include <stdio.h>
#include "ltm_ctrl/lt_control.h"
#include "ltm_ctrl/lt_math.h"

/* ---- 被控对象参数 ---- */
#define POLE_PAIRS        5u
#define ENCODER_CPR       262144u        /* 18 位单圈编码器 */
#define INERTIA_KGM2      0.0005f        /* 转动惯量 kg·m² */
#define TORQUE_CONST_NMA  0.20f          /* 转矩常数 N·m/A */
#define VISC_FRIC_NMS     0.005f         /* 粘性摩擦 N·m·s/rad */

/* ---- 运行参数 ---- */
#define CURRENT_FREQ      25000u         /* 电流环 / 测速调用频率 Hz */
#define SPEED_DIV         5u             /* 速度环分频：25 kHz / 5 = 5 kHz */
#define SPEED_FREQ        (CURRENT_FREQ / SPEED_DIV)
#define SIM_SECONDS       2.0f
#define TARGET_RPM        1500
#define SETTLE_TOL_RPM    10.0f

/* ---- 速度环标幺基准（Q24：1.0 = 2^24）----
 *   SPEED_BASE_CPS：1.0 pu 对应的 count/s
 *   I_BASE_MA     ：1.0 pu 对应的电流 mA
 * 增益按被控对象 pu 模型整定：Kp=4.0、Ki=32 1/s，临界阻尼无超调
 */
#define Q24_ONE           (1 << 24)
#define SPEED_BASE_RPM    2000
#define SPEED_BASE_CPS    ((int32_t)((int64_t)ENCODER_CPR * SPEED_BASE_RPM / 60))
#define SPEED_TARGET_Q24  ((int32_t)((int64_t)TARGET_RPM * Q24_ONE / SPEED_BASE_RPM))
#define I_BASE_MA         5000
#define SPEED_KP_Q15      131072         /* 4.0 */
#define SPEED_KI_Q15      1048576        /* 32 1/s */

/* 测速锁相环：f_n ≈ 90 Hz、ζ ≈ 0.7（见 lt_speed.h 整定公式） */
#define PLL_KP            792u
#define PLL_KI            13u

#define SPEED_PID_IDX     0

typedef struct {
    float omega;        /* 机械角速度 rad/s */
    float pos;          /* 单圈编码器计数，[0, ENCODER_CPR) */
} plant_t;

static void plant_init(plant_t *plant)
{
    plant->omega = 0.0f;
    plant->pos   = 0.0f;
}

/* 推进一个电流环周期，返回本拍的编码器单圈计数 */
static uint32_t plant_step(plant_t *plant, float iq_a)
{
    const float dt = 1.0f / (float)CURRENT_FREQ;

    plant->pos += plant->omega / (2.0f * (float)_PI) * (float)ENCODER_CPR * dt;
    if (plant->pos >= (float)ENCODER_CPR)  plant->pos -= (float)ENCODER_CPR;
    else if (plant->pos < 0.0f)            plant->pos += (float)ENCODER_CPR;

    plant->omega += (TORQUE_CONST_NMA * iq_a - VISC_FRIC_NMS * plant->omega)
                    / INERTIA_KGM2 * dt;
    return (uint32_t)plant->pos;
}

static void control_init(void)
{
    lt_speed_init(ENCODER_CPR, CURRENT_FREQ);
    lt_speed_set(PLL_KP, PLL_KI);

    lt_pid_init(SPEED_PID_IDX, 1, SPEED_FREQ);       /* type 1 = Q24 信号 */
    lt_pid_set(SPEED_PID_IDX, SPEED_KP_Q15, SPEED_KI_Q15, 0);
    lt_pid_set_limits(SPEED_PID_IDX, Q24_ONE, -Q24_ONE);
    lt_pid_set_target(SPEED_PID_IDX, SPEED_TARGET_Q24);

    lt_foc_init();
    lt_foc_set(ENCODER_CPR);
}

int main(void)
{
    plant_t plant;
    int32_t iq_q24  = 0;            /* 速度环输出，电流标幺 */
    int32_t iq_mA   = 0;
    int32_t dutys[3] = { 0 };
    uint32_t pos_count = 0;
    int32_t speed_cps = 0;

    plant_init(&plant);
    control_init();

    const int steps = (int)(SIM_SECONDS * (float)CURRENT_FREQ);
    for (int i = 0; i < steps; i++) {
        pos_count = plant_step(&plant, (float)iq_mA * 0.001f);
        lt_speed_update(pos_count);

        if (i % (int)SPEED_DIV == 0) {
            lt_speed_get(NULL, &speed_cps);
            const int32_t speed_q24 = (int32_t)(((int64_t)speed_cps << 24) / SPEED_BASE_CPS);
            iq_q24 = lt_pi_update(SPEED_PID_IDX, speed_q24);
            iq_mA  = (int32_t)(((int64_t)iq_q24 * I_BASE_MA) >> 24);
        }

        lt_foc_update(0, (int16_t)((int64_t)iq_mA * 32767 / I_BASE_MA),
                      pos_count * POLE_PAIRS, dutys);
    }

    lt_speed_get(NULL, &speed_cps);
    const float speed_rpm = (float)speed_cps * 60.0f / (float)ENCODER_CPR;
    const float err_rpm   = speed_rpm - (float)TARGET_RPM;

    printf("sim: speed=%.1f RPM (target %d), iq=%d mA (%.2f pu), duty=%.3f/%.3f/%.3f\n",
           speed_rpm, TARGET_RPM, (int)iq_mA, (float)iq_mA / (float)I_BASE_MA,
           (float)dutys[0] / 32768.0f, (float)dutys[1] / 32768.0f,
           (float)dutys[2] / 32768.0f);

    const int pass = (fabsf(err_rpm) <= SETTLE_TOL_RPM);
    printf("%s (err %.1f RPM, tol %.0f RPM)\n", pass ? "PASS" : "FAIL",
           (double)err_rpm, (double)SETTLE_TOL_RPM);
    return pass ? 0 : 1;
}