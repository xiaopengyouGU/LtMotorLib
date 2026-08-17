/* lt_ladrc.c */
#include "control/ladrc/lt_ladrc.h"
#include "math/basic/lt_math.h"
#include <string.h>

/* ============================================================================
 * 内部 ESO 接口声明（实现在文件末尾）
 * ============================================================================*/
static void lt_eso_init(float dt, float j_over_kt, float speed_max, float accel_max);
static void lt_eso_set(float wb);           /* 设置 ESO 带宽 */
static void lt_eso_reset(void);
static void lt_eso_process(float pos, float iq);
static void lt_eso_get(float *speed_est, float *iq_dob);

/* ============================================================================
 * LADRC 主模块
 * ============================================================================*/
typedef struct {
    float out_limit;    /* 输出限幅 (A) */
    float kp;           /* 比例增益 */

    float speed_ref;    /* 速度目标值 (rad/s) */
    float iq_out;       /* q 轴电流指令输出 (A) */
    float speed_est;    /* 速度估计 (rad/s) */
} lt_ladrc_obj;

static lt_ladrc_obj ladrc_obj;
static lt_ladrc_obj *ladrc = &ladrc_obj;

/* ============================================================================
 * API 实现
 * ============================================================================*/

void lt_ladrc_init(float j_over_kt, float speed_max, float accel_max, float dt)
{
    memset(ladrc, 0, sizeof(lt_ladrc_obj));

    ladrc->out_limit = 8.0f;
    ladrc->kp = 0.0f;
    lt_eso_init(dt, j_over_kt, speed_max, accel_max);
}

void lt_ladrc_set(float kp, float eso_width, float out_limit)
{
    ladrc->kp = kp;
    if (out_limit > 0.0f) {
        ladrc->out_limit = out_limit;
    }

    if (eso_width <= 0.0f)  lt_eso_reset();
    else                    lt_eso_set(eso_width);
}

void lt_ladrc_set_target(float speed_ref)
{
    ladrc->speed_ref = speed_ref;
}

void lt_ladrc_reset(void)
{
    ladrc->iq_out = 0.0f;
    ladrc->speed_est = 0.0f;
    lt_eso_reset();
}

void lt_ladrc_process(float pos, float iq)
{
    float speed_est, iq_dob;

    /* ESO 更新 */
    lt_eso_process(pos, iq);
    lt_eso_get(&speed_est, &iq_dob);

    /* LADRC 控制律：iq_ref = kp * (speed_ref - speed_est) - iq_dob */
    float iq_raw = ladrc->kp * (ladrc->speed_ref - speed_est) - iq_dob;
    float limit  = ladrc->out_limit;
    /* 限幅 */
    iq_raw = CONSTRAINS(iq_raw, limit, -limit);
    ladrc->iq_out = iq_raw;
    ladrc->speed_est = speed_est;
}

void lt_ladrc_get(float *iq_ref, float *speed_est)
{
    if (iq_ref)     *iq_ref    = ladrc->iq_out;
    if (speed_est)  *speed_est = ladrc->speed_est;
}

/* ============================================================================
 * 内部 ESO 实现（三阶机械域 ESO）
 * ============================================================================*/
typedef struct {
    float dt;           /* 控制周期 (s) */
    float wb;           /* 观测器带宽 (rad/s)，<= 0 时禁用 */
    float j_over_kt;    /* J / Kt (kg·m² / Nm/A) */
    float kt_over_j;    /* 1.0f(J / Kt (kg·m² / Nm/A)) */
    float speed_max;    /* 速度限幅 (rad/s) */
    float accel_max;    /* 加速度限幅 (rad/s²) */

    float x1;           /* 位置增量预测 (rad) */
    float x2;           /* 速度估计 (rad/s) */
    float x3;           /* 扰动加速度估计 (rad/s²) */

    float pos_last;     /* 上一拍位置 (rad) */
    uint8_t inited;     /* 首拍标志，1：已初始化，0：未初始化 */

    float l1_dt;        /* 3*wb*dt，预计算 */
    float l2_dt;        /* 3*wb²*dt，预计算 */
    float l3_dt;        /* wb³*dt，预计算 */
} lt_eso_obj;

static lt_eso_obj eso_obj;
static lt_eso_obj *eso = &eso_obj;

static void lt_eso_init(float dt, float j_over_kt, float speed_max, float accel_max)
{
    memset(eso, 0, sizeof(lt_eso_obj));

    eso->dt = dt;
    eso->j_over_kt = j_over_kt;
    eso->kt_over_j = 1.0f / j_over_kt;
    eso->speed_max = speed_max;
    eso->accel_max = accel_max;
}

static void lt_eso_set(float wb)
{
    float dt = eso->dt;
    eso->wb = wb;
    if (wb > 0.0f) {
        float l1 = 3.0f * wb;
        float l2 = 3.0f * wb * wb;
        float l3 = wb * wb * wb;
        eso->l1_dt = l1 * dt;
        eso->l2_dt = l2 * dt;
        eso->l3_dt = l3 * dt;
    }
}

static void lt_eso_reset(void)
{
    eso->x1 = 0.0f;
    eso->x2 = 0.0f;
    eso->x3 = 0.0f;
    eso->pos_last = 0.0f;
    eso->inited = 0;
}

static void lt_eso_process(float pos, float iq)
{
    if (eso->wb <= 0.0f) {
        return;
    }

    /* 首拍：用测量值初始化位置，避免差分跳变 */
    if (!eso->inited) {
        eso->pos_last = pos;
        eso->inited = 1;
        return;
    }

    float dt = eso->dt;
    float speed_max = eso->speed_max;
    float accel_max = eso->accel_max;
    float eps = (pos - eso->pos_last) - eso->x1;

    float a_ctrl = iq * eso->kt_over_j;      /* iq * Kt / J */

    /* 后向欧拉：用新 x3 更新 x2 */
    float x3_new = eso->x3 + eso->l3_dt * eps;
    x3_new = CONSTRAINS(x3_new, accel_max, -accel_max);

    float x2_new = eso->x2 + eso->l2_dt * eps + dt * (a_ctrl + x3_new);
    x2_new = CONSTRAINS(x2_new, speed_max, -speed_max);

    /* 前向欧拉：用旧 x2 预测位置增量 */
    float x1_new = eso->l1_dt * eps + dt * eso->x2;
    x1_new = CONSTRAINS(x1_new, _PI, -_PI);

    eso->x1 = x1_new;
    eso->x2 = x2_new;
    eso->x3 = x3_new;
    eso->pos_last = pos;
}

static void lt_eso_get(float *speed_est, float *iq_dob)
{
    if (speed_est)  *speed_est = eso->x2;
    if (iq_dob)     *iq_dob = eso->x3 * eso->j_over_kt;
}