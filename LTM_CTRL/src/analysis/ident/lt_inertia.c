/* lt_inertia.c */
#include "analysis/ident/lt_ident.h"
#include "math/basic/lt_math.h"
#include "math/lsq/lt_lsq.h"
#include <string.h>

#define MAX_POINTS 128
#define MIN_POINTS 20

typedef struct {
    uint16_t max_points;
    uint16_t count;
    uint8_t  done;
    uint8_t  solved;
    
    float t_buf[MAX_POINTS];    /* 电磁转矩 Nm，solve时复用作 x */
    float a_buf[MAX_POINTS];    /* 机械角加速度 rad/s²，solve时复用作 y */
    
    float J;        /* 转动惯量 kg·m² */
    float J_std;    /* 标准差 */
} lt_inertia_obj;

static lt_inertia_obj inertia_obj;
static lt_inertia_obj *iner = &inertia_obj;

void lt_inertia_init(uint16_t max_points)
{
    if (max_points > MAX_POINTS) max_points = MAX_POINTS;
    if (max_points < MIN_POINTS) max_points = MIN_POINTS;
    
    memset(iner, 0, sizeof(lt_inertia_obj));
    iner->max_points = max_points;
}

void lt_inertia_start(void)
{
    iner->count = 0;
    iner->done = 0;
    iner->solved = 0;
    iner->J = 0.0f;
    iner->J_std = 0.0f;
}

void lt_inertia_add(float torque, float accel)
{
    if (iner->done) return;
    uint16_t count = iner->count;
    if (count >= iner->max_points) {
        iner->done = 1;
        return;
    }
    
    iner->t_buf[count] = torque;
    iner->a_buf[count] = accel;
    count++;
    
    if (count >= iner->max_points) {
        iner->done = 1;
    }
    iner->count = count;            /* 更新 count */
}

uint8_t lt_inertia_is_done(void)
{
    return iner->done;
}

void lt_inertia_solve(void)
{
    if (!iner->done) return;
    if (iner->count < MIN_POINTS) return;
    if (iner->solved) return;
    
    /* 运动方程：转矩 = J * 角加速度 + 摩擦/扰动 */
    float *x = iner->a_buf;   /* 加速度 rad/s² */
    float *y = iner->t_buf;   /* 转矩 Nm */
    
    /* 剔除零速/零加速度点，原地压缩 */
    uint16_t n = 0;
    for (uint16_t i = 0; i < iner->count; i++) {
        if (lt_absf(x[i]) > 1e-6f) {
            if (n != i) {
                x[n] = x[i];
                y[n] = y[i];
            }
            n++;
        }
    }
    
    if (n < MIN_POINTS) return;
    
    /* 用最小二乘拟合 y = J * x + b */
    float coeff[2];
    if (!lt_lsq_solve(x, y, 1, n, coeff)) return;
    if (coeff[1] <= 0.0f) return;  /* 惯量必须为正 */
    iner->J = coeff[1];
    
    /* 计算标准差：每个点的估计惯量 J_i = torque_i / accel_i */
    /* 复用 y 存 J_i = torque_i / accel_i */
    for (uint16_t i = 0; i < n; i++) {
        y[i] = y[i] / x[i];  /* y 现在变成 J_i */
    }
    lt_mean_std(y, n, NULL, &iner->J_std);
    
    iner->solved = 1;
}

void lt_inertia_get(float *J, float *J_std)
{
    if (J)     *J     = iner->J;
    if (J_std) *J_std = iner->J_std;
}