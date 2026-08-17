/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-4-21      Lvtou        5-segment S-curve、traped-curve、triangle-curve
 */
#include "control/scurve/lt_scurve.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*---------------------- 5段S型曲线对象 ----------------------*/
struct lt_scurve_object {
    int32_t pos;                /* 当前位置 (Unit) */
    int32_t target;             /* 目标位置 (Unit) */
    int32_t vel;                /* 当前速度 (Unit/s ) */
    int32_t acc;                /* 当前加速度 (Unit/s²) */
    int32_t Tsec[5];            /* 5段各自的持续时间 (ms) */
    int32_t j_step;             /* 每周期加加速度产生的加速度增量 */
    int32_t j_step_dec;         /* 减速段加速度单步变化 */
    int32_t v_limit;            /* 实际限制速度 (Unit/s) */
    int32_t a_limit;            /* 最大加速度 (Unit/s²) */
    int32_t a_limit_dec;        /* 减速段最大加速度 (Unit/s²) */
    int32_t elapsed;            /* 当前段已用时间 (ms) */
    int8_t dir;                 /* 方向：1 正向，-1 反向 */
    uint8_t phase;              /* 当前阶段：0~4 对应5段，5 表示完成 */
    uint16_t Ts;                /* 更新周期: 单位 ms */
};

/* 5个阶段的加加速度符号：+1 加加速，-1 减加速，0 匀速 */
static const int8_t J_sign[5] = {1, -1, 0, -1, 1};

/*==============================================================================
 * 创建S型曲线对象
 *==============================================================================*/
lt_scurve_t lt_scurve_create(void)
{
    lt_scurve_t s = (lt_scurve_t)malloc(sizeof(struct lt_scurve_object));
    if (s) memset(s, 0, sizeof(struct lt_scurve_object));
    return s;
}

/*==============================================================================
 * 重置S型曲线（模式切换时调用）
 *==============================================================================*/
void lt_scurve_reset(lt_scurve_t s)
{
    if (s) memset(s, 0, sizeof(struct lt_scurve_object));
}

/* 短距离降级到三角形规划 */
static void _lt_scurve_start_triangle(lt_scurve_t s, lt_scurve_config_t *cfg)
{
    int32_t total = (int32_t)((s->target - s->pos) > 0 ? (s->target - s->pos) : -(s->target - s->pos));
    float total_f = total * 1e-3f;  /* 转换为米或毫米（根据单位） */
    float v_start_f = cfg->v_start * 1e-3f;
    float v_stop_f  = cfg->v_stop * 1e-3f;
    float a_acc_f = s->a_limit * 1e-3f;   /* 加速度 (Unit/ms²) */
    float a_dec_f = s->a_limit_dec * 1e-3f;
    
    /* 解方程求最大速度 v_peak
     * 总位移 = v_start*Ta + 0.5*a_acc*Ta² + v_stop*Td + 0.5*a_dec*Td²
     * 其中 Ta = (v_peak - v_start)/a_acc, Td = (v_peak - v_stop)/a_dec
     * 代入得：
     * total = (v_peak² - v_start²)/(2*a_acc) + (v_peak² - v_stop²)/(2*a_dec)
     */
    float v_peak_f = sqrtf((total_f * 2 + v_start_f*v_start_f/a_acc_f + v_stop_f*v_stop_f/a_dec_f) 
                         / (1.0f/a_acc_f + 1.0f/a_dec_f));
    
    /* 限制不超过最大速度 */
    float v_max_f = cfg->v_max * 1e-3f;
    if (v_peak_f > v_max_f) v_peak_f = v_max_f;
    
    /* 计算实际加减速时间 (ms) */
    int32_t Ta_act = (int32_t)((v_peak_f - v_start_f) / a_acc_f * 1000);
    int32_t Td_act = (int32_t)((v_peak_f - v_stop_f) / a_dec_f * 1000);
    
    /* 确保时间至少为1个周期 */
    if (Ta_act < 1) Ta_act = 1;
    if (Td_act < 1) Td_act = 1;
    
    /* 实际能达到的速度 */
    s->v_limit = (int32_t)(v_peak_f * 1000);
    s->vel = cfg->v_start;
    s->acc = s->a_limit;
    
    /* 三角形只有2段：加速、减速 */
    s->Tsec[0] = Ta_act;
    s->Tsec[1] = 0;
    s->Tsec[2] = Td_act;
    s->Tsec[3] = 0;
    s->Tsec[4] = 0;
    
    s->j_step = 0;
    s->j_step_dec = 0;
    s->phase = 0;
    s->elapsed = 0;
}

/*==============================================================================
 * 梯形加减速初始化
 *==============================================================================*/
static void _lt_scurve_start_trape(lt_scurve_t s, lt_scurve_config_t *cfg)
{
    uint16_t Ta = cfg->acct_ms < 100 ? 100 : cfg->acct_ms;
    uint16_t Td = cfg->dect_ms < 100 ? 100 : cfg->dect_ms;
    uint16_t Ts = cfg->period_ms;

    s->pos    = cfg->start_pos;
    s->target = cfg->target_pos;
    int64_t delta = s->target - s->pos;
    s->dir = (delta >= 0) ? 1 : -1;

    /* 梯形加速段：a = Δv / Ta */
    int32_t a_acc = (int32_t)((cfg->v_max - cfg->v_start) * 1000 / Ta);
    if (a_acc < 0) a_acc = -a_acc;

    /* 梯形减速段：a = Δv / Td */
    int32_t a_dec = (int32_t)((cfg->v_max - cfg->v_stop) * 1000 / Td);
    if (a_dec < 0) a_dec = -a_dec;

    s->a_limit     = a_acc;
    s->a_limit_dec = a_dec;
    s->v_limit     = cfg->v_max;
    s->vel         = cfg->v_start;
    s->acc         = a_acc;
    s->elapsed     = 0;
    s->phase       = 0;
    s->Ts          = Ts;

    /* 位移约束检查（梯形公式） */
    float s_acc = (cfg->v_start + cfg->v_max) * Ta * 1e-3f / 2.0f;
    float s_dec = (cfg->v_max + cfg->v_stop) * Td * 1e-3f / 2.0f;
    int32_t need = (int32_t)(s_acc + s_dec);
    int32_t total = (int32_t)(delta > 0 ? delta : -delta);

    if (need > total || cfg->v_max == 0) {
        /* 短距离：降级为三角形规划 */
        _lt_scurve_start_triangle(s, cfg);
        return;
    }

    int32_t remain = total - need;
    int32_t Tv = (int32_t)(remain * 1000 / abs(cfg->v_max));

    /* 梯形3段：匀加速、匀速、匀减速 */
    s->Tsec[0] = Ta;
    s->Tsec[1] = Tv;
    s->Tsec[2] = Td;
    s->Tsec[3] = 0;
    s->Tsec[4] = 0;

    s->j_step     = 0;
    s->j_step_dec = 0;
}

/*==============================================================================
 * S形加减速初始化
 *==============================================================================*/
static void _lt_scurve_start_s(lt_scurve_t s, lt_scurve_config_t *cfg)
{
    uint16_t Ta = cfg->acct_ms < 100 ? 100 : cfg->acct_ms;
    uint16_t Td = cfg->dect_ms < 100 ? 100 : cfg->dect_ms;
    uint16_t Ts = cfg->period_ms;

    s->pos    = cfg->start_pos;
    s->target = cfg->target_pos;
    int64_t delta = s->target - s->pos;
    s->dir = (delta >= 0) ? 1 : -1;

    /* 加速段：Tj = Ta / 2 */
    int32_t Tj = Ta >> 1;
    int32_t a_acc = (int32_t)((cfg->v_max - cfg->v_start) * 1000 / Tj);
    if (a_acc < 0) a_acc = -a_acc;
    int64_t temp = (int64_t)a_acc * 1000;
    int32_t J = (Tj > 0) ? (int32_t)(temp / Tj) : 0;

    /* 减速段：Tjd = Td / 2 */
    int32_t Tjd = Td >> 1;
    int32_t a_dec = (int32_t)((cfg->v_max - cfg->v_stop) * 1000 / Tjd);
    if (a_dec < 0) a_dec = -a_dec;
    int64_t temp_dec = (int64_t)a_dec * 1000;
    int32_t Jd = (Tjd > 0) ? (int32_t)(temp_dec / Tjd) : 0;

    s->a_limit     = a_acc;
    s->a_limit_dec = a_dec;
    s->v_limit     = cfg->v_max;
    s->vel         = cfg->v_start;
    s->acc         = 0;
    s->elapsed     = 0;
    s->phase       = 0;
    s->Ts          = Ts;

    /* 位移约束检查（S形公式） */
    float s_acc = cfg->v_start * Ta * 1e-3f + a_acc * Tj * Tj * 1e-6f;
    float s_dec = cfg->v_max * Td * 1e-3f - a_dec * Tjd * Tjd * 1e-6f;
    int32_t need = (int32_t)(s_acc + s_dec);
    int32_t total = (int32_t)(delta > 0 ? delta : -delta);

    if (need > total || cfg->v_max == 0) {
        /* 短距离：S形降级为三角形规划 */
        _lt_scurve_start_triangle(s, cfg);
        return;
    }

    int32_t remain = total - need;
    int32_t Tv = (int32_t)(remain * 1000 / abs(cfg->v_max));

    /* S形5段：加加速、减加速、匀速、加减速、减减速 */
    s->Tsec[0] = Tj;
    s->Tsec[1] = Tj;
    s->Tsec[2] = Tv;
    s->Tsec[3] = Tjd;
    s->Tsec[4] = Tjd;

    s->j_step     = (int32_t)(J  * Ts * 1e-3f);
    s->j_step_dec = (int32_t)(Jd * Ts * 1e-3f);
}

void lt_scurve_start(lt_scurve_t s, lt_scurve_config_t *cfg)    /* S形曲线规划启动 */
{
    if (!s || !cfg) return;

    if (cfg->type == 0) {
        _lt_scurve_start_trape(s, cfg);
    } else {
        _lt_scurve_start_s(s, cfg);
    }
}

/* 每周期更新，返回当前位置指令值（Unit）*/
int32_t lt_scurve_update(lt_scurve_t s)
{
    if(!s)    return  0;
    /* 梯形完成判断：phase>=3 且 j_step==0 */
    if (s->j_step == 0 && s->phase >= 3) return s->target;
    /* S形完成判断：phase>=5 */
    if (s->j_step != 0 && s->phase >= 5) return s->target;
    float Ts = s->Ts * 1e-3f;
    
    int32_t step  = (s->phase <= 2) ? s->j_step  : s->j_step_dec;
    int32_t limit = (s->phase <= 2) ? s->a_limit : s->a_limit_dec;
    
    /* 根据当前阶段更新加速度 */
    if (J_sign[s->phase] != 0)
        s->acc += step * J_sign[s->phase];
    
    /* 限幅加速度 */
    if (s->phase <= 2) {
        if (s->acc > limit) s->acc = limit;
        else if (s->acc < 0) s->acc = 0;
    } else {
        if (s->acc > 0) s->acc = 0;
        else if (s->acc < -limit) s->acc = -limit;
    }

    /* 更新速度 */
    s->vel += s->acc * Ts;
    if (s->vel > s->v_limit) s->vel = s->v_limit;
    else if (s->vel < 0) s->vel = 0;

    /* 更新位置 */
    s->pos += s->dir * s->vel * Ts;

    /* 阶段切换 */
    if (++s->elapsed >= s->Tsec[s->phase]) {
        s->elapsed = 0;
        s->phase++;
        /* 梯形：进入匀速段时加速度清零，进入减速段时设为负值 */
        if (s->j_step == 0) {
            if (s->phase == 1) s->acc = 0;
            else if (s->phase == 2) s->acc = -s->a_limit_dec;
        }
    }

    return s->pos;
}

/*==============================================================================
 * 判断曲线是否计算完毕
 *==============================================================================*/
uint8_t lt_scurve_is_done(lt_scurve_t s)
{
    if(!s)      return 1;
    if(s->j_step == 0 && s->phase >= 3) return 1;
    if(s->j_step != 0 && s->phase >= 5) return 1;
    return 0;
}

void lt_scurve_stop(lt_scurve_t s)      /* 曲线停止 */
{
    if (s) s->phase = 5;
}

/*==============================================================================
 * 删除S形曲线对象
 *==============================================================================*/
void lt_scurve_delete(lt_scurve_t s)    /* 曲线对象删除 */
{
    if (s) free(s);
}