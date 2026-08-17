#include "motor_ctrl/tasks/calib_task.h"
#include "motor_ctrl/tasks/control_tasks.h"
#include "motor_ctrl/common/tasks_param_def.h"
#include "analysis/calib/lt_calib.h"
#include "analysis/excit/lt_excit.h"
#include "math/basic/lt_math.h"
#include <string.h>

/* ============ 内部状态 ============ */
typedef struct {
    calib_stage_t stage;                /* 当前阶段 */
    calib_result_t result;              /* 校准结果 */
    uint8_t done;                       /* 校准完成标志 */

    float Vd;                           /* d轴电压指令 */
    float Vq;                           /* q轴电压指令 */
    float angle_el;                     /* 电角度（开环用） */
    float prev_angle_el;                /* 上次电角度，用于计算增量 */
    float prev_theta_mech;              /* 上次机械角度，用于计算增量 */

    uint16_t sample_cnt;                /* 当前阶段已采样数 */
    uint8_t  point_idx;                 /* R:档位索引, L:DQ轴注入编码（0:D轴,1:Q轴） */
} calib_task_obj;

static calib_task_obj calib_obj;
static calib_task_obj *calib = &calib_obj;
static lt_motor_info_t info_obj;
static lt_motor_info_t *info = &info_obj;

/* ============ 静态函数声明 ============ */
static void r_add(void);
static uint8_t r_check_done(void);
static void l_add(void);
static uint8_t l_check_done(void);
static void pp_enc_add(void);
static uint8_t pp_enc_check_done(void);

/* ============ 阶段操作表 ============ */
typedef struct {
    void (*add)(void);
    uint8_t (*check_done)(void);
} calib_phase_ops_t;

static const calib_phase_ops_t phase_ops[] = {
    [CALIB_R]       = { r_add, r_check_done },
    [CALIB_L]       = { l_add, l_check_done },
    [CALIB_PP_ENC]  = { pp_enc_add, pp_enc_check_done },
};

/* ============ 公共接口实现 ============ */

void calib_task_init(void)
{
    memset(calib, 0, sizeof(calib_task_obj));
    calib->stage = CALIB_IDLE;
}

void calib_task_start(void)
{
    memset(calib, 0, sizeof(calib_task_obj));
    calib->stage = CALIB_R;
    calib->point_idx = 0;
    lt_calib_R_init(R_POINTS * R_SAMPLE_TICKS / 4);
}

void calib_task_update(void)
{
    if (calib->done) return;

    switch (calib->stage) {
        case CALIB_R: {
            uint8_t pt = calib->point_idx;
            float i_target = (pt + 1.0f) / R_POINTS * R_CALIB_CURRENT;
            calib->Vd += R_CALIB_KI * CURRENT_LOOP_PERIOD * (i_target - info->Id);
            calib->Vq = 0.0f;
            break;
        }

        case CALIB_L:
            float Vout = lt_excit_update(); /* 激励信号更新 */
            if (calib->point_idx == 0) {
                calib->Vd = Vout;
                calib->Vq = 0.0f;
            } else {
                calib->Vd = 0.0f;
                calib->Vq = Vout;
            }
            calib->angle_el = 0.0f;         /* 电角度固定，电机不转 */
            break;

        case CALIB_PP_ENC:
            calib->angle_el += PP_SPEED * MAIN_LOOP_PERIOD;
            calib->Vd = PP_VOLTAGE;
            calib->Vq = 0.0f;
            break;

        default:
            break;
    }
}

void calib_task_add(void)
{
    if (calib->done) return;
    control_tasks_get(info);
    phase_ops[calib->stage].add();
}

void calib_task_run(void)
{
    if (calib->done) return;
    phase_ops[calib->stage].check_done();
}

uint8_t calib_task_is_done(void)
{
    return calib->done;
}

void calib_task_get(calib_result_t *result, calib_stage_t *stage)
{
    if (result) memcpy(result, &calib->result, sizeof(calib_result_t));
    if (stage)  *stage = calib->stage;
}

void calib_task_get_excit(float *Vd, float *Vq, float *angle_el)
{
    if (Vd) *Vd = calib->Vd;
    if (Vq) *Vq = calib->Vq;
    if (angle_el) *angle_el = calib->angle_el;
}

/* ============ 静态函数实现 ============ */

/*
 * R 校准：多点差分法
 * 分 R_POINTS 档电流逐档采集，每档稳态区间采样后喂 lt_calib_R
 * lt_calib_R 内部做差分最小二乘：R = Σ(ΔV·ΔI) / Σ(ΔI²)
 */
static void r_add(void)
{
    calib->sample_cnt++;
    if (calib->sample_cnt > (R_SAMPLE_TICKS * 0.75f)) {
        lt_calib_R_add(calib->Vd, info->Id);
    }

    if (calib->sample_cnt >= R_SAMPLE_TICKS) {
        calib->point_idx++;
        calib->sample_cnt = 0;
    }
}

static uint8_t r_check_done(void)
{
    if (!lt_calib_R_is_done()) return 0;

    calib->result.R = lt_calib_R_get();
    /* 切换到 DQ 轴电感校准阶段 */
    calib->stage = CALIB_L;
    calib->point_idx = 0;
    /* 初始化并启动激励信号模块 */
    lt_excit_init(CURRENT_LOOP_PERIOD, 0);
    lt_excit_start_sine(L_HFI_VOLTAGE, L_HFI_FREQ, 0);
    lt_calib_L_init(L_HFI_TOTAL_SAMPLES, L_HFI_VOLTAGE, L_HFI_OMEGA);

    return 1;
}

/* L 校准：高频注入法
 * D/Q 轴分时注入高频正弦电压，相干解调提取电流响应幅值
 * L = V_amp / (I_amp * ω)
 */
static void l_add(void)
{
    float ref_sin = lt_excit_get(1) * L_HFI_VOLTAGE_INV;
    float ref_cos = lt_excit_get(2) * L_HFI_VOLTAGE_INV;

    if (calib->point_idx == 0) {      
        lt_calib_L_add(ref_sin, ref_cos, info->Id);
    } else {                           
        lt_calib_L_add(ref_sin, ref_cos, info->Iq);
    }
}

static uint8_t l_check_done(void)
{
    if (!lt_calib_L_is_done()) return 0;

    float L = lt_calib_L_get();

    if (calib->point_idx == 0) {
        calib->result.Ld = L;
        calib->point_idx = 1;
        lt_calib_L_init(L_HFI_TOTAL_SAMPLES, L_HFI_VOLTAGE, L_HFI_OMEGA);
        return 0;
    }
    calib->result.Lq = L;
    /* 切换到 PP_ENC 阶段 */
    calib->stage = CALIB_PP_ENC;
    calib->angle_el = 0.0f;
    calib->prev_angle_el = 0.0f;
    calib->prev_theta_mech = 0.0f;
    calib->sample_cnt = 0;
    lt_calib_pp_init(PP_ENC_TOTAL_SAMPLES);
    lt_calib_encoder_init(PP_ENC_TOTAL_SAMPLES, ENCODER_CPR);

    return 1;
}

/* 极对数 + 编码器偏移校准：一次匀速旋转同时完成
 * - 极对数：pp = Δθ_elec / Δθ_mech
 * - 编码器方向：sign(Δθ_mech)
 * - 编码器偏移：offset = average(encoder_raw - phase_count)
 */
static void pp_enc_add(void)
{
    float theta_mech = info->pos * DEG_TO_RAD;
    float dtheta_elec = calib->angle_el - calib->prev_angle_el;
    float dtheta_mech = theta_mech - calib->prev_theta_mech;

    /* 跨圈修正 */
    if (dtheta_mech > _PI)  dtheta_mech -= _2_PI;
    if (dtheta_mech < -_PI) dtheta_mech += _2_PI;

    lt_calib_pp_add(dtheta_elec, dtheta_mech);

    uint32_t phase_count = (uint32_t)(calib->angle_el * _2_PI_INV * ENCODER_CPR) % ENCODER_CPR;
    uint32_t encoder_raw;
    control_tasks_get2(&encoder_raw, NULL);
    lt_calib_encoder_add(phase_count, encoder_raw);

    calib->prev_angle_el = calib->angle_el;
    calib->prev_theta_mech = theta_mech;
    calib->sample_cnt++;
}

static uint8_t pp_enc_check_done(void)
{
    if (!lt_calib_pp_is_done() || !lt_calib_encoder_is_done()) return 0;

    lt_calib_pp_get(&(calib->result.pole_pairs), &(calib->result.encoder_dir));
    calib->result.encoder_offset = lt_calib_encoder_get();
    calib->result.valid = 1;            /* 标记校准成功 */
    calib->done = 1;                    /* 先标记所有流程完毕，再更新状态 */
    calib->stage = CALIB_DONE;          /* 避免中断读取竞态问题（函数指针表越界）*/

    return 1;
}