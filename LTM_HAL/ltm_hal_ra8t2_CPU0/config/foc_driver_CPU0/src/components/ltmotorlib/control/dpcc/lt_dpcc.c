/* lt_dpcc.c */
#include "control/dpcc/lt_dpcc.h"
#include "math/basic/lt_math.h"
#include <string.h>

/* ============================================================================
 * 内部 ESO 接口声明（实现在文件末尾）
 * ============================================================================*/
static void lt_eso_init(float Ls, float dt, float wb);
static void lt_eso_reset(void);
static void lt_eso_process(float ud, float uq, float id, float iq, float we);
static void lt_eso_get(float *vd_dist, float *vq_dist);   /* 获取 ESO估计的 电压扰动 */

/* ============================================================================
 * DPCC 主模块
 * ============================================================================*/
typedef struct {
    float Ls;           /* 相电感 (H) */
    float Rs;           /* 相电阻 (Ω) */
    float phi;          /* 永磁体磁链 (Wb) */
    float dt;           /* 控制周期 (s) */

    float id_ref;       /* d 轴电流目标 (A) */
    float iq_ref;       /* q 轴电流目标 (A) */
    float out_limit;    /* 输出电压限幅 (V) */
    float Ls_over_dt;   /* Ls/dt */

    float ud_prev;      /* 上一拍 d 轴电压 (V)，供 ESO 使用 */
    float uq_prev;      /* 上一拍 q 轴电压 (V) */
    float ud;           /* 当前拍 d 轴输出电压 (V) */
    float uq;           /* 当前拍 q 轴输出电压 (V) */

    float eso_width;    /* ESO 带宽 (rad/s)，<= 0 时禁用 */
} lt_dpcc_obj;

static lt_dpcc_obj dpcc_obj;
static lt_dpcc_obj *dpcc = &dpcc_obj;

/* ============================================================================
 * API 实现
 * ============================================================================*/

void lt_dpcc_init(float Ls, float Rs, float phi, float dt)
{
    memset(dpcc, 0, sizeof(lt_dpcc_obj));

    dpcc->Ls = Ls;
    dpcc->Rs = Rs;
    dpcc->phi = phi;
    dpcc->dt = dt;
    dpcc->Ls_over_dt = Ls/dt;
    dpcc->out_limit = 12.0f;
    dpcc->id_ref = 0.0f;

    lt_eso_init(Ls, dt, 0.0f);
}

void lt_dpcc_set(float eso_width, float out_limit)
{
    dpcc->eso_width = eso_width;
    if (out_limit > 0.0f) {
        dpcc->out_limit = out_limit;
    }

    lt_eso_init(dpcc->Ls, dpcc->dt, eso_width);
    if (eso_width <= 0.0f) {
        lt_eso_reset();
    }
}

void lt_dpcc_set_target(float id_ref, float iq_ref)
{
    dpcc->id_ref = id_ref;
    dpcc->iq_ref = iq_ref;
}

void lt_dpcc_reset(void)
{
    dpcc->ud_prev = 0.0f;
    dpcc->uq_prev = 0.0f;
    dpcc->ud = 0.0f;
    dpcc->uq = 0.0f;
    lt_eso_reset();
}

void lt_dpcc_process(float id, float iq, float we)
{
    float L_over_dt = dpcc->Ls_over_dt;     /* Ls/dt */
    float L  = dpcc->Ls;
    float R  = dpcc->Rs;
    float phi = dpcc->phi;
    float id_ref = dpcc->id_ref;
    float iq_ref = dpcc->iq_ref;
    float limit  = dpcc->out_limit;
    float vd_eso, vq_eso;

    /* ESO 更新 */
    lt_eso_process(dpcc->ud_prev, dpcc->uq_prev, id, iq, we);
    lt_eso_get(&vd_eso, &vq_eso);

    /* DPCC 无差拍计算，最后减去 ESO 估计的扰动值 */
    float ud_raw = L_over_dt * (id_ref - id)
                 + R * id
                 - we * L * iq
                 - vd_eso;

    float uq_raw = L_over_dt * (iq_ref - iq)
                 + R * iq
                 + we * L * id
                 + we * phi
                 - vq_eso;

    /* 电压限幅 */
    float mag = lt_sqrt(ud_raw * ud_raw + uq_raw * uq_raw);
    if (mag > limit) {
        float scale = limit / mag;
        ud_raw = ud_raw * scale;
        uq_raw = uq_raw * scale;
    }
    /* 保存本拍电压 */
    dpcc->ud      = ud_raw;
    dpcc->uq      = uq_raw;
    /* 更新上一拍电压，供下一拍 ESO 使用 */
    dpcc->ud_prev = ud_raw;
    dpcc->uq_prev = uq_raw;
}

void lt_dpcc_get(float *ud, float *uq)
{
    if (ud) *ud = dpcc->ud;
    if (uq) *uq = dpcc->uq;
}

/* ============================================================================
 * 内部 ESO 实现
 * ============================================================================*/
typedef struct {
    float Ls;           /* 相电感 (H) */
    float dt;           /* 控制周期 (s) */
    float wb;           /* 观测器带宽 (rad/s)，<= 0 时禁用 */

    float id_hat;       /* d 轴电流估计 (A) */
    float iq_hat;       /* q 轴电流估计 (A) */
    float fd_hat;       /* d 轴扰动估计 (A/s) */
    float fq_hat;       /* q 轴扰动估计 (A/s) */

    float ts_over_L;    /* dt / Ls，预计算 */
    float l1_dt, l2_dt; /* 观测器增益：l1 = 3*wb, l2 = 3*wb² ，此处直接乘上dt，提高计算效率 */
    float vd_dist;      /* d 轴等效电压扰动 (V) = fd_hat * Ls */
    float vq_dist;      /* q 轴等效电压扰动 (V) = fq_hat * Ls */
    uint8_t inited;     /* 首拍标志，1：已初始化，0：未初始化 */
} lt_eso_obj;

static lt_eso_obj eso_obj;
static lt_eso_obj *eso = &eso_obj;

static void lt_eso_init(float Ls, float dt, float wb)
{
    memset(eso, 0, sizeof(lt_eso_obj));

    eso->Ls = Ls;
    eso->dt = dt;
    eso->ts_over_L = dt / Ls;
    eso->wb = wb;

    if (wb > 0.0f) {
        eso->l1_dt = 3.0f * wb * dt;
        eso->l2_dt = 3.0f * wb * wb * dt;
    }
}

static void lt_eso_reset(void)
{
    eso->id_hat = 0.0f;
    eso->iq_hat = 0.0f;
    eso->fd_hat = 0.0f;
    eso->fq_hat = 0.0f;
    eso->vd_dist = 0.0f;
    eso->vq_dist = 0.0f;
    eso->inited = 0;            
}

static void lt_eso_process(float ud, float uq, float id, float iq, float we)
{
    if (eso->wb <= 0.0f) {
        eso->vd_dist = 0.0f;
        eso->vq_dist = 0.0f;
        return;
    }

    /* 首拍：用测量值初始化状态，避免从 0 起步的暂态 */
    if (!eso->inited) {
        eso->id_hat = id;
        eso->iq_hat = iq;
        eso->inited = 1;
    }

    float dt = eso->dt;
    float Ls = eso->Ls;
    float ts_over_L = eso->ts_over_L;
    float l1_dt = eso->l1_dt;       /* l1 * dt */
    float l2_dt = eso->l2_dt;       /* l2 * dt */

    /* d 轴：电流估计 + 扰动估计 */
    float fd_hat = eso->fd_hat;          
    float eps_d  = id - eso->id_hat;
    float id_hat_next = ts_over_L * (ud + fd_hat)
                      + id
                      + dt * we * iq
                      + l1_dt * eps_d;
    float fd_hat_next = fd_hat + l2_dt * eps_d;

    /* q 轴：电流估计 + 扰动估计 */
    float fq_hat = eso->fq_hat;
    float eps_q  = iq - eso->iq_hat;
    float iq_hat_next = ts_over_L * (uq + fq_hat)
                      + iq
                      - dt * we * id
                      + l1_dt * eps_q;
    float fq_hat_next = fq_hat + l2_dt * eps_q ;

    eso->id_hat = id_hat_next;
    eso->iq_hat = iq_hat_next;
    eso->fd_hat = fd_hat_next;
    eso->fq_hat = fq_hat_next;

    /* 扰动折算成等效电压，直接用于前馈补偿 */
    eso->vd_dist = fd_hat_next * Ls;
    eso->vq_dist = fq_hat_next * Ls;
}

static void lt_eso_get(float *vd_dist, float *vq_dist)
{
    if (vd_dist) *vd_dist = eso->vd_dist;
    if (vq_dist) *vq_dist = eso->vq_dist;
}