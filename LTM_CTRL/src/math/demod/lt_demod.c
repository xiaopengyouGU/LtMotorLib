/* lt_demod.c */
#include "math/demod/lt_demod.h"
#include "math/basic/lt_math.h"
#include <string.h>

typedef struct {
    float sum_sin;
    float sum_cos;
    uint8_t  done;          /* 数据采集完毕标志，0：未完毕，1：已完毕 */
    uint16_t count;         /* 当前计数值 */
    uint16_t target_cnt;    /* 目标计数值 */
    float amp;              
    float phase_deg;
} lt_demod_obj;

static lt_demod_obj demod_obj;
static lt_demod_obj *demod = &demod_obj;

/* ============================================================================
 * API 实现
 * ============================================================================*/

void lt_demod_init(uint16_t count)
{
    memset(demod, 0, sizeof(lt_demod_obj));
    demod->target_cnt = count;
}

void lt_demod_reset(void)
{
    uint16_t target_cnt = demod->target_cnt;
    memset(demod, 0, sizeof(lt_demod_obj));
    demod->target_cnt = target_cnt;
}

void lt_demod_add(float ref_sin, float ref_cos, float meas_signal)
{
    if(demod->done)         return;         /* 本次数据采集完毕 */
    if(demod->count >= demod->target_cnt){
        demod->done = 1;
        return;
    }

    demod->sum_sin += meas_signal * ref_sin;
    demod->sum_cos += meas_signal * ref_cos;
    demod->count++;
}

void lt_demod_solve(void)
{
    if (!demod->done) return; 

    float inv_n = 1.0f / (float)demod->count;
    float I = demod->sum_sin * inv_n;
    float Q = demod->sum_cos * inv_n;

    demod->amp = 2.0f * lt_sqrt(I * I + Q * Q);
    float phase_rad = lt_atan2(Q, I);           /* [0, 2π) */
    if (phase_rad > _PI) phase_rad -= _2_PI;    /* 映射到 [-π, π) */
    demod->phase_deg = phase_rad * 180.0f / _PI;
    demod->count = 0;                           /* 清零计数器 */
}

void lt_demod_get(float *amp, float *phase_deg)
{
    if (amp) *amp = demod->amp;
    if (phase_deg) *phase_deg = demod->phase_deg;
}

uint8_t lt_demod_is_done(void)
{
    return demod->done;
}