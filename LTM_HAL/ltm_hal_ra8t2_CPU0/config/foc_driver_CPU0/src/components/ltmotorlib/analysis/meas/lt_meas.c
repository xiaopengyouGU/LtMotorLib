#include "analysis/meas/lt_meas.h"
#include "analysis/excit/lt_excit.h"
#include "math/demod/lt_demod.h"
#include <string.h>
#include <math.h>

#define MAX_POINTS      64          /* 最大扫频点数 */
#define SETTLE_TICK     2000        /* 每个频点，稳定等待 2000 tick (20kHz下 100ms)，消除瞬态影响 */
#define COLLECT_CYCLES  20          /* 每个频点，采集 29 个整周期，消除直流偏置和噪声影响 */

typedef enum {
    State_Idle,                     /* 空闲状态 */
    State_Setting,                  /* 调整中 */
    State_Collecting,               /* 采集数据中 */
    State_Done,                     /* 扫频完毕 */
} meas_state_t;

typedef struct {
    meas_state_t state;
    uint16_t points;                /* 扫频点数 */
    uint16_t idx;
    uint16_t settle_cnt;            /* 调整计数 */
    float sample_freq;              /* 采样频率（Hz）*/
    float amp;

    float freq_buf[MAX_POINTS];
    float amp_buf[MAX_POINTS];
    float phase_buf[MAX_POINTS];
    uint8_t done;
} lt_meas_obj;

static lt_meas_obj meas_obj;
static lt_meas_obj *meas = &meas_obj;

void lt_meas_init(uint16_t points, float ts_s, float offset)
{
    ts_s = (ts_s > 1e-9f) ? ts_s : 40e-6f;
    memset(meas, 0, sizeof(lt_meas_obj));
    meas->points = (points > MAX_POINTS) ? MAX_POINTS : points;
    meas->points = (points < 2) ? 2 : points;
    meas->sample_freq = 1.0f/ts_s;
    meas->state = State_Idle;

    lt_excit_init(ts_s, offset);
}

void lt_meas_start(float freq_start, float freq_end, float amp)
{
    if (freq_start <= 0.0f || freq_end <= 0.0f || amp <= 0.0f) return;

    meas->amp = amp;
    meas->idx = 0;
    meas->done = 0;
    meas->settle_cnt = 0;

    uint16_t total = meas->points; 
    float K = freq_end / freq_start;
    for (uint16_t i = 0; i < meas->points; i++) {
        float t = (float)i / (float)(total - 1);
        meas->freq_buf[i] = freq_start * expf(t * logf(K));
    }

    lt_excit_start_sine(amp, meas->freq_buf[0], 0.0f);
    meas->state = State_Setting;
}

void lt_meas_add(float data)
{
    if (meas->state == State_Setting){
        meas->settle_cnt++;             
        return;
    }
    if (meas->done) return;

    float amp_inv = 1.0f / meas->amp;
    float ref_sin = lt_excit_get(1) * amp_inv;
    float ref_cos = lt_excit_get(2) * amp_inv;
    lt_demod_add(ref_sin, ref_cos, data);
}

void lt_meas_run(void)
{
    if (meas->done) return;
    if (meas->state == State_Idle || meas->state == State_Done) return;

    switch (meas->state) {
        case State_Setting:
            if (meas->settle_cnt >= SETTLE_TICK) {
                lt_demod_reset();
                uint16_t idx = meas->idx;
                uint16_t count  = COLLECT_CYCLES * meas->sample_freq / (meas->freq_buf[idx] + 1e-12f); /* 避免除0 */ 
                lt_demod_init(count);           /* 重置相干解调点数，保证整数个周期 */
                meas->state = State_Collecting;
            }
            break;

        case State_Collecting:
            if (lt_demod_is_done()) {
                float amp, phase;
                uint16_t idx = meas->idx;
                lt_demod_solve();
                lt_demod_get(&amp, &phase);
                amp  = amp / meas->amp;            /* 除以参考信号幅值 */
                meas->amp_buf[idx]   = 20.0f * log10f(amp > 1e-10f ? amp : 1e-10f);
                meas->phase_buf[idx] = phase;
                meas->idx = ++idx;                 /* 更新 idx */

                if (idx >= meas->points) {
                    meas->done  = 1;
                    meas->state = State_Done;
                    lt_excit_stop();
                } else {
                    meas->state = State_Setting;
                    meas->settle_cnt = 0;          /* 关键：重置稳定计数器 */
                    lt_excit_start_sine(meas->amp, meas->freq_buf[idx], 0.0f);
                }
            }
            break;
        default:    break;
    }
}

float lt_meas_update(void)
{
    return lt_excit_update();
}

uint8_t lt_meas_is_done(void)
{
    return meas->done;
}

void lt_meas_get(float *freq_hz, float *amp_db, float *phase_deg, uint16_t idx)
{
    if (!meas->done)        return;
    if (idx >= meas->points)   return;

    if (freq_hz)    *freq_hz    = meas->freq_buf[idx];
    if (amp_db)     *amp_db     = meas->amp_buf[idx];
    if (phase_deg)  *phase_deg  = meas->phase_buf[idx];
}