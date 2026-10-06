#include "analysis/calib/lt_calib.h"
#include <string.h>

#define MAX_POINTS 8196
#define MIN_POINTS 4

typedef struct {
    uint16_t max_points;    /* 最大采样点数 */
    uint16_t count;         /* 当前已采样点数 */
    uint8_t  done;          /* 采集完成标志 */

    float sum_V;            /* ΣV，电压累加 */
    float sum_I;            /* ΣI，电流累加 */
    float sum_VI;           /* Σ(V·I)，用于协方差 */
    float sum_II;           /* Σ(I·I)，用于方差 */
} lt_calib_R_obj;

static lt_calib_R_obj calib_R_obj;
static lt_calib_R_obj *calib_R = &calib_R_obj;

void lt_calib_R_init(uint16_t max_points)
{
    if (max_points > MAX_POINTS) max_points = MAX_POINTS;
    if (max_points < MIN_POINTS) max_points = MIN_POINTS;

    memset(calib_R, 0, sizeof(lt_calib_R_obj));
    calib_R->max_points = max_points;
}

void lt_calib_R_add(float Vd, float Id)
{
    if (calib_R->done) return;
    if (calib_R->count >= calib_R->max_points) {
        calib_R->done = 1;
        return;
    }

    calib_R->sum_V  += Vd;
    calib_R->sum_I  += Id;
    calib_R->sum_VI += Vd * Id;
    calib_R->sum_II += Id * Id;
    calib_R->count++;
}

uint8_t lt_calib_R_is_done(void)
{
    return calib_R->done;
}

float lt_calib_R_get(void)
{
    if (!calib_R->done) return 0.0f;
    if (calib_R->count < MIN_POINTS) return 0.0f;
    /* 利用差分最小二乘法计算，可以消除噪声和死区偏置 */
    float n_inv = 1.0f / (float)calib_R->count;
    float mean_V = calib_R->sum_V  * n_inv;
    float mean_I = calib_R->sum_I  * n_inv;
    float cov_VI = calib_R->sum_VI * n_inv - mean_V * mean_I;
    float var_I  = calib_R->sum_II * n_inv - mean_I * mean_I;

    if (var_I < 1e-12f) return 0.0f;
    return cov_VI / var_I; 
}