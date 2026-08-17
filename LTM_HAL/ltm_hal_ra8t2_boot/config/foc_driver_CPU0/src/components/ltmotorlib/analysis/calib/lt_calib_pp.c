/* lt_calib_pp.c */
#include "analysis/calib/lt_calib.h"
#include <string.h>

#define ABS(x)         (((x) >= 0.0f) ? (x) : (-x))

typedef struct {
    uint16_t max_points;    /* 最大采样点数 */
    uint16_t count;         /* 当前已采样点数 */
    uint8_t  done;          /* 采集完成标志 */

    float sum_ratio;        /* Σ(Δθ_elec / Δθ_mech)，极对数累加 */
    float sum_dir;          /* Σ(sign(Δθ_mech))，方向累加 */
    float sum_ratio2;       /* Σ(ratio²)，用于方差 */
} lt_calib_pp_obj;

static lt_calib_pp_obj calib_pp_obj;
static lt_calib_pp_obj *calib_pp = &calib_pp_obj;

void lt_calib_pp_init(uint16_t max_points)
{
    memset(calib_pp, 0, sizeof(lt_calib_pp_obj));
    calib_pp->max_points = max_points;
}

void lt_calib_pp_add(float dtheta_elec, float dtheta_mech)
{
    if (calib_pp->done) return;
    if (calib_pp->count >= calib_pp->max_points) {
        calib_pp->done = 1;
        return;
    }
 

    if (ABS(dtheta_mech) < 1e-6f) return;

    float ratio = dtheta_elec / dtheta_mech;
    int dir = (dtheta_mech >= 0.0f) ? 1 : -1;

    calib_pp->sum_ratio  += ratio;
    calib_pp->sum_dir    += (float)dir;
    calib_pp->sum_ratio2 += ratio * ratio;
    calib_pp->count++;
}

uint8_t lt_calib_pp_is_done(void)
{
    return calib_pp->done;
}

void lt_calib_pp_get(int *pole_pairs, int *encoder_dir)
{
    if (!calib_pp->done) {
        if (pole_pairs)  *pole_pairs  = 0;
        if (encoder_dir) *encoder_dir = 0;
        return;
    }

    float n_inv = 1.0f / (float)calib_pp->count;
    float mean_ratio = calib_pp->sum_ratio * n_inv;
    float mean_dir   = calib_pp->sum_dir   * n_inv;

    int pp = (int)(mean_ratio + 0.5f);
    if (pp < 1) pp = 1;

    int dir = (mean_dir >= 0.0f) ? 1 : -1;

    if (pole_pairs)  *pole_pairs  = pp;
    if (encoder_dir) *encoder_dir = dir;
}