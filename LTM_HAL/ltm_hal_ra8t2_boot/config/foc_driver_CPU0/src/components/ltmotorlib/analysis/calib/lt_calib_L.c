#include "analysis/calib/lt_calib.h"
#include "math/demod/lt_demod.h"
#include <string.h>

#define MAX_POINTS 4096
#define MIN_POINTS 4

typedef struct {
    uint8_t done;       /* 校准完成标准 */
    float amp;          /* 高频注入电压幅值（V）*/
    float we;           /* 高频信号角速度（rad/s）*/
} lt_calib_L_obj;

static lt_calib_L_obj calib_L_obj;
static lt_calib_L_obj *calib_L = &calib_L_obj;

void lt_calib_L_init(uint16_t max_points, float amp, float we)
{
    if (max_points > MAX_POINTS) max_points = MAX_POINTS;
    if (max_points < MIN_POINTS) max_points = MIN_POINTS;
    
    memset(calib_L, 0, sizeof(lt_calib_L_obj));
    calib_L->amp = amp;
    calib_L->we = we;
    lt_demod_init(max_points);      /* 初始化相关解调模块 */
}

void lt_calib_L_add(float ref_sin, float ref_cos, float I)
{
    if (calib_L->done) return;
    lt_demod_add(ref_sin, ref_cos, I);  
    if(lt_demod_is_done())  calib_L->done = 1;
}

uint8_t lt_calib_L_is_done(void)
{
    return calib_L->done;
}

float lt_calib_L_get(void)
{
    if (!calib_L->done) return 0.0f;

    float I_amp, phase;
    lt_demod_solve();
    lt_demod_get(&I_amp, &phase);

    return calib_L->amp / (I_amp * calib_L->we);
}