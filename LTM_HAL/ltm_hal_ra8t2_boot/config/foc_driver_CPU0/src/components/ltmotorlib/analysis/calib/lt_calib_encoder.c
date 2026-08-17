#include "analysis/calib/lt_calib.h"
#include <string.h>


typedef struct {
    uint16_t max_points;    /* 最大采样点数 */
    uint16_t count;         /* 当前已采样点数 */
    uint8_t  done;          /* 采集完成标志 */

    uint32_t cpr;           /* 编码器单圈分辨率 */
    int64_t  sum_err;       /* 误差累加（int64 防溢出） */
} lt_calib_encoder_obj;

static lt_calib_encoder_obj calib_encoder_obj;
static lt_calib_encoder_obj *calib_encoder = &calib_encoder_obj;

void lt_calib_encoder_init(uint16_t points, uint32_t cpr)
{
    memset(calib_encoder, 0, sizeof(lt_calib_encoder_obj));
    calib_encoder->max_points = points;
    calib_encoder->cpr = cpr;
}

void lt_calib_encoder_add(uint32_t phase_count, uint32_t encoder_raw)
{
    if (calib_encoder->done) return;
    if (calib_encoder->count >= calib_encoder->max_points) {
        calib_encoder->done = 1;
        return;
    }

    /* err = raw - ref */
    int32_t err = (int32_t)encoder_raw - (int32_t)phase_count;
    int32_t cpr_half = calib_encoder->cpr >> 1;
    /* 跨圈修正：映射到 [-cpr/2, cpr/2) */
    if (err > cpr_half) {
        err -= calib_encoder->cpr;
    } else if (err < -cpr_half) {
        err += (int32_t)calib_encoder->cpr;
    }

    calib_encoder->sum_err += err;
    calib_encoder->count++;
}

uint8_t lt_calib_encoder_is_done(void)
{
    return calib_encoder->done;
}

int32_t lt_calib_encoder_get(void)
{
    if (!calib_encoder->done) {
        return 0;
    }

    int64_t avg = calib_encoder->sum_err / (int64_t)calib_encoder->count;
    return (int32_t)avg;
}