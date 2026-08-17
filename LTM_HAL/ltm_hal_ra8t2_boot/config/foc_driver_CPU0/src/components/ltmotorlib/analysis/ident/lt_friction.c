/* lt_friction.c */
#include "analysis/ident/lt_ident.h"
#include "math/lsq/lt_lsq.h"
#include "math/interp/lt_interp.h"
#include "math/basic/lt_math.h"
#include <string.h>

#define MAX_POINTS 128
#define MIN_POINTS 10
#define MIN_SPEED  1.0f           /* RPM，太小的速度剔除 */

typedef struct {
    uint16_t max_points;
    uint16_t count;
    uint8_t  done;
    uint8_t  solved;
    
    float speed_buf[MAX_POINTS]; /* 机械转速 RPM */
    float iq_buf[MAX_POINTS];    /* 等效摩擦电流 A */
    
    float Fc;        /* Fc：库仑摩擦电流 A */
    float B_iq;      /* 粘性摩擦系数 A/RPM */
    float r2;        /* 拟合优度 R² */
    uint8_t type;    
} lt_friction_obj;

static lt_friction_obj friction_obj;
static lt_friction_obj *fric = &friction_obj;

/* Classic摩擦模型 */
void lt_friction_init(uint16_t max_points)
{
    if (max_points > MAX_POINTS) max_points = MAX_POINTS;
    if (max_points < MIN_POINTS) max_points = MIN_POINTS;
    
    memset(fric, 0, sizeof(lt_friction_obj));
    fric->max_points = max_points;
}

void lt_friction_start(void)
{
    fric->count = 0;
    fric->done = 0;
    fric->solved = 0;
    fric->Fc = 0.0f;
    fric->B_iq = 0.0f;
    fric->r2 = 0.0f;
}

/* 可以认为：正反转对摩擦力的影响极小（稳态时）
 * 因此采样时，只采正半周或负半周，保存绝对值即可。
 */
void lt_friction_add(float speed, float iq_fric)
{
    if (fric->done) return;
    uint16_t count = fric->count;
    if (count >= fric->max_points) {
        fric->done = 1;
        return;
    }
    
    fric->speed_buf[count] = lt_absf(speed); /* RPM */
    fric->iq_buf[count] = lt_absf(iq_fric);
    count++;
    
    fric->count = count;
    if (count >= fric->max_points) {
        fric->done = 1;
    }
}

uint8_t lt_friction_is_done(void)
{
    return fric->done;
}

void lt_friction_solve(void)
{
    if (!fric->done) return;
    if (fric->count < MIN_POINTS) return;
    if (fric->solved) return;
    
    float *w = fric->speed_buf;
    float *iq = fric->iq_buf;
    uint16_t count = fric->count;
    /* 构造二元回归：iq = B_iq * w + iq_c * sign(w)
     *   x1 = sign(w), x2 = w
     *   coeff[0] = iq_c, coeff[1] = B_iq
     */
    float x[count * 2];
    for (uint16_t j = 0; j < count; j++) {
        x[j * 2 + 0] = (w[j] >= 0.0f) ? 1.0f : -1.0f;
        x[j * 2 + 1] = w[j];
    }
    
    float coeff[2];
    if (!lt_lsq_solve(x, iq, 2, count, coeff)) return;
    if (coeff[0] < 0.0f) return;  /* iq_c 必须为正 */
    if (coeff[1] < 0.0f) return;  /* B_iq 必须为正 */
    
    fric->Fc = coeff[0];
    fric->B_iq = coeff[1];
    
    /* R² = 预测值与实测值的相关系数平方 */
    float *pred = x;
    float Fc    = fric->Fc;
    float B_iq  = fric->B_iq;
    for (uint16_t j = 0; j < count; j++) {
        float sign = (w[j] >= 0.0f) ? 1.0f : -1.0f;
        pred[j] = Fc * sign + B_iq * w[j];
    }
    float r = lt_correlation_coeff(pred, iq, count);
    fric->r2 = r * r;
    
    fric->solved = 1;
}

void lt_friction_get2(float *Fc, float *B_iq, float *r2)
{
    if (Fc)   *Fc   = fric->Fc;
    if (B_iq) *B_iq = fric->B_iq;
    if (r2)   *r2   = fric->r2;
}

float lt_friction_get(float speed)             /* speed : RPM */
{
    if (!fric->done) return 0.0f;              /* 数据未采集完毕，直接返回 */
    float sign  = (speed >= 0.0) ? 1.0f : -1.0f;
    return sign * lt_interp_linear(speed, fric->speed_buf, fric->iq_buf, fric->count);
}