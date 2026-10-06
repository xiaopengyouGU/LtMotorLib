#include "analysis/ident/lt_ident.h"
#include "math/lsq/lt_lsq.h"
#include "math/basic/lt_math.h"
#include <string.h>

#define MAX_POINTS 128
#define MIN_POINTS 20
#define MIN_WE     1.0f           /* 电角速度阈值, 1rad/s */

typedef struct {
    uint16_t max_points;
    uint16_t count;
    uint8_t  done;              /* 采样完毕标志，1：已完毕，0：未完毕 */
    uint8_t  solved;            /* 求解完毕标准，1：已求解，0：未求解，避免重复求解 */
    
    float we_buf[MAX_POINTS];   /* 电角速度 rad/s，solve时复用作x */
    float y_buf[MAX_POINTS];    /* vq - Rs*iq，solve时复用作y */
    
    float phi;      /* 磁链 Wb */
    float Rs;       /* 相电阻：Ω */
    float offset;   /* 偏置 V */
    float r2;       /* 拟合优度 R² */
} lt_flux_obj;

static lt_flux_obj flux_obj;
static lt_flux_obj *flux = &flux_obj;

void lt_flux_init(uint16_t max_points, float Rs)
{
    if (max_points > MAX_POINTS) max_points = MAX_POINTS;
    if (max_points < MIN_POINTS) max_points = MIN_POINTS;
    
    memset(flux, 0, sizeof(lt_flux_obj));
    flux->max_points = max_points;
    flux->Rs = Rs;      
}

void lt_flux_start(void)
{
    flux->count = 0;
    flux->done = 0;
    flux->solved = 0;
    flux->phi = 0.0f;
    flux->r2 = 0.0f;
}

void lt_flux_add(float we, float vq, float iq)
{
    if (flux->done) return;
    uint16_t count = flux->count;
    if (count >= flux->max_points) {
        flux->done = 1;
        return;
    }
    
    flux->we_buf[count] = we;
    flux->y_buf[count] = vq - flux->Rs * iq;  /* 直接存 y = vq - Rs*iq */
    count++;
    
    if (count >= flux->max_points) {
        flux->done = 1;
    }
    flux->count = count;                          /* 更新 count */
}

uint8_t lt_flux_is_done(void)
{
    return flux->done;
}

void lt_flux_solve(void)
{
    if (!flux->done) return;
    if (flux->count < MIN_POINTS) return;
    if (flux->solved) return;
    
    float *x = flux->we_buf;
    float *y = flux->y_buf;
    
    /* 剔除零速点，原地压缩 */
    uint16_t n = 0;
    for (uint16_t i = 0; i < flux->count; i++) {
        if (lt_absf(x[i]) > MIN_WE){     /*  只保留了 |we| > 阈值 */ 
            if (n != i) {
                x[n] = x[i];
                y[n] = y[i];
            }
            n++;
        }
    }
    
    if (n < (MIN_POINTS >> 1)) return;   /* 有效辨识点数过小，直接退出 */
    
    float coeff[2];
    if (!lt_lsq_solve(x, y, 1, n, coeff)) return;
    if (coeff[1] <= 0.0f) return;  /* 磁链必须为正 */
    
    flux->phi = coeff[1];
    flux->offset = coeff[0];
    
    /* 计算 R² */
    float r = lt_correlation_coeff(x, y, n);
    flux->r2 = r * r;
}

void lt_flux_get(float *phi, float *offset, float *r2)
{
    if (phi)    *phi    = flux->phi;
    if (offset) *offset = flux->offset;
    if (r2)     *r2     = flux->r2;
}