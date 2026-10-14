/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author      Notes
 * 2026-10-14     Lvtou       从胶水层的斜坡里抽出来：限速器 + 静态实例池
 */
#include "control/ramp/lt_ramp.h"
#include <string.h>

/*---------------------- 斜坡对象 ----------------------*/
struct lt_ramp_object {
    int32_t  out;               /* 当前输出（Unit）*/
    int32_t  target;            /* 目标（Unit），随时可改 */
    int32_t  step;              /* 每拍步长（Unit，正数）*/
    volatile uint8_t ready;     /* start 执行中挡住中断侧步进 */
};

/* 私有：内部一律用指针书写，对外仍是 uint8_t idx */
typedef struct lt_ramp_object *lt_ramp_t;

/* ---- 实例池：无堆，idx 直接映射 ---- */
static struct lt_ramp_object ramp_pool[LT_RAMP_MAX_INSTANCES];

static struct lt_ramp_object *_ramp(uint8_t idx)
{
    if (idx < LT_RAMP_MAX_INSTANCES) {
        return &ramp_pool[idx];
    }
    return 0;
}

void lt_ramp_reset(uint8_t idx)
{
    lt_ramp_t r = _ramp(idx);
    if (r) memset(r, 0, sizeof(*r));
}

/* 起状态：输出从 start 出发，朝 target 按 step 逼近 */
void lt_ramp_start(uint8_t idx, int32_t start, int32_t target, int32_t step)
{
    lt_ramp_t r = _ramp(idx);
    if (!r) return;
    r->ready  = 0;
    r->out    = start;
    r->target = target;
    r->step   = (step < 0) ? -step : step;
    r->ready  = 1;
}

void lt_ramp_set_target(uint8_t idx, int32_t target)
{
    lt_ramp_t r = _ramp(idx);
    if (!r) return;
    r->target = target;
}

void lt_ramp_set_step(uint8_t idx, int32_t step)
{
    lt_ramp_t r = _ramp(idx);
    if (!r) return;
    r->step = (step < 0) ? -step : step;
}

/* 每拍推一步：差值走 int64，两端都在 int32 范围内不会溢出；输出永远夹在起点和目标之间 */
int32_t lt_ramp_update(uint8_t idx)
{
    lt_ramp_t r = _ramp(idx);
    if (!r) return 0;
    if (!r->ready) return r->out;               /* start 执行中：返回旧值，不跳变 */

    int64_t d = (int64_t)r->target - (int64_t)r->out;
    if      (d >  (int64_t)r->step) r->out += r->step;
    else if (d < -(int64_t)r->step) r->out -= r->step;
    else                            r->out  = r->target;
    return r->out;
}

int32_t lt_ramp_get(uint8_t idx)
{
    lt_ramp_t r = _ramp(idx);
    return r ? r->out : 0;
}

uint8_t lt_ramp_is_done(uint8_t idx)
{
    lt_ramp_t r = _ramp(idx);
    if (!r) return 1;
    return (r->out == r->target) ? 1 : 0;
}
