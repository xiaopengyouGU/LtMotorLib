#ifndef LT_RAMP_H
#define LT_RAMP_H

#include <stdint.h>

/*------------------------- 斜坡发生器 -------------------------
 * 每拍把输出朝目标挪 step（Unit/拍），Unit：用户单位
 *     lt_ramp_start(0, cur, target, step);   // 起状态（step = Unit/拍，取绝对值）
 *     out = lt_ramp_update(0);               // 每拍推一步，返回当前输出
 * -----------------------------------------------------------------------------*/

#ifndef LT_RAMP_MAX_INSTANCES
#define LT_RAMP_MAX_INSTANCES      2    /* 提供的斜坡实例数，索引 0-1 */
#endif

void    lt_ramp_reset(uint8_t idx);                                     /* 清空斜坡状态 */
void    lt_ramp_start(uint8_t idx, int32_t start, int32_t target, int32_t step);
void    lt_ramp_set_target(uint8_t idx, int32_t target);                /* 在线改目标 */
void    lt_ramp_set_step(uint8_t idx, int32_t step);                    /* 切换斜率 */
int32_t lt_ramp_update(uint8_t idx);                                    /* 输出值更新 */
int32_t lt_ramp_get(uint8_t idx);                                       /* 获取当前输出 */
uint8_t lt_ramp_is_done(uint8_t idx);                                   /* 输出达目标值 */

#endif
