#ifndef LT_SCURVE_H
#define LT_SCURVE_H

#include <stdint.h>

/*------------------------- 在线轨迹规划器（全整数，静态实例池）-------------------------
 * Unit 为指令单位，含义由用户自己确定。实例由内部数组提供，idx 就是数组下标
 * （0 ~ LT_SCURVE_MAX_INSTANCES-1），规划器可同时跑多条曲线。
 *
 * 目标位置随时可改：规划器只持有 (位置, 速度) 两个指令状态，每拍自己解下一拍，
 * 不预解算曲线、不缓存终点。位置从当前位置走、速度从当前速度走，改目标不跳变；
 * 目标落在背后时判据会先把它刹到 0 再反向。
 *
 *     lt_scurve_start(0, &cfg);              // 建状态：初值 = (start_pos, v_start)
 *     pos = lt_scurve_update(0);             // 每 period_ms 推一拍，返回位置指令
 *     vel = lt_scurve_get_vel(0);            // 速度指令（可直接做速度前馈）
 *     lt_scurve_set_target(0, new_target);   // 运行中随时改目标
 * -----------------------------------------------------------------------------*/

#ifndef LT_SCURVE_MAX_INSTANCES
#define LT_SCURVE_MAX_INSTANCES    2    /* 提供的规划器实例数，索引 0-1 */
#endif

/* Unit 为指令单位，含义由用户自己确定 */
typedef struct {
    int32_t  start_pos;         /* 起始位置（Unit） */
    int32_t  target_pos;        /* 目标位置（Unit） */
    int32_t  v_start;           /* 起始速度（Unit/s，带符号）*/
    int32_t  v_max;             /* 速度上限（Unit/s，取绝对值）*/
    uint16_t acct_ms;           /* 加速时间（ms），最小 20ms：a_acc = v_max/acct_ms */
    uint16_t dect_ms;           /* 减速时间（ms），最小 20ms：a_dec = v_max/dect_ms */
    uint16_t period_ms;         /* 规划更新周期（ms）*/
    uint16_t jerk_ms;           /* S 形 jerk 段时长（ms）：加速度 0→a_max 耗时；0 = 退回 acct_ms/2 */
    uint8_t  type;              /* 0:T形加减速，1：S形加减速（加速度按 jerk 渐变）*/
} lt_scurve_config_t;

void    lt_scurve_reset(uint8_t idx);                                /* 清空规划器状态 */
void    lt_scurve_start(uint8_t idx, lt_scurve_config_t *config);    /* 建状态（含位置/速度初值）*/
void    lt_scurve_set_target(uint8_t idx, int32_t target);           /* 在线改目标，随时可调 */
int32_t lt_scurve_update(uint8_t idx);                               /* 每周期调用，返回位置指令（Unit）*/
int32_t lt_scurve_get_vel(uint8_t idx);                              /* 速度指令（Unit/s）*/
uint8_t lt_scurve_is_done(uint8_t idx);                              /* 已停在目标上 */
void    lt_scurve_stop(uint8_t idx);                                 /* 就地停住 */

#endif
