#ifndef LT_SCURVE_H
#define LT_SCURVE_H

#include <stdint.h>

/*------------------------- 5段S型曲线（全整数，静态实例池）-------------------------
 * Unit 为指令单位，含义由用户自己确定。实例由内部数组提供，idx 就是数组下标
 * （0 ~ LT_SCURVE_MAX_INSTANCES-1），规划器可同时跑多条曲线。
 *
 *     lt_scurve_start(0, &cfg);        // 启动规划器 0（start 内完成全部初始化）
 *     pos = lt_scurve_update(0);       // 每 period_ms 调一次，返回位置指令
 * -----------------------------------------------------------------------------*/

#ifndef LT_SCURVE_MAX_INSTANCES
#define LT_SCURVE_MAX_INSTANCES    2    /* 提供的规划器实例数，索引 0-1 */
#endif

/* Unit 为指令单位，含义由用户自己确定 */
typedef struct {
    int32_t  start_pos;         /* 起始位置（Unit） */
    int32_t  target_pos;        /* 目标位置 (Unit)  */
    int32_t  v_start;           /* 起始速度（Unit/s）*/
    int32_t  v_max;             /* 最大速度（Unit/s）*/
    int32_t  v_stop;            /* 停止速度（Unit/s）*/
    uint16_t acct_ms;           /* 加速时间（ms)，最小 20ms */
    uint16_t dect_ms;           /* 减速时间 (ms)，最小 20ms */
    uint16_t period_ms;         /* 规划更新时间（ms）*/
    uint8_t  type;              /* 0:T形加减速，1：S形加减速 */
} lt_scurve_config_t;

void    lt_scurve_reset(uint8_t idx);                                /* 清空规划器状态 */
void    lt_scurve_start(uint8_t idx, lt_scurve_config_t *config);    /* 规划启动 */
int32_t lt_scurve_update(uint8_t idx);                               /* 每周期调用，返回位置指令（Unit）*/
uint8_t lt_scurve_is_done(uint8_t idx);
void    lt_scurve_stop(uint8_t idx);                                 /* 就地停住 */

#endif
