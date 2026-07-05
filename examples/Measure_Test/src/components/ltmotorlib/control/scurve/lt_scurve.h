#ifndef LT_SCURVE_H
#define LT_SCURVE_H

#include <stdint.h>

/*------------------------- 5段S型曲线 ---------------------------------*/
typedef struct lt_scurve_object * lt_scurve_t;

typedef struct {
    int32_t  start_pos;         /* 起始位置（Unit） */
    int32_t  target_pos;        /* 目标位置 (Unit)  */
    int32_t  v_start;           /* 起始速度（Unit/s）*/
    int32_t  v_max;             /* 最大速度（Unit/s）*/
    int32_t  v_stop;            /* 停止速度（Unit/s）*/
    uint16_t acct_ms;           /* 加速时间（ms），最小 100ms */
    uint16_t dect_ms;           /* 减速时间 (ms)，最小 100ms */
    uint16_t period_ms;         /* 规划更新时间（ms）*/
    uint8_t  type;              /* 0:T形加减速，1：S形加减速 */
} lt_scurve_config_t;

lt_scurve_t lt_scurve_create(void);
void lt_scurve_reset(lt_scurve_t s);
void lt_scurve_start(lt_scurve_t s, lt_scurve_config_t *config);
int32_t lt_scurve_update(lt_scurve_t s);
uint8_t lt_scurve_is_done(lt_scurve_t s);
void lt_scurve_stop(lt_scurve_t s);
void lt_scurve_delete(lt_scurve_t s);

#endif