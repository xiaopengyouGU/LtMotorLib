#ifndef CALIB_TASK_H
#define CALIB_TASK_H

#include <stdint.h>

/* ============ 校准阶段 ============ */
typedef enum {
    CALIB_IDLE = 0,         /* 空闲 */
    CALIB_R,                /* 电阻校准 */
    CALIB_L,                /* 电感校准 */
    CALIB_PP_ENC,           /* 极对数 + 编码器方向校准 + 机械和电角度零点偏移 */
    CALIB_DONE,             /* 校准完成 */
} calib_stage_t;

/* ============ 校准结果 ============ */
typedef struct {
    float R;                /* 相电阻 Ω */
    float Ld, Lq;           /* DQ轴相电感：H */
    int   pole_pairs;       /* 极对数 */
    int   encoder_dir;      /* 编码器方向 +1/-1 */
    int32_t encoder_offset; /* 编码器偏移 count */
    uint8_t valid;          /* 校准是否成功 */
} calib_result_t;

void calib_task_init(void);             /* 初始化，进入 IDLE 状态 */
void calib_task_start(void);            /* 启动校准，从 R 阶段开始 */
/* ============ 周期调用接口  ================ */
void calib_task_update(void);           /* 更新激励信号（高频调用） */
void calib_task_add(void);              /* 校准数据采集（高频调用，自动间隔采样） */
void calib_task_run(void);              /* 检查阶段完成并切换（主循环调用） */
uint8_t calib_task_is_done(void);       /* 校准是否完成 */
void calib_task_get_excit(float *Vd, float *Vq, float *angle_el);  /* 激励信号获取 */
/* ============ 周期调用接口  ================ */
void calib_task_get(calib_result_t *result, calib_stage_t *stage); /* 获取校准结果 */

#endif