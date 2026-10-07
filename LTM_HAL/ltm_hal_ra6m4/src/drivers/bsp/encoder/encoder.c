/*
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-19     Lvtou        增量式编码器驱动实现
 */
#include "encoder.h"
#include <string.h>

static timer_ctrl_t * tim_ctrl = &g_timer3_ctrl;     /* 编码器用定时器 */
static timer_cfg_t  * tim_cfg  = &g_timer3_cfg;
static timer_status_t tim_status = {0};

/* 多圈绝对值编码器由两个单圈绝对值编码器构成 */
typedef struct {                    /* 单圈编码器实例 */
    uint32_t count_last;            /* 上一次计数值，用于圈数更新 */
    uint32_t count;                 /* 编码器原始计数值 */
    int32_t  circle_count;          /* 圈数计数器 */
} encoder_obj_t;

static encoder_obj_t encoder_obj;
static encoder_obj_t *encoder = &encoder_obj;

void encoder_init(void)
{
    R_GPT_Open(tim_ctrl, tim_cfg);                 /* 打开定时器 */
    R_GPT_Enable(tim_ctrl);                        /* 使能计数 */
    R_GPT_Start(tim_ctrl);                         /* 启动定时器 */
    /* 初始化变量 */
    memset(encoder, 0, sizeof(encoder_obj_t));
}

void encoder_update(void)                          /* 手动更新编码器计数（高频调用）*/   
{
    R_GPT_StatusGet(tim_ctrl, &tim_status);
    uint32_t count =  tim_status.counter;
    int32_t  count_delt = (int32_t)count - (int32_t)encoder->count_last;
    /* 编码器累积圈数更新（半圈法），该接口高频调用，半圈法不会失效 */
    if (count_delt > ENCODER_CPR_HALF) {            /* 主轴反转 */
        encoder->circle_count--;
    } else if (count_delt < -ENCODER_CPR_HALF) {    /* 主轴正转 */
        encoder->circle_count++;
    }
    /* 编码器计数值更新 */
    encoder->count = count;
    encoder->count_last = count;
}

uint32_t encoder_get_count(void)                    /* 获取当前计数值（0~ENCODER_CPR -1）*/
{
    return encoder->count;                 
}

int64_t  encoder_get_position(void)                 /* 获取累计位置（计数值，可跨圈） */
{
    return (int64_t)encoder->count + ENCODER_CPR * (int64_t)encoder->circle_count;
}

void encoder_set_zero(void)                        /* 软归零（清零硬件计数器和圈数） */
{
    R_GPT_Stop(tim_ctrl);                          /* 先关闭定时器 */
    R_GPT_CounterSet(tim_ctrl, 0);                 /* 然后才能清零计数值 */
    memset(encoder, 0, sizeof(encoder_obj_t));   
    R_GPT_Start(tim_ctrl);                         /* 重新打开定时器 */
	R_GPT_Enable(tim_ctrl);					       /* 重新启动计数 */
}