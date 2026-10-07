#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "hal_data.h"

/*---------------------- 编码器配置宏 ----------------------*/
#define ENCODER_CPR                 10000           /* 编码器分辨率（2500线，四倍频）*/
#define ENCODER_CPR_HALF            5000            /* 编码器一半的分辨率（用于半圈法圈数更新）*/

/*---------------------- API ----------------------*/
void encoder_init(void);      
void encoder_update(void);                          /* 手动更新编码器计数（高频调用）*/              
uint32_t encoder_get_count(void);                   /* 获取当前计数值（0~ENCODER_CPR-1）*/
int64_t  encoder_get_position(void);                /* 获取累计位置（计数值，可跨圈） */
void encoder_set_zero(void);                        /* 软归零（清零硬件计数器和圈数） */

#endif /* _ENCODER_H_ */
