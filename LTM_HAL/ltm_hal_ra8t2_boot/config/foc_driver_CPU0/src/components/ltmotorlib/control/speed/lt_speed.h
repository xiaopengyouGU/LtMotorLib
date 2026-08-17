/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-20     Lvtou        锁相环（PLL）测试模块实现 
 */

#ifndef LT_SPEED_H
#define LT_SPEED_H

#include <stdint.h>

/*------------------------- PLL测速接口 （高频调用）---------------------------------*/

void lt_speed_init(uint32_t reso, float freq);	    /* reso : 编码器分辨率， freq : 调用频率（Hz） */
void lt_speed_set(uint32_t reso, float freq);		/* reso : 编码器分辨率， freq : 调用频率（Hz） */
float lt_speed_update(int32_t pos_count);		    /* 测速更新，输入编码器原始值：count，高频调用 */
float lt_speed_get(void);                           /* 该接口是给用户调用的，不影响测速计算 */

/*------------------------- PLL测速接口 （高频调用） ---------------------------------*/

#endif
