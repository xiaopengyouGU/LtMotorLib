#ifndef LT_SPEED_H
#define LT_SPEED_H

#include <stdint.h>

/*------------------------- 自适应测速接口 ---------------------------------*/

void lt_speed_init(uint32_t reso, float freq);				/* reso : 编码器分辨率（增量式4倍频）， freq : 速度环调用频率 */
void lt_speed_set(uint32_t reso, float freq);				/* reso : 编码器分辨率（增量式4倍频）， freq : 速度环调用频率 */
float lt_speed_get(int32_t pos_unit);						/* 输入编码器原始值： Unit */
float lt_speed_get2(void);                                  /* 该接口是给用户调用的，不影响测速计算 */

/*------------------------- 自适应测速接口 ---------------------------------*/

#endif