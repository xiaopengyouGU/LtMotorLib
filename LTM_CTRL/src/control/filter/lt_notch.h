#ifndef LT_NOTCH_H
#define LT_NOTCH_H

#include <stdint.h>

/*------------------------- 陷波滤波器（机械共振抑制）---------------------------
 * 电流环专用，三级（一般 1~2 个齿轮谐振 + 1 个联轴器谐振），全定点，模块内无浮点。
 *
 *     lt_notch_init(20000);                       // 20kHz，先给调用频率
 *     lt_notch_set(0, 1200, 10, -20);             // 第0级：1.2kHz，Q=10，-20dB
 *     lt_notch_set(1, 2600, 15, -15);             // 第1级
 *     ...
 *     curr = lt_notch_update(curr);                 // 每拍调一次，Q15 进 Q15 出
 *
 * level    : 级号 0~2，按调用顺序串起来
 * fc_Hz    : 陷波中心频率 (Hz)，要低于 freq/2，建议 ≥ 100Hz
 * Q        : 品质因数 (5~20)，越大陷波越窄
 * depth_dB : 陷波深度（负整数 dB），常用 -20 ~ -40，≤ -60 按理想陷波算
 * -----------------------------------------------------------------------------*/

void    lt_notch_init(uint32_t freq);                  /* 调用频率 (Hz) */
void    lt_notch_set(uint8_t level, uint32_t fc_Hz, uint32_t Q, int32_t depth_dB);
void    lt_notch_reset(void);                          /* 清空状态，不改变系数 */
int32_t lt_notch_update(int32_t x);                    /* Q15 进 Q15 出 */

#endif
