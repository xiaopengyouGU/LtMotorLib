/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        The first version
 * 2025-10-21     Lvtou        Remove dependancies on RT-Thread
 * 2025-11-29     Lvtou        Modity API and improve computation efficiency
 * 2025-12-15     Lvtou        Exchange the implementations of two types of pid
 * 2026-08-31     Lvtou        实现增量式 PI 接口，无微分运算，性能更好
 * 2026-9-25      Lvtou        全定点化：增益与调用周期全整数，模块零浮点零库调用
 * 2026-9-26      Lvtou        增益改 Q15、Ki·ts 保持 Q24；I 项移位修正；累加器小数位自适应
 * 2026-9-26      Lvtou        去 malloc：idx 索引 + 静态实例池，lt_pid_init 返回 0/1
 * 2026-9-27      Lvtou        init 声明信号格式（Q15/Q24）；增益改 Q24，两种格式共用同一算式
 * 2026-9-27      Lvtou        增益改回 Q15.15（1.0 = 32768，上限 ±65536），I/D 内部存每拍系数
 */
#ifndef LT_PID_H
#define LT_PID_H

#include <stdint.h>

/*------------------------- 定点 PID（全整数，静态实例池）-------------------------
 * 信号格式 init 声明（type）：0 = Q15（32767 = 1.0）、1 = Q24（2^24 = 1.0）。
 * 增益一律 Q15.15（32768 = 1.0）：Kp 无量纲、Ki 1/s、Kd s；内部存每拍系数
 * Ki·ts / Kd·freq，故换调用频率不用重算增益。idx 是静态池下标（0 ~ 上限-1）。
 * Q24 实例误差 target-curr 按 ±2^30 饱和；输出限幅默认给满量程，可 set_limits 改。
 *
 *     lt_pid_init(2, 0, 20000);                 // Q15 信号，20kHz
 *     lt_pid_set(2, Kp, Ki, Kd);                // Q15.15 增益
 *     lt_pid_set_limits(2, out_max, out_min);   // 可非对称
 *     out = lt_pi_update(2, curr);              // 每拍调用
 * -----------------------------------------------------------------------------*/

#ifndef LT_PID_MAX_INSTANCES
#define LT_PID_MAX_INSTANCES    4       /* 提供的 PID 实例数，索引 0-3 */
#endif

uint8_t lt_pid_init(uint8_t idx, uint8_t type, uint32_t freq); 

void    lt_pid_reset(uint8_t idx);
void    lt_pid_set(uint8_t idx, int32_t Kp, int32_t Ki, int32_t Kd);
void    lt_pid_set_target(uint8_t idx, int32_t target);
void    lt_pid_set_limits(uint8_t idx, int32_t out_max, int32_t out_min);

int32_t lt_pid_get(uint8_t idx);
int32_t lt_pi_update(uint8_t  idx, int32_t curr_val);   /* 增量式 PI，电流环 / 速度环 */
int32_t lt_pd_update(uint8_t  idx, int32_t curr_val);   /* 增量式 PD，位置环 */
int32_t lt_pid_update(uint8_t idx, int32_t curr_val);   /* 增量式 PID */

#endif
