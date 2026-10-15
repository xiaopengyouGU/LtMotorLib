#ifndef LT_PID_H
#define LT_PID_H

#include <stdint.h>

/*------------------------- 定点 PID（全整数，静态实例池）------------------------
 * 信号：Q15（32767 = 1.0）或 Q24（2^24 = 1.0）都直接吃得下——模块内部不区分格式：
 *       误差、误差增量、二阶差分、输出累加器全走 int64，没有 int32 乘法、不会溢出。
 * 增益：一律 Q15.15（32768 = 1.0）：Kp 无量纲、Ki 1/s、Kd s；
 *       内部存每拍系数 ki = Ki/freq、kd = Kd*freq，换调用频率不用重算增益。
 *       注意 ki 是每拍整数：Ki < freq/2 会被取整成 0，积分项静默消失（4kHz 下 Ki < 2000 就没积分）。
 * 输出：单位与输入信号一致（Q15 进 Q15 出 / Q24 进 Q24 出），限幅用 set_limits 给（可非对称）。
 * idx 是静态池下标（0 ~ LT_PID_MAX_INSTANCES-1）。
 *
 *     lt_pid_init(2, 20000);                    // 实例 2，20kHz 调用
 *     lt_pid_set(2, Kp, Ki, Kd);                // Q15.15 增益
 *     lt_pid_set_limits(2, out_max, out_min);   // 上下限（信号单位）
 *     out = lt_pi_update(2, curr);              // 每拍调用
 * -----------------------------------------------------------------------------*/

#ifndef LT_PID_MAX_INSTANCES
#define LT_PID_MAX_INSTANCES    4       /* 提供的 PID 实例数，索引 0-3 */
#endif

uint8_t lt_pid_init(uint8_t idx, uint32_t freq);

void    lt_pid_reset(uint8_t idx);
void    lt_pid_set(uint8_t idx, int32_t Kp, int32_t Ki, int32_t Kd);
void    lt_pid_set_target(uint8_t idx, int32_t target);
void    lt_pid_set_limits(uint8_t idx, int32_t out_max, int32_t out_min);

int32_t lt_pid_get(uint8_t idx);
int32_t lt_pi_update(uint8_t  idx, int32_t curr_val);   /* 增量式 PI：电流环 / 速度环 */
int32_t lt_pd_update(uint8_t  idx, int32_t curr_val);   /* 位置式 PD：位置环 */
int32_t lt_pid_update(uint8_t idx, int32_t curr_val);   /* 增量式 PID */

#endif
