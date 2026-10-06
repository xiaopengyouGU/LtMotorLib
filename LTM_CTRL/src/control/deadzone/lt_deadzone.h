#ifndef LT_DEADZONE_H
#define LT_DEADZONE_H

#include <stdint.h>

/*------------------ 电流极性法死区补偿（Q15 标幺）------------------
 * 电流 32767 = i_base_mA；占空比 32767 = 100%
 *   int32_t dutys[3];
 *   lt_deadzone_init(393, 8, 1);            // 死区1.2% / 极性阈值8LSB / 滤波α=1/2^1
 *   lt_deadzone_set(1, 1, 1);               // 三相电流方向与 PWM 正占空比的关系
 *  
 *   lt_deadzone_compensate(ia, ib, ic, dutys); // 每电流环拍一次：三相电流（Q15）进，
 *                                               // 补偿就地叠到 Q15 占空比上
 * ------------------------------------------------------------------*/

void lt_deadzone_init(int32_t dead_duty_q15, int32_t ith_q15, uint8_t alpha_shift);
void lt_deadzone_set(int8_t dirA, int8_t dirB, int8_t dirC);   /* 1 同向，-1 反向 */
void lt_deadzone_compensate(int32_t ia, int32_t ib, int32_t ic, int32_t dutys[3]);

#endif
