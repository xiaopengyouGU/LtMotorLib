#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#define FOC_COUNT_PERIOD         34000      /* HRTIM 半周期计数值（HR 时钟单位：MUL8 = 8×170MHz = 1.36GHz，
                                             25µs×1.36GHz = 34000，步进 735ps；载波 20kHz，与电流环同频。
                                             MUL16 在 20kHz 下需要 PER=68000 超 16bit，故降为 MUL8）*/
#define FOC_TOTAL_PERIOD         68000      /* 完整 PWM 计数值（参考值；PER 寄存器仅 16bit，实际只写半周期 34000）*/

/* HRTIM1 互补输出相序映射（实际硬件）(H/L)：
 * A 相 = Timer A（PA8/PA9），B 相 = Timer D（PB14/PB15），C 相 = Timer C（PB12/PB13）
 * 中心对称 PWM（up-down）+ DTEN 互补死区（800ns）
 * ADC 触发：TimerA 谷底 RESET -> HRTIM TRG1/TRG5/TRG9 -> ADC1/ADC2/ADC3 */
/*---------------------- API ----------------------*/
/* PWM 初始化与占空比设置（HRTIM1） */
void pwm_init(void);
void pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC); /* 三相占空比（Q15：32768 = 100%）*/
void pwm_start(void);
void pwm_stop(void);

#endif
