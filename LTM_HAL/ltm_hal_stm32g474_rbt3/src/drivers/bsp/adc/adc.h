#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include "stm32g4xx_hal.h"

/*---------------------- ADC 配置宏 ----------------------*/
/* 采样电阻阻值（R），运放增益（G）：
 * 运放输出 Vout = I * G * R ==> I = Vout / (G * R)
 */
/* 电流转换系数，每 LSB 对应的数值（12bit）*/
#define ADC_CURRENT_PER_LSB     	0.0161133f  /* 3.3f/4096 /(10*0.005) : 每 LSB 对应的电流值(A) */
#define ADC_VBUS_PER_LSB			0.0241184f  /* 3.3f/4096 * 29.936 : 每 LSB 对应的电压值(V) */

/* ADC 引脚通道映射（12bit，三 ADC 同步采样）：
 *   IA -> ADC1_IN6  (PC0)
 *   IB -> ADC2_IN7  (PC1)
 *   IC -> ADC3_IN1  (PB0)
 * 三颗 ADC 注入组由 HRTIM TRG5/TRG9 触发（TimerA 谷底；ADC1/ADC2 共用 TRG5）+ 相同采样时间，
 * 同一触发沿同时刻采样，硬件保证 Ia+Ib+Ic=0（配合温漂偏置扣除）。
 */
#define ADC_CURRENT_A_CHANNEL       ADC_CHANNEL_6       /* PC0：A 相电流 -> ADC1 */
#define ADC_CURRENT_B_CHANNEL       ADC_CHANNEL_7       /* PC1：B 相电流 -> ADC2 */
#define ADC_CURRENT_C_CHANNEL       ADC_CHANNEL_12      /* PB0：C 相电流 -> ADC3（IN12）*/
#define ADC_VBUS_CHANNEL            ADC_CHANNEL_9       /* PC3：母线电压 -> ADC1 */
#define ADC_TEMP_MOTOR_CHANNEL      ADC_CHANNEL_1       /* PA0：电机温度 -> ADC1（IN1）*/
#define ADC_TEMP_DRIVER_CHANNEL     ADC_CHANNEL_2       /* PA1：驱动器温度 -> ADC1（IN2）*/

/* Q15 标幺接口（与 lt_foc_update 同口径）：
 * 电流 1.0 pu = LTM_ADC_FULL_SCALE_CURRENT_A（33.0A = 半量程 2048 LSB × 0.0161133A/LSB）
 * 母线 1.0 pu = LTM_ADC_FULL_SCALE_VOLTAGE_V（98.79V = 4096 LSB × 0.0241184V/LSB）
 * 温度单位 0.1℃ */
/*---------------------- API ----------------------*/
void adc_init(void);                                        /* ADC 外设初始化 */
void adc_calibrate_zero(uint16_t samples);                  /* 电流零点校准 */
void adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic); /* 获取三相电流（Q15 标幺）*/
void adc_get_temp(int32_t *motor_temp, int32_t *driver_temp); /* 获取电机和驱动器温度（0.1℃）*/
void adc_get_vbus(int32_t *vbus);                             /* 获取母线电压（Q15 标幺）*/
void adc_set_callback(void (*callback)(void));              /* 用户 ADC 扫描完成回调，25kHz */

#endif
