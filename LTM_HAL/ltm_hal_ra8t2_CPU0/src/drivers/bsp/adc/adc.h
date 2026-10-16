#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>
#include "hal_data.h"

/*---------------------- ADC 配置宏 ----------------------*/
/* 采样电阻阻值（R），运放增益（G）：
 * 运放输出 Vout = I * G * R ==> I = Vout / (G * R)
 */
/* 电流转换系数，每 LSB 对应的数值 */
#define ADC_CURRENT_PER_LSB     	0.0012589f              /* 3.3f/65536 /(20*0.002)：16bit，每 LSB 电流(A) */
#define ADC_VBUS_PER_LSB			0.0233643f				/* 3.3f/4096 * 29：12bit，每 LSB 电压(V) */
#define ADC_TEMP_PER_LSB            0.0008057f              /* 3.3f/4096：12bit，NTC 分压电压(V) */

/* Q15 标幺接口（与 lt_foc_update 同口径）：
 * 电流 1.0 pu = 半量程 32768 LSB × 0.0012589 A/LSB = 41.25 A（16bit 中位偏置）
 * 母线 1.0 pu = 满量程 4096 LSB × 0.0233643 V/LSB = 95.70 V（12bit）
 * 温度单位 0.1℃（NTC 查表）*/

/* 引脚通道映射 */
#define ADC_CURRENT_A_CHANNEL       ADC_CHANNEL_0           /* A相电流  */
#define ADC_CURRENT_B_CHANNEL       ADC_CHANNEL_2           /* B相电流  */
#define ADC_CURRENT_C_CHANNEL       ADC_CHANNEL_4           /* C相电流  */
#define ADC_VBUS_CHANNEL            ADC_CHANNEL_6           /* 母线电压 */
#define ADC_TEMP_DRIVER_CHANNEL     ADC_CHANNEL_7           /* 驱动器温度 */ 
#define ADC_TEMP_MOTOR_CHANNEL      ADC_CHANNEL_8           /* 电机温度 */

/*---------------------- API ----------------------*/
void adc_init(void);                                        /* ADC 外设初始化 */
void adc_calibrate_zero(uint16_t samples);                  /* 电流零点校准 */
void adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic); /* 获取三相电流（Q15 标幺）*/
void adc_get_temp(int32_t *motor_temp, int32_t *driver_temp); /* 获取电机和驱动器温度（0.1℃）*/
void adc_get_vbus(int32_t *vbus);                             /* 获取母线电压（Q15 标幺）*/
void adc_set_callback(void (*callback)(void));              /* 用户 ADC 扫描完毕回调（25kHz）*/

#endif
