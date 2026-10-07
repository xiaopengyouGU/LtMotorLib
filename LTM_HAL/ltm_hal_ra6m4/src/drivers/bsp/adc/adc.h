#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "hal_data.h"

//#define TEST_LOOP_TIME          1       /* 启动电流环执行时间测量：翻转GPIO */
#define TEST_GPIO_PORT_PIN      BSP_IO_PORT_03_PIN_06     

/*---------------------- ADC 配置宏 ----------------------*/
/* 采样电阻阻值（R），运放增益（G）：
 * 运放输出 Vout = I * G * R ==> I = Vout / (G * R)
 * 以ADC满量程确定电流和母线电压的标幺基准（Q15）
 */

/* ADC 引脚通道映射（12bit，双 ADC 同步采样）：
 *   IA -> ADC0_IN1  (P001)
 *   IB -> ADC1_IN0  (P000)
 * 双ADC在 中心对齐PWM下溢时刻触发采样，对应下桥采样，硬件保证 Ia+Ib+Ic=0。
 */
#define ADC_CURRENT_A_CHANNEL       ADC_CHANNEL_1       /* P001：A 相电流 -> ADC0 */
#define ADC_CURRENT_B_CHANNEL       ADC_CHANNEL_0       /* P000：B 相电流 -> ADC1 */
#define ADC_VBUS_CHANNEL            ADC_CHANNEL_3       /* P003：母线电压 -> ADC0 */
#define ADC_TEMP_DRIVER_CHANNEL     ADC_CHANNEL_7       /* P007：驱动器温度 -> ADC0 */
// #define ADC_TEMP_MOTOR_CHANNEL      ADC_CHANNEL_9       /* P009：电机温度 -> ADC0 */


/*---------------------- API ----------------------*/
void adc_init(void);                                         /* ADC 外设初始化 */
void adc_calibrate_zero(uint16_t samples);                   /* 电流零点校准 */
void adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic); /* 获取三相电流，Q15标幺值 */
void adc_get_temp(int32_t *motor_temp, int32_t *driver_temp);/* 获取电机和驱动器温度（单位:0.1℃） */
void adc_get_vbus(int32_t *vbus);                            /* 获取母线电压，Q15标幺值 */
void adc_set_callback(void (*callback)(void));               /* 用户 ADC 扫描完毕回调 */

#endif
