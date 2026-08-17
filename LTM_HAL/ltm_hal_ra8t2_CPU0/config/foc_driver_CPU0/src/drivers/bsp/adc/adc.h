#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "hal_data.h"

/*---------------------- ADC 配置宏 ----------------------*/
/* 采样电阻阻值（R），运放增益（G）：
 * 运放输出 Vout = I * G * R ==> I = Vout / (G * R)
 */
/* 电流转换系数，每 LSB 对应的数值 */
#define ADC_CURRENT_PER_LSB     	0.0201416f              /* 3.3f/4096 /(20*0.002) : 每 LSB 对应的电流值 (A) */
#define ADC_VBUS_PER_LSB			0.0233643f				/* 3.3f/4096 * 29 */
#define ADC_TEMP_PER_LSB            0.0008057f              /* 3.3f/4096 */

/* 引脚通道映射 */
#define ADC_CURRENT_U_CHANNEL       ADC_CHANNEL_0           /* U相电流  */
#define ADC_CURRENT_V_CHANNEL       ADC_CHANNEL_2           /* V相电流  */
#define ADC_CURRENT_W_CHANNEL       ADC_CHANNEL_4           /* W相电流  */
#define ADC_VBUS_CHANNEL            ADC_CHANNEL_6           /* 母线电压 */
#define ADC_TEMP_DRIVER_CHANNEL     ADC_CHANNEL_7           /* 驱动器温度 */ 
#define ADC_TEMP_MOTOR_CHANNEL      ADC_CHANNEL_8           /* 电机温度 */

/*---------------------- 类型定义 ----------------------*/
typedef struct {
    float iu;               /* U相电流 (A) */
    float iv;               /* V相电流 (A) */
    float iw;               /* W相电流 (A) */
} adc_current_t;

typedef struct {
    float motor_temp;       /* 电机温度 (°C) */
    float driver_temp;      /* 驱动器温度 (°C) */
    float vbus;             /* 母线电压 (V) */
} adc_temp_vbus_t;

/*---------------------- API ----------------------*/
void adc_init(void);                                     /* ADC 外设初始化 */
void adc_calibrate_zero(uint16_t samples);               /* 电流零点校准 */
void adc_get_current(adc_current_t *current);            /* 获取三相电流 */
void adc_get_temp_vbus(adc_temp_vbus_t *temp);           /* 获取温度、母线电压 */
void adc_set_current_callback(void (*callback)(void));   /* 电流环回调（20kHz） */

#endif