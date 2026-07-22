/*  带宽测试引擎接口: 
 *   - 支持电流环（20kHz采样率）和速度环（3kHz采样率）带宽测试
 *   - 44个频率点，变点数FFT（2048/1024/512）
 *   - 输出每个频率点的幅值（dB）和相位滞后（度）
 */

#ifndef LT_MEAS_H
#define LT_MEAS_H

#include <stdint.h>

typedef enum {
    LT_MEAS_MODE_CURRENT = 0,
    LT_MEAS_MODE_SPEED    = 1
} lt_meas_mode_t;

typedef struct {
    float freq;           /* Hz */
    float amp;            /* 幅值 */
    float phase_lag_deg;  /* 相位滞后（度，正值表示滞后）*/
} lt_meas_result_t;

void lt_meas_init(lt_meas_mode_t mode, float fix_freq);  /* 初始化扫频模块 */
void lt_meas_set(lt_meas_mode_t mode, float fix_freq);   /* 设置扫频模块 */
void lt_meas_start(void);                /* 扫频测试启动 */
void lt_meas_add(float value);           /* 添加测量值 */
uint8_t lt_meas_process(void);           /* 主循环调用，返回1表示一个频点完成 */
uint8_t lt_meas_is_complete(void);       
uint8_t lt_meas_get_progress(void);
uint16_t lt_meas_get_len(void);          /* 获取扫频点个数 */
float lt_meas_get_freq();                /* 获取本次扫频频率 （Hz）*/
void lt_meas_get(lt_meas_result_t * res, uint8_t index); /* 获取扫频测试结果, index : 扫频点位置 */ 

#endif