#ifndef LT_MEAS_H
#define LT_MEAS_H

#include <stdint.h>

/* 系统带宽测量模块：最多支持 64 个扫频点 */

void lt_meas_init(uint16_t points, float ts_s, float offset);      /* points: 扫频点数, ts_s: 采样周期, offset：激励信号偏置量 */
void lt_meas_start(float freq_start, float freq_end, float amp);   /* 启动扫频，输入扫频频率范围和幅值 */   
void lt_meas_add(float data);                                      /* 添加测量数据：高频调用 */
void lt_meas_run(void);                                            /* 测量任务运行：主循环调用 */
float lt_meas_update(void);                                        /* 更新并返回激励信号， 高频调用 */
uint8_t lt_meas_is_done(void);                                     /* 判断测量是否完毕 */
/* idx：扫频点下标（0~points-1），freq_hz : 频率点（Hz）, amp_db : 幅值（dB）, phase_deg：相位滞后（°）*/
void lt_meas_get(float *freq_hz, float *amp_db, float *phase_deg, uint16_t idx); /* 结果获取 */ 

#endif