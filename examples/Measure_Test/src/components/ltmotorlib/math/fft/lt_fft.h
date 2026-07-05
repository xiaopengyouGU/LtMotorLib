#ifndef LT_FFT_H
#define LT_FFT_H

#include <stdint.h>

/* 简易版快速傅里叶变换（FFT） */
void lt_fft_init(uint16_t max_len);     /* 初始化 FFT 引擎（最大点数、查表三角函数）*/
void lt_fft_start(uint16_t len);        /* 开始一次 FFT 采集（清空缓存，准备接收 len 个点）*/
void lt_fft_add(float value);           /* 存入一个采样点（index 自动递增）*/
uint8_t lt_fft_is_ready(void);          /* 返回是否已采集够 len 个点 */
void lt_fft_remove_bias(void);          /* 移除 FFT 采样数据中的固定偏置 */
void lt_fft_process(void);              /* 执行 FFT 计算（主循环调用）*/
float lt_fft_get_amplitude(uint16_t k); /* 获取第 k 条谱线的幅值（真实幅值） */
uint16_t lt_fft_get_len(void);          /* 获取当前 FFT 采样数据点数 */

#endif