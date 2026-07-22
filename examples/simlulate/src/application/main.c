#include <stdio.h>
#include "encoder_test/encoder_test.h"
#include "ltm_test/ltm_test.h"

int main(void)
{
    //encoder_speed_test();          /* PLL测速测试，验证算法逻辑与性能 */
    encoder_test();                  /* 绝对值编码器测试，验证上电解圈与SPI数据解析 */
    // ltm_test();                    /* LTM 通讯协议测试，采用虚拟串口和LTM_Monitor */ 
    return 0;
}

// #include <stdio.h>
// #include <math.h>

// #include "analysis/meas/lt_meas.h"
// #include "math/fft/lt_fft.h"
// #include "math/basic/lt_math.h"

// #include "system/uart/uart.h"
// #include "system/delay/delay.h"
// #include "protocol/ltm_commut.h"

// #define FS      20000.0f    /* 电流环采样率 20kHz */
// #define DT      (1.0f/FS)

// /* 简单随机数生成器（0 ~ 1 均匀分布） */
// static float randf(void) {
//     static unsigned int seed = 123456789;
//     seed = seed * 1103515245 + 12345;
//     return (float)(seed & 0x7fffffff) / 2147483647.0f;
// }


// int main(void)
// {
//     /* 1. 初始化扫频模块 */
//     lt_meas_init(LT_MEAS_MODE_CURRENT, 0.0f);   /* 0 表示扫频模式，>0 表示定点 */
//     lt_meas_start();

//     float t = 0.0f;
//     uint8_t done = 0;
//     float iq_fb = 2.0;
//     /* 2. 主循环：等待扫频完成 */
//     while (!done) {
//         /* 模拟 20kHz 中断：获取当前频点频率，生成注入信号 */
//         float freq = lt_meas_get_freq();          /* 当前扫频点频率（Hz） */
//         float cmd = 2.0f + 0.3f * sinf(_2_PI * freq * t);
//         float iq_raw = cmd + 0.02f * (randf() - 0.5f);   /* 模拟反馈 */
//         iq_fb = LOW_PASS_FILTER(iq_raw, iq_fb, 0.35f);

//         lt_meas_add(iq_fb);                       /* 添加测量数据 */
//         t += DT;
//         done = lt_meas_is_complete();
//         lt_meas_process();                        /* 主循环处理 FFT */
//     }

//     /* 3. 输出结果：44 个频点的幅值和相位 */
//     printf("\nfrquency(Hz), amp, phase_lag(degree)\n");
//     for (uint8_t i = 0; i < lt_meas_get_len(); i++) {
//         lt_meas_result_t res;
//         lt_meas_get(&res, i);
//         printf("%.2f, %.3f, %.2f\n", res.freq, res.amp, i * 3.0f);
//     }

//     return 0;
// }