#ifndef ENCODER_SIM_H
#define ENCODER_SIM_H

#include <stdint.h>

/* 模拟编码器对象，用于测试多圈绝对值Encoder驱动 和PLL测试模块*/
/* ============ 模拟编码器参数 ============ */
#define ENCODER_SIM_CPR         262144      /* 18-bit 编码器 */
#define ENCODER_SIM_CPR_F       262144.0f
#define ENCODER_SIM_GEAR_RATIO  0.9545455f  /* 42/44 */
#define ENCODER_SIM_CALL_FREQ   25000.0f    /* 25kHz */
#define ENCODER_SIM_PI          3.1415927f

/* ============ 编码器模拟器句柄 ============ */
typedef struct {
    double position;        /* 累计角度 (rad) */
    double velocity;        /* 角速度 (rad/s) */
    uint32_t count;         /* 编码器计数值 [0, CPR-1] */
} encoder_sim_t;

typedef enum{
    Spindle_Axis = 0,
    Driven_Axis,
    Encoder_Axis_Num, 
}encoder_axis_t;

/* ============ 接口函数 ============ */

void encoder_sim_init(double init_spindle_turns);   /* 初始化主轴编码器（从轴自动计算） */
void encoder_sim_set_speed(double speed_rpm);       /* 设置主轴速度（RPM），从轴自动反向并乘以齿轮比 */
uint32_t encoder_sim_get_count(encoder_axis_t axis);/* 获取当前轴编码器计数值（单圈） */
double encoder_sim_get_turns(encoder_axis_t axis);  /* 获取当前轴累计角度（圈数） */
void encoder_sim_step(void);                        /* 更新模拟器状态（步进一个采样周期） */
float encoder_sim_get_freq(void);                   /* 获取当前的调用频率 */

#endif /* ENCODER_SIM_H */