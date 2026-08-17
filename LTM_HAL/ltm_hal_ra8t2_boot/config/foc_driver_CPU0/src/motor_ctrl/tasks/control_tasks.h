#ifndef CONTROL_TASKS_H
#define CONTROL_TASKS_H
/* 三环控制任务 */
#include <stdint.h>
#include "motor_ctrl/common/lt_motor_types.h"   

void current_loop_task(void);          /* 电流环 ：高频，默认 20kHz 运行 */
void speed_loop_task(void);            /* 速度环 ：中频，默认 3kHz 运行 */
void position_loop_task(void);         /* 位置环 ：低频，默认 1kHz 运行 */
void control_tasks_init(void);         /* 控制任务初始化 */
/* 设置控制模式，0:开环，1:位置模式（角度°），2:速度模式（RPM），3:力矩模式（N.m）*/
void control_tasks_set(lt_motor_mode_t mode, float target);  
void control_tasks_get(lt_motor_info_t *info);   /* 获取电机信息 */
void control_tasks_get2(uint32_t *encoder_raw, float *angle_el); /* 获取编码器原始计数值（0~CPR-1）和电角度（rad）*/
#endif