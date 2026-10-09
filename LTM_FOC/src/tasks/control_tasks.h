#ifndef CONTROL_TASKS_H
#define CONTROL_TASKS_H
/* 三环控制任务（全定点）：调度、指令、跨环给定、上报都收在这一个模块里
 *
 * 内部域：位置 Q24（1.0 = 一圈）、速度 Q24（1.0 = 一圈/s）、电流/电压 Q15 标幺、
 *        母线 Q15、温度 0.1℃。应用单位（°/RPM/%/A）只在 lt_motor_set 和
 *        lt_motor_get_info 两个边界出现。
 * 上下文：run 在 ADC 中断里跑（20kHz）；其余都在 main 上下文。
 * 分频：电流环每拍、速度环 CURRENT_LOOP_HZ/SPEED_DIV = 4kHz、
 *      位置环 CURRENT_LOOP_HZ/POS_DIV = 1kHz。
 */
#include <stdint.h>
#include "common/tasks_types.h"

void control_tasks_init(void);                          /* 初始化 foc / PID 池 / 测速 / 死区补偿 / 辨识 */
void control_tasks_run(void);                           /* 三环级联分频，ADC 中断里每拍调 */
void control_tasks_set(tasks_mode_t mode, int32_t target);   /* 设置模式与目标；target 已按 mode 编码 */
void control_tasks_stop(uint8_t estop);
void control_tasks_get(tasks_info_t *info);             /* 取内部快照（控制域单位，不含浮点）*/
void control_tasks_zero_find(void);                     /* 电角度绝对零点找寻 */
void control_tasks_plan_start(void);                    /* 位置模式重规划（Start 时调；运行中改目标由 set 触发）*/
tasks_mode_t control_tasks_get_mode(void);              /* 只获取模式快照 */

#endif
