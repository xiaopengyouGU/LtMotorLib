/* lt_ladrc.h */
#ifndef LT_LADRC_H
#define LT_LADRC_H

/* 初始化 LADRC
 *   j_over_kt   : J / Kt (kg·m² / Nm/A),       dt : 控制周期 (s)
 *   speed_max   : 速度限幅 (rad/s),    accel_max   : 加速度限幅 (rad/s²)
 */
void lt_ladrc_init(float j_over_kt, float speed_max, float accel_max, float dt);
/*   kp : 比例增益， out_limit   : 输出限幅 (A)
 *   eso_width   : ESO 带宽 (rad/s)，<= 0 时禁用 ESO 补偿 */
void lt_ladrc_set(float kp, float eso_width, float out_limit);
void lt_ladrc_set_target(float speed_ref);   /* 设置速度目标值 (rad/s) */
void lt_ladrc_reset(void);                   /* 重置 ESO 状态 */
void lt_ladrc_process(float pos, float iq);  /* pos : 当前位置 (rad，累计角度), iq : q 轴电流 (A) */
/*iq_ref: 生成的 q 轴电流指令 (A)，speed_est : 速度估计 (rad/s) */
void lt_ladrc_get(float *iq_ref, float *speed_est); 

#endif