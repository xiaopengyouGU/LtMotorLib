#ifndef LT_DPCC_H
#define LT_DPCC_H

/* DPCC + ESO : 无差拍预测电流控制 + 拓展状态观测器（估计扰动）*/

/* 初始化 DPCC：电机参数
 *   Ls   : 相电感 (H),       Rs   : 相电阻 (Ω)
 *   phi  : 永磁体磁链 (Wb),  dt   : 控制周期 (s)
 */
void lt_dpcc_init(float Ls, float Rs, float phi, float dt);
/* 配置 ESO 带宽和电压限幅（V），eso_width <= 0 时禁用 ESO 补偿 */
void lt_dpcc_set(float eso_width, float out_limit);
void lt_dpcc_set_target(float id_ref, float iq_ref);    /* 设置电流目标值 */
void lt_dpcc_reset(void);                               /* 重置 ESO 状态 */   
void lt_dpcc_process(float id, float iq, float we);     /* id, iq (A), we : 电角速度 (rad/s) */
void lt_dpcc_get(float *ud, float *uq);                 /* 获取输出DQ轴电压 */

#endif