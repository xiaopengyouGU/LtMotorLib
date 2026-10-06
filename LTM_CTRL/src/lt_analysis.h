/*
 * 聚合公共头（自动生成自各模块头，勿手改；内部实现仍用模块头）
 */
#ifndef LT_ANALYSIS_H__
#define LT_ANALYSIS_H__

#include <stdint.h>

/* 系统参数辨识模块：支持 齿槽/摩擦力/磁链/惯量 辨识 
 * 模块仅负责数据添加和处理，数据质量由用户保证
 * 参数校准模块，一律采用 D轴强拖 
 * 每个接口的风格都是统一的：init/add/is_done/get */

/* ===== R 校准 ===== */
void lt_calib_R_init(uint16_t max_points);
void lt_calib_R_add(float Vd, float Id);        /* 输入 D轴相电压（V）和电流（A）*/
uint8_t lt_calib_R_is_done(void);               /* 数据添加完毕 */
float lt_calib_R_get(void);                     /* 相电阻R: Ω */

/* ===== L 校准 ===== */
void lt_calib_L_init(uint16_t max_points, float amp, float we); /* amp: 参考的Vd幅值（V），we: 角速度(rad/s)*/
void lt_calib_L_add(float ref_sin, float ref_cos, float I); /* 输入参考正余弦信号(-1~1)，和实测信号（A）*/
uint8_t lt_calib_L_is_done(void);
float lt_calib_L_get(void);                     /* 相电感L: H */

/* ===== 极对数 + 编码器方向 ===== */
void lt_calib_pp_init(uint16_t max_points);
void lt_calib_pp_add(float dtheta_elec, float dtheta_mech); /* 添加增量数据 */
uint8_t lt_calib_pp_is_done(void);
void lt_calib_pp_get(int *pole_pairs, int *encoder_dir);

/* ===== 编码器偏移校准 ===== */
void lt_calib_encoder_init(uint16_t points, uint32_t cpr);  /* cpr : 编码器单圈分辨率 */
void lt_calib_encoder_add(uint32_t phase_count, uint32_t encoder_raw);  /* 电角度和机械角度对应计数值：[0, cpr-1) */
uint8_t lt_calib_encoder_is_done(void);
int32_t lt_calib_encoder_get(void);                         /* 获取机械零点和电角度绝对零点之差（count）*/

/* 齿槽转矩辨识（查表法），add接口会自动映射 pos，
 * 不会出现重复添加，直至所有位置都采集完毕
 */
void lt_cogging_init(uint16_t table_size, uint32_t reso); /* 初始化，table_size：单圈采样点数，reso：编码器单圈分辨率 */
void lt_cogging_start(void);                            /* 启动辨识（清空缓冲区）*/
void lt_cogging_add(uint32_t pos_count, int32_t Iq);    /* 添加一个采样点（位置 0~reso-1，等效Q轴电流（Q15标幺化））*/
uint8_t lt_cogging_is_done(void);                       /* 返回是否已采集完毕 */
int32_t lt_cogging_get(uint32_t pos_count);             /* 单圈计数 → 等效Q轴电流（Q15标幺化）*/


/* 摩擦力辨识（支持查表法和拟合法）
 * Fc : 库仑摩擦 （A）, B : 粘性摩擦 （A/RPM） , r2 : 决定系数
 * 采用 Classic 摩擦模型，Tf = Fc*sign(w) + B*w 
 * 采样时，只采正半周或负半周即可。
 */
void lt_friction_init(uint16_t max_points);             /* 初始化摩擦辨识，max_points：最大采样点数 */
void lt_friction_start(void);                           /* 启动辨识（清空缓冲区）*/
void lt_friction_add(float speed, float iq_fric);       /* 添加一个采样点（speed：RPM，iq_fric：摩擦电流 A）*/
uint8_t lt_friction_is_done(void);                      /* 返回是否已采集完毕 */
void lt_friction_solve(void);                           /* 解算摩擦参数 */
void  lt_friction_get2(float *Fc, float *B, float *r2); /* 获取解算后的 Classic摩擦 参数 */
float lt_friction_get(float speed);                     /* speed：RPM，输出查表法等效摩擦力（A）*/

/* 磁链辨识 
 *  phi :  磁链 (Wb), offset : 拟合截距（V）, r2 : 决定系数
 */
void lt_flux_init(uint16_t max_points, float Rs);          /* 初始化磁链辨识，max_points：最大采样点数，Rs：相电阻 Ω */
void lt_flux_start(void);                                  /* 启动辨识（清空缓冲区）*/
void lt_flux_add(float we, float vq, float iq);            /* 添加一个采样点（we：电角速度 rad/s，vq：q轴电压 V，iq：q轴电流 A）*/
uint8_t lt_flux_is_done(void);                             /* 返回是否已采集完毕 */
void lt_flux_solve(void);                                  /* 解算磁链 */
void lt_flux_get(float *phi, float *offset, float *r2);    /* 获取解算后磁链 */


/* 惯量辨识 
 * J : 转动惯量 (kg·m²), J_std : 标准差
 */
void lt_inertia_init(uint16_t max_points);                 /* 初始化惯量辨识，max_points：最大采样点数 */
void lt_inertia_start(void);                               /* 启动辨识（清空缓冲区）*/
void lt_inertia_add(float torque, float accel);            /* 添加一个采样点（torque：电磁转矩 Nm，accel：机械角加速度 rad/s²）*/
uint8_t lt_inertia_is_done(void);                          /* 返回是否已采集完毕 */
void lt_inertia_solve(void);                               /* 解算惯量（调用 lt_stats_mean_std）*/
void lt_inertia_get(float *J, float *J_std);               /* 获取解算后参数 */

/* 系统带宽测量模块：最多支持 64 个扫频点 */

void lt_meas_init(uint16_t points, float ts_s, float offset);      /* points: 扫频点数, ts_s: 采样周期, offset：激励信号偏置量 */
void lt_meas_start(float freq_start, float freq_end, float amp);   /* 启动扫频，输入扫频频率范围和幅值 */   
void lt_meas_add(float data);                                      /* 添加测量数据：高频调用 */
void lt_meas_run(void);                                            /* 测量任务运行：主循环调用 */
float lt_meas_update(void);                                        /* 更新并返回激励信号， 高频调用 */
uint8_t lt_meas_is_done(void);                                     /* 判断测量是否完毕 */
/* idx：扫频点下标（0~points-1），freq_hz : 频率点（Hz）, amp_db : 幅值（dB）, phase_deg：相位滞后（°）*/
void lt_meas_get(float *freq_hz, float *amp_db, float *phase_deg, uint16_t idx); /* 结果获取 */ 


void lt_excit_init(float dt, float offset);     /* 初始化，dt(s), offset：固定偏置量 */
/* 启动各类型波形 */
void lt_excit_start_step(float amplitude);      /* 阶跃信号 */
void lt_excit_start_square(float amplitude, float freq_hz); /* 方波信号 */ 
void lt_excit_start_triangle(float v_peak, float accel);    /* 三角波信号 */
void lt_excit_start_sine(float amplitude, float freq_hz, float phase_rad);  /* 正弦波信号 */
void lt_excit_stop(void);                       /* 停止输出（回到 offset） */
/* type = 0:获取当前激励值, type = 1: 去除偏置后激励，2:获取正交激励（cos）*/
float lt_excit_get(uint8_t type);               
float lt_excit_update(void);                    /* 更新并返回当前激励信号 */

#endif /* LT_ANALYSIS_H__ */
