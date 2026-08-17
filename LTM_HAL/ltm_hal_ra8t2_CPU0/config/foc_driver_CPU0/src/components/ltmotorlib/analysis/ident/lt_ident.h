#ifndef LT_IDENT_H
#define LT_IDENT_H

/* 系统参数辨识模块：支持 齿槽/摩擦力/磁链/惯量 辨识 
 * 模块仅负责数据添加和处理，数据质量由用户保证
 */
#include <stdint.h>

/* 齿槽转矩辨识（查表法），add接口会自动映射 pos，
 * 不会出现重复添加，直至所有位置都采集完毕
 */
void lt_cogging_init(uint16_t table_size);              /* 初始化，table_size：单圈采样点数 */
void lt_cogging_start(void);                            /* 启动辨识（清空缓冲区）*/
void lt_cogging_add(float pos, float iq);               /* 添加一个采样点（位置 0~360°，等效Q轴电流（A））*/
uint8_t lt_cogging_is_done(void);                       /* 返回是否已采集完毕 */
float lt_cogging_get(float pos);                        /* 获取补偿的等效Q轴电流（A）*/


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

#endif