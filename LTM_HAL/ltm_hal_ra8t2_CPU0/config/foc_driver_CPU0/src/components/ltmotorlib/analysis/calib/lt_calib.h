#ifndef LT_CALIB_H
#define LT_CALIB_H

#include <stdint.h>

/* 参数校准模块，一律采用 D轴强拖 
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

#endif