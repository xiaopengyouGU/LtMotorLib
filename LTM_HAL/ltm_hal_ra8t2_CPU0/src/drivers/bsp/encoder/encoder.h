#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "bsp/encoder/drv_spi.h"

/* 多圈绝对值编码器 实现(由两个单圈构成)
 * MT6835 SPI 通道宏映射 (已在drv_spi.h中配好) 
 * spindle = 主轴（SPI1,cs=P808）
 * driven  = 从轴（SPI0, cs=PA11）
 */
/* MT6835 操作命令 CMD */
#define Write           0x60FF
#define WriteEEPROM     0xC0FF
#define SetZeroPoint    0x50FF
#define ContinuousRead  0xA030

/* ---- 用户硬件参数 ------------------------------------------------------- */
#define ENCODER_SPINDLE_DIR          (1)      /* spindle 计数方向: +1 / -1   */
#define ENCODER_DRIVEN_DIR           (-1)     /* driven  计数方向: +1 / -1   */
/* 主轴和从轴对应的SPI编号 */
#define SPINDLE_SPI                  SPI_1
#define DRIVEN_SPI                   SPI_0

/* ---- 齿轮传动参数 -------------------------------------------------------
 *   z1 = GEAR_SPINDLE_TEETH (电机轴齿数, spindle MT6835 安装位置)
 *   z2 = GEAR_DRIVEN_TEETH  (从动齿轮齿数, driven  MT6835 安装位置)
 *   主轴转 n 圈 → 从齿轮转 n * z1/z2 圈
 *   唯一性范围: n ∈ [0, z2/gcd(z1,z2))
 *   z1=42, z2=44, gcd=2 → 唯一范围 22 圈, STARTUP_TURN_MAX = 21
 */
#define GEAR_SPINDLE_TEETH           (42U)  
#define GEAR_DRIVEN_TEETH            (44U)  
#define GEAR_RATIO                   0.9545455f  /* 齿轮比: z1/z2  */
#define GEAR_PHASE_OFFSET_TURN       (0.0f)      /* 齿轮安装相位偏差(圈) */

/*---------------------- 编码器配置宏 ----------------------*/
#define ENCODER_CPR                 262144U         /* 编码器分辨率（18位） */
#define ENCODER_CPR_HALF            131072          /* 编码器一半的分辨率（用于半圈法圈数更新）*/
#define ENCODER_CPR_F               262144.0f       /* 编码器分辨率（18位） */
/* 角度转换系数（预计算，18位编码器） */
#define ENCODER_DEG_PER_COUNT       0.0013733f      /* 360.0f/ ENCODER_CPR */
#define ENCODER_RAD_PER_COUNT       2.39674e-5f     /* 2PI   / ENCODER_CPR */ 

/* ---- 上电同步采样 ------------------------------------------------------- */
#define STARTUP_SYNC_SAMPLES         8U
#define STARTUP_TURN_MIN             0
#define STARTUP_TURN_MAX             21
#define STARTUP_ERR_WARN_TURN        0.015f
#define STARTUP_ACCUM_TO_FRAC        4.7683716e-7f  /* 1.0f/(STARTUP_SYNC_SAMPLES * ENCODER_CPR_F) */          

/*---------------------- API ----------------------*/
void encoder_init(void);      
void encoder_update(void);                          /* 手动更新编码器计数（高频调用）*/              
uint32_t encoder_get_count(void);                   /* 获取当前计数值（0~ENCODER_CPR-1）*/
int64_t  encoder_get_position(void);                /* 获取累计位置（计数值，可跨圈） */
float encoder_get_angle_deg(void);                  /* 获取机械角度（度）*/
float encoder_get_angle_rad(void);                  /* 获取机械角度（弧度）*/
void encoder_set_zero(void);                        /* 软归零（清零硬件计数器和圈数） */

#endif /* _ENCODER_H_ */
