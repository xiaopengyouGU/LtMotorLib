#ifndef LT_MOTOR_API_H
#define LT_MOTOR_API_H

/**
 * @file    lt_motor_api.h
 * @brief   电机控制 API - 用户唯一需要包含的头文件
 * @version V1.0
 * @date    2026-07-12
 * 
 * ============================================================
 * 快速开始
 * ============================================================
 * 示例：
 *   lt_motor_init();                     // 上电初始化，自动校准
 *   lt_motor_set(Mode_Position, 90.0f);  // 设置运行模式和目标值, 旋转到 90° 
 *   lt_motor_enable();                   // 使能电机
 *   lt_motor_run();                      // 启动电机
 * 
 *   lt_motor_info info;                    
 *   lt_motor_get_info(&info);            // 获取电机运行信息
 *   printf("位置: %.2f°, 速度: %.2f RPM\n", info.pos, info.speed);
 * ============================================================
 */

#include "motor_ctrl/common/lt_motor_types.h"

// /* ============================================================
//  * 电机控制模式（用户可设置）
//  * ============================================================ */
// typedef enum{
//     Mode_Open_Loop = 0,                     /* 开环模式 (占空比 %) */
//     Mode_Position,                          /* 位置模式（角度°） */
//     Mode_Speed,                             /* 速度模式（RPM）*/
//     Mode_Torque,                            /* 力矩模式 (额定转矩 %) */
//     Mode_Ramp_Speed,                        /* 斜坡速度 */
//     Mode_Ramp_Torque,                       /* 斜坡转矩 */
// }lt_motor_mode_t;

// /* ============================================================
//  * 电机状态（用户可读取）
//  * ============================================================ */
// typedef enum{
//     State_Init = 0,                         /* 初始化，自动启动校准 */
//     State_IDLE,                             /* 空闲状态 */
//     State_Enable,                           /* 电机使能 */    
//     State_Running,                          /* 运行中 */
//     State_Stop,                             /* 停机 */
//     State_Error,                            /* 故障 */
// }lt_motor_state_t;

// /* ============================================================
//  * 电机信息结构体
//  * ============================================================ */
// typedef struct {
//     float pos;                   /* 电机位置（角度°）*/
//     float speed;                 /* 电机速度（RPM）*/
//     float vbus;                  /* 母线电压（V）*/
//     float motor_temp;            /* 电机温度（℃）*/
//     float driver_temp;           /* 驱动器温度（℃）*/
//     float te;                    /* 电机转矩（N.m）*/
//     float Ia, Ib, Ic;            /* 三相电流（A）*/
//     float Id, Iq;                /* D/Q轴电流（A）*/
//     float target;                /* 目标值（单位取决于控制模式） */
//     lt_motor_mode_t  mode;          /* 运行模式 */
//     lt_motor_state_t state;         /* 电机状态 */
// } lt_motor_info_t;

/* API 接口返回值：1：成功， 0：失败或运行中，-1：状态切换不允许 */
void lt_motor_init(void);                       /* 电机初始化 */
int  lt_motor_enable(void);                     /* 电机使能 */
int  lt_motor_disable(void);                    /* 电机失能 */
int  lt_motor_run(void);                        /* 电机启动 */
int  lt_motor_stop(void);                       /* 电机停止 */
// int  lt_motor_identify(lt_ident_mode_t mode);   /* 电机辨识 */
int  lt_motor_set(lt_motor_mode_t mode, float target);    /* 设置电机模式 */
void lt_motor_get_info(lt_motor_info_t *info);            /* 获取电机运行参数 */
void lt_motor_show_info(lt_motor_info_t *info);           /* 上位机显示电机运行参数 */

#endif