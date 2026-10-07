#ifndef LT_MOTOR_H
#define LT_MOTOR_H

/**
 * @file    lt_motor.h
 * @brief   电机控制 API - LTM_FOC 胶水层 SDK 唯一公共头
 * @version V1.0
 *
 * ============================================================
 * 快速开始
 * ============================================================
 * 示例：
 *   lt_motor_init();                                       // 上电初始化，自动校准
 *   if (lt_motor_set(Mode_Position, 90.0f) != LT_OK) { }   // 设置模式与目标
 *   lt_motor_enable();                                     // 使能电机
 *   lt_motor_run();                                        // 启动电机
 *
 *   lt_motor_info_t info;
 *   lt_motor_get_info(&info);                              // 获取电机运行信息
 * ============================================================
 */
#include "common/lt_motor_types.h"

/* API 接口返回值见 lt_err_t：0 = 成功，负数 = 具体错误 */
lt_err_t lt_motor_init(void);                                  /* 电机初始化 */
lt_err_t lt_motor_enable(void);                                /* 电机使能 */
lt_err_t lt_motor_disable(void);                               /* 电机失能 */
lt_err_t lt_motor_run(void);                                   /* 电机启动 */
lt_err_t lt_motor_stop(uint8_t estop);                         /* 电机停止：0 受控停机（默认），1 急停 */
lt_err_t lt_motor_set(lt_motor_mode_t mode, float target);     /* 设置模式与目标 */
void     lt_motor_get_info(lt_motor_info_t *info);             /* 获取电机运行参数 */

#endif /* LT_MOTOR_H */
