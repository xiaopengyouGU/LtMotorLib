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

#include <stdint.h>

/* ============================================================
 * 电机控制模式（用户可设置）
 * ============================================================ */
typedef enum {
    Mode_Open_Loop = 0,                     /* 开环模式 (占空比 %) */
    Mode_Position,                          /* 位置模式（角度°） */
    Mode_Speed,                             /* 速度模式（RPM）*/
    Mode_Torque,                            /* 力矩模式（q 轴电流 A，峰值）*/
    Mode_Ramp_Speed,                        /* 斜坡速度 */
    Mode_Ramp_Torque,                       /* 斜坡转矩 */
    Mode_Stop,                              /* 可控停机：按斜率减速到 0，转速够低才清占空比 */
}lt_motor_mode_t;

typedef enum {
    CMD_Zero_Find = 0,                      /* 电角度零点校准（上电自动执行） */
}lt_motor_cmd_t;

/* ============================================================
 * 电机状态（用户可读取）
 * ============================================================ */
typedef enum {
    State_Init = 0,                         /* 初始化，自动启动校准 */
    State_Idle,                             /* 空闲状态 */
    State_Enable,                           /* 电机使能 */    
    State_Running,                          /* 运行中 */
    State_Stop,                             /* 停机 */
    State_Error,                            /* 故障 */
}lt_motor_state_t;

/* ============================================================
 * API 返回码：0 = 成功，负数 = 具体错误
 * ============================================================ */
typedef enum {
    LT_OK                   =  0,       /* 成功 */
    LT_ERR_STATE            = -1,       /* 状态不允许 */
    LT_ERR_OVER_RANGE       = -2,       /* 目标超量程 */
    LT_ERR_UNSUPPORTED      = -3,       /* 不支持的请求 */
    LT_ERR_OVER_VOLT        = -4,       /* 保护：母线过压 */
    LT_ERR_UNDER_VOLT       = -5,       /* 保护：母线欠压 */
    LT_ERR_OVER_TEMP_DRIVER = -6,       /* 保护：驱动器过温 */
    LT_ERR_OVER_TEMP_MOTOR  = -7,       /* 保护：电机过温 */
    LT_ERR_OVER_CURRENT     = -8,       /* 保护：过流 */
    LT_ERR_OVER_SPEED       = -9,       /* 保护：过速 */
} lt_err_t;

/* ============================================================
 * 电机信息结构体（应用层获取）
 * ============================================================ */
typedef struct {
    float pos;                   /* 电机位置（角度°）*/
    float speed;                 /* 电机速度（RPM）*/
    float speed_pll;             /* PLL测速结果（RPM）*/
    float vbus;                  /* 母线电压（V）*/
    float motor_temp;            /* 电机温度（℃）*/
    float driver_temp;           /* 驱动器温度（℃）*/
    float te;                    /* 电机转矩（N.m）*/
    float Ia, Ib, Ic;            /* 三相电流（A）*/
    float Id, Iq;                /* D/Q轴电流（A）*/
    float target;                /* 目标值（单位取决于控制模式） */
    lt_motor_mode_t  mode;       /* 运行模式 */
    lt_motor_state_t state;      /* 电机状态 */
    lt_err_t  err;               /* 保护错误码：LT_OK = 正常，其他见 lt_err_t（首错锁存）*/
} lt_motor_info_t;

/* API 接口返回值见 lt_err_t：0 = 成功，负数 = 具体错误 */
lt_err_t lt_motor_init(void);                                  /* 电机初始化 */
lt_err_t lt_motor_enable(void);                                /* 电机使能 */
lt_err_t lt_motor_disable(void);                               /* 电机失能 */
lt_err_t lt_motor_run(void);                                   /* 电机启动 */
lt_err_t lt_motor_stop(uint8_t estop);                         /* 电机停止：0 受控停机（默认），1 急停 */
lt_err_t lt_motor_set(lt_motor_mode_t mode, float target);     /* 设置模式与目标 */
void     lt_motor_get_info(lt_motor_info_t *info);             /* 获取电机运行参数 */

#endif /* LT_MOTOR_H */
