#ifndef LT_MOTOR_TYPES_H
#define LT_MOTOR_TYPES_H

/* ============================================================
 * 电机控制模式（用户可设置）
 * ============================================================ */
typedef enum{
    Mode_Open_Loop = 0,                     /* 开环模式 (占空比 %) */
    Mode_Position,                          /* 位置模式（角度°） */
    Mode_Speed,                             /* 速度模式（RPM）*/
    Mode_Torque,                            /* 力矩模式 (额定转矩 %) */
    Mode_Ramp_Speed,                        /* 斜坡速度 */
    Mode_Ramp_Torque,                       /* 斜坡转矩 */
}lt_motor_mode_t;

/* ============================================================
 * 电机状态（用户可读取）
 * ============================================================ */
typedef enum{
    State_Init = 0,                         /* 初始化，自动启动校准 */
    State_Idle,                             /* 空闲状态 */
    State_Enable,                           /* 电机使能 */    
    State_Running,                          /* 运行中 */
    State_Stop,                             /* 停机 */
    State_Error,                            /* 故障 */
}lt_motor_state_t;

/* ============================================================
 * 电机信息结构体
 * ============================================================ */
typedef struct {
    float pos;                   /* 电机位置（角度°）*/
    float speed;                 /* 电机速度（RPM）*/
    float vbus;                  /* 母线电压（V）*/
    float motor_temp;            /* 电机温度（℃）*/
    float driver_temp;           /* 驱动器温度（℃）*/
    float te;                    /* 电机转矩（N.m）*/
    float Ia, Ib, Ic;            /* 三相电流（A）*/
    float Id, Iq;                /* D/Q轴电流（A）*/
    float target;                /* 目标值（单位取决于控制模式） */
    lt_motor_mode_t  mode;       /* 运行模式 */
    lt_motor_state_t state;      /* 电机状态 */
} lt_motor_info_t;

#endif