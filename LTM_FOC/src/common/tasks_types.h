#ifndef TASKS_TYPES_H__
#define TASKS_TYPES_H__

#include <stdint.h>
#include "common/lt_motor_types.h"      /* 模式/状态枚举与应用层共用一套，别各定义一份 */

typedef lt_motor_mode_t  tasks_mode_t;
typedef lt_motor_state_t tasks_state_t;

typedef struct {
    int64_t       pos;          /* 多圈绝对位置（count） */
    int32_t       speed;        /* M 法测速（count/s） */
    int32_t       speed_pll;    /* 锁相环测速（count/s） */
    int32_t       vbus;         /* Q15 标幺：32767 = ADC 满量程母线 */
    int32_t       driver_temp;  /* 驱动器温度：0.1℃ */
    int32_t       motor_temp;   /* 电机温度  ：0.1℃ */
    int32_t       Ia, Ib, Ic;   /* Q15 标幺：32767 = ADC 满量程电流 */
    int32_t       Id, Iq;       /* Q15 标幺：32767 = ADC 满量程电流 */
    int32_t       target;       /* 已按 mode 编码的目标 */
    tasks_mode_t  mode;
    tasks_state_t state;
    lt_err_t      err;          /* 保护错误码（首错锁存）*/
} tasks_info_t;


#endif
