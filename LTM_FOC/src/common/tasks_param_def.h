#ifndef TASKS_PARAM_DEF_H
#define TASKS_PARAM_DEF_H

/* 板级/机型参数：构建时用 -DLTM_FOC_PARAM_H=<路径> 指定
 * 走 CMake 时等价于 -DLTM_FOC_BOARD=<board>，即 boards/<board>/user_param_def.h */
#ifndef LTM_FOC_PARAM_H
#error "LTM_FOC: 未指定参数文件，请用 -DLTM_FOC_PARAM_H=<boards/<board>/user_param_def.h> 或 CMake 变量 LTM_FOC_BOARD"
#endif
#include LTM_FOC_PARAM_H

#define DEG_TO_RAD                         (_PI / 180.0f)
#define Q15_PU                             32767       /* Q15 标幺值 1.0 pu */
#define Q24_PU                             0xFFFFFF    /* Q24 标幺值 1.0 pu */ 
#define Q30_PU                             0x40000000  /* Q30 标幺值 1.0 pu */

/* count ↔ Q24 圈值：
 *   c·2^24/CPR ≈ (c·K) >> 12,  K = round(2^12·2^24/CPR)
 *   q·CPR/2^24 = (q·(CPR<<6)) >> 30 */
#define CNT2Q24_K       ((int32_t)(((1LL << 36) + MOTOR_CPR / 2) / MOTOR_CPR))         /* = round(2^36/CPR) */
#define CNT_TO_Q24(c)   ((int32_t)(((int64_t)(c) * CNT2Q24_K) >> 12))
#define INV_CURRENT_LOOP_HZ_Q24     ((Q24_PU + CURRENT_LOOP_HZ / 2) / CURRENT_LOOP_HZ)   /* 2^24 / f，四舍五入 */

/* 编码器单圈计数值（0~CPR-1）转换为 电角度，自动回绕 */
#define COUNT_TO_THE(pos_count, zero_offset) \
    ((uint32_t)(((int32_t)(pos_count) - (int32_t)(zero_offset)) * MOTOR_PP))  
/* 一拍电角度增量（count）：机械速度(count/s) × 极对数 × 每拍时间，输出 1拍 电角度 */
#define SPEED_TO_THE(speed_pll) \
    ((int32_t)(((int64_t) (speed_pll) * MOTOR_PP * INV_CURRENT_LOOP_HZ_Q24) >> 24))

/* ============ 三环定点化参数（LTM_CTRL 定点库）============ */
#define I_BASE_A                            (float)LTM_ADC_FULL_SCALE_CURRENT_A   /* 电流标幺基准（A） */
#define VBUS_BASE_V                         (float)LTM_ADC_FULL_SCALE_VOLTAGE_V   /* 母线电压标幺基准（V）*/
#define V_B_NOM_V                           (V_BUS_NOM_V / _SQRT_3)               /* 母线/√3 = 1.0pu，13.86V */
#define ZERO_FIND_DUTY_Q15                  (int32_t)((ZERO_FIND_DUTY * Q15_PU))  /* 预定位占空比 */
/* ---- 三环整定（标幺域）----
 * 对象：电流环 1/(Ls·s+Rs) 对消成 ωc/s；速度环 iq_pu→speed_pu 是纯积分，增益
 *       K = Kt·I_BASE/(J·2π)；位置环是 1/s（速度环快 10 倍，按理想内环算）。*/
#define CURRENT_LOOP_WC                     (_2_PI * CURRENT_LOOP_BW)

/* 电流环：连续域带宽法整定，Kp = L_pu·ωc、Ki = R_pu·ωc（L_pu = Ls·I_BASE/V_B，同理 R_pu）*/
#define CURR_RPU                            ((double)MOTOR_RS * I_BASE_A / V_B_NOM_V)
#define CURR_LPU                            ((double)MOTOR_LS * I_BASE_A / V_B_NOM_V)
#define CURR_KP_Q15                         ((int32_t)(CURR_LPU * CURRENT_LOOP_WC * Q15_PU))
#define CURR_KI_Q15                         ((int32_t)(CURR_RPU * CURRENT_LOOP_WC * Q15_PU))
/* 速度环：ωc = 2π·bw，ζ=0.707 → ωn = ωc/1.5538（ζ=0.707 时 ωc = 1.5538·ωn），
 * Kp = 2ζ·ωn/K、Ki = ωn²/K，都是 pu。 */
#define SPEED_LOOP_WC                       (_2_PI * SPEED_LOOP_BW)
#define SPEED_PLANT_K                       ((double)MOTOR_KT * I_BASE_A / ((double)MOTOR_J * _2_PI))
#define SPEED_LOOP_WN                       (SPEED_LOOP_WC / 2.93790)
#define SPEED_KP_Q15                        ((int32_t)(3.0f * SPEED_LOOP_WN / SPEED_PLANT_K * Q15_PU))
#define SPEED_KI_Q15                        ((int32_t)(SPEED_LOOP_WN * SPEED_LOOP_WN / SPEED_PLANT_K * Q15_PU))
/* 位置环PD：ωc = 2π·8，微分零点放 2×ωc（约 27° 超前），
 * Kp = ωc/√(1+(ωc/ωz)²)、Kd = Kp/ωz。D 项吃编码器量化：1 count 抖动在 1kHz
 * 位置环下折合 Kd×0.1pu ≈ 2.7RPM 的速度给定毛刺，别再往上加了 */
#define POS_LOOP_WC                         (_2_PI * POS_LOOP_BW)
#define POS_LOOP_WZ                         (_2_PI * POS_LOOP_WC)
#define POS_KP_Q15                          ((int32_t)(POS_LOOP_WC / 1.118034 * Q15_PU))
#define POS_KD_Q15                          ((int32_t)(POS_LOOP_WC / 1.118034 / POS_LOOP_WZ * Q15_PU))
/* 位置模式：规划 Unit 与位置环同为 Q24 圈值，限幅/加减速沿用 RPM 口径折算 */
#define POS_ACC_MS                          ((uint16_t)(POSITION_MAX_SPEED / POSITION_ACCEL_RATE * 1000.0f))
#define POS_WIN_Q24                         ((int32_t)((int64_t)POS_WIN_CNT * Q24_PU / MOTOR_CPR))
/* 位置模式目标限幅：Q24 圈值（1.0 = 一圈）名义上限 ±128 圈，
 * 应用层收到 ±120 圈留余量。越过就在入口拦截——速度模式跑久了再切回位置模式时，
 * 绝对位置可能已经超出去 */
/* Q24 只覆盖 ±128 圈：多圈 count 送进 CNT_TO_Q24 之前先夹到该范围，否则 int32 溢出（UB）*/
#define POS_CNT_MAX                         ((int64_t)(TARGET_POS_MAX_DEG / 360.0f) * MOTOR_CPR)

/* ============ 电机保护参数 ============ */
#define OC_CURRENT_Q15                      ((int32_t)(OC_CURRENT / I_BASE_A * Q15_PU))   
/* 其余保护阈值。任一触发 → 记错误码 + lt_fsm_update(Event_Fault) */
#define VBUS_OV_Q15                         ((int32_t)(VBUS_OV / VBUS_BASE_V * Q15_PU))/* 母线过压 */
#define VBUS_UV_Q15                         ((int32_t)(VBUS_UV / VBUS_BASE_V * Q15_PU))/* 母线欠压 */
#define OVER_SPEED_Q24                      ((int32_t)(OVER_SPEED / 60.0f * Q24_PU))   /* 过速保护 */
#define STOP_RAMP_STEP_Q24                  ((int32_t)(STOP_RAMP_RATE / 60.0f * Q24_PU / SPEED_LOOP_HZ))
#define ESTOP_RAMP_STEP_Q24                 ((int32_t)(ESTOP_RAMP_RATE / 60.0f * Q24_PU / SPEED_LOOP_HZ))
#define SPEED_IQ_LIMIT_Q24                  ((int32_t)(SPEED_IQ_LIMIT / I_BASE_A * Q24_PU))    /* 速度环输出限幅 */
#define STOP_DONE_Q24                       ((int32_t)(STOP_DONE / 60.0f * Q24_PU))    /* 速度停机窗口 */
/* 摩擦前馈（全 pu）：速度给 Q24（1.0 = 1 圈/s），输出直接叠在 iq 的 Q15 上 */
#define FRICTION_FF_DEADBAND_Q24            ((int32_t)(FRICTION_FF_DEADBAND / 60.0f * Q24_PU))
#define FRICTION_STATIC_Q15                 ((int32_t)(FRICTION_STATIC_CURR / I_BASE_A * Q15_PU))
#define FRICTION_VISCOUS_Q15                ((int32_t)(FRICTION_VISCOUS / I_BASE_A * 60.0f * Q15_PU))
/* 位置环速度限幅（Q24 圈/s）*/
#define POS_VMAX_Q24                        ((int32_t)(POSITION_MAX_SPEED / 60.0f * Q24_PU))
/* 斜坡速度：目标按此斜率逼近（RPM/s），换成每拍 Q24 增量 */
#define SPEED_RAMP_STEP_Q24                 ((int32_t)(SPEED_RAMP_RATE / 60.0f * Q24_PU / SPEED_LOOP_HZ))
/* DQ 解耦前馈系数（Q24）：乘 we(rad/s) 再 >>24 得 Q15 电压；
 * 电流相关项还要乘 Q15 电流（再 >>15），故系数先 ×32768 */
#define FF_L_Q24                            ((int32_t)((double)MOTOR_LS * I_BASE_A / V_B_NOM_V * Q24_PU * Q15_PU))
#define FF_PSI_Q24                          ((int32_t)((double)MOTOR_PSI_F / V_B_NOM_V * Q24_PU * Q15_PU))
/* 电角速度系数（Q30）：we[rad/s] = speed[count/s]  */
#define WE_K_Q30                            ((int32_t)(_2_PI * MOTOR_PP / MOTOR_CPR * Q30_PU))

/* ============ 单位换算宏 ============ */
/* 出口：控制域 → 应用单位 */
#define CNT_TO_DEG(x)       ((float)(x) * (360.0f / (float)MOTOR_CPR))          /* count → ° */
#define CPS_TO_RPM(x)       ((float)(x) * (60.0f  / (float)MOTOR_CPR))          /* count/s → RPM */
#define Q24_TO_DEG(x)       ((float)(x) * (360.0f / (float)Q24_PU))             /* 位置 Q24 → ° */
#define Q24_TO_RPM(x)       ((float)(x) * (60.0f  / (float)Q24_PU))             /* 速度 Q24 → RPM */
#define Q15_TO_A(x)         ((float)(x) * (I_BASE_A / (float)Q15_PU))           /* 电流 Q15 → A（峰值）*/
#define A_TO_IQ15(x)        ((int32_t)((x) / I_BASE_A * (float)Q15_PU))         /* A（峰值）→ 电流 Q15 */
#define Q15_TO_V(x)         ((float)(x) * (LTM_ADC_FULL_SCALE_VOLTAGE_V / (float)Q15_PU))/* 母线 Q15 → V */
#define Q15_TO_PCT(x)       ((float)(x) * (100.0f / (float)Q15_PU))              /* Q15 → % */
#define TENTH_TO_C(x)       ((float)(x) * 0.1f)                                  /* 0.1℃ → ℃ */
/* 入口：应用单位 → 控制域 */
#define PCT_TO_Q15(x)       ((int32_t)((x) * ((float)Q15_PU / 100.0f)))          /* % → Vq Q15 */
#define RPM_TO_Q24(x)       ((int32_t)((x) * ((float)Q24_PU / 60.0f)))           /* RPM → 速度 Q24 */
#define DEG_TO_Q24(x)       ((int32_t)((x) * ((float)Q24_PU / 360.0f)))          /* ° → 位置 Q24 */


/* ============ 允许的 target 范围（控制域 Q 格式）============ */
#define TARGET_TORQUE_MAX_Q15               A_TO_IQ15(TARGET_TORQUE_MAX_A)      /* 转矩：对应 iq Q15 */
#define TARGET_SPEED_MAX_Q24                RPM_TO_Q24(TARGET_SPEED_MAX_RPM)    /* 速度：Q24 圈/s */
#define TARGET_POS_MAX_Q24                  DEG_TO_Q24(TARGET_POS_MAX_DEG)      /* 位置：Q24 圈值 */

#endif
