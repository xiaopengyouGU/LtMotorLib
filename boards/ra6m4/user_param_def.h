#ifndef USER_PARAM_DEF_H
#define USER_PARAM_DEF_H

/* 用户修改该头文件即可完成移植 */

#include "ltm_ctrl/lt_math.h"
#include "ltm_hal/ltm_hal.h"
/* ============ 电机参数 ============ */                                         
#define MOTOR_CPR                           LTM_ENC_CPR /* 编码器分辨率（单圈）*/
#define MOTOR_PP                            5           /* 极对数 */
#define MOTOR_RS                            0.235f      /* 定子电阻（Ω）*/
#define MOTOR_LS                            270e-6f     /* 相电感（H），SPM 下 Lq≈Ls */
#define MOTOR_PSI_F                         0.00483f    /* 永磁体磁链（Wb）：Ke 3.1V/krpm(线电压有效值)、pp=5 反推 */
#define MOTOR_KT                            (1.5f * MOTOR_PP * MOTOR_PSI_F)  /* 转矩常数（N·m/A，峰值口径）≈0.0363 */
#define MOTOR_J                             5.884e-6f   /* 转动惯量（kg·m²）= 0.06 gf·cm·s² */

/* ============ 三环控制任务参数 ============ */   
/* 若修改三环频率，需同步改动 control_tasks.c 中的 control_tasks_run 函数 */
#define CURRENT_LOOP_HZ                     20000U      /* 电流环频率（Hz）：默认20kHz */
#define SPEED_LOOP_HZ                       4000U       /* 速度环频率（Hz）：默认4kHz */
#define POS_LOOP_HZ                         1000U       /* 位置环频率（Hz）：默认1kHz */
#define SPEED_DIV                           5U          /* 速度环 20k/5  = 4kHz */
#define PLL_DIV                             1U          /* 测速分频：PLL 更新率 = 电流环 / PLL_DIV */
#define POS_DIV                             20U         /* 位置环 20k/20 = 1kHz */
#define V_BUS_NOM_V                         24.0f       /* 整定用的设计母线 */
/* 三环带宽 */
#define CURRENT_LOOP_BW                     800.0f      /* 电流环带宽（Hz）*/
#define SPEED_LOOP_BW                       60.0f       /* 速度环带宽（Hz）*/
#define POS_LOOP_BW                         8.0f        /* 位置环带宽（Hz）*/
#define CUR_FB_IIR_SHIFT                   1           /* 电流反馈一阶 IIR：y+=(x-y)>>N，只进 PI/前馈；N=1 时 20kHz 下 fc≈2.3kHz，800Hz 穿越处相移 -14° */
#define SPEED_REP_IIR_SHIFT                5           /* 上报速度一阶 IIR：y+=(x-y)>>N，只滤上报曲线，不进环路 */
#define SPEED_IQ_LIMIT                      7.6f        /* 速度环输出限幅 ±7.6A，避免触发过流保护 */
#define ANGLE_DELAY_STEP                    0           /* 是否有角度测量一拍延迟，1：有，0：无 */
#define DQ_DECOUPLE                         1           /* dq 解耦前馈：1=开 0=关（高速失稳排查用）*/

/* ============ 死区补偿（电流极性法，Q15 标幺）============ */
#define DZ_ENABLE                           0       /* 1=开 0=关 */
#define DZ_DEAD_DUTY_Q15                    655     /* 死区 1us/50us = 2% × 32768 */
#define DZ_ITH_Q15                          64      /* 极性判别阈值：64 Q15 count = 27mA，约 4 个 ADC LSB（1 LSB=6.71mA）；须明显大于采样噪声，又不能接近额定（取到额定 1~3% 反而更差） */
#define DZ_ALPHA_SHIFT                      1       /* 差模补偿一阶低通 α = 1/2 */
#define DZ_DIR_A                            -1       /* 电流方向与 PWM 正占空比：1 同向 / -1 反向 */
#define DZ_DIR_B                            -1
#define DZ_DIR_C                            -1
/* 锁相环：PLL测速 在电流环里跑，f_n = 300Hz、ζ=0.7（取速度环 60Hz 的 5 倍）
 * Kp = 4πζ·f_n、Ki = (2π·f_n)²/freq */
#define PLL_KP                              2639
#define PLL_KI                              ((uint32_t)(39.478f * 300.0f * 300.0f / CURRENT_LOOP_HZ + 0.5f))
/* ============ 低速测速（T 法）============ */
#define SPEED_T_LOOP_BW                     60.0f       /* T 法反馈时的速度环带宽（Hz）：T 窗口 1.5ms，延迟与 PLL 同量级，带宽不用压 */
#define SPD_T_ON_RPM                        40.0f       /* 低于此转速用 T 法反馈 */
#define SPD_T_OFF_RPM                       60.0f       /* 高于此转速切回 PLL（滞环）*/
#define SPD_T_BLIND_RPM                     10.0f       /* T 法输出 0 但 PLL 超过此转速：判 T 法失效，退回 PLL */

/* ============ 允许的 target 范围（应用单位）============ */
#define TARGET_TORQUE_MAX_A                 2.0f            /* 转矩模式：±2.0A（峰值）*/
#define TARGET_SPEED_MAX_RPM                3000.0f         /* 速度模式：±3000RPM */
#define TARGET_POS_MAX_DEG                  (100 * 360.0f)  /* 位置模式：±100 圈（最大只允许±120圈）*/
/* ============ 转子预定位参数 ============ */
#define ZERO_FIND_DUTY                     0.10f       /* 预定位 D 轴强拖幅值（pu）：1.0 = 母线/sqrt3。0.05 时命令电压 0.69V 与死区误差 0.48V 同量级，实际电流不定 -> 对齐点乱跑 */
#define ZERO_SWEEP_TICKS_PER_ELEC           1000U       /* 每电周期扫多少步：1ms 一步 */
#define ZERO_SWEEP_ELEC_TURNS               2U          /* 单方向扫过的电周期数（正反各扫一遍）*/
#define ZERO_SETTLE_MS                      300         /* D 轴吸住的稳定时间（ms）：停稳后再取零点 */
#define ZERO_AVG_N                          8           /* 末段取平均的采样点数（1ms 一点）*/
/* ============ 摩擦力前馈补偿参数 ============ */
#define FRICTION_FF_ENABLE                  1           /* 速度环库仑摩擦前馈 */
#define FRICTION_STATIC_CURR                0.18f       /* 静摩擦突破电流（A）*/
#define FRICTION_VISCOUS                    0.00003f    /* 粘性摩擦系数（A/RPM）*/
#define FRICTION_FF_DEADBAND                1.0f       /* 前馈死区（RPM）：低于此速不给前馈，防零速抖振 */

/* ============ 电机保护参数 ============ */
#define OC_CURRENT                          11.0f       /* 过流保护触发：±11A */
#define VBUS_OV                             33.0f       /* 母线过压：33V */
#define VBUS_UV                             10.0f       /* 母线欠压：10V */
#define DRIVER_OT_01C                       900         /* 驱动器过温 90.0℃（单位 0.1℃）*/
#define MOTOR_OT_01C                        750         /* 电机过温 75.0℃ */
#define OVER_SPEED                          3900        /* 电机过速，±3900RPM */
#define PROTECT_DEBOUNCE                    50          /* 保护去抖：同一故障连续 50 次扫描(50ms)才跳 */

/* ============ 电机运行参数 ============ */
#define POSITION_ACCEL_RATE                 600.0f      /* 梯形加减速斜率（RPM/s）*/
#define POSITION_MAX_SPEED                  1200.0f     /* 位置模式速度上限（RPM）*/
#define POS_WIN_CNT                         15          /* 位置模式到位窗口：15 count */
#define SPEED_RAMP_RATE                     900.0f      /* 斜坡速度：目标按此斜率逼近（RPM/s）*/
#define TORQUE_RAMP_RATE                    1.0f       /* 斜坡转矩：目标按此斜率逼近（A/s）*/
/* 可控停机：速度给定按此斜率降到 0，实测速度进窗口后才清占空比。*/
#define STOP_DONE                           3.0f        /* 速度停机窗口：±3RPM */
#define STOP_RAMP_RATE                      1500.0f     /* 受控停机: 1500RPM/s */
#define ESTOP_RAMP_RATE                     6000.0f     /* 急停: 6000RPM/s*/

/* ============ 校准参数 ============ */
#define R_POINTS            4           /* 多点差分法电流档位数 */
#define R_SAMPLE_TICKS      4000        /* 每档采样 tick */
#define R_CALIB_CURRENT     2.0f        /* 校准最大电流 A */
#define R_CALIB_KI          2.0f        /* 简单电流校准闭环的KI */

/* ============ L 校准参数（高频注入法） ============ */
#define L_HFI_VOLTAGE       3.0f        /* 高频注入电压幅值 (V)，额定电压10%以上 */
#define L_HFI_VOLTAGE_INV   0.3333333f  /* 1.0f / L_HFI_VOLTAGE，预计算常数 */
#define L_HFI_FREQ          500.0f      /* 高频注入频率 (Hz) */
#define L_HFI_OMEGA         (_2_PI * L_HFI_FREQ)  /* 角频率 (rad/s) */
#define L_HFI_CYCLES        100         /* 解调采集周期数 */
#define L_HFI_SAMPLES_PER_CYCLE (uint16_t)(CURRENT_LOOP_HZ / L_HFI_FREQ)
#define L_HFI_TOTAL_SAMPLES (L_HFI_CYCLES * L_HFI_SAMPLES_PER_CYCLE)

/* ============ PP + Encoder 校准参数（合并） ============ */
#define PP_ELEC_OMEGA       _PI         /* 强拖电角速度 rad/s（π rad/s ≈ 0.5 转/秒电角度）*/          
#define PP_ELEC_TURNS       4.0f        /* 旋转电角度圈数 */
#define PP_VOLTAGE          3.0f        /* D轴强拖电压 V */
/* 一次旋转同时完成 PP 和 Encoder，总采样点数由旋转圈数决定 */
#define PP_ENC_TOTAL_SAMPLES  ((uint16_t)(PP_ELEC_TURNS * _2_PI * POS_LOOP_HZ / PP_ELEC_OMEGA))

#endif
