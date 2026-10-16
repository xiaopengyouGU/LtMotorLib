#ifndef USER_PARAM_DEF_H
#define USER_PARAM_DEF_H

/* 用户修改该头文件即可完成移植 */

#include "ltm_ctrl/lt_math.h"
#include "ltm_hal/ltm_hal.h"

/* ============ 电机参数 ============ */
#define MOTOR_CPR                           LTM_ENC_CPR /* 21bit 单圈分辨率：直接取 HAL 的 LTM_ENC_CPR（MT6835 不截断）*/
#define MOTOR_PP                            10          /* 极对数 */
#define MOTOR_RS                            0.5605f     /* 相电阻（Ω）*/
#define MOTOR_LS                            327e-6f     /* 相电感（H），SPM 下 Lq≈Ls */
#define MOTOR_PSI_F                         0.01188f    /* 永磁体磁链（Wb）：由 Kt 0.1782 N·m/A、pp=10 反推 ψf = Kt/(1.5·pp) */
#define MOTOR_KT                            (1.5f * MOTOR_PP * MOTOR_PSI_F)   /* 转矩常数（N·m/A，峰值口径）≈0.1782 */
#define MOTOR_J                             1.976e-5f   /* 转动惯量（kg·m²）：由 inertia 0.000697 A/(turn/s²) 换算 J = inertia·Kt/2π */

/* ============ 三环控制任务参数 ============ */
/* 若修改三环频率，需同步改动 control_tasks.c 中的 control_tasks_run 函数 */
#define CURRENT_LOOP_HZ                     20000U      /* 电流环频率（Hz）：与 PWM 载波同频（HRTIM 20kHz）*/
#define SPEED_LOOP_HZ                       4000U       /* 速度环频率（Hz）：20k/5 */
#define POS_LOOP_HZ                         1000U       /* 位置环频率（Hz）：20k/20 */
#define SPEED_DIV                           5U          /* 速度环 20k/5  = 4kHz */
#define POS_DIV                             20U         /* 位置环 20k/20 = 1kHz（须为 SPEED_DIV 的整数倍）*/
#define V_BUS_NOM_V                         48.0f       /* 整定用的设计母线 */

/* 三环带宽。
 * 注意速度环/位置环只隔 3.2 倍（80/25），RA6M4 台架最终整定的是 60/8（7.5 倍）：
 * 位置模式若出现超调或低频抖动，先把 POS_LOOP_BW 降到 10~15Hz 试。 */
#define CURRENT_LOOP_BW                     (7500.0f / _2_PI)  /* 电流环带宽：沿用台架 current_ctrl_bw = 7500 rad/s ≈ 1194Hz（20kHz 采样下 fs/BW ≈ 17）*/
#define SPEED_LOOP_BW                       80.0f       /* 速度环带宽（Hz）*/
#define POS_LOOP_BW                         25.0f       /* 位置环带宽（Hz）*/
#define CUR_FB_IIR_SHIFT                    1           /* 电流反馈一阶 IIR：y+=(x-y)>>N，只进 PI/前馈；N=1 时 20kHz 下 fc≈2.3kHz */
#define SPEED_REP_IIR_SHIFT                 5           /* 上报速度一阶 IIR：只滤上报曲线，不进环路；N=5 时 4kHz 下 fc≈20Hz */
#define SPEED_IQ_LIMIT                      25.0f       /* 速度环输出限幅 ±25A，留在过流门限 28A 之下 */
#define ANGLE_DELAY_STEP                    0           /* 是否有角度测量一拍延迟，1：有，0：无 */
#define DQ_DECOUPLE                         1           /* dq 解耦前馈：1=开 0=关（高速失稳排查用）*/

/* ============ 死区补偿（电流极性法，Q15 标幺）============ */
#define DZ_ENABLE                           0       /* 1=开 0=关（过零点会叫，标定完再开）*/
#define DZ_DEAD_DUTY_Q15                    524     /* 死区 0.8µs / 50µs = 1.6% × 32768（HRTIM 68tick×2 @170MHz）*/
#define DZ_ITH_Q15                          286     /* 极性判别阈值：286 Q15 = 0.287A（33A 量程的 0.87%）。
                                                        阈值内补偿量与电流线性（TI 那套 linear region）：
                                                        带宽→零电流处补偿被削弱、极性不翻不叫 */
#define DZ_ALPHA_SHIFT                      1       /* 差模补偿一阶低通 α = 1/2 */
#define DZ_DIR_A                            -1      /* 电流方向与 PWM 正占空比：1 同向 / -1 反向（首次上电需实测确认）*/
#define DZ_DIR_B                            -1
#define DZ_DIR_C                            -1

/* 锁相环：PLL测速 在电流环里跑，f_n = 400Hz、ζ=0.7（取速度环 80Hz 的 5 倍）
 * Kp = 4πζ·f_n、Ki = (2π·f_n)²/freq */
#define PLL_KP                              3519
#define PLL_KI                              ((uint32_t)(39.478f * 400.0f * 400.0f / CURRENT_LOOP_HZ + 0.5f))

/* ============ 允许的 target 范围（应用单位）============ */
#define TARGET_TORQUE_MAX_A                 26.0f           /* 转矩模式：±26A（峰值），留在过流门限 28A 之下 */
#define TARGET_SPEED_MAX_RPM                2200.0f         /* 速度模式：±2200RPM。48V 母线下反电势极限 ≈2220RPM（ψf·pp·ω 顶到 Vdc/√3）*/
#define TARGET_POS_MAX_DEG                  (100 * 360.0f)  /* 位置模式：±100 圈（最大只允许±120圈）*/

/* ============ 转子预定位参数 ============ */
#define ZERO_FIND_DUTY                      0.05f       /* 预定位 D 轴强拖幅值（pu）：1.0 = 母线/√3。
                                                       本机 0.05 pu ≈ 1.39V，按 0.5605Ω 折算约 2.5A；
                                                       死区造成的电压误差约 1.6% ≈ 0.44V，留了 3 倍余量。
                                                       太大上电顶驱动器硬件过保，需按本机实测确定 */
#define ZERO_RAMP_UP_MS                     500         /* 预定位软启动：D 轴电压 0→设定值用时（ms）。直接给满会把功率级顶跳 */
#define ZERO_RAMP_DOWN_MS                   200         /* 预定位收尾软关断：D 轴电压→0 用时（ms）*/
#define ZERO_SWEEP_TICKS_PER_ELEC           1000U       /* 每电周期扫多少步：1ms 一步 */
#define ZERO_SWEEP_ELEC_TURNS               2U          /* 单方向扫过的电周期数（正反各扫一遍）*/
#define ZERO_SETTLE_MS                      300         /* D 轴吸住的稳定时间（ms）：停稳后再取零点 */
#define ZERO_AVG_N                          8           /* 末段取平均的采样点数（1ms 一点）*/

/* ============ 摩擦力前馈补偿参数 ============ */
#define FRICTION_FF_ENABLE                  0           /* 库仑+粘性摩擦前馈：静摩擦实测标定后再开 */
#define FRICTION_STATIC_CURR                0.0f        /* 静摩擦突破电流（A），实测填入 */
#define FRICTION_VISCOUS                    0.0f        /* 粘性摩擦系数（A/RPM），实测填入 */
#define FRICTION_FF_DEADBAND                10.0f       /* 前馈死区（RPM）：低于此速不给前馈，防零速抖振 */

/* ============ 齿槽补偿 ============ */
#define COGGING_COMP_ENABLE                 0           /* 1=开 0=关：需先完成齿槽辨识并把表写进 flash */

/* ============ 电机保护参数 ============ */
#define OC_CURRENT                          28.0f       /* 过流保护触发：±28A。
                                                       Q15 满刻度 = I_BASE 33.0A（ADC 半量程），门限必须在其之下：
                                                       写 33.0 正好压在 ADC 轨上、正方向永远触发不了。
                                                       28A 对应 1738 个原始 LSB，要求零点偏置落在 [1738, 2357]
                                                       （中位 2048，窗口 ±310 LSB ≈ ±0.25V，留给温漂与偏置误差）*/
#define VBUS_OV                             85.0f       /* 母线过压：85V（ADC 满量程 98.79V）*/
#define VBUS_UV                             16.0f       /* 母线欠压：16V */
#define DRIVER_OT_01C                       900         /* 驱动器过温 90.0℃（单位 0.1℃）*/
#define MOTOR_OT_01C                        1000        /* 电机过温 100.0℃（按绝缘等级调整）*/
#define OVER_SPEED                          2600        /* 电机过速，±2600RPM（比速度模式上限 2200 高 18%）*/
#define PROTECT_DEBOUNCE                    50          /* 保护去抖：同一故障连续 50 次扫描(50ms)才跳 */

/* ============ 电机运行参数 ============ */
#define POSITION_ACCEL_RATE                 600.0f      /* 梯形加减速斜率（RPM/s）*/
#define POSITION_MAX_SPEED                  960.0f      /* 位置模式速度上限（RPM）*/
#define POS_WIN_CNT                         (MOTOR_CPR / 1000)  /* 位置到位窗口：千分之一圈 = 0.36°（21bit 下 ≈2097 count）*/
#define POS_SCURVE_TYPE                     0           /* 位置规划器加减速：0 = T 形，1 = S 形（加速度按 jerk 渐变）*/
#define POS_JERK_MS                         300         /* S 形 jerk 段时长（ms）：加速度 0→满 用多久；相对 2000ms 斜坡取 10~25% 才看得出 S */
#define SPEED_RAMP_RATE                     300.0f      /* 斜坡速度：目标按此斜率逼近（RPM/s）*/
#define TORQUE_RAMP_RATE                    1.0f        /* 斜坡转矩：目标按此斜率逼近（A/s）*/
/* 可控停机：速度给定按此斜率降到 0，实测速度进窗口后才清占空比。*/
#define STOP_DONE                           3.0f        /* 速度停机窗口：±3RPM */
#define STOP_RAMP_RATE                      1500.0f     /* 受控停机: 1500RPM/s */
#define ESTOP_RAMP_RATE                     6000.0f     /* 急停: 6000RPM/s */

/* ============ 校准参数 ============ */
#define R_POINTS            4           /* 多点差分法电流档位数 */
#define R_SAMPLE_TICKS      5000        /* 每档采样 tick */
#define R_CALIB_CURRENT     5.0f        /* 校准最大电流 A */
#define R_CALIB_KI          2.0f        /* 简单电流校准闭环的KI */

/* ============ L 校准参数（高频注入法）============ */
#define L_HFI_VOLTAGE       5.0f        /* 高频注入电压幅值 (V) */
#define L_HFI_VOLTAGE_INV   0.2f        /* 1.0f / L_HFI_VOLTAGE，预计算常数 */
#define L_HFI_FREQ          500.0f      /* 高频注入频率 (Hz) */
#define L_HFI_OMEGA         (_2_PI * L_HFI_FREQ)  /* 角频率 (rad/s) */
#define L_HFI_CYCLES        100         /* 解调采集周期数 */
#define L_HFI_SAMPLES_PER_CYCLE (uint16_t)(CURRENT_LOOP_HZ / L_HFI_FREQ)
#define L_HFI_TOTAL_SAMPLES (L_HFI_CYCLES * L_HFI_SAMPLES_PER_CYCLE)

/* ============ PP + Encoder 校准参数（合并）============ */
#define PP_ELEC_OMEGA       _PI         /* 强拖电角速度 rad/s（π rad/s ≈ 0.5 转/秒电角度）*/
#define PP_ELEC_TURNS       4.0f        /* 旋转电角度圈数 */
#define PP_VOLTAGE          5.0f        /* D轴强拖电压 V */
/* 一次旋转同时完成 PP 和 Encoder，总采样点数由旋转圈数决定 */
#define PP_ENC_TOTAL_SAMPLES  ((uint16_t)(PP_ELEC_TURNS * _2_PI * POS_LOOP_HZ / PP_ELEC_OMEGA))

#endif
