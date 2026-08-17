#ifndef TASKS_PARAM_DEF_H
#define TASKS_PARAM_DEF_H

/* ============ 控制任务参数 ============ */
#define UNIT_TO_RPM									18.0f		/* 1/10000 * 3000 * 60 ：换算系数 1/reso * fv * 60,fv：速度环频率，reso:分辨率（Unit） */
#define POLE_PAIRS                                  5           /* 电机极对数 */
#define RPM_TO_WR                                   0.5235988f  /* RPM 到电角速度（Rad/s）转换系数 */
#define ANGLE_EL_PER_COUNT                          0.0031416f  /* ENCODER_RAD_PER_COUNT * POLE_PAIRS :机械角度换算系数 * 极对数 ==> 电角度换算系数 */
#define ZERO_FIND_TICKS                             5000        /* 电角度绝对零点寻找时间（Tick）*/
#define ZERO_FIND_DUTY                              0.05f       /* 零点定位占空比 */
#define ENCODER_OFFSET                              52399       /* 电角度绝对零点与机械零点偏移值（Count）*/
#define DEG_TO_RAD                                  0.0174533f
#define CURRENT_LOOP_PERIOD                         0.00005f    /* 20kHz */
#define MAIN_LOOP_PERIOD                            20.0f * CURRENT_LOOP_PERIOD  

/* ============ 校准参数 ============ */
#define R_POINTS            4           /* 多点差分法电流档位数 */
#define R_SETTLE_TICKS      1000        /* 每档暂态等待 tick (20kHz = 50ms) */
#define R_SAMPLE_TICKS      4000        /* 每档采样 tick (20kHz = 200ms) */
#define R_CALIB_CURRENT     2.0f        /* 校准最大电流 A */
#define R_CALIB_KI          2.0f        /* 简单电流校准闭环的KI */

/* ============ L 校准参数（高频注入法） ============ */
#define L_HFI_VOLTAGE       3.0f        /* 高频注入电压幅值 (V)，额定电压10%以上 */
#define L_HFI_VOLTAGE_INV   0.3333333f /* 1.0f / L_HFI_VOLTAGE，预计算常数 */
#define L_HFI_FREQ          500.0f      /* 高频注入频率 (Hz) */
#define L_HFI_OMEGA         (6.2831853f * L_HFI_FREQ)  /* 角频率 (rad/s) */
#define L_HFI_CYCLES        100         /* 解调采集周期数 */
#define L_HFI_SAMPLES_PER_CYCLE (uint16_t)(1.0f / (L_HFI_FREQ * CURRENT_LOOP_PERIOD))
#define L_HFI_TOTAL_SAMPLES (L_HFI_CYCLES * L_HFI_SAMPLES_PER_CYCLE)

/* ============ PP + Encoder 校准参数（合并） ============ */
#define PP_SPEED            3.14159f    /* 电角速度 rad/s */
#define PP_ELEC_TURNS       4.0f        /* 旋转电角度圈数 */
#define PP_VOLTAGE          3.0f        /* D轴强拖电压 V */
/* 一次旋转同时完成 PP 和 Encoder，总采样点数由旋转圈数决定 */
#define PP_ENC_TOTAL_SAMPLES    (uint16_t)(PP_ELEC_TURNS * _2_PI / (PP_SPEED * MAIN_LOOP_PERIOD))
#define ENCODER_CPR         262144      /* 编码器单圈分辨率 */


#endif