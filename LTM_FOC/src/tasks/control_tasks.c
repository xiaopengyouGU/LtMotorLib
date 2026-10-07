#include "tasks/control_tasks.h"
#include "common/tasks_param_def.h"
#include "schedule/lt_fsm.h"
#include "tasks/driver_task.h"   
#include "ltm_hal/ltm_hal.h"

/* 组件层：控制算法（全定点）*/
#include "ltm_ctrl/lt_control.h"
#include "ltm_ctrl/lt_math.h"
#include "ltm_ctrl/lt_analysis.h"
#include <string.h>

/*======================== 三环内部域 ========================
 * 位置   Q24：1.0 = 一圈（多圈累计）
 * 速度   Q24：1.0 = 一圈/s（与位置同基，位置环输出可直接当速度环给定）
 * 电流   Q15：32767 = ADC 满量程电流
 * 电压   Q15：1.0 = 母线/√3（lt_foc_update 的输入口径）
 * 规划器 Unit 也是 Q24 圈值；上报快照里的速度另存 count/s
 * 应用单位（°/RPM/%/A）只在 lt_motor_set 和 lt_motor_get_info 出现
 * PID 池：0=Id 1=Iq（Q15，电流环）2=速度 3=位置（Q24，速度/位置环）
 *==========================================================*/
#define PID_ID          0U
#define PID_IQ          1U
#define PID_SPEED       2U
#define PID_POS         3U
#define SCURVE_IDX      0U

/* ======================== 控制对象 ======================== */
typedef struct {
    uint8_t      div_cnt;                   /* 级联分频计数器 */
    uint32_t     zero_offset;               /* 电角度零点（calib 结果）*/
    int32_t      target;                    /* 目标值：已按模式完成换算 */
    int32_t      speed_ref_q24;             /* 速度给定（Q24 圈/s）*/
    int32_t      speed_ramp_q24;            /* 斜坡速度模式当前值 */
    int32_t      pos_cmd_prev;              /* 上拍规划位置：用于速度前馈 */
    uint8_t      scurve_ready;              /* 位置规划器已启动 */
    uint8_t      in_pos;                    /* 位置到位指示 */
    int32_t      ph_i_q15[3];               /* 本拍三相电流，死区补偿用 */
    int32_t      stop_step_q24;             /* 本次停机的斜率：受控 / 急停 */
    lt_err_t     err;                       /* 保护错误码：首错锁存，重上电才清 */
    tasks_mode_t mode;                      /* 指令：模式 */
    tasks_info_t info;                      /* 上报快照（内部单位）*/
} tasks_obj_t;

static tasks_obj_t  tasks_obj;
static tasks_obj_t *tasks = &tasks_obj;
static tasks_info_t *tasks_info = &tasks_obj.info;

typedef struct {
    uint8_t  idx;           /* PID 池下标 */
    uint8_t  type;          /* 0 = Q15 信号，1 = Q24 */
    uint32_t freq;          /* 调用频率 */
    int32_t  kp, ki, kd;    /* Q15.15 增益 */
    int32_t  out_max;       /* 输出限幅（正负对称）*/
} pid_cfg_t;

/* 三环 PID 参数配置表，Q格式标幺化 */
static const pid_cfg_t pid_cfg[] = {
    { PID_ID,    0, CURRENT_LOOP_HZ, CURR_KP_Q15,  CURR_KI_Q15,  0,          Q15_PU },
    { PID_IQ,    0, CURRENT_LOOP_HZ, CURR_KP_Q15,  CURR_KI_Q15,  0,          Q15_PU },
    { PID_SPEED, 1, SPEED_LOOP_HZ,   SPEED_KP_Q15, SPEED_KI_Q15, 0,          SPEED_IQ_LIMIT_Q24 },
    { PID_POS,   1, POS_LOOP_HZ,     POS_KP_Q15,   0,            POS_KD_Q15, Q24_PU },
};

/* 三环控制任务，内部默认级联分频，无指令更新延迟 */
static void    current_loop_task(void);            /* 电流环任务：默认 20kHz */
static void    speed_loop_task(void);              /* 速度环任务：默认 4kHz */
static void    position_loop_task(void);           /* 位置环任务：默认 1kHz */

/* 上电电角度绝对零点：纯 D 轴静态吸死 —— 磁场钉在 the=0，转子吸到最近的 d 轴，
 * 末段取 ZERO_ALIGN_AVG 点单圈 count 平均作为零点。阻塞，必须在 ISR 回调绑定前调用 */
void control_tasks_zero_find(void)
{
    int32_t dutys[3];
    int32_t sum = 0;

    ltm_pwm_start();

    lt_foc_update(ZERO_FIND_DUTY_Q15, 0, 0, dutys);      /* the=0 的静态 d 轴磁场 */
    ltm_pwm_set_dutys(dutys[0], dutys[1], dutys[2]);
    ltm_delay_ms(ZERO_ALIGN_HOLD_MS);

    for (uint8_t k = 0; k < ZERO_ALIGN_AVG; k++) {
        ltm_enc_update();
        sum += (int32_t)ltm_enc_get_count();
        ltm_delay_ms(1);
    }

    ltm_pwm_set_dutys(0, 0, 0);
    ltm_pwm_stop();

    int32_t off = sum / (int32_t)ZERO_ALIGN_AVG;
    off %= (int32_t)MOTOR_CPR;
    if (off < 0) off += (int32_t)MOTOR_CPR;

    tasks->zero_offset = (uint32_t)off;                  /* 控制层直接持有零点 */
}
void control_tasks_init(void)
{
    memset(tasks, 0, sizeof(tasks_obj_t));
    tasks->err = LT_OK;                       
    tasks->div_cnt = POS_DIV - 1;             /* 第一拍就落到 cnt == 0，位置环立即执行 */
    tasks->stop_step_q24 = STOP_RAMP_STEP_Q24;/* 停机默认走受控斜率 */

    lt_foc_init();
    lt_foc_set(MOTOR_CPR);                    /* 输入编码器单圈分辨率 */

    /* 四个环同构，照表一个循环初始化 */
    for (unsigned i = 0; i < sizeof(pid_cfg) / sizeof(pid_cfg[0]); i++) {
        const pid_cfg_t *cfg = &pid_cfg[i];
        lt_pid_init(cfg->idx, cfg->type, cfg->freq);
        lt_pid_set (cfg->idx, cfg->kp, cfg->ki, cfg->kd);
        lt_pid_set_limits(cfg->idx, cfg->out_max, -cfg->out_max);
        lt_pid_set_target(cfg->idx, 0);
    }

    lt_speed_init(MOTOR_CPR, CURRENT_LOOP_HZ);  /* PLL测速 在电流环里跑 */
    lt_speed_set((uint32_t)PLL_KP, (uint32_t)PLL_KI);
}

void control_tasks_run(void)                /* 三环级联分频，无指令更新延迟 */
{
    ltm_enc_update();                       /* 立即锁存编码器计数 */

    uint8_t cnt = tasks->div_cnt + 1;
    /* 分频用比较，性能更好；POS_DIV = 4·SPEED_DIV */
    if (cnt >= POS_DIV) {                   
        cnt = 0;                            /* 分频计数器重置 */
        position_loop_task();               /* 位置环 → 速度环 → 电流环，逐级给给定 */
        speed_loop_task();
    } else if (cnt == SPEED_DIV || cnt == 2U * SPEED_DIV || cnt == 3U * SPEED_DIV) {
        speed_loop_task();                  /* 每 SPEED_DIV 拍：4kHz 速度环 */
    }
    current_loop_task();
    tasks->div_cnt = cnt;
}

/* 模式与目标值设置，该函数仅在应用层调用，target 已经按 mode 换算 */
void control_tasks_set(tasks_mode_t mode, int32_t target) 
{
    uint8_t changed = (mode != tasks->mode);

    if (changed) {
        tasks->mode = mode;
        lt_pid_reset(PID_SPEED);                /* 换模式清积分，防旧目标残留 */
        lt_pid_reset(PID_POS);
    }
    tasks->target = target;

    switch (mode) {
    case Mode_Torque:                           /* 力矩模式 */
    case Mode_Ramp_Torque:                      /* 目标就是 iq 给定（Q15），应用层给的是 A */
        lt_pid_set_target(PID_IQ, target);
        lt_pid_set_target(PID_ID, 0);
        break;

    case Mode_Speed:                            /* 目标就是速度给定（Q24 圈/s）*/
        tasks->speed_ref_q24 = target;
        break;

    case Mode_Ramp_Speed:                       /* 斜坡起点取当前转速 */
        if (changed) tasks->speed_ramp_q24 = CNT_TO_Q24(tasks_info->speed_pll);
        tasks->speed_ref_q24 = tasks->speed_ramp_q24;
        break;

    case Mode_Stop:                             /* 可控停机：给定从当前转速起收 */
        tasks->speed_ref_q24 = CNT_TO_Q24(tasks_info->speed_pll);
        break;

    case Mode_Position:
        tasks->speed_ref_q24 = 0;             /* 位置环接管速度给定，起步从 0 */
        tasks->scurve_ready  = 0;
        /* 规划只在 main 上下文做：Running 时重规划，lt_scurve 的 ready 会挡住中断步进 */
        if (lt_fsm_get() == State_Running) control_tasks_plan_start();
        break;

    default:   break;                         /* 开环：目标直通电流环 */
    }
}

void control_tasks_stop(uint8_t estop)        /* 停机 */
{
    tasks->stop_step_q24 = estop ? ESTOP_RAMP_STEP_Q24 : STOP_RAMP_STEP_Q24;
    control_tasks_set(Mode_Stop, 0);
}

void control_tasks_get(tasks_info_t *info)   
{
    if (!info) return;
    *info = tasks->info;                     /* 位置 count、速度 count/s、电流 Q15、温度 0.1℃ */
    info->mode   = tasks->mode;
    info->target = tasks->target;            /* 已按 mode 换算 */
    info->state  = lt_fsm_get();
    info->err    = tasks->err;               /* 保护错误码（LT_OK = 正常）*/
}

/* 位置模式重新规划：Unit = Q24 圈值，和位置环/速度环同基准
 * （规划器 Unit 范围 int32，所以绝对位置范围是 ±128 圈，跟位置环一致）*/
void control_tasks_plan_start(void)
{
    if (tasks->mode != Mode_Position) return;

    lt_scurve_config_t cfg;
    int64_t start_cnt = lt_clamp_i64(ltm_enc_get_position(), POS_CNT_MAX, -POS_CNT_MAX);
    cfg.start_pos  = CNT_TO_Q24(start_cnt);        /* 起点：实时绝对位置（先夹 ±120 圈，Q24 不溢出）*/
    cfg.target_pos = tasks->target;                /* 已是 Q24 圈值 */
    cfg.v_start    = 0;
    cfg.v_max      = POS_VMAX_Q24;
    cfg.v_stop     = 0;
    cfg.acct_ms    = POS_ACC_MS;
    cfg.dect_ms    = POS_ACC_MS;
    cfg.period_ms  = 1;                             /* 位置环 1kHz */
    cfg.type       = 0;                             /* T 形加减速 */
    lt_scurve_start(SCURVE_IDX, &cfg);
    tasks->pos_cmd_prev = cfg.start_pos;            /* 前馈起点对齐，首拍不跳变 */

    tasks->scurve_ready = 1;                
}

/****************************************************************************************/
/*==============================================================================
 * 保护动作：错误码首错锁存 → 封锁 PWM 输出 → lt_fsm_update(Event_Fault)
 * 封锁输出走 driver_disable()（停 PWM），自由停机
 *============================================================================*/
static void _protect_trip(lt_err_t err)
{
    if (tasks->err == LT_OK) tasks->err = err;   /* 首错锁存 */
    driver_disable();                            /* 封锁输出 */
    lt_fsm_update(Event_Fault);                  /* fsm 进 Error */
}

/* 电压/温度都是慢变量，位置环上报 */
static void _protect_scan(int32_t vbus_q15, int32_t motor_temp, int32_t driver_temp)
{
    static lt_err_t prot_err;                       /* 上拍故障码（文件域）：初值 0 = LT_OK，而保护码都 ≤ -5，天然不会误判成同一故障*/
    static uint8_t  prot_cnt;                       /* 同一故障连续命中次数 */
    lt_err_t err;

    if      (vbus_q15 > VBUS_OV_Q15)        err = LT_ERR_OVER_VOLT;   /* 母线过压 */
    else if (vbus_q15 < VBUS_UV_Q15)        err = LT_ERR_UNDER_VOLT;  /* 母线欠压 */
    else if (driver_temp > DRIVER_OT_01C)   err = LT_ERR_OVER_TEMP_DRIVER;  /* 驱动器过温 */
    else if (motor_temp  > MOTOR_OT_01C)    err = LT_ERR_OVER_TEMP_MOTOR;   /* 电机过温 */
    else {
            prot_err = 0; 
            prot_cnt = 0; 
            return; 
    }

    /* 去抖：母线/温度都是慢变量，单点毛刺不该停机；连续 N 次同一故障才触发保护停机 */
    if (err != prot_err) {
        prot_err = err;
        prot_cnt = 1;
    } else if (prot_cnt < PROTECT_DEBOUNCE) {
        prot_cnt++;
    }
    if (prot_cnt >= PROTECT_DEBOUNCE) {
        _protect_trip(err);
    }
}

static void current_loop_task(void)                 /* 电流环任务 */
{
    uint32_t pos_count  = ltm_enc_get_count();      /* count = 0~CPR-1 */
    int32_t  Id, Iq, speed_pll;
    int32_t  Ia, Ib, Ic;
    int32_t  vd, vq;
    int32_t  dutys[3] = { 0, 0, 0 };
    lt_speed_update(pos_count);                     /* PLL测速更新：count/s */
    lt_speed_get(NULL, &speed_pll);

    /* 电角度 =（单圈计数 − 零点偏置）× 极对数 */
    uint32_t the  = COUNT_TO_THE(pos_count, tasks->zero_offset);
    /* 1拍 电角度增量，用于延迟补偿 */
    int32_t  adv1 = SPEED_TO_THE(speed_pll);
#if !ANGLE_DELAY_STEP
    uint32_t the_park = the;                        /* 角度采样无一拍延迟 */
#else
    uint32_t the_park = the + (uint32_t)(adv1);     /* 角度采样一拍延迟补偿 */ 
#endif
    uint32_t the_mod = the_park + (uint32_t)((adv1 * 3) >> 1); /* 1.5 拍延迟补偿 */

    ltm_adc_get_current(&Ia, &Ib, &Ic);               /* HAL 已归一化成 Q15 标幺 */
    if (Ia >= OC_CURRENT_Q15 || Ia <= -OC_CURRENT_Q15 ||
        Ib >= OC_CURRENT_Q15 || Ib <= -OC_CURRENT_Q15 ||
        Ic >= OC_CURRENT_Q15 || Ic <= -OC_CURRENT_Q15) {
        _protect_trip(LT_ERR_OVER_CURRENT);                     /* 触发过流保护 */
    }

    int32_t f3[3] = { Ia, Ib, Ic };
    tasks->ph_i_q15[0] = Ia;                          /* 死区补偿用 */
    tasks->ph_i_q15[1] = Ib;
    tasks->ph_i_q15[2] = Ic;

    /* 幅值不变 Clarke+Park：与 lt_foc_update 共用同一张表和 step，严格互逆 */
    lt_foc_clark_park(f3, the_park, &Id, &Iq);
    /* 上报存 Q15 标幺 */
    tasks_info->Ia = Ia;                              
    tasks_info->Ib = Ib;
    tasks_info->Ic = Ic;
    tasks_info->Id = Id;
    tasks_info->Iq = Iq;

    if (lt_fsm_get() != State_Running) {
        /* 非运行态：占空比归零。受控停机完成后就停在这——静止时反电势为 0，
         * 三相直通也几乎不产生力矩（零力矩保持）；反拖时才会变制动 */
        ltm_pwm_set_dutys(0, 0, 0);
        return;
    }
    /* 电流环正式运行 */
    if (tasks->mode == Mode_Open_Loop) {
        vq = tasks->target;                     /* 已是 Q15：1.0 pu = 母线/√3 */
        vd = 0;
    } else {
        vd = lt_pi_update(PID_ID, Id);          /* 输出 Q15 电压标幺（1.0 pu = 母线/√3）*/
        vq = lt_pi_update(PID_IQ, Iq);
        /* DQ 交叉前馈解耦：Vd_ff = −we·Ls·Iq、Vq_ff = we·(Ls·Id + ψf)，
         * 系数按 pu 折在 tasks_param_def.h，释放高速电压裕量 */
        /* we = speed·2π·PP/CPR：系数折成 Q30 常量 */
        int32_t we = (int32_t)(((int64_t)speed_pll * WE_K_Q30) >> 30);
        /* FF_L_Q24 为了让"乘 Q15 电流"这一步成立预先 ×32768，所以必须先 >>15 降回 Q24
         * 再乘 we——漏了这级就把解耦前馈放大 32768 倍，电压直接被焊在限幅上、电流环失控。
         * FF_PSI_Q24 不含电流因子，它的 >>24 正好出 Q15，不用降 */
        int32_t li_d = (int32_t)(((int64_t)FF_L_Q24 * Id) >> 15);
        int32_t li_q = (int32_t)(((int64_t)FF_L_Q24 * Iq) >> 15);
        vd -= (int32_t)(((int64_t)we * li_q) >> 24);
        vq += (int32_t)(((int64_t)we * (li_d + FF_PSI_Q24)) >> 24);
    }
    /* 前馈叠加后仍留在 Q15，避免落到 int16 时回绕 */
    vd = lt_clamp_i32(vd, Q15_PU, -Q15_PU);
    vq = lt_clamp_i32(vq, Q15_PU, -Q15_PU);
    /* SVPWM 输出 Q15 占空比 */
    lt_foc_update((int16_t)vd, (int16_t)vq, the_mod, dutys);   
    ltm_pwm_set_dutys(dutys[0], dutys[1], dutys[2]);
}

static void speed_loop_task(void)                   /* 速度环任务 */
{
    int32_t speed, speed_pll;
    tasks_mode_t mode     = tasks->mode;
    lt_speed_get(&speed, &speed_pll);               /* PLL测速已在电流环里更新，单位 count/s */
    int32_t speed_q24     = CNT_TO_Q24(speed_pll);
    int32_t speed_ref_q24 = tasks->speed_ref_q24;   /* 参考速度给定 */
    /* 测速结果上报，并进行过速保护判断：±OVER_SPEED_Q24 */
    tasks_info->speed     = speed;                  
    tasks_info->speed_pll = speed_pll;
    if (speed_q24 > OVER_SPEED_Q24 || speed_q24 < -OVER_SPEED_Q24) {
        _protect_trip(LT_ERR_OVER_SPEED);
    }                  
    if (lt_fsm_get() != State_Running)      return;

    if (mode == Mode_Stop) {
        /* 可控停机模式：给定按本次斜率（受控/急停）往 0 收 */
        int32_t step = tasks->stop_step_q24;
        if      (speed_ref_q24 >  step) speed_ref_q24 -= step;
        else if (speed_ref_q24 < -step) speed_ref_q24 += step;
        else                            speed_ref_q24  = 0;

        /* 速度给定收到 0、且实测速度也进了窗口 → 清占空比（此刻反电势已可忽略），进 Stop */
        if (speed_ref_q24 == 0 &&
            speed_q24 <= STOP_DONE_Q24 && speed_q24 >= -STOP_DONE_Q24) {
            ltm_pwm_set_dutys(0, 0, 0);
            lt_fsm_update(Event_Stop);          /* 停完 → fsm 进 Stop，mode 停在 Mode_Stop */
            return;
        }
    } else {
        if (mode == Mode_Open_Loop || mode == Mode_Torque ||
            mode == Mode_Ramp_Torque)           return;

        /* 斜坡速度：目标按 SPEED_RAMP_STEP_Q24 逼近，阶跃指令不产生冲击 */
        if (mode == Mode_Ramp_Speed) {
            int32_t d = tasks->target - tasks->speed_ramp_q24;
            if (d > SPEED_RAMP_STEP_Q24)        tasks->speed_ramp_q24 += SPEED_RAMP_STEP_Q24;
            else if (d < -SPEED_RAMP_STEP_Q24)  tasks->speed_ramp_q24 -= SPEED_RAMP_STEP_Q24;
            else                                tasks->speed_ramp_q24  = tasks->target;
            speed_ref_q24 = tasks->speed_ramp_q24;
        }
    }
    /* 速度模式：speed_ref_q24 由 control_tasks_set 直接写；
     * 位置模式：由位置环逐拍写 */

    /* 速度环 PI：给定与反馈都是 Q24 圈/s */
    lt_pid_set_target(PID_SPEED, speed_ref_q24);
    int32_t iq_q15 = lt_pi_update(PID_SPEED, speed_q24) >> 9;   /* Q24 → Q15 */

#if FRICTION_FF_ENABLE
    /* 摩擦前馈（全 pu）：随目标方向给突破电流 + 粘性项，PI 不用憋力矩过静摩擦死区；
     * 死区防零速抖振。本机阻尼小，默认关 */
    int32_t visc_q15 = (int32_t)(((int64_t)speed_ref_q24 * FRICTION_VISCOUS_Q15) >> 24);
    if (speed_ref_q24 > FRICTION_FF_DEADBAND_Q24) {
        iq_q15 +=  FRICTION_STATIC_Q15 + visc_q15;
    } else if (speed_ref_q24 < -FRICTION_FF_DEADBAND_Q24) {
        iq_q15 += -FRICTION_STATIC_Q15 + visc_q15;
    }
#endif

#if COGGING_COMP_ENABLE
    /* 齿槽前馈：按绝对位置查表，正反转都用表原值抵消；同样受速度死区门控，
     * 直接叠 Q15 标幺 */
    if (lt_cogging_is_done() && (speed_ref_q24 >  FRICTION_FF_DEADBAND_Q24 ||
                                 speed_ref_q24 < -FRICTION_FF_DEADBAND_Q24)) {
        iq_q15 += lt_cogging_get(ltm_enc_get_count());
    }
#endif

    iq_q15 = lt_clamp_i32(iq_q15, Q15_PU, -Q15_PU);   /* 前馈叠加后仍留在 Q15 */
    tasks->speed_ref_q24 = speed_ref_q24;               /* 参考速度更新 */    
    lt_pid_set_target(PID_IQ, iq_q15);
}

static void position_loop_task(void)
{
    int64_t pos_cnt = ltm_enc_get_position();       /* 多圈绝对位置：count */
    int32_t motor_temp, driver_temp, vbus;

    ltm_adc_get_temp(&motor_temp, &driver_temp);    /* 0.1℃ */
    ltm_adc_get_vbus(&vbus);                        /* Q15 标幺 */
    /* 上报电机参数 */
    tasks_info->pos         = pos_cnt;
    tasks_info->driver_temp = driver_temp;
    tasks_info->motor_temp  = motor_temp;
    tasks_info->vbus        = vbus;

    _protect_scan(vbus, motor_temp, driver_temp);   /* 保护扫描：任一触发就进 Error */

    if (lt_fsm_get() != State_Running)   return;
    if (tasks->mode  != Mode_Position)   return;   /* 非位置模式直接退出 */

    /* Q24 只覆盖 ±128 圈，多圈 count 超范围先夹住再换算——否则 CNT_TO_Q24 撑爆 int32(UB)。
     * 速度模式跑久 / 位置跑飞都会让 |pos_cnt| 超 ±120 圈，此时位置环失效 */
    if (pos_cnt >= POS_CNT_MAX || pos_cnt <= -POS_CNT_MAX)  return;
    int32_t pos_q24 = CNT_TO_Q24(pos_cnt);          /* 折算到 Q24 圈值 */

    /* 规划器 Unit 也是 Q24 圈值，和位置环同基：输出直接当给定，不用换算。
     * 到位判定同样在 pu 域 */
    int32_t cmd_q24 = tasks->scurve_ready ? lt_scurve_update(SCURVE_IDX) : pos_q24;
    /* 窗口到位判断：*/
    tasks->in_pos = (pos_q24 <= tasks->target + POS_WIN_Q24 &&
                     pos_q24 >= tasks->target - POS_WIN_Q24) ? 1 : 0;
    lt_pid_set_target(PID_POS, cmd_q24);
    int32_t speed_q24 = lt_pd_update(PID_POS, pos_q24);
    int32_t ff_q24 = (cmd_q24 - tasks->pos_cmd_prev) * (int32_t)POS_LOOP_HZ;
    tasks->pos_cmd_prev = cmd_q24;

    /* 前馈叠加后再限幅 */
    tasks->speed_ref_q24 = lt_clamp_i32(speed_q24 + ff_q24, POS_VMAX_Q24, -POS_VMAX_Q24);
}

