/*
 * 聚合公共头（自动生成自各模块头，勿手改；内部实现仍用模块头）
 */
#ifndef LT_CONTROL_H__
#define LT_CONTROL_H__

#include <stdint.h>

/*------------------------- 定点 PID（全整数，静态实例池）-------------------------
 * 信号格式 init 声明（type）：0 = Q15（32767 = 1.0）、1 = Q24（2^24 = 1.0）。
 * 增益一律 Q15.15（32768 = 1.0）：Kp 无量纲、Ki 1/s、Kd s；内部存每拍系数
 * Ki·ts / Kd·freq，故换调用频率不用重算增益。idx 是静态池下标（0 ~ 上限-1）。
 * Q24 实例误差 target-curr 按 ±2^30 饱和；输出限幅默认给满量程，可 set_limits 改。
 *
 *     lt_pid_init(2, 20000);                    // 20kHz 调用
 *     lt_pid_set(2, Kp, Ki, Kd);                // Q15.15 增益
 *     lt_pid_set_limits(2, out_max, out_min);   // 可非对称
 *     out = lt_pi_update(2, curr);              // 每拍调用
 * -----------------------------------------------------------------------------*/

#ifndef LT_PID_MAX_INSTANCES
#define LT_PID_MAX_INSTANCES    4       /* 提供的 PID 实例数，索引 0-3 */
#endif

uint8_t lt_pid_init(uint8_t idx, uint32_t freq);

void    lt_pid_reset(uint8_t idx);
void    lt_pid_set(uint8_t idx, int32_t Kp, int32_t Ki, int32_t Kd);
void    lt_pid_set_target(uint8_t idx, int32_t target);
void    lt_pid_set_limits(uint8_t idx, int32_t out_max, int32_t out_min);

int32_t lt_pid_get(uint8_t idx);
int32_t lt_pi_update(uint8_t  idx, int32_t curr_val);   /* 增量式 PI，电流环 / 速度环 */
int32_t lt_pd_update(uint8_t  idx, int32_t curr_val);   /* 增量式 PD，位置环 */
int32_t lt_pid_update(uint8_t idx, int32_t curr_val);   /* 增量式 PID */

/*------------------------- FOC 算法 ---------------------------------*/

/* 单位约定：
 *  Vd/Vq: 标幺化（1.0 pu = Vdc/√3）并转Q15定点，即 [-1.0, 1.0) <==> [-32768, 32768) 
 *  the  ：电角度，(单圈编码器原始计数 - 电角度零点偏置) × 极对数，可以加上延迟补偿量 
 *  reso ：编码器单圈分辨率
 *  dutys：最终算出的三相占空比（Q15） 
 *  fnum ：三相电流/电压，Q15 标幺；fd/fq ：输出 d/q，Q15 标幺 
 */
void lt_foc_init(void);                                     /* 必须先初始化 */
void lt_foc_set(uint32_t reso);                             /* 初始化后设置分辨率 */
void lt_foc_update(int16_t Vd, int16_t Vq, uint32_t the, int32_t dutys[3]);/* Q15 定点 FOC，输出三相占空比（Q15）*/
void lt_foc_clark_park(int32_t fnum[3], uint32_t the, int32_t *fd, int32_t *fq);

/*------------------------- FOC 算法 ---------------------------------*/

/*自适应 M 法开关：
 *   1 = 启用：M 法测得准，主要用于评估/对比 PLL 参数设计的效果
 *   0 = 关闭（默认）：只跑锁相环，热路径更短
 * 关闭时 lt_speed_get() 的 adap_speed 恒为 0 */
#ifndef LT_SPEED_USE_ADAP_M
#define LT_SPEED_USE_ADAP_M             0
#endif

/*------------------------- 测速接口 （count/s，高频调用）---------------------------------*/

void lt_speed_init(uint32_t reso, uint32_t freq);	/* reso : 编码器分辨率， freq : 调用频率（Hz） */
void lt_speed_set(uint32_t pll_kp, uint32_t pll_ki);/* pll_kp : 比例增益（1/s），pll_ki : 积分增益（1/s^2），内部已乘调用周期 dt */
void lt_speed_update(uint32_t pos_count);		    /* 测速更新，输入编码器原始值：count（单圈计数 0~reso-1）*/
void lt_speed_get(int32_t *adap_speed, int32_t *pll_speed); /* 返回锁相环测速结果（count/s）；adap_speed 仅在开关打开时有效，否则恒为 0 */

/*------------------------- 测速接口 （count/s，高频调用） ---------------------------------*/

/* ==================== 锁相环参数整定（lt_speed_set） ====================
 *   1) 选带宽 f_n：速度环常用 20~100 Hz，锁相环取前者的 3~5 倍。
 *   2) 选阻尼比 ζ：0.7（最快、超调约 4%）~ 1.0（无超调），一般取 0.7。
 *   3) 反算增益：
 *          Kp = 4π·ζ·f_n              （只跟 ζ、f_n 有关）
 *          Ki = (2π·f_n)² / freq      （反比于调用频率）
 *      例：freq = 20 kHz、f_n = 300 Hz、ζ = 0.7 → Kp = 2639、Ki = 178
 *
 *   测速输出的低通系数按 Ki 自动整定，不用配。
 * ======================================================================== */

/*---------------- 速度环 DOB+P ＋ 电流环无差拍 DPCC/ESO ----------------
 * 对外单位：速度 count/s；电流、电压一律 Q15 标幺（32767 = 1.0 pu）。
 *   id、iq：32767 = i_base_mA      vbus：32767 = 满量程母线
 *   Vd、Vq：1.0 pu = 实测母线/√3，可直接进 lt_foc_update
 *
 *   static const lt_pdob_config_t cfg = {
 *       .reso = 4096,        .pp = 10,              .freq_speed = 5000,
 *       .freq_curr = 20000,   .freq_max = 600,       // 填最高转速，不是基速
 *       .Ls_uH = 333,        .Rs_mohm = 560,        .phi_uWb = 13800,
 *       .i_base_mA = 33000,  .v_base_mV = 60000,
 *       .J_nNm_s2 = 50000,   .speed_max = 12000000, .d_limit_iq = 60000 };
 *
 *   lt_pdob_init(&cfg);
 *   lt_pdob_speed_set(kp_q15, 800, 32767);   // kp / DOB带宽(rad/s) / iq限幅（给总电流预算）
 *   lt_pdob_curr_set(1200, 32767);           // ESO带宽(rad/s) / 电压限幅
 *   lt_pdob_fw_set(3000);                    // 弱磁（可选）：反电势 > 0.85pu 自动开，0 = 关
 *
 *   iq_ref = lt_pdob_speed_update(speed, iq);      // 每 freq_speed 拍，we 也在这句更新
 *   lt_pdob_curr_update(id, iq, vbus, &Vd, &Vq);   // 每 freq_curr 拍
 * -------------------------------------------------------------------*/

typedef struct {
    uint32_t reso, pp;                  /* 编码器单圈分辨率、极对数 */
    uint32_t freq_speed, freq_curr;        /* 速度环 / 电流环调用频率（Hz）*/
    uint32_t freq_max;                  /* 频率基准：最高转速对应的电频率（Hz）*/
    int32_t  Ls_uH, Rs_mohm, phi_uWb;   /* 相电感、相电阻、永磁磁链 */
    int32_t  i_base_mA;                 /* 电流基准：ADC 可测最大峰值电流 */
    int32_t  v_base_mV;                 /* 电压基准：ADC 满量程母线电压 */
    int32_t  J_nNm_s2;                  /* 总转动惯量 nN·m·s²（5e-5 kg·m² → 50000）*/
    int32_t  speed_max;                 /* 最高转速 count/s（同时是速度环标幺基准）*/
    int32_t  d_limit_iq;                /* 扰动等效 iq 上限（Q15）*/
} lt_pdob_config_t;

void lt_pdob_init(const lt_pdob_config_t *cfg);                     /* 上电 / 换电机一次 */

void lt_pdob_speed_set(int32_t kp, int32_t wb, int32_t out_limit);  /* kp / DOB带宽(rad/s) / iq限幅（给总电流预算）*/
void lt_pdob_curr_set(int32_t eso_wb, int32_t out_limit);           /* ESO带宽(rad/s，0关) / 电压限幅 */
void lt_pdob_fw_set(int32_t ki);                                    /* 弱磁增益（Q15），0 = 关 */
void lt_pdob_set_speed_ref(int32_t speed_ref);                      /* 速度指令 count/s */
void lt_pdob_set_curr_ref(int32_t id_ref, int32_t iq_ref);          /* 手动 id/iq（Q15）*/
int32_t lt_pdob_speed_update(int32_t speed, int32_t iq);            /* 每 freq_speed 拍 */
void lt_pdob_curr_update(int32_t id, int32_t iq, int32_t vbus,      /* 每 freq_curr 拍 */
                         int32_t *vd, int32_t *uq);
void lt_pdob_reset(void);                                           /* 清运行状态，保留配置 */

/*------------------------- 5段S型曲线（全整数，静态实例池）-------------------------
 * Unit 为指令单位，含义由用户自己确定。实例由内部数组提供，idx 就是数组下标
 * （0 ~ LT_SCURVE_MAX_INSTANCES-1），规划器可同时跑多条曲线。
 *
 *     lt_scurve_start(0, &cfg);        // 启动规划器 0（start 内完成全部初始化）
 *     pos = lt_scurve_update(0);       // 每 period_ms 调一次，返回位置指令
 * -----------------------------------------------------------------------------*/

#ifndef LT_SCURVE_MAX_INSTANCES
#define LT_SCURVE_MAX_INSTANCES    2    /* 提供的规划器实例数，索引 0-1 */
#endif

/* Unit 为指令单位，含义由用户自己确定 */
typedef struct {
    int32_t  start_pos;         /* 起始位置（Unit） */
    int32_t  target_pos;        /* 目标位置 (Unit)  */
    int32_t  v_start;           /* 起始速度（Unit/s）*/
    int32_t  v_max;             /* 最大速度（Unit/s）*/
    int32_t  v_stop;            /* 停止速度（Unit/s）*/
    uint16_t acct_ms;           /* 加速时间（ms)，最小 100ms */
    uint16_t dect_ms;           /* 减速时间 (ms)，最小 100ms */
    uint16_t period_ms;         /* 规划更新时间（ms）*/
    uint8_t  type;              /* 0:T形加减速，1：S形加减速 */
} lt_scurve_config_t;

void    lt_scurve_reset(uint8_t idx);                                /* 清空规划器状态 */
void    lt_scurve_start(uint8_t idx, lt_scurve_config_t *config);    /* 规划启动 */
int32_t lt_scurve_update(uint8_t idx);                               /* 每周期调用，返回位置指令（Unit）*/
uint8_t lt_scurve_is_done(uint8_t idx);
void    lt_scurve_stop(uint8_t idx);                                 /* 就地停住 */

/*------------------------- 电流极性法死区补偿 ---------------------------------*/

/* 死区补偿（Q15 标幺）：电流 32767 = i_base_mA，占空比 32767 = 100% */
void lt_deadzone_init(int32_t dead_duty_q15, int32_t ith_q15, uint8_t alpha_shift);
void lt_deadzone_set(int8_t dirA, int8_t dirB, int8_t dirC);   /* 1 同向，-1 反向 */
void lt_deadzone_compensate(int32_t ia, int32_t ib, int32_t ic, int32_t dutys[3]);

/*------------------------- 电流极性法死区补偿 ---------------------------------*/

/*------------------------- 陷波滤波器（机械共振抑制）---------------------------
 * 电流环专用，三级（一般 1~2 个齿轮谐振 + 1 个联轴器谐振），全定点，模块内无浮点。
 *
 *     lt_notch_init(20000);                       // 20kHz，先给调用频率
 *     lt_notch_set(0, 1200, 10, -20);             // 第0级：1.2kHz，Q=10，-20dB
 *     lt_notch_set(1, 2600, 15, -15);             // 第1级
 *     ...
 *     curr = lt_notch_update(curr);                 // 每拍调一次，Q15 进 Q15 出
 *
 * level    : 级号 0~2，按调用顺序串起来
 * fc_Hz    : 陷波中心频率 (Hz)，要低于 freq/2，建议 ≥ 100Hz
 * Q        : 品质因数 (5~20)，越大陷波越窄
 * depth_dB : 陷波深度（负整数 dB），常用 -20 ~ -40，≤ -60 按理想陷波算
 * -----------------------------------------------------------------------------*/

void    lt_notch_init(uint32_t freq);                  /* 调用频率 (Hz) */
void    lt_notch_set(uint8_t level, uint32_t fc_Hz, uint32_t Q, int32_t depth_dB);
void    lt_notch_reset(void);                          /* 清空状态，不改变系数 */
int32_t lt_notch_update(int32_t x);                    /* Q15 进 Q15 出 */

#endif /* LT_CONTROL_H__ */