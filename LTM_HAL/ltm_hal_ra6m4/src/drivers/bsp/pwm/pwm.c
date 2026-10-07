#include "bsp/pwm/pwm.h"

/* 三相 PWM 模块 */
static three_phase_ctrl_t * htim_ctrl = &g_three_phase0_ctrl;   /* 三相 PWM 控制结构体 */
static three_phase_cfg_t  * htim_cfg  = &g_three_phase0_cfg;    /* 三相 PWM 配置结构体 */

static timer_ctrl_t         *tim_u_ctrl = &g_timer0_ctrl; 
static timer_ctrl_t         *tim_v_ctrl = &g_timer1_ctrl;
static timer_ctrl_t         *tim_w_ctrl = &g_timer2_ctrl;

/* ================= 三相六路互补 + 死区（RA6M4 GPT6/7/8，2026-09-28 实测） =================
 * 硬件事实（RASC 生成的就是这套）：
 *   - GPT6/7/8 = 三角波对称 PWM（GTCR.MD=4），GTIOCnA/GTIOCnB 都设为"比较匹配翻转"。
 *   - GTIOCnA 在 GTCCRA 命中时翻转，GTIOCnB 在 GTCCRB 命中时翻转；
 *     GTIOCnA 初始电平低、GTIOCnB 初始电平高，所以 GTCCRA==GTCCRB 时两路天然反相。
 *   - GTIOCnA 的占空比 ≈ 2*(GTPR-GTCCRA)/周期，GTIOCnB ≈ 2*GTCCRB/周期。
 *
 * 实测踩到的两个坑（都会让某一相“永久反半周期”，也就是题目里 P601/P600 反了）：
 *   1) GTCCRA == GTCCRB（RASC 三相模块默认就是写同一个值）时，输出状态会退化，
 *      实测三相里会随机有一对变成同相而非互补。
 *   2) 计数器启动之后再改比较值，会让该相在本周期多翻转一次，相位就反了。
 *      ⇒ 必须“启动前预置比较值”；占空比运行中也要保证 A/B 比较值不相等。
 *   结论：死区必须靠 A/B 比较值错开，且三个相位的“相位状态”必须从一开始就确定。
 *
 * 死区做法（GTCCRA 取大、GTCCRB 取小）：
 *     GTCCRA = center + DEAD/2 ，GTCCRB = center - DEAD/2     （GTCCRA > GTCCRB）
 *     上计数：GTCCRA 先命中 → 一路先关断，GTCCRB 后命中 → 另一路后开通 ⇒ 两路都关断
 *     下计数：GTCCRB 先命中 → 一路先关断，GTCCRA 后命中 → 另一路后开通 ⇒ 两路都关断
 *   两条边沿各留出 DEAD 个计数；DEAD=50 @PCLKD 99MHz ≈ 505ns（与原 RASC "Dead Time" 同值）。
 *   （反过来取 GTCCRA 小、GTCCRB 大，就会出现“两路同时导通”的重叠——已实测。）
 *
 * 关于 GTDTCR.TDE（自带“死区计数器”）：本项目三角波模式下实测无效——
 *   TDE=1 时（GTDVU=50）GPIOCB 并不会变成 GTIOCA 的反相，而是“同相、滞后 50 计数”，
 *   三相占空比两路一模一样（就是之前看到的 84%/84%）。故必须 TDE=0，死区用比较值错开，
 *   这同样是硬件比较器产生的死区，没有 CPU 抖动。
 *
 * 端点处理：center 被限制在 [DEAD/2+1, GTPR-1-DEAD/2]，
 *   保证 GTCCRA/GTCCRB 都严格落在 (0, GTPR) 内且互不相等（0% / 100% 也不会退化）。
 * ============================================================================ */
#define PWM_DEAD_TIME_COUNTS   (50U)    /* 死区计数（≈505ns） */
#define PWM_BUF_REG_GTIOCA     (2U)     /* GTCCR[2] = 偏移 0x54：GTIOCnA 比较值缓冲 */
#define PWM_BUF_REG_GTIOCB     (3U)     /* GTCCR[3] = 偏移 0x58：GTIOCnB 比较值缓冲 */

/* 关掉 GPT 的硬件负相/死区功能（TDE=0），死区改用比较值偏移实现 */
static void pwm_disable_hw_negphase(timer_ctrl_t *tim)
{
    gpt_instance_ctrl_t *p = (gpt_instance_ctrl_t *) tim;

    /* GTDTCR 清零后 GTDVU/GTDVD/GTDBU/GTDBD/GTSOTR 全部无效 */
    p->p_reg->GTDTCR = 0U;
}

/* 把“中心比较值”换算成 A/B 两个比较值（GTCCRA 取大、GTCCRB 取小） */
static void pwm_phase_compares(uint32_t compare, uint32_t *p_ca, uint32_t *p_cb)
{
    uint32_t half   = PWM_DEAD_TIME_COUNTS / 2U;
    uint32_t center = compare;

    if (center > (FOC_COUNT_PERIOD - 1U - half))     /* 0% 端：不允许等于 GTPR */
    {
        center = FOC_COUNT_PERIOD - 1U - half;
    }
    if (center < (1U + half))                        /* 100% 端：不允许等于 0 */
    {
        center = 1U + half;
    }

    *p_ca = center + half;      /* GTCCRA：大 ⇒ 留死区 */
    *p_cb = center - half;      /* GTCCRB：小 */
}

/* 运行中更新：只写缓冲寄存器（GTBER 单缓冲，会在比较匹配点整体生效，不会丢翻转） */
static void pwm_write_phase(timer_ctrl_t *tim, uint32_t compare)
{
    gpt_instance_ctrl_t *p = (gpt_instance_ctrl_t *) tim;
    uint32_t             c_a;
    uint32_t             c_b;

    pwm_phase_compares(compare, &c_a, &c_b);

    p->p_reg->GTCCR[PWM_BUF_REG_GTIOCA] = c_a;
    p->p_reg->GTCCR[PWM_BUF_REG_GTIOCB] = c_b;
}

/* 启动前预置：比较寄存器和缓冲寄存器一起写。
 * 计数器停止时写 GTCCRA/GTCCRB 不会丢翻转；这样计数器一起来第一周期就是正确相位。 */
static void pwm_preload_phase(timer_ctrl_t *tim, uint32_t compare)
{
    gpt_instance_ctrl_t *p = (gpt_instance_ctrl_t *) tim;
    uint32_t             c_a;
    uint32_t             c_b;

    pwm_phase_compares(compare, &c_a, &c_b);

    p->p_reg->GTCCR[0] = c_a;   /* GTCCRA (0x4C) */
    p->p_reg->GTCCR[1] = c_b;   /* GTCCRB (0x50) */
    p->p_reg->GTCCR[PWM_BUF_REG_GTIOCA] = c_a;
    p->p_reg->GTCCR[PWM_BUF_REG_GTIOCB] = c_b;
}

static void pwm_write_dutys(uint32_t cU, uint32_t cV, uint32_t cW)
{
    pwm_write_phase(tim_u_ctrl, cU);
    pwm_write_phase(tim_v_ctrl, cV);
    pwm_write_phase(tim_w_ctrl, cW);
}

static void pwm_preload_dutys(uint32_t cU, uint32_t cV, uint32_t cW)
{
    pwm_preload_phase(tim_u_ctrl, cU);
    pwm_preload_phase(tim_v_ctrl, cV);
    pwm_preload_phase(tim_w_ctrl, cW);
}

void pwm_init(void)     /* 三相 PWM 初始化 */
{
   /* 初始化阶段让 GPT 三相定时器先跑起来，保证 ADC 触发链路已经建立，
    * 但把三相输出先置于安全状态，避免功率级在上电初始化时对电机施加电压。*/
    R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, FOC_ENABLE_PORT_PIN, BSP_IO_LEVEL_HIGH);
    R_GPT_THREE_PHASE_Open(htim_ctrl, htim_cfg);
    /* TDE=0：关掉硬件负相，避免它把 GTIOCB 覆盖成“同相、延后 50 计数” */
    pwm_disable_hw_negphase(tim_u_ctrl);
    pwm_disable_hw_negphase(tim_v_ctrl);
    pwm_disable_hw_negphase(tim_w_ctrl);
    /* 关键：比较值必须在计数器启动之前预置好（启动后再改会让该相多翻转一次 → 反半周期）*/
    pwm_preload_dutys(FOC_COUNT_PERIOD >> 1, FOC_COUNT_PERIOD >> 1, FOC_COUNT_PERIOD >> 1);
    R_GPT_THREE_PHASE_Start(htim_ctrl);
}

#define DUTY_MIN    0
#define DUTY_MAX    31130   /* 下桥臂采样FOC，最高输出占空比，对应 0.95 */

#define GET_MAX(duty)    ((duty) < DUTY_MAX ? (duty) : DUTY_MAX)
#define GET_MIN(duty)    ((duty) > DUTY_MIN ? (duty) : DUTY_MIN)
#define CLAMP_DUTY(duty) (32768 - GET_MAX(GET_MIN(duty)))   /* 需精确截断，不能改成32767 */

void pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC)
{
    /* 占空比限幅 */
    dutyA = CLAMP_DUTY(dutyA);    
    dutyB = CLAMP_DUTY(dutyB); 
    dutyC = CLAMP_DUTY(dutyC); 

    /* 更新三相占空比 */
    three_phase_duty_cycle_t  htim_dutys;                          /* 占空比配置模块  */
    htim_dutys.duty[0] = (uint32_t)(dutyA * FOC_COUNT_PERIOD) >> 15;    /* A相 */   
    htim_dutys.duty[1] = (uint32_t)(dutyB * FOC_COUNT_PERIOD) >> 15;    /* B相 */
    htim_dutys.duty[2] = (uint32_t)(dutyC * FOC_COUNT_PERIOD) >> 15;    /* C相 */
    pwm_write_dutys(htim_dutys.duty[0], htim_dutys.duty[1], htim_dutys.duty[2]);   /* 互补 + 比较值偏移死区 */
}

void pwm_start(void)            /* 启动 pwm 输出 */
{   
    R_GPT_OutputEnable(tim_u_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputEnable(tim_u_ctrl, GPT_IO_PIN_GTIOCB);
    R_GPT_OutputEnable(tim_v_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputEnable(tim_v_ctrl, GPT_IO_PIN_GTIOCB);
    R_GPT_OutputEnable(tim_w_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputEnable(tim_w_ctrl, GPT_IO_PIN_GTIOCB);
}

void pwm_stop(void)
{   
   /* 关闭三路 PWM 的 A/B 输出，共 6 路互补通道全部关断。
    * 注意：这里只是关闭引脚输出，GPT 计数器仍在运行，因此 ADC 触发仍然有效。*/
    R_GPT_OutputDisable(tim_u_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(tim_u_ctrl, GPT_IO_PIN_GTIOCB);
    R_GPT_OutputDisable(tim_v_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(tim_v_ctrl, GPT_IO_PIN_GTIOCB);
    R_GPT_OutputDisable(tim_w_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(tim_w_ctrl, GPT_IO_PIN_GTIOCB);
}