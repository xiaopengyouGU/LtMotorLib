#include "bsp/pwm/pwm.h"

/* 三相 PWM 模块 */
static three_phase_ctrl_t * htim_ctrl = &g_three_phase0_ctrl;   /* 三相 PWM 控制结构体 */
static three_phase_cfg_t  * htim_cfg  = &g_three_phase0_cfg;    /* 三相 PWM 配置结构体 */

static timer_ctrl_t         *tim_u_ctrl = &g_timer0_ctrl; 
static timer_ctrl_t         *tim_v_ctrl = &g_timer1_ctrl;
static timer_ctrl_t         *tim_w_ctrl = &g_timer2_ctrl;         

void pwm_init(void)     /* 三相 PWM 初始化 */
{
   /* 初始化阶段让 GPT 三相定时器先跑起来，保证 ADC 触发链路已经建立，
    * 但把三相输出先置于安全状态，避免功率级在上电初始化时对电机施加电压。*/
    R_GPT_THREE_PHASE_Open(htim_ctrl, htim_cfg);
    R_GPT_THREE_PHASE_Start(htim_ctrl);
    /* 设置三相占空比，所有下桥臂导通 */
    three_phase_duty_cycle_t  htim_dutys;            /* 占空比配置模块  */
    htim_dutys.duty[0] = (FOC_COUNT_PERIOD >> 1);    /* U相 */   
    htim_dutys.duty[1] = (FOC_COUNT_PERIOD >> 1);    /* V相 */
    htim_dutys.duty[2] = (FOC_COUNT_PERIOD >> 1);    /* W相 */
    R_GPT_THREE_PHASE_DutyCycleSet(htim_ctrl, &htim_dutys);
}

#define DUTY_MIN    0.0f
#define DUTY_MAX    0.95f            /* 下桥臂采样FOC，最高输出占空比 */

#define GET_MAX(duty)    ((duty) < DUTY_MAX ? (duty) : DUTY_MAX)
#define GET_MIN(duty)    ((duty) > DUTY_MIN ? (duty) : DUTY_MIN)
#define CLAMP_DUTY(duty) (1.0f - GET_MAX(GET_MIN(duty)))

void pwm_set_dutys(float dutyA, float dutyB, float dutyC)
{
    /* 占空比限幅 */
    dutyA = CLAMP_DUTY(dutyA);    
    dutyB = CLAMP_DUTY(dutyB); 
    dutyC = CLAMP_DUTY(dutyC); 
    
    /* 更新三相占空比 */
    three_phase_duty_cycle_t  htim_dutys;                          /* 占空比配置模块  */
    htim_dutys.duty[0] = (uint32_t)(dutyA * FOC_COUNT_PERIOD);     /* A相 */   
    htim_dutys.duty[1] = (uint32_t)(dutyB * FOC_COUNT_PERIOD);     /* B相 */
    htim_dutys.duty[2] = (uint32_t)(dutyC * FOC_COUNT_PERIOD);     /* C相 */
    R_GPT_THREE_PHASE_DutyCycleSet(htim_ctrl, &htim_dutys); 
}

void pwm_start(void)            /* 启动 pwm 输出 */
{   
    R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, FOC_ENABLE_PORT_PIN, BSP_IO_LEVEL_HIGH);
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
    R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, FOC_ENABLE_PORT_PIN, BSP_IO_LEVEL_LOW);
    R_GPT_OutputDisable(tim_u_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(tim_u_ctrl, GPT_IO_PIN_GTIOCB);
    R_GPT_OutputDisable(tim_v_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(tim_v_ctrl, GPT_IO_PIN_GTIOCB);
    R_GPT_OutputDisable(tim_w_ctrl, GPT_IO_PIN_GTIOCA);
    R_GPT_OutputDisable(tim_w_ctrl, GPT_IO_PIN_GTIOCB);
}