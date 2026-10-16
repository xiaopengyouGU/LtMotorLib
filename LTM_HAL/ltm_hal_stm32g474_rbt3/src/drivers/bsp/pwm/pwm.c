/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 三相 PWM（TIM1 中心对齐 + 互补输出 + 死区）
 * 2026-09-02     Lvtou        升级 HRTIM1：Timer A/C/D 中心对称（up-down）+ DTEN 互补死区；
 *                             ADC 触发由 HRTIM 内部事件直接产生（TimerA 谷底 RESET），
 *                             TRG1/TRG5/TRG9 直达 ADC1/ADC2/ADC3，无需外部 TRGO 连线；
 *                             高分辨率时钟 MUL16（DLL 校准，16×170MHz=2.72GHz，368ps 步进）
 * 2026-09-19     Lvtou        修改PWM引脚定义
 * 2026-10-16     Lvtou        载波 25kHz -> 20kHz（与电流环同频）：MUL16 在 20kHz 下 PER=68000 超 16bit，
 *                             高分辨率时钟降为 MUL8（1.36GHz）；占空比接口浮点 -> Q15 定点。
 *                             死区时基是 fHRTIM（与 MUL 无关），68 tick 仍为 0.4µs/边沿
 * 
 * 时序：
 *   - DLL 校准后使能高分辨率时钟：MUL8 倍频 -> fHRCK = 8×170MHz = 1.36GHz（735ps 步进）
 *   - 计数器 up-down（中心对称）：PER=34000 -> 20kHz（完整周期 2*PER，精确锁频，与电流环同频）
 *     （20kHz 下 MUL16 需要 PER=68000 超 16bit，故取 MUL8）
 *   - CMP1 = PER*(1-duty)：Set on CMP1 —— 同一比较事件按计数方向自动反相
 *     （上数到 CMP1 置高、下数到 CMP1 复位），单比较点即构成对称 PWM；
 *     高脉冲关于波峰对称，谷底(cnt=0)为 000 零矢量 -> ADC 采样点
 *   - CMP 预装载：占空比在谷底 update 一次性生效（一拍延迟，FOC 标准节奏）
 *   - 死区插入（DTEN）：输出2 由输出1 自动派生互补，上升/下降各 68tick = 0.4µs，合计死区 0.8µs
 *   - 互补对（AF13）：A 相 PA8/PA9（Timer A），B 相 PB14/PB15（Timer D），C 相 PB12/PB13（Timer C）
 */
#include "bsp/pwm/pwm.h"
#include "stm32g4xx_hal.h"

#define PWM_HRTIM             HRTIM1
#define PWM_DEAD_TICK         68U         /* 死区：68 tick @170MHz = 0.4µs/边沿，上升+下降合计 0.8µs（DIV1，1tick=5.88ns）*/
#define PWM_HR_PRESCALER      HRTIM_PRESCALERRATIO_MUL8   /* 高分辨率时钟：8×170MHz = 1.36GHz，步进 735ps（20kHz 所需）*/

#define PWM_DUTY_MIN          0
#define PWM_DUTY_MAX          31457       /* 0.96 × 32768：下桥臂采样 FOC，最高输出占空比 */
#define GET_MAX(duty)         ((duty) < PWM_DUTY_MAX ? (duty) : PWM_DUTY_MAX)
#define GET_MIN(duty)         ((duty) > PWM_DUTY_MIN ? (duty) : PWM_DUTY_MIN)
#define CLAMP_DUTY(duty)      (32768 - GET_MAX(GET_MIN(duty)))
#define PWM_CMP_MIN           24U         /* 比较器最小 3 个 fHRTIM 周期（MUL8：3×8 = 24 HR tick）*/

static HRTIM_HandleTypeDef g_hrtim;

/* 相序映射：dutyA -> Timer A（PA8/PA9），dutyB -> Timer D（PB14/PB15），dutyC -> Timer C（PB12/PB13）*/
static const uint32_t _tim_map[3] = {
    HRTIM_TIMERINDEX_TIMER_A,
    HRTIM_TIMERINDEX_TIMER_D,
    HRTIM_TIMERINDEX_TIMER_C,
};

/* 输出标识映射：输出1/输出2（与 _tim_map 同序：A/C/D 相）*/
static uint32_t _out1_of(uint32_t timer_idx)
{
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_C)  return HRTIM_OUTPUT_TC1;
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_D)  return HRTIM_OUTPUT_TD1;
    return HRTIM_OUTPUT_TA1;
}

static uint32_t _out2_of(uint32_t timer_idx)
{
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_C)  return HRTIM_OUTPUT_TC2;
    if (timer_idx == HRTIM_TIMERINDEX_TIMER_D)  return HRTIM_OUTPUT_TD2;
    return HRTIM_OUTPUT_TA2;
}

void HAL_HRTIM_MspInit(HRTIM_HandleTypeDef *hhrtim)
{
    if (hhrtim->Instance == PWM_HRTIM) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_HRTIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* A 相：PA8 = TA1（上桥）、PA9 = TA2（下桥）；
         * A/B 口 HRTIM1 复用为 AF13（AF3 仅用于 C 口）*/
        gpio.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF13_HRTIM1;
        HAL_GPIO_Init(GPIOA, &gpio);

        /* C 相：PB12 = TC1（上桥）、PB13 = TC2（下桥）；
         * B 相：PB14 = TD1（上桥）、PB15 = TD2（下桥），同为 AF13 */
        gpio.Pin       = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        gpio.Alternate = GPIO_AF13_HRTIM1;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

/* 单个子定时器：时基 + up-down + 预装载 + 死区插入 + 输出交叉开关（A/C/D 共用一套配置）*/
static void _hrtim_timer_config(uint32_t timer_idx)
{
    HRTIM_TimeBaseCfgTypeDef  tb  = {0};
    HRTIM_TimerCfgTypeDef     tc  = {0};
    HRTIM_TimerCtlTypeDef     ctl = {0};
    HRTIM_CompareCfgTypeDef   cmp = {0};
    HRTIM_OutputCfgTypeDef    out = {0};
    HRTIM_DeadTimeCfgTypeDef  dt  = {0};

    /* 时基：MUL8 高分辨率时钟（1.36GHz），PER=34000 -> 20kHz 对称 PWM（up-down 完整周期 = 2*PER）*/
    tb.Period            = FOC_COUNT_PERIOD;
    tb.RepetitionCounter = 0;
    tb.PrescalerRatio    = PWM_HR_PRESCALER;
    tb.Mode              = HRTIM_MODE_CONTINUOUS;
    HAL_HRTIM_TimeBaseConfig(&g_hrtim, timer_idx, &tb);

    /* up-down 计数：中心对称，谷底(cnt=0)为 000 零矢量（三相下桥全开，ADC 采样点）*/
    ctl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UPDOWN;
    HAL_HRTIM_WaveformTimerControl(&g_hrtim, timer_idx, &ctl);

    /* CMP 预装载：占空比在谷底 update 一次性生效；DTEN：输出2 自动派生互补 */
    tc.PreloadEnable     = HRTIM_PRELOAD_ENABLED;
    tc.ResetUpdate       = HRTIM_TIMUPDATEONRESET_ENABLED;  /* 谷底(cnt=0)复位事件触发预装载搬移 */
    tc.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    HAL_HRTIM_WaveformTimerConfig(&g_hrtim, timer_idx, &tc);

    /* CMP1 = PER（初始 0 占空比）；中心对称 PWM 仅需单比较点 CMP1 */
    cmp.CompareValue       = FOC_COUNT_PERIOD;
    cmp.AutoDelayedMode    = HRTIM_AUTODELAYEDMODE_REGULAR;
    cmp.AutoDelayedTimeout = 0;
    HAL_HRTIM_WaveformCompareConfig(&g_hrtim, timer_idx, HRTIM_COMPAREUNIT_1, &cmp);

    /* 输出1：Set on CMP1 —— RM0440 26.3.6：SET 源事件上数置高、下数自动复位。
     * ResetSource 必须为 NONE：若再挂 CMP2 Reset 会与 CMP1 Set 同刻竞争，
     * 把高脉冲顶到谷底（实际相位反 180°），AN4539 §12 三相整流同款写法 */
    out.Polarity              = HRTIM_OUTPUTPOLARITY_HIGH;
    out.SetSource             = HRTIM_OUTPUTSET_TIMCMP1;
    out.ResetSource           = HRTIM_OUTPUTRESET_NONE;
    out.IdleMode              = HRTIM_OUTPUTIDLEMODE_NONE;
    out.IdleLevel             = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    out.FaultLevel            = HRTIM_OUTPUTFAULTLEVEL_INACTIVE;
    out.ChopperModeEnable     = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    out.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    HAL_HRTIM_WaveformOutputConfig(&g_hrtim, timer_idx, _out1_of(timer_idx), &out);

    /* 输出2：DTEN 下由死区发生器从输出1 派生互补，Set/Reset 交叉开关被硬件旁路 */
    out.SetSource   = HRTIM_OUTPUTSET_NONE;
    out.ResetSource = HRTIM_OUTPUTRESET_NONE;
    HAL_HRTIM_WaveformOutputConfig(&g_hrtim, timer_idx, _out2_of(timer_idx), &out);

    /* 死区：以 fHRTIM 为基准（DIV1，1tick=5.88ns），上升/下降各 68 tick = 0.4µs，合计 0.8µs */
    dt.Prescaler       = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
    dt.RisingValue     = PWM_DEAD_TICK;
    dt.RisingSign      = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    dt.RisingLock      = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
    dt.RisingSignLock  = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
    dt.FallingValue    = PWM_DEAD_TICK;
    dt.FallingSign     = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    dt.FallingLock     = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
    dt.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
    HAL_HRTIM_DeadTimeConfig(&g_hrtim, timer_idx, &dt);
}

void pwm_init(void)                     /* 三相 PWM 初始化（HRTIM1）*/
{
    HRTIM_ADCTriggerCfgTypeDef trig = {0};

    __HAL_RCC_HRTIM1_CLK_ENABLE();

    g_hrtim.Instance = PWM_HRTIM;
    g_hrtim.Init.HRTIMInterruptResquests = 0;
    g_hrtim.Init.SyncOptions             = 0;
    HAL_HRTIM_Init(&g_hrtim);           /* 内部调用 MspInit：时钟 + 引脚 */

    /* DLL 校准：MUL 倍频时钟依赖延迟链，必须先校准（Start 锁句柄，Poll 释放）再配置/启动 */
    HAL_HRTIM_DLLCalibrationStart(&g_hrtim, HRTIM_SINGLE_CALIBRATION);
    HAL_HRTIM_PollForDLLCalibration(&g_hrtim, HAL_MAX_DELAY);

    for (uint8_t i = 0; i < 3; i++) {   /* 配置三相时钟 */
        _hrtim_timer_config(_tim_map[i]);
    }

    /* ADC 触发：TimerA RESET（谷底）经 HRTIM TRG1/TRG5/TRG9 直达 ADC1/ADC2/ADC3，
     * 三路并行同步采样，硬件锁定同一时刻，无需外部 TRGO 连线 */
    trig.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
    trig.Trigger      = HRTIM_ADCTRIGGEREVENT13_TIMERA_RESET;
    HAL_HRTIM_ADCTriggerConfig(&g_hrtim, HRTIM_ADCTRIGGER_1, &trig);
    trig.Trigger      = HRTIM_ADCTRIGGEREVENT579_TIMERA_RESET;
    HAL_HRTIM_ADCTriggerConfig(&g_hrtim, HRTIM_ADCTRIGGER_5, &trig);
    HAL_HRTIM_ADCTriggerConfig(&g_hrtim, HRTIM_ADCTRIGGER_9, &trig);

    /* 计数器先跑起来（ADC 触发链路依赖），默认关闭 PWM 输出。
     * 注意：__HAL_HRTIM_ENABLE 写的是 Master MCR 的 TACEN/TCCEN/TDCEN 位，
     * 必须用 HRTIM_TIMERID_*（=MCR 位掩码），不能用 HRTIM_TIMERINDEX_*（寄存器数组下标） */
    __HAL_HRTIM_ENABLE(&g_hrtim,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_C | HRTIM_TIMERID_TIMER_D);
    pwm_stop();
}

void pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC)
{
    int32_t dutys[3] = { dutyA, dutyB, dutyC };
    for (uint8_t i = 0; i < 3; i++) {
        /* 对称 PWM：CMP = PER*(1-duty)，占空比越高高脉冲越宽（关于波峰对称）。
         * Q15：先取反（32768-duty）再乘周期右移 15，热路径只有一次 32 位乘、无浮点。
         * 上界 32768×34000 = 1.11e9 < 2^31，int32 不溢出 */
        uint32_t cmp = (uint32_t)(CLAMP_DUTY(dutys[i]) * (int32_t)FOC_COUNT_PERIOD) >> 15;
        if (cmp < PWM_CMP_MIN) cmp = PWM_CMP_MIN;   /* 比较器最小 3 个 fHRTIM 周期 */
        __HAL_HRTIM_SETCOMPARE(&g_hrtim, _tim_map[i], HRTIM_COMPAREUNIT_1, cmp);
    }
}

void pwm_start(void)                    /* 启动 PWM 互补输出 */
{
    HAL_HRTIM_WaveformOutputStart(&g_hrtim,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 |
        HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
}

void pwm_stop(void)                     /* 停止 PWM 输出（输出强制 idle 低，占空比清零）*/
{
    HAL_HRTIM_WaveformOutputStop(&g_hrtim,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2 |
        HRTIM_OUTPUT_TD1 | HRTIM_OUTPUT_TD2);
    for (uint8_t i = 0; i < 3; i++) {
        __HAL_HRTIM_SETCOMPARE(&g_hrtim, _tim_map[i], HRTIM_COMPAREUNIT_1, FOC_COUNT_PERIOD);
    }
}
