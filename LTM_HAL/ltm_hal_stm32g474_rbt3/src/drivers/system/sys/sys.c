/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 系统层：时钟/时基/1ms 回调/软复位
 *                             24MHz HSE -> PLL1 -> 170MHz，SysTick 1ms，DWT 微秒时基
 */
#include "system/sys/sys.h"

#include "stm32g4xx_hal.h"

#define SYS_SYSCLK_HZ       170000000UL     /* 系统主频 170MHz */
#define SYS_TICKS_PER_US    170U            /* 每微秒 DWT 计数（170MHz）*/

static void (*s_callback)(void) = NULL;
static volatile uint64_t s_ms_tick = 0;     /* 系统时基：ms */

/* 1ms 系统节拍中断（SysTick） */
void SysTick_Handler(void)
{
    HAL_IncTick();                          /* HAL 时基（HAL_Delay 依赖） */
    s_ms_tick++;
    if (s_callback) {
        s_callback();
    }
}

/* 时钟树：24MHz HSE -> PLL1（VCO 340MHz）-> 170MHz，APB1/APB2 = 170MHz
 * PLL：N=85，M=6，P=4，R=2，Q=8（FDCAN/ADC 内核时钟可取自 PLL 分频） */
static void _clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /* HSE 为 PLL 时钟源 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = 6;      /* 24MHz / 6 = 4MHz（PLL 输入；VCO = 4×85 = 340MHz 保持不变）*/
    osc.PLL.PLLN       = 85;
    osc.PLL.PLLP       = 4;
    osc.PLL.PLLQ       = 8;
    osc.PLL.PLLR       = 2;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        while (1) { }                       /* 时钟初始化失败：直接卡死 */
    }

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
        while (1) { }
    }

    /* STM32G4 Z 版本支持预取缓冲 */
    if (HAL_GetREVID() == 0x1001) {
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}

/* DWT 周期计数：微秒时基（SYSCLK 计数，170 tick = 1µs） */
static void _dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

void system_init(void)
{
    HAL_Init();                             /* HAL 库初始化（NVIC 分组等） */
    _clock_config();                        /* 24MHz HSE -> PLL1 -> 170MHz */
    /* 主频切换后重配 SysTick 1ms（HAL_Init 时还是默认 HSI） */
    HAL_SYSTICK_Config(SYS_SYSCLK_HZ / 1000U);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    _dwt_init();                            /* DWT 使能：微秒时基 */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);      /* 系统时基最低优先级 */
}

uint64_t system_get_tick(void) { return s_ms_tick; }
uint64_t system_get_ms(void)   { return s_ms_tick; }
uint64_t system_get_us(void)
{
    uint32_t cnt = DWT->CYCCNT;             /* 周期计数 */
    uint64_t ms  = s_ms_tick;

    /* 极端情况：读取时计数器恰好接近 1ms 翻转（下一拍 s_ms_tick 会自增） */
    if (cnt < 1000U) {
        ms++;
    }

    return ms * 1000ULL + (uint64_t)(cnt / SYS_TICKS_PER_US);
}

void system_set_callback(void (*callback)(void))
{
    s_callback = callback;
}

void system_reset(void)
{
    /* 软复位前手动关闭所有外设：内核复位不覆盖外设域，
     * 外设保留运行状态会导致 App 重新初始化冲突（与 HPM 版同思路） */
    __disable_irq();

    /* PWM：停 TIM1 计数器、关输出（MOE 清 0 -> 输出进入 idle 低电平） */
    TIM1->CR1  &= ~TIM_CR1_CEN;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER  = 0;
    TIM1->CCR1  = 0;
    TIM1->CCR2  = 0;
    TIM1->CCR3  = 0;

    /* ADC：禁中断（转换触发已随 PWM 停止） */
    ADC1->IER = 0;

    /* CAN-FD / UART：禁中断 */
    FDCAN1->IE = 0;
    USART3->CR1 = 0;

    /* DMA：停编码器 SPI 通道 */
    DMA1_Channel2->CCR = 0;
    DMA1_Channel3->CCR = 0;
    DMA1_Channel4->CCR = 0;
    DMA1_Channel5->CCR = 0;

    NVIC_SystemReset();                     /* 内核软复位 */
    while (1) { }
}
