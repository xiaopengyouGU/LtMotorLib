/*
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-02     Lvtou        STM32G474 ADC：三相电流 ADC1 注入组并行采样 + 规则组母线/温度
 * 2026-09-02     Lvtou        驱动器温度：PA1(IN2) -> PA2(IN3)，PA1 释放
 * 2026-09-03     Lvtou        ADC1 注入触发 HRTIM_TRG1 -> HRTIM_TRG5：
 *                             G474 注入组触发映射中 TRG1 仅 ADC3/4/5 可用，
 *                             ADC1 注入与 ADC2 同用 TRG5（同一 TimerA 谷底事件）
 * 2026-09-03     Lvtou        修复规则组 rank：G4 的 ADC_REGULAR_RANK_x 为 LL 编码
 *                             （RANK_1/2/3 = 6/12/18...），不能 +i 递增，
 *                             否则 SQR 被写乱，VBUS/温度通道全部失效
 * 2026-09-19     Lvtou        修改ADC引脚和转换系数
 */
#include "bsp/adc/adc.h"
#include "bsp/encoder/drv_spi.h"
#include <string.h>

#define ADC_SEQ_LEN                 3U          /* 规则队列长度（VBUS、驱动器温度、电机温度）*/
#define ADC_INJ_LEN                 1U          /* 注入队列长度（每颗 ADC 一个电流通道）*/
#define ADC_CURRENT_SAMPLE_CYCLES   ADC_SAMPLETIME_12CYCLES_5   /* 电流：12.5 周期 */
#define ADC_TEMP_SAMPLE_CYCLES      ADC_SAMPLETIME_47CYCLES_5   /* 温度：47.5 周期（NTC 高阻）*/

/* 温度范围：10℃ ~ 90℃，2℃ 步进，共 41 点。*/
#define NTC_TABLE_LEN     41
#define NTC_TEMP_START   (10)
#define NTC_TEMP_STEP     2

/* 热敏电阻型号：NCP18XH103F03RB，10k电阻下拉分压 ：
 * 对应的 ADC 值（12位，0~4095），四舍五入取整 */
static const uint16_t ntc_raw_table[NTC_TABLE_LEN] = {
    1467, 1544, 1621, 1699, 1777, 1855, 1933, 2010,
    2086, 2162, 2236, 2310, 2382, 2452, 2520, 2587,
    2652, 2715, 2776, 2835, 2892, 2948, 3001, 3052,
    3101, 3147, 3192, 3234, 3274, 3313, 3350, 3385,
    3419, 3451, 3481, 3510, 3538, 3564, 3589, 3613,
    3635,
};

static ADC_HandleTypeDef g_adc1;                    /* ADC1：IA 注入 + 规则组母线/温度 DMA */
static ADC_HandleTypeDef g_adc2;                    /* ADC2：IB 注入 */
static ADC_HandleTypeDef g_adc3;                    /* ADC3：IC 注入 */
static DMA_HandleTypeDef g_dma_seq;

static volatile uint16_t s_seq_buff[ADC_SEQ_LEN];   /* 规则组 DMA 缓冲：母线/温度 */

static void (*s_callback)(void) = NULL;     /* 用户回调函数指针 */
static void _adc_pins_init(void);           /* ADC 引脚初始化 */
static void _adc_seq_init(void);            /* 规则组：母线/温度，DMA 连续搬运 */
static void _adc_inj_init(ADC_HandleTypeDef *adc, ADC_TypeDef *instance,
                          uint32_t channel,  uint32_t trig_inj,
                          uint32_t scan,     uint32_t nbr_conv,
                          uint32_t trig_reg, uint32_t dma_cont);  /* 注入 ADC 统一初始化 */
static int32_t _ntc_temp(uint16_t raw);     /* 查表法温度读取（0.1℃）*/

/* ADC 采样结构体（ISR 内/ 主循环读，必须 volatile：Release(-O3) 下编译器不会缓存跨函数共享变量）*/
typedef struct {
    int32_t  offset[3];             /* A、B、C 相零点偏置（原始值，int32）*/
    uint16_t curr_buf[3];           /* A、B、C 相电流采样（原始值）*/
    uint16_t temp_buf[3];           /* [0]: 母线电压, [1]: 驱动器温度, [2]: 电机温度 */
    uint16_t calib_tar;             /* 目标校准采样次数 */
    uint16_t calib_cnt;             /* 已校准采样次数 */
    uint8_t  calib_done;            /* 校准标志位：0：未校准、1：校准完毕 */
} adc_sample_t;

/* ISR 与主循环共享，volatile 防优化缓存；
 * 放 CCM（.ccm_noinit）：CPU 独占、DMA 摸不到，JEOS 内访问无需竞争 AHB 总线；
 * adc_init 会先 memset 清零，故 NOLOAD 段无需启动拷贝 */
static volatile adc_sample_t adc_sample __attribute__((section(".ccm_noinit")));
static volatile adc_sample_t *adc = &adc_sample;
void adc_init(void)
{
    memset(adc, 0, sizeof(adc_sample_t)); /* 清空采样结构体 */
    _adc_pins_init();                                     /* ADC 引脚初始化 */
    __HAL_RCC_ADC12_CLK_ENABLE();                         /* ADC1/ADC2 时钟 */
    __HAL_RCC_ADC345_CLK_ENABLE();                        /* ADC3/4/5 时钟 */

    /* 三颗 ADC 注入组完全同构：_adc_inj_init 统一完成
     * 基座配置（ADCCLK=42.5MHz/12bit/单端）+ 校准 + 注入单通道序列；
     * 差异仅 Instance、规则组形态（扫描/队列长/DMA/触发）与注入触发源 */
    _adc_inj_init(&g_adc1, ADC1, ADC_CURRENT_A_CHANNEL, ADC_EXTERNALTRIGINJEC_HRTIM_TRG5,
                  ADC_SCAN_ENABLE, ADC_SEQ_LEN, ADC_EXTERNALTRIG_HRTIM_TRG1, ENABLE);   /* 规则 VBUS/温度 DMA + 注入 IA */
    _adc_inj_init(&g_adc2, ADC2, ADC_CURRENT_B_CHANNEL, ADC_EXTERNALTRIGINJEC_HRTIM_TRG5,
                  ADC_SCAN_DISABLE, 1U, ADC_EXTERNALTRIG_HRTIM_TRG5, DISABLE);          /* 仅注入 IB */
    _adc_inj_init(&g_adc3, ADC3, ADC_CURRENT_C_CHANNEL, ADC_EXTERNALTRIGINJEC_HRTIM_TRG9,
                  ADC_SCAN_DISABLE, 1U, ADC_EXTERNALTRIG_HRTIM_TRG9, DISABLE);          /* 仅注入 IC（PB1）*/

    /* DMA：规则组结果连续搬运（循环，无需重装）*/
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    g_dma_seq.Instance                = DMA1_Channel1;
    g_dma_seq.Init.Request            = DMA_REQUEST_ADC1;
    g_dma_seq.Init.Direction          = DMA_PERIPH_TO_MEMORY;
    g_dma_seq.Init.PeriphInc          = DMA_PINC_DISABLE;
    g_dma_seq.Init.MemInc             = DMA_MINC_ENABLE;
    g_dma_seq.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    g_dma_seq.Init.MemDataAlignment   = DMA_MDATAALIGN_HALFWORD;
    g_dma_seq.Init.Mode               = DMA_CIRCULAR;
    g_dma_seq.Init.Priority           = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&g_dma_seq);
    __HAL_LINKDMA(&g_adc1, DMA_Handle, g_dma_seq);

    _adc_seq_init();     /* 规则组：VBUS/温度，DMA 循环搬运 */

    HAL_ADC_Start_DMA(&g_adc1, (uint32_t *)s_seq_buff, ADC_SEQ_LEN);
    HAL_ADCEx_InjectedStart(&g_adc1);                        /* 注入组就绪，等待 HRTIM 触发 */
    HAL_ADCEx_InjectedStart(&g_adc2);
    HAL_ADCEx_InjectedStart(&g_adc3);
    __HAL_ADC_ENABLE_IT(&g_adc1, ADC_IT_JEOS);               /* 注入序列完成中断（电流环），仅一个就足够 */
    HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);                 /* 工程内最高优先级 */
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

void adc_set_callback(void (*callback)(void))
{
    if (callback)   s_callback = callback;
}

void adc_calibrate_zero(uint16_t samples)         /* 校准零点（软件触发）*/
{
    memset(adc, 0, sizeof(adc_sample_t));         /* tar 在 done 之前置零，可消除竞态 bug */
    adc->calib_tar  = samples;                    /* 更新校准采样次数目标值 */
}

void adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic)
{
    if (!Ia || !Ib || !Ic)              return;   /* 判空 */
    uint16_t *curr_buf = adc->curr_buf;
    int32_t  *offset   = adc->offset;

    if (!adc->calib_done) {                       /* 校准完毕之前，获取的电流按 0 算 */
        *Ia = 0;
        *Ib = 0;
        *Ic = 0;
        return;
    }
    /* 三相各自扣零点、转 Q15，下桥臂采样所以取负：
     * 12bit 中位偏置，半量程 ±2048 LSB，左移 4 位正好映射到 ±32768（1.0 pu = 33.0A）*/
    int32_t ia = -(((int32_t)curr_buf[0] - offset[0]) << 4);
    int32_t ib = -(((int32_t)curr_buf[1] - offset[1]) << 4);
    int32_t ic = -(((int32_t)curr_buf[2] - offset[2]) << 4);
    /* 去共模：三颗 ADC 同步触发同步转换，硬件保证 Ia+Ib+Ic=0，残余项是温漂共模；
     * Q15 下用 *10923>>15 替代 /3，热路径不出现除法 */
    int32_t sum = ia + ib + ic;
    int32_t cm  = (sum * 10923) >> 15;
    *Ia = ia - cm;
    *Ib = ib - cm;
    *Ic = ic - cm;
}

void adc_get_vbus(int32_t *vbus)        /* 获取母线电压，Q15 标幺（1.0 = 满量程 98.79V）*/
{
    if (vbus)   *vbus = adc->temp_buf[0] << 3;      /* 12bit 0~4095 -> Q15 0~32760 */
}

void adc_get_temp(int32_t *motor_temp, int32_t *driver_temp)  /* 获取电机和驱动器温度（0.1℃）*/
{
    if (!motor_temp || !driver_temp)    return;
    uint16_t *temp_buf = adc->temp_buf;
    *driver_temp = _ntc_temp(temp_buf[1]);
    *motor_temp  = _ntc_temp(temp_buf[2]);                /* NTC 查表 */
}

/* ADC 注入序列完成中断（JEOS）：三相电流转换完毕即电流环触发点
 * 工程内最高优先级（1），保证电流环节拍时序确定：
 * 谷底触发后 ≈0.6µs 注入组转换完成，随即发起编码器 SPI 帧（≈9.0µs @5.31MHz）；
 * 电流环回调读 rx_buf 时新帧 DMA 尚未覆盖（数据字 rx_buf[1]/[2] 约 6µs/9µs 后才落地），
 * 恒为上一拍完整数据——这是一拍补偿的前提（详见 drv_spi.c 时序契约）。
 * 三颗 ADC 同步触发、同步转换（同 ADCCLK/同采样时间），
 * ADC1 JEOS 到达时 ADC2/ADC3 的 JDR1 同为当拍数据。
 */
void ADC1_2_IRQHandler(void)
{
    if (!(ADC1->ISR & ADC_ISR_JEOS))    return; /* 只启用了ADC1 注入完成中断，故只需清ADC1中断标志位 */    
    ADC1->ISR = ADC_ISR_JEOS;                   /* 先清中断标志，避免处理期间重入 */

    uint16_t *curr_buf = adc->curr_buf;

    /* 注入数据：三颗 ADC 各自 JDR1（同步触发，同一拍 A/B/C）*/
    curr_buf[0] = (uint16_t)(ADC1->JDR1 & 0xFFFF);
    curr_buf[1] = (uint16_t)(ADC2->JDR1 & 0xFFFF);
    curr_buf[2] = (uint16_t)(ADC3->JDR1 & 0xFFFF);

    /* 电流拷贝后立即发起编码器 SPI 读取（一拍一次）：帧锚定在谷底触发点，
     * 回调读取恒为上一拍完整数据——这是一拍补偿的前提（详见 drv_spi.c 时序契约）*/
    spi_kick_all();

    /* 校准完毕之前不进入电流环 */
    if (!adc->calib_done) {
        if (adc->calib_tar) {                   /* 调用了校准 API，手动启动校准 */
            int32_t *offset  = adc->offset;
            uint16_t count = (++(adc->calib_cnt));
            for (uint8_t i = 0; i < 3; i++) {
                offset[i] += curr_buf[i];
            }
            if (count >= adc->calib_tar) {
                for (uint8_t i = 0; i < 3; i++) {
                    offset[i] /= count;
                }
                adc->calib_done = 1;            /* 校准完毕 */
            }
        }
    } else if (s_callback) {                    /* 校准完毕，调用用户电流环回调 */
        s_callback();
    }

    /* 规则组 DMA 结果解析：队列顺序 = temp_buf 顺序（VBUS、驱动温度、电机温度）*/
    for (uint8_t i = 0; i < ADC_SEQ_LEN; i++)
        adc->temp_buf[i] = s_seq_buff[i];
}

/**************************************************************************************/
static void _adc_pins_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC0/PC1/PC3：IA/IB/VBUS */
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &gpio);

/* PB1：IC（ADC3_IN1，第三相采样引脚，原 Motor ERR 已挪至 PB6）*/
    gpio.Pin  = GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* PA0/PA2：电机温度/驱动器温度（PA1 释放）*/
    gpio.Pin  = GPIO_PIN_0 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOA, &gpio);
}

/* 规则组：母线/温度按队列顺序转换，DMA 连续搬运 */
static void _adc_seq_init(void)
{
    ADC_ChannelConfTypeDef ch = {0};
    /* 队列：母线电压 -> 驱动器温度 -> 电机温度 */
    uint32_t seq_ch[ADC_SEQ_LEN] = {
        ADC_VBUS_CHANNEL, ADC_TEMP_DRIVER_CHANNEL, ADC_TEMP_MOTOR_CHANNEL
    };
    /* rank 为 LL 编码值（RANK_1/2/3 = 6/12/18），不能用 RANK_1 + i 递增 */
    uint32_t seq_rank[ADC_SEQ_LEN] = {
        ADC_REGULAR_RANK_1, ADC_REGULAR_RANK_2, ADC_REGULAR_RANK_3
    };

    ch.SamplingTime = ADC_TEMP_SAMPLE_CYCLES;   /* NTC 分压源阻抗高需加长 */
    ch.SingleDiff   = ADC_SINGLE_ENDED;
    ch.OffsetNumber = ADC_OFFSET_NONE;
    ch.Offset       = 0;
    for (uint8_t i = 0; i < ADC_SEQ_LEN; i++) {
        ch.Channel = seq_ch[i];
        ch.Rank    = seq_rank[i];
        HAL_ADC_ConfigChannel(&g_adc1, &ch);
    }
}

/* 注入 ADC 统一初始化：基座配置 + 校准 + 注入单通道电流序列
 * 三颗 ADC 同构，差异仅 Instance/规则组形态/触发源：
 *   - 基座：ADCCLK=170/4=42.5MHz，12bit 右对齐，硬件触发单次扫描，溢出覆盖
 *   - 规则组：ADC1 扫描 3 通道（VBUS/温度）+ DMA 连续；ADC2/3 不使用
 *   - 注入组：单通道电流（JDR1），HRTIM TRGx 上升沿触发，JEOS 中断 = 电流环
 *     （G474 注入触发映射：TRG1 仅 ADC3/4/5；ADC1/ADC2 注入用 TRG5/TRG9 等）*/
static void _adc_inj_init(ADC_HandleTypeDef *adc, ADC_TypeDef *instance,
                          uint32_t channel, uint32_t trig_inj,
                          uint32_t scan, uint32_t nbr_conv,
                          uint32_t trig_reg, uint32_t dma_cont)
{
    ADC_InjectionConfTypeDef inj = {0};

    adc->Instance                    = instance;
    adc->Init.ClockPrescaler         = ADC_CLOCK_SYNC_PCLK_DIV4;  /* ADCCLK = 42.5MHz（三颗同频保证同步）*/
    adc->Init.Resolution             = ADC_RESOLUTION_12B;
    adc->Init.DataAlign              = ADC_DATAALIGN_RIGHT;
    adc->Init.ScanConvMode           = scan;
    adc->Init.EOCSelection           = ADC_EOC_SEQ_CONV;
    adc->Init.LowPowerAutoWait       = DISABLE;
    adc->Init.ContinuousConvMode     = DISABLE;          /* 硬件触发单次扫描 */
    adc->Init.NbrOfConversion        = nbr_conv;
    adc->Init.DiscontinuousConvMode  = DISABLE;
    adc->Init.NbrOfDiscConversion    = 0;
    adc->Init.ExternalTrigConv       = trig_reg;
    adc->Init.ExternalTrigConvEdge   = ADC_EXTERNALTRIGCONVEDGE_RISING;
    adc->Init.DMAContinuousRequests  = dma_cont;
    adc->Init.Overrun                = ADC_OVR_DATA_OVERWRITTEN;
    adc->Init.OversamplingMode       = DISABLE;
    HAL_ADC_Init(adc);
    HAL_ADCEx_Calibration_Start(adc, ADC_SINGLE_ENDED);

    /* 注入组：单通道电流（A/B/C 各一颗 ADC），JEOS 中断（一次触发转换完成）*/
    inj.InjectedNbrOfConversion       = ADC_INJ_LEN;
    inj.InjectedDiscontinuousConvMode = DISABLE;
    inj.AutoInjectedConv              = DISABLE;
    inj.QueueInjectedContext          = DISABLE;
    inj.ExternalTrigInjecConv         = trig_inj;                    /* HRTIM TRGx 触发 */
    inj.ExternalTrigInjecConvEdge     = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
    inj.InjectedSamplingTime          = ADC_CURRENT_SAMPLE_CYCLES;   /* 运放输出阻抗低 */
    inj.InjectedSingleDiff            = ADC_SINGLE_ENDED;
    inj.InjectedOffsetNumber          = ADC_OFFSET_NONE;
    inj.InjectedOffset                = 0;
    inj.InjectedChannel               = channel;
    inj.InjectedRank                  = ADC_INJECTED_RANK_1;
    HAL_ADCEx_InjectedConfigChannel(adc, &inj);
}

static int32_t _ntc_temp(uint16_t raw)               /* 查表法温度获取，返回 0.1℃ */
{
    /* 边界检查 */
    if (raw <= ntc_raw_table[0])                    return NTC_TEMP_START * 10;
    if (raw >= ntc_raw_table[NTC_TABLE_LEN - 1])    return (NTC_TEMP_START + (NTC_TABLE_LEN - 1) * NTC_TEMP_STEP) * 10;
    /* 二分定位：找到 raw 所在区间 [lo, hi]，lo < hi */
    int lo = 0, hi = NTC_TABLE_LEN - 1;
    while (hi - lo > 1) {    /* hi - lo > 1，确保退出时 lo < hi */
        int mid = (lo + hi) >> 1;
        if (raw >= ntc_raw_table[mid])
            lo = mid;
        else
            hi = mid;
    }
    /* 区间内线性插值，全程整数（0.1℃ 单位）*/
    int32_t span = (int32_t)ntc_raw_table[hi] - (int32_t)ntc_raw_table[lo];
    int32_t num  = ((int32_t)raw - (int32_t)ntc_raw_table[lo]) * (NTC_TEMP_STEP * 10);
    return (NTC_TEMP_START + lo * NTC_TEMP_STEP) * 10 + num / span;
}
