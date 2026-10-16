/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 双 MT6835 SPI 编码器快照读取（SPI1 + SPI3）
 *
 * 时序契约（一拍补偿）：
 *   - 每 PWM 周期一次短事务（5×16bit @5.31MHz，≈9.0µs），其余时间总线完全空闲
 *   - 事务由 ADC JEOS 中断在电流拷贝后统一发起（spi_kick_all），非阻塞 DMA
 * 2026-09-02     Lvtou        SPI 降速至 5.31MHz（DIV32）留裕量，事务 ~9.0µs；
 * 2026-09-02     Lvtou        单轴版：仅主轴（SPI1/ENC1），从轴（SPI3）暂不使用；
 * 2026-09-19     Lvtou        恢复双轴读取：ENC1=SPI1，ENC2=SPI3（PB3_SCK / PB4_MISO /
 *                             PB5_MOSI，PA15_NSS 硬件 CS）；两路同一拍并发发起，
 *                             统一由 DMA1 承载（CH2/3=SPI1，CH4/5=SPI3）
 */
#include "bsp/encoder/drv_spi.h"
#include "stm32g4xx_hal.h"
/* LL Init 结构体/接口需要该宏（本文件自包含，不依赖工程编译选项）*/
#define USE_FULL_LL_DRIVER
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_dma.h"

#define ENC_TX_WORDS        3U          /* 命令 1 字 + 2 字空发送 */
#define ENC_RX_WORDS        3U
#define SPI_CS_HIGH_HOLD_TICKS  56U     /* tCSH：CS 高电平保持 ≥300ns（170MHz，85 tick）*/

/* SPI 运行上下文（每个 SPI 独立）*/
typedef struct {
    SPI_TypeDef        *spi;                    /* SPI 外设基址 */
    uint32_t            dma_ch_tx;              /* TX DMA 通道号（LL）*/
    uint32_t            dma_ch_rx;              /* RX DMA 通道号（LL）*/
    uint32_t            dma_req_tx;             /* DMAMUX TX 请求源 */
    uint32_t            dma_req_rx;             /* DMAMUX RX 请求源 */
    uint16_t            tx_buf[ENC_TX_WORDS];   /* 固定读命令帧 */
    volatile uint16_t   rx_buf[ENC_RX_WORDS];   /* RX DMA 直写：上一拍完整数据 */
} spi_context_t;

/* 资源表：
 *   SPI_0 = SPI3（ENC2/从轴：PB3_SCK / PB4_MISO / PB5_MOSI，PA15_NSS 硬件 CS）
 *   SPI_1 = SPI1（ENC1/主轴：PA5_SCK / PA6_MISO / PA7_MOSI，PA4_NSS 硬件 CS）
 * DMA1 通道分配：CH1=ADC，CH2/CH3=SPI1(ENC1)，CH4/CH5=SPI3(ENC2)*/
static spi_context_t g_spi_ctx[SPI_COUNT] = {
    [SPI_0] = { .spi = SPI3,
                .dma_ch_tx = LL_DMA_CHANNEL_4, .dma_ch_rx = LL_DMA_CHANNEL_5,
                .dma_req_tx = LL_DMAMUX_REQ_SPI3_TX, .dma_req_rx = LL_DMAMUX_REQ_SPI3_RX },
    [SPI_1] = { .spi = SPI1,
                .dma_ch_tx = LL_DMA_CHANNEL_2, .dma_ch_rx = LL_DMA_CHANNEL_3,
                .dma_req_tx = LL_DMAMUX_REQ_SPI1_TX, .dma_req_rx = LL_DMAMUX_REQ_SPI1_RX },
};

/* ====================================================================
 * 静态函数声明
 * ==================================================================== */
static void _spi_pins_init(spi_id_t id);
static void _spi_hw_init(spi_id_t id);
static void _dma_config(spi_id_t id);
static void _spi_kick_prep(spi_id_t id);    /* SPE=0 拉高 CS + 重装 DMA */
static void _spi_kick_go(spi_id_t id);      /* 使能 DMA + SPE=1 拉低 CS */

/* ====================================================================
 * 公共初始化流程
 * ==================================================================== */
void spi_init(void)
{
    /* 外设时钟：SPI1 在 APB2、SPI3 在 APB1，本工程 APB1/APB2 同为 170MHz，
     * 所以两路 DIV32 都得到 5.31MHz，无需分别配置 */
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();               /* SPI1: PA4~PA7；SPI3: PA15_NSS */
    __HAL_RCC_GPIOB_CLK_ENABLE();               /* SPI3: PB3/PB4/PB5 */

    for (spi_id_t id = 0; id < SPI_COUNT; id++) {
        spi_context_t *ctx = &g_spi_ctx[id];
        /* 预置 TX 命令：0xA030 连续读（CSN↓ 锁存 0x003~0x006）+ 4 空字
         * （8 字节数据流，覆盖任意相位偏移下的完整角度组）*/
        ctx->tx_buf[0] = 0xA030;      /* SPI 连续读指令 */
        for (uint8_t i = 1; i < ENC_TX_WORDS; i++)
            ctx->tx_buf[i] = 0xFFFF;  /* 空发送（仅产生读时钟） */
        _spi_pins_init(id);                         /* 1. 引脚初始化 */
        _spi_hw_init(id);                           /* 2. SPI 硬件初始化 */
        _dma_config(id);                            /* 3. DMA 通道绑定 */
    }
}

/* ====================================================================
 * SPI 读取接口（快照读取，零等待）
 * 返回值：1：成功，0：失败。tx_buf 仅为接口兼容保留（命令已预置）。
 * 纯快照读取：kick 由 ADC JEOS 中断统一发起（spi_kick_all），一拍一次。
 * 读取时机由 JEOS 保证：kick 在电流拷贝后、回调之前发起，
 * 本函数在回调内读取，rx_buf 恒为上一拍完整数据（一拍补偿）。
 * ==================================================================== */
int spi_write_read(spi_id_t id, uint16_t *tx_buf, uint16_t *rx_buf, uint32_t n_words)
{
    if (id >= SPI_COUNT)                                return 0;
    if (!rx_buf || !n_words || n_words > ENC_RX_WORDS)  return 0;
    (void)tx_buf;   /* TX 已预置，无需每次打包 */

    spi_context_t *ctx = &g_spi_ctx[id];
    /* RX DMA 直写 rx_buf（上一拍完整数据）：rx[0]=命令期间回显（丢弃），
     * rx[1..4] 为 8 字节角度流（0x003~0x006 循环，可能整体偏移），
     * 由上层对 rx[1..4] 做 CRC 扫描定位完整角度组。
     * @5.31MHz 下新帧约 6µs 后才落地，回调起始读取远早于此，
     * 恒为上一拍完整数据 */
    for (uint32_t i = 0; i < n_words; i++) {
        rx_buf[i] = ctx->rx_buf[i];
    }
    return 1;
}

/* ====================================================================
 * 统一发起两路编码器 SPI 短事务（ADC JEOS 电流拷贝后、回调前调用，一拍一次）
 * 帧锚定在谷底触发点，回调读取恒为上一拍完整数据——一拍补偿的前提
 * CS 高电平（tCSH）：SPE=0 后等待 ≥300ns 再 SPE=1，
 * 满足 MT6835 两次读取间隔要求，JEOS 仅多 ~0.5µs
 * ==================================================================== */
void spi_kick_all(void)
{
    for (spi_id_t id = 0; id < SPI_COUNT; id++) {
        _spi_kick_prep(id);                 /* SPE=0：NSS 拉高 + 重装 DMA */
    }

    /* 两路共用一次等待（DWT 周期计数，170 tick/µs）
     * 注意：CYCCNT 为 32 位自由计数器，无重装载寄存器，~25.3s（2^32/170MHz）回绕一次。
     * 无符号差值写法在回绕点依然正确（跨度 < 2^31 tick 即安全，500ns 远小于此），
     * 切勿改成有符号比较或绝对值形式，否则回绕处会算错 */
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < SPI_CS_HIGH_HOLD_TICKS) { ; }

    for (spi_id_t id = 0; id < SPI_COUNT; id++) {
        _spi_kick_go(id);                   /* SPE=1：NSS 拉低，DMA 立即启动事务 */
    }
}

/* ====================================================================
 * 底层：SPE=0 拉高 CS 并重装 DMA（正常模式，CNDTR 已消耗；通道必须先禁用再写）
 * ==================================================================== */
static void _spi_kick_prep(spi_id_t id)
{
    spi_context_t *ctx = &g_spi_ctx[id];

    LL_SPI_Disable(ctx->spi);                       /* SPE=0：NSS 拉高，结束上一帧 */

    LL_DMA_DisableChannel(DMA1, ctx->dma_ch_tx);
    LL_DMA_DisableChannel(DMA1, ctx->dma_ch_rx);
    LL_DMA_SetDataLength(DMA1, ctx->dma_ch_tx, ENC_TX_WORDS);
    LL_DMA_SetDataLength(DMA1, ctx->dma_ch_rx, ENC_RX_WORDS);
}

/* ====================================================================
 * 底层：使能 DMA 通道并 SPE=1 拉低 CS，DMA 立即启动事务（约 9.0µs @5.31MHz）
 * ==================================================================== */
static void _spi_kick_go(spi_id_t id)
{
    spi_context_t *ctx = &g_spi_ctx[id];

    LL_DMA_EnableChannel(DMA1, ctx->dma_ch_tx);
    LL_DMA_EnableChannel(DMA1, ctx->dma_ch_rx);
    LL_SPI_EnableDMAReq_TX(ctx->spi);
    LL_SPI_EnableDMAReq_RX(ctx->spi);
    LL_SPI_Enable(ctx->spi);                        /* SPE=1：NSS 拉低，事务开始 */
}

/* ====================================================================
 * 底层：引脚初始化（SCK/MISO/MOSI/NSS 全部走复用，CS 由硬件 NSS 管理）
 * ==================================================================== */
static void _spi_pins_init(spi_id_t id)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_AF_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;

    if (id == SPI_1) {
        /* ENC1/主轴：PA4_NSS / PA5_SCK / PA6_MISO / PA7_MOSI（AF5）*/
        gpio.Pin       = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &gpio);
    } else {
        /* ENC2/从轴：PA15_NSS（GPIOA，AF6）+ PB3_SCK / PB4_MISO / PB5_MOSI（AF6）
         * 注意：PA15/PB3/PB4 复位后是 JTAG 引脚（JTDI/JTDO/NJTRST），改成 AF6 后
         * JTAG 不可用，调试只能用 SWD（PA13/PA14），烧录不受影响 */
        gpio.Pin       = GPIO_PIN_15;
        gpio.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOA, &gpio);

        gpio.Pin       = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

/* ====================================================================
 * 底层：SPI 硬件初始化（LL 驱动，硬件 NSS 输出）
 * ==================================================================== */
static void _spi_hw_init(spi_id_t id)
{
    spi_context_t *ctx = &g_spi_ctx[id];
    LL_SPI_InitTypeDef spi_init = {0};

    spi_init.TransferDirection = LL_SPI_FULL_DUPLEX;
    spi_init.Mode              = LL_SPI_MODE_MASTER;
    spi_init.DataWidth         = LL_SPI_DATAWIDTH_16BIT;
    spi_init.ClockPolarity     = LL_SPI_POLARITY_HIGH;      /* MT6835 模式 3：CPOL=1（SCK 空闲高）*/
    spi_init.ClockPhase        = LL_SPI_PHASE_2EDGE;        /* CPHA=1（第二个边沿采样）*/
    spi_init.NSS               = LL_SPI_NSS_HARD_OUTPUT;    /* 硬件 NSS：SPE=1 拉低，SPE=0 拉高 */
    spi_init.BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV32; /* 170MHz/32 ≈ 5.31MHz（降速留裕量）*/
    spi_init.BitOrder          = LL_SPI_MSB_FIRST;
    spi_init.CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE;
    spi_init.CRCPoly           = 7;
    LL_SPI_Init(ctx->spi, &spi_init);
}

/* ====================================================================
 * 底层：DMA 通道配置（TX/RX 正常模式，RX 不使能中断，直写 rx_buf）
 * ==================================================================== */
static void _dma_config(spi_id_t id)
{
    spi_context_t *ctx = &g_spi_ctx[id];
    LL_DMA_InitTypeDef dma_init = {0};

    /* TX：内存 tx_buf → SPI DR */
    dma_init.PeriphOrM2MSrcAddress  = LL_SPI_DMA_GetRegAddr(ctx->spi);
    dma_init.MemoryOrM2MDstAddress  = (uint32_t)ctx->tx_buf;
    dma_init.Direction              = LL_DMA_DIRECTION_MEMORY_TO_PERIPH;
    dma_init.Mode                   = LL_DMA_MODE_NORMAL;
    dma_init.PeriphOrM2MSrcIncMode  = LL_DMA_PERIPH_NOINCREMENT;
    dma_init.MemoryOrM2MDstIncMode  = LL_DMA_MEMORY_INCREMENT;
    dma_init.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD;
    dma_init.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD;
    dma_init.NbData                 = ENC_TX_WORDS;
    dma_init.PeriphRequest          = ctx->dma_req_tx;
    dma_init.Priority               = LL_DMA_PRIORITY_LOW;
    LL_DMA_Init(DMA1, ctx->dma_ch_tx, &dma_init);

    /* RX：SPI DR → 内存 rx_buf（直写，无完成中断）*/
    dma_init.PeriphOrM2MSrcAddress  = LL_SPI_DMA_GetRegAddr(ctx->spi);
    dma_init.MemoryOrM2MDstAddress  = (uint32_t)ctx->rx_buf;
    dma_init.Direction              = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
    dma_init.Mode                   = LL_DMA_MODE_NORMAL;
    dma_init.NbData                 = ENC_RX_WORDS;
    dma_init.PeriphRequest          = ctx->dma_req_rx;
    dma_init.Priority               = LL_DMA_PRIORITY_MEDIUM;
    LL_DMA_Init(DMA1, ctx->dma_ch_rx, &dma_init);
}