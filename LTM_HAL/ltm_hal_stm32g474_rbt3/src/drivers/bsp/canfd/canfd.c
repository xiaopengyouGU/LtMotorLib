/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 CAN-FD（FDCAN1，PA11_RX/PA12_TX）
 *                             仲裁段 1Mbps + 数据段 2Mbps（内核时钟 PCLK1 = 170MHz）
 * 2026-09-02     Lvtou        CAN-FD 引脚调整：FDCAN1 -> PB8_RX/PB9_TX（原 PA11/PA12）
 */
#include "bsp/canfd/canfd.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#define CAN_DEV             FDCAN1

#define FILTER_MAX          16
#define RX_RING_SIZE        16

/* CAN-FD 接收帧 */
typedef struct {
    uint16_t id;            /* 标准帧 ID */
    uint8_t  len;           /* 数据长度（字节） */
    uint8_t  canfd_frame;   /* 帧类型：1：CAN-FD 帧，0：经典 CAN 帧 */
    uint8_t  data[64];      /* 数据 */
} canfd_frame_t;

/* 接收环形缓冲 */
typedef struct {
    canfd_frame_t     frame[RX_RING_SIZE];  /* 帧缓冲 */
    volatile uint16_t head;                 /* 写指针 */
    volatile uint16_t tail;                 /* 读指针 */
} canfd_ring_t;

/* 接收环形缓冲：CPU 独占，放 CCM（.ccm_noinit）避免 FDCAN ISR 与 DMA 争 AHB；
 * canfd_init 会 memset 清零，NOLOAD 段无需启动拷贝 */
static canfd_ring_t canfd_ring __attribute__((section(".ccm_noinit")));

/* CAN-FD 软件滤波器（白名单） */
typedef struct {
    uint16_t ids[FILTER_MAX];               /* 最多支持 16个 ID */
    uint8_t  count;                         /* 当前白名单 ID 数 */
    uint8_t  enable;                        /* 使能标志，1：使能，0：失能 */
} canfd_filter_t;

/* 白名单过滤器：CPU 独占，同样进 CCM（canfd_init memset 清零）*/
static  canfd_filter_t  canfd_filter __attribute__((section(".ccm_noinit")));
static  canfd_filter_t * filter = &canfd_filter;

/* DLC -> 实际长度 */
static const uint8_t dlc_to_len[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

static FDCAN_HandleTypeDef g_fdcan;
static void(*_canfd_rxcall)(void) = NULL;   /* CAN-FD 接收中断回调 */
static uint8_t _len_map_dlc(uint16_t *length);  /* 将数据长度映射到 DLC，并截断 */
static int  _filter_check(uint16_t id);     /* 白名单 ID 检查，0：未找到，1：找到 */
static void _canfd_send(uint16_t id, uint8_t *buf, uint16_t len, uint8_t canfd_frame);

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan->Instance == CAN_DEV) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_FDCAN_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;     /* PA11=RX，PA12=TX */
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF9_FDCAN1;
        HAL_GPIO_Init(GPIOA, &gpio);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    }
}

void canfd_init(void)                       /* CAN-FD 外设初始化 */
{
    memset(&canfd_ring, 0, sizeof(canfd_ring_t));   /* 清空接收环形缓冲 */
    memset(filter, 0, sizeof(canfd_filter_t));      /* 清空滤波器 */

    /* FDCAN 内核时钟 = PCLK1（170MHz）：
     * 仲裁 1Mbps：预分频 10 -> 17MHz tq，17tq/bit（Sync1+Seg1 12+Seg2 4，采样点 76.5%）
     * 数据 2Mbps：预分频 5  -> 34MHz tq，17tq/bit */
    RCC_PeriphCLKInitTypeDef periph = {0};
    periph.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    periph.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&periph);

    g_fdcan.Instance                   = CAN_DEV;
    g_fdcan.Init.ClockDivider          = FDCAN_CLOCK_DIV1;
    g_fdcan.Init.FrameFormat           = FDCAN_FRAME_FD_BRS;   /* FD 帧开 BRS */
    g_fdcan.Init.Mode                  = FDCAN_MODE_NORMAL;
    g_fdcan.Init.AutoRetransmission    = ENABLE;
    g_fdcan.Init.TransmitPause         = DISABLE;
    g_fdcan.Init.ProtocolException     = DISABLE;
    g_fdcan.Init.NominalPrescaler      = 10;
    g_fdcan.Init.NominalSyncJumpWidth  = 4;
    g_fdcan.Init.NominalTimeSeg1       = 12;
    g_fdcan.Init.NominalTimeSeg2       = 4;
    g_fdcan.Init.DataPrescaler         = 5;
    g_fdcan.Init.DataSyncJumpWidth     = 4;
    g_fdcan.Init.DataTimeSeg1          = 12;
    g_fdcan.Init.DataTimeSeg2          = 4;
    g_fdcan.Init.StdFiltersNbr         = 1;
    g_fdcan.Init.ExtFiltersNbr         = 0;
    g_fdcan.Init.TxFifoQueueMode       = FDCAN_TX_FIFO_OPERATION;
    HAL_FDCAN_Init(&g_fdcan);

    /* 默认放行所有帧（白名单未使能时全收；使能后在中断内过滤，非白名单帧不占环形缓冲） */
    HAL_FDCAN_ConfigGlobalFilter(&g_fdcan, FDCAN_REJECT, FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    /* 标准帧过滤表（1 个元素，StdFiltersNbr=1）：掩码全 0 = 全部标准帧进 FIFO0。
     * 全局过滤保持 REJECT 兜底：只有命中此表的标准帧能进 FIFO0，扩展帧/远程帧不进；
     * 软件白名单（canfd_filter_*）在中断内二次过滤，此处只管硬件放行 */
    FDCAN_FilterTypeDef flt = {0};
    flt.IdType       = FDCAN_STANDARD_ID;
    flt.FilterIndex  = 0;
    flt.FilterType   = FDCAN_FILTER_MASK;
    flt.FilterID1    = 0x000;                /* 过滤值：无关（全收）*/
    flt.FilterID2    = 0x000;                /* 掩码：0 = 该位不参与匹配 */
    flt.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    HAL_FDCAN_ConfigFilter(&g_fdcan, &flt);
    HAL_FDCAN_ConfigRxFifoOverwrite(&g_fdcan, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&g_fdcan);
    HAL_FDCAN_ActivateNotification(&g_fdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void FDCAN1_IT0_IRQHandler(void)        /* CAN-FD 收发中断 */
{
    HAL_FDCAN_IRQHandler(&g_fdcan);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[64];
    (void)RxFifo0ITs;
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
        uint16_t id = (uint16_t)header.Identifier;
        if (!_filter_check(id))         continue;   /* 白名单过滤：非白名单 ID 直接丢弃，不占环形缓冲 */
        if (canfd_ring.head - canfd_ring.tail < RX_RING_SIZE) {  /* 环形缓冲未满 */
            canfd_frame_t *frame = &canfd_ring.frame[canfd_ring.head % RX_RING_SIZE];
            uint8_t len        = dlc_to_len[(header.DataLength >> 16) & 0x0F];   /* DLC -> 实际数据长度 */
            frame->id          = id;                     /* 标准帧 ID */
            frame->len         = len;
            frame->canfd_frame = (header.FDFormat == FDCAN_FD_CAN) ? 1 : 0;      /* 帧类型 */
            memcpy(frame->data, data, len);
            canfd_ring.head++;                          /* 写指针前进 */
        }
    }
    if (_canfd_rxcall)  _canfd_rxcall();                /* 调用接收中断回调 */
}

void canfd_send(uint16_t id, uint8_t *buf, uint16_t len)   /* CAN-FD 发送接口 */
{
    _canfd_send(id, buf, len, 1);           /* CAN-FD 帧（BRS） */
}

void can_send(uint16_t id, uint8_t *buf, uint16_t len)     /* CAN 发送接口 */
{
    _canfd_send(id, buf, len, 0);           /* 经典 CAN 帧 */
}

/* 返回 1：有CAN-FD帧，2：有CAN帧，0：无 */
uint8_t canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len)   /* CAN-FD 接收接口 */
{
    if (!id || !buf || !len)                return 0;   /* 判空 */
    if (canfd_ring.tail == canfd_ring.head) return 0;   /* 无新数据 */

    /* 环形缓冲中的帧已在中断内做过白名单过滤，这里直接取出 */
    canfd_frame_t *frame = &canfd_ring.frame[canfd_ring.tail % RX_RING_SIZE];

    *id  = frame->id;                                   /* 帧 ID */
    *len = frame->len;                                  /* 数据长度 */
    memcpy(buf, frame->data, frame->len);               /* 数据拷贝 */
    canfd_ring.tail++;                                  /* 读指针前进 */

    return frame->canfd_frame ? 1 : 2;                  /* 1：CAN-FD 帧，2：CAN 帧 */
}

void canfd_set_rxcall(void (*callback)(void))           /* 设置接收回调 */
{
    if (callback)   _canfd_rxcall = callback;
}

void canfd_filter_add(uint16_t id)                      /* 添加白名单 ID */
{
    if (filter->count >= FILTER_MAX)    return;
    for (int i = 0; i < filter->count; i++) {
        if (filter->ids[i] == id)       return;         /* 已存在 */
    }
    filter->ids[filter->count++] = id;
}

void canfd_filter_clear(void)                           /* 清空白名单 */
{
    filter->count = 0;
}

void canfd_filter_enable(void)                          /* 使能白名单 */
{
    filter->enable = 1;
}

void canfd_filter_disable(void)                         /* 失能白名单 */
{
    filter->enable = 0;
}

/************************************************************************/
static uint8_t _len_map_dlc(uint16_t *length)   /* 将数据长度映射到 DLC，并截断 */
{
    uint16_t len = *length;
    uint8_t dlc  = 0;
    if (len <= 8)   return (uint8_t)len;
    /* 利用 CAN-FD DLC 编码规律：超过 8 字节时，DLC = 8 + (len - 8 + 3) / 4 */
    /* 9-64 字节映射到 DLC 9-15 */
    dlc = 8 + ((len - 8 + 3) >> 2);         /* 位运算，性能更好 */
    if (dlc > 15) dlc = 15;
    *length = (uint16_t)dlc_to_len[dlc];

    return dlc;
}

static int _filter_check(uint16_t id)                   /* 白名单 ID 检查，0：未找到，1：找到 */
{
    if (!filter->enable)            return 1;
    if (!filter->count)             return 1;
    for (int i = 0; i < filter->count; i++) {
        if (filter->ids[i] == id)   return 1;
    }
    return 0;
}

static void _canfd_send(uint16_t id, uint8_t *buf, uint16_t len, uint8_t canfd_frame)
{
    if (!buf || !len || len > 64)       return;         /* 判空 */

    uint16_t len_map = len;
    uint8_t  dlc     = _len_map_dlc(&len_map);          /* 字节长度 -> DLC（向上取整） */
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier         = id;
    header.IdType             = FDCAN_STANDARD_ID;
    header.TxFrameType        = FDCAN_DATA_FRAME;
    header.DataLength         = (uint32_t)dlc << 16;    /* DLC 编码（与 FDCAN_DLC_BYTES_x 一致） */
    header.FDFormat           = canfd_frame ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;    /* 帧类型 */
    header.BitRateSwitch      = canfd_frame ? FDCAN_BRS_ON : FDCAN_BRS_OFF;         /* FD 帧才用 BRS */
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    HAL_FDCAN_AddMessageToTxFifoQ(&g_fdcan, &header, buf);
}
