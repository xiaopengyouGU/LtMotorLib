#include "bsp/canfd/canfd.h"

static canfd_instance_ctrl_t * canfd_ctrl = &g_canfd0_ctrl;
static can_cfg_t             * canfd_cfg  = &g_canfd0_cfg;
static can_frame_t             tx_frame;        /* CAN-FD 发送帧 */
static can_frame_t             rx_frame;        /* CAN-FD 接收帧 */
static volatile uint8_t  canfd_tx_cplt = 1;     /* CAN-FD 发送完成标志 */    
static volatile uint8_t  canfd_rx_cplt = 0;     /* CAN-FD 接收完成标志 */

#define FILTER_MAX  16

typedef struct{
    uint16_t ids[FILTER_MAX];   /* 最多支持 16个 ID */
    uint8_t  count;             /* 当前白名单 ID 数 */
    uint8_t  enable;            /* 使能标志，1：使能，0：失能 */
} canfd_filter_t;               /* CAN-FD 软件滤波器 */

static  canfd_filter_t  canfd_filter;
static  canfd_filter_t * filter = &canfd_filter;

/* DLC → 实际长度 */
static const uint8_t dlc_to_len[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};
static void(*_canfd_rxcall)(void) = NULL;          /* CAN-FD 接收中断回调 */

static uint8_t _len_map_dlc(uint16_t *length);     /* 将数据长度映射到 DLC，并截断 */
static int     _filter_check(uint16_t id);         /* 白名单 ID 检查，0：未找到，1：找到 */
static void    _canfd_send(uint16_t id, uint8_t *buf, uint16_t length, uint32_t options);

const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
    {
        .id =
        {
            /* Specify the ID, ID type and frame type to accept. */
            .id         = 0x000,
            .frame_type = CAN_FRAME_TYPE_DATA,
            .id_mode    = CAN_ID_MODE_STANDARD
        },

        .mask =
        {
            /* These values mask which ID/mode bits to compare when filtering messages. */
            .mask_id         = 0x000,
            .mask_frame_type = 1,
            .mask_id_mode    = 1
        },

        .destination =
        {
            /* If DLC checking is enabled any messages shorter than the below setting will be rejected. */
            .minimum_dlc = CANFD_MINIMUM_DLC_0,

            /* Optionally specify a Receive Message Buffer (RX MB) to store accepted frames. RX MBs do not have an
             * interrupt or overwrite protection and must be checked with R_CANFD_InfoGet and R_CANFD_Read. */
            .rx_buffer   = CANFD_RX_MB_NONE,

            /* Specify which FIFO(s) to send filtered messages to. Multiple FIFOs can be OR'd together. */
            .fifo_select_flags = CANFD_RX_FIFO_0
        }
    },
};

void canfd_init(void)           /* CAN-FD 外设初始化 */
{
    R_CANFD_Open(canfd_ctrl, canfd_cfg);
    memset(filter, 0, sizeof(canfd_filter_t));      /* 清空滤波器 */
    canfd_tx_cplt = 1;                              
    canfd_rx_cplt = 0;
}

void canfd_send(uint16_t id, uint8_t *buf, uint16_t len)   /* CAN-FD 发送接口 */
{
    /* 标记 CAN-FD 帧 */
    _canfd_send(id, buf, len, (CANFD_FRAME_OPTION_ERROR | CANFD_FRAME_OPTION_BRS | CANFD_FRAME_OPTION_FD));
}

void canfd_send2(uint16_t id, uint8_t *buf, uint16_t len)    /* CAN 发送接口 */
{
    /* 标记 CAN 帧，CAN 只能发送最多 8字节的数据 */
    if(len > 8)          len = 8;                          
    _canfd_send(id, buf, len, 0);                                      
}

uint8_t canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len)   /* CAN-FD 接收接口 */
{
    if(!id || !buf || !len)            return 0;      /* 判空 */
    if(!canfd_rx_cplt)                 return 0;      /* 无新数据 */
    /* 接收数据拷贝 */
    *id  = rx_frame.id;
    *len = dlc_to_len[rx_frame.data_length_code];
    memcpy(buf, (uint8_t*)rx_frame.data, *len);
    canfd_rx_cplt = 0;                 /* 更新数据接收完成标志位 */

    return 1;
}

void canfd_set_rxcall(void (*callback)(void))       /* 设置接收回调 */
{
    if (callback) _canfd_rxcall = callback;
}

void canfd_filter_add(uint16_t id)     /* 添加 白名单ID */ 
{
    if (filter->count >= FILTER_MAX) return;

    for (int i = 0; i < filter->count; i++) {
        if (filter->ids[i] == id) return;
    }

    filter->ids[filter->count++] = id;
}

void canfd_filter_clear(void)
{
    filter->count = 0;
}

void canfd_filter_enable(void)
{
    filter->enable = 1;
}

void canfd_filter_disable(void)
{
    filter->enable = 0;
}


void canfd0_callback(can_callback_args_t * p_args)  /* CAN-FD 收发中断回调 */
{
    switch(p_args->event)
    {
        case CAN_EVENT_TX_COMPLETE:
        {
            canfd_tx_cplt = 1;                      /* 标记发送完毕 */
            break;
        }
        case CAN_EVENT_RX_COMPLETE:
        {
            if(!_filter_check(p_args->frame.id))    return;
            /* 仅当接收到白名单中 的ID后，才能调用接收回调 */
            memcpy(&rx_frame, &p_args->frame, sizeof(can_frame_t)); /* 帧数据拷贝 */
            canfd_rx_cplt = 1;                      /* 标记帧接收完毕 */
            if(_canfd_rxcall)   _canfd_rxcall();    /* 调用接收中断回调 */
            break;
        }
        default:    break;
    }
}

/************************************************************************/
static uint8_t _len_map_dlc(uint16_t *length)   /* 将数据长度映射到 DLC，并截断 */
{
    uint16_t len = *length;
    uint8_t dlc  = 0;
    if (len <= 8)   return (uint8_t)len;
    if (len > 64) {
        *length = 64;
        return 15;
    }
    
    /* 利用 CAN-FD DLC 编码规律：超过 8 字节时，DLC = 8 + (len - 8 + 3) / 4 */
    /* 9-64 字节映射到 DLC 9-15 */
    dlc = 8 + ((len - 8 + 3) >> 2);         /* 位运算，性能更好 */
    if (dlc > 15) dlc = 15;
    *length = (uint16_t)dlc_to_len[dlc];

    return dlc;
}

static int _filter_check(uint16_t id)       /* 白名单 ID 检查，0：未找到，1：找到 */
{
    if (!filter->enable)    return 1;
    if (!filter->count)     return 1;
    for (int i = 0; i < filter->count; i++) {
        if (filter->ids[i] == id) return 1;
    }

    return 0;
}

static void _canfd_send(uint16_t id, uint8_t *buf, uint16_t length, uint32_t options)
{
    if(!buf || !length)                 return;     /* 判空 */
    if(!canfd_tx_cplt)                  return;     /* 发送未完毕，直接退出 */

    uint16_t len = length;          
    uint8_t  dlc = _len_map_dlc(&len);               /* 数据长度调整 */
    /* 发送数据帧 */
    tx_frame.id      = id;                           
    tx_frame.id_mode = CAN_ID_MODE_STANDARD;         /* 采用标准帧 */
    tx_frame.type    = CAN_FRAME_TYPE_DATA;          /* 发送数据帧 */
    tx_frame.options = options;                      /* 发送帧选项：CAN或CAN-FD帧 */
    tx_frame.data_length_code = dlc;                 /* 获取数据对应的 DLC */
    /* 数据拷贝 */
    if(len > length){                                /* len 映射时，向上取整了，需手动填充多个 0字节 */
        memcpy(tx_frame.data, buf, length);
        for(uint16_t i = length; i < len; i++){
            tx_frame.data[i] = 0;
        } 
    }else{                                           /* 用户数据过长，强行截断 */
        memcpy(tx_frame.data, buf, len);
    }
    canfd_tx_cplt = 0;                               /* 更新发送标志位 */
    
    R_CANFD_Write(canfd_ctrl, CANFD_TX_MB_0, &tx_frame);
}