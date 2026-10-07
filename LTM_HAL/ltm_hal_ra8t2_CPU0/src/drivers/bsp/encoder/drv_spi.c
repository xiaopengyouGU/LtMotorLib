/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-10     chengbb      The first version
 * 2026-07-20     Lvtou        精简SPI实现，同步SPI即可（16M bit/s） 
 */
#include "drv_spi.h"

/* SPI传输结构体，volatile不能少，否则Release模式下，下文的等待循环会被优化掉 */
typedef struct{
    spi_b_instance_ctrl_t  * ctrl;  /* SPI 控制指针 */
    bsp_io_port_pin_t cs_pin;       /* CS 引脚 */
    volatile uint8_t  tx_cplt;      /* 发送完成标志位，0：未完成，1：已完成 */ 
}spi_config_t;

static spi_config_t g_spi_config[SPI_COUNT] = {
    [SPI_0] = { &g_spi0_ctrl, SPI0_CS_PIN, 0 },
    [SPI_1] = { &g_spi1_ctrl, SPI1_CS_PIN, 0 },
};

void spi_init(void)
{
    R_SPI_B_Open(&g_spi0_ctrl, &g_spi0_cfg);
    R_SPI_B_Open(&g_spi1_ctrl, &g_spi1_cfg);
    /* 重置接收标志位 */
    for(int i = 0; i < SPI_COUNT; i++){
        g_spi_config[i].tx_cplt = 0;
    }
}

// 绝对可靠的8个NOP（编译器永远不敢删）
#define BUS_RELEASE() \
    do { \
        volatile uint32_t _nop_count = 8; \
        while(_nop_count--) { \
            __asm volatile("NOP" : "+r"(_nop_count) : : "memory"); \
        } \
    } while(0)

/* spi 读写接口, 1：成功，0：失败 */
int spi_write_read(spi_id_t id, uint16_t *tx_buf, uint16_t *rx_buf, uint32_t n_words)
{   
    if(id >= SPI_COUNT)		                return 0;
    if(!tx_buf || !n_words || n_words > 8)  return 0;   /* 输入参数为空或大小超限制，直接返回 */
    uint16_t dummy[8];                                  /* 支持只写操作 */
    uint16_t *rx_use = (rx_buf != NULL) ? rx_buf : dummy;

    int rw_flag = 0;
    spi_config_t *config = &g_spi_config[id];          
    config->tx_cplt = 0;                            /* 先清零发送标志位 */
    /* SPI 传输开始 */
    bsp_io_port_pin_t cs_pin = config->cs_pin;
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs_pin, BSP_IO_LEVEL_LOW);     /* 拉低 CS 引脚 */
    fsp_err_t res = R_SPI_B_WriteRead(config->ctrl, tx_buf, rx_use, n_words, SPI_BIT_WIDTH_16_BITS);
    if(res == FSP_SUCCESS){
        volatile int count = 5000;                  /* 最大等待 5000次轮询 */
        while(!config->tx_cplt){                    /* 等待数据发送完毕 */
            if(count-- <= 0)     break;             /* 等待超时，退出 */
            /* 8个总线空闲指令，避免while循环中，因总线饥饿导致高优先级中断触发压栈失败 */
            BUS_RELEASE();                          /* 释放总线 */
        }
        rw_flag = (count > 0) ?  1 : 0;             /* 等待时间中，数据发送完毕（1）*/
    }
    R_IOPORT_PinWrite(&g_ioport_ctrl, cs_pin, BSP_IO_LEVEL_HIGH);    /* 释放 CS 引脚 */
    
    return rw_flag;
}

/* SPI 配置回调函数 */
void spi0_callback(spi_callback_args_t * p_args)
{   
    /* 电流环中断中调用同步SPI, 此时SPI发送完毕中断优先级最高 */
    switch(p_args->event)
    {
        case SPI_EVENT_TRANSFER_COMPLETE:
        {
            g_spi_config[SPI_0].tx_cplt = 1;    /* 标记发送完毕 */
            break;
        }
        default:    break;
    }
}

void spi1_callback(spi_callback_args_t * p_args)
{   
    /* 电流环中断中调用同步SPI, SPI发送完毕中断优先级最高 */
    switch(p_args->event)
    {
        case SPI_EVENT_TRANSFER_COMPLETE:
        {
            g_spi_config[SPI_1].tx_cplt = 1;    /* 标记发送完毕 */
            break;
        }
        default:    break;
    }
}