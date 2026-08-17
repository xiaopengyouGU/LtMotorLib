/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-01-10     chengbb      The first version
 * 2026-07-20     Lvtou        精简SPI实现，同步SPI即可（16M bit/s） 
 */
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "hal_data.h"

/* SPI CS 引脚端口号 */
#define SPI0_CS_PIN                  BSP_IO_PORT_10_PIN_11
#define SPI1_CS_PIN                  BSP_IO_PORT_08_PIN_08        

typedef enum {
    SPI_0,
    SPI_1,
    SPI_COUNT
} spi_id_t;

/* 函数声明 */
void spi_init(void);       
/* spi 读写实现，返回值：1：成功，0：失败。rx_buf和tx_buf 为收发缓冲区 */
int spi_write_read(spi_id_t id, uint16_t *tx_buf, uint16_t *rx_buf, uint32_t n_words);

#endif