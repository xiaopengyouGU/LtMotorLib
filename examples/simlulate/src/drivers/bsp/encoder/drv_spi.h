/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-22     Lvtou        虚拟SPI外设实现 
 */
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include <stdint.h>
       
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