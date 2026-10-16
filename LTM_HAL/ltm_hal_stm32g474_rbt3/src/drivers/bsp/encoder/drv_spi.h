#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include <stdint.h>

/* 双编码器 SPI 通道（MT6835 单圈绝对值编码器，21bit）
 *   SPI_0 = SPI3（ENC2/从轴：PB3_SCK / PB4_MISO / PB5_MOSI，PA15_NSS 硬件 CS）
 *   SPI_1 = SPI1（ENC1/主轴：PA5_SCK / PA6_MISO / PA7_MOSI，PA4_NSS 硬件 CS）
 */
typedef enum {
    SPI_0,
    SPI_1,
    SPI_COUNT,
} spi_id_t;

/* 函数声明 */
void spi_init(void);
/* spi 读写实现（快照读取），返回值：1：成功，0：失败。
 * TX 为固定读命令（spi_init 预置，DMA 自动发送），tx_buf 仅为接口兼容保留，
 * rx_buf 为接收快照缓冲区（RX DMA 直写，上一拍完整数据）*/
int spi_write_read(spi_id_t id, uint16_t *tx_buf, uint16_t *rx_buf, uint32_t n_words);
/* 统一发起两路编码器 SPI 短事务（ADC JEOS 电流拷贝后，用户回调前调用，一拍一次），
 * CS 由硬件 NSS 管理 */
void spi_kick_all(void);

#endif