#ifndef BSP_FLASH_H
#define BSP_FLASH_H

#include <stdint.h>

/* STM32G474 片内 Flash（0x08000000，128KB）：
 * 页擦除 2KB，64bit 双字编程
 * 注：FLASH_BASE 由 stm32g474xx.h 提供（0x08000000），此处不再重复定义 */

int      flash_init(void);                       /* Flash 初始化（幂等），0=成功 */
uint32_t flash_get_sector_size(void);            /* 扇区大小（字节，2KB） */
int      flash_erase(uint32_t addr, uint32_t size);      /* 按扇区擦除，addr 需扇区对齐，0=成功 */
int      flash_write(uint32_t addr, const uint8_t *data, uint32_t len);  /* 编程（4B 对齐更优），0=成功 */
int      flash_read(uint32_t addr, uint8_t *data, uint32_t len);         /* 读取（直接内存读取），0=成功 */
int      flash_boot0_enable_optbit(void);       /* 一次性：BOOT0 改由 nBOOT0 选项位决定（释放 PB8/BOOT0 引脚） */

#endif /* BSP_FLASH_H */
