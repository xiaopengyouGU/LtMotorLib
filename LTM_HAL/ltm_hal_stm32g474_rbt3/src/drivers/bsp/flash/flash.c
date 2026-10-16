/*
 * SPDX-License-Identifier: MIT
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 片内 Flash（页擦除 2KB，64bit 双字编程）
 *                             接口与其它平台保持一致（地址基于 0x08000000）
 * 2026-09-02     Lvtou        BOOT0 来源切换：PB8/BOOT0 引脚 -> nBOOT0 选项位，释放 PB8 给 FDCAN1_RX
 */
#include "bsp/flash/flash.h"
#include "stm32g4xx_hal.h"
#include <string.h>

#ifndef FLASH_PAGE_SIZE
#define FLASH_PAGE_SIZE         2048U       /* G474 页大小（2KB，HAL 已定义则沿用） */
#endif
#define FLASH_DW_SIZE           8U          /* 编程粒度：双字 64bit */

static uint8_t s_inited = 0;                /* 自动配置完成标志（HAL_FLASH 幂等占位） */

int flash_init(void)
{
    if (s_inited) return 0;                 /* 幂等：只处理一次 */
    s_inited = 1;
    return 0;
}

uint32_t flash_get_sector_size(void)
{
    return FLASH_PAGE_SIZE;                 /* 页大小（字节，2KB） */
}

int flash_erase(uint32_t addr, uint32_t size)
{
    if (size == 0) return -1;
    if (!s_inited && flash_init() != 0) return -1;
    if (addr < FLASH_BASE) return -1;

    uint32_t page  = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    uint32_t pages = (size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks     = FLASH_BANK_1;
    erase.Page      = page;
    erase.NbPages   = pages;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK) {
        HAL_FLASH_Lock();
        return -1;
    }
    HAL_FLASH_Lock();
    return 0;
}

static int _flash_program_dw(uint32_t addr, const uint8_t *buf)   /* 编程一个双字 */
{
    uint64_t data;
    memcpy(&data, buf, FLASH_DW_SIZE);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data) != HAL_OK) {
        return -1;
    }
    return 0;
}

int flash_write(uint32_t addr, const uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return -1;
    if (!s_inited && flash_init() != 0) return -1;
    if (addr < FLASH_BASE) return -1;

    HAL_FLASH_Unlock();

    /* 头部：非 8B 对齐的起始部分（读改写，前提是周边已擦除/已知） */
    uint32_t cur = addr;
    while (cur < addr + len && (cur & (FLASH_DW_SIZE - 1)) != 0) {
        uint8_t buf[FLASH_DW_SIZE];
        memcpy(buf, (const void *)(cur & ~(FLASH_DW_SIZE - 1)), FLASH_DW_SIZE);
        buf[cur & (FLASH_DW_SIZE - 1)] = data[cur - addr];
        if (_flash_program_dw(cur & ~(FLASH_DW_SIZE - 1), buf) != 0) {
            HAL_FLASH_Lock();
            return -1;
        }
        cur++;
    }

    /* 中间：整 8B 对齐部分直接编程 */
    while (cur + FLASH_DW_SIZE <= addr + len) {
        uint8_t buf[FLASH_DW_SIZE];
        memcpy(buf, data + (cur - addr), FLASH_DW_SIZE);
        if (_flash_program_dw(cur, buf) != 0) {
            HAL_FLASH_Lock();
            return -1;
        }
        cur += FLASH_DW_SIZE;
    }

    /* 尾部：不足 8B 的收尾部分（读改写） */
    while (cur < addr + len) {
        uint8_t buf[FLASH_DW_SIZE];
        memcpy(buf, (const void *)(cur & ~(FLASH_DW_SIZE - 1)), FLASH_DW_SIZE);
        buf[cur & (FLASH_DW_SIZE - 1)] = data[cur - addr];
        if (_flash_program_dw(cur & ~(FLASH_DW_SIZE - 1), buf) != 0) {
            HAL_FLASH_Lock();
            return -1;
        }
        cur++;
    }

    HAL_FLASH_Lock();
    return 0;
}

int flash_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (!data || len == 0) return -1;
    memcpy(data, (const void *)addr, len);  /* 片内 Flash 直接内存读取 */
    return 0;
}

/* BOOT0 来源切换（一次性）：PB8/BOOT0 引脚采样 -> nBOOT0 选项位
 * 目的：释放 PB8/BOOT0 焊盘给 FDCAN1_RX（CAN-FD），复位瞬间不再受引脚电平影响
 * 目标配置：nSWBOOT0=0（BOOT0 取 nBOOT0 选项位）+ nBOOT0=1（主 Flash 启动）
 * 首次执行：编程选项字节并 OBL_LAUNCH 复位一次；此后每次上电直接返回 0
 * 如需恢复引脚采样：用 STM32CubeProgrammer 写回 nSWBOOT0=1 即可 */
int flash_boot0_enable_optbit(void)
{
    /* 已生效：BOOT0 取自选项位且 nBOOT0=1（主 Flash），无需重复编程 */
    if (((FLASH->OPTR & FLASH_OPTR_nSWBOOT0) == 0U) &&
        ((FLASH->OPTR & FLASH_OPTR_nBOOT0)  != 0U)) {
        return 0;
    }

    FLASH_OBProgramInitTypeDef ob = {0};
    HAL_FLASHEx_OBGetConfig(&ob);           /* 读取当前配置，仅改 BOOT 相关位 */
    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;
    ob.USERConfig &= ~FLASH_OPTR_nSWBOOT0;  /* BOOT0 取自 nBOOT0 选项位 */
    ob.USERConfig |=  FLASH_OPTR_nBOOT0;    /* nBOOT0=1 -> 主 Flash 启动 */

    HAL_FLASH_Unlock();
    if (HAL_FLASH_OB_Unlock() != HAL_OK) {
        HAL_FLASH_Lock();
        return -1;
    }
    if (HAL_FLASHEx_OBProgram(&ob) != HAL_OK) {
        HAL_FLASH_OB_Lock();
        HAL_FLASH_Lock();
        return -1;
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
    HAL_FLASH_OB_Launch();                  /* 重载选项字节并触发系统复位 */
    NVIC_SystemReset();                     /* 保险：确保以新配置重启 */
    while (1) { }
}
