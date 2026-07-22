/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-22     Lvtou        CRC校验模块（查表法）
 */
#ifndef LT_CRC_H
#define LT_CRC_H

#include <stdint.h>
/*---------------------- CRC 校验实现 ----------------------*/
/* data为字节数据，len为数据长度（字节）*/
uint8_t  lt_crc8_check(const uint8_t * data, uint8_t len);      /* 多项式0x07，初始值0x00 */
uint16_t lt_crc16_check(const uint8_t *data, uint16_t len);     /* 多项式0x8005，初始值0x0000 */

#endif
