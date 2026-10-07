#ifndef _CANFD_H_
#define _CANFD_H_

#include "hal_data.h"
#include <stdint.h>

/* CAN-FD 底层操作接口，收发标准帧，支持软件滤波功能（白名单）
 * 仲裁段：1Mbps，数据段：2Mbps，提供 CAN 发送接口 */

void canfd_init(void);                  
void canfd_send(uint16_t id, uint8_t *buf, uint16_t len);      /* CAN-FD 发送接口 */
void can_send(uint16_t id, uint8_t *buf, uint16_t len);        /* CAN 发送接口 */
uint8_t canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len); /* 返回 1：有CAN-FD帧，2：有CAN帧，0：无 */
void canfd_set_rxcall(void (*callback)(void));                 /* 设置 CAN-FD 接收回调 */  
/* 滤波器：白名单模式，最多 16 个 ID，使能后只收白名单中的帧 */
void canfd_filter_add(uint16_t id);
void canfd_filter_clear(void);                                 
void canfd_filter_enable(void);
void canfd_filter_disable(void);

#endif
