#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "hal_data.h"

/* LED 端口与引脚号 */
#define LED_STOP_PORT_PIN                 BSP_IO_PORT_03_PIN_01
#define LED_RUN_PORT_PIN                  BSP_IO_PORT_03_PIN_08
#define LED_ERR_PORT_PIN                  BSP_IO_PORT_09_PIN_02

/* LED 初始状态：0：低电平，1：高电平 */
#define LED_STOP_STATE                     0
#define LED_RUN_STATE                      0
#define LED_ERR_STATE                      0
//#define LED_ON_STATE                       0

typedef enum {
    LED_STOP = 0,
    LED_RUN,
    LED_ERR,
    LED_COUNT
} led_id_t;

void led_init(void);
void led_set(led_id_t id, uint8_t state);
void led_toggle(led_id_t id);

#endif
