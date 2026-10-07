#ifndef __BSP_LED_H
#define __BSP_LED_H
#include "hal_data.h"

/* LED 端口与引脚号 */
#define LED_STOP_PORT_PIN                 BSP_IO_PORT_04_PIN_05
#define LED_RUN_PORT_PIN                  BSP_IO_PORT_04_PIN_04
#define LED_ON_OFF_PORT_PIN               BSP_IO_PORT_04_PIN_04     /* 这个LED实际常亮，宏仅做占位 */
#define LED_ERR_PORT_PIN                  BSP_IO_PORT_00_PIN_02

/* LED 初始状态：0：灭，1：亮 */
#define LED_STOP_STATE                     0
#define LED_RUN_STATE                      0
#define LED_ON_OFF_STATE                   1
#define LED_ERR_STATE                      0

typedef enum {
    LED_STOP = 0,
    LED_RUN,
    LED_ON_OFF,             /* 电机上电标志 */
    LED_ERR,
    LED_COUNT
} led_id_t;

void led_init(void);
void led_set(led_id_t id, uint8_t state);
void led_toggle(led_id_t id);

#endif
