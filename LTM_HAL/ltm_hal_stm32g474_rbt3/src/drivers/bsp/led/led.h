#ifndef __BSP_LED_H
#define __BSP_LED_H
#include <stdint.h>
#include "stm32g4xx_hal.h"

/* LED 端口与引脚号（板级运动控制信号 GPIO 映射，推挽输出）*/
#define LED_STOP_GPIO                   GPIOB
#define LED_STOP_PIN                    GPIO_PIN_6       /* Motor STOP */
#define LED_RUN_GPIO                    GPIOB
#define LED_RUN_PIN                     GPIO_PIN_9       /* Motor RUN */
#define LED_ON_OFF_GPIO                 GPIOB
#define LED_ON_OFF_PIN                  GPIO_PIN_7       /* Motor ON/OFF */
#define LED_ERR_GPIO                    GPIOC
#define LED_ERR_PIN                     GPIO_PIN_12      /* Motor ERR */

/* LED 初始状态：0：灭，1：亮 */
#define LED_STOP_STATE                     0
#define LED_RUN_STATE                      0
#define LED_ON_OFF_STATE                   0
#define LED_ERR_STATE                      0

typedef enum {
    LED_STOP = 0,
    LED_RUN,
    LED_ON_OFF,
    LED_ERR,
    LED_COUNT
} led_id_t;

void led_init(void);
void led_set(led_id_t id, uint8_t state);
void led_toggle(led_id_t id);

#endif
