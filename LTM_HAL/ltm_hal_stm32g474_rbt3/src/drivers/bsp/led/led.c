/*
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-19     Lvtou        STM32G474 LED：板级运动控制信号 GPIO 映射
 */
#include "bsp/led/led.h"
#include "stm32g4xx_hal.h"

/* LED 配置表，与头文件中的宏定义对应 */
typedef struct {
    GPIO_TypeDef *port;         /* GPIO 端口 */
    uint16_t      pin;          /* 引脚号 */
    uint8_t       state;        /* 端口电平 1：高电平，0：低电平 */
} led_config_t;

static led_config_t g_led_config[LED_COUNT] = {
    [LED_STOP]   = { LED_STOP_GPIO,   LED_STOP_PIN,   LED_STOP_STATE },
    [LED_RUN]    = { LED_RUN_GPIO,    LED_RUN_PIN,    LED_RUN_STATE },
    [LED_ON_OFF] = { LED_ON_OFF_GPIO, LED_ON_OFF_PIN, LED_ON_OFF_STATE },
    [LED_ERR]    = { LED_ERR_GPIO,    LED_ERR_PIN,    LED_ERR_STATE },
};

void led_init(void)                         /* 初始化 LED */
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    for (int i = 0; i < LED_COUNT; i++) {
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin   = g_led_config[i].pin;
        gpio.Mode  = GPIO_MODE_OUTPUT_PP;
        gpio.Pull  = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(g_led_config[i].port, &gpio);
        HAL_GPIO_WritePin(g_led_config[i].port, g_led_config[i].pin,
                          g_led_config[i].state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/* 设置 LED 电平，state : 0：灭，1：亮 */
void led_set(led_id_t id, uint8_t state)
{
    if (id >= LED_COUNT)    return;
    HAL_GPIO_WritePin(g_led_config[id].port, g_led_config[id].pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_led_config[id].state = state;         /* 更新 state */
}

/* 翻转 LED 电平 */
void led_toggle(led_id_t id)
{
    if (id >= LED_COUNT)    return;
    uint8_t state = g_led_config[id].state;
    if (state)  state = 0;
    else        state = 1;
    HAL_GPIO_WritePin(g_led_config[id].port, g_led_config[id].pin,
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_led_config[id].state = state;         /* 更新 state */
}
