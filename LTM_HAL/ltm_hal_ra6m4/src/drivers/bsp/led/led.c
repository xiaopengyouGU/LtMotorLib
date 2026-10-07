#include "led.h"

/* LED 配置表，与头文件中的宏定义对应 */
typedef struct{
	bsp_io_port_pin_t port_pin;			/* 端口号 */
	uint8_t state;						/* 0：灭，1：亮 */
}led_config_t;

static led_config_t g_led_config[LED_COUNT] = {
		[LED_STOP] = { LED_STOP_PORT_PIN, LED_STOP_STATE },
		[LED_RUN]  = { LED_RUN_PORT_PIN,  LED_RUN_STATE },
		[LED_ON_OFF] = { LED_ON_OFF_PORT_PIN, LED_ON_OFF_STATE },
		[LED_ERR]  = { LED_ERR_PORT_PIN,  LED_ERR_STATE },
};

void led_init(void)			/* 初始化 LED */
{
    /* IO 的初始化在 hardware 层自动完成了，不需要手动操作。*/
}

/* 设置 LED 电平，state : 0：灭，1：亮 
   本驱动板IO口高电平时，LED亮灯 */
void led_set(led_id_t id, uint8_t state)
{
	if (id >= LED_COUNT)		return;
	if (id == LED_ON_OFF)		return;
	R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, g_led_config[id].port_pin, state ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
	g_led_config[id].state = state;				/* 更新 state */
}

/* 翻转 LED 电平 */
void led_toggle(led_id_t id)
{
	if (id >= LED_COUNT)		return;
	if (id == LED_ON_OFF)		return;
	uint8_t state = g_led_config[id].state;
	if (state)	state = 0;
	else		state = 1;
	R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, g_led_config[id].port_pin, state ? BSP_IO_LEVEL_HIGH : BSP_IO_LEVEL_LOW);
	g_led_config[id].state = state;				/* 更新 state */ 
}
