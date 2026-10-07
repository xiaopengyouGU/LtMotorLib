/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-08-12     Lvtou        统一 HAL 层：封装 BSP/系统底层调用
 */
#include "ltm_hal/ltm_hal.h"

#include "hal_data.h"        /* FSP: g_flash0 (r_flash_hp) */
#include <string.h>

/* 系统层 */
#include "system/sys/sys.h"
#include "system/delay/delay.h"
#include "system/uart/uart.h"

/* 外设层 */
#include "bsp/led/led.h"
#include "bsp/pwm/pwm.h"
#include "bsp/adc/adc.h"
#include "bsp/encoder/encoder.h"
#include "bsp/canfd/canfd.h"

/* BootLoader 跳转标记：RAM 顶部保留字（与 BootLoader 约定一致，App 链接脚本已留 4 字节） */
#define LTM_BL_JUMP_FLAG_ADDR   (0x20000000UL + 0x40000UL - 4)   /* RA6M4 SRAM 顶部 0x2003FFFC（256KB）*/
#define LTM_BL_JUMP_MAGIC       0x4A554D50UL                     /* "JUMP" */

/*==================== 总初始化（唯一 init 入口） ====================*/
void ltm_hal_init(void)
{
    /* BootLoader 跳转恢复：软跳转不复位 PRIMASK，检测标记则恢复中断并清标记，必须先于任何外设/中断初始化 */
    if (*(volatile uint32_t *)LTM_BL_JUMP_FLAG_ADDR == LTM_BL_JUMP_MAGIC) {
        *(volatile uint32_t *)LTM_BL_JUMP_FLAG_ADDR = 0;
        __asm volatile ("cpsie i");
    }

    system_init();      /* 系统 */
    delay_init();       /* 延时 */
    uart_init();        /* 调试串口 */
    canfd_init();       /* CAN-FD */
    led_init();         /* LED */
    pwm_init();         /* 三相 PWM（先启动 GPT，保证 ADC 触发链路建立） */
    encoder_init();     /* 编码器 */
    adc_init();         /* ADC */
}

/*==================== 系统 ====================*/
uint64_t ltm_sys_get_tick(void)      { return system_get_tick(); }
uint64_t ltm_sys_get_ms(void)        { return system_get_ms(); }
uint64_t ltm_sys_get_us(void)        { return system_get_us(); }
void ltm_sys_set_callback(void (*callback)(void)) { system_set_callback(callback); }

/* 系统软复位：写 SCB->AIRCR（VECTKEY + SYSRESETREQ），复位后 BootLoader 进入升级窗口。
 * 不依赖 CMSIS 头文件，直接操作寄存器（Cortex-M 通用）。 */
void ltm_sys_reset(void)
{
    __asm volatile ("dsb");
    *(volatile uint32_t *)0xE000ED0C = 0x05FA0004UL;   /* SCB->AIRCR */
    __asm volatile ("dsb");
    while (1) { }
}

/*==================== 延时 ====================*/
void ltm_delay_ms(uint16_t ms)       { delay_ms(ms); }
void ltm_delay_us(uint32_t us)       { delay_us(us); }

/*==================== 调试串口 ====================*/
void ltm_uart_write(uint8_t *buf, uint16_t len)          { uart_write(buf, len); }
void ltm_uart_write_nonblock(uint8_t *buf, uint16_t len) { uart_write_nonblock(buf, len); }
void ltm_uart_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len)) { uart_set_rxcall(rxcall); }

/*==================== 三相 PWM ====================*/
void ltm_pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC) { pwm_set_dutys(dutyA, dutyB, dutyC); }
void ltm_pwm_start(void)             { pwm_start(); }
void ltm_pwm_stop(void)              { pwm_stop(); }

/*==================== ADC ====================*/
void ltm_adc_calibrate_zero(uint16_t samples) { adc_calibrate_zero(samples); }
void ltm_adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic)  { adc_get_current(Ia, Ib, Ic); }
void ltm_adc_get_temp(int32_t *motor_temp, int32_t *driver_temp) { adc_get_temp(motor_temp, driver_temp); }
void ltm_adc_get_vbus(int32_t *vbus)   { adc_get_vbus(vbus); }
void ltm_adc_set_callback(void (*callback)(void)) { adc_set_callback(callback); }

/*==================== 编码器 ====================*/
void     ltm_enc_update(void)        { encoder_update(); }
uint32_t ltm_enc_get_count(void)     { return encoder_get_count(); }
int64_t  ltm_enc_get_position(void)  { return encoder_get_position(); }
void     ltm_enc_set_zero(void)      { encoder_set_zero(); }

/*==================== LED ====================*/
void ltm_led_set(ltm_led_id_t id, uint8_t state) { led_set((led_id_t)id, state); }
void ltm_led_toggle(ltm_led_id_t id) { led_toggle((led_id_t)id); }

/*==================== CAN-FD ====================*/
void    ltm_canfd_send(uint16_t id,  uint8_t *buf, uint16_t len)  { canfd_send(id, buf, len); }
void    ltm_can_send(uint16_t id,    uint8_t *buf, uint16_t len)  { can_send(id, buf, len); }
uint8_t ltm_canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len) { return canfd_recv(id, buf, len); }
void    ltm_canfd_set_rxcall(void (*callback)(void)) { canfd_set_rxcall(callback); }
void    ltm_canfd_filter_add(uint16_t id)      { canfd_filter_add(id); }
void    ltm_canfd_filter_clear(void)           { canfd_filter_clear(); }
void    ltm_canfd_filter_enable(void)          { canfd_filter_enable(); }
void    ltm_canfd_filter_disable(void)         { canfd_filter_disable(); }

/*==================== Flash ====================*/
/* 片内 code flash 只读：片内 flash 直接寻址访问，memcpy 即可。
 * 齿槽表由 JLink 写在 flash 0x80024000，固件上电时读进 RAM 重建表。*/
int ltm_flash_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    if (!buf || !len) return -1;
    memcpy(buf, (const void *)addr, len);      /* 片内 flash 可直接寻址读 */
    return 0;
}

/* 写/擦不提供：齿槽表由 JLink 直接写入 flash 0x80024000，固件只读。
 * 真实现要处理 code flash 写期间的取指问题（关中断 + FSP r_flash_hp），
 * 现在不需要，留着桩挡住误用。*/
int ltm_flash_erase(uint32_t addr, uint32_t len)
{
    (void)addr; (void)len;
    return -1;
}

int ltm_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    (void)addr; (void)buf; (void)len;
    return -1;
}
