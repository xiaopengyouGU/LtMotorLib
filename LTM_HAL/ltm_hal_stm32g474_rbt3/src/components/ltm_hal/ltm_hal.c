/*
 * Change Logs:
 * Date           Author       Notes
 * 2026-08-22     Lvtou        LTM_HAL —— HPM6E8Y（HPM6E00 SiP PMSM 关节板）
 * 2026-09-01     Lvtou        LTM_HAL —— STM32G474RCT6（迁移：API 不变，底层换 STM32 HAL）
 * 2026-09-19     Lvtou        STM32G474RBT3版本驱动迁移       
 */
#include "ltm_hal/ltm_hal.h"

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
#include "bsp/flash/flash.h"

/* BootLoader 跳转标记：RAM 顶部保留字（与 BootLoader 约定一致，链接脚本已留 4 字节）
 * G474RBT3：SRAM 96KB（SRAM1 80K + SRAM2 16K）@ 0x20000000 → 顶部 4 字节 = 0x20017FFC
 * 注意 CCM SRAM(0x10000000) 不算在内，且 DMA 访问不到 CCM；此地址须与 BootLoader 侧一致 */
#define BL_JUMP_FLAG_ADDR       0x20017FFCUL
#define BL_JUMP_MAGIC           0x4A554D50UL   /* "JUMP" */

/*==================== 总初始化（唯一 init 入口） ====================*/
void ltm_hal_init(void)
{
    /* BootLoader 软跳转：清标记只处理一次（须在任何外设/中断使用之前，统一收口到 HAL） */
    if (*(volatile uint32_t *)BL_JUMP_FLAG_ADDR == BL_JUMP_MAGIC) {
        *(volatile uint32_t *)BL_JUMP_FLAG_ADDR = 0;
    }

    system_init();      /* 系统：时钟 + 时基 + 1ms 回调 */
    delay_init();       /* 延时 */
    uart_port_init();   /* 调试串口 USART3 */
    canfd_init();       /* CAN-FD FDCAN1 */
    led_init();         /* LED */
    pwm_init();         /* 三相 PWM（先启动计数器，保证 ADC/编码器触发链路建立） */
    encoder_init();     /* MT6835 绝对值编码器（21bit，仅主轴 SPI1 + DMA） */
    adc_init();         /* ADC1（注入组电流 + 规则组母线/温度） */

    /* 全局中断使能：各外设 IRQ 已逐个使能，最后统一放开，
     * 保证中断只在全部初始化完成后开始触发（含 BootLoader 软跳转场景） */
    __enable_irq();
}

/*==================== 系统 ====================*/
uint64_t ltm_sys_get_tick(void)      { return system_get_tick(); }
uint64_t ltm_sys_get_ms(void)        { return system_get_ms(); }
uint64_t ltm_sys_get_us(void)        { return system_get_us(); }
void ltm_sys_set_callback(void (*callback)(void)) { system_set_callback(callback); }
void ltm_sys_reset(void)             { system_reset(); }  /* 系统软复位：NVIC_SystemReset */

/*==================== 延时 ====================*/
void ltm_delay_ms(uint16_t ms)       { delay_ms(ms); }
void ltm_delay_us(uint32_t us)       { delay_us(us); }

/*==================== 调试串口 ====================*/
void ltm_uart_write(uint8_t *buf, uint16_t len)          { uart_port_write(buf, len); }
void ltm_uart_write_nonblock(uint8_t *buf, uint16_t len) { uart_port_write_nonblock(buf, len); }
void ltm_uart_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len)) { uart_port_set_rxcall(rxcall); }

/*==================== 三相 PWM ====================*/
void ltm_pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC) { pwm_set_dutys(dutyA, dutyB, dutyC); }
void ltm_pwm_start(void)             { pwm_start(); }
void ltm_pwm_stop(void)              { pwm_stop(); }

/*==================== ADC ====================*/
void ltm_adc_calibrate_zero(uint16_t samples) { adc_calibrate_zero(samples); }
void ltm_adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic) { adc_get_current(Ia, Ib, Ic); }
void ltm_adc_get_temp(int32_t *motor_temp, int32_t *driver_temp) { adc_get_temp(motor_temp, driver_temp); }
void ltm_adc_get_vbus(int32_t *vbus)   { adc_get_vbus(vbus); }
void ltm_adc_set_callback(void (*callback)(void)) { adc_set_callback(callback); }

/*==================== 编码器 ====================*/
void     ltm_enc_update(void)        { encoder_update(); }
uint32_t ltm_enc_get_count(void)     { return encoder_get_count(); }
int64_t  ltm_enc_get_position(void)  { return encoder_get_position(); }
float    ltm_enc_get_angle_deg(void) { return encoder_get_angle_deg(); }
float    ltm_enc_get_angle_rad(void) { return encoder_get_angle_rad(); }
void     ltm_enc_set_zero(void)      { encoder_set_zero(); }

/*==================== LED ====================*/
void ltm_led_set(ltm_led_id_t id, uint8_t state) { led_set((led_id_t)id, state); }
void ltm_led_toggle(ltm_led_id_t id) { led_toggle((led_id_t)id); }

/*==================== CAN-FD ====================*/
void    ltm_canfd_send(uint16_t id, uint8_t *buf, uint16_t len) { canfd_send(id, buf, len); }
void    ltm_can_send(uint16_t id, uint8_t *buf, uint16_t len)   { can_send(id, buf, len); }
uint8_t ltm_canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len) { return canfd_recv(id, buf, len); }
void    ltm_canfd_set_rxcall(void (*callback)(void)) { canfd_set_rxcall(callback); }
void    ltm_canfd_filter_add(uint16_t id)      { canfd_filter_add(id); }
void    ltm_canfd_filter_clear(void)           { canfd_filter_clear(); }
void    ltm_canfd_filter_enable(void)          { canfd_filter_enable(); }
void    ltm_canfd_filter_disable(void)         { canfd_filter_disable(); }

/*==================== Flash（片内 Flash） ====================*/
uint32_t ltm_flash_get_sector_size(void)       { return flash_get_sector_size(); }
int ltm_flash_erase(uint32_t addr, uint32_t size) { return flash_erase(addr, size); }
int ltm_flash_write(uint32_t addr, const uint8_t *data, uint32_t len) { return flash_write(addr, data, len); }
int ltm_flash_read(uint32_t addr, uint8_t *data, uint32_t len)        { return flash_read(addr, data, len); }
