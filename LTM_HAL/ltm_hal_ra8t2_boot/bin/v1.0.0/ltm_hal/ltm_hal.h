#ifndef LTM_HAL_H
#define LTM_HAL_H

#include <stdint.h>

/*==================== 底层常量（与 BSP 保持一致，供上层使用） ====================*/
#define LTM_ENC_CPR             262144U        /* 编码器单圈分辨率（18位） */
#define LTM_ENC_RAD_PER_COUNT   2.39674e-5f    /* 2PI / ENCODER_CPR */

/*==================== LED ====================*/
typedef enum {
    LTM_LED_STOP = 0,       /* 停机指示灯 */
    LTM_LED_RUN,            /* 运行指示灯 */
    LTM_LED_ON_OFF,         /* 上电指示灯 */
    LTM_LED_ERR,            /* 故障指示灯 */
} ltm_led_id_t;

/*==================== 总初始化（唯一 init 入口） ====================*/
void ltm_hal_init(void);    /* 按序初始化全部底层外设：系统/延时/串口/CAN/LED/PWM/编码器/ADC */

/*==================== 系统 ====================*/
uint64_t ltm_sys_get_tick(void);                        /* 获取系统时基（tick）*/
uint64_t ltm_sys_get_ms(void);          
uint64_t ltm_sys_get_us(void);          
void     ltm_sys_set_callback(void (*callback)(void));  /* 设置 系统中断回调（1ms）*/

/*==================== 延时 ====================*/
void ltm_delay_ms(uint16_t ms);
void ltm_delay_us(uint32_t us);

/*==================== 调试串口 ====================*/
void ltm_uart_write(uint8_t *buf, uint16_t len);
void ltm_uart_write_nonblock(uint8_t *buf, uint16_t len);
void ltm_uart_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len));

/*==================== 三相 PWM ====================*/
void ltm_pwm_set_dutys(float dutyA, float dutyB, float dutyC);
void ltm_pwm_start(void);
void ltm_pwm_stop(void);

/*==================== ADC ====================*/
void ltm_adc_calibrate_zero(uint16_t samples);
void ltm_adc_get_current(float *Ia, float *Ib, float *Ic);
void ltm_adc_get_temp(float *motor_temp, float *driver_temp);
void ltm_adc_get_vbus(float *vbus);
void ltm_adc_get_offset(float *off);           /* 获取三相电流零点偏置（原始 LSB 平均）*/
void ltm_adc_set_callback(void (*callback)(void));

/*==================== 编码器 ====================*/
void     ltm_enc_update(void);
uint32_t ltm_enc_get_count(void);
int64_t  ltm_enc_get_position(void);
float    ltm_enc_get_angle_deg(void);
float    ltm_enc_get_angle_rad(void);
void     ltm_enc_set_zero(void);

/*==================== LED ====================*/
void ltm_led_set(ltm_led_id_t id, uint8_t state);
void ltm_led_toggle(ltm_led_id_t id);

/*==================== CAN-FD ====================*/
void    ltm_canfd_send(uint16_t id, uint8_t *buf, uint16_t len);
void    ltm_can_send(uint16_t id, uint8_t *buf, uint16_t len);
uint8_t ltm_canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len);
void    ltm_canfd_set_rxcall(void (*callback)(void));  /* 接收回调 */
void    ltm_canfd_filter_add(uint16_t id);      /* CAN-FD 软件白名单 */
void    ltm_canfd_filter_clear(void);
void    ltm_canfd_filter_enable(void);
void    ltm_canfd_filter_disable(void);

#endif /* LTM_HAL_H */
