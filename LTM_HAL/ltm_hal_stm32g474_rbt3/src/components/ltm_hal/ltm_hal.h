#ifndef LTM_HAL_H
#define LTM_HAL_H

#include <stdint.h>

/*==================== 底层常量（与 BSP 保持一致，供上层使用） ====================*/
#define LTM_ENC_CPR                     (2097152U)  /* MT6835 绝对值编码器 21bit，单圈 2097152 */
#define LTM_ENC_RAD_PER_COUNT           (6.2831853f / (float)LTM_ENC_CPR)
#define LTM_ENC_ANGLE_PER_COUNT         (360.0f / (float)LTM_ENC_CPR)
#define LTM_ADC_TEMP_PER_COUNT          (0.1f)     /* 温度接口单位：0.1℃ */

/* Q15 标幺基准（1.0 pu = 下列满量程），与 RA6M4 版同口径：
 * 电流：运放 10 倍 + 5mΩ 采样电阻，12bit 中位偏置 -> 半量程 ±2048 LSB，每 LSB 0.0161133A
 *       1.0 pu = 2048 × 0.0161133 = 33.0 A
 * 母线：分压比 29.936，每 LSB 0.0241184V -> 1.0 pu = 4096 × 0.0241184 = 98.79 V */
#define LTM_ADC_FULL_SCALE_CURRENT_A    (33.0f)
#define LTM_ADC_FULL_SCALE_VOLTAGE_V    (98.79f)
#define LTM_ADC_CURRENT_PER_COUNT       (LTM_ADC_FULL_SCALE_CURRENT_A / 32768.0f)
#define LTM_ADC_VOLTAGE_PER_COUNT       (LTM_ADC_FULL_SCALE_VOLTAGE_V / 32768.0f)

/*==================== LED ====================*/
typedef enum {
    LTM_LED_STOP = 0,       /* 停机指示灯（黄）*/
    LTM_LED_RUN,            /* 运行指示灯（绿）*/
    LTM_LED_ON_OFF,         /* 上电指示灯（蓝）*/
    LTM_LED_ERR,            /* 故障指示灯（红）*/
} ltm_led_id_t;

/*==================== 总初始化（唯一 init 入口） ====================*/
void ltm_hal_init(void);    /* 按序初始化全部底层外设：系统/延时/串口/CAN/LED/PWM/编码器/ADC */

/*==================== 系统 ====================*/
uint64_t ltm_sys_get_tick(void);                        /* 获取系统时基（tick） */
uint64_t ltm_sys_get_ms(void);
uint64_t ltm_sys_get_us(void);
void     ltm_sys_set_callback(void (*callback)(void));  /* 设置 系统中断回调（1ms） */
void     ltm_sys_reset(void);                           /* 系统软复位 */

/*==================== 延时 ====================*/
void ltm_delay_ms(uint16_t ms);
void ltm_delay_us(uint32_t us);

/*==================== 调试串口 ====================*/
void ltm_uart_write(uint8_t *buf, uint16_t len);
void ltm_uart_write_nonblock(uint8_t *buf, uint16_t len);
void ltm_uart_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len));

/*==================== 三相 PWM ====================*/
void ltm_pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC);  /* Q15：32768 = 100% */
void ltm_pwm_start(void);
void ltm_pwm_stop(void);

/*==================== ADC ====================*/
void ltm_adc_calibrate_zero(uint16_t samples);
void ltm_adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic);  /* Q15 标幺 */
void ltm_adc_get_temp(int32_t *motor_temp, int32_t *driver_temp); /* 0.1℃ */
void ltm_adc_get_vbus(int32_t *vbus);                             /* Q15 标幺 */
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

/*==================== Flash（片内 Flash） ====================*/
uint32_t ltm_flash_get_sector_size(void);                 /* 扇区大小（字节） */
int      ltm_flash_erase(uint32_t addr, uint32_t size);   /* 按扇区擦除，addr 需扇区对齐，0=成功 */
int      ltm_flash_write(uint32_t addr, const uint8_t *data, uint32_t len);  /* 编程，0=成功 */
int      ltm_flash_read(uint32_t addr, uint8_t *data, uint32_t len);         /* 读取，0=成功 */

#endif /* LTM_HAL_H */
