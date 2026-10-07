#ifndef LTM_HAL_H
#define LTM_HAL_H

#include <stdint.h>

/*==================== 底层常量（与 BSP 保持一致，供上层使用） ====================*/
#define LTM_ENC_CPR                     (10000U)  /* 编码器单圈分辨率 */
#define LTM_ENC_RAD_PER_COUNT           (6.2831853f / (float)LTM_ENC_CPR)  /* 2PI / LTM_ENC_CPR */
#define LTM_ENC_ANGLE_PER_COUNT         (360.0f / (float)LTM_ENC_CPR)
#define LTM_ADC_TEMP_PER_COUNT          (0.1f)    /* 温度接口单位：0.1℃ */
#define LTM_ADC_CURRENT_PER_COUNT       (4.1962e-4f)
#define LTM_ADC_VOLTAGE_PER_COUNT       (2.5177e-3f)

#define LTM_ADC_FULL_SCALE_CURRENT_A    (13.75f)  /* ADC 满量程电流（A）*/
#define LTM_ADC_FULL_SCALE_VOLTAGE_V    (82.5f)   /* ADC 满量程母线电压（V）*/


/*==================== LED ====================*/
typedef enum {
    LTM_LED_STOP = 0,       /* 停机指示灯 */
    LTM_LED_RUN,            /* 运行指示灯 */
    LTM_LED_ON_OFF,         /* 上电指示灯 */
    LTM_LED_ERR,            /* 故障指示灯 */
} ltm_led_id_t;

/*==================== 总初始化（唯一 init 入口） ====================*/
void ltm_hal_init(void);      /* 按序初始化全部底层外设：系统/延时/串口/CAN/LED/PWM/编码器/ADC */

/*==================== 系统 ====================*/
uint64_t ltm_sys_get_tick(void);                        /* 获取系统时基（tick）*/
uint64_t ltm_sys_get_ms(void);          
uint64_t ltm_sys_get_us(void);          
void     ltm_sys_set_callback(void (*callback)(void));  /* 设置 系统中断回调（1ms）*/
void     ltm_sys_reset(void); /* 系统软复位（SCB->AIRCR），复位后进 BootLoader 升级窗口 */

/*==================== 延时 ====================*/
void ltm_delay_ms(uint16_t ms);
void ltm_delay_us(uint32_t us);

/*==================== 调试串口 ====================*/
void ltm_uart_write(uint8_t *buf, uint16_t len);
void ltm_uart_write_nonblock(uint8_t *buf, uint16_t len);
void ltm_uart_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len));

/* 占空比 Q15 标幺：32768 = 100%（与 lt_foc_update 输出同口径，底层按自己的 PWM
 * 周期折算成比较值，越界由底层夹住）*/
void ltm_pwm_set_dutys(int32_t dutyA, int32_t dutyB, int32_t dutyC);
void ltm_pwm_start(void);
void ltm_pwm_stop(void);

/*==================== ADC ====================*/
/* 返回Q15 标幺电流和标幺电压，温度单位 0.1℃ */
void ltm_adc_calibrate_zero(uint16_t samples);
void ltm_adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic);
void ltm_adc_get_temp(int32_t *motor_temp, int32_t *driver_temp);
void ltm_adc_get_vbus(int32_t *vbus);
void ltm_adc_set_callback(void (*callback)(void));

/*==================== 编码器 ====================*/
void     ltm_enc_update(void);
uint32_t ltm_enc_get_count(void);               /* 获取单圈绝对值（0~CPR-1）*/
int64_t  ltm_enc_get_position(void);            /* 获取绝对位置（count）*/
void     ltm_enc_set_zero(void);

/*==================== LED ====================*/
void ltm_led_set(ltm_led_id_t id, uint8_t state);
void ltm_led_toggle(ltm_led_id_t id);

/*==================== CAN-FD ====================*/
void    ltm_canfd_send(uint16_t id,  uint8_t *buf, uint16_t len);
void    ltm_can_send(uint16_t id,    uint8_t *buf, uint16_t len);
uint8_t ltm_canfd_recv(uint16_t *id, uint8_t *buf, uint16_t *len);
void    ltm_canfd_set_rxcall(void (*callback)(void));  /* 接收回调 */
void    ltm_canfd_filter_add(uint16_t id);      /* CAN-FD 软件白名单 */
void    ltm_canfd_filter_clear(void);
void    ltm_canfd_filter_enable(void);
void    ltm_canfd_filter_disable(void);

/*==================== Flash ====================*/
/* 片内 flash 读/写/擦，地址与长度都按字节；返回 0 成功、非 0 失败。
 * RA6M4 只实现读（直接寻址 memcpy）；写/擦返回失败，齿槽表由 JLink 写入 flash。*/
int ltm_flash_read(uint32_t addr, uint8_t *buf, uint32_t len);
int ltm_flash_write(uint32_t addr, const uint8_t *buf, uint32_t len);
int ltm_flash_erase(uint32_t addr, uint32_t len);

#endif /* LTM_HAL_H */
