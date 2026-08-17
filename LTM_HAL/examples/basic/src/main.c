/*
 * LTM_HAL 简单开环例程
 * - 底层：LTM_HAL 静态库（ltm_hal.h）
 * - 通信：LTM 协议（ltm_commut，基于 ltm_uart 接口）
 * - 控制：开环 FOC（Data_Target 下发 Vq 占空比 %，SVPWM 调制）
 * - 上报：5 条曲线（A/B/C 相电流、母线电压、机械角度）
 *
 * 电流零点校准说明：
 *   校准的本质 = 在"电流为 0"时对 ADC 原始值取平均作为零点偏置 offset，
 *   adc_get_current = (raw - offset) * 系数，校准完成前返回 0。
 *   两种可行的零电流状态：
 *     a) 三相 0% 占空比（全下桥导通）：无开关动作，电流严格为 0，采样最干净；
 *     b) 三相 50% 等占空比：线电压为 0 → 相电流为 0（本例采用，与旧固件一致）。
 *   前提：电机停转；ADC 扫描由 GPT/ELC 触发，与占空比无关。
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ltm_hal/ltm_hal.h"
#include "protocol/ltm_commut.h"
#include "control/foc/lt_foc.h"
#include "math/basic/lt_math.h"

/* ---- 电机参数 ---- */
#define POLE_PAIRS           10
#define ENCODER_ZERO_OFFSET  203609.0f    /* 电角度绝对零点偏置（count）*/

static float   vq_target = 0.0f;          /* 开环 Q 轴电压指令（%，-100~100）*/
static uint8_t run_flag  = 0;             /* 运行标志 */

/* CAN-FD 升级触发 ID：与 BootLoader 升级 ID 一致，上位机发该 ID 帧 → App 软复位进 BootLoader */
#define APP_UPGRADE_TRIGGER_ID  0x7F0

void user_func(uint8_t data_type, uint8_t *buf, uint16_t len);
void user_canfd_rxcall(void);

int main(void)
{
    /* 底层外设统一初始化（系统/延时/串口/CAN/LED/PWM/编码器/ADC） */
    ltm_hal_init();

    /* LTM 通讯通道（基于 ltm_uart 接口）——提前初始化，便于打印校准状态 */
    ltm_commut_init();
    ltm_commut_set_send(ltm_uart_write);
    ltm_uart_set_rxcall(ltm_commut_recv);
    ltm_canfd_set_rxcall(user_canfd_rxcall);

    ltm_led_set(LTM_LED_ON_OFF, 1);
    ltm_led_set(LTM_LED_STOP, 1);
    ltm_led_set(LTM_LED_RUN, 0);

    /* ===== 电流零点校准 =====
     * 三相 50% 等占空比 → 线电压 0 → 相电流 0（电机停转），
     * 对 5000 次 ADC 扫描取平均作为零点偏置（25kHz ≈ 200ms）。
     * 校准完成前 adc_get_current 返回 0。 */
    ltm_pwm_start();
    ltm_pwm_set_dutys(0.0f, 0.0f, 0.0f);
    ltm_delay_ms(200);                   /* 等电流尖峰消除 */
    ltm_adc_calibrate_zero(2500);        /* 2500 次扫描平均 */
    ltm_delay_ms(500);                   /* 等校准完成 */
    ltm_commut_printf("ADC calib done! \r\n");

    /* ---- 调试：打印校准 offset + 停机偏置自检 ---- */
    float sIa = 0, sIb = 0, sIc = 0;
    float dIa, dIb, dIc;
    for (int i = 0; i < 500; i++) {
        ltm_adc_get_current(&dIa, &dIb, &dIc);
        sIa += dIa; sIb += dIb; sIc += dIc;
        ltm_delay_ms(2);
    }
    char fbuf[3][16];
    LT_FTOAT3(fbuf[0], sIa / 500.0f);
    LT_FTOAT3(fbuf[1], sIb / 500.0f);
    LT_FTOAT3(fbuf[2], sIc / 500.0f);
    ltm_commut_printf("standstill mean A: %s A\r\n", fbuf[0]);
    ltm_commut_printf("standstill mean B: %s A\r\n", fbuf[1]);
    ltm_commut_printf("standstill mean C: %s A\r\n", fbuf[2]);

    /* 开环 FOC：SVPWM 调制 */
    lt_foc_init(FOC_TYPE_SVPWM);

    ltm_curves curves;
    uint8_t data_type, data[128];
    uint16_t data_len;
    uint32_t count = 0;
    ltm_led_set(LTM_LED_STOP, 1);
    for (int i = 0; i < 80; i++) {
        ltm_enc_update();           /* 定时更新编码器 */
    }
    ltm_commut_printf("Hello Lvtou!!! \n");
    // /* 预定位操作，获取电角度对应的编码器零点偏置 */
    // ltm_pwm_set_dutys(0.05, 0, 0);
    // ltm_delay_ms(200);
    // int32_t my_offset = ltm_enc_get_count();
    // ltm_commut_printf("offset = %d \r\n", my_offset);

    while(1)
    {
        /* 协议命令处理 */
        if(ltm_commut_process(&data_type, data, &data_len)){
            user_func(data_type, data, data_len);
        }
        ltm_enc_update();           /* 定时更新编码器 */
        /* 开环控制：编码器电角度 → SVPWM → 三相占空比 */
        if(run_flag){
            float enc = (float)ltm_enc_get_count();
            float the = (enc - ENCODER_ZERO_OFFSET) * LTM_ENC_RAD_PER_COUNT * POLE_PAIRS;
            the = lt_normalize(the);
            lt_foc_process(0.0f, vq_target * 0.01f, the);
            float dA, dB, dC;
            lt_foc_get_dutys(&dA, &dB, &dC);
            ltm_pwm_set_dutys(dA, dB, dC);
            ltm_led_set(LTM_LED_STOP, 0);
            ltm_led_set(LTM_LED_RUN, 1);
        }else{
            ltm_pwm_set_dutys(0.0f, 0.0f, 0.0f);
            ltm_led_set(LTM_LED_STOP, 1);
            ltm_led_set(LTM_LED_RUN, 0);
        }
        if (count % 1500 == 0) {
            uint16_t id = 0x008;
            uint8_t tx_buf[16] = {0x11,0x22,0x33,0x44,0x56,0x78,0x90,0xAB,0xCD};
            ltm_canfd_send(id, tx_buf, 16);
        }
        /* 5 条曲线：A/B/C 相电流、母线电压、机械角度 */
        float Ia, Ib, Ic, vbus;
        ltm_adc_get_current(&Ia, &Ib, &Ic);
        ltm_adc_get_vbus(&vbus);
        curves.size = 5;
        curves.values[0] = Ia;
        curves.values[1] = Ib;
        curves.values[2] = Ic;
        curves.values[3] = vbus;
        curves.values[4] = (float)ltm_enc_get_position() * (360.0f / LTM_ENC_CPR);
        ltm_commut_send_curves(&curves);

        ltm_delay_ms(2);
        count++;
    }
}

/* 协议命令：开环目标 + 启动/停止 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len)
{
    switch(data_type)
    {
        case Data_Target:
        {
            float value = 0;
            memcpy(&value, buf, len);
            if(value > 100.0f)  value = 100.0f;
            if(value < -100.0f) value = -100.0f;
            vq_target = value;
            char vqBuf[16];
            LT_FTOAT3(vqBuf, vq_target);
            ltm_commut_printf("OpenLoop Vq : %s %%\n", vqBuf);
            break;
        }
        case Data_CMD_Start:
            ltm_pwm_start();
            run_flag = 1;
            ltm_commut_printf("Start\n");
            ltm_commut_send(Data_CMD_Start, &run_flag, 1);
            break;
        case Data_CMD_Reset:
            /* 升级入口：软复位到 BootLoader（6s 窗口内上位机开始烧录） */
            ltm_commut_printf("Reset -> BootLoader\r\n");
            ltm_sys_reset();
            break;
        case Data_CMD_Stop:
            run_flag = 0;
            ltm_pwm_set_dutys(0.0f, 0.0f, 0.0f);
            ltm_pwm_stop();
            ltm_commut_printf("Stop\n");
            ltm_commut_send(Data_CMD_Stop, &run_flag, 1);
            break;
        default: break;
    }
}

/* CAN-FD 回显 */
void user_canfd_rxcall(void)
{
    static uint8_t rx_buf[64] = {0};
    uint16_t id = 0, len = 0;
    uint8_t res = ltm_canfd_recv(&id, rx_buf, &len);
    if (id == APP_UPGRADE_TRIGGER_ID) {
        ltm_sys_reset();                 /* CAN-FD 升级触发：软复位进 BootLoader */
    }
    if(res == 1)      ltm_canfd_send(id, rx_buf, len);
    else if(res == 2) ltm_can_send(id, rx_buf, len);
}
