/*
 * LTM_HAL 开环例程（LTM_HAL 各平台共用，板级由 boards/<board>/board.cmake 驱动）
 * - 底层：LTM_HAL 静态库（${LTM_HAL_SDK_DIR}/bin）
 * - 通信：LTM 协议（ltm_commut，基于 ltm_uart / ltm_canfd 接口）
 * - 控制：开环 FOC（Data_Target 下发 Vq 占空比百分比，SVPWM 调制）
 * - 上报：10 条曲线（A/B/C、α/β、母线电压、机械角度、Id/Iq、驱动板温度）
 *
 * 电流零点校准说明：
 *   校准的本质 = 在电流为 0 时对 ADC 原始值取平均作为零点偏置 offset，
 *   adc_get_current = (raw - offset) * 系数，校准完成前返回 0。
 *   两种可行的零电流状态：
 *     a) 三相 0% 占空比（全下桥导通）：无开关动作，电流严格为 0，采样最干净；
 *     b) 三相 50% 等占空比：线电压为 0，相电流为 0（本例采用，与旧固件一致）。
 *   前提：电机停转；ADC 扫描由定时器触发，与占空比无关。
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ltm_hal/ltm_hal.h"
#include "protocol/ltm_commut.h"
#include "control/foc/lt_foc.h"
#include "math/basic/lt_math.h"
#include "math/basic/lt_str.h"

/* ---- 机型参数：板级可覆盖（boards/<board>/board.cmake 的 LTM_BOARD_EXAMPLE_DEFINES）---- */
#ifndef POLE_PAIRS
#define POLE_PAIRS           7             /* 极对数（板级未给定时用此默认值）*/
#endif
#define Q15_MAX              32767         /* Q15 能表示的最大值 */
#define ZERO_FIND_DUTY_Q15   1638          /* 预定位 D 轴电压 0.05 pu（Q15）*/
#ifndef ENCODER_ZERO_OFFSET
#define ENCODER_ZERO_OFFSET  0.0f          /* 电角度零点 count；0 = 上电预定位实测 */
#endif
static float   vq_target = 0.0f;          /* 开环 Q 轴电压指令（%，-100~100）*/
static uint8_t run_flag  = 0;             /* 运行标志 */

/* CAN-FD 升级触发 ID：与 BootLoader 升级 ID 一致，上位机发该 ID 帧 → App 软复位进 BootLoader */
#define APP_UPGRADE_TRIGGER_ID  0x7F0

/* LTM-over-CANFD：LTM 接收协议帧经 CAN-FD 0x100 承载（载荷 = LTM 帧字节流，>64B 分片） */
/* 发送协议帧的ID需要进行区分，避免总线上挂载多电机时，重复触发 */
#define LTM_CANFD_SEND_ID   0x101
#define LTM_CANFD_DATA_ID   0x100

/* LTM-over-CANFD 发送适配：帧字节流按 64B 切片，每片一个 0x100 帧 */
static void ltm_canfd_send_adapter(uint8_t *datas, uint16_t len)
{
    uint16_t off = 0;
    while (off < len) {
        uint16_t chunk = (len - off > 64) ? 64 : (len - off);
        ltm_canfd_send(LTM_CANFD_SEND_ID, datas + off, chunk);
        off += chunk;
    }
}

void user_func(uint8_t data_type, uint8_t *buf, uint16_t len);
void user_canfd_rxcall(void);

int main(void)
{
    /* 底层外设统一初始化（系统/延时/串口/CAN/LED/PWM/编码器/ADC） */
    ltm_hal_init();

    /* LTM 通讯通道（经 CAN-FD 0x100 承载：LTM-over-CANFD），串口仅保留接收 */
    ltm_commut_init();
    ltm_commut_set_send(ltm_canfd_send_adapter);
    ltm_uart_set_rxcall(ltm_commut_recv);
    ltm_canfd_set_rxcall(user_canfd_rxcall);

    ltm_led_set(LTM_LED_STOP, 1);
    ltm_led_set(LTM_LED_RUN, 0);

    /* ===== 电流零点校准 =====
     * 三相 50% 等占空比 → 线电压 0 → 相电流 0（电机停转），
     * 对 5000 次 ADC 扫描取平均作为零点偏置（25kHz ≈ 200ms）。
     * 校准完成前 adc_get_current 返回 0。 */
    ltm_pwm_start();
    ltm_pwm_set_dutys(0, 0, 0);
    ltm_delay_ms(200);                   /* 等电流尖峰消除 */
    ltm_adc_calibrate_zero(5000);        /* 5000 次扫描平均 */
    ltm_delay_ms(500);                   /* 等校准完成 */
    ltm_commut_printf("ADC calib done! \r\n");

    /* ---- 调试：打印校准 offset + 停机偏置自检 ---- */
    int32_t sIa = 0, sIb = 0, sIc = 0, dIa, dIb, dIc;
    for (int i = 0; i < 500; i++) {
        ltm_adc_get_current(&dIa, &dIb, &dIc);
        sIa += dIa; sIb += dIb; sIc += dIc;
        ltm_delay_ms(2);
    }
    char fbuf[3][16];
    LT_FTOAT3(fbuf[0], sIa / 500.0f * LTM_ADC_CURRENT_PER_COUNT);
    LT_FTOAT3(fbuf[1], sIb / 500.0f * LTM_ADC_CURRENT_PER_COUNT);
    LT_FTOAT3(fbuf[2], sIc / 500.0f * LTM_ADC_CURRENT_PER_COUNT);
    ltm_commut_printf("standstill mean A: %s A\r\n", fbuf[0]);
    ltm_commut_printf("standstill mean B: %s A\r\n", fbuf[1]);
    ltm_commut_printf("standstill mean C: %s A\r\n", fbuf[2]);

    /* 开环 FOC：定点 SVPWM（components/control/foc）*/
    lt_foc_init();
    lt_foc_set(LTM_ENC_CPR);           /* 电角度单圈分辨率 = 编码器单圈计数 */

    ltm_curves curves;
    uint8_t data_type, data[128];
    uint16_t data_len;
    uint32_t count = 0;
    ltm_led_set(LTM_LED_STOP, 1);
    for (int i = 0; i < 80; i++) {
        ltm_enc_update();           /* 定时更新编码器 */
    }
    ltm_commut_printf("Hello Lvtou!!! \n");
    /* 电角度零点：板级给了固定值就直接用（绝对值编码器），否则上电预定位实测一次 */
    int32_t zero_count;                  /* 电角度零点（编码器单圈计数）*/
    if (ENCODER_ZERO_OFFSET > 0.0f) {
        zero_count = (int32_t)ENCODER_ZERO_OFFSET;
    } else {
        int32_t dutys[3];
        lt_foc_update(ZERO_FIND_DUTY_Q15, 0, 0, dutys);  /* D 轴强拖到 0 电角度 */
        ltm_pwm_set_dutys(dutys[0], dutys[1], dutys[2]);
        ltm_delay_ms(200);
        ltm_enc_update();
        zero_count = (int32_t)ltm_enc_get_count();
        ltm_pwm_set_dutys(0, 0, 0);
        ltm_delay_ms(50);
    }
    ltm_commut_printf("zero offset = %d\r\n", zero_count);

    while(1)
    {
        /* 协议命令处理 */
        if(ltm_commut_process(&data_type, data, &data_len)){
            user_func(data_type, data, data_len);
        }
        ltm_enc_update();           /* 定时更新编码器 */
        /* 开环控制：编码器电角度 → 定点 SVPWM → 三相占空比 */
        uint32_t the = (uint32_t)(((int32_t)ltm_enc_get_count() - zero_count) * POLE_PAIRS);
        if(run_flag){
            int32_t dutys[3];
            lt_foc_update(0, (int32_t)(vq_target * 0.01f * Q15_MAX), the, dutys);
            ltm_pwm_set_dutys(dutys[0], dutys[1], dutys[2]);
            ltm_led_set(LTM_LED_STOP, 0);
            ltm_led_set(LTM_LED_RUN, 1);
        }else{
            ltm_pwm_set_dutys(0, 0, 0);
            ltm_led_set(LTM_LED_STOP, 1);
            ltm_led_set(LTM_LED_RUN, 0);
        }
        if (count % 1500 == 0) {
            uint16_t id = 0x008;
            uint8_t tx_buf[16] = {0x11,0x22,0x33,0x44,0x56,0x78,0x90,0xAB,0xCD};
            ltm_canfd_send(id, tx_buf, 16);
        }
        /* 5 条曲线：A/B/C 相电流、母线电压、机械角度 */
        int32_t rawIa, rawIb, rawIc, rawVbus, rawMotor, rawDriver;
        ltm_adc_get_current(&rawIa, &rawIb, &rawIc);
        ltm_adc_get_vbus(&rawVbus);
        ltm_adc_get_temp(&rawMotor, &rawDriver);
        /* D/Q 电流用定点 Clarke+Park（与 lt_foc_update 共用同一 foc->step 口径）*/
        int32_t f3[3] = { rawIa, rawIb, rawIc };
        int32_t Id_q15, Iq_q15;
        int32_t fa = ((2 * rawIa - rawIb - rawIc) * 10923) >> 15;   /* α 轴（Q15）*/
        int32_t fb = ((rawIb - rawIc) * 18919) >> 15;               /* β 轴（Q15）*/
        lt_foc_clark_park(f3, the, &Id_q15, &Iq_q15);
        curves.size = 10;
        curves.values[0] = rawIa * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[1] = rawIb * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[2] = rawIc * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[3] = fa * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[4] = fb * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[5] = rawVbus * LTM_ADC_VOLTAGE_PER_COUNT;
        curves.values[6] = (float)ltm_enc_get_position() * (360.0f / LTM_ENC_CPR);
        curves.values[7] = Id_q15 * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[8] = Iq_q15 * LTM_ADC_CURRENT_PER_COUNT;
        curves.values[9] = rawDriver * LTM_ADC_TEMP_PER_COUNT;
        ltm_commut_send_curves(&curves);

        ltm_delay_us(900);
        count++;
    }
}

/* 协议命令：开环目标 + 启动/停止 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len)
{
    switch(data_type)
    {
        case Data_CMD_Text:                                 /* 文本指令：原样回显给上位机 */
            ltm_commut_send(Data_CMD_Text, buf, len);
            break;
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
            ltm_commut_send(Data_CMD_Start, &run_flag, 1);
            ltm_commut_printf("Start\n");
            break;
        case Data_CMD_Reset:
            /* 升级入口：软复位到 BootLoader（6s 窗口内上位机开始烧录） */
            ltm_commut_printf("Reset -> BootLoader\r\n");
            ltm_sys_reset();
            break;
        case Data_CMD_Stop:
            run_flag = 0;
            ltm_pwm_set_dutys(0, 0, 0);
            ltm_pwm_stop();
            ltm_commut_send(Data_CMD_Stop, &run_flag, 1);
            ltm_commut_printf("Stop\n");
            break;
        default: break;
    }
}

/* CAN-FD 接收分发（快进快出，不做回显）：
 * 0x7F0 = 升级触发软复位；0x100 = LTM-over-CANFD 载荷直接丢进协议缓冲 */
void user_canfd_rxcall(void)
{
    static uint8_t rx_buf[64] = {0};
    uint16_t id = 0, len = 0;
    if (ltm_canfd_recv(&id, rx_buf, &len) == 0)     return;
    if (id == APP_UPGRADE_TRIGGER_ID) {
        ltm_sys_reset();                 /* CAN-FD 升级触发：软复位进 BootLoader */
        return;
    } else if (id == LTM_CANFD_DATA_ID) {
        ltm_commut_recv(rx_buf, len);    /* 载荷进协议环形缓冲，user_func 统一处理 */
    }
}
