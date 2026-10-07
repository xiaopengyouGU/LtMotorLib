/*
 * LTM_FOC 胶水层 + LTM_CTRL 开环示例（CAN-FD 承载 LTM 协议）
 * - 底层：LTM_HAL 静态库（lib/libLTM_HAL.a + lib/ltm_hal/ltm_hal.h）
 * - 控制：LTM_FOC 胶水层（lib/libLTM_FOC.a，已内含 LTM_CTRL 算法）
 *         lt_motor_init() 一键完成：HAL 初始化 + 电流零点校准 + 状态机 + 25kHz 三环回调绑定
 * - 通讯：LTM 协议（ltm_commut，经 CAN-FD 0x100 承载，串口仅保留接收）
 * - 开环：Data_Target 下发 Vq 占空比(%)，SVPWM 调制
 * - 上报：10 条曲线（A/B/C 电流、母线电压、位置、速度、Id/Iq、驱动/电机温度）
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ltm_hal/ltm_hal.h"
#include "ltm_foc/lt_motor.h"
#include "protocol/ltm_commut.h"
#include "util/lt_str.h"

/* CAN-FD 升级触发 ID：与 BootLoader 一致，上位机发 0x7F0 帧 -> App 软复位进 BootLoader */
#define APP_UPGRADE_TRIGGER_ID  0x7F0

/* LTM-over-CANFD：LTM 接收协议帧经 CAN-FD 0x100 承载（载荷 = LTM 帧字节流，>64B 分片）；
 * 发送协议帧的 ID 区分开，避免总线上挂载多电机时重复触发 */
#define LTM_CANFD_SEND_ID       0x101
#define LTM_CANFD_DATA_ID       0x100

/* LTM-over-CANFD 发送适配：帧字节流按 64B 切片成 CAN-FD 帧 */
static void ltm_canfd_send_adapter(uint8_t *datas, uint16_t len)
{
    uint16_t off = 0;
    while (off < len) {
        uint16_t chunk = (uint16_t)((len - off > 64) ? 64 : (len - off));
        ltm_canfd_send(LTM_CANFD_SEND_ID, datas + off, chunk);
        off += chunk;
    }
}

void user_func(uint8_t data_type, uint8_t *buf, uint16_t len);
void user_canfd_rxcall(void);

int main(void)
{
    /* 胶水层一键初始化：HAL 外设 + 电流零点校准 + 状态机 + 25kHz 三环回调绑定 */
    lt_motor_init();

    /* LTM 通讯通道（LTM-over-CANFD：0x100 承载），串口仅保留接收 */
    ltm_commut_init();
    ltm_commut_set_send(ltm_canfd_send_adapter);
    ltm_uart_set_rxcall(ltm_commut_recv);
    ltm_canfd_set_rxcall(user_canfd_rxcall);

    /* 默认开环模式：Data_Target 下发占空比 % */
    lt_motor_set(Mode_Open_Loop, 0.0f);

    ltm_led_set(LTM_LED_ON_OFF, 1);
    ltm_led_set(LTM_LED_STOP, 1);
    ltm_led_set(LTM_LED_RUN, 0);

    /* 停转偏置自检：打印校准后三相机电流均值（ADC 校准已在 lt_motor_init 内部完成） */
    /* ra6m4 HAL 的电流接口是 Q15 标幺（32767 = ADC 满量程），换算成安培显示 */
    int32_t sIa = 0, sIb = 0, sIc = 0;
    int32_t dIa, dIb, dIc;
    for (int i = 0; i < 500; i++) {
        ltm_adc_get_current(&dIa, &dIb, &dIc);
        sIa += dIa; sIb += dIb; sIc += dIc;
        ltm_delay_ms(2);
    }
    char fbuf[3][16];
    LT_FTOAT3(fbuf[0], (float)sIa / 500.0f / 32767.0f * LTM_ADC_FULL_SCALE_CURRENT_A);
    LT_FTOAT3(fbuf[1], (float)sIb / 500.0f / 32767.0f * LTM_ADC_FULL_SCALE_CURRENT_A);
    LT_FTOAT3(fbuf[2], (float)sIc / 500.0f / 32767.0f * LTM_ADC_FULL_SCALE_CURRENT_A);
    ltm_commut_printf("standstill mean A: %s A\r\n", fbuf[0]);
    ltm_commut_printf("standstill mean B: %s A\r\n", fbuf[1]);
    ltm_commut_printf("standstill mean C: %s A\r\n", fbuf[2]);
    ltm_commut_printf("LTM_FOC open-loop ready!\r\n");

    lt_motor_info_t info;
    uint8_t data_type, data[128];
    uint16_t data_len;
    uint32_t count = 0;

    while (1)
    {
        /* 协议解析 + 命令分发 */
        if (ltm_commut_process(&data_type, data, &data_len)) {
            user_func(data_type, data, data_len);
        }

        lt_motor_get_info(&info);

        /* LED 指示：Running -> RUN 亮；否则 STOP 亮 */
        if (info.state == State_Running) {
            ltm_led_set(LTM_LED_STOP, 0);
            ltm_led_set(LTM_LED_RUN, 1);
        } else {
            ltm_led_set(LTM_LED_STOP, 1);
            ltm_led_set(LTM_LED_RUN, 0);
        }

        /* 10 条曲线：A/B/C 电流、母线电压、位置、速度、Id/Iq、驱动/电机温度 */
        ltm_curves curves;
        curves.size = 10;
        curves.values[0] = info.Ia;
        curves.values[1] = info.Ib;
        curves.values[2] = info.Ic;
        curves.values[3] = info.vbus;
        curves.values[4] = info.pos;
        curves.values[5] = info.speed;
        curves.values[6] = info.speed_pll;
        curves.values[7] = info.Id;
        curves.values[8] = info.Iq;
        curves.values[9] = info.driver_temp;
        ltm_commut_send_curves(&curves);

        ltm_delay_us(900);
        count++;
    }
}

/* 协议命令：开环目标 + 启动/停止/复位/文本回显 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len)
{
    uint8_t tmp;
    switch (data_type)
    {
        case Data_CMD_Text:                                 /* 文本指令：原样回显给上位机 */
            ltm_commut_send(Data_CMD_Text, buf, len);
            break;
        case Data_Target:                                   /* 开环目标：占空比 % */
        {
            float value = 0;
            if (len > sizeof(value)) len = sizeof(value);
            memcpy(&value, buf, len);
            if (value > 100.0f)  value = 100.0f;
            if (value < -100.0f) value = -100.0f;
            lt_motor_set(Mode_Open_Loop, value);
            char vqBuf[16];
            LT_FTOAT3(vqBuf, value);
            ltm_commut_printf("OpenLoop Vq : %s %%\n", vqBuf);
            break;
        }
        case Data_CMD_Start:
            lt_motor_enable();                              /* 使能：PWM 上电，进入 Enable */
            lt_motor_run();                                 /* 运行：进入 Running，25kHz 电流环输出 */
            ltm_commut_send(Data_CMD_Start, &tmp, 1);
            ltm_commut_printf("Start\n");
            break;
        case Data_CMD_Stop:
            lt_motor_stop(0);                                /* 停机：0 占空比 */
            lt_motor_disable();                             /* 失能：关闭 PWM */
            ltm_commut_send(Data_CMD_Stop, &tmp, 1);
            ltm_commut_printf("Stop\n");
            break;
        case Data_CMD_Reset:                                /* 升级入口：软复位到 BootLoader（6s 窗口） */
            ltm_commut_printf("Reset -> BootLoader\r\n");
            ltm_sys_reset();
            break;
        default: break;
    }
}

/* CAN-FD 接收分发（中断快进快出，不做回显）：
 * 0x7F0 = 升级触发软复位；0x100 = LTM-over-CANFD 载荷直接丢进协议环形缓冲 */
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
