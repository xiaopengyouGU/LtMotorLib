/*
 * LTM_FOC 胶水层 + LTM_CTRL 开环示例（串口承载 LTM 协议）
 * - 底层：LTM_HAL 静态库（lib/libLTM_HAL.a + lib/ltm_hal/ltm_hal.h）
 * - 控制：LTM_FOC 胶水层（lib/libLTM_FOC.a，已内含 LTM_CTRL 算法）
 *         lt_motor_init() 一键完成：HAL 初始化 + 电流零点校准 + 上电预定位
 *         + 状态机 + 20kHz 三环回调绑定
 * - 通讯：LTM 协议（ltm_commut，经串口 UART0 承载）
 * - 开环：Data_Target 下发 Vq 占空比(%)，SVPWM 调制
 * - 上报：10 条曲线（A/B/C 电流、母线电压、驱动板温度、电机温度、机械角度、
 *          PLL测速、DQ轴电流）
 */
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "ltm_hal/ltm_hal.h"
#include "ltm_foc/lt_motor.h"
#include "protocol/ltm_commut.h"
#include "util/user_common.h"

void user_func(uint8_t data_type, uint8_t *buf, uint16_t len);

/* 状态 / 错误码 → 名字：串口一眼看出原因（未知值回落 "?"，后面仍带原始数字）*/
static const char *state_name(int s)
{
    switch (s) {
        case State_Init:    return "Init";
        case State_Idle:    return "Idle";
        case State_Enable:  return "Enable";
        case State_Running: return "Run";
        case State_Stop:    return "Stop";
        case State_Error:   return "Error";
        default:            return "?";
    }
}

static const char *err_name(int e)
{
    switch (e) {
        case LT_OK:                   return "OK";
        case LT_ERR_OVER_RANGE:       return "OVER_RANGE";
        case LT_ERR_UNSUPPORTED:      return "UNSUPPORTED";
        case LT_ERR_OVER_VOLT:        return "OVER_VOLT";
        case LT_ERR_UNDER_VOLT:       return "UNDER_VOLT";
        case LT_ERR_OVER_TEMP_DRIVER: return "OVER_TEMP_DRIVER";
        case LT_ERR_OVER_TEMP_MOTOR:  return "OVER_TEMP_MOTOR";
        case LT_ERR_OVER_CURRENT:     return "OVER_CURRENT";
        case LT_ERR_OVER_SPEED:       return "OVER_SPEED";
        default:                      return "?";
    }
}

int main(void)
{
    /* 胶水层一键初始化：HAL 外设 + 电流零点校准 + 上电预定位 + 状态机 + 25kHz 三环回调绑定 */
    lt_motor_init();

    /* LTM 通讯通道（串口 UART0 承载）*/
    ltm_commut_init();
    ltm_commut_set_send(ltm_uart_write);
    ltm_uart_set_rxcall(ltm_commut_recv);

    /* 默认开环模式：Data_Target 下发占空比 % */
    lt_motor_set(Mode_Open_Loop, 0.0f);

    ltm_led_set(LTM_LED_ON_OFF, 1);
    ltm_led_set(LTM_LED_STOP, 1);
    ltm_led_set(LTM_LED_RUN, 0);

    /* 停转偏置自检：打印校准后三相机电流均值（ADC 校准已在 lt_motor_init 内部完成） */
    int32_t sIa = 0, sIb = 0, sIc = 0;
    int32_t dIa, dIb, dIc;
    for (int i = 0; i < 500; i++) {
        ltm_adc_get_current(&dIa, &dIb, &dIc);
        sIa += dIa; sIb += dIb; sIc += dIc;
        ltm_delay_ms(2);
    }
    ltm_commut_printf("LTM_FOC ready! T=A / V=RPM / RV=RPM / P=deg / S / ES, H for help\r\n");

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

        ltm_curves curves;
        curves.size = 10;
        curves.values[0] = info.Ia;
        curves.values[1] = info.Ib;
        curves.values[2] = info.Ic;
        curves.values[3] = info.vbus;
        curves.values[4] = info.driver_temp;
        curves.values[5] = info.motor_temp;
        curves.values[6] = info.pos;
        curves.values[7] = info.speed_pll;
        curves.values[8] = info.Iq;
        curves.values[9] = info.Id;
        ltm_commut_send_curves(&curves);

        /* 定时上报电机状态 */
        if ((count % 2000) == 0) {
            /* 周期报状态：停机/跳保护时能直接从串口看出原因 */
            ltm_commut_printf("tick=%d state=%s(%d) err=%s(%d) \r\n",
                              (int)count, state_name((int)info.state), (int)info.state,
                              err_name((int)info.err), (int)info.err);
        }

        ltm_delay_us(750);
        count++;
    }
}

/* 指令帮助（H / ?）*/
static void user_help(void)
{   
    /* printf接口单次能打印字符数最多：128字节 */
    ltm_commut_printf("LTM_FOC CMD:\r\n"
                      "  T=<A>     torque mode (Iq, unit A)\r\n"
                      "  V=<RPM>   speed mode\r\n"
                      "  RV=<RPM>  ramp speed mode\r\n");
    ltm_commut_printf("  P=<deg>   position mode\r\n"
                      "  S         controlled stop (1500 RPM/s)\r\n"
                      "  ES        emergency stop (6000 RPM/s)\r\n"
                      "  H / ?     this help\r\n");
}

/* 模式切换辅助：解析参数 → 交给 lt_motor_set 判范围 → 按返回码回显 */
static void user_set_mode(uint8_t *buf, uint16_t len, uint8_t mode, const char *name)
{
    float val    = user_parse_float(buf + 2, (uint16_t)(len - 2));
    lt_err_t err = lt_motor_set((lt_motor_mode_t)mode, val);
    char tmpBuf[16];
    user_float3(val, tmpBuf);
    if (err != LT_OK) {
        ltm_commut_printf("%s target %s rejected: %s\r\n", name, tmpBuf, err_name((int)err));
    } else {
        ltm_commut_printf("%s target: %s\r\n", name, tmpBuf);
    }
}

/* 协议命令：开环目标 + 启动/停止/复位/文本回显 */
void user_func(uint8_t data_type, uint8_t *buf, uint16_t len)
{
    switch (data_type)
    {
        case Data_CMD_Text:                       /* 文本指令，见 user_help() */
            if (len >= 1 && (buf[0] == 'H' || buf[0] == '?')) {
                user_help();
            } else if (len >= 3 && buf[0] == 'R' && buf[1] == 'V' && buf[2] == '=') {
                user_set_mode(buf + 1, (uint16_t)(len - 1), Mode_Ramp_Speed, "Ramp speed");
            } else if (len >= 2 && buf[0] == 'E' && buf[1] == 'S') {   /* 急停 */
                lt_motor_stop(1);
                ltm_commut_printf("E-Stop (6000 RPM/s)\r\n");
            } else if (len >= 1 && buf[0] == 'S') {                    /* 受控停机 */
                lt_motor_stop(0);
                ltm_commut_printf("Stop (1500 RPM/s)\r\n");
            } else if (len >= 2 && buf[1] == '=') {
                switch (buf[0]) {
                    case 'T': user_set_mode(buf, len, Mode_Torque, "Torque Iq"); break;
                    case 'V': user_set_mode(buf, len, Mode_Speed,    "Speed");     break;
                    case 'P': user_set_mode(buf, len, Mode_Position, "Position");  break;
                }
            } else {
                ltm_commut_send(Data_CMD_Text, buf, len);   /* 其他文本原样回显 */
            }
            break;

        case Data_Target:                                   /* 开环目标：占空比 % */
        {
            float value;
            memcpy(&value, buf, sizeof(value));
            value = USER_CONSTRAINS(value, 100.0f, -100.0f);
            lt_motor_set(Mode_Open_Loop, value);
            char vqBuf[16];
            user_float3(value, vqBuf);
            ltm_commut_printf("OpenLoop Vq : %s %%\n", vqBuf);
            break;
        }

        case Data_CMD_Start:
            lt_motor_enable();                              /* 使能：PWM 上电，进入 Enable */
            lt_motor_run();                                 /* 运行：进入 Running，电流环输出 */
            ltm_commut_send(Data_CMD_Start, (uint8_t[]){0}, 1);
            ltm_commut_printf("Start\n");
            break;

        case Data_CMD_Stop:
            lt_motor_stop(0);                               /* 停机：0 占空比 */
            lt_motor_disable();                             /* 失能：关闭 PWM */
            ltm_commut_send(Data_CMD_Stop, (uint8_t[]){0}, 1);
            ltm_commut_printf("Stop\n");
            break;

        case Data_CMD_Reset:                                /* 升级入口：软复位到 BootLoader（6s 窗口） */
            ltm_commut_printf("Reset -> BootLoader\r\n");
            ltm_sys_reset();
            break;
        default:    break;
    }
}