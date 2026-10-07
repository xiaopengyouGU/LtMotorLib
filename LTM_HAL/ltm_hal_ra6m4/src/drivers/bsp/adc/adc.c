#include "adc.h"
#include "r_elc.h"			/* 引入 ELC 头文件 */
#include "r_elc_api.h"
#include <string.h>

/* 温度范围：10℃ ~ 90℃，2℃ 步进，共 41 点。*/
#define NTC_TABLE_LEN     41
#define NTC_TEMP_START   (10)
#define NTC_TEMP_STEP     2

#ifdef TEST_LOOP_TIME    
static void test_gpio_on(void)              /* 测试引脚拉高 */ 
{
    R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, TEST_GPIO_PORT_PIN, BSP_IO_LEVEL_HIGH);
}   

static void test_gpio_off(void)             /* 测试引脚拉低 */
{
    R_IOPORT_PinWrite(&IOPORT_CFG_CTRL, TEST_GPIO_PORT_PIN, BSP_IO_LEVEL_LOW);
}

#endif

/* 正点原子无刷驱动板，板载 NTC 热敏电阻的ADC采样输出值表（间隔2℃，10℃—90℃）：
 * 对应的 ADC 值（12位，0~4095），四舍五入取整 */
static const uint16_t ntc_raw_table[NTC_TABLE_LEN] = {
    /* 10℃ ~ 90℃，2℃ 步进 */
    840,  897,  956,  1016, 1077, 1139, 1202, 1265,
    1329, 1393, 1458, 1522, 1587, 1651, 1716, 1780,
    1843, 1906, 1968, 2029, 2090, 2149, 2207, 2264,
    2320, 2374, 2428, 2480, 2530, 2580, 2628, 2674,
    2720, 2764, 2806, 2848, 2888, 2927, 2964, 3001,
    3036
};

/* ADC 外设结构体 */
static adc_ctrl_t *adc_ctrl0 = &g_adc0_ctrl;
static adc_ctrl_t *adc_ctrl1 = &g_adc1_ctrl;
static adc_cfg_t const *adc_cfg0   = &g_adc0_cfg;
static adc_cfg_t const *adc_cfg1   = &g_adc1_cfg;
static adc_channel_cfg_t const *adc_chs0 = &g_adc0_channel_cfg;
static adc_channel_cfg_t const *adc_chs1 = &g_adc1_channel_cfg;
static elc_ctrl_t *elc_ctrl = &g_elc_ctrl;
static elc_cfg_t const *elc_cfg = &g_elc_cfg;

/* 回调函数指针 */
static void (*s_callback)(void) = NULL;
static int32_t _ntc_temp(uint16_t raw);          /* 查表法温度获取 */

/* ADC 采样结构体，0,1,2 ==> A,B,C */
typedef struct {
    int32_t  offset[3];     /* 三相零点偏置（原始值）*/       
    uint16_t curr_buf[3];   /* 三相电流采样（原始值）*/
    uint16_t temp_buf[3];   /* [0]: 母线电压, [1]: 驱动器温度, [2]: 电机温度 */
    uint16_t calib_tar;     /* 目标校准采样次数 */
    uint16_t calib_cnt;     /* 已校准采样次数 */
    uint8_t  calib_done;    /* 校准标志位 0：未校准、1校准完毕 */
} adc_sample_t;             

static adc_sample_t adc_sample;
static adc_sample_t *adc  = &adc_sample;

extern void adc_scan_cplt_callback(adc_callback_args_t *);  /* 三相电流采集完毕回调 */

static void _adc_init(adc_ctrl_t *ctrl, adc_cfg_t const *cfg, adc_channel_cfg_t const *chs); /* adc 配置 */

void adc_init(void)
{
    memset(adc, 0, sizeof(adc_sample_t));

    _adc_init(adc_ctrl0, adc_cfg0, adc_chs0);
    _adc_init(adc_ctrl1, adc_cfg1, adc_chs1);

    R_ELC_Open(elc_ctrl, elc_cfg);
    R_ELC_Enable(elc_ctrl);
}

void adc_calibrate_zero(uint16_t samples)           /* 校准零点（软件触发）*/
{
    memset((void *)adc, 0, sizeof(adc_sample_t));   /* tar 在 done 之前置零，可消除竞态 bug */
    adc->calib_tar  = samples;                      /* 更新校准采样次数目标值 */
}

/* 获取电流采样值，由AB相重构C相电流 */
void adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic)
{
    if (!Ia || !Ib || !Ic)                return;  /* 判空 */
    uint16_t *curr_buf = adc->curr_buf;
    int32_t  *offset   = adc->offset;
    /* 读取三相电流，扣除零点漂移，转换为电流值(Q15)：0 电流在 offset，单边满量程
     * 2048 count，左移 4 位即 1.0 pu = 32768；下桥臂采样，电流方向与占空比相反，取负 */
    int32_t ia = - (((int32_t)curr_buf[0] - offset[0]) << 4);
    int32_t ib = - (((int32_t)curr_buf[1] - offset[1]) << 4);
	
	if (!adc->calib_done) {        /* 校准完毕之前，获取的电流按 0 算 */
		*Ia = 0;
		*Ib = 0;
		*Ic = 0;						
	} else {
        *Ia = ia;
        *Ib = ib;
        *Ic = - (ia + ib);      /* 下桥臂采样、Ia+Ib+Ic=0：与前两相同号即为 Ic */
    }
}

void adc_get_temp(int32_t *motor_temp, int32_t *driver_temp)  /* 获取电机和驱动器温度（0.1℃）*/
{
    if (!motor_temp || !driver_temp)    return;
    /* 本驱动板未焊电机 NTC，temp_buf[2] 由驱动器通道复制，故两者同值；
     * 以后焊上电机 NTC，把 temp_buf[2] 送 _ntc_temp 即可 */
    *motor_temp  = _ntc_temp(adc->temp_buf[2]);
    *driver_temp = _ntc_temp(adc->temp_buf[1]);
}

void adc_get_vbus(int32_t *vbus)          /* 获取母线电压，Q15 标幺（1.0 = ADC 满量程母线）*/
{
    if (vbus)   *vbus = adc->temp_buf[0] << 3;      /* 12bit 0~4095 → Q15 0~32760 */
}

void adc_set_callback(void (*callback)(void))       /* 设置用户 ADC 扫描完毕回调（高频） */
{
    if (callback)     s_callback = callback;
}

/* B相电流（IB）扫描完毕回调函数，负责校准和数据搬运 */
void adc_scan_cplt_callback(adc_callback_args_t *p_arg) /* adc 扫描完毕回调 */
{   
    /* 软件设置上，中心PWM 下溢时刻触发 ADC 采样（对应下桥臂导通），每周期执行一次（20kHz） */
    if(p_arg->event != ADC_EVENT_SCAN_COMPLETE) 	return;      /* 非扫描完成中断，直接退出 */      
    #ifdef TEST_LOOP_TIME
        test_gpio_on();
    #endif
    adc_ctrl_t *ctrl = adc_ctrl0;
    uint16_t *curr_buf = adc->curr_buf;
    uint16_t *temp_buf = adc->temp_buf;
    R_ADC_Read(ctrl,      ADC_CURRENT_A_CHANNEL, &curr_buf[0]);   /* A相电流原始值 */
    R_ADC_Read(adc_ctrl1, ADC_CURRENT_B_CHANNEL, &curr_buf[1]);   /* B相电流原始值 */

    if (!adc->calib_done){           /* 未校准完毕 */    
        if(adc->calib_tar) {         /* 调用了校准API, 手动启动校准 */
            int32_t *offset = adc->offset;
            uint16_t count = (++(adc->calib_cnt));
            offset[0] += (int32_t)curr_buf[0];
            offset[1] += (int32_t)curr_buf[1];
            if (count >= adc->calib_tar) {
                offset[0] /= count;
                offset[1] /= count;
                adc->calib_done = 1;  /* 校准完毕 */
            }
        }
    } else {      /* 校准完毕，调用用户回调函数 */
        if (s_callback)  s_callback();          
    }
    /* 最后搬运非电流量，对精度要求不高 */
    R_ADC_Read(ctrl, ADC_VBUS_CHANNEL,        &temp_buf[0]);  /* 母线电压原始值 */
    R_ADC_Read(ctrl, ADC_TEMP_DRIVER_CHANNEL, &temp_buf[1]);  /* 驱动器温度原始值 */
    // R_ADC_Read(ctrl, ADC_TEMP_MOTOR_CHANNEL,  &temp_buf[2]);  /* 电机温度原始值 */
    #ifdef TEST_LOOP_TIME
        test_gpio_off();
    #endif
}

/***************************************************************************************/
static int32_t _ntc_temp(uint16_t raw)               /* 查表法温度获取，返回 0.1℃ */
{
    /* 边界检查 */
    if (raw <= ntc_raw_table[0])
        return NTC_TEMP_START * 10;
    if (raw >= ntc_raw_table[NTC_TABLE_LEN - 1])
        return (NTC_TEMP_START + (NTC_TABLE_LEN - 1) * NTC_TEMP_STEP) * 10;
    /* 二分定位：找到 raw 所在区间 [lo, hi]，lo < hi */
    int lo = 0, hi = NTC_TABLE_LEN - 1;
    int32_t span, num;
    while (hi - lo > 1) {    /* hi - lo > 1，确保退出时 lo < hi */
        int mid = (lo + hi) >> 1;
        if (raw >= ntc_raw_table[mid])  lo = mid;
        else                            hi = mid;
    }
    /* 区间内线性插值，全程整数（0.1℃ 单位）*/
    span = (int32_t)ntc_raw_table[hi] - (int32_t)ntc_raw_table[lo];
    num  = ((int32_t)raw - (int32_t)ntc_raw_table[lo]) * (NTC_TEMP_STEP * 10);
    return (NTC_TEMP_START + lo * NTC_TEMP_STEP) * 10 + num / span;
}

static void _adc_init(adc_ctrl_t *ctrl, adc_cfg_t const *cfg, adc_channel_cfg_t const *chs) /* adc 配置 */
{
    /* 校准接口无效，强行使用会导致程序卡死 */
    R_ADC_Open(ctrl, cfg);                  /* 打开 ADC 外设 */
    R_ADC_ScanCfg(ctrl, chs);               /* 配置 ADC 扫描组 */
    R_ADC_ScanStart(ctrl);                  /* 启动 ADC */
}
