#include "adc.h"
#include "r_elc.h"			/* 引入 ELC 头文件 */
#include "r_elc_api.h"
#include <string.h>

/* ADC 外设结构体 */
static adc_ctrl_t *adc_ctrl = &g_adc0_ctrl;
static adc_cfg_t const *adc_cfg = &g_adc0_cfg;
static adc_b_scan_cfg_t const *adc_scan_cfg = &g_adc0_scan_cfg;
static elc_ctrl_t *elc_ctrl = &g_elc_ctrl;
static elc_cfg_t const *elc_cfg = &g_elc_cfg;

/* 回调函数指针 */
static void (*s_callback)(void) = NULL;

/* ADC 采样结构体，0,1,2 ==> A,B,C */
typedef struct {
    int32_t offset[3];      /* 三相零点偏置（原始计数）*/
    uint16_t curr_buf[3];   /* 三相电流采样（原始值）*/
    uint16_t temp_buf[3];   /* [0]: 母线电压, [1]: 电机温度, [2]: 驱动器温度（原始值）*/
    uint16_t calib_tar;     /* 目标校准采样次数 */
    uint16_t calib_cnt;     /* 已校准采样次数 */
    uint8_t  calib_done;    /* 校准标志位 0：未校准、1校准完毕 */
} adc_sample_t;

static adc_sample_t adc_sample;

extern void adc_scan_cplt_callback(adc_callback_args_t *);  /* 三相电流采集完毕回调 */

void adc_init(void)                               /* ADC 初始化 */
{
    memset(&adc_sample, 0, sizeof(adc_sample_t)); /* 清空采样结构体 */

    R_ADC_B_CallbackSet(adc_ctrl, adc_scan_cplt_callback, NULL, NULL);  	/* 先设置采集完毕回调函数 */
    R_ADC_B_Open(adc_ctrl, adc_cfg);              /* 打开 ADC 外设 */
    R_ADC_B_ScanCfg(adc_ctrl, adc_scan_cfg);      /* ADC 通道配置 */
    /* 开启 ADC 硬件校准，校准完成中断要使能，否则内部状态机不会动，直接卡死了 */
    R_ADC_B_Calibrate(adc_ctrl, NULL);            
	adc_status_t adc_status;
	do {
		R_ADC_B_StatusGet(adc_ctrl, &adc_status);
	} while (ADC_STATE_IDLE != adc_status.state);

    R_ADC_B_ScanGroupStart(adc_ctrl, ADC_GROUP_MASK_0 | ADC_GROUP_MASK_1);   /* ADC 组0和组1启动 */  
	
	/* ELC 配置 */
	R_ELC_Open(elc_ctrl, elc_cfg);				/* 打开 ELC，配置好 GPT 计数器下溢 触发 ADC 采样 */
	R_ELC_Enable(elc_ctrl);						/* 使能 ELC, 并运行 */
}


void adc_calibrate_zero(uint16_t samples)       /* 校准零点（阻塞模式，软件触发）*/
{
    adc_sample_t *adc = &adc_sample;            /* 指针调用，直接进寄存器，性能更好 */
    if(adc->calib_done || adc->calib_tar) return;
    adc->calib_tar = samples;                   /* 更新校准采样次数目标值 */
}

/* 温度范围：10℃ ~ 90℃，2℃ 步进，共 41 点。表沿用 g474 板那套
 * NCP18XH103F03RB（10k 下拉分压、12bit、3.3V）——ra8t2 板的 NTC/分压若不同，只换这张表 */
#define NTC_TABLE_LEN     41
#define NTC_TEMP_START   (10)
#define NTC_TEMP_STEP     2

static const uint16_t ntc_raw_table[NTC_TABLE_LEN] = {
    1467, 1544, 1621, 1699, 1777, 1855, 1933, 2010,
    2086, 2162, 2236, 2310, 2382, 2452, 2520, 2587,
    2652, 2715, 2776, 2835, 2892, 2948, 3001, 3052,
    3101, 3147, 3192, 3234, 3274, 3313, 3350, 3385,
    3419, 3451, 3481, 3510, 3538, 3564, 3589, 3613,
    3635,
};

static int32_t _ntc_temp(uint16_t raw)          /* 查表法温度读取，返回 0.1℃ */
{
    if (raw <= ntc_raw_table[0])                    return NTC_TEMP_START * 10;
    if (raw >= ntc_raw_table[NTC_TABLE_LEN - 1])    return (NTC_TEMP_START + (NTC_TABLE_LEN - 1) * NTC_TEMP_STEP) * 10;
    int lo = 0, hi = NTC_TABLE_LEN - 1;
    while (hi - lo > 1) {                           /* 二分定位 raw 所在区间 */
        int mid = (lo + hi) >> 1;
        if (raw >= ntc_raw_table[mid])  lo = mid;
        else                            hi = mid;
    }
    int32_t span = (int32_t)ntc_raw_table[hi] - (int32_t)ntc_raw_table[lo];
    int32_t num  = ((int32_t)raw - (int32_t)ntc_raw_table[lo]) * (NTC_TEMP_STEP * 10);
    return (NTC_TEMP_START + lo * NTC_TEMP_STEP) * 10 + num / span;
}
/* 获取电流采样值（Q15 标幺），三相采样，由两相（大电流）重构剩余相电流 */
void adc_get_current(int32_t *Ia, int32_t *Ib, int32_t *Ic)
{
    if (!Ia || !Ib || !Ic)                return;  /* 判空 */
    uint16_t *curr_buf = adc_sample.curr_buf;
    int32_t  *offset   = adc_sample.offset;

    if (!adc_sample.calib_done) {          /* 校准完毕之前，获取的电流按 0 算 */
        *Ia = 0;
        *Ib = 0;
        *Ic = 0;
        return;
    }
    /* 三相各自扣零点转 Q15：16bit 中位偏置，半量程 32768 LSB 正好映射到 32768（1.0 pu = 41.25A）；
     * 下桥臂采样所以取负，C 相由 A/B 两相重构（C 相信噪比低）*/
    int32_t ia = -((int32_t)curr_buf[0] - offset[0]);
    int32_t ib = -((int32_t)curr_buf[1] - offset[1]);
    int32_t ic = -(ia + ib);
    *Ia = ia;
    *Ib = ib;
    *Ic = ic;
}

void adc_get_temp(int32_t *motor_temp, int32_t *driver_temp)  /* 获取电机和驱动器温度（0.1℃）*/
{
    if (!motor_temp || !driver_temp)    return;
    uint16_t *temp_buf = adc_sample.temp_buf;
    *motor_temp  = _ntc_temp(temp_buf[1]);      /* 电机温度 */
    *driver_temp = _ntc_temp(temp_buf[2]);      /* 驱动器温度 */
}

void adc_get_vbus(int32_t *vbus)        /* 获取母线电压，Q15 标幺（1.0 = 满量程 95.70V）*/
{
    if (vbus)   *vbus = adc_sample.temp_buf[0] << 3;    /* 12bit 0~4095 -> Q15 0~32760 */
}

void adc_set_callback(void (*callback)(void))   /* 设置用户 ADC 扫描完毕回调（高频） */
{
    if (callback)     s_callback = callback;
}

/* 系统 ADC 扫描完毕回调函数，负责校准和数据搬运 */
void adc_scan_cplt_callback(adc_callback_args_t *p_arg)  /* adc 扫描完毕回调 */
{   
    /* 软件设置上，中心PWM 下溢时刻触发 ADC 采样（对应下桥臂导通？），每周期执行一次（20kHz） */
    if(p_arg->event != ADC_EVENT_SCAN_COMPLETE) 	return;                	/* 非扫描完成中断，直接退出 */
    /* 指针调用，局部变量直接进寄存器，性能更好 */
    adc_sample_t *adc  = &adc_sample;      
    adc_ctrl_t   *ctrl = adc_ctrl;
    if(p_arg->group_mask == ADC_GROUP_MASK_0){					   /* 组0 扫描完毕 */
		uint16_t *curr_buf = adc->curr_buf;
        R_ADC_B_Read(ctrl, ADC_CURRENT_A_CHANNEL, &curr_buf[0]);   /* A相电流原始值 */
		R_ADC_B_Read(ctrl, ADC_CURRENT_B_CHANNEL, &curr_buf[1]);   /* B相电流原始值 */
        R_ADC_B_Read(ctrl, ADC_CURRENT_C_CHANNEL, &curr_buf[2]);   /* C相电流原始值 */
			
		if(!adc->calib_done){           /* 未校准完毕 */    
			if(adc->calib_tar){         /* 调用了校准API, 手动启动校准 */
                int32_t *offset = adc->offset;
				uint16_t count = (++(adc->calib_cnt)); 
                for(int i = 0; i < 3; i++){
                    offset[i] += curr_buf[i];
                }
				if(count >= adc->calib_tar){
                    for(int i = 0; i < 3; i++){
                        offset[i] /= count;
                    }
                    adc->calib_done = 1;  /* 校准完毕 */
				}
			}
		}else{      /* 校准完毕，调用用户回调函数 */
			if(s_callback)  s_callback();          
		}
	}else if(p_arg->group_mask == ADC_GROUP_MASK_1){	            /* 组1 扫描完毕 */
        uint16_t *temp_buf = adc->temp_buf;
		R_ADC_B_Read(ctrl, ADC_VBUS_CHANNEL,        &temp_buf[0]);  /* 母线电压原始值 */
        R_ADC_B_Read(ctrl, ADC_TEMP_MOTOR_CHANNEL,  &temp_buf[1]);  /* 电机温度原始值 */
		R_ADC_B_Read(ctrl, ADC_TEMP_DRIVER_CHANNEL, &temp_buf[2]);  /* 驱动器温度原始值 */
	}
}
