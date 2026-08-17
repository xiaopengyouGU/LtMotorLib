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

#define ABS(x)          (((x) >= 0.0f) ? (x) : (-x)) 

/* 回调函数指针 */
static void (*s_callback)(void) = NULL;

/* ADC 采样结构体，0,1,2 ==> A,B,C */
typedef struct {
    float offset[3];        /* 三相零点偏置（原始值）*/
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

/* 获取电流采样值，三相采样，由两相（大电流）重构剩余相电流 */
void adc_get_current(float *Ia, float *Ib, float *Ic)
{
    if (!Ia || !Ib || !Ic)                return;  /* 判空 */
    adc_sample_t *adc  = &adc_sample;              /* 指针调用，直接进寄存器，性能更好 */
    uint16_t *curr_buf = adc->curr_buf;
    float    *offset   = adc->offset;
    /* 读取 三相电流原始值 */
    uint16_t a_raw = curr_buf[0];             /* A 相电流原始值 */
    uint16_t b_raw = curr_buf[1];             /* B 相电流原始值 */
    uint16_t c_raw = curr_buf[2];             /* C 相电流原始值 */
	
	if(!adc->calib_done){          /* 校准完毕之前，获取的电流按 0 算 */
		*Ia = 0;
		*Ib = 0;
		*Ic = 0;						
		return;
	}
    /* 扣除零点漂移，转换为电流值 (A)，注意是下桥臂采样，因此加个负号 */
    float ia =  - ((float)a_raw - offset[0]) * ADC_CURRENT_PER_LSB;
    float ib =  - ((float)b_raw - offset[1]) * ADC_CURRENT_PER_LSB;
    float ic =  - (ia + ib);
    // float ic =  - ((float)c_raw - offset[2]) * ADC_CURRENT_PER_LSB;
    /* 某相电流在过零点附近，信噪比过低，误差大，可由剩余两相电流重构 */
    // float abs_a = ABS(ia);
    // float abs_b = ABS(ib);
    // float abs_c = ABS(ic);
    
    // if (abs_a <= abs_b && abs_a <= abs_c) {         /* 重构 A 相电流 */
    //     ia = -(ib + ic);
    // } else if (abs_b <= abs_a && abs_b <= abs_c) {  /* 重构 B 相电流 */
    //     ib = -(ia + ic);
    // } else {                                        /* 重构 C 相电流 */      
    //     ic = -(ia + ib);
    // }
    /* 返回重构后的三相电流 */
    *Ia = ia;
    *Ib = ib;
    *Ic = ic;
}

void adc_get_temp(float *motor_temp, float *driver_temp)  /* 获取电机和驱动器温度 */
{
    if (!motor_temp || !driver_temp)    return;
    adc_sample_t *adc  = &adc_sample;                     /* 指针调用，直接进寄存器，性能更好 */
    uint16_t motor_raw  = adc->temp_buf[1];
    uint16_t driver_raw = adc->temp_buf[2];
    
    /* 转换为温度值和母线电压 */
    *motor_temp  = (float)motor_raw  * ADC_TEMP_PER_LSB;
    *driver_temp = (float)driver_raw * ADC_TEMP_PER_LSB;
}

void adc_get_vbus(float *vbus)          /* 获取母线电压（V) */        
{
    if (vbus)   *vbus = (float)(adc_sample.temp_buf[0]) * ADC_VBUS_PER_LSB;
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
                float *offset = adc->offset;
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
