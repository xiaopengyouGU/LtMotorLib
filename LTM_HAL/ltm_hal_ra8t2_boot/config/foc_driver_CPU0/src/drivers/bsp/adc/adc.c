#include "adc.h"
#include "r_elc.h"			/* 引入 ELC 头文件 */
#include "r_elc_api.h"
#include <string.h>

/* ADC 外设结构体 */
static adc_ctrl_t * adc_ctrl = &g_adc0_ctrl;
static adc_cfg_t  * adc_cfg  = &g_adc0_cfg;
static adc_b_scan_cfg_t *adc_scan_cfg = &g_adc0_scan_cfg;
static elc_ctrl_t * elc_ctrl = &g_elc_ctrl;
static elc_cfg_t  * elc_cfg  = &g_elc_cfg;

#define ABS(x)          (((x) >= 0.0f) ? (x) : (-x)) 

/* 回调函数指针 */
static void (*s_current_callback)(void) = NULL;

/* ADC 采样结构体，0,1,2 ==> U,V,W */
typedef struct {
    float offset[3];        /* 三相零点偏置（原始值）*/
    uint16_t curr_buf[3];   /* 三相电流采样（原始值）*/
    uint16_t temp_buf[3];   /* [0]: 母线电压, [1]: 电机温度, [2]: 驱动器温度（原始值）*/
    uint16_t calib_tar;     /* 目标校准采样次数 */
    uint16_t calib_cnt;     /* 已校准采样次数 */
    uint8_t  calib_done;    /* 校准标志位 0：未校准、1校准完毕 */
} adc_sample_t;

static adc_sample_t adc_sample;
static adc_sample_t * adc = &adc_sample;        /* 指针调用，性能更好 */

extern void adc_scan_cplt_callback(adc_callback_args_t *);  /* 三相电流采集完毕回调 */

void adc_init(void)                             /* ADC 初始化 */
{
    memset(adc, 0, sizeof(adc_sample_t));       /* 清空采样结构体 */

    R_ADC_B_CallbackSet(adc_ctrl, adc_scan_cplt_callback, NULL, NULL);  	/* 先设置采集完毕回调函数 */
    R_ADC_B_Open(adc_ctrl, adc_cfg);            /* 打开 ADC 外设 */
    R_ADC_B_ScanCfg(adc_ctrl, adc_scan_cfg);    /* ADC 通道配置 */
    //R_ADC_Calibrate(adc_ctrl, NULL);          /* 硬件 ADC 校准, 可以略过 */
    R_ADC_B_ScanGroupStart(adc_ctrl, ADC_GROUP_MASK_0 | ADC_GROUP_MASK_1);   /* ADC 组0和组1启动 */  
	
	/* ELC 配置 */
	R_ELC_Open(elc_ctrl, elc_cfg);				/* 打开 ELC，配置好 GPT 计数器下溢 触发 ADC 采样 */
	R_ELC_Enable(elc_ctrl);						/* 使能 ELC, 并运行 */
}


void adc_calibrate_zero(uint16_t samples)       /* 校准零点（阻塞模式，软件触发）*/
{
    if(adc->calib_done || adc->calib_tar) return;
    adc->calib_tar = samples;                   /* 更新校准采样次数目标值 */
}

/* 获取电流采样值，三相采样，由两相（大电流）重构剩余相电流 */
void adc_get_current(adc_current_t *current)
{
    if (current == NULL) return;

    /* 读取 U V相电流原始值 */
    uint16_t u_raw = adc->curr_buf[0];             /* U 相电流原始值 */
    uint16_t v_raw = adc->curr_buf[1];             /* V 相电流原始值 */
    uint16_t w_raw = adc->curr_buf[2];             /* W 相电流原始值 */
	
	if(!adc->calib_done){          /* 校准完毕之前，获取的电流按 0 算 */
		current->iu = 0;
		current->iv = 0;
		current->iw = 0;						
		return;
	}
    /* 扣除零点漂移，转换为电流值 (A)，注意是下桥臂采样，因此加个负号 */
    float iu =  - ((float)u_raw - adc->offset[0]) * ADC_CURRENT_PER_LSB;
    float iv =  - ((float)v_raw - adc->offset[1]) * ADC_CURRENT_PER_LSB;
    float iw =  - ((float)w_raw - adc->offset[2]) * ADC_CURRENT_PER_LSB;
    /* 某相电流在过零点附近，信噪比过低，误差大，可由剩余两相电流重构 */
    float abs_u = ABS(iu);
    float abs_v = ABS(iv);
    float abs_w = ABS(iw);
    
    if (abs_u <= abs_v && abs_u <= abs_w) {         /* 重构 U 相电流 */
        iu = -(iv + iw);
    } else if (abs_v <= abs_u && abs_v <= abs_w) {  /* 重构 V 相电流 */
        iv = -(iu + iw);
    } else {                                        /* 重构 W 相电流 */      
        iw = -(iu + iv);
    }
    /* 返回重构后的三相电流 */
    current->iu = iu;
    current->iv = iv;
    current->iw = iw;
}

/* 获取电机温度、驱动器温度、母线电压 */
void adc_get_temp_vbus(adc_temp_vbus_t *temp)
{
    if (temp == NULL) return;
    
    uint16_t vbus_raw    = adc->temp_buf[0];
    uint16_t motor_raw   = adc->temp_buf[1];
    uint16_t driver_raw  = adc->temp_buf[2];
    
    /* 转换为温度值和母线电压 */
    temp->motor_temp  = (float)motor_raw  * ADC_TEMP_PER_LSB;   
    temp->driver_temp = (float)driver_raw * ADC_TEMP_PER_LSB;
    temp->vbus        = (float)vbus_raw   * ADC_VBUS_PER_LSB;     
}

void adc_set_current_callback(void (*callback)(void))   /* 设置电流环回调 */
{
    if(!callback)       return;         /* 判空 */
    s_current_callback = callback;
}

/* ADC 扫描完毕回调函数，负责校准和数据搬运 */
void adc_scan_cplt_callback(adc_callback_args_t *p_arg)  /* adc 扫描完毕回调 */
{   
    /* 软件设置上，中心PWM 下溢时刻触发 ADC 采样（对应下桥臂导通？），每周期执行一次（20kHz） */
    if(p_arg->event != ADC_EVENT_SCAN_COMPLETE) 	return;                	/* 非扫描完成中断，直接退出 */
    
	if(p_arg->group_mask == ADC_GROUP_MASK_0){							  	/* 组0 扫描完毕 */
		R_ADC_B_Read(adc_ctrl, ADC_CURRENT_U_CHANNEL, &adc->curr_buf[0]);   /* U相电流原始值 */
		R_ADC_B_Read(adc_ctrl, ADC_CURRENT_V_CHANNEL, &adc->curr_buf[1]);   /* V相电流原始值 */
        R_ADC_B_Read(adc_ctrl, ADC_CURRENT_W_CHANNEL, &adc->curr_buf[2]);   /* W相电流原始值 */
			
		if(!adc->calib_done){           /* 未校准完毕 */    
			if(adc->calib_tar){         /* 调用了校准API, 手动启动校准 */
				uint16_t count = (++(adc->calib_cnt)); 
                for(int i = 0; i < 3; i++){
                    adc->offset[i] += adc->curr_buf[i];
                }
				if(count >= adc->calib_tar){
                    for(int i = 0; i < 3; i++){
                        adc->offset[i] /= count;
                    }
                    adc->calib_done = 1;  /* 校准完毕 */
				}
			}
		}else{                      /* 校准完毕，启动电流环回调 */
			if(s_current_callback)  s_current_callback();               	 /* 调用电流环回调 */
		}
	}else if(p_arg->group_mask == ADC_GROUP_MASK_1){						 /* 组1 扫描完毕 */
		R_ADC_B_Read(adc_ctrl, ADC_VBUS_CHANNEL,        &adc->temp_buf[0]);  /* 母线电压原始值 */
        R_ADC_B_Read(adc_ctrl, ADC_TEMP_MOTOR_CHANNEL,  &adc->temp_buf[1]);  /* 电机温度原始值 */
		R_ADC_B_Read(adc_ctrl, ADC_TEMP_DRIVER_CHANNEL, &adc->temp_buf[2]);  /* 驱动器温度原始值 */
	}
}