
#include "motor_ctrl/tasks/control_tasks.h"     
#include "motor_ctrl/common/tasks_param_def.h"  /* 控制参数定义 */
#include "motor_ctrl/schedule/lt_fsm.h"         /* 状态机主要由外部模块维护 */
#include <string.h>
/* 外设层头文件 */
#include "bsp/led/led.h"
#include "bsp/tim/tim.h"
#include "bsp/adc/adc.h"
#include "bsp/encoder/encoder.h"

/* 组件层头文件: 控制算法 */
#include "control/pid/lt_pid.h"
#include "control/foc/lt_foc.h"
#include "control/speed/lt_speed.h"
#include "control/deadzone/lt_deadzone.h"
#include "math/basic/lt_math.h"

/* 测试用全局变量 */
static lt_foc_t       g_foc    = NULL;
static lt_pid_t       g_pid_id = NULL;
static lt_pid_t       g_pid_iq = NULL;
static lt_pid_t       g_pid_speed = NULL;
static lt_pid_t		  g_pid_pos   = NULL;			
static lt_motor_info_t motor_info_obj;             /* 电机信息结构体 */
static lt_motor_info_t * g_info = &motor_info_obj; /* 指针调用 */
/*==============================================================================
 * 电流环单次执行（完整模拟真实场景，含过流保护）
 *==============================================================================*/
static void    _get_dq_current(float *Id_actual, float *Iq_actual, float the);/* 获取 DQ轴电流 */
static void    _current_loop_run(float Id, float Iq, float the, float wr);    /* 运行 FOC 电流环 */
static uint8_t _find_absolute_zero(lt_motor_state_t state);      /* 电流环电角度绝对零点定位 */

void current_loop_task(void)                       /* 电流环任务：20kHz */
{
    // led_set(LED_RUN, 1);                             /* 由翻转电平判断电流环执行时间 */
    lt_motor_state_t state = lt_fsm_get();         /* 获取当前状态 */

    /* 读取电角度，由编码器原始计数值（0~9999）转换，后者天然带有归一化特性
     * 避免了长时间运行后，因机械角度值极大（仅在速度模式下可能出现）
	 * 导致的浮点数精度下降（约 1.34万圈，此时精度小于0.01 RAD）*/
    encoder_update();                              /* 先主动更新编码器计数值 */
    float angle_el = ((float)encoder_get_count() - ENCODER_OFFSET) * ANGLE_EL_PER_COUNT;	
	float the = lt_normalize(angle_el);			   /* 归一化到 [0,2*pi) */
    float Id_actual, Iq_actual;
    float wr  = lt_speed_get() * RPM_TO_WR;       /* 得到电角速度（rad/s），用于前馈解耦 */
    
    _get_dq_current(&Id_actual, &Iq_actual, the);  /* 获取 DQ 轴电流 */
    if(!_find_absolute_zero(state))        return; /* 还未找到绝对零点，跳过 FOC 电流环 */
    /* 电机状态和模式判断 */
    if(state != State_Running){                    
        foc_pwm_set_duty(0, 0, 0);                 /* 非运行模式，直接输出 0 占空比 */ 
        return;                                    /* 立马返回 */
    };                                             /* 空闲状态或初始化中 */

    _current_loop_run(Id_actual, Iq_actual, the, wr); /* FOC 电流环 ：20kHz */
    
    // led_set(LED_2, 0);                             /* 由翻转电平判断电流环执行时间 */
}

void speed_loop_task(void)                         /* 速度环任务：3kHz */
{
    int32_t count = (int32_t)encoder_get_count();
    lt_motor_mode_t mode   = g_info->mode;         /* 获取控制模式 */
    lt_motor_state_t state = lt_fsm_get();         /* 获取当前状态 */
	float motor_speed = lt_speed_update(count);	   /* 获取速度值: RPM */
    g_info->speed = motor_speed;                   /* 更新电机速度 */

    /* 电机状态和模式判断 */
    if(state != State_Running)                        return; /* 非运行模式，暂时直接跳过 */
    if(mode == Mode_Open_Loop || mode == Mode_Torque) return; /* 转距或开环模式，直接跳过 */
    float Iq_target = lt_pid_process(g_pid_speed, motor_speed);
    lt_pid_set_target(g_pid_iq, Iq_target);        /* 更新电流环目标值 */
}

void position_loop_task(void)                       /* 位置环任务：1kHz */
{
    float pos = encoder_get_angle_deg();            /* 角度(°) */
    adc_temp_vbus_t temp;
    adc_get_temp_vbus(&temp);                       /* 同时获取温度和母线电压 */

    lt_motor_mode_t mode   = g_info->mode;          /* 获取控制模式 */
    lt_motor_state_t state = lt_fsm_get();          /* 获取当前状态 */
    /* 更新电机位置、温度与母线电压 */
    g_info->pos = pos;                              
    g_info->driver_temp = temp.driver_temp;
    g_info->motor_temp  = temp.motor_temp;
    g_info->vbus        = temp.vbus;

    /* 电机状态和模式判断 */
    if(state != State_Running)    return;           /* 速度环仅在运行状态可以执行 */
    if(mode != Mode_Position)     return;           /* 非位置模式，直接跳过 */
    float speed_target = lt_pid_process(g_pid_pos, pos);
    lt_pid_set_target(g_pid_speed, speed_target);   /* 更新速度环目标值 */
}

/* 控制任务初始化 */
void control_tasks_init(void)
{
    memset(g_info, 0, sizeof(lt_motor_info_t));            /* 清零电机信息结构体 */         
    /* 创建控制对象 */
    g_foc = lt_foc_create(0, FOC_TYPE_SVPWM);
    g_pid_id  = lt_pid_create(0.5f, 0.02f, 0.0f, 0.05f);
    g_pid_iq  = lt_pid_create(0.5f, 0.02f, 0.0f, 0.05f);
	g_pid_pos   = lt_pid_create(0.02f, 0.5f, 0.0f, 1.0f);
    g_pid_speed = lt_pid_create(0.01f, 0.0f, 0.0f, 1.0f/3);
	/* 设置 PID 积分和输出限幅值 */
    lt_pid_set_limits(g_pid_id, 12*1.154f, 12*1.154f);      /* DQ轴电压给定母线（电压 24V）*/
    lt_pid_set_limits(g_pid_iq, 12*1.154f, 12*1.154f);      /* DQ轴电压给定母线（电压 24V）*/
    lt_pid_set_limits(g_pid_speed, 8, 8);                   /* 电流限幅：+-8A */
    lt_pid_set_limits(g_pid_pos, 3000, 3000);			    /* 速度限幅：+-3000RPM */
	/* 设置 PID 目标值 */
	lt_pid_set_target(g_pid_id, 0);
    lt_pid_set_target(g_pid_iq, 0);
    lt_pid_set_target(g_pid_speed, 0);
    lt_pid_set_target(g_pid_pos, 0);
	/* 初始化高精度测试模块 */
	lt_speed_init(ENCODER_CPR, 25000.0f);				    /* 25kHz 电流环 */
	/* 初始化死区补偿模块（电流极性法判别）*/
	lt_deadzone_init(0.006f, 0.1f, 0.30f);					/* 死区占比：1%，阈值电流：0.10A (额定6A), 滤波系数 0.30f */
}

void control_tasks_set(lt_motor_mode_t mode, float target)  /* 只有停机时，才能更改模式，在motor_api中确保 */
{
    if(mode != g_info->mode){
        g_info->mode = mode; 
    } 
    g_info->target = target;                        /* 更新目标值 */
}  

void control_tasks_get(lt_motor_info_t *info)       /* 获取电机信息 */
{
    if(!info)       return;                         /* 判空 */
    g_info->state  = lt_fsm_get();                  /* 获取电机状态 */
    memcpy(info, g_info, sizeof(lt_motor_info_t));  /* 直接拷贝即可 */
}

/* 获取编码器原始计数值（0~CPR-1）和电角度（rad）*/
void control_tasks_get2(uint32_t *encoder_raw, float *angle_el) 
{
    uint32_t count_raw =  encoder_get_count(); 
    if(encoder_raw) *encoder_raw = count_raw;
    if(angle_el)    *angle_el    = ((int32_t)count_raw - ENCODER_OFFSET) * ANGLE_EL_PER_COUNT;
}

/*************************************************************************************/
/************************** 电流环相关静态函数 *********************/
static void _get_dq_current(float *Id_actual, float *Iq_acutal, float the)  /* 读取 DQ轴电流 */
{
    /* 读取电流反馈值 */
    adc_current_t g_current;
    adc_get_current(&g_current);

	float Ia = g_current.iu;                       /* 读取三相电流 */
	float Ib = g_current.iv;
	float Ic = g_current.iw;
	/* Clark 变换 ：    
	 *   I_alpha = 2/3 * (Ia - 1/2*(Ib - Ic)) = Ia 
	 * 	 I_beta  = 2/3 * sqrt(3/2)*(Ib - Ic)
	 */
	float I_alpha = Ia;
	float I_beta = _SQRT_3_3 * (Ib - Ic);	
	
	/* Park 变换 ：
	 * 	 Id =  c * I_alpha + s * I_beta  
	 *	 Iq = -s * I_alpha + c * I_beta;
	 */
	float c = lt_cos(the);
	float s = lt_sin(the);
	float Id =  c * I_alpha + s * I_beta;
	float Iq = -s * I_alpha + c * I_beta;
	/* 对坐标变换后的直流分量做低通滤波，去除高频噪声 */
    float Iq_act = g_info->Iq;
    float Id_act = g_info->Id;
	Iq_act = LOW_PASS_FILTER(Iq, Iq_act, 0.32f);
	Id_act = LOW_PASS_FILTER(Id, Id_act, 0.32f);
    /* 更新电机电流 */
    g_info->Ia = Ia;
    g_info->Ib = Ib;
    g_info->Ic = Ic;
    g_info->Id = Id_act;
    g_info->Iq = Iq_act;
    /* 返回滤波后的 DQ轴电流 */
    *Id_actual = Id_act;
    *Iq_acutal = Iq_act;
}

static uint8_t _find_absolute_zero(lt_motor_state_t state)            /* 电流环电角度绝对零点定位 */
{
	static int count = 0;
	/* 给定小占空比，强制定位到 电角度 0°，并延时几百ms */
    if(count < ZERO_FIND_TICKS) {
		/* 占空比输出，强制定位到 A相 */
		foc_pwm_set_duty(ZERO_FIND_DUTY, 0, 0);
		count++;
    }else if(count == ZERO_FIND_TICKS && state == State_Init){
		/* 对准完成，转子现在在电角度 0° */
		encoder_set_zero();                       /* 把当前位置定义为 0 */
        count++;
		foc_pwm_set_duty(0, 0, 0);                /* PWM输出清零 */
        lt_fsm_update(Event_Init_Done);           /* 初始化完毕 */
        state = lt_fsm_get();                     /* 获取新的状态 */
	}

	return (state == State_Init) ? 0 : 1;         /* 0 : 未找到，1：已找到 */
}

static void _current_loop_run(float Id_actual, float Iq_actual, float the, float wr)	/* FOC 电流环 ：默认20kHz */
{   
    float Vd = 0, Vq = 0;
    lt_motor_mode_t mode   = g_info->mode;         /* 获取控制模式 */
    if(mode == Mode_Open_Loop){                    /* 开环控制模式 */
        Vq = g_info->target;
        Vq = lt_minf(lt_maxf(-100.0f, Vq), 100.0f);/* 占空比单位为 % */   
    }else{
        /* 电流环 PID */
        Vd = lt_pid_process(g_pid_id, Id_actual)  * 0.0833333f;		/* 将控制器输出做归一化处理：[-1.154,1.154] */
        Vq = lt_pid_process(g_pid_iq, Iq_actual)  * 0.0833333f;		/* 将控制器输出做归一化处理：[-1.154,1.154] */
    }
        
    /* FOC 处理 */
    lt_foc_process(g_foc, Vd, Vq, the);					        /* 内部自动进行电角度归一化，SVPWM */
    float dutyA, dutyB, dutyC;
    lt_foc_get_dutys(g_foc, &dutyA, &dutyB, &dutyC);			/* 获取 FOC 计算后占空比 */
	/* 进行死区补偿 */				
//	if(Vq > 0.01f || Vq < -0.01f){								/* 停机时，不需要补偿 */
//		lt_deadzone_compensate(Ia, Ib, Ic);
//		lt_deadzone_get(&dutyA, &dutyB, &dutyC);				/* 获取补偿后的占空比 */
//	}
	
    /* 占空比输出 */
	foc_pwm_set_duty(dutyA, dutyB, dutyC);
}