/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-7-2       Lvtou        自适应 M法 测速实现
 * 
 */
#include "control/speed/lt_speed.h"
#include "math/basic/lt_math.h"
#include <string.h>

/* 窗口到期时的累计脉冲小于此值，视为极低速（约 3RPM，6ms 窗口下），分辨率：10000，3kHz速度环 */
#define MIN_POS_DELT					3 		/* 允许的最小脉冲差值，对应极低速 */ 

/* ====== M法 测速状态机 ====== */
typedef enum {
    WINDOW_6MS = 0,						/* 节拍 ：18 */
    WINDOW_4MS,							/* 节拍 ：12 */
    WINDOW_3MS,							/* 节拍 ：9 */
    WINDOW_2MS,							/* 节拍 ：6 */
    WINDOW_1MS,							/* 节拍 ：3 */
    WINDOW_333US,						/* 节拍 ：1 */
    WINDOW_COUNT
}lt_speed_state_t;

/* 定义一个测速对象 */
typedef struct{
	float pos_last;			/* Unit */
	float speed;			/* RPM */
	float speed_last;		/* RPM */
	float half_reso;		/* 一半的分辨率，用于半圈法测速 */
	uint8_t cycle_cnt;		/* 当前节拍数 */
	uint8_t cycle_target;	/* 目标节拍数 */
	lt_speed_state_t state; /* 测速状态机 */
	uint8_t flag;			/* 1: 初始化成功，0：未初始化 */
}lt_speed_obj;

typedef lt_speed_obj * lt_speed_t;

/* 窗口结构体： 采用滞环，保证速度在切换点反复波动 */
typedef struct {
    float speed_up;         /* 升档阈值 (RPM) */
    float speed_down;       /* 降档阈值 (RPM) */
    float unit_to_rpm;      /* Unit → RPM 转换系数 */
	float alpha;			/* 速度低通滤波系数 */
    uint8_t cycle;          /* 速度环拍数 (窗口时间 / 速度环周期)，拍数的是固定的 */
}lt_speed_config_t;

static lt_speed_obj speed_obj;			/* 测速对象 */
static lt_speed_t  sp = &speed_obj;	 	/* 用指针接口，单纯用语法糖 */

/* 默认的窗口配置表： 编码器分辨率 10000 Unit，速度环 3kHz */
static  lt_speed_config_t win_config[WINDOW_COUNT] = {
    { 12.0f,  8.0f,  60.0f / (10000.0f * 0.006f),    0.12f , 18 },  /* 6ms */
    { 27.0f, 20.0f,  60.0f / (10000.0f * 0.004f),    0.16f , 12 },  /* 4ms */
    { 52.0f, 45.0f,  60.0f / (10000.0f * 0.003f),    0.20f , 9  },  /* 3ms */
    {105.0f, 95.0f,  60.0f / (10000.0f * 0.002f),    0.25f , 6  },  /* 2ms */
    {250.0f,230.0f,  60.0f / (10000.0f * 0.001f),    0.30f , 3  },  /* 1ms */
    {999.0f,240.0f,  60.0f / (10000.0f * 0.0003333f),0.35f , 1  }   /* 333.3μs */
};

static void _speed_update_state(void);							/* 速度状态机更新 */		
static void _speed_set(uint32_t reso, float freq);

void lt_speed_init(uint32_t reso, float freq)
{
	_speed_set(reso, freq);
}

void lt_speed_set(uint32_t reso, float freq)					/* 调用该接口，默认停机 */
{
	_speed_set(reso, freq);
}

float lt_speed_get(int32_t pos_unit)							/* 获取当前速度： RPM */
{
	if(!sp->flag)			return 0;							/* 未初始化，直接返回 0 */

	float pos_delt = (float)pos_unit - sp->pos_last;			/* 获取编码器计数值 */
	float half_reso = sp->half_reso;							/* 采样半圈法测速 */
	float reso = half_reso * 2.0f;								/* 编码器分辨率 */
	float speed_last = sp->speed_last;							/* 上次更新的速度 RPM */
	float speed = 0;											/* 当前速度 */
	float speed_raw = 0;										/* 差分法速度原始值 */				
	float alpha = win_config[sp->state].alpha;					/* 低通滤波系数 */

	sp->cycle_cnt++;											/* 周期计数值 + 1 */
	/* 半圈法补偿：*/
	if(pos_delt > half_reso){									/* 出现编码器跨圈：反向补偿 */
		pos_delt -= reso;
	}else if(pos_delt < -half_reso){							/* 出现编码器跨圈：正向补偿 */
		pos_delt += reso;
	}

	if(sp->cycle_cnt >= sp->cycle_target){						/* 预期的节拍达到 */
		sp->cycle_cnt = 0;										/* 更新当前计数值 */
	}else{														/* 否则直接返回上一次计数值 */
		speed = speed_last;
		sp->speed = speed;
		return speed;
	}

	/* 开始进行 M法测速 */
	if(pos_delt < MIN_POS_DELT && pos_delt > -MIN_POS_DELT){	/* 脉冲数太小，M法误差过大，说明此时极低速，< +-3RPM */
		speed_raw = 0;											/* 直接认为速度等于 0 */
	}else{														/* 脉冲速足够 */
		speed_raw = pos_delt * win_config[sp->state].unit_to_rpm; /* 乘以换算系数，获取原始速度 */
	}
	speed = LOW_PASS_FILTER(speed_raw, speed_last, alpha);		/* 低通滤波处理 */
	
	/* 更新 speed_last 和 pos_last */
	sp->speed_last = speed;
	sp->pos_last   = (float)pos_unit;
	sp->speed      = speed;
	_speed_update_state();										/* 更新速度状态机 */

	return speed;												/* 返回最终速度 RPM */
}

float lt_speed_get2(void)
{
	if(!sp->flag)			return 0;							/* 未初始化，直接返回 0 */
	return sp->speed;											/* 返回测速完毕后的速度 */
}


/**********************************************************************************************************/
static void _speed_update_state(void)							/* 速度状态机更新 */	
{
	lt_speed_state_t curr_state = sp->state;					/* 获取当前状态 */
	lt_speed_state_t new_state = WINDOW_6MS;
	float speed = sp->speed;									/* 获取当前速度 RPM */
	float speed_abs = (speed < 0.0f)?  -speed : speed;			/* 速度窗口是基于速度绝对值判断的 */
	float speed_up 	 = win_config[curr_state].speed_up;			/* 速度窗口上沿 */
	float speed_down = win_config[curr_state].speed_down;		/* 速度窗口下沿 */
	
	if(speed_abs > speed_up && curr_state != WINDOW_333US){			/* 状态能继续更新 */
		new_state = curr_state + 1;									/* 状态前进 1步 */
	}else if(speed_abs < speed_down && curr_state != WINDOW_6MS){	/* 状态能继续更新 */
		new_state = curr_state - 1;									/* 状态后退 1步 */
	}
	
    if (new_state != curr_state){								/* 状态更新了 */
        sp->state = new_state;
        sp->cycle_cnt = 0;										/* 更新当前节拍数 */
		sp->cycle_target = win_config[new_state].cycle;			/* 更新计数目标值 */
    }

}	

static void _speed_set(uint32_t reso, float freq)
{
	memset(sp, 0, sizeof(lt_speed_obj)); /* 将结构体清零 */
	if(!reso || freq <= 0.0f)		return;				 		/* 判空 */
	/* 更新窗口配置表 */
	for(uint8_t i = 0; i < WINDOW_COUNT; i++){
		float ts = 1.0/freq * win_config[i].cycle;
		win_config[i].unit_to_rpm = 60.0f / (reso * ts);		/* 计算新的换算系数 */ 
	}
	/* 从 6ms 状态机开始 */
	sp->half_reso    = reso >> 1;								/* 获取一半的分辨率 */							
	sp->cycle_target = win_config[WINDOW_6MS].cycle;
	sp->state        = WINDOW_6MS;
	sp->flag 		 = 1;										/* 标记初始化成功 */
}