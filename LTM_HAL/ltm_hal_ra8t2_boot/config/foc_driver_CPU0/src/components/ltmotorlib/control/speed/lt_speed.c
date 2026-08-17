/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-07-20     Lvtou        锁相环（PLL）测试模块实现 
 */

#include <math.h>
#include <string.h>
#include <stdint.h>
#include "control/speed/lt_speed.h"

#define     ABS(x)                          ((x) > 0 ? (x) : -(x))
#define     MAX(x, y)                       (((x) > (y)) ? (x) : (y))
#define     MIN(x, y)                       (((x) < (y)) ? (x) : (y))
#define     LOW_PASS_FILTER(X_k,Y_k_1,a) 	((a)*(X_k) + (1.0f - (a))*(Y_k_1))  /* 低通滤波 */
#define     PLL_FILTER_ALPHA                0.3333333f  /* 速度滤波系数 */
#define     PLL_BANDWIDTH                   300.0f      /* 锁相环带宽：300 Hz */
#define     PLL_KP_PARAM                    2.0f        /* PLL_KP = 2.0f * bandwitdh */
#define     PLL_KI_PARAM                    0.15f       /* PLL_KI = 0.15f * PLL_KP ^ 2 */
#define     PLL_MIN_SPEED                   3.0f        /* 锁相环允许的最低速度 ：RPM */       

/* 定义一个测速对象 */
typedef struct{
    float pos_est;          /* 锁相环估计位置（Count）*/
	float speed;			/* RPM */
	float speed_est;		/* 锁相环估计速度（RPM）*/
	float half_reso;		/* 一半的分辨率，用于半圈法测速 */
    float count_2_rpm;      /* count/s 到RPM的换算系数 */
    float Kp_dt;            /* PLL_Kp * PLL_Period */
    float Ki_dt;            /* PLL_Ki * PLL_Period */
    float rpm_2_count;      
	uint8_t flag;			/* 1: 初始化成功，0：未初始化 */
}lt_speed_obj;

typedef lt_speed_obj * lt_speed_t;
static lt_speed_obj speed_obj;			/* 测速对象 */
static lt_speed_t  sp = &speed_obj;	 	/* 用指针接口，效率更高 */

static inline float _normalize(float value, float range)    /* 简易归一化实现:[0, range) */
{
    /* 快速归一化，防止大量循环 */
    if (value >= 0 && value < range) return value;
    
    /* 通用路径：利用浮点特性快速截断 */
    float inv_range = 1.0f / range;         // 预计算倒数
    float div = value * inv_range;
    int ipart = (int)div;
    float fpart = div - (float)ipart;
    if (fpart < 0) fpart += 1.0f;
    return fpart * range;
} 

static void _speed_set(uint32_t reso, float freq);

void lt_speed_init(uint32_t reso, float freq)	    /* reso : 编码器分辨率， freq : 调用频率（Hz） */
{
    _speed_set(reso, freq);
}

void lt_speed_set(uint32_t reso, float freq)		/* reso : 编码器分辨率， freq : 调用频率（Hz） */
{
    _speed_set(reso, freq);
}   

float lt_speed_update(int32_t pos_count)				        /* 获取当前速度： RPM */
{
	if(!sp->flag)			return 0;							/* 未初始化，直接返回 0 */

    float speed_est = sp->speed_est;							/* 上次更新的速度 RPM */
    float pos_est   = sp->pos_est + speed_est * sp->rpm_2_count; /* 估计位置： Count */
	float pos_delt  = (float)pos_count - pos_est;			    /* 获取编码器计数值（Count）*/
	float half_reso = sp->half_reso;							/* 采用半圈法测速 */
	float reso  = half_reso * 2.0f;								/* 编码器分辨率 */
	float speed = sp->speed;								    /* 上一时刻速度 */				

	/* 半圈法补偿：中高频调用下，该实现可以保证始终成立，无需多次补偿 */
	if(pos_delt > half_reso){									/* 出现编码器跨圈：反向补偿 */
		pos_delt -= reso;
	}else if(pos_delt < -half_reso){							/* 出现编码器跨圈：正向补偿 */
		pos_delt += reso;
	}

	/* 开始进行 PLL 测速 */
    pos_est   += sp->Kp_dt * pos_delt;                          /* Count */
    pos_est   = _normalize(pos_est, reso);                      /* 计算位置归一化：[0, CPR) */
    speed_est += sp->Ki_dt * pos_delt * sp->count_2_rpm;        /* RPM */
    if(ABS(speed_est) < PLL_MIN_SPEED)  speed_est = 0;          /* 最低速限幅 */
    speed     = LOW_PASS_FILTER(speed_est, speed, PLL_FILTER_ALPHA);  /* 低通滤波 */

	/* 更新 speed_est 和 pos_est */
    sp->pos_est    = pos_est;
    sp->speed_est  = speed_est; 
	sp->speed      = speed;

	return speed;										/* 返回最终速度 RPM */
}

float lt_speed_get(void)                                /* 该接口是给用户调用的，不影响测速计算 */
{  
    if(!sp->flag)           return  0;
    return sp->speed;                                   /* 返回最终速度 RPM */
}

/******************************************************************************************/
static void _speed_set(uint32_t reso, float freq)
{
    memset(sp, 0, sizeof(lt_speed_obj));
    if(!reso || freq <= 0.0f)		return;				 		/* 判空 */
    /* 更新测试模块配置 */
    float bandwidth  =  MIN(PLL_BANDWIDTH, freq / 4.0f);        /* 获取锁相环带宽（rad/s）*/
    float pll_Kp     =  PLL_KP_PARAM  * bandwidth;
    float pll_Ki     =  PLL_KI_PARAM * pll_Kp * pll_Kp;
    
	sp->half_reso    = reso >> 1;								/* 获取一半的分辨率 */							
	sp->count_2_rpm  = 60.0f * freq / reso;                     /* count/s到 RPM 的换算系数 */
    sp->rpm_2_count  = reso / freq / 60.0f;
    sp->Kp_dt        = pll_Kp / freq;
    sp->Ki_dt        = pll_Ki / freq;
	sp->flag 		 = 1;										/* 标记初始化成功 */
}
