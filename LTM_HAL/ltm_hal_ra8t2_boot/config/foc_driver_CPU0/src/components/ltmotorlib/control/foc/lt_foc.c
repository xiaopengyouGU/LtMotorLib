/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author        Notes
 * 2025-6-21      Lvtou         The first version
 * 2025-8-27      Lvtou         Use lookup table sin and cos
 * 2025-9-7       Lvtou         Fix 6 commutation method and add modulation modes
 * 2025-11-30     Lvtou         Reimplement foc object and remove six-step commutation
 * 2026-6-25      Lvtou         整合六步换向及HALL，修改API接口，添加真正的 SVPWM实现 
 */

#include "control/foc/lt_foc.h"
#include "math/basic/lt_math.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* 六步换相表：1:正向导通，-1：反向导通，0：不导通 */
static int8_t trapz_6_table[6][3] = {   {0,1,-1},	/* 扇区1 */
										{-1,1,0},	/* 扇区2 */
										{-1,0,1},	/* 扇区3 */
										{0,-1,1},	/* 扇区4 */
										{1,-1,0},	/* 扇区5 */
										{1,0,-1}	/* 扇区6 */
};

/* 霍尔信号映射表，表中的值 - 1（0除外），恰好对应 trapz_6_table的主索引
 * 信号格式 : ABC ==》 bit2 : A, bit1 ：B, bit0 ：C
 * eg: 101（5） --> 扇区 1, 000 or 111 --> 错误信号
 * 注：此处按理说不应该出现 HALL的，但恰好 HALL 和 六步换向能整合到 lt_foc 框架下
 *     因此没有必要搞一个 lt_trapz.c, 读者往下看就能体会到这一点
 */

static uint8_t hall_map[8] = {  0,		/* 错误信号 */
								6,		/* 扇区 6 */
								4,		/* 扇区 4 */
								5,		/* 扇区 5 */
								2,		/* 扇区 2 */
								1,		/* 扇区 1 */
								3,		/* 扇区 3 */
								0		/* 错误信号 */
};


/*---------------------- FOC 对象定义 ----------------------*/
struct lt_foc_object {
    uint8_t mode;                 /* 控制模式 */
    uint8_t type;                 /* 调制类型 */
	/* 输出占空比 [0,1] */
    float dutyA;
 	float dutyB;
	float dutyC;    			  /* 浮点数版本 */
};

/*---------------------- API 实现 ----------------------*/
lt_foc_t lt_foc_create(uint8_t mode, uint8_t type)
{
    lt_foc_t foc = (lt_foc_t)malloc(sizeof(struct lt_foc_object));
    if (foc == NULL) return NULL;
    memset(foc, 0, sizeof(struct lt_foc_object));
    
    if (type > FOC_TYPE_SVPWM) type = FOC_TYPE_SPWM;
    foc->mode = mode;
    foc->type = type;
    
    return foc;
}

void lt_foc_delete(lt_foc_t foc)
{
    if(foc) free(foc);
}

void lt_foc_set(lt_foc_t foc, uint8_t mode, uint8_t type)
{
    if (!foc) return;
    if (type > FOC_TYPE_SVPWM) type = FOC_TYPE_SPWM;
    foc->mode = mode;
    foc->type = type;
}

/*==============================================================================
 * 核心处理函数：输入 浮点数 格式的 D/Q 轴相电压指令[-1,1]，输出占空比 [0-1]
 * 输入值 = 原始值相对母线电压一半的标幺值，Vq = 1.0 pu 等价于 Vq = Vdc/2 (V)。  
 * 实例：Vd = 0, Vq = 1.0,  
 * 即Q轴相电压为母线电压Vdc的一半，即 Vq（原始值） = Vdc/2
 * 对应的线电压：sqrt(3)*Vdc/2 = 0.866 Vdc, 母线电压利用率 86.6%
 * SVPWM 线性调制区允许输出的 Vq <= 1.0 * 2/sqrt(3) = 1.154 ==> 母线电压利用率：100%
 * 用乘法代替除法，提高执行效率
 * SVPWM ：空间电压矢量 PWM 调制
 * 这个函数部分参考了 SimpleFOC 的实现，见 https://www.simplefoc.com/，主要还是自己推导的
 *==============================================================================*/

#define SVPWM_LIMIT  (1.1547005f)  				/* 2/sqrt(3)，对应100%母线电压利用率 */

void lt_foc_process(lt_foc_t foc, float Vd, float Vq, float angle_el)
{
	if(!foc) return;
	/* 电压限幅 [-1.154, 1.154] */				
	Vd = CONSTRAINS(Vd, SVPWM_LIMIT, -SVPWM_LIMIT);	/* SVPWM的线性调制区更大 */
	Vq = CONSTRAINS(Vq, SVPWM_LIMIT, -SVPWM_LIMIT);
    /* 转子电角度归一化并获取正余弦 */
    angle_el = lt_normalize(angle_el);  		/* 输出：[0, 2*pi) */
    float c = lt_cos(angle_el);					/* 快速查表法 */
    float s = lt_sin(angle_el);					/* 快速查表法 */

    /* Park 逆变换：得到 alpha 和 beta 相归一化电压 [-1,1]
	 * V_alpha = c*Vd - s*Vq, V_beta = s*Vd + c*Vq 
	 */
	float V_alpha = c*Vd - s*Vq;
    float V_beta  = s*Vd + c*Vq;
	/* 获取当前所在扇区 ： 1,2,3,4,5,6 */
	angle_el = lt_atan2(V_beta, V_alpha);		/* 获取空间电压矢量的电角度: [0, 2*pi) */
	/* 注意！！！ 此时的电角度含义发生了改变 */
	uint8_t sector = (uint8_t)(angle_el * _PI_3_INV) + 1;
	
	/* 计算空间电压矢量的作用时间 [0,1] */
	float Uout = lt_sqrt(Vd*Vd + Vq*Vq);	 	/* 计算空间电压矢量幅值, 采用快速开方，精度很高 */
	/* 线性调制区：Uout <= 1.154 */
	float T1 = _SQRT_3_2 * lt_sin(sector *_PI_3 - angle_el) * Uout;
	float T2 = _SQRT_3_2 * lt_sin(angle_el - (sector-1.0f)  * _PI_3) * Uout;
	
	/* 过调制处理：T1 + T2 > 1 时等比例缩小 
	 * 最大程度利用电机转矩
	 * 即 |Uout| > 0.577f 时，触发过调制
	 */
	float T_sum = T1 + T2;
	if (T_sum > 1.0f) {
		float T_inv = 1.0f / T_sum;
		T1 *= T_inv;
		T2 *= T_inv;
	}
	float T0 = 1.0f - T1 - T2;					/* 获取零矢量作用时间 */
	float T0_half = T0 * 0.5f;					/* T0/2 ==> 改乘法为除法优化性能 */
	float Ta, Tb, Tc;							/* 三相占空比 [0,1] */
	/* 计算最终的占空比 */						
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0_half;
			Tb = T2 + T0_half;
			Tc = T0_half;
			break;
		case 2:
			Ta = T1 +  T0_half;
			Tb = T1 + T2 + T0_half;
			Tc = T0_half;
			break;
		case 3:
			Ta = T0_half;
			Tb = T1 + T2 + T0_half;
			Tc = T2 + T0_half;
			break;
		case 4:
			Ta = T0_half;
			Tb = T1+ T0_half;
			Tc = T1 + T2 + T0_half;
			break;
		case 5:
			Ta = T2 + T0_half;
			Tb = T0_half;
			Tc = T1 + T2 + T0_half;
			break;
		case 6:
			Ta = T1 + T2 + T0_half;
			Tb = T0_half;
			Tc = T1 + T0_half;
			break;
		default:  
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	
	/* 记录最终的占空比 */
	foc->dutyA = Ta;
	foc->dutyB = Tb;
	foc->dutyC = Tc;
}

/*==============================================================================
 * 核心处理函数：输入 浮点数 格式的 D/Q 轴电压指令[-1,1]，输出占空比 [0-1]
 * 输入值 = 原始值相对母线电压一半的标幺值，Vq = 1.0 pu 等价于 Vq = Vdc/2 (V)。  
 * 其余电压调制方法（非 SVPWM）：
 * SPWM+均值注入 在线性调制区和SVPWM是等效的，大部分情况下能模拟SVPWM
 * 函数代码部分参考了 SimpleFOC 的实现，见 https://www.simplefoc.com/，主要是自己写的
 *==============================================================================*/
void lt_foc_process2(lt_foc_t foc, float Vd, float Vq, float angle_el)
{
    if(!foc) return;
	/* 电压限幅 [-1,1] */
	if(foc->type != FOC_TYPE_SVPWM){
		Vd = CONSTRAINS(Vd, 1.0f, -1.0f);
		Vq = CONSTRAINS(Vq, 1.0f, -1.0f);
	}else{
		Vd = CONSTRAINS(Vd, SVPWM_LIMIT, -SVPWM_LIMIT);		/* SVPWM的线性调制区更大 */
		Vq = CONSTRAINS(Vq, SVPWM_LIMIT, -SVPWM_LIMIT);
	}
    /* 转子电角度归一化并获取正余弦 */
    angle_el = lt_normalize(angle_el);  		/* 输出：[0, 2*pi) */
    float c = lt_cos(angle_el);					/* 快速查表法 */
    float s = lt_sin(angle_el);					/* 快速查表法 */

    /* Park 逆变换：得到 alpha 和 beta 相归一化电压 [-1,1]
	 * V_alpha = c*Vd - s*Vq, V_beta = s*Vd + c*Vq 
	 */
	float V_alpha = c*Vd - s*Vq;
    float V_beta  = s*Vd + c*Vq;

    /* Clark 逆变换：得到三相归一化电压 [-1,1]
     * Va = V_alpha 
     * Vb = -0.5*V_alpha + sqrt3/2*V_beta
     * Vc = -0.5*V_alpha - sqrt3/2*V_beta 
	 */
    float half_alpha = 0.5f * V_alpha;                      /* 0.5f * V_alpha */
    float beta_part =  _SQRT_3_2 * V_beta;                  /* sqrt(3)/2 * V_beta ≈ 0.8660254 * V_beta */

    float Va = V_alpha;										/* A相电压 */
    float Vb = -half_alpha + beta_part;						/* B相电压 */
    float Vc = -half_alpha - beta_part;						/* C相电压 */

    /* 零序分量注入，提高直流母线电压利用率 */
    float up = 0;
    switch (foc->type) {
        case FOC_TYPE_SPWM_1: {                             /* 最小值注入 */
            float min_abc = lt_minf(Va, lt_minf(Vb, Vc));   /* 获取最小值 */
            up = -min_abc - 1.0f;                           /* -min - 1，使最小相电压为 -1 */
            break;
        }
		case FOC_TYPE_SVPWM:								/* 调用该接口时，用均值注入模拟 SVPWM */
        case FOC_TYPE_SPWM_2: {                             /* 均值注入 */
            float min_abc = lt_minf(Va, lt_minf(Vb, Vc));   /* 获取最小值 */
            float max_abc = lt_maxf(Va, lt_maxf(Vb, Vc));
            up = -(max_abc + min_abc) * 0.5f;               /* -(max+min)/2, 等效 SVPWM 的三角波零序分量 */
            break;
        }
		case FOC_TYPE_TRAPZ: {			/* 六步换相法 */
			if(Vq < 0){					/* 反向旋转，即电角度滞后 180° */
				angle_el = lt_normalize(angle_el + _PI);
				Vq = -Vq;
			}
			/* 获取当前电角度对应的 导通区 */	
			uint8_t i = (uint8_t)((angle_el+_PI_6) * _PI_3_INV);	
			if(i >= 6 || i == 0) i = 6;
			i = i - 1;
			Va = Vq * trapz_6_table[i][0];
			Vb = Vq * trapz_6_table[i][1];
			Vc = Vq * trapz_6_table[i][2];
			break;
		}
        case FOC_TYPE_SPWM: break;                         /* 标准 SPWM, 注入值为0 */
        default:    break;
    }

    /* 注入零序分量 */
    Va += up;
    Vb += up;
    Vc += up;

	/* 将归一化电压 V [-1, 1] 映射到占空比 [0, 1] */
    /* 数学公式: duty = (V + 1) / 2                                */
	/* 得到最终的占空比 */
    foc->dutyA = (Va + 1.0f) * 0.5f;
    foc->dutyB = (Vb + 1.0f) * 0.5f;
    foc->dutyC = (Vc + 1.0f) * 0.5f;
}

/*==============================================================================
 * 核心处理函数：输入 浮点数 格式的 D/Q 轴电压指令[-1,1]，输出占空比 [0-1]
 * 用乘法代替除法，提高执行效率
 * 两个六步换向电压调制方法（适合于 BLDC）
 * 虽然挂羊头卖狗肉，但仍然是在同一套框架下的，没必要额外搞一个 lt_trapz.c 文件
 *==============================================================================*/
void lt_foc_process3(lt_foc_t foc, float Vq, uint8_t hallA, uint8_t hallB, uint8_t hallC)
{
    if(!foc)     return;     /* 判空 */
	/* 电压限幅 [-1,1] */
	Vq = CONSTRAINS(Vq, 1.0f, -1.0f);
	
	uint8_t signal = 0;
	signal |= (hallA << 2);								/* bit2 */
	signal |= (hallB << 1);								/* bit1 */
	signal |= (hallC << 0);								/* bit0 */
	
	uint8_t i = hall_map[signal];
	if(i == 0){									/* 霍尔信号故障，直接返回 */
		foc->dutyA = 0;
		foc->dutyB = 0;
		foc->dutyC = 0;
		return;
	}
	
	uint8_t sector = ((i - 1) + 3) % 6;							    /* 下标 0 -> 5 */
	if(Vq < 0){
		Vq 	   = -Vq;
		sector = (sector + 3) % 6;						/* 电角度 + 180°，对应反向 */
	}
	
	float Va = Vq * trapz_6_table[sector][0];
	float Vb = Vq * trapz_6_table[sector][1];
	float Vc = Vq * trapz_6_table[sector][2];
	/* 这里不需要注入 零序分量 */
	/* 将归一化电压 V [-1, 1] 映射到占空比 [0, 1] <==> 中心电压调制 */
    /* 数学公式: duty = (V + 1) / 2                                */
	/* 得到最终的占空比 */
    foc->dutyA = (Va + 1.0f) * 0.5f;
    foc->dutyB = (Vb + 1.0f) * 0.5f;
    foc->dutyC = (Vc + 1.0f) * 0.5f;
}

void lt_foc_get_dutys(lt_foc_t foc, float *dutyA, float *dutyB,  float *dutyC)
{                               
    if(!foc)     return;     /* 判空 */
    *dutyA = foc->dutyA;
    *dutyB = foc->dutyB;
    *dutyC = foc->dutyC;
}