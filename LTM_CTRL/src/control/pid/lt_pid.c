/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        The first version
 * 2025-10-21     Lvtou        Remove dependancies on RT-Thread
 * 2025-11-29     Lvtou        Modity API and improve computation efficiency
 * 2025-12-15     Lvtou        Exchange the implementations of two types of pid
 * 2026-08-31     Lvtou        实现增量式 PI 接口，无微分运算，性能更好
 * 2026-9-25      Lvtou        全定点化：增益与调用周期全整数，模块零浮点零库调用
 * 2026-9-26      Lvtou        增益改 Q15、Ki·ts 保持 Q24；I 项移位修正；累加器小数位自适应
 * 2026-9-26      Lvtou        去 malloc：idx 索引 + 静态实例池，lt_pid_init 返回 0/1
 * 2026-9-27      Lvtou        init 声明信号格式（Q15/Q24）；增益改 Q24，两种格式共用同一算式
 * 2026-9-27      Lvtou        增益改回 Q15.15（1.0 = 32768，上限 ±65536），I/D 存每拍系数
 */
/* lt_pid —— 增量式定点 PID（静态实例池）
 *
 * 信号：Q15（32767 = 1.0）或 Q24（2^24 = 1.0），init 的 type 声明；
 * 增益：一律 Q15.15（32768 = 1.0），Kp 无量纲、Ki 1/s、Kd s，
 *       内部存每拍系数 ki = Ki·ts、kd = Kd·freq（换频率不用重算增益）；
 * 累加器：int64，信号单位 << PID_ACC_SH；GAIN_FRAC == ACC_SH，三项乘完不用移位。
 * type = 0：Kp ≤ 32767、Ki·ts ≤ 32767、Kd·freq ≤ 2^22-1，Δe 在热路径夹到 ±65535，
 *           三项乘积都在 int32 内，M0+/M23 上无 __aeabi_lmul（超限在 set 里夹住）；
 * type = 1：误差按 ±2^30 饱和，差值/二阶差分/乘积全走 int64，不做范围假设。
 */
#include "control/pid/lt_pid.h"
#include "math/basic/lt_math.h"
#include <string.h>

#define PID_GAIN_FRAC               15      /* 增益 Q15.15：2^15 = 1.0 */
#define PID_ACC_SH                  15      /* 累加器小数位（与 GAIN_FRAC 对齐）*/
#define PID_D2_LIMIT_Q15            255     /* 二阶差分限幅（信号单位）*/
#define PID_D2_LIMIT_Q24            (255 << 9)
#define PID_OUT_LIM_Q15             32767   /* 默认输出限幅：信号满量程 */
#define PID_OUT_LIM_Q24             (1 << 24)

/* int32 路径的增益上限：保证 系数 × 满摆信号 < 2^31 */
#define PID_KP_MAX_I32              32767    /* Kp ≤ 1.0 */
#define PID_KI_MAX_I32              32767    /* Ki·ts ≤ 1.0（65535 会越 int32） */
#define PID_KD_MAX_I32              4194303  /* 2^22-1：× d2(≤255) 仍在 int32 内 */
#define PID_DE_MAX_I32              65535    /* Δe 夹限：|kp·Δe| < 2^31 */
#define PID_E24_MAX                 (1 << 30)   /* Q24：误差上限，target-curr ≤ 2^31 */

typedef struct{
	int32_t  target;		/* 目标值（信号单位）*/

	/* ---- 增益，全部 Q15.15 ---- */
	int32_t  kp;			/* Kp */
	int32_t  ki;			/* Ki·ts（每拍）*/
	int32_t  kd;			/* Kd·freq（每拍）*/

	/* ---- 输出累加器 ---- */
	int64_t  out_acc;		/* 输出（信号单位）× 2^PID_ACC_SH */
	int32_t  out_max;		/* 输出上限（信号单位）*/
	int32_t  out_min;		/* 输出下限（信号单位）*/

	int32_t  err;			/* 上一拍偏差 */
	int32_t  err2;			/* 上上拍偏差 */
	uint8_t  type;			/* 0：信号 Q15；1：信号 Q24 */
	uint32_t freq;			/* 调用频率（Hz）*/
} lt_pid_object;

/* 私有：内部一律用指针书写，对外仍是 uint8_t idx */
typedef lt_pid_object *lt_pid_t;

/* ---- 实例池：无堆，idx 直接映射 ---- */
static lt_pid_object pid_pool[LT_PID_MAX_INSTANCES];

static lt_pid_object *_pid(uint8_t idx)
{
	if (idx < LT_PID_MAX_INSTANCES) {
		return &pid_pool[idx];
	}
	return 0;
}

uint8_t lt_pid_init(uint8_t idx, uint8_t type, uint32_t freq)
{
	lt_pid_t pid;
	if (idx >= LT_PID_MAX_INSTANCES) 	return 0;	/* 索引越界 */

	pid = &pid_pool[idx];
	memset(pid, 0, sizeof(lt_pid_object));
	pid->freq    = freq ? freq : 1;
	pid->type    = (type == 1) ? 1 : 0;
	pid->out_max =  pid->type ? PID_OUT_LIM_Q24 : PID_OUT_LIM_Q15;	/* 默认给满量程 */
	pid->out_min = -pid->out_max;
	return 1;
}

void lt_pid_reset(uint8_t idx)
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return;
	pid->out_acc = 0;
	pid->err     = 0;
	pid->err2    = 0;
}

void lt_pid_set(uint8_t idx, int32_t Kp, int32_t Ki, int32_t Kd)
{
	lt_pid_t pid = _pid(idx);
	if (!pid || !pid->freq) 	return;			/* 未初始化：freq == 0 */

	int32_t  f  = (int32_t)pid->freq;
	int64_t  ki = ((int64_t)Ki + (f >> 1)) / f;		/* Ki·ts */
	int64_t  kd = (int64_t)Kd * f;					/* Kd/ts */
	int64_t  fs = 0x7FFFFFFFLL;						/* int32 上限 */

	if (pid->type == 0) {							/* int32 路径：夹到乘积安全上限 */
		Kp = lt_clamp_i32(Kp, PID_KP_MAX_I32, -PID_KP_MAX_I32);
		ki = lt_clamp_i64(ki, PID_KI_MAX_I32, -PID_KI_MAX_I32);
		kd = lt_clamp_i64(kd, PID_KD_MAX_I32, -PID_KD_MAX_I32);
	} else {										/* Q24 信号：只防 int32 截断 */
		ki = lt_clamp_i64(ki,  fs, -fs);
		kd = lt_clamp_i64(kd,  fs, -fs);
	}

	pid->kp = Kp;
	pid->ki = (int32_t)ki;
	pid->kd = (int32_t)kd;
}

void lt_pid_set_target(uint8_t idx, int32_t target)
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return;
	pid->target = target;
}

void lt_pid_set_limits(uint8_t idx, int32_t out_max, int32_t out_min)
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return;

	if (out_max < out_min) {					/* 上下限写反了也认 */
		int32_t t = out_max;
		out_max = out_min;
		out_min = t;
	}
	pid->out_max = out_max;						/* 信号单位 */
	pid->out_min = out_min;
}

int32_t lt_pid_get(uint8_t idx)
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return 0;
	return (int32_t)(pid->out_acc >> PID_ACC_SH);
}

/* 输出累加 + 限幅：在累加器尺度做，避免小增量被截成 0 */
static inline int32_t _out_commit(lt_pid_t pid, int64_t d)
{
	int64_t hi  = (int64_t)pid->out_max << PID_ACC_SH;
	int64_t lo  = (int64_t)pid->out_min << PID_ACC_SH;
	int64_t out = pid->out_acc + d;
	if (out > hi)      out = hi;
	else if (out < lo) out = lo;
	pid->out_acc = out;
	return (int32_t)(out >> PID_ACC_SH);
}

static inline int32_t _d2_clamp(lt_pid_t pid, int32_t d2)
{
	int32_t lim = pid->type ? PID_D2_LIMIT_Q24 : PID_D2_LIMIT_Q15;
	if (d2 >  lim) return  lim;
	if (d2 < -lim) return -lim;
	return d2;
}

int32_t lt_pi_update(uint8_t idx, int32_t curr_val)		/* 增量式 PI */
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return 0;

	int32_t err;
	int64_t d;

	if (pid->type) {							/* Q24 信号：差值、乘积都走 64 位 */
		int64_t e = lt_clamp_i64((int64_t)pid->target - curr_val, PID_E24_MAX, -PID_E24_MAX);
		err = (int32_t)e;
		d   = (int64_t)pid->kp * ((int64_t)err - pid->err) + (int64_t)pid->ki * err;
	} else {								/* Q15 信号：乘积夹进 int32 */
		int32_t de;
		err = pid->target - curr_val;
		de  = lt_clamp_i32(err - pid->err, PID_DE_MAX_I32, -PID_DE_MAX_I32);
		d   = (int64_t)(pid->kp * de) + (pid->ki * err);
	}
	pid->err = err;
	return _out_commit(pid, d);
}

int32_t lt_pd_update(uint8_t idx, int32_t curr_val)		/* 增量式 PD */
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return 0;

	int32_t err;
	int64_t d;

	if (pid->type) {
		int64_t e  = lt_clamp_i64((int64_t)pid->target - curr_val, PID_E24_MAX, -PID_E24_MAX);
		int64_t d2 = e - 2 * (int64_t)pid->err + pid->err2;
		err = (int32_t)e;
		d2  = lt_clamp_i64(d2, PID_D2_LIMIT_Q24, -PID_D2_LIMIT_Q24);
		d   = (int64_t)pid->kp * ((int64_t)err - pid->err) + (int64_t)pid->kd * d2;
	} else {
		int32_t de, d2;
		err = pid->target - curr_val;
		de  = lt_clamp_i32(err - pid->err, PID_DE_MAX_I32, -PID_DE_MAX_I32);
		d2  = _d2_clamp(pid, err - 2 * pid->err + pid->err2);
		d   = (int64_t)(pid->kp * de) + (int64_t)(pid->kd * d2);
	}
	pid->err2 = pid->err;
	pid->err  = err;
	return _out_commit(pid, d);
}

int32_t lt_pid_update(uint8_t idx, int32_t curr_val)		/* 增量式 PID */
{
	lt_pid_t pid = _pid(idx);
	if (!pid) 	return 0;

	int32_t err;
	int64_t d;

	if (pid->type) {
		int64_t e  = lt_clamp_i64((int64_t)pid->target - curr_val, PID_E24_MAX, -PID_E24_MAX);
		int64_t d2 = e - 2 * (int64_t)pid->err + pid->err2;
		err = (int32_t)e;
		d2  = lt_clamp_i64(d2, PID_D2_LIMIT_Q24, -PID_D2_LIMIT_Q24);
		d   = (int64_t)pid->kp * ((int64_t)err - pid->err) + (int64_t)pid->ki * err
		    + (int64_t)pid->kd * d2;
	} else {
		int32_t de, d2;
		err = pid->target - curr_val;
		de  = lt_clamp_i32(err - pid->err, PID_DE_MAX_I32, -PID_DE_MAX_I32);
		d2  = _d2_clamp(pid, err - 2 * pid->err + pid->err2);
		d   = (int64_t)(pid->kp * de) + (int64_t)(pid->ki * err) + (int64_t)(pid->kd * d2);
	}
	pid->err2 = pid->err;
	pid->err  = err;
	return _out_commit(pid, d);
}
