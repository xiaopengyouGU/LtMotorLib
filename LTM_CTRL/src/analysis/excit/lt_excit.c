#include "analysis/excit/lt_excit.h"
#include "math/basic/lt_math.h"
#include <string.h>

/* 激励信号类型（内部使用，用户不可见） */
typedef enum {
    Excit_None = 0,     /* 无输出，返回 offset */
    Excit_Step,         /* 阶跃：offset → offset + amplitude */
    Excit_Square,       /* 方波：offset ± amplitude */
    Excit_Triangle,     /* 三角波：0 → +peak → 0 → -peak → 0 */
    Excit_Sine,         /* 正弦波：offset + amplitude * sin(2π·freq·t + phase) */
} lt_excit_type_t;

/* 激励发生器状态对象 */
typedef struct {
    lt_excit_type_t type;   /* 当前波形类型 */

    float amplitude;        /* 幅值（阶跃/方波/正弦/扫频） */
    float offset;           /* 偏置（所有波形） */
    float freq_hz;          /* 频率（方波/正弦） */
    float phase_rad;        /* 当前相位（正弦）/ 三角波段状态复用 */

    union {
        struct {
            float freq_start;       /* 扫频起始频率 Hz */
            float freq_end;         /* 扫频终止频率 Hz */
            float sweep_duration;   /* 扫频总时长 s */
        } chirp;
        struct {
            float v_peak;           /* 三角波峰值 */
            float accel;            /* 三角波加速度（决定斜率） */
        } triangle;
    };

    float dt;               /* 步进时间间隔 s */
    float time;             /* 累计运行时间 s */
    float out_cos;          /* 正交输出（用于相关解调）*/
    float out;              /* 当前输出值（缓存） */
} lt_excit_obj;

/* 全局单例，指针访问减少结构体拷贝 */
static lt_excit_obj excit_obj;
static lt_excit_obj *excit = &excit_obj;

/* ============================================================================
 * 生命周期
 * ============================================================================*/

void lt_excit_init(float dt, float offset)
{
    memset(excit, 0, sizeof(lt_excit_obj));
    excit->dt = dt;
    excit->out = offset;
    excit->offset = offset;
}

/* ============================================================================
 * 启动各类型波形
 * ============================================================================*/

void lt_excit_start_step(float amplitude)
{
    excit->type = Excit_Step;
    excit->amplitude = amplitude;
    excit->time = 0.0f;
}

/* type = 0:获取当前激励值, type = 1: 去除偏置后激励，2:获取正交激励（cos）*/
float lt_excit_get(uint8_t type) 
{
    if(type == 1)       return excit->out - excit->offset; 
    else if(type == 2)  return excit->out_cos;
    else                return excit->out;
}

void lt_excit_start_square(float amplitude, float freq_hz)
{
    excit->type = Excit_Square;
    excit->amplitude = amplitude;
    excit->freq_hz = freq_hz;
    excit->time = 0.0f;
}

void lt_excit_start_triangle(float v_peak, float accel)
{
    excit->type = Excit_Triangle;
    excit->triangle.v_peak = v_peak;
    excit->triangle.accel = accel;
    excit->phase_rad = 0.0f;   /* phase 复用为段状态 */
    excit->time = 0.0f;
}

void lt_excit_start_sine(float amplitude, float freq_hz, float phase_rad)
{
    excit->type = Excit_Sine;
    excit->amplitude = amplitude;
    excit->freq_hz = freq_hz;
    excit->phase_rad = phase_rad;
    excit->time = 0.0f;
}

void lt_excit_stop(void)
{
    excit->type = Excit_None;
    excit->out = excit->offset;
}

/* 激励信号更新 */
float lt_excit_update(void)
{
    float dt = excit->dt;
    float offset = excit->offset;
    float amp    = excit->amplitude;
    float out = offset;

    switch (excit->type) {
        case Excit_None:    break;
        case Excit_Step:
            out = offset + amp;
            break;

        case Excit_Square: {
            float period = 1.0f / excit->freq_hz;
            float half_period = period * 0.5f;
            float t = lt_normalize_quick(excit->time, period);
            out = offset + (t < half_period ? amp : -amp);
            break;
        }

        case Excit_Triangle: {
            float v_peak = excit->triangle.v_peak;
            float accel = excit->triangle.accel;
            uint8_t seg = (uint8_t)excit->phase_rad;
            float t_in_seg = excit->time;  /* 当前段已运行时间 */

            float v_cmd = 0.0f;

            switch (seg) {
                case 0: /* 0 → +v_peak */
                    v_cmd = t_in_seg * accel;
                    if (v_cmd >= v_peak) {
                        v_cmd = v_peak;
                        excit->phase_rad = 1.0f;
                        excit->time = 0.0f;   /* 重置段内时间 */
                    }
                    break;

                case 1: /* +v_peak → 0 */
                    v_cmd = v_peak - t_in_seg * accel;
                    if (v_cmd <= 0.0f) {
                        v_cmd = 0.0f;
                        excit->phase_rad = 2.0f;
                        excit->time = 0.0f;
                    }
                    break;

                case 2: /* 0 → -v_peak */
                    v_cmd = -t_in_seg * accel;
                    if (v_cmd <= -v_peak) {
                        v_cmd = -v_peak;
                        excit->phase_rad = 3.0f;
                        excit->time = 0.0f;
                    }
                    break;

                case 3: /* -v_peak → 0 */
                default:
                    v_cmd = -v_peak + t_in_seg * accel;
                    if (v_cmd >= 0.0f) {
                        v_cmd = 0.0f;
                        excit->phase_rad = 0.0f;
                        excit->time = 0.0f;   /* 回到段0，重新开始 */
                    }
                    break;
            }

            out = offset + v_cmd;
            break;
        }

        case Excit_Sine:
            float phase = excit->phase_rad;
            out = offset + amp * lt_sin(phase);
            excit->out_cos = amp * lt_cos(phase);
            /* 更新相位 */
            phase += _2_PI * excit->freq_hz * dt;
            if (phase >= _2_PI) phase -= _2_PI;
            excit->phase_rad = phase;
            break;
            
        default:    break;
    }

    excit->time += dt;
    excit->out = out;
    return out;
}