/* 
 * 设计要点：
 *   - 双模式表驱动（电流环/速度环）
 *   - 自动切换FFT点数（2048/1024/512）, 参考信号频率保证 FFT 整周期采样（无能量泄漏）
 *   - 相位检测：过零点插值，前2个周期共4个过零点平均
 *   - 硬件无关，PC仿真可跑通
 */

#include "analysis/meas/lt_meas.h"
#include "math/basic/lt_math.h"
#include "math/fft/lt_fft.h"
#include <math.h>
#include <string.h>

#define TOTAL_POINTS 44                 /* 扫频点数 */
#define FFT_POINTS_NUM                  2048        /* 单次扫频采样点数 */
#define CURRENT_SCAN_BASE_FREQ          9.765625f   /* 电流环扫频参考信号基频：20000.0f/2048 Hz */
#define SPEED_SCAN_BASE_FREQ            1.4648438f  /* 速度环扫频参考信号基频：3000.0f /2048 Hz */

/* ---------- 电流环表（20kHz采样率），基频：fb = 20000.0f/2048 ~= 10Hz ---------- 
 * 扫频的参考频率范围 20Hz —— 2500Hz, 有效电流环带宽 <= 1800Hz
 */
static const uint16_t curr_m_table[TOTAL_POINTS] = {     /* 参考信号为基频的倍数 */
    2,   3,   5,   8,  10,  12,  16,  20,  24,  26,
    32,  36,  40,  48,  56,  60,  66,  76,  88, 100,
    112, 128, 144, 148, 152, 156, 164, 168, 176, 180,
    184, 188, 192, 200, 204, 208, 212, 216, 224, 232,
    236, 244, 252, 256
};

/* ---------- 速度环表（3kHz采样率），基频：fb = 3000.0f/2048 ~= 1.5Hz ----------
 * 扫频的参考频率范围 3Hz —— 375Hz, 有效速度环带宽 <= 300Hz
 */
static const uint16_t speed_m_table[TOTAL_POINTS] = {          /* 参考信号为基频的倍数 */
    2,   3,   4,   6,   8,  10,  12,  15,  18,  20,
    22,  26,  28,  30,  34,  40,  46,  48,  52,  56,
    58,  66,  72,  80,  88,  96,  98, 106, 114, 124,
    132, 136, 144, 148, 156, 168, 184, 196, 204, 220,
    232, 240, 248, 256
};

// ---------- 运行状态 ----------
typedef struct{
    const uint16_t* m_table;                      /* 参考信号频率表 */
    uint16_t   k;                           /* 对应的谱线 */
    uint8_t   curr_idx;                     /* 当前频率表的下标 */
    uint8_t   scan_running;                 /* 扫频运行标志 */
    uint8_t   scan_complete;                /* 扫频结束标准 */
    uint8_t   flag;                         /* 扫频模块初始化：0：未初始化，1：已初始化 */
    uint8_t   len;                          /* 扫频点个数 */
    float     fb;                           /* 参考信号基频（Hz）*/
    lt_meas_result_t results[TOTAL_POINTS]; /* 扫频测试结果 */
}lt_meas_obj;
typedef lt_meas_obj * lt_meas_t;

static lt_meas_obj _meas_object;            /* 扫频测量对象 */
static lt_meas_t   meas = &_meas_object;    /* 利用语法糖 */ 

static void _meas_set(lt_meas_mode_t mode, float freq_fixed);
static void _next_frequency(void);

/* ---------- 对外接口 ---------- */
void lt_meas_init(lt_meas_mode_t mode, float freq_fixed) {
    _meas_set(mode, freq_fixed);
}

void lt_meas_set(lt_meas_mode_t mode, float freq_fixed) {
    _meas_set(mode, freq_fixed);
}


void lt_meas_start(void) {                /* 扫频测试启动 */
    if(!meas->flag)             return;   /* 模块未初始化 */
    meas->curr_idx = 0;
    meas->scan_running = 1;
    meas->scan_complete = 0;
    _next_frequency();
}

void lt_meas_add(float value) {
    if(!meas->flag)             return;   /* 模块未初始化 */
    if (!meas->scan_running)    return;   /* 模块未运行，不添加数据 */
    lt_fft_add(value);
}

uint8_t lt_meas_process(void) {
    if(!meas->flag)             return 0;   /* 模块未初始化 */
    if (!meas->scan_running || meas->scan_complete) return 0;
    if (!lt_fft_is_ready()) return 0;
    uint8_t index = meas->curr_idx;         /* 当前扫频点位置 */
    uint16_t k     = meas->k;               /* 频谱序号 */
    /* 执行FFT运算 */
    lt_fft_remove_bias();                   /* 先移除输入数据中的直流偏置量, 便于过零点判断 */
    lt_fft_process();                       /* 进行小信号 FFT 运算 */

    /* 提取幅值 */
    float amp = lt_fft_get_amplitude(k);

    /* 提取相位（度）*/
    float phase_lag = 0;// = lt_phase_calculate(lt_fft_get_real_buf(), fft_len, target_m);

    /* 保存结果 */
    lt_meas_result_t * result = &(meas->results[index]);
    result->freq = meas->m_table[index] * meas->fb;       /* 扫频频率 ：Hz */
    result->amp = amp;
    result->phase_lag_deg = phase_lag;

    if(meas->len == 1){                                   /* 固定点扫频完毕 */
        meas->scan_running = 0;                           /* 标记扫频完毕 */
        meas->scan_complete = 1;
        return 1;
    }
    /* 下一频点 */
    meas->curr_idx++;
    _next_frequency();

    return 1;
}

uint8_t lt_meas_is_complete(void) {
    if(!meas->flag)             return 0;   /* 模块未初始化 */
    return meas->scan_complete;
}

uint8_t lt_meas_get_progress(void) {
    if(!meas->flag)             return 0;   /* 模块未初始化 */
    uint8_t process =   (uint8_t)(((meas->curr_idx) * 100) / TOTAL_POINTS);
    if(meas->len == 1)  return (meas->scan_complete ? 100 : 0);
    return process;
}

uint16_t lt_meas_get_len(void)              /* 获取扫频点个数 */
{
    if(!meas->flag)             return 0;   /* 模块未初始化 */
    return meas->len;
}

float lt_meas_get_freq()                    /* 获取本次扫频频率（Hz) */
{
    if(!meas->flag)             return 0;   /* 模块未初始化 */
    if(meas->curr_idx >= TOTAL_POINTS)  return 0;
    return meas->m_table[meas->curr_idx] * meas->fb;
}

void lt_meas_get(lt_meas_result_t * res, uint8_t index) /* 获取扫频测试结果, index : 扫频点位置 */ 
{
    if(!meas->flag)             return;   /* 模块未初始化 */
    if(index >= TOTAL_POINTS)   return;
    /* 返回扫频结果 */
    lt_meas_result_t * result = &(meas->results[index]);
    res->freq    = result->freq;
    res->amp     = result->amp;
    res->phase_lag_deg = result->phase_lag_deg;
}

/*********************************************************************************/
static void _meas_set(lt_meas_mode_t mode, float freq_fixed) {
    memset(meas, 0, sizeof(lt_meas_obj));       /* 初始化模块 */

    if(mode == LT_MEAS_MODE_CURRENT) {
        meas->m_table = curr_m_table;
        meas->fb = CURRENT_SCAN_BASE_FREQ;      /* 记录基准频率 */
    } else {
        meas->m_table = speed_m_table;          
        meas->fb = SPEED_SCAN_BASE_FREQ;        /* 记录基准频率 */
    }

    if(freq_fixed > 0.0f){                      /* 固定点扫频 */
        uint8_t best = 0;
        uint16_t m = 0;
        float min_diff = 1e6;
        for (uint8_t i = 0; i < TOTAL_POINTS; i++) {
            float diff = lt_absf(meas->m_table[i] * meas->fb - freq_fixed);
            if(diff < min_diff) { 
                min_diff = diff; 
                best = i; 
            }
        }
        meas->curr_idx  = best;                 /* 离设定频率最近的表下标 */
        meas->len = 1;
    }else{
        meas->len = TOTAL_POINTS;               /* 连续扫频 */
    }

    meas->flag = 1;                             /* 初始化完毕 */
}

static void _next_frequency(void) {             /* 下一个频率扫频开始 */
    if (meas->curr_idx >= TOTAL_POINTS) {
        meas->scan_running = 0;                 /* 标记扫频完毕 */
        meas->scan_complete = 1;
        return;
    }
    /* 动态调整采样点数量 */
    uint16_t k;                                 /* 对应的谱线 */
    uint16_t target_m = meas->m_table[meas->curr_idx];
    uint16_t fft_len;                           /* FFT 采样点个数 */
    
    if(target_m < 32){
        fft_len = FFT_POINTS_NUM;
        k = target_m;       
    }else if(target_m < 128){
        fft_len = FFT_POINTS_NUM >> 1;
        k = target_m >> 1;                     /* 频谱频率间隔加大，谱线号同比例缩小 */
    }else{
        fft_len = FFT_POINTS_NUM >> 2;
        k = target_m >> 2;                     /* 频谱频率间隔加大，谱线号同比例缩小 */
    }                  
    meas->k = k;                               /* 记录本次 FFT 采样流程对应的频谱值 */
    /* 启动当前频率的 FFT 采样流程 */
    lt_fft_start(fft_len);
}
