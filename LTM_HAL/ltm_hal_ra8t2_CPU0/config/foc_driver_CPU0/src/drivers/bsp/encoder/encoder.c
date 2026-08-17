/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-04-10     Houjiayu     The first version
 * 2026-07-20     Lvtou        精简多圈绝对值编码器实现 
 */
#include "bsp/encoder/encoder.h"

/* 默认CRC8函数指针（初始为NULL） */
static crc8_check_t math_crc8 = NULL;
static uint8_t sample_cnt = STARTUP_SYNC_SAMPLES;  /* 记录上电同步采样次数 */
static int accum_spindle = 0;              /* 上电累积值 */
static int accum_driven  = 0;

/* 多圈绝对值编码器由两个单圈绝对值编码器构成 */
typedef struct {                    /* 单圈编码器实例 */
    uint32_t count_last;            /* 上一次计数值，用于圈数更新 */
    uint32_t count;                 /* 编码器原始计数值 */
    int32_t  circle_count;          /* 圈数计数器 */
    spi_id_t spi_id;                /* 外设对应 SPI编号 */
} encoder_config_t;

static encoder_config_t g_encoder_cfg[2] = {
    { 0,   0,  0, SPINDLE_SPI },       /* 主编码器 */
    { 0,   0,  0, DRIVEN_SPI  }        /* 从编码器 */
};


static int   _encoder_update(uint8_t id);         /* 单个编码器数据更新, 返回值 <0 时，说明读取出错 */
static void  _finish_startup(int accum_spindle, int accum_driven);   /* 联合解圈 */  
static float _wrap_half_f(float ref, float meas, float range);  /* 简易半圈法，测差值 */

static inline float _normalize(float value, float range)        /* 简易归一化实现:[0, range) */
{
    while (value >= range)  value -= range;
    while (value <  0.0f)   value += range;
    return value;
} 

void encoder_init(void)
{
    spi_init();                                     /* SPI 外设初始化 */
    /* 清空上电采集相关变量 */
    sample_cnt    = STARTUP_SYNC_SAMPLES;           
    accum_spindle = 0;
    accum_driven  = 0;
    math_crc8     = NULL;                          /* 校验指针置零 */
    /* 编码器置零 */                              
    encoder_set_zero();
}

void encoder_update(void)                          /* 手动更新编码器计数（高频调用）*/   
{
    int raw_spindle = _encoder_update(0);          /* 主编码器计数值 */ 
    int raw_driven  = 0;                           /* 从编码器计数值 */

    if(sample_cnt > 0){                            /* 仅上电同步阶段采集从编码器值 */
        raw_driven = _encoder_update(1);
        if(raw_spindle >= 0 && raw_driven >= 0){   /* 编码器读数正常 */
            accum_spindle += raw_spindle;
            accum_driven  += raw_driven;
            sample_cnt--;
        }
        /* 上电同步采样完毕，开始联合解圈 */
        if(sample_cnt <= 0)   _finish_startup(accum_spindle, accum_driven);   
        return; 
    }
    /* 主编码累积圈数更新（半圈法） */
    if(raw_spindle < 0) return;  /* 读取失败直接返回 */
    encoder_config_t * config = &g_encoder_cfg[0];
    int32_t count_delt = (int32_t)raw_spindle - (int32_t)(config->count_last);
    
    if(count_delt > ENCODER_CPR_HALF){             /* 主轴反转 */
        config->circle_count--;
    }else if(count_delt < -ENCODER_CPR_HALF){      /* 主轴正转 */                                      
        config->circle_count++;
    }

    config->count_last = raw_spindle;              /* 更新上一次计数值 */
}

void encoder_set_crc8_check(crc8_check_t fn)       /* 注册外部CRC8校验函数（依赖注入）*/     
{
    if(!fn)                 return;                /* 判空 */
    math_crc8 = fn;                                /* 绑定 CRC8 校验函数 */ 
}

uint32_t encoder_get_count(void)                    /* 获取当前计数值（0~ENCODER_CPR -1）*/
{
    return g_encoder_cfg[0].count;                 
}

int64_t  encoder_get_position(void)                 /* 获取累计位置（计数值，可跨圈） */
{
    encoder_config_t *config = &g_encoder_cfg[0];
    return (int64_t)config->count + ENCODER_CPR * (int64_t)config->circle_count;
}

float encoder_get_angle_deg(void)                  /* 获取机械角度（度） */
{
    return (float)encoder_get_position() * ENCODER_DEG_PER_COUNT; 
}

float encoder_get_angle_rad(void)                  /* 获取机械角度（弧度） */
{
    return (float)encoder_get_position() * ENCODER_RAD_PER_COUNT;
}

void encoder_set_zero(void)                        /* 软归零（清零硬件计数器和圈数） */
{
    for(int i = 0; i < 2; i++){
        encoder_config_t * config = &g_encoder_cfg[i];
        int count = _encoder_update(i);            /* 读取一次位置值 */
        count = (count < 0) ? 0 : count;           /* 判断读取值是否合理 */
        config->count = count;
        config->count_last = count;
        config->circle_count = 0;
    }
}


/***********************************************************************************************/
/* 单个编码器数据更新, 返回值 <0 时，说明读取出错 */
static int _encoder_update(uint8_t id)                        
{  
    if(id >= 2)                    return -1;
    int8_t dir = (id == 0 )? ENCODER_SPINDLE_DIR : ENCODER_DRIVEN_DIR;
    encoder_config_t * config = &g_encoder_cfg[id];
    /* 单圈编码器 SPI 数据读取 */
    uint16_t tx_buf[3] = { ContinuousRead, 0xFFFF, 0xFFFF };
    uint16_t rx_buf[3];
    int res = spi_write_read(config->spi_id, tx_buf, rx_buf, 3);      /* 同步阻塞读取 */
    if(!res)                       return -1;

    /* 字段解析开始 */
    uint16_t w1 = rx_buf[1];
    uint16_t w2 = rx_buf[2];
    uint8_t data_8bits[4];
    data_8bits[0] = (uint8_t)(w1 >> 8);    /* Angle[20:13] */
    data_8bits[1] = (uint8_t)(w1 & 0xFF);  /* Angle[12:5]  */
    data_8bits[2] = (uint8_t)(w2 >> 8);    /* Angle[4:0] | Status[2:0] */
    data_8bits[3] = (uint8_t)(w2 & 0xFF);  /* CRC */

    uint8_t crc = 0;
    if(math_crc8)   crc = math_crc8(data_8bits, 3); /* 用户必须手动注入 CRC8 校验函数指针 */
    if(crc != data_8bits[3])        return -1;      /* CRC 校验失败，直接返回 */     

    uint32_t count_raw = ((uint32_t)data_8bits[0] << 13)
                        | ((uint32_t)data_8bits[1] << 5)
                        | (data_8bits[2] >> 3);
    count_raw = count_raw >> 3;                    /* 21bit -> 18bit */
    /* 编码器方向自动转换（公式适用于2^n分辨率编码器），并更新计数值 */
    count_raw = (dir >= 0) ? count_raw : ((ENCODER_CPR - count_raw) & (ENCODER_CPR - 1));
    config->count = count_raw;

    return (int)count_raw;
}

static void _finish_startup(int accum_spin, int accum_dri)   /* 联合解圈 */  
{
    int32_t best_n   = STARTUP_TURN_MIN;
    float   best_err = 1.0f;
    float   sp_frac = (float)accum_spin * STARTUP_ACCUM_TO_FRAC;   /* 范围：[0, 1.0f) */
    float   dr_frac = (float)accum_dri  * STARTUP_ACCUM_TO_FRAC;   /* 范围：[0, 1.0f) */

    /* 联合解圈开始，显然圈数不能太多, 避免解算时间过长 */
    for (int32_t n = STARTUP_TURN_MIN; n <= STARTUP_TURN_MAX; n++)
    {
        float pred_dr   = ((float)n + sp_frac) * GEAR_RATIO + GEAR_PHASE_OFFSET_TURN;
        float pred_frac = _normalize(pred_dr, 1.0f);                  /* 归一化到 [0, 1.0f) */
        float err       = _wrap_half_f(pred_frac, dr_frac, 1.0f);     /* 半圈法测误差 */
        if (err < best_err) { best_err = err; best_n = n; }
    }
    /* 记录主编码器参数 */
    encoder_config_t * config = &g_encoder_cfg[0];
    uint32_t count = (uint32_t)(sp_frac * ENCODER_CPR_F);
    config->circle_count = best_n;
    config->count        = count;
    config->count_last   = count;
}

static float _wrap_half_f(float ref, float meas, float range)  /* 简易半圈法，测差值 */
{
    float e = ref - meas;
    float half = range * 0.5f;
    while (e > half)  e -= range;
    while (e < -half) e += range;
    return (e >= 0) ? e : -e;
}