/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-04-10     Houjiayu     The first version
 * 2026-07-20     Lvtou        精简多圈绝对值编码器实现
 * 2026-08-12     Lvtou        取消CRC8依赖注入，直接实现查表法。
 */
#include "bsp/encoder/encoder.h"

/* ==================== CRC8 (多项式 0x07)，X^8+X^2+X^1+1   ==================== */
static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
    0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
    0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
    0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
    0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
    0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
    0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
    0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
    0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
    0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
    0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
    0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
    0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
    0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
    0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
    0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
    0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

/*  CRC8 查表法计算（多项式 0x07，初始值 0x00）, 返回 CRC校验值 */
static uint8_t _crc8_check(const uint8_t *data, uint8_t len) 
{
    uint8_t crc = 0x00;
    while (len--) {
        crc = crc8_table[crc ^ *data++];
    }
    return crc;
}

static volatile int sample_cnt = STARTUP_SYNC_SAMPLES;  /* 记录上电同步采样次数 */
static volatile int accum_spindle = 0;                  /* 上电累积值 */
static volatile int accum_driven  = 0;

/* 多圈绝对值编码器由两个单圈绝对值编码器构成 */
typedef struct {                    /* 单圈编码器实例 */
    uint32_t count_last;            /* 上一次计数值，用于圈数更新 */
    uint32_t count;                 /* 编码器原始计数值 */
    int32_t  circle_count;          /* 圈数计数器 */
    spi_id_t spi_id;                /* 外设对应 SPI编号 */
} encoder_config_t;

static encoder_config_t g_encoder_cfg[2] = {
    { 0,  0,  0, SPINDLE_SPI },       /* 主编码器 */
    { 0,  0,  0, DRIVEN_SPI  }        /* 从编码器 */
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

    uint8_t crc = _crc8_check(data_8bits, 3);
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