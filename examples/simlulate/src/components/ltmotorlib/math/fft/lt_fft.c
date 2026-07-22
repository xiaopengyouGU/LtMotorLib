/* 实现功能：
 *   - 基 2 实数 FFT，变点数 512/1024/2048
 *   - 使用查表 sin/cos（lt_sin / lt_cos）
 *   - 原地 FFT（In-Place），内存占用仅 2 * max_len * float
 *   - 整周期采样配合，无能量泄漏，无需加窗
 */

#include "math/basic/lt_math.h"
#include "math/fft/lt_fft.h"
#include <string.h>

#define FFT_MAX_LEN 2048                    /* 允许的最大 FFT 点数 */

static float real_buf[FFT_MAX_LEN];         /* 实部缓冲区 */
static float imag_buf[FFT_MAX_LEN];         /* 虚部缓冲区 */
static uint16_t fft_len = 0;                /* FFT 采样点数量 */
static uint16_t data_cnt = 0;               /* 当前采样点数 */
static uint8_t  data_ready = 0;             /* 数据记录完毕 */

/* ---------- 内部辅助函数 ---------- */
static uint16_t bit_reverse(uint16_t x, uint16_t bits);     /* 位反转 */
static void fft_core(float* real, float* imag, uint16_t n); /* 原地 FFT 计算 */

/* ---------- 对外接口 ---------- */
void lt_fft_init(uint16_t max_len) {
    (void)max_len;      /* 预留，固定 FFT_MAX_LEN */
    memset(real_buf, 0, sizeof(real_buf));
    memset(imag_buf, 0, sizeof(imag_buf));
    fft_len = 0;
    data_cnt = 0;
    data_ready = 0;
}

void lt_fft_start(uint16_t len) {                /* 设置 FFT 的数据点数 */
    if(len > FFT_MAX_LEN)   len = FFT_MAX_LEN;   /* 限制最大采样点 */
    fft_len = len;
    data_cnt = 0;
    data_ready = 0;
    memset(real_buf, 0, len * sizeof(float));
    memset(imag_buf, 0, len * sizeof(float));
}

void lt_fft_add(float value) {                    /* 添加采样数据 */  
    if (data_cnt < fft_len) {
        real_buf[data_cnt++] = value;             
    }else{
        data_ready = 1;                           /* 一次 FFT 采样流程完毕 */
    }
}

uint8_t lt_fft_is_ready(void) {
    return data_ready;
}

void lt_fft_remove_bias(void)                    /* 移除 FFT 采样数据中的固定偏置 */
{
    if (!data_ready) return;                      /* 采样完毕前，不进行计算 */
    float bias = 0;
    for(uint16_t i = 0; i < fft_len; i++){
        bias += real_buf[i];    
    }
    bias /= fft_len;                             /* 得到直流偏置量 */
    /* 移除偏置量 */
    for(uint16_t i = 0; i < fft_len; i++){
        real_buf[i] -= bias;
    }
}

void lt_fft_process(void) {
    if (!data_ready) return;                      /* 采样完毕前，不进行计算 */
    fft_core(real_buf, imag_buf, fft_len);
}

/*  k 谱线索引 (0 ~ N/2)，N：FFT 采样点数，fs : 采样频率（Hz）
 *  对应频率 f = k * fs / N
 *  k=0 为直流，k=N/2 为奈奎斯特频率
 *  返回绝对幅值，越界返回 0
 */
float lt_fft_get_amplitude(uint16_t k) {           /* 获取绝对幅值 */
    if (k >= fft_len) return 0.0f;
    
    uint16_t half_len = fft_len >> 1;
    /* 如果请求的是负频率，自动映射到对应的正频率 */
    if (k > half_len) {
        k = fft_len - k;                   /* eg：half_len = 256， k=300 → k=212 */
    }
    float real_val = real_buf[k];
    float imag_val = imag_buf[k];
    float amplitude = lt_sqrt(real_val * real_val + imag_val * imag_val);
    
    /* 幅值修正 */
    if (k == 0 || k == half_len) {
        return amplitude / fft_len;        /* 直流量/奈奎斯特不乘2 */
    }else{
        return 2.0f * amplitude / fft_len; /* 正频率乘2 */
    }
}

uint16_t lt_fft_get_len(void) {                             /* 返回有效数据点数 */
    return fft_len;
}

/*************************************************************************************/
static uint16_t bit_reverse(uint16_t x, uint16_t bits) {    /* 位反转 */
    uint16_t y = 0;
    for (uint16_t i = 0; i < bits; i++) {
        y = (y << 1) | (x & 1);
        x >>= 1;
    }
    return y;
}

/* real: 实部缓冲区，imag: 虚部缓冲区， len: FFT 的总采样点数 
 * 算法时间复杂度 ：O(nlog(n))，空间复杂度：O(n)
 */
static void fft_core(float* real, float* imag, uint16_t len) {
    /* ========== Step 1: 计算比特位数 ========== */
    uint16_t bits = 0;
    uint16_t tmp = len;
    while (tmp >>= 1) bits++;   /* 例如 len=512 → bits=9 */
    len = 1 << bits;            /* 确保 len 是2的幂（防止外部传入非2幂值） */
    
    /* ========== Step 2: 位反转重排 ========== 
     * 时域抽取（DIT）FFT 要求输入按比特反转顺序排列
     * 例如 len=8 (bits=3)：
     *   原始索引 0,1,2,3,4,5,6,7
     *   反转后   0,4,2,6,1,5,3,7
     */
    for (uint16_t i = 0; i < len; i++) {
        uint16_t j = bit_reverse(i, bits);  /* 计算比特反转索引 */
        if (i < j) {
            /* 交换实部 */
            float tr = real[i];
            real[i] = real[j];
            real[j] = tr;
            
            /* 交换虚部 */
            float ti = imag[i];
            imag[i] = imag[j];
            imag[j] = ti;
        }
    }
    
    /* ========== Step 3: 多级蝶形运算 ========== 
     * stage: 当前蝶形的跨度（即两个输入节点的距离）
     * 
     * 第1级：stage=1，  合并相邻2个点（间隔1）
     * 第2级：stage=2，  合并相邻4个点（间隔2）
     * 第3级：stage=4，  合并相邻8个点（间隔4）
     * ...
     * 第m级：stage=2^(m-1)
     * 
     * 总级数 = log2(N)
     */
    uint16_t stage = 1;
    while (stage < len) {
        uint16_t step = stage << 1;  /* step = 2*stage，当前蝶形组的大小 */
        
        /* 旋转因子角度步长：
         * 第m级有 2^(m-1) 个不同的旋转因子
         * 角度步长 = -2π / step
         */
        float angle_step = - _2_PI / step;
        
        /* 遍历所有蝶形组 */
        for (uint16_t i = 0; i < len; i += step) {
            /* 遍历当前组内的蝶形 */
            for (uint16_t j = 0; j < stage; j++) {
                /* ===== 蝶形的两个输入节点 =====
                 * idx1: 上节点（蝶形的左半部分）
                 * idx2: 下节点（蝶形的右半部分）
                 * 
                 * 例：stage=4，i=0，j=1
                 *   idx1 = 0+1 = 1
                 *   idx2 = 1+4 = 5
                 *   表示合并 X[1] 和 X[5]
                 */
                uint16_t idx1 = i + j;
                uint16_t idx2 = idx1 + stage;
                
                /* ===== 计算旋转因子 W = cos(θ) - j*sin(θ) =====
                 * θ = angle_step * j = -2π*j/step
                 * 注意：加上 _2_PI 将角度映射到 [0, 2π) 范围
                 */
                float angle = angle_step * j + _2_PI;
                float wr = lt_cos(angle);   /* 旋转因子实部 cos(θ) */
                float wi = lt_sin(angle);   /* 旋转因子虚部 sin(θ) 
                                             * 注意：代码中用正sin，但实际应为 -sin
                                             * 但因为 θ 为负，所以 lt_sin(负) = -正sin
                                             * 等价于旋转因子 = cos(θ) - j*sin(θ) ✓
                                             */
                
                /* ===== 保存下节点的值（因为会被覆盖） ===== */
                float tr = real[idx2];      /* 下节点实部 */
                float ti = imag[idx2];      /* 下节点虚部 */
                
                /* ===== 复数乘法：W * X[idx2] =====
                 * (wr + j*wi) * (tr + j*ti)
                 * = (wr*tr - wi*ti) + j*(wr*ti + wi*tr)
                 */
                float tmp_real = tr * wr - ti * wi;  /* 乘积实部 */
                float tmp_imag = tr * wi + ti * wr;  /* 乘积虚部 */
                
                /* ===== 蝶形运算（核心） =====
                 * X[idx1] = X[idx1] + W * X[idx2]
                 * X[idx2] = X[idx1] - W * X[idx2]
                 * 
                 * 注意：X[idx1] 的上节点值在后面才更新
                 * 必须用 tmp_real/tmp_imag 而不是直接修改 real[idx2]
                 */
                real[idx2] = real[idx1] - tmp_real;
                imag[idx2] = imag[idx1] - tmp_imag;
                real[idx1] = real[idx1] + tmp_real;
                imag[idx1] = imag[idx1] + tmp_imag;
            }
        }
        stage = step;  /* 进入下一级，跨度翻倍 */
    }
}
