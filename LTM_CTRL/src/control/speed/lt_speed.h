#ifndef LT_SPEED_H
#define LT_SPEED_H

#include <stdint.h>

/* 自适应 M 法开关：
 *   1 = 启用：M 法测得准，主要用于评估/对比 PLL 参数设计的效果
 *   0 = 关闭（默认）：热路径更短
 * 关闭时 ADAP_M 实例的输出恒为 0 */
#ifndef LT_SPEED_USE_ADAP_M
#define LT_SPEED_USE_ADAP_M             0
#endif

/* 测速实例池：无堆，idx 直接映射（与 lt_pid 同款）*/
#ifndef LT_SPEED_MAX_INSTANCES
#define LT_SPEED_MAX_INSTANCES          4
#endif

/* 测速方式：一个实例一种，互不干扰 */
typedef enum {
    LT_SPEED_TYPE_PLL      = 0,     /* 锁相环：带宽可整定，中高速主用 */
    LT_SPEED_TYPE_T        = 1,     /* T 法（周期法）：低速主用，误差只跟窗口拍数有关 */
    LT_SPEED_TYPE_ADAP_M   = 2      /* 自适应 M 法：需打开 LT_SPEED_USE_ADAP_M */
} lt_speed_type_t;

/*------------------------- 测速接口（count/s，高频调用）---------------------------------*/

/* reso：编码器分辨率；freq：调用频率（Hz）；返回 0 = 索引越界或参数非法 */
uint8_t lt_speed_init(uint8_t idx, uint8_t type, uint32_t reso, uint32_t freq);
void    lt_speed_reset(uint8_t idx);                    /* 清状态，保留配置 */
void    lt_speed_set(uint8_t idx, uint32_t pll_kp, uint32_t pll_ki);  /* 仅 PLL 用 */
void    lt_speed_update(uint8_t idx, uint32_t pos_count);
int32_t lt_speed_get(uint8_t idx);                      /* 返回 count/s，未初始化返回 0 */

/* ==================== 锁相环参数整定（lt_speed_set） ====================
 *   1) 选带宽 f_n：速度环常用 20~100 Hz，锁相环取前者的 3~5 倍。
 *   2) 选阻尼比 ζ：0.7（最快、超调约 4%）/ 1.0（无超调），一般取 0.7。
 *   3) 反算增益：
 *          Kp = 4π·ζ·f_n               （只跟 ζ、f_n 有关）
 *          Ki = (2π·f_n)² / freq       （反比于调用频率）
 *      例：freq = 20 kHz、f_n = 300 Hz、ζ = 0.7 → Kp = 2639、Ki = 178
 *
 *   测速输出的低通系数按 Ki 自动整定，不用配。
 * ======================================================================== */

#endif
