#ifndef LT_SPEED_H
#define LT_SPEED_H

#include <stdint.h>

/*自适应 M 法开关：
 *   1 = 启用：M 法测得准，主要用于评估/对比 PLL 参数设计的效果
 *   0 = 关闭（默认）：只跑锁相环，热路径更短
 * 关闭时 lt_speed_get() 的 adap_speed 恒为 0 */
#ifndef LT_SPEED_USE_ADAP_M
#define LT_SPEED_USE_ADAP_M             0
#endif

/*------------------------- 测速接口 （count/s，高频调用）---------------------------------*/

void lt_speed_init(uint32_t reso, uint32_t freq);	/* reso : 编码器分辨率， freq : 调用频率（Hz） */
void lt_speed_set(uint32_t pll_kp, uint32_t pll_ki);/* pll_kp : 比例增益（1/s），pll_ki : 积分增益（1/s^2），内部已乘调用周期 dt */
void lt_speed_update(uint32_t pos_count);		    /* 测速更新，输入编码器原始值：count（单圈计数 0~reso-1）*/
void lt_speed_get(int32_t *adap_speed, int32_t *pll_speed); /* 返回锁相环测速结果（count/s）；adap_speed 仅在开关打开时有效，否则恒为 0 */

/* ==================== 锁相环参数整定（lt_speed_set） ====================
 *   1) 选带宽 f_n：速度环常用 20~100 Hz，锁相环取前者的 3~5 倍。
 *   2) 选阻尼比 ζ：0.7（最快、超调约 4%）~ 1.0（无超调），一般取 0.7。
 *   3) 反算增益：
 *          Kp = 4π·ζ·f_n              （只跟 ζ、f_n 有关）
 *          Ki = (2π·f_n)² / freq      （反比于调用频率）
 *      例：freq = 20 kHz、f_n = 300 Hz、ζ = 0.7 → Kp = 2639、Ki = 178
 *
 *   测速输出的低通系数按 Ki 自动整定，不用配。
 * ======================================================================== */

#endif
