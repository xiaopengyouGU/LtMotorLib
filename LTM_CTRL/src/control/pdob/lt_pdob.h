#ifndef LT_PDOB_H
#define LT_PDOB_H

#include <stdint.h>

/*---------------- 速度环 DOB+P ＋ 电流环无差拍 DPCC/ESO ----------------
 * 对外单位：速度 count/s；电流、电压一律 Q15 标幺（32767 = 1.0 pu）。
 *   id、iq：32767 = i_base_mA      vbus：32767 = 满量程母线
 *   Vd、Vq：1.0 pu = 实测母线/√3，可直接进 lt_foc_update
 *   内部相电压基准 V_B = 3·v_base_mV/√3（1.0pu(V_B) = 3 倍线性边界，调制限幅 1/3 pu）
 *
 *   static const lt_pdob_config_t cfg = {
 *       .reso = 4096,        .pp = 10,              .freq_speed = 5000,
 *       .freq_curr = 20000,   .freq_max = 600,       // 填最高转速，不是基速
 *       .Ls_uH = 333,        .Rs_mohm = 560,        .phi_uWb = 13800,
 *       .i_base_mA = 33000,  .v_base_mV = 60000,
 *       .J_nNm_s2 = 50000,   .speed_max = 12000000, .d_limit_iq = 60000 };
 *
 *   lt_pdob_init(&cfg);
 *   lt_pdob_speed_set(kp_q15, 800, 32767);   // kp / DOB带宽(rad/s) / iq限幅（给总电流预算）
 *   lt_pdob_curr_set(1200, 32767);           // ESO带宽(rad/s) / 电压限幅
 *   lt_pdob_fw_set(3000);                    // 弱磁（可选）：反电势 > 0.85pu 自动开，0 = 关
 *
 *   iq_ref = lt_pdob_speed_update(speed, iq);      // 每 freq_speed 拍，we 也在这句更新
 *   lt_pdob_curr_update(id, iq, vbus, &Vd, &Vq);   // 每 freq_curr 拍
 * -------------------------------------------------------------------*/

typedef struct {
    uint32_t reso, pp;                  /* 编码器单圈分辨率、极对数 */
    uint32_t freq_speed, freq_curr;        /* 速度环 / 电流环调用频率（Hz）*/
    uint32_t freq_max;                  /* 频率基准：最高转速对应的电频率（Hz）*/
    int32_t  Ls_uH, Rs_mohm, phi_uWb;   /* 相电感、相电阻、永磁磁链 */
    int32_t  i_base_mA;                 /* 电流基准：ADC 可测最大峰值电流 */
    int32_t  v_base_mV;                 /* 电压基准：ADC 满量程母线电压 */
    int32_t  J_nNm_s2;                  /* 总转动惯量 nN·m·s²（5e-5 kg·m² → 50000）*/
    int32_t  speed_max;                 /* 最高转速 count/s（同时是速度环标幺基准）*/
    int32_t  d_limit_iq;                /* 扰动等效 iq 上限（Q15）*/
} lt_pdob_config_t;

void lt_pdob_init(const lt_pdob_config_t *cfg);                     /* 上电 / 换电机一次 */

void lt_pdob_speed_set(int32_t kp, int32_t wb, int32_t out_limit);  /* kp / DOB带宽(rad/s) / iq限幅（给总电流预算）*/
void lt_pdob_curr_set(int32_t eso_wb, int32_t out_limit);           /* ESO带宽(rad/s，0关) / 电压限幅 */
void lt_pdob_fw_set(int32_t ki);   /* 弱磁增益（Q15），0 = 关；标定值与 V_B² 成反比 */
void lt_pdob_set_speed_ref(int32_t speed_ref);                      /* 速度指令 count/s */
void lt_pdob_set_curr_ref(int32_t id_ref, int32_t iq_ref);          /* 手动 id/iq（Q15）*/
int32_t lt_pdob_speed_update(int32_t speed, int32_t iq);            /* 每 freq_speed 拍 */
void lt_pdob_curr_update(int32_t id, int32_t iq, int32_t vbus,      /* 每 freq_curr 拍 */
                         int32_t *vd, int32_t *uq);
void lt_pdob_reset(void);                                           /* 清运行状态，保留配置 */

#endif
