// #include "simulate/pmsm/pmsm.h"
// #include "math/basic/lt_math.h"
// #include <string.h>

// /* 中断回调函数指针 */
// static void (*s_irq_current_callback)(void) = NULL;
// static void (*s_irq_speed_callback)(void) = NULL;
// static void (*s_irq_position_callback)(void) = NULL;

// /* 初始化电机 */
// void pmsm_init(pmsm_t *motor, pmsm_param_t *param, float dt) {
//     memset(motor, 0, sizeof(pmsm_t));
//     memcpy(&motor->param, param, sizeof(pmsm_param_t));
//     motor->dt = dt;
//     pmsm_param_t *param = &motor->param;
//     param->Ld_inv = 1.0f / param->Ld;
//     param->Lq_inv = 1.0f / param->Lq;
//     param->J_inv  = 1.0f / param->J;
//     param->ppr_inv = 1.0f / param->ppr;
// }

// /* 设置三相电压 */
// void pmsm_set_voltage(pmsm_t *motor, float va, float vb, float vc) {
//     motor->voltage.va = va;
//     motor->voltage.vb = vb;
//     motor->voltage.vc = vc;
// }

// /* 设置负载转矩 */
// void pmsm_set_load(pmsm_t *motor, float torque) {
//     motor->state.load = torque;
// }

// /* 获取三相电流 */
// void pmsm_get_current(pmsm_t *motor, pmsm_current_t *current)
// {
//     pmsm_state_t *state = &motor->state;
//     current->ia = state->ia;
//     current->ib = state->ib;
//     current->ic = state->ic;
// }

// /* 步进仿真：更新电机状态 */
// void pmsm_step(pmsm_t *motor) {
//     pmsm_param_t *param = &motor->param;
//     pmsm_state_t *state = &motor->state;
//     pmsm_voltage_t *volt = &motor->voltage;
//     float dt = motor->dt;
    
//     float V_alpha, V_beta;
//     float I_alpha, I_beta;
//     float Id, Iq;
//     float Vd, Vq;
//     float d_id, d_iq;
//     float pn = param->pn;
//     float Ld = param->Ld;
//     float Lq = param->Lq;
//     float omega = state->speed * pn;  /* 电角速度 */
//     float theta = state->theta;       /* 电角度 */
    
//     /* 1. Clark变换：三相电流→两相静止αβ */
//     I_alpha = state->ia;
//     I_beta = _SQRT_3_3 * (state->ib - state->ic);
    
//     /* 2. Park变换：两相静止αβ→旋转DQ */
//     theta = lt_normalize(theta);    /* 电角度归一化 ：[0~2*pi] */
//     float c = lt_cos(theta);
//     float s = lt_sin(theta);
//     Id = c * I_alpha + s * I_beta;
//     Iq = s * I_alpha - c * I_beta;
    
//     /* 3. 三相电压→两相静止αβ */
//     V_alpha = volt->va;
//     V_beta  = _SQRT_3_3 * (volt->vb - volt->vc);

//     /* 4. 反Park变换：两相静止αβ→旋转DQ电压 */
//     Vd = c * V_alpha + s * V_beta;
//     Vq = s * V_alpha - c * V_beta;
    
//     /* 5. 电流微分方程（前向欧拉） */
//     /* di_d/dt = (v_d - Rs*i_d + omega*Lq*i_q) / Ld */
//     /* di_q/dt = (v_q - Rs*i_q - omega*Ld*i_d - omega*flux_pm) / Lq */
//     d_id = (Vd - param->Rs * Id + omega * Lq * Iq) * param->Ld_inv;
//     d_iq = (Vq - param->Rs * Iq - omega * Ld * Id - omega * param->flux) * param->Lq_inv;
    
//     /* 6. 更新电流 */
//     Id += d_id * dt;
//     Iq += d_iq * dt;
    
//     /* 7. 反Park变换：DQ电流→两相静止αβ */
//     I_alpha = c * Id - s * Iq;
//     I_beta  = s * Id + c * Iq;

//     /* 8. 反Clark变换：两相静止αβ→三相电流 */
//     float half_alpha = 0.5f * I_alpha;                      /* 0.5f * V_alpha */
//     float beta_part =  _SQRT_3_2 * I_beta;                  /* sqrt(3)/2 * V_beta ≈ 0.8660254 * V_beta */

//     state->ia = I_alpha;
//     state->ib = -half_alpha + beta_part;
//     state->ic = -half_alpha - beta_part;
    
//     /* 9. 电磁转矩 */
//     /* Te = 1.5 * pole_pairs * (flux_pm*i_q + (Ld-Lq)*i_d*i_q) */
//     float Te = 1.5f * pn * (param->flux * Iq + (Ld - Lq) * Id * Iq);
    
//     /* 10. 机械方程 */
//     /* domega/dt = (Te - Tload - B*omega) / J */
//     float d_omega = (Te - state->load - param->B * omega) * param->J_inv;
//     omega += d_omega * dt;
//     state->speed = omega * param->pn_inv;
    
//     /* 11. 电角度更新 */
//     theta += omega * dt;
//     /* 角度归一化到 0~2π */
//     if (theta > _2_PI) {
//         theta -= _2_PI;
//     } else if (theta < 0) {
//         theta +=  _2_PI;
//     }
//     state->theta = theta;

//     float pos = state->pos + state->speed * dt;            /* 位置由速度积分得到 */
//     if(pos > _2_PI){                                       /* 保证位置精度，自动归一化 */
//         pos -= _2_PI;
//     }else if(theta < 0){
//         pos += _2_PI;
//     }
//     state->pos = pos;
//     state->count = pos * param->ppr_inv;                   /* 得到编码器计数值 */

//     /* 仿真步数累加 */
//     motor->step_count++;
//     uint64_t step = motor->step_count;

//     /* 检查中断触发 - 直接执行回调 */
//     if(motor->step_count % motor->irq_cfg.current_step == 0) {
//         if (s_irq_current_callback) {
//             s_irq_current_callback();           /* 启动电流环回调 */
//         }
//     }

//     if(motor->step_count % motor->irq_cfg.speed_step == 0) {
//         if (s_irq_speed_callback) {
//             s_irq_speed_callback();             /* 启动速度环回调 */
//         }
//     }

//     if(motor->step_count % motor->irq_cfg.position_step == 0) {
//         if (s_irq_position_callback) {
//             s_irq_position_callback();          /* 启动位置环回调 */
//         }
//     }
// }

// /*==============================================================================
//  * 设置电流环中断回调
//  *==============================================================================*/
// void pmsm_set_current_irq(void (*callback)(void)) {
//     if (!callback) return;
//     s_irq_current_callback = callback;
// }

// /*==============================================================================
//  * 设置速度环中断回调
//  *==============================================================================*/
// void pmsm_set_speed_irq(void (*callback)(void)) {
//     if (!callback) return;
//     s_irq_speed_callback = callback;
// }

// /*==============================================================================
//  * 设置位置环中断回调
//  *==============================================================================*/
// void pmsm_set_position_irq(void (*callback)(void)) {
//     if (!callback) return;
//     s_irq_position_callback = callback;
// }