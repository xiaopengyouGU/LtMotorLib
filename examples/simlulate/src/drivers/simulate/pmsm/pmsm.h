// #ifndef PMSM_H
// #define PMSM_H

// /* 高精度虚拟电机模型 */
// #include <stdint.h>

// #ifdef __cplusplus
// extern "C" {
// #endif

// /* 电机参数结构体 */
// typedef struct {
//     float Rs;           /* 定子电阻，单位：Ω */
//     float Ld;           /* D轴电感，单位：H */
//     float Lq;           /* Q轴电感，单位：H */
//     float flux;         /* 永磁体磁链，单位：Wb */
//     float pn;           /* 极对数 */
//     float J;            /* 转动惯量，单位：kg·m² */
//     float B;            /* 阻尼系数，单位：N·m·s/rad */
//     uint32_t ppr;       /* 电机编码器分辨率 */
//     /* 计算系数 */
//     float Ld_inv;
//     float Lq_inv;
//     float J_inv;
//     float pn_inv;
//     float ppr_inv;
// } pmsm_param_t;

// typedef struct {
//     float ia;          /* A相电流，单位：A */
//     float ib;          /* B相电流，单位：A */
//     float ic;          /* C相电流，单位：A */
// }pmsm_current_t;

// /* 电机状态结构体 */
// typedef struct {
//     float id;          /* D轴电流，单位：A */
//     float iq;          /* Q轴电流，单位：A */
//     float ia;          /* A相电流，单位：A */
//     float ib;          /* B相电流，单位：A */
//     float ic;          /* C相电流，单位：A */
//     float theta;       /* 电角度，  单位：rad */
//     float pos;         /* 转子位置，单位：rad */
//     float speed;       /* 转子速度，单位：rad/s */
//     float torque;      /* 电磁转矩，单位：N·m */
//     float load;        /* 负载转矩，单位：N·m */
//     int32_t count;     /* 编码器计数值 ：Unit */
//     uint64_t motor_step;/* 电机仿真步数，对应物理时间 */
// } pmsm_state_t;

// /* 电压输入结构体 */
// typedef struct {
//     float va;          /* A相电压，单位：V */
//     float vb;          /* B相电压，单位：V */
//     float vc;          /* C相电压，单位：V */
// } pmsm_voltage_t;

// /* 中断配置 */
// typedef struct {
//     uint32_t current_step;   /* 电流环中断步数 (例: 500步 = 50us @0.1us步长) */
//     uint32_t speed_step;     /* 速度环中断步数 (例: 3333步 = 333.3us) */
//     uint32_t position_step;  /* 位置环中断步数 (例: 10000步 = 1ms) */
// } pmsm_interrupt_t;

// /* PMSM对象 */
// typedef struct {
//     pmsm_param_t param;           /* 电机参数 */
//     pmsm_state_t state;           /* 电机状态 */
//     pmsm_voltage_t voltage;       /* 当前电压 */
//     pmsm_interrupt_t irq_cfg;     /* 中断配置 */
//     uint64_t step_count;          /* 仿真步数计数器 */
//     float dt;                     /* 仿真步长，单位：秒 */
// } pmsm_t;

// /* 公共接口 */
// void pmsm_init(pmsm_t *motor, pmsm_param_t *param, float dt);
// void pmsm_set_voltage(pmsm_t *motor, float v_a, float v_b, float v_c);
// void pmsm_step(pmsm_t *motor);                      /* 单步步进 */
// void pmsm_set_load(pmsm_t *motor, float torque);    /* 设置负载 ：N.m */
// void pmsm_get_current(pmsm_t *motor, pmsm_current_t *current);  /* 获取三相电流 */
// void pmsm_set_pos_callback(void (*callback)(void));      /* 设置位置环中断回调 */
// void pmsm_set_speed_callback(void (*callback)(void));    /* 设置速度环中断回调 */
// void pmsm_set_current_callback(void (*callback)(void));  /* 设置速度环中断回调 */

// static inline int32_t pmsm_get_count(pmsm_t *motor)       { return motor->state.count; }  /* 返回编码器计数值 */

// static inline uint64_t pmsm_get_step_count(pmsm_t *motor) {
//     return motor->step_count;
// }

// static inline float pmsm_get_time(pmsm_t *motor) {
//     return motor->step_count * motor->dt;  /* 物理时间，单位：秒 */
// }


// #ifdef __cplusplus
// }
// #endif

// #endif // PMSM_H