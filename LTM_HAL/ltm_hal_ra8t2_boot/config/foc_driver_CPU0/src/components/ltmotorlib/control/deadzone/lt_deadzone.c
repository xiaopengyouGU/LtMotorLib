#include "control/deadzone/lt_deadzone.h"
#include <string.h>
typedef struct{
    float dutys_comp[3];                                /* 死区补偿占空比 */
    int8_t signs[3];                                    /* 电流极性符号 +1，-1，0 */                              
    float dead_duty;                                    /* 死区对应占空比 */
    float Ith;                                          /* Ith: 电流极性判别阈值（A）*/
    float alpha;                                        /* 死区滤波系数 */
    uint8_t flag;                                       /* 初始化标志，1：已初始化，0：未初始化 */
}lt_deadzone_obj;                                       /* 死区补偿结构体 */

typedef lt_deadzone_obj * lt_deadzone_t;
static lt_deadzone_obj deadzone_obj;
static lt_deadzone_t    ds = &deadzone_obj;

#define DEAD_DUTY       0.012f  /* 默认的死区占空比：1.2% */
#define FILTER_ALPHA    0.35f   /* 死区滤波系数 */
#define DEFAULT_ITH     0.12f   /* 默认的阈值电流 A */
/* Ith: 电流极性判别阈值，一般为额定电流的 1%~3% A ,决定值 */
static void _deadzone_set(float dead_duty, float Ith, float alpha);

void lt_deadzone_init(float dead_duty, float Ith, float alpha)     /* 死区补偿初始化 */
{
    _deadzone_set(dead_duty, Ith, alpha);
}      

void lt_deadzone_set(float dead_duty, float Ith, float alpha)       /* 设置死区对应占空比（[0-1]），和电流阈值，已经滤波系数 */
{
    _deadzone_set(dead_duty, Ith, alpha);
}

void lt_deadzone_compensate(float Ia, float Ib, float Ic)
{
    if(!ds->flag)           return;                             /* 未初始化, 直接返回 */

    float curr_thre   = ds->Ith;                                /* 获取极性判断阈值 */
    float currents[3] = {Ia, Ib, Ic};
    int8_t signs[3] = {0, 0, 0};                                /* 电流极性数组 */

    for (int phase = 0; phase < 3; phase++) {
        float i = currents[phase];

        /* 1. 电流方向判断 (滞环 + 方向保持) */
        if(i > curr_thre){
            signs[phase] = 1;
        } else if (i < -curr_thre) {
            signs[phase] = -1;
        } else {
            signs[phase] = ds->signs[phase];     /* 采用上一次的结果 */
        }

        /* 2. 补偿占空比 = 方向 × 死区占空比 (0-0.05) */
        float duty_comp  = signs[phase] * ds->dead_duty;

        /* 3. 一阶低通滤波，减轻死区补偿引起的波动效果 */
        float alpha = ds->alpha;
        float comp_filtered = alpha * duty_comp + (1.0f - alpha) * ds->dutys_comp[phase];

        /* 4. 更新状态 */
        ds->signs[phase]       = signs[phase];
        ds->dutys_comp[phase]  = comp_filtered;
    }
}

/* 输入值为 FOC 计算后得到的占空比 */
void lt_deadzone_get(float *dutyA, float *dutyB, float *dutyC)  /* 得到补偿后的PWM输出占空比 */
{
    if(!ds->flag)          return;                              /* 未初始化，直接返回 */
    *dutyA += ds->dutys_comp[0];
    *dutyB += ds->dutys_comp[1];
    *dutyC += ds->dutys_comp[2];
}

/*****************************************************************************/
static void _deadzone_set(float dead_duty, float Ith, float alpha)
{
    memset(ds, 0, sizeof(lt_deadzone_obj));
    if(dead_duty <= 0 || dead_duty > 0.05f)  dead_duty = DEAD_DUTY;
    if(alpha <= 0 || alpha > 0.6f)           alpha     = FILTER_ALPHA;
    if(Ith < 0)                              Ith       = -Ith;
    if(Ith == 0.0f)                          Ith       = DEFAULT_ITH;
    /* 记录设定值 */

    ds->dead_duty = dead_duty;
    ds->Ith   = Ith;
    ds->alpha = alpha;
    ds->flag  = 1;                          /* 标记初始化完毕 */
}
