#ifndef LT_FOC_H
/* 本文件是 LTM_CTRL/src/control/foc/lt_foc.h 的同步副本（例程用），改动请同步两边 */
#define LT_FOC_H

#include <stdint.h>
/*------------------------- FOC 算法 ---------------------------------*/

/* 单位约定：
 *  Vd/Vq: 标幺化（1.0 pu = Vdc/√3）并转Q15定点，即 [-1.0, 1.0) <==> [-32768, 32768) 
 *  the  ：电角度，(单圈编码器原始计数 - 电角度零点偏置) × 极对数，可以加上延迟补偿量 
 *  reso ：编码器单圈分辨率（PWM 周期在底层 HAL，本模块不关心）
 *  dutys：输出三相占空比，Q15 标幺（0 ~ 32768 == 0 ~ 100%）
 *  fnum ：三相电流/电压，Q15 标幺；fd/fq ：输出 d/q，Q15 标幺 
 */
void lt_foc_init(void);                                     /* 必须先初始化 */
void lt_foc_set(uint32_t reso);                             /* 初始化后设置分辨率 */
void lt_foc_update(int32_t Vd, int32_t Vq, uint32_t the, int32_t dutys[3]);/* Q15 定点 FOC，输出三相占空比（Q15）*/
void lt_foc_clark_park(int32_t fnum[3], uint32_t the, int32_t *fd, int32_t *fq); 

/*------------------------- FOC 算法 ---------------------------------*/

#endif
