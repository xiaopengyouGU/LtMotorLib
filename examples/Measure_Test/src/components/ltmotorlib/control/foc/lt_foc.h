#ifndef LT_FOC_H
#define LT_FOC_H

#include <stdint.h>
/*------------------------- FOC 算法 ---------------------------------*/
/* FOC 对象，隐藏实现细节 */
typedef struct lt_foc_object *lt_foc_t;

/* 调制类型 */
typedef enum {
    FOC_TYPE_SPWM = 0,      /* 标准 SPWM（无零序注入）*/
    FOC_TYPE_SPWM_1,        /* SPWM + 最小值注入 */
    FOC_TYPE_SPWM_2,        /* SPWM + 均值注入 */
    FOC_TYPE_SVPWM,         /* SVPWM（扇区+过调制）*/
	FOC_TYPE_TRAPZ,			/* 六步换相法 （同步整流版）*/
} foc_type_t;

lt_foc_t lt_foc_create(uint8_t mode, uint8_t type);
void lt_foc_delete(lt_foc_t foc);
void lt_foc_set(lt_foc_t foc, uint8_t mode, uint8_t type);
void lt_foc_process(lt_foc_t foc, float Vd, float Vq, float angle_el);		/* Vd 和 Vq 的范围：[-1, 1] */
void lt_foc_process2(lt_foc_t foc, float Vd, float Vq, float angle_el);		/* 其余电压调制方法（不包括SVPWM） */
void lt_foc_process3(lt_foc_t foc, float Vq, uint8_t hallA, uint8_t hallB, uint8_t hallC); /* 六步换向，配Hall传感器（或过零点信号） */
void lt_foc_get_dutys(lt_foc_t foc, float *dutyA, float *dutyB, float *dutyC);
/*------------------------- FOC 算法 ---------------------------------*/

#endif