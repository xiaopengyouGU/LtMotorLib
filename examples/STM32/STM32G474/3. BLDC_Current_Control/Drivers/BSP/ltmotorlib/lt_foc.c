/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       	Notes
 * 2025-6-21      Lvtou			The first version
 * 2025-8-27	  Lvtou			Use lookup table sin and cos
 * 2025-9-7       Lvtou			Fix 6 commutation method and add modulation modes
 * 2025-11-30     Lvtou			Reimplement foc object and remove six-step commutation
 */
#include "ltmotorlib.h"

lt_foc_t lt_foc_create(float mag,uint8_t mode,uint8_t type)
{
	lt_foc_t foc;
	foc = lt_malloc(sizeof(struct lt_foc_object));
	if(foc == NULL) return NULL;
	memset(foc,0,sizeof(struct lt_foc_object));
	foc->mag = fabsf(mag);
	if(mode > FOC_MODE_ADAPTIVE)	mode = FOC_MODE_DEFAULT;	/* meet unknown mode */
	if(type > FOC_TYPE_SVPWM)		type = FOC_TYPE_DEFAULT;	/* meet unknown type */
	foc->mode = mode;
	foc->type = type;
	
	return foc;
}

void lt_foc_delete(lt_foc_t foc)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(foc); 
#endif
	lt_free(foc);
}

void lt_foc_set(lt_foc_t foc, float mag, uint8_t mode, uint8_t type)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(foc); 
#endif
	foc->mag = fabsf(mag);
	if(mode > FOC_MODE_ADAPTIVE)	mode = FOC_MODE_DEFAULT;	/* meet unknown mode */
	if(type > FOC_TYPE_SVPWM)		type = FOC_TYPE_DEFAULT;	/* meet unknown type */
	foc->mode = mode;
	foc->type = type;
}

void lt_foc_process(lt_foc_t foc,float fd, float fq, float angle_el)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(foc); 
	LT_CHECK_MAG(foc);
#endif
	if(fd == 0 && fq == 0)				/* close all switches */
	{
		foc->dutyA = 0;
		foc->dutyB = 0;
		foc->dutyC = 0;
		return;
	}	
	
	float f_alpha, f_beta;
	float _c,_s;
	float fa,fb,fc;
	float up = 0;
	float mag = foc->mag;			/* magnitude */
	uint8_t sector;
//	if(foc->mod_type == FOC_MOD_TYPE_SPWM || foc->mod_type == FOC_MOD_TYPE_6_TRAPZOID)
//	{
//		fq = _constrains(fq,mag,-mag);	/* avoid output saturation */
//		fd = _constrains(fd,mag,-mag);
//	}
//	else
//	{
//		fq = _constrains(fq,1.1547f*mag,-1.1547f*mag);	/* avoid output saturation */
//		fd = _constrains(fd,1.1547f*mag,-1.1547f*mag);
//	}
//	
//	switch(foc->mod_type)
//	{
//		case FOC_MOD_TYPE_SPWM:
//		case FOC_MOD_TYPE_SPWM_1:
//		case FOC_MOD_TYPE_SPWM_2:
//		case FOC_MOD_TYPE_SVPWM:			   /* we use center modulation */
//		{
//			angle_el = _normalize_angle(angle_el);
//			/* Park inverse transform */
//			_c = lt_cos(angle_el);			/* use lookup table sin and cos */
//			_s = lt_sin(angle_el);
//			f_alpha = _c*fd - _s*fq;
//			f_beta  = _s*fd + _c*fq;
//			/* Clark inverse transform */
//			fa = f_alpha;
//			fb = -0.5f*f_alpha + _SQRT_3_2*f_beta;
//			fc = -0.5f*f_alpha - _SQRT_3_2*f_beta;
//			
//			if(foc->mod_type == FOC_MOD_TYPE_SPWM_1)
//			{
//				up = - _min(fa,_min(fb,fc)) - foc->mag;
//			}
//			else if(foc->mod_type == FOC_MOD_TYPE_SPWM_2)
//			{
//				float u_min = _min(fa,_min(fb,fc));
//				float u_max = _max(fa,_max(fb,fc));
//				up = -(u_max + u_min)/2;
//			}
//			break;
//		}
//		case FOC_MOD_TYPE_6_TRAPZOID:
//		{
//			sector = 6 * (_normalize_angle(angle_el + _PI_6) / _2_PI);
//			fa = fq * _6_table[sector][0];
//			fb = fq * _6_table[sector][1];
//			fc = fq * _6_table[sector][2];
//		}
//		default:break;
//	}
//	/* map to duty cycle */
//	foc->dutyA = 0.5f + 0.5f*(fa + up)/mag;
//	foc->dutyB = 0.5f + 0.5f*(fb + up)/mag;
//	foc->dutyC = 0.5f + 0.5f*(fc + up)/mag;
//	
}

void lt_foc_get(lt_foc_t foc,float*duty_A, float*duty_B, float*duty_C)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(foc); 
#endif
	/* return duty cycle */
	*duty_A = foc->dutyA;
	*duty_B = foc->dutyB;
	*duty_C = foc->dutyC;
}
