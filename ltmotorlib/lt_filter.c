/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

lt_filter_t lt_filter_create(float Tf, float dt)
{
	lt_filter_t filter = rt_malloc(sizeof(struct lt_filter_object));
	if(filter == RT_NULL) return RT_NULL;
	
	rt_memset(filter,0,sizeof(struct lt_filter_object));
	filter->Tf = Tf;
	filter->dt = dt;
	return filter;
}

void lt_filter_set_tf(lt_filter_t filter, float Tf)
{
	RT_ASSERT(filter != RT_NULL);
	filter->Tf = Tf;
}

void lt_filter_set_dt(lt_filter_t filter, float dt)
{
	RT_ASSERT(filter != RT_NULL);
	filter->dt = dt;
}

void lt_filter_set(lt_filter_t filter, float Tf, float dt)
{
	RT_ASSERT(filter != RT_NULL);
	filter->Tf = Tf;
	filter->dt = dt;
}

void lt_filter_reset(lt_filter_t filter)
{
	RT_ASSERT(filter != RT_NULL);
	filter->val_prev = 0;
}

float lt_filter_process(lt_filter_t filter, float value)
{	/* low pass filter */
	RT_ASSERT(filter != RT_NULL);
	float Tf = filter->Tf;
	float dt = filter->dt;
	float a = Tf/(Tf+dt);
	value = a*(filter->val_prev) + (1.0f - a)*value;
	
	filter->val_prev = value;				/* preserve filtered result */
	
	return value;
}


rt_err_t lt_filter_delete(lt_filter_t filter)
{
	RT_ASSERT(filter != RT_NULL);
	rt_free(filter);
	return RT_EOK;
}

