/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-19	  Lvtou        The first version
 */
#include "ltmotorlib.h"

/* codes bellow are related with object */
void* lt_malloc(size_t size)
{
	/* when using RTOS, malloc and free in C standard library are replaced by counterparts in RTOS */
	return malloc(size);
}
void lt_free(void *p)
{
	/* when using RTOS, malloc and free in C standard library are replaced by counterparts in RTOS */
	free(p);
}


