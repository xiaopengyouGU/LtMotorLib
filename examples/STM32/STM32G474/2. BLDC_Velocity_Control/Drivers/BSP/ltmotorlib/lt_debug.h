/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-30	  Lvtou 	   The first version
 */
#ifndef __LT_DEBUG_H__
#define __LT_DEBUG_H__
#include "lt_def.h"

#ifdef LT_DEBUG_ENABLE
    #include <assert.h>
    #include <stdio.h>
    
    /* basic check macro */
    #define LT_DEBUG(condition, fmt, ...) \
        do { \
            if (!(condition)) { \
                printf("[DEBUG] %s:%d - " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
                assert(condition); \
            } \
        } while(0)
    
	/* check nullptr */
	#define LT_CHECK_NULL(ptr) LT_DEBUG((ptr) != NULL, #ptr " is NULL")
	
	/* check magnitude  */
	#define LT_CHECK_MAG(mag) LT_DEBUG((mag) != 0, " Mag is 0 ")
		
   
    #define LT_CHECK_DEVICE(dev) \
        LT_DEBUG((dev) != NULL && ((dev)->flag & DEVICE_FLAG_CHECKED), \
                "Device '%s' is not checked or NULL", (dev) ? (dev)->name : "NULL")
    
    
    #define LT_CHECK_CURRENT(current) \
        LT_DEBUG((current) != NULL && ((current)->flag & DEVICE_FLAG_CHECKED), \
                "Current sensor '%s' is not checked", (current) ? (current)->name : "NULL")
    
    #define LT_CHECK_SENSOR(sensor) \
        LT_DEBUG((sensor) != NULL && ((sensor)->flag & DEVICE_FLAG_CHECKED), \
                "Position sensor '%s' is not checked", (sensor) ? (sensor)->name : "NULL")
    
    #define LT_CHECK_DRIVER(driver) \
        LT_DEBUG((driver) != NULL && ((driver)->flag & DEVICE_FLAG_CHECKED), \
                "Driver '%s' is not checked", (driver) ? (driver)->name : "NULL")
		
	#define LT_CHECK_MOTOR(motor) \
        LT_DEBUG((motor) != NULL && ((motor)->flag & MOTOR_FLAG_CHECKED), \
                "Motor '%s' is not checked", (motor) ? (motor)->name : "NULL")

#else
	/* release mode : zero macro */
    #define LT_DEBUG(condition, fmt, ...) 	((void)0)
	#define LT_CHECK_NULL(ptr)				((void)0)
	#define LT_CHECK_MAG(mag)				((void)0)
    #define LT_CHECK_DEVICE(dev) 			((void)0)
    #define LT_CHECK_CURRENT(current) 		((void)0)
    #define LT_CHECK_SENSOR(sensor) 		((void)0)
    #define LT_CHECK_DRIVER(driver) 		((void)0)
	#define LT_CHECK_MOTOR(motor)			((void)0)
#endif


#endif
