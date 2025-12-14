/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        The first version
 * 2025-11-15	  Lvtou 	   The second version
 */
#ifndef __LT_DEF_H__
#define __LT_DEF_H__

/* version information */
#define LT_MOTOR_LIB_VERSION_MAJOR 0
#define LT_MOTOR_LIB_VERSION_MINOR 1
#define LT_MOTOR_LIB_VERSION_PATCH 1
#define LT_MOTOR_LIB_VERSION "0.1.1"

/* debug mode */
#define LT_DEBUG_ENABLE
#define LT_COMMUTE_ENABLE                /* enable communicator */
#define LT_COMMUTE_ENABLE_CRC            /* enable CRC check */

/* macro bit functions */
#define GET_BIT(x,pos)					(1 & ((x) >> (pos)) )
#define CLEAR_BITS(x,bits)				((x) & (~(bits)))
/* macro math functions */
#define LOW_PASS_FILTER(X_k,Y_k_1,a) 	((a)*(X_k) + (1.0f - (a))*(Y_k_1))
#define HIGH_PASS_FILTER(X_k,Y_k_1,a)	((1.0f - (a))*(X_k) + (a)*(Y_k_1))
#define CONSTRAINS(x,up,down)			((x) < (down) ? (down) : ((x) > (up) ? (up) : (x)))

    
/* macro values */
#define _SQRT_3						1.7320508f
#define _SQRT_3_2					0.8660254f
#define _SQRT_3_3					0.5773503f
#define _PI							3.1415926f
#define _2_PI						6.2831853f
#define _PI_2						1.5707963f
#define _PI_3						1.0471976f
#define _PI_6						0.5235988f
#define _2_DIV_3					0.6666667f
#define _RAD_2_DEG					57.295780f	/* 180/PI */
#define _RAD_2_RPM					9.5492966f	/* (30/PI) */
#define PID_INTEGRAL_LIMIT			2000
#define PID_OUTPUT_LIMIT			2000
#define LT_NAME_MAX					8

/* motor status flags */
#define MOTOR_FLAG_RUN			0x08
#define MOTOR_FLAG_CHECKED		0x04
#define MOTOR_FLAG_STOP			0x02
#define MOTOR_FLAG_INIT			0x01
#define MOTOR_FLAG_UNINIT		0x00
#define MOTOR_FLAG_OPEN_LOOP	0x10
#define MOTOR_FLAG_CLOSE_LOOP	0x20
#define MOTOR_FLAG_BREAKED		0x40
#define MOTOR_FLAG_BLOCKED		0x80

/* device flags*/
#define DEVICE_FLAG_CHECKED		0x20
#define DEVICE_FLAG_RUN			0x10
#define DEVICE_FLAG_CALIB		0x08
#define DEVICE_FLAG_CALIBING	0x04
#define DEVICE_FLAG_STOP		0x02
#define DEVICE_FLAG_INIT		0x01
#define DEVICE_FLAG_UNINIT		0x00
/* states flags */
#define LT_EOK							0x00
#define LT_ERROR						0x01
#define LT_EBUSY						0x02

/* close loop flag */
#define LOOP_FLAG_CURRENT				0x01
#define LOOP_FLAG_VELOCITY				0x02
#define LOOP_FLAG_POSITION				0x03

/* direction */
#define DIR_CCW							0x00
#define DIR_CW							0x01

/* many types of error may appear at the same time !!! */
#define  ERROR_TYPE_HALL_ENC    		0x01 	/* hall sensor or encoder error*/   
#define  ERROR_TYPE_OVER_SPEED			0x02	
#define  ERROR_TYPE_OVER_TEMP_MOTOR		0x04	
#define  ERROR_TYPE_OVER_TEMP_DRIVER	0x08
#define  ERROR_TYPE_OVER_VOLT			0x10
#define  ERROR_TYPE_UNDER_VOLT			0x20
#define  ERROR_TYPE_OVER_CURR			0x40
#define  ERROR_TYPE_UNKNOWN				0x80

#endif
