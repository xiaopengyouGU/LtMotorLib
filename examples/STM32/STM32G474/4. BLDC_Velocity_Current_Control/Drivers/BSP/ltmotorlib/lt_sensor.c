/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-23      Lvtou       The first version
 * 2025-12-09	   Lvtou	   Add hall velocity measure function and modify APIs
 */
#include "ltmotorlib.h"
#define SENSOR_FILTER_PARAM 		0.1f		

lt_sensor_t lt_sensor_create(const char* dev_name)
{
	lt_sensor_t _sensor = (lt_sensor_t)lt_malloc(sizeof(struct lt_sensor_object));
	if(_sensor == NULL) return NULL;
	memset(_sensor,0,sizeof(struct lt_sensor_object));
	/* copy device name */
	strcpy(_sensor->name, dev_name);
	_sensor->dir = DIR_CCW;
	_sensor->flag = DEVICE_FLAG_UNINIT;

	return _sensor;
}

void lt_sensor_set(lt_sensor_t sensor, struct lt_sensor_config * config)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(sensor);
	LT_CHECK_NULL(config);
	LT_CHECK_NULL(config->ops);
	LT_CHECK_NULL(config->ops->calibrate);
	LT_CHECK_NULL(config->ops->get);
	LT_CHECK_NULL(config->ops->start);
	LT_CHECK_NULL(config->ops->stop);
#endif
	
	if(config->name != NULL)
	{
		strcpy(sensor->name,config->name);		/* set device name */
	}
	
	sensor->type = config->type;
	if(config->type == SENSOR_TYPE_MAGNET)
	{
		sensor->val_pos_gain = _2_PI / (1 << config->bit_num);
	}
	else if(sensor->type == SENSOR_TYPE_ENCODER)
	{
		sensor->val_pos_gain = _2_PI / config->revolutions;
	}
	if(config->ts == 0) config->ts = 1;		/* default sample period : 1ms */
	sensor->val_vel_gain = 	_RAD_2_RPM / (config->ts/1000.0f);		/* unit transform : ms to s */
	sensor->flag |= DEVICE_FLAG_INIT;
	sensor->ops = config->ops;				/* check ops successfully */
	sensor->flag |= DEVICE_FLAG_CHECKED;
}

void lt_sensor_set_dir(lt_sensor_t sensor, uint8_t dir)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	if(dir == DIR_CW || dir == DIR_CW)
			sensor->dir = dir;
	else	sensor->dir = DIR_CCW;
}

void lt_sensor_start(lt_sensor_t sensor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	if(sensor->flag & DEVICE_FLAG_CALIB)	/* calibrate successfully */
	{
		if(!(sensor->flag & DEVICE_FLAG_RUN))	/* not start */
		{
			sensor->ops->start(sensor);		/* start current sense */
			sensor->flag |= DEVICE_FLAG_RUN;
			sensor->flag = CLEAR_BITS(sensor->flag, DEVICE_FLAG_STOP);
		}
	}
}

void lt_sensor_stop(lt_sensor_t sensor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	sensor->ops->stop(sensor);
	sensor->flag |= DEVICE_FLAG_STOP;
	sensor->flag = CLEAR_BITS(sensor->flag,DEVICE_FLAG_RUN);
	sensor->flag = CLEAR_BITS(sensor->flag,DEVICE_FLAG_CALIB);	
	/* reset device datas */
	sensor->pos_async = 0;
	sensor->vel_async = 0;
}

void lt_sensor_get2(lt_sensor_t sensor, uint8_t* hall_signals, float* vel)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	float vel_t;
	sensor->ops->get2(sensor,hall_signals,&vel_t);
	*vel = LOW_PASS_FILTER(vel_t,*vel,SENSOR_FILTER_PARAM);
	/* save pos and vel */
	sensor->pos_async = (float)(*hall_signals);
	sensor->vel_async = *vel;
}

/* pos is only inited once and will be process by sensor device */
void lt_sensor_get(lt_sensor_t sensor, float *pos, float * vel)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	float pos_t, vel_t;
	sensor->ops->get(sensor,&pos_t);
	pos_t = sensor->val_pos_gain * pos_t;							/* unit : rad */
	vel_t =  (pos_t - *pos)	* sensor->val_vel_gain;					/* unit : rpm */
	*vel = LOW_PASS_FILTER(vel_t,*vel,SENSOR_FILTER_PARAM);
	*pos = pos_t;
	/* save pos and vel */
	sensor->pos_async = pos_t;
	sensor->vel_async = *vel;
}

void lt_sensor_get_async(lt_sensor_t sensor,float* pos,float* vel)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	*pos = sensor->pos_async;
	*vel = sensor->vel_async;
}

uint32_t lt_sensor_calibrate(lt_sensor_t sensor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	if(!(sensor->flag & DEVICE_FLAG_CALIB))
	{
		sensor->flag |= DEVICE_FLAG_CALIBING;
		sensor->ops->calibrate(sensor);
		return LT_EBUSY;
	}
		
	return LT_EOK;
}

void lt_sensor_delete(lt_sensor_t sensor)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_SENSOR(sensor);
#endif
	sensor->ops->stop(sensor);
	sensor->flag = DEVICE_FLAG_UNINIT;
	lt_free(sensor);
}


