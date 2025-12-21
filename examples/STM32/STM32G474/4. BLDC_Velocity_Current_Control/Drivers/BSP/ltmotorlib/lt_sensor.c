/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-23      Lvtou       The first version
 * 2025-12-09	   Lvtou	   Add hall velocity measure function and modify APIs
 */
#include "ltmotorlib.h"
#define SENSOR_FILTER_PARAM 		0.1f		


static struct lt_sensor_ops _soft_ops;

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
	/* when using software position sensor, user doesn't need to config ops */
	if (config->type != SENSOR_TYPE_SOFTWARE && config->type != SENSOR_TYPE_SOFTWARE2)
	{
		LT_CHECK_NULL(config->ops);
		LT_CHECK_NULL(config->ops->calibrate);
		LT_CHECK_NULL(config->ops->get);
		LT_CHECK_NULL(config->ops->get2);
		LT_CHECK_NULL(config->ops->start);
		LT_CHECK_NULL(config->ops->stop);
	}
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
	else if(config->type == SENSOR_TYPE_ENCODER)
	{
		sensor->val_pos_gain = _2_PI / config->revolutions;
	}
	
	if(config->ts == 0) config->ts = 1;		/* default sample period : 1ms */
	sensor->val_vel_gain = 	_RAD_2_RPM / (config->ts/1000.0f);		/* unit transform : ms to s */
	sensor->flag |= DEVICE_FLAG_INIT;
	sensor->ops = config->ops;				/* check ops successfully */
	sensor->flag |= DEVICE_FLAG_CHECKED;
	
	/* support 2 software position sensors */
	if(config->type == SENSOR_TYPE_SOFTWARE || config->type == SENSOR_TYPE_SOFTWARE2)
	{
		sensor->ops = &_soft_ops;
	}
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

/*******************************************************************************/
/* software position sensor's implementation */
static uint8_t _hall_tab_for[6] = {5,1,3,2,6,4};	/* forward rotation hall signal */
static uint8_t _hall_tab_rev[6] = {5,4,6,2,3,1};	/* reversal rotation hall signal */

typedef struct
{
	uint16_t cnt_target;
	uint16_t count;
	uint8_t sector;						/* sector 1 : A+B- */
	uint8_t flag;						/* software sensor status flag */
	uint8_t hall_signal;
	uint8_t dir;
	float vel;
	float pos;
}soft_sensor;
typedef soft_sensor* soft_sensor_t;

static soft_sensor _soft_object;
static soft_sensor _soft2_object;
static soft_sensor_t _soft = &_soft_object;
static soft_sensor_t _soft2 = &_soft2_object;

static void _soft_sensor_init(soft_sensor_t soft)
{
	soft->cnt_target = 0;
	soft->count = 0;
	soft->flag = 0;
	soft->sector = 0;
	soft->hall_signal = 0;
	soft->vel = 0;
	soft->pos = 0;
	soft->dir = DIR_CCW;
}

static void _soft_sensor_process2(soft_sensor_t soft)
{
	switch(soft->flag)
	{
		case 0:		/* idle stage */ 
		{
			soft->flag = 1;
			soft->hall_signal = _hall_tab_for[0];
			soft->sector = 0;
			soft->cnt_target = 1000;
			soft->count = 0;
			soft->vel = 0;
			break;
		}
		case 1:		/* delay stage */
		{
			if(soft->count == soft->cnt_target)		soft->flag = 2;
			else soft->count++;
			break;
		}
		case 2:		/* change sector */
		{
			uint16_t cnt_target = soft->cnt_target;
			uint8_t sector = soft->sector;
			
			sector = (sector + 1) % 6;
			cnt_target -= cnt_target/12 + 1;
			if(cnt_target < 200) cnt_target = 200;						
			/* get pos and vel */
			if(soft->dir == DIR_CCW)
			{
				soft->hall_signal = _hall_tab_for[sector];
			}
			else
			{
				soft->hall_signal = _hall_tab_rev[sector];
			}
			soft->vel =  100000.0f/cnt_target;
			soft->cnt_target = cnt_target;
			soft->count = 0;
			soft->sector = sector;
			soft->flag = 1;
		}
	}
}

static void _soft_start(lt_sensor_t sensor)
{
	if(sensor->flag & DEVICE_FLAG_CALIBING)	/* this hall sensor is firstly started, we need to init it */
	{
		sensor->flag = CLEAR_BITS(sensor->flag,DEVICE_FLAG_CALIBING);
	}
}

static void _soft_calibrate(lt_sensor_t sensor)
{
	if(sensor->type == SENSOR_TYPE_SOFTWARE)
	{
		_soft_sensor_init(_soft);
	}
	else if(sensor->type == SENSOR_TYPE_SOFTWARE2)
	{
		_soft_sensor_init(_soft2);
	}
	sensor->flag |= DEVICE_FLAG_CALIB;
}

static void _soft_get(lt_sensor_t sensor, float* pos)
{
	
}

/* in this case, vel is used as */
static void _soft_get2(lt_sensor_t sensor, uint8_t* signal, float* vel)
{	
	if(sensor->type == SENSOR_TYPE_SOFTWARE)
	{
		_soft->dir = sensor->dir;
		_soft_sensor_process2(_soft);
		*vel = _soft->vel;
		*signal = _soft->hall_signal;
		
	}
	else if(sensor->type == SENSOR_TYPE_SOFTWARE2)
	{
		_soft2->dir = sensor->dir;
		_soft_sensor_process2(_soft2);
		*vel = _soft2->vel;
		*signal = _soft2->hall_signal;
	}
}

static void _soft_stop(lt_sensor_t sensor)
{
	
}

static struct lt_sensor_ops _soft_ops = {	
										_soft_start,
										_soft_calibrate,
										_soft_get,
										_soft_get2,
										_soft_stop,
};


