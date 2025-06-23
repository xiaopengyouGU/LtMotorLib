/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

#define _TF				0.0005	/* unit: s */

struct lt_sensor_ops _sensor_encoder_ops;
static lt_sensor_t _sensor_encoder_create(char* sensor_name,rt_uint16_t resolution, rt_uint8_t type)
{
	lt_sensor_t _sensor;
	lt_filter_t _filter;
	rt_device_t _encoder;
	
	_encoder = rt_device_find(sensor_name);
	if(_encoder != RT_NULL)
	{
		rt_device_open(_encoder,RT_DEVICE_OFLAG_RDONLY);
		rt_device_control(_encoder, PULSE_ENCODER_CMD_CLEAR_COUNT,RT_NULL);	/* clear count */
	}
	else
	{
		return RT_NULL;
	}
	
	_sensor = rt_malloc(sizeof(struct lt_sensor_object));
	_filter = lt_filter_create(_TF,0);										/* create a low pass filter */
	if(_sensor == RT_NULL) return RT_NULL;
	if(_filter == RT_NULL) return RT_NULL;
	
	if(_sensor != RT_NULL)
	{
		rt_memset(_sensor,0,sizeof(struct lt_sensor_object));
		_sensor->count = 0;
		_sensor->resolution = resolution;
		_sensor->type = type;
		_sensor->dev = _encoder;
		_sensor->ops = &_sensor_encoder_ops;
		_sensor->lpf = _filter;
	}
		
	return _sensor;
}

float _sensor_encoder_get_angle(lt_sensor_t sensor)
{
	rt_device_t encoder = sensor->dev;
	rt_int32_t count;
	float position;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_read(encoder,0,&count,1);
	
	position = (float)(-count)/(sensor->resolution);		/* notice direction! */
	position *= 2*PI;										/* unit: rad */
	return position;
}

float _sensor_encoder_get_velocity(lt_sensor_t sensor,rt_uint32_t measure_time_us)
{
	rt_device_t encoder = sensor->dev;
	rt_int32_t curr_count;
	float velocity;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_read(encoder,0,&curr_count,1);
	/* use M method to measure velocity */
	if(measure_time_us == 0)
	{
		sensor->count = curr_count;						/* refrech encoder count! */
		return 0;										/* unvalid measure time */
	}
	velocity = (float)(sensor->count - curr_count)*1000000.0f/measure_time_us;	/* here we already consider the direction */
	velocity = velocity/(sensor->resolution)*2*PI;				/* unit: rad/s */
	/* low pass filter process */
	lt_filter_set_dt(sensor->lpf,measure_time_us/1000000.0f);	/* uint: s */
	velocity = lt_filter_process(sensor->lpf,velocity);
	sensor->count = curr_count;						/* refrech encoder count! */
	
	return velocity;
}

rt_err_t _sensor_encoder_calibrate(lt_sensor_t sensor)
{
	rt_device_t encoder = sensor->dev;
	sensor->count = 0;
	
	rt_device_open(encoder,RT_DEVICE_OFLAG_RDONLY);
	rt_device_control(encoder,PULSE_ENCODER_CMD_CLEAR_COUNT,RT_NULL);	/* clear pulse encoder */
	rt_device_close(encoder);
	
	return RT_EOK;
}

struct lt_sensor_ops _sensor_encoder_ops = {	_sensor_encoder_create,
												_sensor_encoder_get_angle,
												_sensor_encoder_get_velocity,
												_sensor_encoder_calibrate,
};
