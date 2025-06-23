/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

extern struct lt_sensor_ops _sensor_encoder_ops;
extern struct lt_sensor_ops _sensor_magnetic_ops;

lt_sensor_t lt_sensor_create(char*sensor_name, rt_uint16_t resolution, rt_uint8_t type)
{
	lt_sensor_t _sensor;
	
	switch(type)
	{
		case SENSOR_TYPE_ENCODER:
			_sensor = _sensor_encoder_ops.create(sensor_name,resolution,type);
			break;
		case SENSOR_TYPE_MAGNETIC:
			_sensor = _sensor_magnetic_ops.create(sensor_name,resolution,type);
			break;
		default: break;
	}
	return _sensor;
}

float lt_sensor_get_angle(lt_sensor_t sensor)
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->get_angle != RT_NULL);
	return sensor->ops->get_angle(sensor);
}

float lt_sensor_get_velocity(lt_sensor_t sensor, rt_uint32_t measure_time_us)
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->get_velocity != RT_NULL);
	return sensor->ops->get_velocity(sensor,measure_time_us);
}

rt_err_t lt_sensor_calibrate(lt_sensor_t sensor)
{
	RT_ASSERT(sensor != RT_NULL);
	RT_ASSERT(sensor->ops != RT_NULL);
	RT_ASSERT(sensor->ops->calibrate != RT_NULL);
	return sensor->ops->calibrate(sensor);
}

rt_err_t lt_sensor_delete(lt_sensor_t sensor)
{
	RT_ASSERT(sensor != RT_NULL);
	if(sensor->dev != RT_NULL)
	{
		rt_device_close(sensor->dev);
	}
	if(sensor->lpf != RT_NULL)
	{
		lt_filter_delete(sensor->lpf);
		sensor->lpf = RT_NULL;
	}
	rt_free(sensor);
	
	return RT_EOK;
}
