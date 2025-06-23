/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

#define _TF				0.016	/* unit: s */
#define MAGNET_ADDR		0x36
#define HIGH_BYTE		0x0C
#define LOW_BYTE		0x0D

#define MAGNET_DIR		0		/* 0: value increase cw, 1: value increase ccw */

/* we will use i2c device in interrupt callback function, so we modify rt_i2c_tranfer */
static rt_ssize_t _i2c_transfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg         msgs[],
                          rt_uint32_t               num);
static float _magnetic_get_curr(lt_sensor_t sensor);
static float _magnetic_get_pos(lt_sensor_t sensor);
							 
struct lt_sensor_ops _sensor_magnetic_ops;
static lt_sensor_t _sensor_magnetic_create(char*sensor_name,rt_uint16_t resolution, rt_uint8_t type)
{
	lt_sensor_t _sensor;
	lt_filter_t _filter;
	rt_device_t _magnetic;
	
	_magnetic = rt_device_find(sensor_name);
	if(_magnetic == RT_NULL) return RT_NULL;
	
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
		_sensor->dev = _magnetic;
		_sensor->ops = &_sensor_magnetic_ops;
		_sensor->lpf = _filter;
	}

	return _sensor;
}

float _sensor_magnetic_get_angle(lt_sensor_t sensor)
{
	return _magnetic_get_pos(sensor);
}

float _sensor_magnetic_get_velocity(lt_sensor_t sensor,rt_uint32_t measure_time_us)
{
	if(measure_time_us == 0) return 0;					/* unvalid measure time */
	float pos_last = sensor->count*2*PI + sensor->last;
	float pos_curr = _magnetic_get_pos(sensor);
	float velocity = (pos_curr - pos_last)*1000000.0f/measure_time_us;		/* unit: rad/s*/
	/* low pass filter process */
	lt_filter_set_dt(sensor->lpf,measure_time_us/1000000.0f);				/* uint: s */
	velocity = lt_filter_process(sensor->lpf,velocity);
	
	return velocity;
}

rt_err_t _sensor_magnetic_calibrate(lt_sensor_t sensor)
{
	sensor->count = 0;
	sensor->last = _magnetic_get_curr(sensor);	
	return RT_EOK;
}


struct lt_sensor_ops _sensor_magnetic_ops = {	_sensor_magnetic_create,
												_sensor_magnetic_get_angle,
												_sensor_magnetic_get_velocity,
												_sensor_magnetic_calibrate,
};

static float _magnetic_get_curr(lt_sensor_t sensor)
{
	struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)sensor->dev->user_data;
	RT_ASSERT(bus != RT_NULL);
	rt_uint8_t buf,reg_addr;					/* 12 bits: 0~4095 */
	rt_uint16_t two_byte;
	float curr_val;
	rt_ssize_t ret;
    struct rt_i2c_msg msg;
	
	/* send HIGH_BYTE addr */
	reg_addr = HIGH_BYTE;
    msg.addr   = MAGNET_ADDR;
    msg.flags  = RT_I2C_WR;
    msg.len    = 1;
    msg.buf    = &reg_addr;
    ret = _i2c_transfer(bus, &msg, 1);
	if(ret != 1) return 0;									/* error input */
    /* read HIGH_BYTE */
	msg.flags = RT_I2C_RD;
	msg.buf = &buf;
	ret = _i2c_transfer(bus, &msg, 1);
	if(ret != 1) return 0;									/* error input */
	two_byte = buf;
	two_byte = two_byte << 8;
	
	/* send LOW_BYTE addr */
	reg_addr = LOW_BYTE;
    msg.addr   = MAGNET_ADDR;
    msg.flags  = RT_I2C_WR;
    msg.len    = 1;
    msg.buf    = &reg_addr;
    ret = _i2c_transfer(bus, &msg, 1);
	if(ret != 1) return 0;									/* error input */
    /* read LOW_BYTE */
	msg.flags = RT_I2C_RD;
	msg.buf = &buf;
	ret = _i2c_transfer(bus, &msg, 1);
	if(ret != 1) return 0;									/* error input */
	two_byte |= buf ;									

	curr_val  =  two_byte * 2.0f * PI/4096.0f;			    /* transform to rad */
	
	return curr_val;
}

static float _magnetic_get_pos(lt_sensor_t sensor)
{
	float curr = _magnetic_get_curr(sensor),position;
	int count =  _get_count(curr,sensor->last,sensor->count,MAGNET_DIR);
	sensor->count = count;
	position = count*2.0f*PI + curr;
	sensor->last = curr;
	return position;
	
}

/******************************************************************************************************************/
/* we will use i2c device in interrupt callback function, so we modify rt_i2c_tranfer  */

static rt_ssize_t _i2c_transfer(struct rt_i2c_bus_device *bus,
                          struct rt_i2c_msg         msgs[],
                          rt_uint32_t               num)
{
    rt_ssize_t ret;

    if (bus->ops->master_xfer)
    {
        ret = bus->ops->master_xfer(bus, msgs, num);
		return ret;
    }
    else
    {
        return -RT_EINVAL;
    }
}
/******************************************************************************************************************/
