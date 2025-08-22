/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "string.h"
#define _TF				0.016	/* unit: s */
#define MAGNET_ADDR		0x36
#define HIGH_BYTE		0x0C
#define LOW_BYTE		0x0D

#define MAGNET_DIR		0		/* 0: value increase cw, 1: value increase ccw */

///* we will use i2c device in interrupt callback function, so we modify rt_i2c_tranfer */
//static rt_ssize_t _i2c_transfer(struct rt_i2c_bus_device *bus,
//                          struct rt_i2c_msg         msgs[],
//                          rt_uint32_t               num);

static float _magnetic_get_curr(lt_sensor_t sensor);
static float _magnetic_get_pos(lt_sensor_t sensor);
static void _magnetic_thread_entry(void* parameter);
static char* _str_add(char*str1, char*str2);
rt_int32_t _get_count(float curr, float last,rt_int32_t count, rt_uint8_t volt);

struct lt_sensor_magnetic_object
{
	struct lt_sensor_object parent;
	char name[LT_NAME_MAX];
	float angle;			/* unit: degree */
	rt_thread_t thread;		/* async read thread */						
	rt_uint8_t flag;							
};
typedef struct lt_sensor_magnetic_object* lt_magnetic_t;
struct lt_sensor_ops _sensor_magnetic_ops;


static lt_sensor_t _sensor_magnetic_create(char*sensor_name,rt_uint16_t resolution, rt_uint8_t type)
{
	//lt_sensor_t _sensor;
	lt_magnetic_t _sensor;
	lt_filter_t _filter;
	rt_device_t _magnetic;
	
	_magnetic = rt_device_find(sensor_name);
	if(_magnetic == RT_NULL) return RT_NULL;
	
	_sensor = rt_malloc(sizeof(struct lt_sensor_magnetic_object));
	_filter = lt_filter_create(_TF,0);										/* create a low pass filter */
	if(_sensor == RT_NULL) return RT_NULL;
	if(_filter == RT_NULL) return RT_NULL;
	
	
	if(_sensor != RT_NULL)
	{
		rt_memset(_sensor,0,sizeof(struct lt_sensor_magnetic_object));
		rt_strcpy(_sensor->name,sensor_name);
		
		_sensor->parent.count = 0;
		_sensor->parent.resolution = resolution;
		_sensor->parent.type = type;
		_sensor->parent.dev = _magnetic;
		_sensor->parent.ops = &_sensor_magnetic_ops;
		_sensor->parent.lpf = _filter;
	}

	return (lt_sensor_t)_sensor;
}

float _sensor_magnetic_get_angle(lt_sensor_t sensor)
{
	lt_magnetic_t magnet = (lt_magnetic_t)sensor;
	if(magnet->flag == SENSOR_FLAG_ASYNC_READ)
	{
		return magnet->angle;
	}
	else
	{
		return _magnetic_get_pos(sensor);
	}
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

static rt_err_t _sensor_magnetic_control(lt_sensor_t sensor, int cmd, void* arg)
{
	lt_magnetic_t magnet = (lt_magnetic_t)sensor;
	switch(cmd)
	{
		case SENSOR_CTRL_ASYNC_READ:
		{
			magnet->flag = SENSOR_FLAG_ASYNC_READ;
			strcat(magnet->name,"ath");
			magnet->thread = rt_thread_create(magnet->name,_magnetic_thread_entry,magnet,1024,15,20);
			if(magnet->thread == RT_NULL) return RT_ERROR;
			rt_thread_startup(magnet->thread);
			break;
		}
		case SENSOR_CTRL_SYNC_READ:
		{
			magnet->flag = SENSOR_FLAG_SYNC_READ;
			if(magnet->thread != RT_NULL)
			{
				rt_thread_delete(magnet->thread);
			}
			break;
		}
		default:break;
		
	}
	return RT_EOK;
}

struct lt_sensor_ops _sensor_magnetic_ops = {	_sensor_magnetic_create,
												_sensor_magnetic_get_angle,
												_sensor_magnetic_get_velocity,
												_sensor_magnetic_calibrate,
												_sensor_magnetic_control,
};



//static float _magnetic_get_curr(lt_sensor_t sensor)
//{
//	struct rt_i2c_bus_device *bus = (struct rt_i2c_bus_device *)sensor->dev->user_data;
//	RT_ASSERT(bus != RT_NULL);
//	rt_uint8_t buf,reg_addr;					/* 12 bits: 0~4095 */
//	rt_uint16_t two_byte;
//	float curr_val;
//	rt_ssize_t ret;
//    struct rt_i2c_msg msg;
//	
//	/* send HIGH_BYTE addr */
//	reg_addr = HIGH_BYTE;
//    msg.addr   = MAGNET_ADDR;
//    msg.flags  = RT_I2C_WR;
//    msg.len    = 1;
//    msg.buf    = &reg_addr;
//    ret = _i2c_transfer(bus, &msg, 1);
//	if(ret != 1) return 0;									/* error input */
//    /* read HIGH_BYTE */
//	msg.flags = RT_I2C_RD;
//	msg.buf = &buf;
//	ret = _i2c_transfer(bus, &msg, 1);
//	if(ret != 1) return 0;									/* error input */
//	two_byte = buf;
//	two_byte = two_byte << 8;
//	
//	/* send LOW_BYTE addr */
//	reg_addr = LOW_BYTE;
//    msg.addr   = MAGNET_ADDR;
//    msg.flags  = RT_I2C_WR;
//    msg.len    = 1;
//    msg.buf    = &reg_addr;
//    ret = _i2c_transfer(bus, &msg, 1);
//	if(ret != 1) return 0;									/* error input */
//    /* read LOW_BYTE */
//	msg.flags = RT_I2C_RD;
//	msg.buf = &buf;
//	ret = _i2c_transfer(bus, &msg, 1);
//	if(ret != 1) return 0;									/* error input */
//	two_byte |= buf ;									

//	curr_val  =  two_byte * 2.0f * PI/4096.0f;			    /* transform to rad */
//	
//	return curr_val;
//}

static float _magnetic_get_curr(lt_sensor_t sensor){
	rt_device_t i2c = (rt_device_t)sensor->dev;
	struct rt_i2c_bus_device *i2c_bus = (struct rt_i2c_bus_device *)sensor->dev->user_data;
    rt_uint8_t addr = MAGNET_ADDR;     
	rt_uint8_t mem_addr = HIGH_BYTE;
    rt_uint8_t buf[2] = {0};          	
    struct rt_i2c_msg msgs[2];          
    
    /* 1. start writing */
    msgs[0].addr  = MAGNET_ADDR;          	/* slave device address */
    msgs[0].flags = RT_I2C_WR;             
    msgs[0].buf   = &mem_addr;             	/* memory address */
    msgs[0].len   = 1;                     	/* memory length£¨1 byte£©*/

    /* 2. start reading two byte */
    msgs[1].addr  = MAGNET_ADDR;           	/* slave device address */
    msgs[1].flags = RT_I2C_RD;             
    msgs[1].buf   = buf;              		/* data buf */
    msgs[1].len   = sizeof(buf);      		/* data length (2 byte) */

	/* transger i2c data */
	rt_i2c_transfer(i2c_bus, msgs, 2);
	
	/* statrt process read data */
	rt_uint16_t two_byte = (buf[0] << 8) | buf[1];
	float curr_val = two_byte * 2.0f * PI/4096.0f;			    /* transform to rad */
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

rt_int32_t _get_count(float curr, float last,rt_int32_t count, rt_uint8_t volt)
{
	float dist = curr - last;
	if (dist == 0) return count;
	
	if(volt)			/* value increase counter-clockwise */
	{
		if(dist < - PI)
		{
			count = count - 1;		/* finish a rotation */
		}
		else if(dist >= PI)	/* dist >= PI */
		{
			count = count + 1;
		}
	}
	else				/* value increase clockwise */
	{
		if (dist < - PI)
		{
			count = count + 1;
		}
		else if(dist >= PI)	/* dist >= PI */
		{
			count = count - 1;
		}
	}
	return count;
}

/******************************************************************************************************************/
static void _magnetic_thread_entry(void* parameter)
{
	lt_magnetic_t magnet = (lt_magnetic_t)parameter;
	
	while(1)
	{
		magnet->angle = _magnetic_get_pos(&(magnet->parent));
		rt_thread_mdelay(1);									/* delay 1 tick */
	}
}