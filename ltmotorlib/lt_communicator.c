/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

static struct lt_communicator_object _commun;
lt_commun_t _communicator = &_commun;

#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
static rt_err_t _rx_ind(rt_device_t dev, rt_size_t size)
{
	if(size > 0)
	rt_sem_release(&_communicator->rx_sem);
	return RT_EOK;
}

static void serial_thread_entry(void *parameter)
{
    rt_uint8_t ch;
	rt_sem_t rx_sem = &_communicator->rx_sem;
	rt_device_t serial = _communicator->dev;
    while (1)
    {
       rt_sem_take(rx_sem, RT_WAITING_FOREVER);
        //read serial data
        while (rt_device_read(serial, -1, &ch, 1) == 1) {
			_communicator->ops->data_recv(_communicator,&ch,1);	/* process per byte */
        }
    }
}
#endif

void lt_communicator_send(int cmd,rt_uint8_t channel,void*data,rt_uint8_t num)
{
	RT_ASSERT(_communicator != RT_NULL);
	RT_ASSERT(_communicator->ops != RT_NULL);
	RT_ASSERT(_communicator->ops->send != RT_NULL);
	_communicator->ops->send(_communicator,cmd,channel,data,num);
}

void lt_communicator_set(char* dev,rt_size_t buf_size,struct lt_commun_ops* ops)
{
	RT_ASSERT(_communicator != RT_NULL);
#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
	rt_device_t _serial = rt_device_find(dev);
	rt_thread_t _thread;
	rt_err_t res;
	
	if(_serial == RT_NULL) 	return;
	if(ops == RT_NULL)		return;
	
	_communicator->dev = _serial;
	_communicator->ops = ops;
	rt_device_open(_serial,RT_DEVICE_FLAG_INT_RX);			/* open serial */
	//rt_device_open(_serial,RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX)
	_communicator->buffer = rt_malloc(buf_size);
	if(_communicator->buffer == RT_NULL) return;
	rt_memset(_communicator->buffer,0,buf_size);
	_communicator->buf_size = buf_size;
	
	/* init semaphore */
	res = rt_sem_init(&_communicator->rx_sem, "_rx_sem", 0, RT_IPC_FLAG_FIFO);
	if(res != RT_EOK) return;
	/* set receive callback function */
	res = rt_device_set_rx_indicate(_serial,_rx_ind);
	/* create thread to reiceive serial data */
	_thread = rt_thread_create("_serial_",serial_thread_entry,RT_NULL,256,10,20);
	if(_thread != RT_NULL)
	{
		_communicator->thread = _thread;
		rt_thread_startup(_thread);
	}
#else
	rt_device_t _dac = rt_device_find(dev);
	if(_dac == RT_NULL) return;
	if(ops == RT_NULL) return;
	rt_dac_device_t dac_dev = (rt_dac_device_t)_dac;	/* enable dac device at first */
	rt_dac_enable(dac_dev,1);
	rt_dac_enable(dac_dev,2);
	rt_dac_write(dac_dev,1,2048);
	rt_dac_write(dac_dev,2,2048);
	rt_kprintf("communicator for oscilloscope is configured successfully! \n");
	
	_communicator->dev = _dac;
	_communicator->ops = ops;
#endif
}

rt_uint8_t lt_communicator_receive(void*info)
{
	RT_ASSERT(_communicator != RT_NULL);
	RT_ASSERT(_communicator->ops != RT_NULL);
	RT_ASSERT(_communicator->ops->process != RT_NULL);
	return _communicator->ops->process(_communicator,info);
}

rt_err_t lt_communicator_delete(void)
{
	RT_ASSERT(_communicator != RT_NULL);
	
	if(_communicator->buffer != RT_NULL)
	{
		rt_free(_communicator->buffer);
		_communicator->buffer = RT_NULL;
	}
	if(_communicator->thread != RT_NULL)
	{
		rt_thread_delete(_communicator->thread);
		_communicator->thread = RT_NULL;
	}
	if(_communicator->dev != RT_NULL)
	{
		rt_device_close(_communicator->dev);
		_communicator->dev = RT_NULL;
	}
	_communicator = RT_NULL;
	
	return RT_EOK;
}

