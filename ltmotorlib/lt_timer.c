/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
int lt_timer_manager_create(void);
lt_timer_t lt_timer_manager_get(char* name);
rt_err_t lt_timer_manager_add(lt_timer_t timer);
rt_err_t lt_timer_manager_delete(lt_timer_t timer);
rt_err_t lt_timer_manager_delete_object(void);

static void _timer_soft_timeout(void*parameter)
{
	lt_timer_t timer = (lt_timer_t)parameter;
	if(timer->soft_timeout)
	{
		timer->soft_timeout(timer);
	}
}

static rt_err_t _timer_hw_timeout(rt_device_t dev, rt_size_t size)
{
	/* get timer from timer manager based on device's name */
	lt_timer_t timer = lt_timer_manager_get(dev->parent.name);
	if(timer->hw_timeout)
	{
		timer->hw_timeout(timer);
	}
	return RT_EOK;
}

static rt_err_t _enable_timer(lt_timer_t timer, rt_uint8_t type);
static rt_err_t _disable_timer(lt_timer_t timer,rt_uint8_t type);
static rt_err_t _set_timer(lt_timer_t timer, rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type);
static rt_err_t _set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type);

lt_timer_t lt_timer_create(char*soft_name,char* hw_name,rt_uint32_t freq)
{
	lt_timer_t _timer;
	rt_device_t hw_timer;
	rt_timer_t soft_timer;
	
	_timer = rt_malloc(sizeof(struct lt_timer_object));
	if(_timer == RT_NULL) return RT_NULL;
	rt_memset(_timer,0,sizeof(struct lt_timer_object));
	/* find hardware timer device */
	hw_timer = (rt_device_t)rt_device_find(hw_name);
	rt_strcpy(_timer->hw_name,hw_name);				/* record timer's name */
	
	if(hw_timer != RT_NULL)
	{	
		if(freq >= 30000)		/* hwtimer's frequency must higher enough, eg: 30kHz */
		{
			rt_device_control(hw_timer,HWTIMER_CTRL_FREQ_SET,&freq);
		}	/* default timer frequency: 1MHz */
		_timer->hw_timer = hw_timer;
	}
	/* create software timer */
	soft_timer = rt_timer_create(soft_name,_timer_soft_timeout,_timer,0,RT_TIMER_FLAG_ONE_SHOT);
	if(soft_timer != RT_NULL)
	{
		_timer->soft_timer = soft_timer;
	}
	
	/* add timer to manager */
	lt_timer_manager_add(_timer);
	return _timer;
}

rt_err_t lt_timer_set(lt_timer_t timer,rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _set_timer(timer,period,mode,type);
}

rt_err_t lt_timer_set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _set_timeout(timer,timeout,type);
}

rt_err_t lt_timer_enable(lt_timer_t timer,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _enable_timer(timer,type);
}

rt_err_t lt_timer_disable(lt_timer_t timer,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	return _disable_timer(timer,type);
}

rt_err_t lt_timer_delete(lt_timer_t timer)
{
	RT_ASSERT(timer != RT_NULL);
	if(timer->hw_timer != RT_NULL)
	{
		rt_device_close(timer->hw_timer);
		timer->hw_timer = RT_NULL;
	}
	if(timer->soft_timer != RT_NULL)
	{
		rt_timer_delete(timer->soft_timer);
		timer->soft_timer = RT_NULL;
	}
	
	/* remove timer from manager */
	lt_timer_manager_delete(timer);
	rt_free(timer);
	
	return RT_EOK;
}

rt_err_t lt_timer_period_call(lt_timer_t timer,rt_uint32_t period, void(*timeout)(lt_timer_t),void* user_data,rt_uint8_t type)
{
	RT_ASSERT(timer != RT_NULL);
	rt_err_t res = RT_EOK;
	timer->user_data = user_data;
	res |= _disable_timer(timer,type);			/* disable timer at first */
	res |= _set_timer(timer,period,TIMER_MODE_PERIODIC,type);
	res |=_set_timeout(timer,timeout,type);
	res |= _enable_timer(timer,type);			/* eable timer finally */
	
	return res;
}


static rt_err_t _enable_timer(lt_timer_t timer, rt_uint8_t type)
{
	if(type == TIMER_TYPE_HW)
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		static rt_hwtimerval_t timeout_s;
		rt_device_t hw_timer = timer->hw_timer;
		timeout_s.usec = timer->hw_period;			/* unit: us */
		/* start timer at first, then output */
		rt_device_close(hw_timer);											/* we close hwtimer at first to avoid multiple open */
		rt_device_open(hw_timer,RT_DEVICE_OFLAG_RDWR);						/* open hwtimer, then set hwtimer mode SINGLE_SHOT */
		rt_device_control(hw_timer,HWTIMER_CTRL_MODE_SET,&timer->hw_mode);	/* set hwtimer mode */
		rt_device_set_rx_indicate(hw_timer,_timer_hw_timeout);				
		rt_device_write(hw_timer,0,&timeout_s,sizeof(timeout_s));
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		rt_timer_start(timer->soft_timer);
	}
	return RT_EOK;
}

static rt_err_t _disable_timer(lt_timer_t timer, rt_uint8_t type)
{
	if(type == TIMER_TYPE_HW)
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		rt_device_close(timer->hw_timer);
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		rt_timer_stop(timer->soft_timer);
	}
	return RT_EOK;
}

static rt_err_t _set_timer(lt_timer_t timer, rt_uint32_t period, rt_uint8_t mode,rt_uint8_t type)
{
	if(type == TIMER_TYPE_HW)		/* set hardware timer */
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		timer->hw_period = period;
		rt_hwtimer_mode_t  hw_mode;		/* hardward timer mode, period/single */
		if(mode == TIMER_MODE_SINGLE_SHOT)
		{
			hw_mode = HWTIMER_MODE_ONESHOT;
		}
		else if(mode == TIMER_MODE_PERIODIC)
		{
			hw_mode = HWTIMER_MODE_PERIOD;
		}
		else
		{
			return RT_EOK;	/* use default mode */
		}
		timer->hw_mode = hw_mode;
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		if(period < 1000)
		{
			period = 0;
		}
		else
		{
			period /= 1000;		/* unit trans: us --> ms */
		}
		rt_timer_control(timer->soft_timer,RT_TIMER_CTRL_SET_TIME,&period);
		if(mode == TIMER_MODE_SINGLE_SHOT)
		{
			rt_timer_control(timer->soft_timer,RT_TIMER_CTRL_SET_ONESHOT,RT_NULL);
		}
		else if(mode == TIMER_MODE_PERIODIC)
		{
			rt_timer_control(timer->soft_timer,RT_TIMER_CTRL_SET_PERIODIC,RT_NULL);
		}
	}
	return RT_EOK;
}

static rt_err_t _set_timeout(lt_timer_t timer,void(*timeout)(lt_timer_t),rt_uint8_t type)
{
	if(timeout == RT_NULL) return RT_ERROR;
	
	if(type == TIMER_TYPE_HW)
	{
		if(timer->hw_timer == RT_NULL) return RT_ERROR;
		timer->hw_timeout = timeout;		
	}
	else
	{
		if(timer->soft_timer == RT_NULL) return RT_ERROR;
		timer->soft_timeout = timeout;
	}
	return RT_EOK;
}

/* finish a small timer manager to help find timer in hw_timer callback functions */
typedef struct _timer_node_object* _timer_node_t;
struct _timer_node_object
{
	_timer_node_t prev;
	_timer_node_t next;
	lt_timer_t timer;
	char name[RT_NAME_MAX];
};

static _timer_node_t _list;

static _timer_node_t _timer_node_get(_timer_node_t list,char* name)
{
	_timer_node_t curr = list->next;
	rt_int32_t res;
	
	while(curr != list)
	{
		res = rt_strcasecmp(curr->name,name);		/* ascend sequence */
		if(res == 0) return curr;
		else if(res < 0)
		{
			curr = curr->next;
		}
		else break;		
	}
	return RT_NULL;
}

static _timer_node_t _timer_node_create(lt_timer_t timer)
{
	_timer_node_t _node = rt_malloc(sizeof(struct _timer_node_object));
	if(_node == RT_NULL) return RT_NULL;
	rt_memset(_node,0,sizeof(struct _timer_node_object));
	if(timer == RT_NULL)
	{
		rt_strcpy(_node->name,"");
	}
	else
	{
		rt_strcpy(_node->name,timer->hw_name);
	}
	_node->timer = timer;
	_node->prev = _node;
	_node->next = _node;
	
	return _node;
}

static void _timer_node_add(_timer_node_t list,_timer_node_t node)
{
	_timer_node_t curr = list->next;
	rt_int32_t res;
	
	while(curr != list)	
	{
		res = rt_strcasecmp(curr->name,node->name);
		if(res == 0) return;
		if(res < 0)
		{
			curr = curr->next;
		}
		else				/* find pos */
		{
			break;
		}
	}
	/* in cycle list case, list->prev = end, this is so beautiful */
	curr->prev->next = node;
	node->prev = curr->prev;
	node->next = curr;
	curr->prev = node;
		
}

static void _timer_node_delete(_timer_node_t list,_timer_node_t node)
{
	/* in cycle list case, list->prev = end, this is so beautiful */
	node->prev->next = node->next;
	node->next->prev = node->prev;
	rt_free(node);								/* release memory */
}

/* when created a timer, it would be added to the manager, when deleting a timer, it would be removed from the manager */
/* there is only one timer manager */

/* implements a cycle list */
int lt_timer_manager_create(void)
{
	_list = _timer_node_create(RT_NULL);
	rt_kprintf("LtMotorLib timer manager is created successfully! \n");
	return RT_EOK;
}

rt_err_t lt_timer_manager_add(lt_timer_t timer)
{
	RT_ASSERT(_list != RT_NULL);
	_timer_node_t _node = _timer_node_create(timer);		/* create a node */
	_timer_node_add(_list,_node);							/* add node to list */
	return RT_EOK;
}

rt_err_t lt_timer_manager_delete(lt_timer_t timer)
{
	RT_ASSERT(_list != RT_NULL);
	_timer_node_t _node = _timer_node_get(_list,timer->hw_name);		/* get node */
	if(_node != RT_NULL)
	{
		_timer_node_delete(_list,_node);
		return RT_EOK;
	}
	else
	{
		return RT_ERROR;
	}
}

lt_timer_t lt_timer_manager_get(char* name)
{
	RT_ASSERT(_list != RT_NULL);
	_timer_node_t node = _timer_node_get(_list,name);
	if(node == RT_NULL) return RT_NULL;
	else return node->timer;
}

rt_err_t lt_timer_manager_delete_object(void)
{
	RT_ASSERT(_list != RT_NULL);
	_timer_node_t tmp = _list;
	_timer_node_t curr = _list->next;
	while(curr != _list)
	{
		tmp = curr->next;				/* save next pointer */
		_timer_node_delete(_list,curr);
		curr = tmp;
	}
	_timer_node_delete(_list,_list);	/* delete head pointer */
	_list = RT_NULL;
	return RT_EOK;
}
INIT_DEVICE_EXPORT(lt_timer_manager_create);			/* create motor manager automatically */
