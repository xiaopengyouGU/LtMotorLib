/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-11-23      Lvtou       The first version
 */
#include "ltmotorlib.h"

lt_driver_t lt_driver_create(const char* dev_name)
{
	lt_driver_t _driver = (lt_driver_t)lt_malloc(sizeof(struct lt_driver_object));
	if(_driver == NULL) return NULL;
	memset(_driver,0,sizeof(struct lt_driver_object));
	/* copy device name */
	strcpy(_driver->name, dev_name);
	_driver->flag = DEVICE_FLAG_UNINIT;

	return _driver;
}

void lt_driver_set(lt_driver_t driver, struct lt_driver_config * config)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(driver);
	LT_CHECK_NULL(config);
	LT_CHECK_NULL(config->ops);
	LT_CHECK_NULL(config->ops->output);
	LT_CHECK_NULL(config->ops->output2);
	LT_CHECK_NULL(config->ops->start);
	LT_CHECK_NULL(config->ops->stop);
#endif
	if(config->name != NULL)
	{
		strcpy(driver->name,config->name);		/* set device name */
	}
	
	driver->band = config->band;
	driver->flag |= DEVICE_FLAG_INIT;
	driver->ops = config->ops;			/* check ops successfully */
	driver->flag |= DEVICE_FLAG_CHECKED;
}

void lt_driver_start(lt_driver_t driver)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DRIVER(driver);
#endif
	if(!(driver->flag & DEVICE_FLAG_RUN))	/* don't start driver */
	{
		driver->ops->start(driver);
		driver->flag |= DEVICE_FLAG_RUN;
		driver->flag = CLEAR_BITS(driver->flag,DEVICE_FLAG_STOP);
	}
}

void lt_driver_stop(lt_driver_t driver)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DRIVER(driver);
#endif
	driver->ops->stop(driver);
	driver->flag |= DEVICE_FLAG_STOP;
	driver->flag = CLEAR_BITS(driver->flag,DEVICE_FLAG_RUN);
}

void lt_driver_output2(lt_driver_t driver, uint8_t* pulses, float duty)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DRIVER(driver);
	LT_CHECK_NULL(pulses);
#endif
	duty = CONSTRAINS(duty, 1.0f, 0.0f);  
	driver->ops->output2(driver,pulses,duty);
}

void lt_driver_output(lt_driver_t driver, float dutyA,float dutyB,float dutyC)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DRIVER(driver);
#endif
	driver->ops->output(driver,dutyA, dutyB, dutyC);
}

void lt_driver_delete(lt_driver_t driver)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_DRIVER(driver);
#endif
	driver->ops->stop(driver);
	driver->flag = DEVICE_FLAG_UNINIT;
	lt_free(driver);
}


