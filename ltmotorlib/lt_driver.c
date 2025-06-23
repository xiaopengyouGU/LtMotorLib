/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

extern struct lt_driver_ops _driver_dc_ops;
extern struct lt_driver_ops _driver_bldc_ops;
extern struct lt_driver_ops _driver_stepper_ops;

lt_driver_t lt_driver_create(rt_uint8_t type)
{
	lt_driver_t _driver;
	_driver = rt_malloc(sizeof(struct lt_driver_object));
	rt_memset(_driver,0,sizeof(struct lt_driver_object));
	_driver->type = type;
	
	switch(type)
	{
		case DRIVER_TYPE_DC:
			_driver->ops = &_driver_dc_ops;
			break;
		case DRIVER_TYPE_STEPPER:
			_driver->ops = &_driver_stepper_ops;
			break;
		case DRIVER_TYPE_BLDC:
			_driver->ops = &_driver_bldc_ops;
			break;
		default: break;
	}
	return _driver;
}

rt_err_t lt_driver_set_pins(lt_driver_t driver,rt_base_t forward_pin,rt_base_t reversal_pin,rt_base_t enable_pin)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops!= RT_NULL);
	RT_ASSERT(driver->ops->set_pins != RT_NULL);
	
	driver->forward_pin = forward_pin;
	driver->reversal_pin = reversal_pin;
	driver->enable_pin = enable_pin;
	
	return driver->ops->set_pins(driver);
}

rt_err_t lt_driver_set_pwm(lt_driver_t driver,char* pwm_name,rt_uint8_t pwm_channel,rt_uint8_t phase)
{
	RT_ASSERT(driver != RT_NULL);
	struct rt_device_pwm* pwm;
	
	pwm = (struct rt_device_pwm *)rt_device_find(pwm_name);
	if(pwm != RT_NULL)
	{
		rt_pwm_disable(pwm,pwm_channel);
	}
	else return RT_ERROR;
	
	switch(phase)
	{
		case PWM_PHASE_DEFAULT:
			driver->pwm_A = pwm;
			driver->pwm_channel_A = pwm_channel;
			break;
		case PWM_PHASE_A:
			driver->pwm_A = pwm;
			driver->pwm_channel_A = pwm_channel;
			break;
		case PWM_PHASE_B:
			driver->pwm_B = pwm;
			driver->pwm_channel_B = pwm_channel;
			break;
		case PWM_PHASE_C:
			driver->pwm_C = pwm;
			driver->pwm_channel_C = pwm_channel;
			break;
		default : return RT_ERROR;
	}
	return RT_EOK;
}

rt_err_t lt_driver_set_output(lt_driver_t driver,rt_uint32_t period,float duty_cycle)
{
	RT_ASSERT(driver != RT_NULL);
	struct rt_device_pwm *pwm = driver->pwm_A;
	rt_uint8_t channel = driver->pwm_channel_A;			
	duty_cycle = _constrains(duty_cycle,0,1);
	period = period*1000;		/* unit: us-> ns */
	rt_uint32_t pulse = (rt_uint32_t)(period*duty_cycle);
	
	//rt_pwm_disable(pwm,channel);							/* disable pwm first */
	rt_pwm_set(pwm,channel,period,pulse);					/* set pwm output */
	
	return RT_EOK;
}

rt_err_t lt_driver_3pwm_output(lt_driver_t driver,rt_uint32_t period,float dutyA,float dutyB, float dutyC)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops!= RT_NULL);
	RT_ASSERT(driver->ops->enable != RT_NULL);
	if(driver->pwm_A == RT_NULL || driver->pwm_channel_A == RT_NULL)	return RT_ERROR;
	if(driver->pwm_B == RT_NULL || driver->pwm_channel_B == RT_NULL)	return RT_ERROR;
	if(driver->pwm_C== RT_NULL  || driver->pwm_channel_C == RT_NULL)	return RT_ERROR;
	dutyA = _constrains(dutyA,0,1);
	dutyB = _constrains(dutyB,0,1);
	dutyC = _constrains(dutyC,0,1);
	
	period = period*1000;		/* unit: us-> ns */
	//driver->ops->disable(driver);			/* disable output first */			
	/* set pwm output */
	rt_pwm_set(driver->pwm_A,driver->pwm_channel_A,period,(rt_uint32_t)(dutyA*period));
	rt_pwm_set(driver->pwm_B,driver->pwm_channel_B,period,(rt_uint32_t)(dutyB*period));
	rt_pwm_set(driver->pwm_C,driver->pwm_channel_C,period,(rt_uint32_t)(dutyC*period));
	
	return driver->ops->enable(driver,0);
}

rt_err_t lt_driver_enable(lt_driver_t driver,rt_uint8_t dir)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops!= RT_NULL);
	RT_ASSERT(driver->ops->enable != RT_NULL);
	return driver->ops->enable(driver,dir);
}

rt_err_t lt_driver_disable(lt_driver_t driver)
{
	RT_ASSERT(driver != RT_NULL);
	RT_ASSERT(driver->ops != RT_NULL);
	RT_ASSERT(driver->ops->disable != RT_NULL);
	return driver->ops->disable(driver);
}

rt_err_t lt_driver_delete(lt_driver_t driver)
{
	RT_ASSERT(driver != RT_NULL);
	if(driver->pwm_A != RT_NULL)
	{
		rt_pwm_disable(driver->pwm_A,driver->pwm_channel_A);
		driver->pwm_A = RT_NULL;
	}
	if(driver->pwm_B != RT_NULL)
	{
		rt_pwm_disable(driver->pwm_B,driver->pwm_channel_B);
		driver->pwm_B = RT_NULL;
	}
	if(driver->pwm_C != RT_NULL)
	{
		rt_pwm_disable(driver->pwm_C,driver->pwm_channel_C);
		driver->pwm_C = RT_NULL;
	}

	rt_free(driver);
	return RT_EOK;
}

