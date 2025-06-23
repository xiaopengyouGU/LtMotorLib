/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

struct lt_driver_ops _driver_stepper_ops;

rt_err_t _driver_stepper_set_pins(lt_driver_t driver)
{
	rt_pin_mode(driver->forward_pin,PIN_MODE_OUTPUT);
	rt_pin_mode(driver->enable_pin,PIN_MODE_OUTPUT);
	return RT_EOK;
}

rt_err_t _driver_stepper_enable(lt_driver_t driver,rt_uint8_t dir)
{
	if(dir == ROT_FORWARD)
	{
		rt_pin_write(driver->forward_pin,PIN_HIGH);
	}
	else if(dir == ROT_REVERSAL)
	{
		rt_pin_write(driver->forward_pin,PIN_LOW);
	}
	
	rt_pin_write(driver->enable_pin,PIN_LOW);
	rt_pwm_enable(driver->pwm_A,driver->pwm_channel_A);
	return RT_EOK;
}

rt_err_t _driver_stepper_disable(lt_driver_t driver)
{
	rt_pin_write(driver->enable_pin,PIN_HIGH);
	rt_pwm_disable(driver->pwm_A,driver->pwm_channel_A);
	return RT_EOK;
}

struct lt_driver_ops _driver_stepper_ops = { 	_driver_stepper_set_pins,
												_driver_stepper_enable,
												_driver_stepper_disable
};
