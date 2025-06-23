/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"

struct lt_driver_ops _driver_bldc_ops;

rt_err_t _driver_bldc_set_pins(lt_driver_t driver)
{	
	rt_pin_mode(driver->enable_pin,PIN_MODE_OUTPUT);
	return RT_EOK;
}

rt_err_t _driver_bldc_enable(lt_driver_t driver,rt_uint8_t dir)
{
	rt_pwm_enable(driver->pwm_A,driver->pwm_channel_A);
	rt_pwm_enable(driver->pwm_B,driver->pwm_channel_B);
	rt_pwm_enable(driver->pwm_C,driver->pwm_channel_C);
	rt_pin_write(driver->enable_pin,PIN_HIGH);
	return RT_EOK;
}

rt_err_t _driver_bldc_disable(lt_driver_t driver)
{
	rt_pwm_disable(driver->pwm_A,driver->pwm_channel_A);
	rt_pwm_disable(driver->pwm_B,driver->pwm_channel_B);
	rt_pwm_disable(driver->pwm_C,driver->pwm_channel_C);
	return RT_EOK;
}

struct lt_driver_ops _driver_bldc_ops = { 	_driver_bldc_set_pins,
											_driver_bldc_enable,
											_driver_bldc_disable
};


