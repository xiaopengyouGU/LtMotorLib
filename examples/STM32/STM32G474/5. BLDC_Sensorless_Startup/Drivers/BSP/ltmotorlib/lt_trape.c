/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       	Notes
 * 2025-11-20     Lvtou			The first version
 * 2025-11-30	  Lvtou			Modify implementaions and add voltage modulation modes
 */
#include "ltmotorlib.h"
/* Six-Step Commutation ,also called Trapezoidal Control */
/* Note : The index of _hall_map is hall signals from sensor object, 
*	whose format : signal : ABC, eg: 101 --> sector 1, 000 or 111 --> error hall signal
*   "1" : high voltage of GPIO, "0" : low voltage of GPIO  
*/
static uint8_t _hall_map[8] = { 0,		/* error hall signal */
								6,		/* sector number 6 */
								4,		/* sector number 4 */
								5,		/* sector number 5 */
								2,		/* sector number 2 */
								1,		/* sector number 1 */
								3,		/* sector number 3 */
								7		/* error hall signal */
};

/* Note : The first index of _pulses_tab is sector number,
	the second index corresponds to switch's index of three half bridge,
	whose format :	0 : A upper, 	1 : A lower;
					2 : B upper,	3 : B lower;
					4 : C upper,	5 : C lower.
	The values of this table are switches' states --> 0 : OFF, 1 : ON 
*/

static uint8_t _pulses_tab[8][6] = { { 0, 0, 0, 0, 0, 0 },	/* error hall signal */
									 { 1, 0, 0, 1, 0, 0 },	/* sector number 1 */
									 { 1, 0, 0, 0, 0, 1 },	/* sector number 2 */
									 { 0, 0, 1, 0, 0, 1 },	/* sector number 3 */
									 { 0, 1, 1, 0, 0, 0 },	/* sector number 4 */
									 { 0, 1, 0, 0, 1, 0 },	/* sector number 5 */
									 { 0, 0, 0, 1, 1, 0 },	/* sector number 6 */
									 { 0, 0, 0, 0, 0, 0 },	/* error hall signal */
};

static void _get_duty(lt_trape_t trape, uint8_t* pulses,float duty);

lt_trape_t lt_trape_create(float mag,uint8_t mode)
{
	lt_trape_t _trape;
	_trape = lt_malloc(sizeof(struct lt_trape_object));
	if(_trape == NULL) return NULL;
	memset(_trape,0,sizeof(struct lt_trape_object));
	_trape->mag = fabsf(mag);
	if(mode != TRAPZOID_MODE_CENTER && mode != TRAPZOID_MODE_ADAPTIVE) 
	{
		mode = TRAPZOID_MODE_DEFAULT;
	}
	_trape->mode = mode;
	
	return _trape;
}

void lt_trape_delete(lt_trape_t trape)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(trape);
#endif
	lt_free(trape);
}

void lt_trape_set(lt_trape_t trape, float mag,uint8_t mode)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(trape);
#endif
	trape->mag = fabsf(mag);
	if(mode != TRAPZOID_MODE_CENTER && mode != TRAPZOID_MODE_ADAPTIVE) 
	{
		mode = TRAPZOID_MODE_DEFAULT;
	}
	trape->mode = mode;
}

uint8_t* lt_trape_process(lt_trape_t trape,uint8_t hall_signal, float input, float*duty)
{	
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(trape);
	LT_CHECK_MAG(trape->mag);
#endif
	
	uint8_t* pulses;
	uint8_t sector = 0;
	if(hall_signal == 0 || hall_signal >= 7 )				/* error hall signal */
	{
		*duty = 0;
		pulses = _pulses_tab[sector];
		/* get three phases duty cycle */
		_get_duty(trape,pulses,*duty);
		return pulses;										/* return immediately */
	}
	
	if(input < 0)
	{
		sector = ((_hall_map[hall_signal] - 1) + 3) % 6 + 1;	/* sector start from 1 */
		input = -input; 
	}
	else
	{
		sector = _hall_map[hall_signal];
	}
	/* get duty cycle */
	float tmp = input/trape->mag;
	*duty = CONSTRAINS(tmp,1,0);
	pulses = _pulses_tab[sector];
	/* get three phases duty cycle */
	_get_duty(trape,pulses,*duty);
	
	return pulses;
}

void lt_trape_process2(lt_trape_t trape,float angle_el, float input)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(trape);
	LT_CHECK_MAG(trape->mag);
#endif
	uint8_t sector;
	uint8_t *pulses;
	float mag = trape->mag;
	float duty;
	/* get sector number */
	if(input < 0)
	{
		angle_el = angle_el + _PI;
		input = -input;
	}
	sector = (uint8_t)(6.0f * (lt_normalize(angle_el - _PI_6)/ _2_PI)) % 6 + 1;	/* from 1 to 6 */
	/* get target duty */
	input = (input > mag) ? mag : input;
	duty = input/mag;
	pulses = _pulses_tab[sector];
	/*get final duty cycles */
	_get_duty(trape,pulses,duty);
}

void lt_trape_get(lt_trape_t trape,float*duty_A, float*duty_B, float*duty_C)
{
#ifdef LT_DEBUG_ENABLE
	LT_CHECK_NULL(trape);
#endif
	/* return duty cycle */
	*duty_A = trape->dutyA;
	*duty_B = trape->dutyB;
	*duty_C = trape->dutyC;
}

static void _get_duty(lt_trape_t trape, uint8_t* pulses,float duty)
{
	float dutyA, dutyB, dutyC;
	float duty_p, duty_n;									/* positive and negative duty values */
	float center;
	
	/* check voltage modulation mode */
	if(trape->mode == TRAPZOID_MODE_ADAPTIVE)
	{
		center = duty/2.0f;
		duty_p = center;
		duty_n = -center;
	}
	else if(trape->mode == TRAPZOID_MODE_CENTER)					/* PWM modulation mode */
	{
		center = 0.5f;
		duty_p = duty/2.0f;									
		duty_n = -duty_p;
	}
	else
	{
		center = 0.0f;
		duty_p = duty;
		duty_n = 0.0f;
	}
	
	if(pulses[0] == pulses[1])	/* phase A high impedance */
	{
		dutyA = 0.0f;
		if(pulses[2] == 1)									/* B+C-*/
		{
			dutyB = duty_p;
			dutyC = duty_n;
		}
		else												/* B-C+*/
		{
			dutyB = duty_n;
			dutyC = duty_p;
		}
	}
	else if(pulses[2] == pulses[3]) /* phase B high impedance */
	{
		dutyB = 0.0f;
		if(pulses[0] == 1)									/* A+C-*/
		{
			dutyA = duty_p;
			dutyC = duty_n;
		}
		else												/* A-C+*/
		{
			dutyA = duty_n;
			dutyC = duty_p;
		}
	}
	else							/* phase C high impedance */
	{
		dutyC = 0;
		if(pulses[0] == 1)									/* A+B-*/
		{
			dutyA = duty_p;
			dutyB = duty_n;
		}
		else												/* A-B+*/
		{
			dutyA = duty_n;
			dutyB = duty_p;
		}
	}
		
	trape->dutyA = dutyA + center;
	trape->dutyB = dutyB + center;
	trape->dutyC = dutyC + center;
}
