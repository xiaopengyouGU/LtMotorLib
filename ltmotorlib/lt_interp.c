/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "math.h"

/* circular interp map table, save large amounts of calculation time */
rt_uint8_t _map_circular[4][4] = {	{ 0x0,	 0x3,	0x1,	0x2},
									{ 0x1,	 0x0,	0x2,	0x3},
									{ 0x2,	 0x1,	0x3,	0x0},
									{ 0x3,	 0x2,	0x0,	0x1}
};
/* each value has 4 bits, eg: 0x6 = 2#0110, bit3~bit0					
*  	bit0: move axis, 			0: X, 		1: Y
*	bit1: move direction		1: +,FOR, 		0: -,REV
*	the x axis of this _map_table corresponds to four quadrant: 0~3 => 1th ~ 4th
* 	the y axis of this _map_table corresponds to interp direction plus deviation 
*	0: CCW + devia >= 0;	1: CCW + devia < 0;		2: CW + devia >= 0;		3: CW + devia < 0
*/
rt_uint8_t _map_line[4] = { 0x3,
							0x2,
							0x0,
							0x1};
/* bit0: x_dir, bit1: y_dir; 1: FORWARD, 0: REVERSAL */

static rt_err_t _pre_process(lt_interp_t interp,int x_start, int y_start, int x_end, int y_end, rt_uint16_t r, rt_uint8_t dir);
static rt_uint8_t _calc_line(lt_interp_t interp,rt_uint8_t*dir);
static rt_uint8_t _calc_circular(lt_interp_t interp, rt_uint8_t*dir);

lt_interp_t lt_interp_create(void)
{
	lt_interp_t _interp;
	_interp = rt_malloc(sizeof(struct lt_interp_object));
	if(_interp != RT_NULL)
	{
		rt_memset(_interp,0,sizeof(struct lt_interp_object));
	}
	return _interp;
}

rt_err_t lt_interp_set(lt_interp_t interp,struct lt_interp_config* config)
{
	RT_ASSERT(interp != RT_NULL);
	RT_ASSERT(config != RT_NULL);
	/* check input */
	if(config->dir != DIR_CCW && config->dir != DIR_CW) return RT_ERROR;
	rt_err_t res = _pre_process(interp,config->x_start,config->y_start,config->x_end,config->y_end,config->radius,config->dir);
	return res;
}

rt_uint8_t lt_interp_process(lt_interp_t interp,rt_uint8_t*dir)
{
	RT_ASSERT(interp != RT_NULL);
	rt_uint8_t move_axis;
	if(interp->radius == 0)				/* line interp */
	{
		move_axis = _calc_line(interp,dir);
	}
	else
	{
		move_axis = _calc_circular(interp,dir);
	}
	
	return move_axis;
}
	
rt_err_t lt_interp_delete(lt_interp_t interp)
{
	RT_ASSERT(interp != RT_NULL);
	if(interp != RT_NULL)
	{
		rt_free(interp);
	}
	return RT_EOK;
}

static rt_err_t _pre_process(lt_interp_t interp,int x_start, int y_start, int x_end, int y_end, rt_uint16_t r, rt_uint8_t dir)
{
	interp->deviation = 0;
	interp->dir = dir;
	interp->radius = r;
	
	if (r == 0)	/* line interp */
	{
		interp->x_pos = 0;
		interp->y_pos = 0;
		interp->x_end = x_end - x_start;
		interp->y_end = y_end - y_start;
		interp->quadrant = _get_quard(interp->x_end,interp->y_end,0);
		interp->num_pulse = _abs(interp->x_end) + _abs(interp->y_end);
		interp->x_end = _abs(interp->x_end);
		interp->y_end = _abs(interp->y_end);
	}
	else		/* circular interp */
	{
		float x_center, y_center;
		rt_uint8_t res = _get_center(x_start,y_start,x_end,y_end,r,dir,&x_center,&y_center);
		if (res == 0) return RT_ERROR;						/* radius is too small */
		int x_cen = (int)x_center, y_cen = (int)y_center;
		float bias = _absf(x_center - x_cen) + _absf(y_center - y_cen);
		if(bias <= 0.000001f)		/* we regard that cicular center is integers */
		{
			interp->exact = 1;
		}
		else
		{
			interp->exact = 0;
		}
		interp->x_pos = x_start - x_cen;
		interp->y_pos = y_start - y_cen;
		interp->x_end = x_end - x_cen;
		interp->y_end = y_end - y_cen;
	}
	return RT_EOK;
}	

static rt_uint8_t _get_line_dir(rt_uint8_t quadrant,int devia)
{
	rt_uint8_t val = _map_line[quadrant];
	if (devia >= 0)
		val = GET_BIT(val,0);
	else
		val = GET_BIT(val,1);
	
	if(val == 1) return ROT_FORWARD;
	else return ROT_REVERSAL;
}

static rt_uint8_t _calc_line(lt_interp_t interp,rt_uint8_t*dir)
{
	rt_uint8_t move_axis;
	int devia = interp->deviation;
	if (interp->num_pulse == 0) return 0;
	interp->num_pulse = interp->num_pulse - 1;
	/* get move_dir */
	*dir = _get_line_dir(interp->quadrant,devia);
  
	if (devia >= 0)
	{
		devia = devia - interp->y_end;
		move_axis = X_MOTOR_MOVE;
	}
	else
	{
		devia = devia + interp->x_end;
		move_axis = Y_MOTOR_MOVE;
	}
	interp->deviation = devia;				/* refresh deviation */
	
	return move_axis;
}

static rt_uint8_t _get_circular_dir(int x_pos, int y_pos,rt_uint8_t dir, int devia,rt_uint8_t *move_dir)
{
	rt_uint8_t _x = _get_quard(x_pos,y_pos,dir), _y;
	rt_uint8_t val, move_axis;
	
	if (dir == DIR_CCW)
	{
		if(devia >= 0)
			_y = 0;
		else
			_y = 1;
	}
	else
	{
		if(devia >= 0)
			_y = 2;
		else
			_y = 3;
	}
	
	val = _map_circular[_x][_y];
	move_axis = GET_BIT(val,0) + 1;
	*move_dir = _abs(ROT_REVERSAL - GET_BIT(val,1));
	
	return move_axis;
}

static rt_uint8_t _calc_circular(lt_interp_t interp, rt_uint8_t*dir)
{
	int x_pos = interp->x_pos;
	int y_pos = interp->y_pos;
	int devia = interp->deviation;
	rt_uint8_t res = _check_end(x_pos,y_pos,interp->x_end,interp->y_end,interp->exact);
	rt_uint8_t move_axis, move_dir,_dir;
	if(res == 1) return 0;
    /* get move dir and move axis */
    move_axis = _get_circular_dir(x_pos,y_pos,interp->dir,devia,&move_dir);
	_dir = _abs(move_dir - ROT_REVERSAL);	/* 1: FOR, 0: REV */
	*dir = move_dir;						
	
	if(move_axis == X_MOTOR_MOVE)
	{
		if(devia >= 0)
		{
			devia = devia - 2*_abs(x_pos) + 1;
			x_pos = _abs_sub_1(x_pos);
		}
		else
		{
			devia = devia + 2*_abs(x_pos) + 1;
			x_pos = _abs_plus_1(x_pos,_dir);
		}
	}
	else		/* Y motor move */
	{
		if(devia >= 0)
		{
			devia = devia - 2*_abs(y_pos) + 1;
			y_pos = _abs_sub_1(y_pos);
		}
		else
		{
			devia = devia + 2*_abs(y_pos) + 1;
			y_pos = _abs_plus_1(y_pos,_dir);
		}
	}
    /* refresh deviation and pos */
	interp->deviation = devia;
	interp->x_pos = x_pos;
	interp->y_pos = y_pos;
	
	return move_axis;
}
