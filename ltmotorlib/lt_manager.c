/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2025-6-21      Lvtou        the first version
 */
#include "ltmotorlib.h"
#include "stdlib.h"
lt_manager_t _manager;

static lt_node_t _node_get(lt_node_t list,char* name)
{
	lt_node_t curr = list->next;
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

static lt_node_t _node_create(lt_motor_t motor,char*name)
{
	lt_node_t _node = rt_malloc(sizeof(struct lt_motor_node_object));
	if(_node == RT_NULL) return RT_NULL;
	rt_memset(_node,0,sizeof(struct lt_motor_node_object));
	_node->motor = motor;
	rt_strcpy(_node->name,name);
	_node->prev = _node;
	_node->next = _node;
	
	return _node;
}

static void _node_add(lt_node_t list,lt_node_t node)
{
	lt_node_t curr = list->next;
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

static void _node_delete(lt_node_t list,lt_node_t node)
{
	/* in cycle list case, list->prev = end, this is so beautiful */
	node->prev->next = node->next;
	node->next->prev = node->prev;
	rt_free(node);								/* release memory */
}


/* implements a cycle list */
int lt_manager_create(void)
{
	_manager = rt_malloc(sizeof(struct lt_motor_manager_object));
	if(_manager != RT_NULL) 
	{
		rt_memset(_manager,0,sizeof(struct lt_motor_manager_object));
		_manager->list = _node_create(RT_NULL,"");
		rt_kprintf("LtMotorLib motor manager is created successfully! \n");
		rt_kprintf("Input 'ltmotorlib' to start the journey!!! \n");
	}
	return RT_EOK;
}

rt_err_t lt_manager_add_motor(lt_motor_t motor)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_motor_get_info(motor,&(_manager->info));						/* get motor info */
	lt_node_t _node = _node_create(motor,_manager->info.name);		/* create a node */
	_node_add(_manager->list,_node);									/* add node to list */
	return RT_EOK;
}

rt_err_t lt_manager_delete_motor(lt_motor_t motor)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_motor_get_info(motor,&(_manager->info));							/* get motor info */
	lt_node_t _node = _node_get(_manager->list,_manager->info.name);		/* get node */
	if(_node != RT_NULL)
	{
		_node_delete(_manager->list,_node);
		return RT_EOK;
	}
	else
	{
		return RT_ERROR;
	}
}

lt_motor_t lt_manager_get_motor(char* name)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_node_t node = _node_get(_manager->list,name);
	if(node == RT_NULL) return RT_NULL;
	else return node->motor;
}

rt_err_t lt_manager_delete(void)
{
	RT_ASSERT(_manager != RT_NULL);
	lt_node_t list = _manager->list;
	lt_node_t tmp = list;
	lt_node_t curr = list->next;
	while(curr != list)
	{
		tmp = curr->next;		/* save next pointer */
		_node_delete(list,curr);
		curr = tmp;
	}
	_node_delete(list,list);	/* delete head pointer */
	rt_free(_manager);			/* release memory */
	return RT_EOK;
}
INIT_DEVICE_EXPORT(lt_manager_create);			/* create motor manager automatically */

/* use FINSH to help test functions */
#ifdef RT_USING_FINSH
#ifdef LT_USING_MOTOR_MSH_TEST
/* list motors  */
#define MANAGER_UNVALID				0x00
#define MANAGER_INTRO				0x01
#define MANAGER_HELP				0x02
#define MANAGER_LIST				0x03
#define MANAGER_PID					0x04
static void _manager_info(int type)
{
	if(type == MANAGER_UNVALID)
	{
		rt_kprintf("unvalid cmd!!! \n");
		rt_kprintf("Input 'ltmotorlib help' to see more details \n");
	}
	else if(type == MANAGER_INTRO)
	{
		rt_kprintf("LtMotorLib --> A motor control library based on RT_Thread RTOS!!!\n ");
		rt_kprintf("Author: LvTou, Date: 2025/4/22, Version: 0.2 \n");
		rt_kprintf("LtMotorLib supports finsh and simple pid interface to help you control motors !\n ");
		rt_kprintf("Input like this 'ltmotorlib motor cmd' to call correspond functions \n");
		rt_kprintf("Input 'ltmotorlib list' to see motors' information \n");
		rt_kprintf("Input 'ltmotorlib test motor' to config test motor \n");
		rt_kprintf("Input 'ltmotorlib pid' to see close loop pid details \n");
		rt_kprintf("Input 'ltmotorlib help' to see cmd called formats and details \n");
		rt_kprintf("Hope you enjoy it!!! \n\n");
	}
	else if(type == MANAGER_HELP)
	{
		rt_kprintf("Input 'ltmotorlib list' to see motors' information \n");
		rt_kprintf("Input 'ltmotorlib test motor' to config test motor \n");
		rt_kprintf("Input 'ltmotorlib pid' to see close loop pid details \n");
		rt_kprintf("Note:  supported test motors: motor_dc, x_stepper, y_stepper \n");
		rt_kprintf("LtMotorLib provides simple pid interface for user \n");
		rt_kprintf("Notice that bellow 'motor' is your configured motor name! \n");
		rt_kprintf("ltmotorlib call formats: \n\n");
		rt_kprintf("******************* basic part ************************\n");
		rt_kprintf("ltmotorlib motor output val \n");
		rt_kprintf("ltmotorlib motor output_angle val \n");
		rt_kprintf("ltmotorlib motor output_pid val \n");
		rt_kprintf("ltmotorlib motor output_angle_pid val \n");
		rt_kprintf("ltmotorlib motor get_pos \n");
		rt_kprintf("ltmotorlib motor get_vel \n");
		rt_kprintf("ltmotorlib motor disbale \n");
		rt_kprintf("ltmotorlib motor delete \n");
		rt_kprintf("******************* basic part ************************\n\n");
		rt_kprintf("******************* stepper specified part ************************\n");
		rt_kprintf("Note: the s_curve and 5_section output symmetric velocity curves and parameter units are as followed \n");
		rt_kprintf("trapzoid  --> step: step, acc: Hz/ms,   dec:  Hz/ms speed: Hz \n");
		rt_kprintf("s_curve   --> step: gap,   acc_t: gap,  freq: Hz, flexible: the bigger, the closer to s curve\n");
		rt_kprintf("5_section --> step: step, acc_t: ms,    speed: Hz \n\n");
		rt_kprintf("ltmotorlib motor trapzoid step acc dec speed \n");
		rt_kprintf("ltmotorlib motor s_curve step acc_t freq_max freq_min flexible \n");
		rt_kprintf("ltmotorlib motor 5_section step acc_t speed \n");
		rt_kprintf("ltmotorlib motor line_interp y_motor x_start, y_start, x_end, y_end \n");
		rt_kprintf("ltmotorlib motor circular_interp y_motor x_start y_start x_end y_end  radius dir(CW/CCW) \n");
		rt_kprintf("******************* stepper specified part ************************\n\n");
		rt_kprintf("******************* bldc specified part ************************\n");
		rt_kprintf("ltmotorlib motor elec_info \n");
		rt_kprintf("ltmotorlib motor torque value\n");
		rt_kprintf("******************* bldc specified part ************************\n\n");
	}
	else if(type == MANAGER_LIST)
	{
		lt_node_t list = _manager->list;
		lt_node_t curr = list->next;
		struct lt_motor_info * info = &(_manager->info);
		rt_kprintf("%-*s %-15s %-15s %-15s \n",LT_NAME_MAX+5,"NAME","TYPE","STATUS","POSITION(degree)");
		while(curr != list)
		{
			lt_motor_get_info(curr->motor,info);
			rt_kprintf("%-*s %-15s %-15s %-15.2f \n",LT_NAME_MAX+5,info->name,_type[info->type],_status[info->status],info->position*180.0f/PI);
			curr = curr->next;
		}
	}
	else if(type == MANAGER_PID)
	{
		rt_kprintf("LtMotorLib provides pid adjust interface for user \n");
		rt_kprintf("This part is coorporated with communicator for oscilloscope \n");
		rt_kprintf("Note: dac output voltage (dV), dV < 1.65V : (-); dV > 1.65V : (+) \n");
		rt_kprintf("1.65V <==> 2048 output! \n");
		rt_kprintf("supported pid type: pid_vel/pid_pos/pid_current \n");
		rt_kprintf("current pid is only for BLDC motor\n");
		rt_kprintf("ltmotorlib motor pid_type set Kp Ki Kd \n");
		rt_kprintf("ltmotorlib motor pid_type set_target value \n");
		rt_kprintf("ltmotorlib motor pid_type set_dt time(ms) \n");
		rt_kprintf("ltmotorlib motor pid_type reset \n ");
		rt_kprintf("ltmotorlib motor pid_start \n");
		rt_kprintf("ltmotorlib motor pid_stop \n ");
	}
}

/* motor basic part */
static void _motor_basic(int argc,char *argv[],lt_motor_t motor, struct lt_motor_info* info)
{
	rt_uint8_t flag;
	if(!rt_strcmp(argv[2],"output"))
	{	/* motor basic part*/
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"output_angle"))
	{
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output_angle(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"get_pos"))
	{
		test_motor_get_position(motor);
		flag = 1;
	}
	else if(!rt_strcmp(argv[2],"get_vel"))
	{
		test_motor_get_velocity(motor);
		flag = 1;
	}
	else if(!rt_strcmp(argv[2],"output_pid"))
	{
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output_pid(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"output_angle_pid"))
	{
		if(argc == 4)	/* check params */
		{
			float input = (float)atof(argv[3]);
			test_motor_output_angle_pid(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"disable"))
	{
		lt_motor_disable(motor);
		flag = 1;
	}
	if(!flag) _manager_info(MANAGER_UNVALID);
}
/* stepper motor part */
static void _motor_stepper_part(int argc,char *argv[],lt_motor_t motor, struct lt_motor_info* info)
{
	rt_uint8_t flag = 0;
	if(!rt_strcmp(argv[2],"trapzoid"))
	{
		if(argc == 7)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc = (float)atof(argv[4]);
			float dec = (float)atof(argv[5]);
			float speed = (float)atof(argv[6]);
			test_stepper_trapzoid(motor,step,acc,dec,speed);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"s_curve"))
	{
		if(argc == 8)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc_t = (float)atof(argv[4]);
			float max = (float)atof(argv[5]);
			float min = (float)atof(argv[6]);
			float flex = (float)atof(argv[7]);
			test_stepper_s_curve(motor,step,acc_t,max,min,flex);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"5_section"))
	{
		if(argc == 6)		/* check paramaters */
		{
			int step = atoi(argv[3]);
			float acc_t = (float)atof(argv[4]);
			float speed = (float)atof(argv[5]);
			test_stepper_5_section(motor,step,acc_t,speed);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"line_interp") || !rt_strcmp(argv[2],"circular_interp"))
	{
		/* check y_motor */
		lt_motor_t y_motor = lt_manager_get_motor(argv[3]);
		if(y_motor == RT_NULL)
		{
			rt_kprintf("there is no motor named:%s !!! \n",argv[3]);
			return;
		}
		else										/* check y_motor type*/
		{
			lt_motor_get_info(y_motor,info);		/* get motor info */
			if(info->type != MOTOR_TYPE_STEPPER)	
			{
				rt_kprintf("motor %s type is incorrect! expected: %s , actual: %s !!!\n",info->name, _type[MOTOR_TYPE_STEPPER],_type[info->type]);
				return;
			}
		}
		/* check params*/
		if(argc == 8 || argc == 10)	
		{
			int x_start = atoi(argv[4]);
			int y_start = atoi(argv[5]);
			int x_end = atoi(argv[6]);
			int y_end = atoi(argv[7]);
			if(argc == 8)				/* line interp */
			{
				test_stepper_line_interp(motor,y_motor,x_start,y_start,x_end,y_end);
				flag = 1;
			}
			else						/* circular interp */
			{
				float radius = atoi(argv[8]);
				rt_uint8_t dir;
				if(!rt_strcmp(argv[9],"CW"))
				{
					dir = DIR_CW;
					flag = 1;
				}
				else
				{
					dir = DIR_CCW;
					flag = 1;
				}
				test_stepper_circular_interp(motor,y_motor,x_start,y_start,x_end,y_end,radius,dir);
			}
		}
	}
	if(!flag) _manager_info(MANAGER_UNVALID);
}

/* bldc motor part */
static void _motor_bldc_part(int argc,char *argv[],lt_motor_t motor, struct lt_motor_info* info)
{
	rt_uint8_t flag = 0;
	if(!rt_strcmp(argv[2],"torque"))
	{
		if(argc == 4)		/* check paramaters */
		{
			float input = (float)atof(argv[3]);
			test_bldc_torque(motor,input);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"elec_info"))
	{
		if(argc == 3)		/* check paramaters */
		{
			test_bldc_elec_info(motor);
			flag = 1;
		}
	}
	
	if(!flag) _manager_info(MANAGER_UNVALID);
}

static void _motor_close_loop_part(int argc,char *argv[],lt_motor_t motor, struct lt_motor_info* info)
{
	rt_uint8_t flag = 0;
	lt_timer_t timer = motor->timer;
	if(!rt_strcmp(argv[2],"pid_vel") || !rt_strcmp(argv[2],"pid_pos") || !rt_strcmp(argv[2],"pid_current"))
	{
		lt_pid_t pid;
		if(!rt_strcmp(argv[2],"pid_vel"))
		{
			pid = motor->pid_vel;
		}
		else if(!rt_strcmp(argv[2],"pid_pos"))
		{
			pid = motor->pid_pos;
		}
		else if(!rt_strcmp(argv[3],"pid_current"))
		{
			if(info->type != MOTOR_TYPE_BLDC)
			{
				rt_kprintf("motor %s type is incorrect! expected: %s , actual: %s !!!\n",info->name, _type[MOTOR_TYPE_BLDC],_type[info->type]);
				return;
			}
		}
		/* check paramaters */
		if(argc < 4){
			_manager_info(MANAGER_UNVALID);
			return;
		};
		if(!rt_strcmp(argv[3],"set"))	/* set pid */
		{
			if(argc == 7)
			{
				float Kp = atof(argv[4]);
				float Ki = atof(argv[5]);
				float Kd = atof(argv[6]);
				lt_pid_set(pid,Kp,Ki,Kd);
				flag = 1;
			}
		}
		else if(!rt_strcmp(argv[3],"set_target"))
		{
			if(argc == 5)
			{
				float target = atof(argv[4]);
				lt_pid_set_target(pid,target);
				flag = 1;
			}
		}
		else if(!rt_strcmp(argv[3],"set_dt"))
		{
			if(argc == 5)
			{
				float dt = atof(argv[4]);
				lt_pid_set_dt(pid,dt);
				lt_timer_set(timer,dt*1000,TIMER_MODE_PERIODIC,TEST_PID_TIMER_TYPE);	/* ms --> us */
				flag = 1;
			}
		}
		else if(!rt_strcmp(argv[3],"reset"))
		{
			lt_pid_reset(pid);
			lt_motor_disable(motor);
			lt_timer_disable(timer,TEST_PID_TIMER_TYPE);
			flag = 1;
		}
	}
	else if(!rt_strcmp(argv[2],"pid_stop"))
	{
		lt_motor_disable(motor);
		lt_timer_disable(timer,TEST_PID_TIMER_TYPE);
		flag = 1;
	}
	else if(!rt_strcmp(argv[2],"pid_start"))
	{
		lt_timer_enable(timer,TEST_PID_TIMER_TYPE);
		flag = 1;
	}
	
	if(!flag) _manager_info(MANAGER_UNVALID);
}

static void _motor_test_config(char* name)
{
	lt_motor_t motor = lt_manager_get_motor(name);
	rt_err_t res;
	
	if(motor != RT_NULL)
	{
		rt_kprintf("already config motor: %s !!! \n",name);
		return;
	}
	
	if(!rt_strcmp(name,"motor_dc"))
	{
		res = test_motor_dc_config();
	}
	else if(!rt_strcmp(name,"x_stepper"))
	{
		res = test_stepper_x_config();
	}
	else if(!rt_strcmp(name,"y_stepper"))
	{
		res = test_stepper_y_config();
	}
	else if(!rt_strcmp(name,"x_bldc"))
	{
		res = test_bldc_x_config();
	}
	else if(!rt_strcmp(name,"y_bldc"))
	{
		res = test_bldc_y_config();
	}
	else
	{
		rt_kprintf("there is no test motor named:%s !!! \n",name);
		return;
	}
	if(res != RT_EOK)
	{
		rt_kprintf("config %s failed! \n",name);
	}
	else
	{
		rt_kprintf("config %s successfully! \n",name);
	}
}

static void ltmotorlib(int argc, char*argv[])
{
	RT_ASSERT(_manager != RT_NULL);
	lt_motor_t motor;
	struct lt_motor_info* info = &(_manager->info);
	
	if(argc == 1)
	{
		_manager_info(MANAGER_INTRO);
		return;
	}
	else if(argc == 2)
	{
		if(!rt_strcmp(argv[1],"list"))
		{
			_manager_info(MANAGER_LIST);
		}
		else if(!rt_strcmp(argv[1],"help"))
		{
			_manager_info(MANAGER_HELP);
		}
		else if(!rt_strcmp(argv[1],"pid"))
		{
			_manager_info(MANAGER_PID);
		}
		else
		{
			_manager_info(MANAGER_UNVALID);
		}
		return;
	}
	
	if(argc == 3 && !rt_strcmp(argv[1],"test"))
	{
		_motor_test_config(argv[2]);
		return;
	}
	/* find motor */
	motor = lt_manager_get_motor(argv[1]);
	if(motor == RT_NULL)
	{
		rt_kprintf("there is no motor named:%s !!! \n",argv[1]);
		return;
	}
	
	/* check command */
	if(!rt_strcmp(argv[2],"delete"))
	{
		lt_motor_delete(motor);
		rt_kprintf("delete motor: %s successfully! \n",argv[1]);
		return;
	}
	else if(!rt_strcmp(argv[2],"output") || !rt_strcmp(argv[2],"output_angle") || !rt_strcmp(argv[2],"get_pos") || !rt_strcmp(argv[2],"get_vel") || !rt_strcmp(argv[2],"output_pid") || !rt_strcmp(argv[2],"output_angle_pid") || !rt_strcmp(argv[2],"disable") )
	{	/* motor basic part*/
		_motor_basic(argc,argv,motor,info);
	}
	else if(!rt_strcmp(argv[2],"trapzoid") || !rt_strcmp(argv[2],"s_curve") ||!rt_strcmp(argv[2],"5_section") || !rt_strcmp(argv[2],"line_interp") || !rt_strcmp(argv[2],"circular_interp"))
	{	/* stepper motor part */
		lt_motor_get_info(motor,info);			/* get motor info */
		if(info->type != MOTOR_TYPE_STEPPER)	/* check motor type */
		{
			rt_kprintf("motor %s type is incorrect! expected: %s , actual: %s !!!\n",info->name, _type[MOTOR_TYPE_STEPPER],_type[info->type]);
			return;
		}
		_motor_stepper_part(argc,argv,motor,info);
	}
	else if(!rt_strcmp(argv[2],"torque") || !rt_strcmp(argv[2],"elec_info"))
	{	/* bldc motor part */
		lt_motor_get_info(motor,info);		/* get motor info */
		if(info->type != MOTOR_TYPE_BLDC)	/* check motor type */
		{
			rt_kprintf("motor %s type is incorrect! expected: %s , actual: %s !!!\n",info->name, _type[MOTOR_TYPE_BLDC],_type[info->type]);
			return;
		}
		_motor_bldc_part(argc,argv,motor,info);
	}
	else if(!rt_strcmp(argv[2],"pid_vel") || !rt_strcmp(argv[2],"pid_pos") || !rt_strcmp(argv[2],"pid_current") || !rt_strcmp(argv[2],"pid_start") || !rt_strcmp(argv[2],"pid_stop"))
	{
		lt_motor_get_info(motor,info);		/* get motor info */
		_motor_close_loop_part(argc,argv,motor,info);
	}		
	else
	{
		_manager_info(MANAGER_UNVALID);
	}
	
}
MSH_CMD_EXPORT(ltmotorlib, A powerful motor control library);
#endif
#endif
