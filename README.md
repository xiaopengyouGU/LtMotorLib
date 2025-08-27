# LtMotorLib电机控制库
LtMotorLib是一个基于RT-Thread实时操作系统的电机控制库，亦是一套电机控制的软件解决方案。相较simpleFOC和DengFOC等电机库更适合于多线程开发，支持跨平台移植。覆盖直流，步进，无刷直流三类常用电机，配备了强大的命令行调试工具，支持上位机PID调试和FOC等常用控制算法，附带MATLAB算法仿真器和CSV数据处理器。库的底层源码采用面向对象和模块化设计相结合的方式实现，代码整洁，调用API接口直观明了。同时提供非常详细的使用说明文档和移植教程。本项目持续更新中，不定期发布相关技术文档。
​
## 一、项目指南
### [LtMotorLib电机控制库（1）项目介绍](https://blog.csdn.net/askedGU/article/details/148498337)
### [LtMotorLib电机控制库（2）使用说明](https://blog.csdn.net/askedGU/article/details/148871007?spm=1001.2014.3001.5501)
### [LtMotorLib电机控制库（3）面向对象与模块化](https://blog.csdn.net/askedGU/article/details/149283572?spm=1001.2014.3001.5501)
### [LtMotorLib电机控制库（4）步进加减速与插补](https://blog.csdn.net/askedGU/article/details/149465020?spm=1001.2014.3001.5501)
因篇幅有限，下文各部分仅有少量展示，更详细的细节见上述博客。
​
## 二、总体架构
下图可以很好地说明LtMotorLib电机控制库的总体架构和生态位。
<div align="center">
  <img src="https://github.com/user-attachments/assets/f4caff33-5e9e-4452-a9f0-a1833fb1e264" width="80%" style="border:1px solid #eee; padding:10px">
  
  **图1-1**：LtMotorLib架构图（左）和生态位（右）
</div>

本库由电机对象、调试组件、抽象外设、功能块、测试框架及示例五个部分组成。各部分的分工明确：电机对象负责标准电机控制接口、调试组件处理与上位机通讯有关的事项、抽象外设完成了电机开闭环底层配置的封装、功能块内置步进加减速、foc控制等常用算法、测试框架让快速验证代码成为可能。从生态位的角度看，LtMotorLib可算作在RT-Thread实时操作系统基础上进行的应用层开发。

## 三、基于RT-Thread RTOS
LtMotorLib深度对接了RT-Thread的标准设备驱动框架，做到与具体外设的隔离，支持核心代码零修改跨平台移植。同时将RT-Thread内核层的软件定时器和设备驱动层的硬件定时器有机整合，极大简化了定时器中断配置流程。并通过多线程和信号量等机制完成了对野火上位机PID通讯协议的解析和闭环PID调试模块。也正因此，LtMotorLib非常适合于基于多线程的电机控制方案。在FOC，DTC等高性能电机控制使用场景下，LtMotorLib还可以发挥出多线程和定时器中断的潜力。这一点是LtMotorLib相较于simpleFOC，DengFOC等常用电机控制库的最大优势。
<details>
<summary>点击展开双闭环中断回调函数示例</summary>

```c
void test_position_velocity_loop(lt_timer_t timer)
{
	lt_motor_t motor = (lt_motor_t)timer->user_data;
	lt_pid_t pid_vel = motor->pid_vel;
	lt_pid_t pid_pos = motor->pid_pos;
	/* we adjust vel_pid first */
	/* vel_pid: DC motor: Kp = 5.6, Ki = 2.0, Kd = 0.01 */
	/* pos_pid: DC motor: Kp = 10, Ki = 0, Kd = 0 */
	static rt_uint32_t count = 0;
	/* process position loop first, sample time 3T */
	if(count % 3 == 0)
	{
		float position = lt_motor_get_position(motor)*180.0f/PI;	/* unit: degree */
		float desired_vel = PID_POS_CONST * lt_pid_control(pid_pos,position);
		//float desired_vel = 0.0225 * lt_pid_control(pid_pos,position); /* for stepper pos_vel pid */
		lt_pid_set_target(pid_vel,desired_vel);
		if(count % (3*TEST_PID_COMMUT_TIMES) == 0)
		{
			int t_position = position;
			lt_communicator_send(SEND_FACT_CMD,CURVES_CH1,&t_position,1);
		}
	}
	/* velocity loop follow, sample time T */
	float vel = lt_motor_get_velocity(motor,pid_vel->dt*1000)*9.55;	/* get rpm, s --> ms */
	float control_u = PID_VEL_CONST * lt_pid_control(pid_vel,vel);
	lt_motor_control(motor,MOTOR_CTRL_OUTPUT,&control_u);		/* output! */
	int t_speed = vel, t_control_u = control_u;
	
	if(count % TEST_PID_COMMUT_TIMES == 0)
	{
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH2,&t_speed,1);
#ifndef COMMUNICATOR_TYPE_OSCILLOSCOPE
		lt_communicator_send(SEND_FACT_CMD,CURVES_CH3,&t_control_u,1);
#endif
	}
	count++;
}
```
</details>


## 四、面向对象与模块化
LtMotorLib采用C语言实现了面向对象（OOP）的核心机制：通过数据抽象建立硬件隔离层，基于结构体组合模拟类继承体系，利用函数指针动态绑定实现多态，并构建了完整的对象生命周期管理模型。该库综合应用抽象工厂、策略及单例等设计模式，极大简化了代码底层逻辑，具备良好的可扩展性。基于此架构定义了电机标准控制接口，并将插补算法、FOC控制等核心算法封装为即插即用的功能模块，提供简洁的API接口。针对嵌入式场景的实时性需求与资源约束，实施了关键工程优化：开放结构体成员以降低访问开销，采用动态内存管理。
​<details>
<summary>点击展开电机结构体（基类）和标准电机控制接口</summary>

```c
/* define motor struct */
struct lt_motor_object{
	
	char  name[LT_NAME_MAX]; 
	rt_uint8_t reduction_ratio;
	lt_driver_t driver;					/* motor driver */
	lt_sensor_t sensor;					/* position sensor */
	lt_timer_t timer;					/* timer */
	lt_pid_t pid_vel;					/* pid object for simple closed loop output */
	lt_pid_t pid_pos;
	lt_pid_t pid_current;				/* current pid object */
	
	rt_uint8_t type;
	rt_uint8_t pid_type;				/* used pid type: pos/vel pid */
	rt_uint8_t status;					/* motor status */
	
	const struct lt_motor_ops *ops;		/* motor control operators */
	rt_err_t (*callback)(void*);		/* done callback function, used for accel and interp */
	void* call_param;					/* callback function param */
	
	void* user_data;
};
typedef struct lt_motor_object* lt_motor_t;

/* motor control operator */
struct lt_motor_ops
{	
	lt_motor_t(*create)(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
	rt_err_t (*control)(lt_motor_t motor, int cmd,void* arg);
	rt_err_t(*_delete)(lt_motor_t motor);
};

lt_motor_t lt_motor_create(char* name,rt_uint8_t reduction_ration,rt_uint8_t type);
rt_err_t lt_motor_set_driver(lt_motor_t motor, lt_driver_t driver);
rt_err_t lt_motor_set_sensor(lt_motor_t motor, lt_sensor_t sensor);
rt_err_t lt_motor_set_callback(lt_motor_t, rt_err_t (*callback)(void*),void*call_param);
rt_err_t lt_motor_set_pid(lt_motor_t,lt_pid_t pid,rt_uint8_t pid_type);
rt_err_t lt_motor_set_timer(lt_motor_t motor, lt_timer_t timer);

float lt_motor_get_velocity(lt_motor_t, rt_uint32_t measure_time_ms);
float lt_motor_get_position(lt_motor_t);
rt_err_t lt_motor_get_info(lt_motor_t, struct lt_motor_info*);
rt_err_t lt_motor_control(lt_motor_t, int cmd, void* arg);
rt_err_t lt_motor_enable(lt_motor_t,rt_uint8_t dir);
rt_err_t lt_motor_disable(lt_motor_t);
rt_err_t lt_motor_delete(lt_motor_t);					/* delete a motor! */

```
</details>


## 五、调试工具
LtMotorLib实现了一个强大又简单易用的调试工具（基于MSH），支持PID、开闭环在内的基本所有功能的调试，仅需几行指令就可快速验证，大大减低了用户的开发工作量。同时，借助多线程、信号量等机制，LtMotorLib完成了对野火上位机PID调试助手通讯协议的解析和闭环PID调试模块。值得一提的是，LtMotorLib还支持示波器版本的PID调试功能（采用DAC输出），与上位机版本共享一套操作接口。用户通过宏开关即可完成已有的闭环示例配置和选择，上手难度非常低。
<div align="center">
  <img src="https://github.com/user-attachments/assets/eae8a758-e1da-4c1a-8e32-3b257a7ad626" width="80%" style="border:1px solid #eee; padding:10px">
  
  **图1-2**：调试工具示例
</div>

## 六、MATLAB部分
LtMotorLib附带基于MATLAB的算法仿真器和上位机CSV数据处理器。前者支持foc（SPWM，SVPWM）、逐点比较法插补（直线与圆弧）和加减速算法（梯形、S型、5段）的仿真，调用格式与库对应部分相同。后者针对野火上位机调试助手导出的csv数据格式，实现了去毛刺，缩放，格式化输出等功能。
<div align="center">
  <img src="https://github.com/user-attachments/assets/16e00e54-dcf7-4d72-b14f-15b4399dd7f1" width="80%" style="border:1px solid #eee; padding:10px">
  
  **图1-3**：csv数据格式化输出示例
</div>

​
