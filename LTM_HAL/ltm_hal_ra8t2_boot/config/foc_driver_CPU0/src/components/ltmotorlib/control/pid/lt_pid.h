#ifndef LT_PID_H
#define LT_PID_H

#include <stdint.h>
#define PID_INTEGRAL_LIMIT          2000        /* 默认积分限幅 (‰) */
#define PID_OUTPUT_LIMIT            2000        /* 默认输出限幅 (‰) */

/*------------------------- PID 算法 ---------------------------------*/
struct lt_pid_object;
typedef struct lt_pid_object* lt_pid_t;

lt_pid_t lt_pid_create(float Kp, float Ki, float Kd, float ts_ms);
void lt_pid_delete(lt_pid_t pid);

void lt_pid_reset(lt_pid_t pid);
void lt_pid_set(lt_pid_t pid, float Kp, float Ki, float Kd);
void lt_pid_set_target(lt_pid_t pid, float target);
void lt_pid_set_ts(lt_pid_t pid, float ts_ms);
void lt_pid_set_limits(lt_pid_t pid, float int_limit, float output_limit);
float lt_pid_get(lt_pid_t pid);
float lt_pid_process(lt_pid_t pid, float curr_val);          /* 增量式 PID */
float lt_pid_process2(lt_pid_t pid, float curr_val);         /* 位置式 PID */
/*------------------------- PID 算法 ---------------------------------*/

#endif