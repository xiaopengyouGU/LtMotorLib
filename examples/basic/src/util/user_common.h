#ifndef __USER_COMMON_H__
#define __USER_COMMON_H__

#include <stdint.h>

#define USER_CONSTRAINS(x,up,down)	    ((x) < (down) ? (down) : ((x) > (up) ? (up) : (x)))
float user_parse_float(const uint8_t *s, uint16_t len);     /* 简易字符串转浮点数 */
int   user_float3(float val, char *buf);                    /* 浮点数转字符串，固定三位小数 */

#endif