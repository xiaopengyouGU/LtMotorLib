#ifndef LT_STR_H
#define LT_STR_H

#include <stdio.h>

/* 简易浮点转字符串：仅用整数格式化（%ld），不依赖标准库浮点 printf。
 * 链接时无需 -u _printf_float，可省数 KB 代码。固定 3 位小数，带四舍五入与进位。 */
#define LT_FTOAT3(buf, val)                                         \
    do {                                                            \
        float _v = (val);                                           \
        long  _ip = (long)_v;                                       \
        long  _fp = (long)(((_v < 0.0f ? -_v : _v) - (_ip < 0 ? -_ip : _ip)) \
                           * 1000.0f + 0.5f);                       \
        if (_fp >= 1000) { _fp = 0; _ip += (_v < 0 ? -1 : 1); }     \
        if (_ip < 0) sprintf((buf), "-%ld.%03ld", -_ip, _fp);       \
        else         sprintf((buf), "%ld.%03ld", _ip, _fp);         \
    } while (0)

#endif /* LT_STR_H */
