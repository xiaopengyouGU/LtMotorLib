#ifndef LT_STR_H
#define LT_STR_H

#include <stdint.h>

/*---------------------- 浮点 ↔ 字符串（纯整数实现） ----------------------*/
/* 不依赖标准库浮点 printf，也不链 sprintf：固定 3 位小数，带四舍五入与进位。
 * 这是全仓库唯一的浮点格式化实现，例程与组件都用它。 */
int   lt_ftoa3(float val, char *buf);              /* 浮点 → 字符串（固定 3 位小数）*/
float lt_atof(const uint8_t *s, uint16_t len);     /* 字符串 → 浮点（简易解析）*/

#define LT_FTOAT3(buf, val)   lt_ftoa3((val), (buf))   /* 旧调用点兼容别名 */

#endif /* LT_STR_H */
