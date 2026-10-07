#include "user_common.h"
#include <stdlib.h>
/* ========== 无符号整数 → 十进制字符串 ========== */
static inline int user_utoa(uint32_t v, char *buf)
{
    char tmp[12];
    int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    for (int i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
    return n;
}


float user_parse_float(const uint8_t *s, uint16_t len)     /* 简易字符串转浮点数 */
{
    float sign = 1.0f, val = 0.0f;
    uint16_t i = 0;
    /* 跳过空白 */
    while (i < len && s[i] <= ' ') i++;
    if (i < len && (s[i] == '-' || s[i] == '+')) {         /* 符号处理 */
        sign = (s[i] == '-') ? -1.0f : 1.0f;
        i++;
    }
   
    while (i < len && s[i] >= '0' && s[i] <= '9') {         /* 整数部分 */
        val = val * 10.0f + (s[i++] - '0');
    }
    
    if (i < len && s[i] == '.') {                           /* 小数部分，整数运算，保证精度 */
        i++;
        uint32_t frac = 0, div = 1;
        while (i < len && s[i] >= '0' && s[i] <= '9' && div <= 1000000) {
            frac = frac * 10 + (s[i++] - '0');
            div *= 10;
        }
        val += (float)frac / (float)div;                   /* div超过1000000后，只跳过剩余数字，不累加 */
    }
    
    return sign * val;
}

int user_float3(float val, char *buf)                    /* 浮点数转字符串，固定三位小数 */
{
    if (buf == NULL) return 0;
    /* 1. 处理符号 */
    char *p = buf;
    if (val < 0.0f) {
        *p++ = '-';
        val = -val;
    }
    /* 2. 分离整数和小数（带四舍五入）*/
    uint32_t int_part = (uint32_t)val;
    uint32_t frac = (uint32_t)((val - int_part) * 1000.0f + 0.5f);
    /* 3. 小数进位处理 */
    if (frac >= 1000) {
        frac = 0;
        int_part++;
    }
    /* 4. 输出整数部分 */
    p += user_utoa(int_part, p);
    /* 5. 输出小数部分（固定3位）*/
    *p++ = '.';
    *p++ = (char)('0' + (frac / 100) % 10);
    *p++ = (char)('0' + (frac / 10) % 10);
    *p++ = (char)('0' + frac % 10);
    *p = '\0';

    return (int)(p - buf);
}