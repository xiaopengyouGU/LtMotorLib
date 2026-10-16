#include "math/basic/lt_str.h"

static int _lt_utoa(uint32_t v, char *buf)      /* 无符号整数 → 十进制字符串 */
{
    char tmp[12];
    int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    for (int i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
    return n;
}

int lt_ftoa3(float val, char *buf)              /* 浮点 → 字符串：固定 3 位小数 */
{
    if (!buf) return 0;
    char *p = buf;
    if (val < 0.0f) { *p++ = '-'; val = -val; }
    uint32_t int_part = (uint32_t)val;
    uint32_t frac = (uint32_t)((val - (float)int_part) * 1000.0f + 0.5f);
    if (frac >= 1000) { frac = 0; int_part++; }   /* 进位 */
    p += _lt_utoa(int_part, p);
    *p++ = '.';
    *p++ = (char)('0' + (frac / 100) % 10);
    *p++ = (char)('0' + (frac / 10) % 10);
    *p++ = (char)('0' + frac % 10);
    *p = '\0';
    return (int)(p - buf);
}

float lt_atof(const uint8_t *s, uint16_t len)   /* 字符串 → 浮点（小数用整数累加，保证精度）*/
{
    float sign = 1.0f, val = 0.0f;
    uint16_t i = 0;
    while (i < len && s[i] <= ' ') i++;                        /* 跳过空白 */
    if (i < len && (s[i] == '-' || s[i] == '+')) {
        sign = (s[i] == '-') ? -1.0f : 1.0f;
        i++;
    }
    while (i < len && s[i] >= '0' && s[i] <= '9') val = val * 10.0f + (s[i++] - '0');
    if (i < len && s[i] == '.') {
        i++;
        uint32_t frac = 0, div = 1;
        while (i < len && s[i] >= '0' && s[i] <= '9' && div <= 1000000) {
            frac = frac * 10 + (s[i++] - '0');
            div *= 10;
        }
        val += (float)frac / (float)div;                       /* div 超上限后只跳过剩余位 */
    }
    return sign * val;
}
