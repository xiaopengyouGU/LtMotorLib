#include "protocol/ltm_commut.h"
#include "protocol/protocol.h"
#include <string.h>
#include <stdarg.h>

#define LTM_BUF_SIZE          PROTOCOL_DATA_SIZE  /* 收发缓冲区大小 */

/************************* LTM_Monitor 通讯协议  ***********************************/
typedef struct{
    uint8_t init;               /* 初始化标志，0：未初始化，1：初始化完毕 */
    uint8_t rx_buf[LTM_BUF_SIZE]; /* 接收缓冲区，单次最多 BUF_SIZE 字节数据 */
    uint8_t data_type;          /* 接收到的数据类型 */
    uint16_t rx_len;            /* 接收到的数据长度 */
    void (*send)(uint8_t *datas, uint16_t len);  /* 底层发送接口 */
} ltm_commut_obj;

/* 静态实例 */
static ltm_commut_obj commut_obj;
static ltm_commut_obj *commut = &commut_obj; 

/* 内部发送：打包并调用底层发送 */
static void my_send(uint8_t data_type, uint8_t *datas, uint16_t len);
/* 无符号整数 → 十进制字符串（不依赖任何库，替代 sprintf 的 %ld 路径）*/
static int lt_utoa(uint32_t v, char *buf);
static int ltm_vfmt(char *out, int size, const char *fmt, va_list ap);  /* 轻量格式化 */

/*==============================================================================
 * 公共 API 实现
 *==============================================================================*/

void ltm_commut_init(void)
{
    protocol_init(0);                       /* 初始化协议对象 */
    memset(commut, 0, sizeof(ltm_commut_obj));
    commut->init = 1;
}

void ltm_commut_set_send(void (*send)(uint8_t *datas, uint16_t len))	
{
	if(!commut->init)		return;					/* 协议对象未初始化，直接返回 */
	if(!send)				return; 				/* 判空 */
	commut->send = send;							/* 设置底层发送接口 */
}

void ltm_commut_recv(uint8_t *datas, uint16_t len)	/* 数据接收接口：封装 protocol_recv */
{
	if(!commut->init)		return;					/* 协议对象未初始化，直接返回 */
	protocol_recv(datas, len);						/* 将数据写入 protocol 环形缓冲区 */
}

void ltm_commut_send(uint8_t data_type, uint8_t *datas, uint16_t len)
{
    my_send(data_type, datas, len);
}

void ltm_commut_printf(const char *fmt, ...)
{
    if (!commut->init) 	return;
    va_list args;
    va_start(args, fmt);
    static char buf[LTM_BUF_SIZE];
    int len = ltm_vfmt(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        if (len >= (int)sizeof(buf)) len = sizeof(buf) - 1;
        my_send(Data_CMD_Text, (uint8_t *)buf, len);
    }
}

void ltm_commut_send_curves(ltm_curves * curves){
	if(!commut->init || !curves) return;
	uint8_t size = curves->size;
	if(size == 0 || size > LTM_CURVE_SIZE) return;
	my_send(Data_Channel_ALL, (uint8_t *)&(curves->values), size * sizeof(float));
}

bool ltm_commut_process(uint8_t *data_type, uint8_t* datas, uint16_t *len)
{
	uint8_t *_buf = commut->rx_buf;
	uint8_t  _type = 0xFF;			        /* 未知数据,说明接收错误或数据不完整 */
	uint16_t _len;

	if(!protocol_process(&_type, _buf, &_len))	return false;/* 未接收到完整的数据帧 */
	commut->rx_len = _len;
	commut->data_type = _type;

	/* 输出协议解析结果 */
	*data_type = _type;
	*len       = _len;
	memcpy(datas, _buf, _len);				/* 将数据拷贝到用户的缓冲区中 */

	return true;
}

/*****************************************************************/
/* 内部发送：打包并调用底层发送 */
static void my_send(uint8_t data_type, uint8_t *datas, uint16_t len)
{
    if (!commut->init) return;
    protocol_package(data_type, (uint8_t *)datas, len);     /* 数据打包 */
    uint16_t out_len;
    const uint8_t *out = protocol_datas(&out_len);          /* 获取打包后的数据 */
    if (out && out_len && commut->send) {
        commut->send((uint8_t *)out, out_len);              /* 调用底层发送接口 */
    }
}

/* 无符号整数 → 十进制字符串（组件内部实现，ltm_commut 不依赖外部 util）*/
static int lt_utoa(uint32_t v, char *buf)
{
    char tmp[12];
    int n = 0;
    do { tmp[n++] = (char)('0' + v % 10); v /= 10; } while (v);
    for (int i = 0; i < n; i++) buf[i] = tmp[n - 1 - i];
    buf[n] = '\0';
    return n;
}

/* 轻量格式化：支持 %s/%d/%u/%x/%X/%c/%% 及宽度/0填充/左对齐；
 * 浮点先经 LT_FTOAT3 转字符串再按 %s 输出。
 * 替代 vsnprintf，避免链入 newlib printf/浮点库（省 ~35KB）*/
static int ltm_vfmt(char *out, int size, const char *fmt, va_list ap)
{
    int n = 0;
    while (*fmt && n < size - 1) {
        if (*fmt != '%') { out[n++] = *fmt++; continue; }
        fmt++;
        int left = 0, width = 0;
        char pad = ' ';
        if (*fmt == '-') { left = 1; fmt++; }
        if (*fmt == '0') { pad = '0'; fmt++; }
        while (*fmt >= '0' && *fmt <= '9') { width = width * 10 + (*fmt - '0'); fmt++; }
        char tmp[16];
        const char *s = tmp;
        int len = 0;
        switch (*fmt) {
        case 's':
            s = va_arg(ap, const char *);
            if (!s) s = "(null)";
            len = 0; while (s[len]) len++;
            break;
        case 'c':
            tmp[0] = (char)va_arg(ap, int); tmp[1] = '\0'; len = 1;
            break;
        case 'd': {
            int32_t v = va_arg(ap, int32_t);
            if (v < 0) { tmp[0] = '-'; lt_utoa((uint32_t)(-(int64_t)v), tmp + 1); }
            else       lt_utoa((uint32_t)v, tmp);
            len = 0; while (tmp[len]) len++;
            break; }
        case 'u':
            lt_utoa(va_arg(ap, uint32_t), tmp);
            len = 0; while (tmp[len]) len++;
            break;
        case 'x':
        case 'X': {
            uint32_t v = va_arg(ap, uint32_t);
            int i = 0;
            int up = (*fmt == 'X');
            do {
                uint32_t d = v & 0xF;
                tmp[i++] = (char)(d < 10 ? '0' + d : (up ? 'A' : 'a') - 10 + d);
                v >>= 4;
            } while (v);
            for (int j = 0; j < i / 2; j++) { char t = tmp[j]; tmp[j] = tmp[i - 1 - j]; tmp[i - 1 - j] = t; }
            tmp[i] = '\0';
            len = i;
            break; }
        case '%':
            out[n++] = '%'; fmt++; continue;
        default:
            out[n++] = '%'; continue;
        }
        fmt++;
        int padn = width - len;
        if (padn < 0) padn = 0;
        if (!left) while (padn-- > 0 && n < size - 1) out[n++] = pad;
        for (int i = 0; i < len && n < size - 1; i++) out[n++] = s[i];
        if (left)  while (padn-- > 0 && n < size - 1) out[n++] = ' ';
    }
    out[n] = '\0';
    return n;
}
