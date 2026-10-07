#include "system/uart/uart.h"
#include "system/sys/sys.h"
#include "hal_data.h"
#include <string.h>

#define UART_TX_BUF_SIZE  (128 + 6)  /* 最大单次发送数据 + 固定帧结构 */
static uart_ctrl_t * uart_ctrl = &g_uart0_ctrl;
static uart_cfg_t  * uart_cfg  = &g_uart0_cfg;
static uint8_t uart_buf[UART_TX_BUF_SIZE]; 

static void (*_uart_rxcall)(uint8_t*buf, uint16_t len)  = NULL;
/* 发送完成标志, 阻塞发送时也需要， 否则发送数据可能出现错误 */
volatile uint8_t uart_send_cplt = 1;

/* 调试串口 UART 初始化 */
void uart_init(void)
{
    fsp_err_t err = FSP_SUCCESS;
    
    err = R_SCI_UART_Open (uart_ctrl, uart_cfg);
    uart_send_cplt = 1;
    
    assert(FSP_SUCCESS == err);
}

void uart_set_rxcall(void(*rxcall)(uint8_t* buf, uint16_t len))     /* 设置接收回调函数 */
{
    if(!rxcall)     return;     /* 判空 */
    _uart_rxcall = rxcall;
}

void uart_write(uint8_t* buf, uint16_t len)        /* 串口写入 */
{
    if(!buf || !len)            return;            /* 判空 */
    /* 把数据拷贝到缓冲区, 阻塞发送 */
    int64_t start = (int64_t)system_get_ms();                            
    while(!uart_send_cplt){
        int64_t time = (int64_t)system_get_ms() - start;
        if(time >= 50)   break;                   /* 50ms后，自动退出 */
    }                                              /* 必须发送完后，才能再次发送 */
    uart_send_cplt = 0;                            /* 调用该接口，标志位立即置零，等待发送完毕 */
    
    len = (len < UART_TX_BUF_SIZE) ? len : UART_TX_BUF_SIZE;        
    memcpy(uart_buf, buf, len);

	R_SCI_UART_Write(uart_ctrl, uart_buf, len);			
}

void uart_write_nonblock(uint8_t* buf, uint16_t len)  /* 串口发送，非阻塞 */
{
    if(!buf || !len)            return;            /* 判空 */
    /* 把数据拷贝到缓冲区, 非阻塞发送（仅第一个字节阻塞发送，后续均中断发送） */
    if(!uart_send_cplt)         return;            /* 数据发送中，直接返回 */
    uart_send_cplt = 0;
    
    len = (len < UART_TX_BUF_SIZE) ? len : UART_TX_BUF_SIZE;        
    memcpy(uart_buf, buf, len);

    R_SCI_UART_Write(uart_ctrl, uart_buf, len);		
}

/* 串口中断回调 */
void uart_callback (uart_callback_args_t * p_args)
{
    switch (p_args->event)
    {
        case UART_EVENT_RX_CHAR:
        {
			/* 串口数据接收 */
            uint8_t *data = (uint8_t *)&(p_args->data);;
            if(_uart_rxcall)    _uart_rxcall(data, 1);
            break;
        }
        case UART_EVENT_TX_COMPLETE:
        {   /* 串口数据发送完毕，修改标志位 */
            uart_send_cplt = 1;
        }
        default:
            break;
    }
}

/* Suppress "unused parameter" warnings for stub functions */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/* Syscall stub function declarations */
int _close(int fd);
int _lseek(int fd, int offset, int whence);
int _read(int fd, char *buf, int count);
int _isatty(int fd);

/**
 * @brief  Close a file descriptor (stub).
 */
int _close(int fd)
{
    return -1;
}

/**
 * @brief  Seek within a file (stub).
 */
int _lseek(int fd, int offset, int whence)
{
    return -1;
}

/**
 * @brief  Read from a file descriptor (stub).
 */
int _read(int fd, char *buf, int count)
{
    return -1;
}

/**
 * @brief  Check if a file descriptor refers to a terminal (stub).
 */
int _isatty(int fd)
{
    return 1;
}

#pragma GCC diagnostic pop

/* 重定向 printf */
int _write(int fd, char *pBuffer, int size); //防止编译警告
int _write(int fd, char *pBuffer, int size)
{
    (void)fd;
    uart_write((uint8_t *)pBuffer, (uint16_t)size);
    return size;
}





