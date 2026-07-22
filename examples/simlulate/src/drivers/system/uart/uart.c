#include "system/uart/uart.h"
#include <windows.h>
#include <string.h>

static HANDLE hCom = INVALID_HANDLE_VALUE;
static HANDLE hThread = NULL;
static void (*rx_call)(uint8_t* buf, uint16_t len) = NULL;

/* 串口数据接收线程 */
static DWORD WINAPI thread_proc(LPVOID arg) {
    (void)arg;
    uint8_t buf[256];
    DWORD read_len;
    OVERLAPPED ov = {0};
    ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);   /* 事件驱动，减少CPU占用 */
    if (!ov.hEvent) return 0;
    
    /* 启动异步读 */
    ReadFile(hCom, buf, sizeof(buf), &read_len, &ov);
    
    while (1) {
        WaitForSingleObject(ov.hEvent, INFINITE);
        
        if (GetOverlappedResult(hCom, &ov, &read_len, FALSE)) {
            if (read_len > 0 && rx_call) {
                rx_call(buf, (uint16_t)read_len);
            }
        }
        
        /* 复位事件，继续读 */
        ResetEvent(ov.hEvent);
        ReadFile(hCom, buf, sizeof(buf), &read_len, &ov);
    }
    
    CloseHandle(ov.hEvent);
    return 0;
}

void uart_init(void) {
    if (hCom != INVALID_HANDLE_VALUE) return;
    
    hCom = CreateFileA(COM_NAME, 
                    GENERIC_READ | GENERIC_WRITE, 
                    0, NULL, 
                    OPEN_EXISTING, 
                    0,                    /* 同步模式 */
                    NULL);
    if (hCom == INVALID_HANDLE_VALUE){
        DWORD err = GetLastError();
        printf("错误码: %d\n", err);
        return;
    }
    
    DCB dcb = {0};
    dcb.DCBlength = sizeof(DCB);
    GetCommState(hCom, &dcb);
    dcb.BaudRate = CBR_115200;                     /* 波特率：115200 */
    dcb.ByteSize = 8;                              /* 8 数据位 */
    dcb.StopBits = ONESTOPBIT;                     /* 1 停止位 */
    dcb.Parity = NOPARITY;                         /* 无校验位 */
    SetCommState(hCom, &dcb);
    
    COMMTIMEOUTS to = {0};
    to.ReadIntervalTimeout = MAXDWORD;
    SetCommTimeouts(hCom, &to);
    
    SetupComm(hCom, 2048, 2048);
    PurgeComm(hCom, PURGE_RXCLEAR | PURGE_TXCLEAR);
    /* 创建串口接收线程, 立即运行 */
    hThread = CreateThread(NULL, 2048, thread_proc, NULL,  0, NULL);
}

void uart_set_rxcall(void(*rxcall)(uint8_t* buf, uint16_t len)) {
    if(!rxcall)            return;         /* 判空 */
    rx_call = rxcall;
}

void uart_write(uint8_t* buf, uint16_t len) {
    if (hCom == INVALID_HANDLE_VALUE || !buf || !len) return;
    
    DWORD written;
    WriteFile(hCom, buf, len, &written, NULL);      /* 同步写入，即阻塞 */
}

void uart_read(uint8_t* buf, uint16_t *max_len) {
    if (hCom == INVALID_HANDLE_VALUE || !buf) return;
    
    DWORD read_len;
    if (ReadFile(hCom, buf, 128, &read_len, NULL)) {
        *max_len = (uint16_t)read_len;
    }else{
        *max_len = 0; 
    }
}






