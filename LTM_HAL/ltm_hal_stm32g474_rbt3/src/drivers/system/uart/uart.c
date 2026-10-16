/*
 * SPDX-License-Identifier: Apache-2.0
 * Change Logs:
 * Date           Author       Notes
 * 2026-09-01     Lvtou        STM32G474 调试串口 USART3（PC10/PC11，115200）
 *                             中断接收（批量上交）+ 阻塞发送，与 HPM uart.c 同构
 */
#include "system/uart/uart.h"
#include "stm32g4xx_hal.h"

#define UART_DEV            USART3
#define UART_BAUDRATE       115200UL
#define UART_BATCH_MAX      16U         /* 一次中断最多批量上交的字节数 */

static UART_HandleTypeDef g_uart;
static void (*s_rxcall)(uint8_t *buf, uint16_t len) = NULL;

/* USART3 中断：RX 数据可用 -> 批量读数据寄存器交给协议层（中断快进快出） */
void USART3_IRQHandler(void)
{
    uint8_t batch[UART_BATCH_MAX];
    uint16_t n = 0;

    if (USART3->ISR & USART_ISR_ORE) {              /* 溢出保护：清标志防 RX 卡死 */
        USART3->ICR = USART_ICR_ORECF;
    }
    while (n < sizeof(batch) && (USART3->ISR & USART_ISR_RXNE)) {
        batch[n++] = (uint8_t)(USART3->RDR & 0xFF); /* 读 RDR 即清 RXNE */
    }
    if (n && s_rxcall) {
        s_rxcall(batch, n);                         /* 批量交给协议层（环形缓冲区） */
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART_DEV) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;  /* PC10=TX，PC11=RX */
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOC, &gpio);

        HAL_NVIC_SetPriority(USART3_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
}

void uart_port_init(void)
{
    g_uart.Instance          = UART_DEV;
    g_uart.Init.BaudRate     = UART_BAUDRATE;
    g_uart.Init.WordLength   = UART_WORDLENGTH_8B;
    g_uart.Init.StopBits     = UART_STOPBITS_1;
    g_uart.Init.Parity       = UART_PARITY_NONE;
    g_uart.Init.Mode         = UART_MODE_TX_RX;
    g_uart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    g_uart.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&g_uart);
    __HAL_UART_ENABLE_IT(&g_uart, UART_IT_RXNE);    /* 字节级 RX 中断 */
}

void uart_port_write(uint8_t *buf, uint16_t len)    /* 串口写入（阻塞） */
{
    if (!buf || !len)               return;         /* 判空 */
    for (uint16_t i = 0; i < len; i++) {
        while (!(USART3->ISR & USART_ISR_TXE)) { }  /* 等发送寄存器空 */
        USART3->TDR = buf[i];
    }
    while (!(USART3->ISR & USART_ISR_TC)) { }       /* 等最后字节移出 */
}

void uart_port_write_nonblock(uint8_t *buf, uint16_t len)
{
    uart_port_write(buf, len);                      /* 暂与阻塞一致 */
}

void uart_port_set_rxcall(void (*rxcall)(uint8_t *buf, uint16_t len))
{
    s_rxcall = rxcall;
}
