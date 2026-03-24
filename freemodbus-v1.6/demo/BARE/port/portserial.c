/*
 * FreeModbus Libary: BARE Port -> STM32F103 + HAL + USART1 + RS485
 */

#include "port.h"
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
static volatile UCHAR ucUartRxByte;
static volatile BOOL xTxBusy = FALSE;

/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR(void);
static void prvvUARTRxISR(void);

void xMBPortSerial_IRQHandler(void);

/* ----------------------- Start implementation -----------------------------*/
void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    if (xTxEnable)
    {
        /* 进入发送模式 */
        xTxBusy = TRUE;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);   // PB1=1 发送

        /* 稍微等待485方向切换稳定 */
        for (volatile int i = 0; i < 2000; i++);

        /* 关闭接收中断，开启发送中断 */
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    }
    else
    {
        /* 关闭发送中断 */
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);

        /* 如果前面处于发送状态，要等最后一个字节真正发完 */
        if (xTxBusy)
        {
            while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET);
            xTxBusy = FALSE;
        }

        /* 切回接收模式 */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // PB1=0 接收

        if (xRxEnable)
        {
            __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
        }
        else
        {
            __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
        }
    }
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
    (void)ucPORT;
    (void)ulBaudRate;
    (void)ucDataBits;
    (void)eParity;

    /* USART1 已由 CubeMX 初始化为 9600 8N1 */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // 默认接收

    xTxBusy = FALSE;

    __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);

    return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    huart1.Instance->DR = (uint8_t)ucByte;
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR *pucByte)
{
    *pucByte = (CHAR)ucUartRxByte;
    return TRUE;
}

/* 串口发送寄存器空中断处理 */
static void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* 串口接收中断处理 */
static void prvvUARTRxISR(void)
{
    ucUartRxByte = (UCHAR)(huart1.Instance->DR & 0x00FF);
    pxMBFrameCBByteReceived();
}

/* 供 stm32f1xx_it.c 调用的中断分发函数 */
void xMBPortSerial_IRQHandler(void)
{
    /* 接收中断 */
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE) != RESET))
    {
        prvvUARTRxISR();
    }

    /* 发送寄存器空中断 */
    if ((__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE) != RESET) &&
        (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) != RESET))
    {
        prvvUARTTxReadyISR();
    }
}
