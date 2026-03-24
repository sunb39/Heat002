/*
 * FreeModbus Libary: BARE Port -> STM32F103 + HAL + TIM2
 */

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "main.h"
#include "tim.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- Static variables ---------------------------------*/
static USHORT usTimerReloadValue = 0;

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR(void);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortTimersInit(USHORT usTim1Timerout50us)
{
    /* TIM2 已经由 CubeMX 配置成 1us/tick
       FreeModbus 参数单位是 50us，所以换算成 us */
    usTimerReloadValue = usTim1Timerout50us * 50;

    /* 先停止定时器 */
    HAL_TIM_Base_Stop_IT(&htim2);

    /* 清零计数器 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);

    /* 设置自动重装值
       因为计数从 0 到 ARR，所以要减 1 */
    __HAL_TIM_SET_AUTORELOAD(&htim2, usTimerReloadValue - 1);

    return TRUE;
}

void vMBPortTimersEnable(void)
{
    /* 重新装载并启动 TIM2 */
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim2, usTimerReloadValue - 1);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
    HAL_TIM_Base_Start_IT(&htim2);
}

void vMBPortTimersDisable(void)
{
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
}

/* 定时器超时中断处理 */
static void prvvTIMERExpiredISR(void)
{
    (void)pxMBPortCBTimerExpired();
}

/* 供 stm32f1xx_it.c 调用的中断分发函数 */
void xMBPortTimer_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_UPDATE) != RESET)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);

            /* 单次定时，超时后先停表 */
            HAL_TIM_Base_Stop_IT(&htim2);

            prvvTIMERExpiredISR();
        }
    }
}
