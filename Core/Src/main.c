/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mb.h"
#include "m1820.h"
#include "mbport.h"
#include "modbus_regs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t g_surf_raw_buf[9] = {0};
uint8_t g_env_raw_buf[9]  = {0};

uint16_t g_surf_debug_enter = 0;   /* 第二路代码进入次数 */
uint16_t g_surf_debug_ok    = 0;   /* 第二路读取成功次数 */
uint16_t g_env_debug_enter  = 0;   /* 第一路代码进入次数 */
uint16_t g_env_debug_ok     = 0;   /* 第一路读取成功次数 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* 浮点限幅函数 */
static float LimitFloat(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }
    if (value > max_val)
    {
        return max_val;
    }
    return value;
}

/* =========================================================
 * 设置加热 PWM 占空比
 * duty_percent（PWM占空比百分比）范围：0~100
 *
 * 当前硬件：
 * - TIM4（定时器4）
 * - TIM_CHANNEL_1（定时器4通道1）
 * - PB6（PWM输出脚）
 *
 * 当前 TIM4 配置：
 * - ARR（自动重装值）= 999
 * 所以比较值 CCR 范围大致也是 0~999
 * ========================================================= */
static void Heat_SetPwmDuty(float duty_percent)
{
    uint32_t arr;
    uint32_t compare;

    duty_percent = LimitFloat(duty_percent, 0.0f, 100.0f);

    /* 读取 TIM4 的 ARR（自动重装值） */
    arr = __HAL_TIM_GET_AUTORELOAD(&htim4);

    if (duty_percent <= 0.0f)
    {
        compare = 0;
    }
    else if (duty_percent >= 100.0f)
    {
        compare = arr;
    }
    else
    {
        compare = (uint32_t)((duty_percent * (float)(arr + 1U)) / 100.0f);

        if (compare > arr)
        {
            compare = arr;
        }
    }

    /* 把比较值写入 TIM4_CH1（定时器4通道1） */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, compare);
}

/* =========================================================
 * 控制逻辑任务
 * 说明：
 * 1. 控制模式判断前置
 * 2. 自动模式下才严格依赖探头异常联锁
 * 3. 强制全功率 / 手动PWM 模式下，探头异常仅上报，不强制关断
 * 4. 本版已经把 g_pwm_duty_real（当前实际PWM占空比）
 *    正式下发到 TIM4_CH1（定时器4通道1）硬件输出
 * ========================================================= */
static void Heating_ControlTask(void)
{
    float pwm_cmd = 0.0f;
    float error = 0.0f;

    uint8_t full_power_stage = 0;
    uint8_t regulation_stage = 0;
    uint8_t temp_valid = 0;
    uint8_t output_enable = 0;

    /* -------------------------
     * 1. 先根据当前探头状态判断温度数据是否有效
     * ------------------------- */
    if ((g_env_probe_status == 0U) && (g_surf_probe_status == 0U))
    {
        temp_valid = 1;
    }
    else
    {
        temp_valid = 0;
    }

    /* -------------------------
     * 2. 控制模式判断
     * ------------------------- */
    switch (g_ctrl_mode_set)
    {
        case CTRL_MODE_OFF:   /* 关闭模式 */
        {
            pwm_cmd = 0.0f;
            output_enable = 0;
            break;
        }

        case CTRL_MODE_FORCE_FULL:   /* 强制全功率模式 */
        {
            pwm_cmd = 100.0f;
            output_enable = 1;
            /* 这里只是强制全功率模式，不是自动模式的全功率阶段 */
            break;
        }

        case CTRL_MODE_MANUAL_PWM:   /* 手动PWM模式 */
        {
            pwm_cmd = LimitFloat(g_manual_pwm_set, 0.0f, 100.0f);
            output_enable = (pwm_cmd > 0.0f) ? 1U : 0U;
            break;
        }

        case CTRL_MODE_AUTO:   /* 自动控制模式 */
        default:
        {
            /* 自动模式下，探头异常才参与联锁 */
            if ((g_env_probe_status != 0U) || (g_surf_probe_status != 0U))
            {
                pwm_cmd = 0.0f;
                output_enable = 0;
            }
            else
            {
                /* 环境温度高于等于阈值：不加热 */
                if (g_env_temp_real >= g_env_temp_threshold)
                {
                    pwm_cmd = 0.0f;
                    output_enable = 0;
                }
                else
                {
                    /* 环境温度低于阈值，进入表面温度闭环前的简化控制 */
                    error = g_surf_temp_target - g_surf_temp_real;

                    if (error >= 5.0f)
                    {
                        /* 远低于目标值：全功率快速加热 */
                        pwm_cmd = 100.0f;
                        output_enable = 1;
                        full_power_stage = 1;
                    }
                    else if (error > 0.0f)
                    {
                        /* 接近目标值：简化比例调功 */
                        pwm_cmd =  5.0f + error * 15.0f;   /* 例如差 3℃ -> 30% */
                        pwm_cmd = LimitFloat(pwm_cmd, 0.0f, 100.0f);
                        output_enable = (pwm_cmd > 0.0f) ? 1U : 0U;
                        regulation_stage = (pwm_cmd > 0.0f) ? 1U : 0U;
                    }
                    else
                    {
                        /* 已达到或超过目标值 */
                        pwm_cmd = 0.0f;
                        output_enable = 0;
                    }
                }
            }
            break;
        }
    }

    /* -------------------------
     * 3. 更新运行时实时变量
     * ------------------------- */
    g_pwm_duty_real = LimitFloat(pwm_cmd, 0.0f, 100.0f);
    g_heat_output_status = (g_pwm_duty_real > 0.0f) ? 1U : 0U;

    /* 当前先按 12V 满量程线性估算输出电压 */
    g_heat_out_volt = 12.0f * g_pwm_duty_real / 100.0f;

    /* -------------------------
     * 4. 更新故障字
     * ------------------------- */
    g_fault_word = 0;

    if (g_temp_alarm_en)
    {
        if (g_env_probe_status != 0U)
        {
            g_fault_word |= (1U << 0);   /* ENV_SENSOR_FAULT（环境探头故障） */
        }

        if (g_surf_probe_status != 0U)
        {
            g_fault_word |= (1U << 1);   /* SURFACE_SENSOR_FAULT（表面探头故障） */
        }

        if (!temp_valid)
        {
            g_fault_word |= (1U << 6);   /* TEMP_DATA_INVALID（温度数据无效） */
        }
    }

    /* -------------------------
     * 5. 更新状态字
     * ------------------------- */
    g_status_word = 0;

    g_status_word |= (1U << 0);   /* SYS_RUN（系统运行） */
    g_status_word |= (1U << 6);   /* COMM_OK（通信正常） */

    if (g_ctrl_mode_set == CTRL_MODE_AUTO)
    {
        g_status_word |= (1U << 1);   /* AUTO_MODE（自动控制模式） */
    }

    if (g_ctrl_mode_set == CTRL_MODE_MANUAL_PWM)
    {
        g_status_word |= (1U << 2);   /* MANUAL_PWM_MODE（手动PWM模式） */
    }

    if (full_power_stage)
    {
        g_status_word |= (1U << 3);   /* FULL_POWER_STAGE（全功率阶段） */
    }

    if (regulation_stage)
    {
        g_status_word |= (1U << 4);   /* REGULATION_STAGE（调功阶段） */
    }

    if (g_heat_output_status)
    {
        g_status_word |= (1U << 5);   /* HEAT_OUT（加热输出） */
    }

    if (temp_valid)
    {
        g_status_word |= (1U << 7);   /* TEMP_VALID（温度数据有效） */
    }

    if (g_env_probe_status == 0U)
    {
        g_status_word |= (1U << 8);   /* ENV_SENSOR_ONLINE（环境探头在线） */
    }

    if (g_surf_probe_status == 0U)
    {
        g_status_word |= (1U << 9);   /* SURFACE_SENSOR_ONLINE（表面探头在线） */
    }

    if (output_enable)
    {
        g_status_word |= (1U << 11);  /* OUTPUT_ENABLE（输出允许） */
    }

    /* -------------------------
     * 6. 把 PWM 真正下发到 TIM4_CH1 硬件输出
     * ------------------------- */
    Heat_SetPwmDuty(g_pwm_duty_real);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 启动 TIM4_CH1（定时器4通道1）PWM 输出 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  /* 上电先关输出，避免一启动就误输出 */
  Heat_SetPwmDuty(0.0f);

  Modbus_RegsInit();
  M1820_Init();

  if (eMBInit(MB_RTU, 0x0C, 0, 9600, MB_PAR_NONE) == MB_ENOERR)
  {
      eMBEnable();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    static uint32_t wd_tick = 0;
    static uint32_t temp_tick = 0;

    static uint16_t env_fail_streak = 0;   /* 环境探头连续失败次数 */
    static uint16_t surf_fail_streak = 0;  /* 表面探头连续失败次数 */

    float env_temp_value  = 0.0f;
    float surf_temp_value = 0.0f;

    uint8_t env_ok = 0;
    uint8_t surf_ok = 0;

    /* 处理 Modbus 协议轮询 */
    eMBPoll();

    /* 处理 Modbus 命令寄存器 */
    Modbus_CmdProcess();

    /* 每 1000ms 采集一次两路温度 */
    if (HAL_GetTick() - temp_tick >= 1000)
    {
        temp_tick = HAL_GetTick();

        /* =====================================================
         * 1. 读取环境温度（PA8，对应 M1820 格式）
         * ===================================================== */
        g_env_debug_enter++;

        env_ok = M1820_ReadTemperature(&g_m1820_env, &env_temp_value);

        if (!env_ok)
        {
            HAL_Delay(10);
            env_ok = M1820_ReadTemperature(&g_m1820_env, &env_temp_value);
        }

        if (env_ok)
        {
            g_env_debug_ok++;

            g_env_temp_real = env_temp_value;
            g_env_probe_status = 0;
            env_fail_streak = 0;
        }
        else
        {
            if (g_env_probe_err_cnt < 65535)
            {
                g_env_probe_err_cnt++;
            }

            if (env_fail_streak < 65535)
            {
                env_fail_streak++;
            }

            if (env_fail_streak >= 5)
            {
                g_env_probe_status = 1;
            }
        }

        /* 两路探头之间留一点时间间隔 */
        HAL_Delay(20);

        /* =====================================================
         * 2. 读取表面温度（PB2，对应 DS18B20 兼容格式）
         * ===================================================== */
        g_surf_debug_enter++;

        surf_ok = M1820_ReadTemperature(&g_m1820_surf, &surf_temp_value);

        if (!surf_ok)
        {
            HAL_Delay(10);
            surf_ok = M1820_ReadTemperature(&g_m1820_surf, &surf_temp_value);
        }

        if (surf_ok)
        {
            g_surf_debug_ok++;

            g_surf_temp_real = surf_temp_value;
            g_surf_probe_status = 0;
            surf_fail_streak = 0;
        }
        else
        {
            if (g_surf_probe_err_cnt < 65535)
            {
                g_surf_probe_err_cnt++;
            }

            if (surf_fail_streak < 65535)
            {
                surf_fail_streak++;
            }

            if (surf_fail_streak >= 5)
            {
                g_surf_probe_status = 1;
            }
        }

        /* =====================================================
         * 3. 温度采样完成后，执行一次控制逻辑
         * ===================================================== */
        Heating_ControlTask();
    }

    /* =====================================================
     * 非阻塞喂狗
     * 每 100ms 翻转一次 PB12
     * ===================================================== */
    if (HAL_GetTick() - wd_tick >= 100)
    {
        wd_tick = HAL_GetTick();
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    }
}
  
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
//13213131