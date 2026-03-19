#include "m1820.h"

/* =========================================================
 * 两路探头对象定义
 *
 * 当前硬件连接：
 * 1) PA8 -> DQ -> 环境温度探头
 *    该探头按 M1820 温度格式解析
 *
 * 2) PB2 -> DS -> 表面温度探头
 *    该探头按 DS18B20/兼容格式解析
 *
 * 说明：
 * 两路数据线现在都已经补了外部上拉电阻，
 * 因此软件统一按“外部上拉正式版”处理：
 * - 拉低总线：开漏输出
 * - 释放总线：输入浮空
 * ========================================================= */
M1820_Device_t g_m1820_env  = {GPIOA, GPIO_PIN_8, SENSOR_FMT_M1820};
M1820_Device_t g_m1820_surf = {GPIOB, GPIO_PIN_2, SENSOR_FMT_DS18B20};

/* =========================================================
 * 1. 微秒延时
 *
 * 使用 DWT CYCCNT 做微秒级延时，
 * 适合单总线时序控制。
 * ========================================================= */

/* 初始化 DWT 周期计数器 */
static void M1820_DelayUsInit(void)
{
    /* 使能 DWT */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 清零计数器 */
    DWT->CYCCNT = 0;

    /* 开启计数 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* 微秒延时 */
static void M1820_DelayUs(uint32_t us)
{
    uint32_t start_tick = DWT->CYCCNT;
    uint32_t delay_tick = us * (SystemCoreClock / 1000000U);

    while ((DWT->CYCCNT - start_tick) < delay_tick)
    {
        /* 等待 */
    }
}

/* =========================================================
 * 2. GPIO 底层操作
 *
 * 现在两路都已经有外部上拉电阻，因此采用标准单总线方式：
 *
 * - 拉低总线：GPIO 配成开漏输出，并输出低电平
 * - 释放总线：GPIO 配成输入浮空，不再依赖内部上拉
 *
 * 注意：
 * 单总线高电平不是 MCU 主动输出的，而是由外部上拉提供。
 * ========================================================= */

/* 主机主动拉低总线 */
static void M1820_LineDriveLow(M1820_Device_t *dev)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin   = dev->pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;   /* 开漏输出 */
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(dev->port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(dev->port, dev->pin, GPIO_PIN_RESET);
}

/* 主机释放总线，让外部上拉把总线拉高 */
static void M1820_LineRelease(M1820_Device_t *dev)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin  = dev->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;   /* 现在用外部上拉，因此这里不要内部上拉 */
    HAL_GPIO_Init(dev->port, &GPIO_InitStruct);
}

/* 读取当前总线电平 */
static uint8_t M1820_LineRead(M1820_Device_t *dev)
{
    return (uint8_t)HAL_GPIO_ReadPin(dev->port, dev->pin);
}

/* =========================================================
 * 3. 单总线基础协议
 * ========================================================= */

/* 复位并检测存在脉冲
 * 返回：
 * 1 = 检测到存在脉冲
 * 0 = 未检测到
 */
static uint8_t M1820_ResetAndPresence(M1820_Device_t *dev)
{
    uint8_t presence = 0;

    /* 1. 主机拉低总线发送复位脉冲 */
    M1820_LineDriveLow(dev);
    M1820_DelayUs(500);      /* >= 480us */

    /* 2. 主机释放总线 */
    M1820_LineRelease(dev);
    M1820_DelayUs(70);       /* 等待从机响应 */

    /* 3. 检测存在脉冲 */
    if (M1820_LineRead(dev) == 0)
    {
        presence = 1;
    }
    else
    {
        presence = 0;
    }

    /* 4. 等待本轮复位结束 */
    M1820_DelayUs(430);

    return presence;
}

/* 写 1 bit */
static void M1820_WriteBit1(M1820_Device_t *dev)
{
    M1820_LineDriveLow(dev);
    M1820_DelayUs(5);

    M1820_LineRelease(dev);
    M1820_DelayUs(60);
}

/* 写 0 bit */
static void M1820_WriteBit0(M1820_Device_t *dev)
{
    M1820_LineDriveLow(dev);
    M1820_DelayUs(60);

    M1820_LineRelease(dev);
    M1820_DelayUs(5);
}

/* 写 1 个 bit */
static void M1820_WriteBit(M1820_Device_t *dev, uint8_t bit_value)
{
    if (bit_value)
    {
        M1820_WriteBit1(dev);
    }
    else
    {
        M1820_WriteBit0(dev);
    }
}

/* 读 1 个 bit */
static uint8_t M1820_ReadBit(M1820_Device_t *dev)
{
    uint8_t bit_value;

    /* 主机先短暂拉低，启动读时隙 */
    M1820_LineDriveLow(dev);
    M1820_DelayUs(2);

    /* 释放总线，让从机输出 */
    M1820_LineRelease(dev);
    M1820_DelayUs(10);

    /* 在读时隙前段采样 */
    bit_value = M1820_LineRead(dev);

    /* 补齐时隙 */
    M1820_DelayUs(50);

    return bit_value;
}

/* 写 1 个字节，LSB first */
static void M1820_WriteByte(M1820_Device_t *dev, uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        M1820_WriteBit(dev, data & 0x01);
        data >>= 1;
    }
}

/* 读 1 个字节，LSB first */
static uint8_t M1820_ReadByte(M1820_Device_t *dev)
{
    uint8_t i;
    uint8_t data = 0;

    for (i = 0; i < 8; i++)
    {
        if (M1820_ReadBit(dev))
        {
            data |= (1U << i);
        }
    }

    return data;
}

/* =========================================================
 * 4. CRC8
 *
 * 按 Dallas/Maxim 常见 OneWire CRC8 算法实现。
 * ========================================================= */
static uint8_t M1820_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t i, j;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x01)
            {
                crc = (crc >> 1) ^ 0x8C;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/* =========================================================
 * 5. 温度换算
 * ========================================================= */

/* M1820 温度格式：
 * T = 40 + raw / 256
 */
static float M1820_RawToTemp(int16_t raw)
{
    return 40.0f + ((float)raw / 256.0f);
}

/* DS18B20 / 兼容格式：
 * T = raw / 16
 */
static float DS18B20_RawToTemp(int16_t raw)
{
    return ((float)raw / 16.0f);
}

/* =========================================================
 * 6. 初始化
 * ========================================================= */
void M1820_Init(void)
{
    /* 初始化微秒延时 */
    M1820_DelayUsInit();

    /* 默认释放两路总线 */
    M1820_LineRelease(&g_m1820_env);
    M1820_LineRelease(&g_m1820_surf);
}

/* =========================================================
 * 7. 读取指定探头温度
 *
 * 流程：
 * 1) Reset + Presence
 * 2) Skip ROM
 * 3) Convert T
 * 4) 等待转换完成
 * 5) Reset + Presence
 * 6) Skip ROM
 * 7) Read Scratchpad
 * 8) 读 9 字节
 * 9) CRC 校验
 * 10) 解析温度
 *
 * 返回：
 * 1 = 成功
 * 0 = 失败
 * ========================================================= */
uint8_t M1820_ReadTemperature(M1820_Device_t *dev, float *temp)
{
    uint8_t scratchpad[9];
    uint8_t i;
    int16_t raw;

    /* 第一次复位：准备启动温度转换 */
    if (!M1820_ResetAndPresence(dev))
    {
        return 0;
    }

    /* 单器件场景，直接跳过 ROM */
    M1820_WriteByte(dev, M1820_CMD_SKIP_ROM);

    /* 启动温度转换 */
    M1820_WriteByte(dev, M1820_CMD_CONVERT_T);

    /* 等待转换完成 */
    HAL_Delay(12);

    /* 第二次复位：准备读取暂存器 */
    if (!M1820_ResetAndPresence(dev))
    {
        return 0;
    }

    M1820_WriteByte(dev, M1820_CMD_SKIP_ROM);
    M1820_WriteByte(dev, M1820_CMD_READ_SCRATCHPAD);

    /* 连续读取 9 字节 */
    for (i = 0; i < 9; i++)
    {
        scratchpad[i] = M1820_ReadByte(dev);
    }

    /* CRC 校验 */
    if (M1820_Crc8(scratchpad, 8) != scratchpad[8])
    {
        return 0;
    }

    /* 前两个字节组成温度原始值 */
    raw = (int16_t)(((uint16_t)scratchpad[1] << 8) | scratchpad[0]);

    /* 根据探头类型选择解析公式 */
    if (dev->format == SENSOR_FMT_M1820)
    {
        *temp = M1820_RawToTemp(raw);
    }
    else
    {
        *temp = DS18B20_RawToTemp(raw);
    }

    return 1;
}


