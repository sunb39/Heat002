#ifndef __M1820_H
#define __M1820_H

#include "main.h"
#include <stdint.h>

/* =========================================================
 * 单总线基础命令
 * ========================================================= */
#define M1820_CMD_SKIP_ROM          0xCC
#define M1820_CMD_CONVERT_T         0x44
#define M1820_CMD_READ_SCRATCHPAD   0xBE
#define M1820_CMD_MATCH_ROM         0x55

/* =========================================================
 * 传感器温度数据格式类型
 *
 * 说明：
 * 1) 环境温度探头：按 M1820 格式解析
 * 2) 表面温度探头：按 DS18B20/兼容格式解析
 * ========================================================= */
#define SENSOR_FMT_M1820            0
#define SENSOR_FMT_DS18B20          1

/* =========================================================
 * 探头对象
 *
 * port   : 探头所接 GPIO 端口
 * pin    : 探头所接 GPIO 引脚
 * format : 温度数据解析格式
 * ========================================================= */
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       format;
} M1820_Device_t;

/* 两路探头对象 */
extern M1820_Device_t g_m1820_env;
extern M1820_Device_t g_m1820_surf;

/* 对外接口 */
void M1820_Init(void);
uint8_t M1820_ReadTemperature(M1820_Device_t *dev, float *temp);

#endif
