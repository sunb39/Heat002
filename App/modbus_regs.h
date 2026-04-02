

#ifndef __MODBUS_REGS_H
#define __MODBUS_REGS_H

#include "mb.h"
#include "mbport.h"
#include <stdint.h>

/* =========================================================
 * 协议表寄存器范围定义
 * 你的协议表地址范围是 1 ~ 68
 * 因此保持寄存器缓冲区也按 1~68 来映射
 * ========================================================= */
#define HREG_START_ADDR                 2
#define HREG_END_ADDR                   69
#define HREG_NUMBER                     69



/* =========================================================
 * R/W Float 区（每个 float 占 2 个寄存器）
 * 采用高字在前、低字在后
 * ========================================================= */
#define REG_ENV_TEMP_THRESHOLD_H        2   /* 环境温度阈值 高16位 */
#define REG_ENV_TEMP_THRESHOLD_L        3   /* 环境温度阈值 低16位 */

#define REG_SURF_TEMP_TARGET_H          4   /* 表面温度目标值 高16位 */
#define REG_SURF_TEMP_TARGET_L          5   /* 表面温度目标值 低16位 */

#define REG_MANUAL_PWM_SET_H            6   /* 手动PWM设定值 高16位 */
#define REG_MANUAL_PWM_SET_L            7   /* 手动PWM设定值 低16位 */

/* =========================================================
 * R/W Word 区
 * ========================================================= */

#define REG_CTRL_MODE_SET               8   /* 控制模式 */
#define REG_CMD_CLEAR_FAULT             9   /* 清故障命令 */


/* 9~23 保留 */
#define REG_RESERVED_RW_01             10
#define REG_RESERVED_RW_15             24

/* =========================================================
 * R Float 区
 * ========================================================= */
#define REG_ENV_TEMP_REAL_H            25   /* 环境温度实时值 高16位 */
#define REG_ENV_TEMP_REAL_L            26   /* 环境温度实时值 低16位 */

#define REG_SURF_TEMP_REAL_H           27   /* 表面温度实时值 高16位 */
#define REG_SURF_TEMP_REAL_L           28   /* 表面温度实时值 低16位 */

#define REG_HEAT_OUT_VOLT_H            29   /* 加热板输出电压 高16位 */
#define REG_HEAT_OUT_VOLT_L            30   /* 加热板输出电压 低16位 */

#define REG_PWM_DUTY_REAL_H            31   /* 当前PWM占空比 高16位 */
#define REG_PWM_DUTY_REAL_L            32   /* 当前PWM占空比 低16位 */

/* =========================================================
 * R Word 区
 * ========================================================= */
#define REG_STATUS_WORD                33   /* 设备状态字 */
#define REG_FAULT_WORD                 34   /* 设备故障字 */
#define REG_HEAT_OUTPUT_STATUS         35   /* 加热输出状态 */
#define REG_ENV_PROBE_STATUS           36   /* 环境探头状态 */
#define REG_SURF_PROBE_STATUS          37   /* 表面探头状态 */
#define REG_ENV_PROBE_ERR_CNT          38   /* 环境探头异常计数 */
#define REG_SURF_PROBE_ERR_CNT         39   /* 表面探头异常计数 */
#define REG_SLAVE_ADDR_R							 40		/* 从机地址默认12 */
#define REG_BAUD_CODE_R                41		/* 波特率默认0 = 9600 */
#define REG_TEMP_ALARM_EN_R            42		/* 温度异常上报使能开启1 */
#define REG_FW_MAJOR_VER               43   /* 固件主版本号 */
#define REG_FW_MINOR_VER               44   /* 固件次版本号 */

/* 44~68 保留 */
#define REG_RESERVED_R_01              45
#define REG_RESERVED_R_25              69

/* =========================================================
 * 控制模式定义
 * ========================================================= */
#define CTRL_MODE_OFF                   0   /* 关闭模式 */
#define CTRL_MODE_AUTO                  1   /* 自动控制 */
#define CTRL_MODE_FORCE_FULL            2   /* 强制全功率 */
#define CTRL_MODE_MANUAL_PWM            3   /* 手动PWM模式 */

/* =========================================================
 * 波特率代码定义
 * ========================================================= */
#define BAUD_CODE_9600                  0
#define BAUD_CODE_19200                 1
#define BAUD_CODE_38400                 2
#define BAUD_CODE_115200                3

/* =========================================================
 * 全局寄存器镜像缓冲区
 * 下标 0 不使用，直接使用 1~69 对应协议表地址
 * ========================================================= */
extern uint16_t usHoldingRegBuf[HREG_NUMBER + 1];

/* =========================================================
 * 参数变量（R/W）
 * 这些变量是“配置参数”，可通过 Modbus 读写
 * ========================================================= */
extern float    g_env_temp_threshold;   /* 环境温度阈值 */
extern float    g_surf_temp_target;     /* 表面温度目标值 */
extern float    g_manual_pwm_set;       /* 手动PWM设定值 */

extern uint16_t g_ctrl_mode_set;        /* 控制模式设定 */
extern uint16_t g_slave_addr;           /* 从机地址 */
extern uint16_t g_baud_code;            /* 波特率代码 */
extern uint16_t g_temp_alarm_en;        /* 温度异常上报使能 */

/* 命令寄存器 */

extern uint16_t g_cmd_clear_fault;      /* 清故障命令 */
//extern uint16_t g_cmd_soft_reset;       /* 软复位命令 */

/* =========================================================
 * 实时变量（R）
 * 这些变量是运行时实时数据，只读上报
 * ========================================================= */
extern float    g_env_temp_real;        /* 环境温度实时值 */
extern float    g_surf_temp_real;       /* 表面温度实时值 */
extern float    g_heat_out_volt;        /* 加热板输出电压 */
extern float    g_pwm_duty_real;        /* 当前PWM占空比 */

extern uint16_t g_status_word;          /* 设备状态字 */
extern uint16_t g_fault_word;           /* 设备故障字 */
extern uint16_t g_heat_output_status;   /* 加热输出状态 */
extern uint16_t g_env_probe_status;     /* 环境探头状态 */
extern uint16_t g_surf_probe_status;    /* 表面探头状态 */
extern uint16_t g_env_probe_err_cnt;    /* 环境探头异常计数 */
extern uint16_t g_surf_probe_err_cnt;   /* 表面探头异常计数 */
extern uint16_t g_fw_major_ver;         /* 固件主版本号 */
extern uint16_t g_fw_minor_ver;         /* 固件次版本号 */

/* =========================================================
 * 接口函数
 * ========================================================= */

/* 加载默认参数 */
void Modbus_LoadDefaultParams(void);

/* Modbus 寄存器初始化
 * 内部流程：先默认参数 -> 再尝试从 Flash 加载 -> 再同步到寄存器 */
void Modbus_RegsInit(void);

/* 业务变量 -> 寄存器镜像 */
void Modbus_RegsSyncToBuffer(void);

/* 寄存器镜像 -> 参数变量 */
void Modbus_RegsSyncFromBuffer(void);

/* 命令处理函数
 * 在主循环中调用，负责处理保存参数、清故障、恢复默认等命令 */
void Modbus_CmdProcess(void);
/*自动保存任务*/
void Modbus_AutoSaveTask(void);

/* FreeModbus 保持寄存器回调函数 */
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode);

#endif

