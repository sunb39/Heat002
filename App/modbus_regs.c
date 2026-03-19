
#include "main.h"
#include "m1820.h"
#include "modbus_regs.h"
#include "param_store.h"
#include <string.h>

/* =========================================================
 * 寄存器镜像缓冲区
 * 下标 0 不使用
 * usHoldingRegBuf[1] 对应协议表地址 1
 * ========================================================= */
uint16_t usHoldingRegBuf[HREG_NUMBER + 1] = {0};

/* =========================================================
 * 参数变量（可读可写）
 * 这些是系统配置参数
 * 上电后先装载默认值，再尝试从 Flash 覆盖
 * ========================================================= */
float    g_env_temp_threshold = 10.0f;
float    g_surf_temp_target   = 20.0f;
float    g_manual_pwm_set     = 50.0f;

uint16_t g_ctrl_mode_set      = CTRL_MODE_AUTO;
uint16_t g_slave_addr         = 12;
uint16_t g_baud_code          = BAUD_CODE_9600;
uint16_t g_temp_alarm_en      = 1;

uint16_t g_cmd_save_param      = 0;
uint16_t g_cmd_clear_fault     = 0;
uint16_t g_cmd_restore_default = 0;
uint16_t g_cmd_soft_reset      = 0;

/* =========================================================
 * 实时变量（只读）
 * 目前先给假数据，后面接真实采集与控制逻辑
 * ========================================================= */
float    g_env_temp_real      = 25.5f;
float    g_surf_temp_real     = 31.2f;
float    g_heat_out_volt      = 12.0f;
float    g_pwm_duty_real      = 0.0f;

uint16_t g_status_word        = 0;
uint16_t g_fault_word         = 0;
uint16_t g_heat_output_status = 0;
uint16_t g_env_probe_status   = 0;
uint16_t g_surf_probe_status  = 0;
uint16_t g_env_probe_err_cnt  = 0;
uint16_t g_surf_probe_err_cnt = 0;
uint16_t g_comm_err_cnt       = 0;
uint16_t g_fw_major_ver       = 1;
uint16_t g_fw_minor_ver       = 0;

/* =========================================================
 * 内部工具函数：float <-> 2 个寄存器
 * 采用高字在前、低字在后
 * ========================================================= */

/* 把 float 转成两个 16 位寄存器（高16位 + 低16位） */
static void FloatToRegs(float value, uint16_t *hi, uint16_t *lo)
{
    union
    {
        float    f;
        uint32_t u32;
    } data;

    data.f = value;

    *hi = (uint16_t)(data.u32 >> 16);
    *lo = (uint16_t)(data.u32 & 0xFFFF);
}

/* 把两个 16 位寄存器还原成 float */
static float RegsToFloat(uint16_t hi, uint16_t lo)
{
    union
    {
        float    f;
        uint32_t u32;
    } data;

    data.u32 = ((uint32_t)hi << 16) | lo;
    return data.f;
}

/* =========================================================
 * 加载默认参数
 * 恢复默认参数命令会调用这个函数
 * ========================================================= */
void Modbus_LoadDefaultParams(void)
{
    g_env_temp_threshold = 10.0f;
    g_surf_temp_target   = 20.0f;
    g_manual_pwm_set     = 50.0f;

    g_ctrl_mode_set      = CTRL_MODE_AUTO;
    g_slave_addr         = 12;
    g_baud_code          = BAUD_CODE_9600;
    g_temp_alarm_en      = 1;
}

/* =========================================================
 * Modbus 寄存器初始化
 * 上电初始化时调用
 * 顺序非常重要：
 * 1. 先加载默认参数
 * 2. 再尝试从 Flash 加载保存参数
 * 3. 最后同步到寄存器缓冲区
 * ========================================================= */
void Modbus_RegsInit(void)
{
    /* 先清空寄存器镜像 */
    memset(usHoldingRegBuf, 0, sizeof(usHoldingRegBuf));

    /* 先加载默认参数 */
    Modbus_LoadDefaultParams();

    /* 再尝试从 Flash 读取保存过的参数
       如果 Flash 中有有效数据，会覆盖默认参数；
       如果没有有效数据，则继续使用默认参数 */
    ParamStore_Load();

    /* 最后把当前变量同步到寄存器缓冲区 */
    Modbus_RegsSyncToBuffer();
}

/* =========================================================
 * 业务变量 -> 寄存器镜像
 * 每次主站读保持寄存器前，都会把最新变量同步到寄存器
 * ========================================================= */
void Modbus_RegsSyncToBuffer(void)
{
    /* ---------- R/W Float 区 ---------- */
    FloatToRegs(g_env_temp_threshold,
                &usHoldingRegBuf[REG_ENV_TEMP_THRESHOLD_H],
                &usHoldingRegBuf[REG_ENV_TEMP_THRESHOLD_L]);

    FloatToRegs(g_surf_temp_target,
                &usHoldingRegBuf[REG_SURF_TEMP_TARGET_H],
                &usHoldingRegBuf[REG_SURF_TEMP_TARGET_L]);

    FloatToRegs(g_manual_pwm_set,
                &usHoldingRegBuf[REG_MANUAL_PWM_SET_H],
                &usHoldingRegBuf[REG_MANUAL_PWM_SET_L]);

    /* ---------- R/W Word 区 ---------- */
    usHoldingRegBuf[REG_CTRL_MODE_SET]       = g_ctrl_mode_set;
    usHoldingRegBuf[REG_SLAVE_ADDR]          = g_slave_addr;
    usHoldingRegBuf[REG_BAUD_CODE]           = g_baud_code;
    usHoldingRegBuf[REG_TEMP_ALARM_EN]       = g_temp_alarm_en;

    usHoldingRegBuf[REG_CMD_SAVE_PARAM]      = g_cmd_save_param;
    usHoldingRegBuf[REG_CMD_CLEAR_FAULT]     = g_cmd_clear_fault;
    usHoldingRegBuf[REG_CMD_RESTORE_DEFAULT] = g_cmd_restore_default;
    usHoldingRegBuf[REG_CMD_SOFT_RESET]      = g_cmd_soft_reset;

    /* ---------- R Float 区 ---------- */
    FloatToRegs(g_env_temp_real,
                &usHoldingRegBuf[REG_ENV_TEMP_REAL_H],
                &usHoldingRegBuf[REG_ENV_TEMP_REAL_L]);

    FloatToRegs(g_surf_temp_real,
                &usHoldingRegBuf[REG_SURF_TEMP_REAL_H],
                &usHoldingRegBuf[REG_SURF_TEMP_REAL_L]);

    FloatToRegs(g_heat_out_volt,
                &usHoldingRegBuf[REG_HEAT_OUT_VOLT_H],
                &usHoldingRegBuf[REG_HEAT_OUT_VOLT_L]);

    FloatToRegs(g_pwm_duty_real,
                &usHoldingRegBuf[REG_PWM_DUTY_REAL_H],
                &usHoldingRegBuf[REG_PWM_DUTY_REAL_L]);

    /* ---------- R Word 区 ---------- */
    usHoldingRegBuf[REG_STATUS_WORD]        = g_status_word;
    usHoldingRegBuf[REG_FAULT_WORD]         = g_fault_word;
    usHoldingRegBuf[REG_HEAT_OUTPUT_STATUS] = g_heat_output_status;
    usHoldingRegBuf[REG_ENV_PROBE_STATUS]   = g_env_probe_status;
    usHoldingRegBuf[REG_SURF_PROBE_STATUS]  = g_surf_probe_status;
    usHoldingRegBuf[REG_ENV_PROBE_ERR_CNT]  = g_env_probe_err_cnt;
    usHoldingRegBuf[REG_SURF_PROBE_ERR_CNT] = g_surf_probe_err_cnt;
    usHoldingRegBuf[REG_COMM_ERR_CNT]       = g_comm_err_cnt;
    usHoldingRegBuf[REG_FW_MAJOR_VER]       = g_fw_major_ver;
    usHoldingRegBuf[REG_FW_MINOR_VER]       = g_fw_minor_ver;
}

/* =========================================================
 * 寄存器镜像 -> 参数变量
 * 每次主站写保持寄存器后，把寄存器数据回写到参数变量
 * ========================================================= */
void Modbus_RegsSyncFromBuffer(void)
{
    g_env_temp_threshold = RegsToFloat(usHoldingRegBuf[REG_ENV_TEMP_THRESHOLD_H],
                                       usHoldingRegBuf[REG_ENV_TEMP_THRESHOLD_L]);

    g_surf_temp_target   = RegsToFloat(usHoldingRegBuf[REG_SURF_TEMP_TARGET_H],
                                       usHoldingRegBuf[REG_SURF_TEMP_TARGET_L]);

    g_manual_pwm_set     = RegsToFloat(usHoldingRegBuf[REG_MANUAL_PWM_SET_H],
                                       usHoldingRegBuf[REG_MANUAL_PWM_SET_L]);

    g_ctrl_mode_set      = usHoldingRegBuf[REG_CTRL_MODE_SET];
    g_slave_addr         = usHoldingRegBuf[REG_SLAVE_ADDR];
    g_baud_code          = usHoldingRegBuf[REG_BAUD_CODE];
    g_temp_alarm_en      = usHoldingRegBuf[REG_TEMP_ALARM_EN];

    g_cmd_save_param      = usHoldingRegBuf[REG_CMD_SAVE_PARAM];
    g_cmd_clear_fault     = usHoldingRegBuf[REG_CMD_CLEAR_FAULT];
    g_cmd_restore_default = usHoldingRegBuf[REG_CMD_RESTORE_DEFAULT];
    g_cmd_soft_reset      = usHoldingRegBuf[REG_CMD_SOFT_RESET];
}

/* =========================================================
 * 命令处理函数
 * 在主循环中周期调用
 * 负责处理：
 * - 清故障
 * - 恢复默认参数
 * - 保存参数到 Flash
 * - 软复位（当前先占位）
 * ========================================================= */
void Modbus_CmdProcess(void)
{
    /* ------------------------------
     * 清故障命令
     * 协议表地址 12
     * 写 1 后执行，执行后自动清零
     * ------------------------------ */
    if (g_cmd_clear_fault == 1U)
    {
        /* 清故障字 */
        g_fault_word = 0;

        /* 清探头状态 */
        g_env_probe_status  = 0;
        g_surf_probe_status = 0;

        /* 清异常计数 */
        g_env_probe_err_cnt  = 0;
        g_surf_probe_err_cnt = 0;

        /* 命令位自动清零 */
        g_cmd_clear_fault = 0;

        /* 把更新后的变量同步回寄存器 */
        Modbus_RegsSyncToBuffer();
    }

    /* ------------------------------
     * 恢复默认参数命令
     * 协议表地址 13
     * 写 1 后执行，执行后自动清零
     * ------------------------------ */
    if (g_cmd_restore_default == 1U)
    {
        /* 重新加载默认参数 */
        Modbus_LoadDefaultParams();

        /* 命令位自动清零 */
        g_cmd_restore_default = 0;

        /* 把默认参数同步回寄存器 */
        Modbus_RegsSyncToBuffer();
    }

    /* ------------------------------
     * 保存参数命令
     * 协议表地址 11
     * 写 1 后执行，执行后自动清零
     * ------------------------------ */
    if (g_cmd_save_param == 1U)
    {
        /* 保存当前参数到 Flash */
        (void)ParamStore_Save();

        /* 命令位自动清零 */
        g_cmd_save_param = 0;

        /* 把清零后的命令位同步回寄存器 */
        Modbus_RegsSyncToBuffer();
    }

    /* ------------------------------
 * 软复位命令
 * 协议表地址 14
 * 写 1 后执行软件复位，执行前先清命令位
 * ------------------------------ */
	if (g_cmd_soft_reset == 1U)
	{
    /* 1. 先把命令位清零，避免复位后主站仍读到 1 */
    g_cmd_soft_reset = 0;

    /* 2. 把清零后的结果同步回寄存器镜像 */
    Modbus_RegsSyncToBuffer();

    /* 3. 给一点短暂延时，让前面的寄存器更新和总线响应更稳定
       注意：这里只是很短的延时，不影响整体逻辑 */
    HAL_Delay(20);

    /* 4. 执行 MCU 软件复位 */
    NVIC_SystemReset();
	}
}

/* =========================================================
 * FreeModbus 保持寄存器回调
 * 当主站执行 03 / 06 / 16 等保持寄存器操作时，会进入这里
 * ========================================================= */
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode)
{
    USHORT i;
    USHORT reg;

    /* 地址越界检查 */
    if ((usAddress < HREG_START_ADDR) ||
        ((usAddress + usNRegs - 1) > HREG_END_ADDR))
    {
        return MB_ENOREG;
    }

    reg = usAddress;

    /* ------------------------------
     * 读保持寄存器
     * ------------------------------ */
    if (eMode == MB_REG_READ)
    {
        /* 先把最新变量同步到寄存器镜像 */
        Modbus_RegsSyncToBuffer();

        /* 再按 Modbus 协议格式把 16 位寄存器拆成高字节/低字节输出 */
        for (i = 0; i < usNRegs; i++)
        {
            *pucRegBuffer++ = (UCHAR)(usHoldingRegBuf[reg] >> 8);
            *pucRegBuffer++ = (UCHAR)(usHoldingRegBuf[reg] & 0xFF);
            reg++;
        }
    }
    /* ------------------------------
     * 写保持寄存器
     * ------------------------------ */
    else if (eMode == MB_REG_WRITE)
    {
        /* 先把主站写来的数据写入寄存器镜像 */
        for (i = 0; i < usNRegs; i++)
        {
            usHoldingRegBuf[reg]  = ((uint16_t)(*pucRegBuffer++) << 8);
            usHoldingRegBuf[reg] |=  (uint16_t)(*pucRegBuffer++);
            reg++;
        }

        /* 再把寄存器镜像同步回参数变量 */
        Modbus_RegsSyncFromBuffer();
    }

    return MB_ENOERR;
}
