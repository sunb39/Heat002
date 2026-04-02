
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


uint16_t g_cmd_clear_fault = 0;
//uint16_t g_cmd_soft_reset  = 0;

/* 自动保存相关内部状态 */
static uint8_t  s_auto_save_pending = 0U;
static uint32_t s_auto_save_tick = 0U;

/* float 参数写入完整性跟踪
   bit0 = 高16位已写
   bit1 = 低16位已写 */
static uint8_t  s_env_wr_mask    = 0U;   /* 地址 1~2 */
static uint8_t  s_surf_wr_mask   = 0U;   /* 地址 3~4 */
static uint8_t  s_manual_wr_mask = 0U;   /* 地址 5~6 */

/* 用来判断 float 两个寄存器是否都写完整 */
//static uint8_t  s_env_threshold_write_mask = 0U;   /* bit0=地址1, bit1=地址2 */
//static uint8_t  s_surf_target_write_mask   = 0U;   /* bit0=地址3, bit1=地址4 */

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
//uint16_t g_comm_err_cnt       = 0;
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



void Modbus_AutoSaveTask(void)
{
    if (s_auto_save_pending)
    {
        /* 延时 300ms，避免同一次连续写多个寄存器时反复擦写 Flash */
        if ((HAL_GetTick() - s_auto_save_tick) >= 300U)
        {
            (void)ParamStore_Save();
            s_auto_save_pending = 0U;
        }
    }
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

    /* 再尝试从 Flash 读取-保存过的参数
       如果 Flash 中有有效数据，会覆盖默认参数；
       如果没有有效数据，则继续使用默认参数 */
    ParamStore_Load();

    /* 最后把当前变量同步到寄存器缓冲区 */
    Modbus_RegsSyncToBuffer();
}



static void Modbus_RequestAutoSave(void)
{
    s_auto_save_pending = 1U;
    s_auto_save_tick = HAL_GetTick();
}

static void Modbus_CheckAutoSaveTrigger(USHORT start_addr, USHORT nregs)
{
    USHORT end_addr = start_addr + nregs - 1U;

    /* ---------- 环境温度阈值：地址 1~2 ---------- */
    if ((start_addr <= REG_ENV_TEMP_THRESHOLD_H) &&
        (end_addr   >= REG_ENV_TEMP_THRESHOLD_H))
    {
        s_env_wr_mask |= 0x01U;
    }

    if ((start_addr <= REG_ENV_TEMP_THRESHOLD_L) &&
        (end_addr   >= REG_ENV_TEMP_THRESHOLD_L))
    {
        s_env_wr_mask |= 0x02U;
    }

    if (s_env_wr_mask == 0x03U)
    {
        s_env_wr_mask = 0U;
        Modbus_RequestAutoSave();
    }

    /* ---------- 表面温度目标值：地址 3~4 ---------- */
    if ((start_addr <= REG_SURF_TEMP_TARGET_H) &&
        (end_addr   >= REG_SURF_TEMP_TARGET_H))
    {
        s_surf_wr_mask |= 0x01U;
    }

    if ((start_addr <= REG_SURF_TEMP_TARGET_L) &&
        (end_addr   >= REG_SURF_TEMP_TARGET_L))
    {
        s_surf_wr_mask |= 0x02U;
    }

    if (s_surf_wr_mask == 0x03U)
    {
        s_surf_wr_mask = 0U;
        Modbus_RequestAutoSave();
    }

    /* ---------- 手动 PWM：地址 5~6 ---------- */
    if ((start_addr <= REG_MANUAL_PWM_SET_H) &&
        (end_addr   >= REG_MANUAL_PWM_SET_H))
    {
        s_manual_wr_mask |= 0x01U;
    }

    if ((start_addr <= REG_MANUAL_PWM_SET_L) &&
        (end_addr   >= REG_MANUAL_PWM_SET_L))
    {
        s_manual_wr_mask |= 0x02U;
    }

    if (s_manual_wr_mask == 0x03U)
    {
        s_manual_wr_mask = 0U;
        Modbus_RequestAutoSave();
    }

    /* ---------- 控制模式：地址 7 ---------- */
    if ((start_addr <= REG_CTRL_MODE_SET) &&
        (end_addr   >= REG_CTRL_MODE_SET))
    {
        Modbus_RequestAutoSave();
    }
}

/* =========================================================
 * 业务变量 -> 寄存器镜像
 * 每次主站读保持寄存器前，都会把最新变量同步到寄存器
 * ========================================================= */
void Modbus_RegsSyncToBuffer(void)
{
    uint16_t reg;

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
    usHoldingRegBuf[REG_CTRL_MODE_SET]    = g_ctrl_mode_set;
    usHoldingRegBuf[REG_CMD_CLEAR_FAULT]  = g_cmd_clear_fault;

    /* 9~23 保留 */
    for (reg = REG_RESERVED_RW_01; reg <= REG_RESERVED_RW_15; reg++)
    {
        usHoldingRegBuf[reg] = 0U;
    }

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
    usHoldingRegBuf[REG_SLAVE_ADDR_R]       = g_slave_addr;
    usHoldingRegBuf[REG_BAUD_CODE_R]        = g_baud_code;
    usHoldingRegBuf[REG_TEMP_ALARM_EN_R]    = 1U;
    usHoldingRegBuf[REG_FW_MAJOR_VER]       = g_fw_major_ver;
    usHoldingRegBuf[REG_FW_MINOR_VER]       = g_fw_minor_ver;

    /* 44~68 保留 */
    for (reg = REG_RESERVED_R_01; reg <= REG_RESERVED_R_25; reg++)
    {
        usHoldingRegBuf[reg] = 0U;
    }
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
    g_cmd_clear_fault    = usHoldingRegBuf[REG_CMD_CLEAR_FAULT];

    /* 固定只读参数，强制保持 */
    g_slave_addr         = 12;
    g_baud_code          = BAUD_CODE_9600;
    g_temp_alarm_en      = 1U;
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
    if (g_cmd_clear_fault == 1U)
    {
        g_fault_word = 0;
        g_env_probe_status  = 0;
        g_surf_probe_status = 0;
        g_env_probe_err_cnt  = 0;
        g_surf_probe_err_cnt = 0;

        g_cmd_clear_fault = 0;
        Modbus_RegsSyncToBuffer();
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
    USHORT write_start;
    USHORT write_end;

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
        write_start = usAddress;
        write_end   = usAddress + usNRegs - 1;

        /* 只允许写 1~23，24~68 为只读区 */
        if ((write_start > REG_RESERVED_RW_15) || (write_end > REG_RESERVED_RW_15))
        {
            return MB_ENOREG;
        }

        for (i = 0; i < usNRegs; i++)
        {
            usHoldingRegBuf[reg]  = ((uint16_t)(*pucRegBuffer++) << 8);
            usHoldingRegBuf[reg] |=  (uint16_t)(*pucRegBuffer++);
            reg++;
        }

        /* 先把寄存器镜像同步回参数变量 */
        Modbus_RegsSyncFromBuffer();

        /* 自动保存触发判断 */
        Modbus_CheckAutoSaveTrigger(usAddress, usNRegs);
    }

    return MB_ENOERR;
}
