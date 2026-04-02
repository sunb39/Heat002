#include "param_store.h"
#include "stm32f1xx_hal.h"
#include "modbus_regs.h"

/* =========================================================
 * Flash 存储地址定义
 *
 * STM32F103C8T6 按 64KB Flash 计算：
 * Flash 地址范围：0x08000000 ~ 0x0800FFFF
 * 每页大小：1KB
 * 最后一页起始地址：0x0800FC00
 *
 * 这里先把参数存放在最后一页。
 * 注意：如果后面程序代码变很大，不能覆盖到这一页。
 * ========================================================= */
#define PARAM_FLASH_ADDR   0x0800FC00U

/* =========================================================
 * 魔术字
 * 用于判断这一页 Flash 是否存的是我们的有效参数
 * ========================================================= */
#define PARAM_MAGIC        0x50415241U   /* ASCII: 'PARA' */

/* =========================================================
 * Flash 参数镜像结构体
 *
 * 这里全部使用 uint32_t 字段：
 * - float 先转换成原始 32 位数据再保存
 * - uint16_t 参数也扩展成 uint32_t 保存
 *
 * 这样做的好处：
 * - 写 Flash 时每次按 32 位字写入，最简单
 * - 结构体对齐更稳定
 * ========================================================= */
typedef struct
{
    uint32_t magic;               /* 魔术字，用于判断参数是否有效 */

    uint32_t env_temp_threshold;  /* 环境温度阈值（float转uint32_t） */
    uint32_t surf_temp_target;    /* 表面温度目标值（float转uint32_t） */
    uint32_t manual_pwm_set;      /* 手动PWM设定值（float转uint32_t） */

    uint32_t ctrl_mode_set;       /* 控制模式 */
    uint32_t temp_alarm_en;       /* 温度异常上报使能 */

    uint32_t checksum;            /* 简单校验 */
} ParamStoreImage_t;

/* =========================================================
 * float 与 uint32_t 互转工具函数
 * ========================================================= */

/* 把 float 的原始二进制转换成 uint32_t */
static uint32_t FloatToWord(float value)
{
    union
    {
        float    f;
        uint32_t u;
    } data;

    data.f = value;
    return data.u;
}

/* 把 uint32_t 原始二进制还原成 float */
static float WordToFloat(uint32_t value)
{
    union
    {
        float    f;
        uint32_t u;
    } data;

    data.u = value;
    return data.f;
}

/* =========================================================
 * 简单校验函数
 * 这里采用异或校验，足够做最小版参数校验
 * ========================================================= */
static uint32_t ParamStore_CalcChecksum(const ParamStoreImage_t *img)
{
    return  img->magic
          ^ img->env_temp_threshold
          ^ img->surf_temp_target
          ^ img->manual_pwm_set
          ^ img->ctrl_mode_set
          ^ img->temp_alarm_en;
}

/* =========================================================
 * 从 Flash 加载参数
 * 成功返回 1，失败返回 0
 * ========================================================= */
uint8_t ParamStore_Load(void)
{
    /* 直接把 Flash 指定地址映射成参数结构体指针 */
    const ParamStoreImage_t *img = (const ParamStoreImage_t *)PARAM_FLASH_ADDR;

    /* 第一步：判断魔术字 */
    if (img->magic != PARAM_MAGIC)
    {
        /* 魔术字不对，说明还没保存过有效参数 */
        return 0;
    }

    /* 第二步：判断校验 */
    if (img->checksum != ParamStore_CalcChecksum(img))
    {
        /* 校验不通过，说明 Flash 数据可能损坏 */
        return 0;
    }

    /* 第三步：做简单合法性检查 */
    if (img->ctrl_mode_set > CTRL_MODE_MANUAL_PWM)
    {
        return 0;
    }

    if (img->temp_alarm_en > 1U)
    {
        return 0;
    }

    /* 第四步：把 Flash 数据加载到当前参数变量 */
    g_env_temp_threshold = WordToFloat(img->env_temp_threshold);
    g_surf_temp_target   = WordToFloat(img->surf_temp_target);
    g_manual_pwm_set     = WordToFloat(img->manual_pwm_set);

//    g_ctrl_mode_set      = (uint16_t)img->ctrl_mode_set;
		g_ctrl_mode_set      = CTRL_MODE_AUTO;
    g_temp_alarm_en      = 1U;

    return 1;
}

/* =========================================================
 * 保存当前参数到 Flash
 * 成功返回 1，失败返回 0
 * ========================================================= */
uint8_t ParamStore_Save(void)
{
    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    uint32_t i;

    /* 用一个临时镜像结构体，把当前参数打包 */
    ParamStoreImage_t img;

    /* 第一步：填充参数镜像 */
    img.magic              = PARAM_MAGIC;
    img.env_temp_threshold = FloatToWord(g_env_temp_threshold);
    img.surf_temp_target   = FloatToWord(g_surf_temp_target);
    img.manual_pwm_set     = FloatToWord(g_manual_pwm_set);

//    img.ctrl_mode_set      = (uint32_t)g_ctrl_mode_set;
		img.ctrl_mode_set      = (uint32_t)CTRL_MODE_AUTO;
    img.temp_alarm_en      = 1U;

    img.checksum           = ParamStore_CalcChecksum(&img);

    /* 第二步：解锁 Flash */
    HAL_FLASH_Unlock();

    /* 第三步：擦除最后一页 Flash */
    erase_init.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = PARAM_FLASH_ADDR;
    erase_init.NbPages     = 1;

    status = HAL_FLASHEx_Erase(&erase_init, &page_error);
    if (status != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0;
    }

    /* 第四步：按 32 位字逐个写入 */
    {
        const uint32_t *src = (const uint32_t *)&img;
        uint32_t addr = PARAM_FLASH_ADDR;

        for (i = 0; i < (sizeof(ParamStoreImage_t) / 4U); i++)
        {
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src[i]);
            if (status != HAL_OK)
            {
                HAL_FLASH_Lock();
                return 0;
            }

            addr += 4U;
        }
    }

    /* 第五步：重新上锁 Flash */
    HAL_FLASH_Lock();

    return 1;
}
