#ifndef __PARAM_STORE_H
#define __PARAM_STORE_H

#include <stdint.h>

/* =========================================================
 * 参数存储接口
 * ========================================================= */

/* 从 Flash 加载参数
 * 返回 1 = 加载成功
 * 返回 0 = Flash 中无有效参数，继续使用默认参数 */
uint8_t ParamStore_Load(void);

/* 保存当前参数到 Flash
 * 返回 1 = 保存成功
 * 返回 0 = 保存失败 */
uint8_t ParamStore_Save(void);

#endif

