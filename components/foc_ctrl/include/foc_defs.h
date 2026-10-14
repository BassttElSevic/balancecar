/*
 * ============================================================================
 *  foc_ctrl —— FOC 控制器元件（PID / 低通滤波）
 *
 *  移植自 SimpleFOC:  src/common/pid.cpp / pid.h
 *                    src/common/lowpass_filter.cpp / lowpass_filter.h
 *
 *  纯计算，不使用堆；时间源统一走 foc_time.h。
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  "未设置" 哨兵（与 SimpleFOC 的 NOT_SET 一致）                              */
/* -------------------------------------------------------------------------- */

#define FOC_NOT_SET         (-12345.0f)

/** @brief 判断一个可选项是否被设置过 */
static inline bool foc_is_set(float v)
{
    return v != FOC_NOT_SET;
}
