/*
 * ============================================================================
 *  foc_lpf —— 一阶低通滤波器
 *
 *  逐行移植自 SimpleFOC src/common/lowpass_filter.cpp，行为保持一致：
 *    * y = alpha*y_prev + (1-alpha)*x，alpha = Tf/(Tf+dt)
 *    * Ts 未设置时用两次调用的实际时间差
 *    * dt < 0 时回退 1ms；dt > 0.3s 时判定为断流，直接输出当前值并重置
 * ============================================================================
 */

#pragma once

#include "foc_defs.h"

/** @brief 低通滤波器状态（成员内部使用） */
typedef struct {
    float   Tf;             /**< 时间常数 (s) */
    float   Ts;             /**< 固定采样时间 (s)；FOC_NOT_SET 表示自适应 */
    float   y_prev;         /**< 上一次滤波输出 */
    int64_t timestamp_prev; /**< 上一次调用的时间戳 (us) */
} foc_lpf_t;

/**
 * @brief 初始化低通滤波器
 * @param lpf 句柄（调用者提供存储）
 * @param Tf  时间常数 (s)
 * @param Ts  固定采样时间 (s)；不启用传 FOC_NOT_SET
 */
void foc_lpf_init(foc_lpf_t *lpf, float Tf, float Ts);

/** @brief 复位内部状态 */
void foc_lpf_reset(foc_lpf_t *lpf);

/** @brief 运行一次，返回滤波后的值 */
float foc_lpf_run(foc_lpf_t *lpf, float x);
