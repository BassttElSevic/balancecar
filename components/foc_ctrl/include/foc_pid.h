/*
 * ============================================================================
 *  foc_pid —— PID 控制器
 *
 *  逐行移植自 SimpleFOC src/common/pid.cpp，行为保持一致：
 *    * 积分项用 Tustin（梯形）离散化
 *    * 积分抗饱和：积分项本身与总输出都按 limit 限幅
 *    * 可选输出斜坡 output_ramp（单位: 输出/秒）
 *    * Ts 未设置时用两次调用的实际时间差；dt<=0 或 dt>0.5s 时回退到 1ms
 * ============================================================================
 */

#pragma once

#include <stdbool.h>

#include "foc_defs.h"

/** @brief PID 状态（成员内部使用） */
typedef struct {
    /* ---- 参数 ---- */
    float P;                /**< 比例增益 */
    float I;                /**< 积分增益 */
    float D;                /**< 微分增益 */
    float output_ramp;      /**< 输出最大变化率；FOC_NOT_SET 表示不限 */
    float limit;            /**< 输出限幅；FOC_NOT_SET 表示不限 */
    float Ts;               /**< 固定采样时间 (s)；FOC_NOT_SET 表示自适应 */

    /* ---- 内部状态 ---- */
    float error_prev;       /**< 上一次误差 */
    float output_prev;      /**< 上一次输出 */
    float integral_prev;    /**< 上一次积分值 */
    int64_t timestamp_prev; /**< 上一次调用的时间戳 (us) */
} foc_pid_t;

/**
 * @brief 初始化 PID
 * @param pid   句柄（调用者提供存储）
 * @param P     比例增益
 * @param I     积分增益
 * @param D     微分增益
 * @param ramp  输出最大变化率；不启用传 FOC_NOT_SET
 * @param limit 输出限幅；不启用传 FOC_NOT_SET
 * @param Ts    固定采样时间 (s)；不启用传 FOC_NOT_SET
 */
void foc_pid_init(foc_pid_t *pid, float P, float I, float D,
                  float ramp, float limit, float Ts);

/** @brief 复位内部状态（积分/上一次误差/上一次输出清零） */
void foc_pid_reset(foc_pid_t *pid);

/** @brief 运行一次 PID，入参为误差，返回输出 */
float foc_pid_run(foc_pid_t *pid, float error);

/** @brief 只改参数，不动状态（用于在线调参） */
void foc_pid_set_gains(foc_pid_t *pid, float P, float I, float D);
