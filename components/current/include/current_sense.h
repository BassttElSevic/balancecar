/*
 * ============================================================================
 *  balancecar — 双电阻在线电流采样 (INA240 x2 -> ADC1)
 *
 *  硬件:
 *      A 相采样 -> P4 GPIO21 (ADC1_CH5)
 *      B 相采样 -> P4 GPIO20 (ADC1_CH4)
 *      第三相由基尔霍夫定律:  Ic = -Ia - Ib
 *
 *  INA240A2 增益 50V/V，采样电阻 0.02Ω -> 灵敏度 1.00 V/A
 *
 *  ⚠️ 零点校准必须在【电机不转、无电流】时调用 current_sense_calibrate()
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

/** @brief 初始化 ADC1 双通道并做校准 */
esp_err_t current_sense_init(void);

/** @brief 零点校准：必须在电机静止、输出为零时调用 */
esp_err_t current_sense_calibrate(void);

/** @brief 读一次两相电流（并算出第三相） */
esp_err_t current_sense_update(void);

/** @brief 三相电流 (安培) */
float current_get_a(void);
float current_get_b(void);
float current_get_c(void);

/** @brief 两相采样的原始电压 (mV) —— 用于诊断引脚是否真的接上了 */
int current_get_mv_a(void);
int current_get_mv_b(void);
/** @brief 两相采样的 12bit 原始值 (0~4095) —— 用于区分“ADC 没读到”和“换算错了” */
int current_get_raw_a(void);
int current_get_raw_b(void);
/** @brief 零点电压 (mV) */
float current_get_offset_mv_a(void);
float current_get_offset_mv_b(void);

/** @brief 打印诊断信息 */
void current_sense_print_diag(void);
