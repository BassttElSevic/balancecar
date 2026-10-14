/*
 * ============================================================================
 *  foc_ctrl —— 时间源
 *
 *  SimpleFOC 用 _micros()（time_utils.h）。这里是 ESP-IDF 下的等价实现，
 *  单独抽出来，方便以后换源（比如换成 ISR 里的 systimer）。
 * ============================================================================
 */

#pragma once

#include <stdint.h>
#include "esp_timer.h"

/** @brief 微秒时间戳 */
static inline int64_t foc_micros(void)
{
    return esp_timer_get_time();
}
