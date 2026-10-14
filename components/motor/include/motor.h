/*
 * ============================================================================
 *  balancecar — 三相无刷电机驱动 (MCPWM)
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 初始化三相 PWM 与使能脚
 *
 * 初始化完成后：
 *   - 三相输出为零矢量 (各 50% 占空比)
 *   - 驱动器处于【失能】状态, 必须显式调用 motor_enable(true) 才会输出
 */
esp_err_t motor_init(void);

/**
 * @brief 运行时把三相 PWM 输出切换到另一组 GPIO
 *
 * 内部会删除并重建 MCPWM generator（自动释放旧 GPIO 占用）。
 * 用于排查“到底哪组引脚才是真正的三相输出”。
 */
esp_err_t motor_set_phase_pins(int gpio_a, int gpio_b, int gpio_c);

/**
 * @brief 使能 / 失能 驱动器 (控制 EN 引脚)
 */
void motor_enable(bool en);

/**
 * @brief 直接设置三相占空比
 * @param du,dv,dw  各相占空比, 范围 0.0 ~ 1.0 (会被自动限幅)
 */
esp_err_t motor_set_duty(float du, float dv, float dw);

/**
 * @brief 设置三相归一化相电压 (更适合 FOC/SVPWM 使用)
 * @param u,v,w  范围 -1.0 ~ +1.0, 其中 0.0 对应 50% 占空比 (零矢量)
 */
esp_err_t motor_set_uvw(float u, float v, float w);

/**
 * @brief 输出零矢量 (三相 50%), 电机不产生力矩, 但驱动器仍在工作
 */
esp_err_t motor_zero(void);

/**
 * @brief 失能驱动器, 电机自由滑行 (coast)
 */
void motor_coast(void);
