/*
 * ============================================================================
 *  foc_hw —— 把本板的 motor / encoder / current_sense 绑到 FOC 的接口上
 *
 *  对应 SimpleFOC 里的:
 *      BLDCDriver3PWM      -> foc_hw_driver()   (包 motor.c 的 MCPWM)
 *      编码器 Sensor       -> foc_hw_sensor()   (包 encoder.c 的 AS5600)
 *      InlineCurrentSense  -> foc_hw_current()  (包 current_sense.c 的 INA240 x2)
 *
 *  这一层只做"转发"，不含任何算法。
 * ============================================================================
 */

#pragma once

#include "esp_err.h"

#include "foc_iface.h"
#include "encoder.h"
#include "current_sense.h"

/* -------------------------------------------------------------------------- */
/*  三相驱动（motor.c / MCPWM）                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief 取驱动接口
 * @note 本驱动无实例句柄，ctx 为 NULL；
 *       voltage_limit 取 board_pins.h 的 MOTOR_SUPPLY_V
 */
const foc_driver_t *foc_hw_driver(void);

/* -------------------------------------------------------------------------- */
/*  传感器（encoder.c / AS5600）                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief 把 encoder 句柄包成传感器接口
 * @param out  调用者提供的接口存储
 * @param enc  已 encoder_init 的编码器句柄
 */
void foc_hw_sensor_init(foc_sensor_t *out, encoder_t *enc);

/* -------------------------------------------------------------------------- */
/*  电流采样（current_sense.c / INA240 x2）                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief 把 current_sense 句柄包成电流接口
 * @param out  调用者提供的接口存储
 * @param cs   已 current_sense_init 的电流采样句柄
 */
void foc_hw_current_init(foc_current_t *out, current_sense_t *cs);
