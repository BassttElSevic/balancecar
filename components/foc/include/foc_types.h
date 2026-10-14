/*
 * ============================================================================
 *  foc_types —— FOC 的枚举类型
 *
 *  对照 SimpleFOC src/common/base_classes/FOCMotor.h 的三个枚举。
 * ============================================================================
 */

#pragma once

#include <stdint.h>

/** @brief 力矩控制方式（SimpleFOC: TorqueControlType） */
typedef enum {
    FOC_TORQUE_VOLTAGE     = 0x00,  /**< 直接给电压 */
    FOC_TORQUE_DC_CURRENT  = 0x01,  /**< 用电流幅值闭环 */
    FOC_TORQUE_FOC_CURRENT = 0x02,  /**< 用 dq 电流闭环 */
} foc_torque_ctrl_t;

/** @brief 运动控制方式（SimpleFOC: MotionControlType） */
typedef enum {
    FOC_MOTION_TORQUE            = 0x00,  /**< 力矩/开环电压 */
    FOC_MOTION_VELOCITY          = 0x01,  /**< 速度闭环 */
    FOC_MOTION_ANGLE             = 0x02,  /**< 位置闭环（内环速度） */
    FOC_MOTION_VELOCITY_OPENLOOP = 0x03,  /**< 速度开环（不需要传感器） */
    FOC_MOTION_ANGLE_OPENLOOP    = 0x04,  /**< 位置开环（不需要传感器） */
    FOC_MOTION_ANGLE_NOCASCADE   = 0x05,  /**< 位置单环（无速度级联） */
} foc_motion_t;

/** @brief 调制方式（SimpleFOC: FOCModulationType） */
typedef enum {
    FOC_MOD_SINE_PWM         = 0x00,  /**< 正弦 PWM */
    FOC_MOD_SPACE_VECTOR_PWM = 0x01,  /**< 空间矢量 PWM（中点钳位） */
} foc_modulation_t;
