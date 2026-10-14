/*
 * ============================================================================
 *  balancecar — 板级引脚定义
 *
 *  ★ 换硬件 / 改接线，只改这一个文件，其它代码一律不用动。
 *    所有引脚都以宏定义，禁止在别处出现裸引脚号。
 *
 *  硬件: ESP32-P4  +  灯哥 FOC V3 驱动板 (L6234 + INA240 x2)  +  AS5600 编码器
 * ============================================================================
 */

#pragma once

#include "driver/gpio.h"
#include "hal/adc_types.h"

/* ============================================================================
 *  1. 三相电机驱动 (L6234: IN1/IN2/IN3 + EN)
 * ============================================================================ */

#define MOTOR_PHASE_A_GPIO      GPIO_NUM_32
#define MOTOR_PHASE_B_GPIO      GPIO_NUM_27
#define MOTOR_PHASE_C_GPIO      GPIO_NUM_26

/* 使能脚 —— 注意: 板子上有两个标签，必须用 MOTOR_EN，不是 EN
 *   EN       = ESP32 模块的芯片复位脚，与电机无关
 *   MOTOR_EN = L6234 驱动器的使能脚          ← 用这个
 * 高电平使能 */
#define MOTOR_EN_GPIO           GPIO_NUM_33
#define MOTOR_EN_ACTIVE_LEVEL   1

/* ---- PWM ---- */
/* ESP32-P4 的 MCPWM 时钟源是 PLL_F160M = 160MHz
 * 30kHz 时 period_ticks = 160e6/30e3 = 5333，计数器峰值 2666 */
#define MOTOR_PWM_RESOLUTION_HZ 160000000
#define MOTOR_PWM_FREQ_HZ       30000

/* 中心对齐: period_ticks 是整个周期的 tick 数 */
#define MOTOR_PWM_PERIOD_TICKS  (MOTOR_PWM_RESOLUTION_HZ / MOTOR_PWM_FREQ_HZ)
/* 占空比换算: compare = duty * PEAK */
#define MOTOR_PWM_PEAK_TICKS    (MOTOR_PWM_PERIOD_TICKS / 2)

/* ============================================================================
 *  2. 电流采样 (INA240 x2 -> ADC1)
 *
 *  板子:  M0_OUT1_CS -> P4 GPIO21,  M0_OUT2_CS -> P4 GPIO20
 *  P4 的 ADC 通道:  GPIO20 = ADC1_CH4,  GPIO21 = ADC1_CH5
 *  两个都在 ADC1，可以同步采样，没有跨单元相位误差
 *  第三相电流由基尔霍夫定律得出:  Ic = -Ia - Ib
 * ============================================================================ */

#define BOARD_CS_A_GPIO         GPIO_NUM_21     /* ADC1_CH5 */
#define BOARD_CS_B_GPIO         GPIO_NUM_20     /* ADC1_CH4 */

/* ADC 单元 / 通道 / 衰减 / 位宽 —— 全部用枚举宏，不出现裸数字 */
#define BOARD_CS_ADC_UNIT       ADC_UNIT_1
#define BOARD_CS_A_CHANNEL      ADC_CHANNEL_5
#define BOARD_CS_B_CHANNEL      ADC_CHANNEL_4
#define BOARD_CS_ATTEN          ADC_ATTEN_DB_12     /* 量程约 0 ~ 3.3V */
#define BOARD_CS_BITWIDTH       ADC_BITWIDTH_12

/* INA240A2 增益 50V/V，采样电阻 0.01Ω
 * 来源: DengFOC 源码 InlineCurrent.cpp
 *     _shunt_resistor = 0.01;
 *     amp_gain = 50;
 *     volts_to_amps_ratio = 1.0 / 0.01 / 50 = 2.0 A/V
 * 灵敏度 = 0.01Ω x 50 = 0.50 V/A */
#define BOARD_CS_SHUNT_OHMS     0.01f
#define BOARD_CS_AMP_GAIN       50.0f
/* 灵敏度 (V/A) */
#define BOARD_CS_SENS_V_PER_A   (BOARD_CS_SHUNT_OHMS * BOARD_CS_AMP_GAIN)

/* ============================================================================
 *  3. AS5600 磁编码器 (I2C)
 * ============================================================================ */

#define BOARD_I2C_PORT          0               /* = I2C_NUM_0 */
#define BOARD_I2C_SDA_GPIO      GPIO_NUM_5
#define BOARD_I2C_SCL_GPIO      GPIO_NUM_49
#define BOARD_I2C_FREQ_HZ       400000          /* 400kHz 快速模式 */

#define ENCODER_I2C_ADDR        0x36            /* AS5600 7bit 地址 */

/* ============================================================================
 *  4. 电机参数
 * ============================================================================ */

/* 极对数 (pole pairs) —— 云台电机常见 7 对极 */
#define MOTOR_POLE_PAIRS        7

/* 编码器安装方向: +1 或 -1，视磁铁转向而定 */
#define ENCODER_DIRECTION       (+1)
