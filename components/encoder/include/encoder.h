/*
 * ============================================================================
 *  balancecar — AS5600 磁编码器驱动 (I2C)
 *
 *  AS5600 是 12 位绝对角度磁编码器:
 *      - 每圈 4096 个计数
 *      - I2C 地址 0x36
 *      - 角度寄存器 0x0E/0x0F (高字节在前)
 *
 *  本驱动在单圈角度基础上做了多圈累加，并提供差分+低通后的角速度，
 *  供后续 FOC 电角度计算和位置/速度环使用。
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief 初始化 I2C 总线与 AS5600（使用 board_pins.h 里的默认引脚）
 */
esp_err_t encoder_init(void);

/**
 * @brief 用指定的 SDA/SCL 引脚初始化（用于扫描排查接线）
 */
esp_err_t encoder_init_pins(int sda_gpio, int scl_gpio);

/** @brief 释放当前 I2C 总线（扫描时切换引脚用） */
esp_err_t encoder_deinit(void);

/**
 * @brief 读取一次编码器并刷新内部状态
 * @return 当前机械角度（弧度，多圈累计）
 */
float encoder_update(void);

/** @brief 机械角度（弧度，多圈累计，可正可负） */
float encoder_get_angle(void);

/** @brief 机械角速度（rad/s），已低通滤波 */
float encoder_get_velocity(void);

/** @brief 单圈机械角度 0 ~ 2π */
float encoder_get_single_turn(void);

/** @brief 12 位原始计数 0 ~ 4095（经零点/方向编程后的角度寄存器 0x0E） */
uint16_t encoder_get_raw(void);

/** @brief 磁场原始角度 (0x0C) —— 不受任何编程影响，磁铁转就一定变 */
uint16_t encoder_get_raw_angle(void);

/** @brief AGC 自动增益 (0x1A)，典型值 0~128；饱和表示磁场超范围 */
uint8_t  encoder_get_agc(void);

/** @brief STATUS 寄存器 (0x0B)：bit5=MD 检测到磁铁, bit4=ML 太弱, bit3=MH 太强 */
uint8_t  encoder_get_status(void);

/** @brief 磁场幅值 (0x1B/0x1C) */
uint16_t encoder_get_magnitude(void);

/** @brief 编码器是否通信正常 */
bool encoder_is_ok(void);

/** @brief 把当前位置设为零点 */
void encoder_set_zero(void);

/** @brief 打印诊断信息（磁铁状态、原始值等） */
void encoder_print_diag(void);

/** @brief dump AS5600 全部关键寄存器（ZMCO/ZPOS/MPOS/MANG/CONF/STATUS/RAW/ANGLE/AGC/MAG/BURN） */
void encoder_dump_regs(void);
