/*
 * ============================================================================
 *  encoder —— AS5600 磁编码器驱动
 *
 *  ┌─ I2C 用法 ────────────────────────────────────────────────────────────
 *  │   全部走 components/i2c_bus（其形态照 ESP-IDF 官方例程 i2c_basic）
 *  └──────────────────────────────────────────────────────────────────────
 *
 *  ┌─ 读角度 / 多圈 / 速度的逻辑：照 DengFOC ─────────────────────────────
 *  │   ToanTech/DengFOC_Lib → DengFOC 库/V0.6 支持SVPWM算法/AS5600.cpp
 *  │     getSensorAngle()  读 0x0C, 掩码 (buf[1]&0x1FF) + ((buf[0]&0x1F)<<8)
 *  │                       返回 (readValue / 4096) * 2π
 *  │     Sensor_init()     begin → delay(500) → 连读 3 次
 *  │     Sensor_update()   d_angle = val - angle_prev;
 *  │                       |d_angle| > 0.8*2π 时圈数 ±1
 *  │     getAngle()        full_rotations*2π + angle_prev
 *  │     getVelocity()     (Δfull_rotations*2π + Δangle) / Ts
 *  └──────────────────────────────────────────────────────────────────────
 *
 *  存储由调用者提供（不使用堆）:
 *      static i2c_bus_t s_i2c;
 *      static encoder_t s_enc;
 *      i2c_bus_init(&s_i2c, &bus_cfg);
 *      encoder_init(&s_enc, &s_i2c, &enc_cfg);
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "i2c_bus.h"

/* -------------------------------------------------------------------------- */
/*  AS5600 芯片固有参数                                                        */
/* -------------------------------------------------------------------------- */

/* 寄存器 */
#define AS5600_REG_ZMCO        0x00   /**< 零点/方向编程次数 */
#define AS5600_REG_ZPOS        0x01   /**< 起始位置 (2 字节) */
#define AS5600_REG_MPOS        0x03   /**< 终止位置 (2 字节) */
#define AS5600_REG_MANG        0x05   /**< 最大角度 (2 字节) */
#define AS5600_REG_CONF        0x07   /**< 配置 (2 字节) */
#define AS5600_REG_STATUS      0x0B   /**< bit5 MD / bit4 ML / bit3 MH */
#define AS5600_REG_RAW_ANGLE   0x0C   /**< 磁场原始角度 (2 字节) —— DengFOC 用这个 */
#define AS5600_REG_ANGLE       0x0E   /**< 经零点/方向编程的角度 (2 字节) */
#define AS5600_REG_AGC         0x1A   /**< 自动增益 */
#define AS5600_REG_MAGNITUDE   0x1B   /**< 磁场幅值 (2 字节) */
#define AS5600_REG_BURN        0xFF   /**< 烧录次数 */

#define AS5600_COUNTS_PER_REV  4096   /**< 12bit，每圈 4096 计数 */
#define AS5600_ANGLE_BITS      12
#define AS5600_ANGLE_MASK      0x0FFF

/* STATUS 位 */
#define AS5600_STATUS_MD_SHIFT 5      /**< 检测到磁铁 */
#define AS5600_STATUS_ML_SHIFT 4      /**< 磁场太弱 */
#define AS5600_STATUS_MH_SHIFT 3      /**< 磁场太强 */

/* -------------------------------------------------------------------------- */
/*  配置                                                                       */
/* -------------------------------------------------------------------------- */

/** @brief 编码器配置。为 0 的字段取默认值（括号内为默认值） */
typedef struct {
    uint8_t  addr7;           /**< 7bit 从机地址；用 board_pins.h 的 ENCODER_I2C_ADDR */
    uint32_t freq_hz;         /**< SCL 速率；0 -> 用总线的默认速率 */
    uint16_t settle_ms;       /**< 总线建好后到第一次读的等待；0 -> 500 (DengFOC) */
    int      probe_retry;     /**< 探测重试次数；0 -> 10 */
    uint32_t probe_gap_ms;    /**< 重试间隔；0 -> 50 */
    float    counts_per_rev;  /**< 每圈计数；0 -> 4096 */
    float    direction;       /**< +1 / -1；0 -> +1 */
} encoder_cfg_t;

#define ENCODER_SETTLE_MS_DEFAULT      500u    /**< DengFOC: Wire.begin() 后 delay(500) */
#define ENCODER_PROBE_RETRY_DEFAULT    10
#define ENCODER_PROBE_GAP_MS_DEFAULT   50u

/* -------------------------------------------------------------------------- */
/*  句柄（成员内部使用）                                                        */
/* -------------------------------------------------------------------------- */

/** @brief 编码器句柄（成员内部使用） */
typedef struct {
    i2c_slave_t slave;              /**< 内部 */
    bool        ready;              /**< 内部 */
    bool        ok;                 /**< 内部：最近一次读是否成功 */

    float       counts_per_rev;     /**< 每圈计数 */
    float       direction;          /**< +1 / -1 */

    /* ---- DengFOC Sensor_AS5600 同款状态 ---- */
    float       angle_prev;         /**< 上次机械角 0 ~ 2π */
    int64_t     angle_prev_ts;      /**< 上次时间戳 (us) */
    int32_t     full_rotations;     /**< 累计圈数 */
    float       vel_angle_prev;     /**< 上次算速度时的角度 */
    int64_t     vel_angle_prev_ts;
    int32_t     vel_full_rotations;

    /* ---- 诊断（不参与角度计算） ---- */
    uint16_t    raw_angle;          /**< 0x0C */
    uint16_t    angle_reg;          /**< 0x0E */
    uint8_t     status;             /**< 0x0B */
    uint8_t     agc;                /**< 0x1A */
    uint16_t    magnitude;          /**< 0x1B */
    float       velocity;           /**< 最近一次算出的角速度 rad/s */
    uint32_t    err_count;          /**< 累计读失败次数 */
} encoder_t;

/* -------------------------------------------------------------------------- */
/*  生命周期                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化 AS5600
 * @param enc  调用者提供的句柄
 * @param bus  已初始化的 I2C 总线（其引脚来自 board_pins.h）
 * @param cfg  配置
 */
esp_err_t encoder_init(encoder_t *enc, i2c_bus_t *bus, const encoder_cfg_t *cfg);

/** @brief 反初始化（摘下从机） */
esp_err_t encoder_deinit(encoder_t *enc);

/* -------------------------------------------------------------------------- */
/*  更新与读取                                                                 */
/* -------------------------------------------------------------------------- */

/** @brief 读一次并推进状态（DengFOC 的 Sensor_update）。返回单圈机械角 (rad) */
float encoder_update(encoder_t *enc);

/** @brief 单圈机械角 0 ~ 2π（DengFOC getMechanicalAngle） */
float encoder_get_single_turn(const encoder_t *enc);

/** @brief 多圈累计机械角 (rad)（DengFOC getAngle） */
float encoder_get_angle(const encoder_t *enc);

/** @brief 机械角速度 (rad/s)（DengFOC getVelocity）。会推进速度基准 */
float encoder_get_velocity(encoder_t *enc);

/** @brief 累计圈数（DengFOC full_rotations） */
int32_t encoder_get_full_rotations(const encoder_t *enc);

/** @brief 最近一次通信是否正常 */
bool encoder_is_ok(const encoder_t *enc);

/* -------------------------------------------------------------------------- */
/*  原始值 / 诊断                                                              */
/* -------------------------------------------------------------------------- */

/** @brief 0x0C 磁场原始角度（12bit）—— 不受零点/方向编程影响，磁铁转就一定变 */
uint16_t encoder_get_raw_angle(const encoder_t *enc);

/** @brief 0x0E 经零点/方向编程后的角度（12bit） */
uint16_t encoder_get_angle_reg(const encoder_t *enc);

/** @brief 0x0B STATUS：bit5 MD / bit4 ML / bit3 MH */
uint8_t encoder_get_status(const encoder_t *enc);

/** @brief 0x1A AGC */
uint8_t encoder_get_agc(const encoder_t *enc);

/** @brief 0x1B 磁场幅值 */
uint16_t encoder_get_magnitude(const encoder_t *enc);

/** @brief 累计读失败次数 */
uint32_t encoder_get_err_count(const encoder_t *enc);

/** @brief 把当前多圈角清零 */
void encoder_set_zero(encoder_t *enc);

/** @brief 刷新 0x0E / STATUS / AGC / MAG 这些诊断寄存器（不参与角度计算） */
esp_err_t encoder_refresh_diag(encoder_t *enc);

/** @brief 打印一行自检信息 */
void encoder_print_diag(const encoder_t *enc);

/** @brief dump AS5600 全部关键寄存器 */
void encoder_dump_regs(const encoder_t *enc);
