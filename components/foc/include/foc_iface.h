/*
 * ============================================================================
 *  foc_iface —— FOC 与硬件之间的接口（函数指针）
 *
 *  对应 SimpleFOC 里的 BLDCDriver3PWM / Sensor / CurrentSense 三个抽象基类。
 *  用函数指针做依赖倒置：foc_motor 不直接依赖 motor/encoder/current_sense，
 *  具体绑定放在 foc_hw.c。
 * ============================================================================
 */

#pragma once

#include <stdbool.h>

#include "esp_err.h"
#include "foc_math.h"

/* -------------------------------------------------------------------------- */
/*  三相驱动接口（BLDCDriver3PWM 等价物）                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief 三相驱动接口
 * @note set_phase_voltage 收到的是【相电压，单位 V，范围 0 ~ voltage_limit】，
 *       由实现自己换算成占空比。
 */
typedef struct {
    void      *ctx;                                      /**< 具体驱动的上下文 */
    float      voltage_limit;                            /**< 可用相电压上限 (V) */
    esp_err_t (*set_phase_voltage)(void *ctx, float ua, float ub, float uc, float vlimit);
    esp_err_t (*enable)(void *ctx, bool en);
    bool      (*is_enabled)(void *ctx);
} foc_driver_t;

/* -------------------------------------------------------------------------- */
/*  传感器接口（Sensor 等价物）                                                 */
/* -------------------------------------------------------------------------- */

/** @brief 位置/速度传感器接口 */
typedef struct {
    void  *ctx;
    void   (*update)(void *ctx);                 /**< 更新一次读数 */
    float  (*get_angle)(void *ctx);              /**< 多圈机械角 (rad) */
    float  (*get_mechanical_angle)(void *ctx);   /**< 单圈机械角 0~2π (rad) */
    float  (*get_velocity)(void *ctx);           /**< 机械角速度 (rad/s) */
} foc_sensor_t;

/* -------------------------------------------------------------------------- */
/*  电流采样接口（CurrentSense 等价物）                                         */
/* -------------------------------------------------------------------------- */

/** @brief 电流采样接口 */
typedef struct {
    void *ctx;
    void  (*update)(void *ctx);                          /**< 刷新一次采样 */
    bool  (*read_phase)(void *ctx, foc_abc_t *out_abc);  /**< 读三相电流 (A) */
    /* 对齐用（对应 SimpleFOC 的 gain_a/gain_b 与 pinA/pinB 交换）*/
    void  (*set_sign_a)(void *ctx, float sign);          /**< 可空 */
    void  (*set_sign_b)(void *ctx, float sign);          /**< 可空 */
    void  (*set_swap)(void *ctx, bool swap);             /**< 可空 */
} foc_current_t;
