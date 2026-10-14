/*
 * ============================================================================
 *  foc_math —— FOC 基础数学
 *
 *  移植自 SimpleFOC:  src/common/foc_utils.h / foc_utils.cpp
 *                    src/common/base_classes/CurrentSense.cpp (Clarke/Park)
 *                    src/BLDCMotor.cpp (反 Park / 反 Clarke)
 *
 *  纯计算，无外设依赖，不使用堆。
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*  常量（与 SimpleFOC foc_utils.h 一致）                                      */
/* -------------------------------------------------------------------------- */

#define FOC_PI              3.14159265359f
#define FOC_PI_2            1.57079632679f
#define FOC_PI_3            1.0471975512f
#define FOC_PI_6            0.52359877559f
#define FOC_2PI             6.28318530718f
#define FOC_3PI_2           4.71238898038f
#define FOC_SQRT2           1.41421356237f
#define FOC_SQRT3           1.73205080757f
#define FOC_SQRT3_2         0.86602540378f
#define FOC_1_SQRT3         0.57735026919f
#define FOC_2_SQRT3         1.15470053838f
#define FOC_120_D2R         2.09439510239f
#define FOC_RPM_TO_RADS     0.10471975512f

/* -------------------------------------------------------------------------- */
/*  向量结构                                                                   */
/* -------------------------------------------------------------------------- */

/** @brief dq 轴分量（电压或电流） */
typedef struct {
    float d;
    float q;
} foc_dq_t;

/** @brief αβ 静止坐标系分量 */
typedef struct {
    float alpha;
    float beta;
} foc_ab_t;

/** @brief 三相分量 */
typedef struct {
    float a;
    float b;
    float c;
} foc_abc_t;

/* -------------------------------------------------------------------------- */
/*  标量工具                                                                   */
/* -------------------------------------------------------------------------- */

/** @brief 限幅 */
static inline float foc_clampf(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/** @brief 符号：x<0 -> -1，x>0 -> +1，x==0 -> 0 */
static inline int foc_signf(float x)
{
    return (x < 0.0f) ? -1 : ((x > 0.0f) ? 1 : 0);
}

/** @brief 把弧度归一化到 [0, 2π) */
float foc_normalize_angle(float angle);

/**
 * @brief 电角度
 * @param shaft_angle 机械角 (rad)
 * @param pole_pairs  极对数
 */
float foc_electrical_angle(float shaft_angle, int pole_pairs);

/* -------------------------------------------------------------------------- */
/*  三角函数（65 点查找表 + 线性插值，与 SimpleFOC 同款）                       */
/* -------------------------------------------------------------------------- */

/** @brief 正弦，入参建议在 [0, 2π) */
float foc_sin(float a);

/** @brief 余弦 */
float foc_cos(float a);

/** @brief 一次同时算出 sin / cos */
void foc_sincos(float a, float *out_sin, float *out_cos);

/** @brief atan2（多项式近似，来自 ODrive） */
float foc_atan2(float y, float x);

/* -------------------------------------------------------------------------- */
/*  坐标变换                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief Clarke 变换：三相电流 -> αβ
 * @note 与 SimpleFOC CurrentSense::getABCurrents 的两相测量分支一致:
 *         i_alpha = i.a
 *         i_beta  = 1/√3 * i.a + 2/√3 * i.b
 */
foc_ab_t foc_clarke(foc_abc_t iabc);

/**
 * @brief Park 变换：αβ -> dq
 * @note SimpleFOC: d = alpha*cos + beta*sin ; q = beta*cos - alpha*sin
 */
foc_dq_t foc_park(foc_ab_t iab, float angle_el);

/**
 * @brief 反 Park：dq -> αβ
 * @note SimpleFOC BLDCMotor::setPhaseVoltage:
 *         Ualpha = cos*Ud - sin*Uq
 *         Ubeta  = sin*Ud + cos*Uq
 */
foc_ab_t foc_inverse_park(foc_dq_t udq, float angle_el);

/**
 * @brief 反 Clarke：αβ -> 三相
 * @note SimpleFOC: Ua = Ualpha
 *                  Ub = -0.5*Ualpha + √3/2*Ubeta
 *                  Uc = -0.5*Ualpha - √3/2*Ubeta
 */
foc_abc_t foc_inverse_clarke(foc_ab_t uab);
