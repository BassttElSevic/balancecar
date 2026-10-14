/*
 * ============================================================================
 *  foc_motor —— FOC 电机对象（BLDCMotor / FOCMotor 的移植）
 *
 *  四个环 + 调制，全部按 SimpleFOC 的结构分层：
 *
 *      控制周期:
 *          foc_motor_loop_foc(&m);        // 1. 采样传感器与电流, 2. 电流环, 3. 调制输出
 *          foc_motor_move(&m, target);    // 4. 按 motion 方式算目标 -> 速度环/位置环/力矩环
 *
 *      四个环（都单独暴露，方便单独调试/复用）:
 *          foc_loop_torque()      力矩环
 *          foc_loop_current_dq()  电流环（dq，对应 FOC_TORQUE_FOC_CURRENT）
 *          foc_loop_current_dc()  电流环（幅值，对应 FOC_TORQUE_DC_CURRENT）
 *          foc_loop_velocity()    速度环
 *          foc_loop_angle()       位置环
 *
 *  存储由调用者提供，不使用堆。
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "foc_math.h"
#include "foc_pid.h"
#include "foc_lpf.h"
#include "foc_types.h"
#include "foc_iface.h"

/* -------------------------------------------------------------------------- */
/*  默认参数（取 SimpleFOC src/common/defaults.h 的值）                         */
/* -------------------------------------------------------------------------- */

#define FOC_DEF_POWER_SUPPLY        12.0f    /**< DEF_POWER_SUPPLY */

#define FOC_DEF_PID_VEL_P           0.5f     /**< DEF_PID_VEL_P */
#define FOC_DEF_PID_VEL_I           10.0f    /**< DEF_PID_VEL_I */
#define FOC_DEF_PID_VEL_D           0.0f     /**< DEF_PID_VEL_D */
#define FOC_DEF_PID_VEL_RAMP        FOC_NOT_SET
#define FOC_DEF_PID_VEL_LIMIT       FOC_DEF_POWER_SUPPLY

#define FOC_DEF_PID_CURR_P          3.0f     /**< DEF_PID_CURR_P (非 AVR 平台) */
#define FOC_DEF_PID_CURR_I          300.0f   /**< DEF_PID_CURR_I */
#define FOC_DEF_PID_CURR_D          0.0f     /**< DEF_PID_CURR_D */
#define FOC_DEF_PID_CURR_RAMP       0.0f     /**< DEF_PID_CURR_RAMP */
#define FOC_DEF_PID_CURR_LIMIT      FOC_DEF_POWER_SUPPLY

#define FOC_DEF_CURRENT_LIM         2.0f     /**< DEF_CURRENT_LIM */
#define FOC_DEF_CURR_BANDWIDTH      300.0f   /**< DEF_CURR_BANDWIDTH */
#define FOC_DEF_CURR_FILTER_TF      (1.0f / (FOC_2PI * FOC_DEF_CURR_BANDWIDTH))

#define FOC_DEF_P_ANGLE_P           20.0f    /**< DEF_P_ANGLE_P */
#define FOC_DEF_VEL_LIM             20.0f    /**< DEF_VEL_LIM */

#define FOC_DEF_VEL_FILTER_TF       0.005f   /**< DEF_VEL_FILTER_Tf */
#define FOC_DEF_VOLTAGE_SENSOR_ALIGN 3.0f    /**< DEF_VOLTAGE_SENSOR_ALIGN */

/* 传感器对齐：扫描的电周期步数（SimpleFOC 用 500） */
#define FOC_ALIGN_STEPS             500
#define FOC_ALIGN_STEP_DELAY_MS     2
#define FOC_ALIGN_SETTLE_MS         200
/* d/q 轴系偏差修正：对齐算出的零点若与实际 q 轴差一个固定角，
 * 就用它补上（实测确定：命令 q 轴电压而响应全落在 d 轴上 -> 差 90°）*/
#define FOC_ALIGN_ZERO_OFFSET_DEFAULT   0.0f

/* 电流采样对齐（对应 SimpleFOC CurrentSense::alignBLDCDriver）*/
#define FOC_CS_ALIGN_RAMP_STEPS  100     /**< 单相升压斜坡步数 */
#define FOC_CS_ALIGN_STEP_MS     3       /**< 每步延时 */
#define FOC_CS_ALIGN_SETTLE_MS   500     /**< 升压后稳定时间 */
#define FOC_CS_ALIGN_AVG_N       10      /**< 取平均次数 */
#define FOC_CS_ALIGN_AVG_MS      10      /**< 取平均间隔 */
#define FOC_CS_ALIGN_MIN_A       0.1f    /**< 判定有效的最小电流 (A) */
#define FOC_CS_ALIGN_MIN_RATIO   1.5f    /**< 两路电流幅值比的下限 */

/* 极对数自检容差 (rad)：实测转动 x 极对数 与 2π 之差超过它就告警 */
#define FOC_PP_CHECK_TOLERANCE  0.5f

/** 方向标定时每隔多少步打印一次进度（0=不打印）*/
#define FOC_ALIGN_PROGRESS_STEPS  100

/* 传感器方向（对应官方 Direction 枚举）*/
#define FOC_SENSOR_DIR_UNKNOWN   0.0f   /**< 自动判定 */
#define FOC_SENSOR_DIR_CW        1.0f   /**< 顺时针 */
#define FOC_SENSOR_DIR_CCW      -1.0f   /**< 逆时针 */

/* 中点对齐调制的中心偏移比例（占驱动器相电压上限）*/
#define FOC_MOD_CENTER_RATIO    0.5f

/* 对齐电压相对相电压上限的比例（SimpleFOC 默认 3V 是给 12V 母线用的，
 * 这里改成按比例，换母线不用改代码）*/
#define FOC_ALIGN_VOLTAGE_RATIO     0.7f

/* -------------------------------------------------------------------------- */
/*  配置                                                                       */
/* -------------------------------------------------------------------------- */

/** @brief FOC 电机配置 */
typedef struct {
    /* ---- 电机本体 ---- */
    int      pole_pairs;             /**< 极对数 */
    float    voltage_limit;          /**< 相电压上限 (V) */
    float    current_limit;          /**< 电流上限 (A) */
    float    velocity_limit;         /**< 速度上限 (rad/s) */
    float    voltage_sensor_align;   /**< 传感器对齐时施加的电压 (V) */
    float    align_zero_offset;      /**< 对齐零点附加偏移 (rad)，修正 d/q 轴系偏差 */
    float    sensor_direction;       /**< 0=自动判定, +1=CW, -1=CCW（对应官方 motor.sensor_direction）*/

    /* ---- 控制方式 ---- */
    foc_torque_ctrl_t torque_ctrl;   /**< 力矩环方式 */
    foc_motion_t      motion;        /**< 运动控制方式 */
    foc_modulation_t  modulation;    /**< 调制方式 */
    bool              modulation_centered; /**< 是否中点对齐调制 */

    /* ---- 速度环 PID ---- */
    float pid_vel_p, pid_vel_i, pid_vel_d, pid_vel_ramp, pid_vel_limit;

    /* ---- 电流环 PID ---- */
    float pid_cur_p, pid_cur_i, pid_cur_d, pid_cur_ramp, pid_cur_limit;

    /* ---- 位置环 P ---- */
    float p_angle_p;

    /* ---- 低通滤波时间常数 (s) ---- */
    float lpf_velocity_tf;
    float lpf_current_tf;
    float lpf_angle_tf;
} foc_motor_cfg_t;

/** @brief 用 SimpleFOC 的默认值填满配置 */
void foc_motor_default_cfg(foc_motor_cfg_t *cfg);

/* -------------------------------------------------------------------------- */
/*  句柄（成员内部使用，只读；改参数请用 foc_motor_set_xxx）                    */
/* -------------------------------------------------------------------------- */

/** @brief FOC 电机句柄 */
typedef struct {
    /* ---- 关联的外设（可为 NULL） ---- */
    const foc_driver_t  *driver;
    const foc_sensor_t  *sensor;
    const foc_current_t *current_sense;

    /* ---- 配置副本 ---- */
    int      pole_pairs;
    float    voltage_limit;
    float    current_limit;
    float    velocity_limit;
    float    voltage_sensor_align;
    float    align_zero_offset;       /**< 对齐零点附加偏移 (rad) */
    foc_torque_ctrl_t torque_ctrl;
    foc_motion_t      motion;
    foc_modulation_t  modulation;
    bool     modulation_centered;

    /* ---- 对齐结果 ---- */
    float    sensor_direction;       /**< +1 / -1 */
    float    zero_electric_angle;    /**< 电角度零点 (rad) */
    bool     aligned;

    /* ---- 运行状态 ---- */
    float    target;                 /**< 当前目标（含义随 motion 变化） */
    float    shaft_angle;            /**< 机械角 (rad) */
    float    shaft_velocity;         /**< 机械角速度 (rad/s) */
    float    electrical_angle;       /**< 电角度 (rad) */
    float    current_sp;             /**< 目标电流/电压 */
    float    shaft_velocity_sp;      /**< 目标速度 */
    float    shaft_angle_sp;         /**< 目标角度 */
    float    feed_forward_velocity;  /**< 前馈速度 */
    foc_dq_t voltage;                /**< 待施加的 dq 电压 */
    foc_dq_t current;                /**< 实测 dq 电流（已低通） */
    foc_abc_t current_abc;           /**< 实测三相电流（原始，供监视） */
    float    current_magnitude;      /**< dc_current 模式用的电流幅值 */

    /* ---- 控制器 ---- */
    foc_pid_t pid_current_q;
    foc_pid_t pid_current_d;
    foc_lpf_t lpf_current_q;
    foc_lpf_t lpf_current_d;
    foc_pid_t pid_velocity;
    foc_pid_t p_angle;
    foc_lpf_t lpf_velocity;
    foc_lpf_t lpf_angle;

    /* ---- 内部 ---- */
    bool      initialized;
    int64_t   open_loop_timestamp;
} foc_motor_t;

/* -------------------------------------------------------------------------- */
/*  生命周期                                                                   */
/* -------------------------------------------------------------------------- */

/**
 * @brief 初始化 FOC 电机
 * @param m        调用者提供的句柄
 * @param cfg      配置
 * @param driver   驱动接口（必填）
 * @param sensor   传感器接口（开环模式可传 NULL）
 * @param current  电流采样接口（力矩方式为电压时可传 NULL）
 */
esp_err_t foc_motor_init(foc_motor_t *m, const foc_motor_cfg_t *cfg,
                         const foc_driver_t *driver,
                         const foc_sensor_t *sensor,
                         const foc_current_t *current);

/** @brief 关闭驱动并复位控制器 */
esp_err_t foc_motor_deinit(foc_motor_t *m);

/* -------------------------------------------------------------------------- */
/*  参数设置                                                                   */
/* -------------------------------------------------------------------------- */

void foc_motor_set_torque_ctrl(foc_motor_t *m, foc_torque_ctrl_t t);
void foc_motor_set_motion(foc_motor_t *m, foc_motion_t v);
void foc_motor_set_modulation(foc_motor_t *m, foc_modulation_t mod, bool centered);
void foc_motor_set_voltage_limit(foc_motor_t *m, float v);
void foc_motor_set_current_limit(foc_motor_t *m, float a);
void foc_motor_set_velocity_limit(foc_motor_t *m, float w);
void foc_motor_set_velocity_pid(foc_motor_t *m, float p, float i, float d);
void foc_motor_set_current_pid(foc_motor_t *m, float p, float i, float d);
void foc_motor_set_angle_p(foc_motor_t *m, float p);

/* -------------------------------------------------------------------------- */
/*  传感器对齐                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief 找电角度零点与传感器方向（SimpleFOC 的 alignSensor）
 * @note 会施加 voltage_sensor_align 的电压让转子对齐，必须是空载
 */
esp_err_t foc_motor_align_sensor(foc_motor_t *m);

/**
 * @brief 电流采样对齐（SimpleFOC FOCMotor::alignCurrentSense -> alignBLDCDriver）
 *
 * 单独给某一相加电压并读两路采样，判定：
 *   - 哪一个采样通道对应哪一相（需要时自动交换）
 *   - 每相采样的极性（需要时自动翻转符号）
 * 不需要按住转子。
 */
esp_err_t foc_motor_align_current_sense(foc_motor_t *m);

/** @brief 当前电角度（含 zero_electric_angle 与方向） */
float foc_motor_electrical_angle(const foc_motor_t *m);

/* -------------------------------------------------------------------------- */
/*  四个环                                                                     */
/* -------------------------------------------------------------------------- */

/** @brief 力矩环：把 target 解释成电压或目标电流（SimpleFOC MotionControlType::torque） */
float foc_loop_torque(foc_motor_t *m);

/** @brief 电流环（dq）：Park 变换 + d/q 双 PI（SimpleFOC TorqueControlType::foc_current） */
float foc_loop_current_dq(foc_motor_t *m);

/** @brief 电流环（幅值）：单个 PI（SimpleFOC TorqueControlType::dc_current） */
float foc_loop_current_dc(foc_motor_t *m);

/** @brief 速度环：PI（SimpleFOC MotionControlType::velocity） */
float foc_loop_velocity(foc_motor_t *m);

/** @brief 位置环：P 外环 -> 速度环（SimpleFOC MotionControlType::angle） */
float foc_loop_angle(foc_motor_t *m);

/** @brief 位置单环：直接 P -> 电流/电压（SimpleFOC MotionControlType::angle_nocascade） */
float foc_loop_angle_nocascade(foc_motor_t *m);

/* -------------------------------------------------------------------------- */
/*  主循环                                                                     */
/* -------------------------------------------------------------------------- */

/**
 * @brief 采样 + 电流环 + 调制输出（SimpleFOC BLDCMotor::loopFOC）
 * @note 每个控制周期调用一次，且必须在 foc_motor_move() 之前
 */
void foc_motor_loop_foc(foc_motor_t *m);

/**
 * @brief 按 motion 方式算目标并下发（SimpleFOC FOCMotor::move）
 * @param new_target 新目标；不需要改目标时传 FOC_NOT_SET
 * @return 当前生效的目标
 */
float foc_motor_move(foc_motor_t *m, float new_target);

/**
 * @brief 按 dq 电压和电角度做反Park + 反Clarke + 调制，然后写驱动
 *        （SimpleFOC BLDCMotor::setPhaseVoltage）
 */
esp_err_t foc_motor_set_phase_voltage(foc_motor_t *m, float uq, float ud, float angle_el);
