/*
 * ============================================================================
 *  foc_motor —— 实现（BLDCMotor / FOCMotor 的移植）
 *
 *  对齐关系（SimpleFOC -> 这里）:
 *      FOCMotor::electricalAngle()      -> foc_motor_electrical_angle()
 *      BLDCMotor::setPhaseVoltage()     -> foc_motor_set_phase_voltage()
 *      FOCMotor::loopFOC()              -> foc_motor_loop_foc()
 *      FOCMotor::move()                 -> foc_motor_move()
 *      FOCMotor::alignSensor()          -> foc_motor_align_sensor()
 *      FOCMotor::velocityOpenloop()     -> foc_motor_velocity_openloop()  (static)
 *      FOCMotor::angleOpenloop()        -> foc_motor_angle_openloop()     (static)
 *
 *  四个环单独暴露（对应 SimpleFOC 中 loopFOC 的 switch 分支与 move 的分支）:
 *      foc_loop_torque / foc_loop_current_dq / foc_loop_current_dc /
 *      foc_loop_velocity / foc_loop_angle / foc_loop_angle_nocascade
 * ============================================================================
 */

#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include "foc_motor.h"
#include "foc_time.h"

static const char *TAG = "foc";

#define FOC_OPENLOOP_MAX_DT     0.5f
#define FOC_OPENLOOP_FALLBACK_DT 1e-3f

/* -------------------------------------------------------------------------- */
/*  默认配置                                                                   */
/* -------------------------------------------------------------------------- */

void foc_motor_default_cfg(foc_motor_cfg_t *cfg)
{
    if (!cfg) {
        return;
    }
    cfg->pole_pairs            = 1;
    cfg->voltage_limit         = FOC_DEF_POWER_SUPPLY;
    cfg->current_limit         = FOC_DEF_CURRENT_LIM;
    cfg->velocity_limit        = FOC_DEF_VEL_LIM;
    cfg->voltage_sensor_align  = FOC_DEF_VOLTAGE_SENSOR_ALIGN;
    cfg->align_zero_offset     = FOC_ALIGN_ZERO_OFFSET_DEFAULT;

    cfg->torque_ctrl           = FOC_TORQUE_VOLTAGE;
    cfg->motion                = FOC_MOTION_TORQUE;
    cfg->modulation            = FOC_MOD_SINE_PWM;
    cfg->modulation_centered   = true;

    cfg->pid_vel_p             = FOC_DEF_PID_VEL_P;
    cfg->pid_vel_i             = FOC_DEF_PID_VEL_I;
    cfg->pid_vel_d             = FOC_DEF_PID_VEL_D;
    cfg->pid_vel_ramp          = FOC_DEF_PID_VEL_RAMP;
    cfg->pid_vel_limit         = FOC_DEF_PID_VEL_LIMIT;

    cfg->pid_cur_p             = FOC_DEF_PID_CURR_P;
    cfg->pid_cur_i             = FOC_DEF_PID_CURR_I;
    cfg->pid_cur_d             = FOC_DEF_PID_CURR_D;
    cfg->pid_cur_ramp          = FOC_DEF_PID_CURR_RAMP;
    cfg->pid_cur_limit         = FOC_DEF_PID_CURR_LIMIT;

    cfg->p_angle_p             = FOC_DEF_P_ANGLE_P;

    cfg->lpf_velocity_tf       = FOC_DEF_VEL_FILTER_TF;
    cfg->lpf_current_tf        = FOC_DEF_CURR_FILTER_TF;
    cfg->lpf_angle_tf          = 0.0f;
}

/* -------------------------------------------------------------------------- */
/*  生命周期                                                                   */
/* -------------------------------------------------------------------------- */

/** @brief 把控制器的输出限幅钳到可用相电压之内 */
static void clamp_limits_to_voltage(foc_motor_t *m)
{
    /*
     * 为什么必须钳：
     *   中点对齐调制的相电压是 ua = vlimit/2 + Uq*cos(θ)，
     *   只有 |Uq| <= vlimit/2 时才不会撞到 [0, vlimit] 被削顶。
     *   如果 PID 的输出限幅（SimpleFOC 默认是 DEF_POWER_SUPPLY = 12V）
     *   远大于实际可用电压，积分会一路积到顶，调制器饱和，
     *   磁场矢量失真，电机会剧烈抖动。
     */
    if (!foc_is_set(m->pid_current_q.limit) || m->pid_current_q.limit > m->voltage_limit) {
        ESP_LOGW(TAG, "电流环 PID 限幅 %.2f V 超过相电压上限，已钳到 %.2f V",
                 (double)m->pid_current_q.limit, (double)m->voltage_limit);
        m->pid_current_q.limit = m->voltage_limit;
        m->pid_current_d.limit = m->voltage_limit;
    }
    if (!foc_is_set(m->pid_velocity.limit) || m->pid_velocity.limit > m->voltage_limit) {
        ESP_LOGW(TAG, "速度环 PID 限幅 %.2f V 超过相电压上限，已钳到 %.2f V",
                 (double)m->pid_velocity.limit, (double)m->voltage_limit);
        m->pid_velocity.limit = m->voltage_limit;
    }
    /* 对齐时施加的电压同样不能超过上限（SimpleFOC 默认 3V 是给 12V 母线用的）*/
    if (m->voltage_sensor_align > m->voltage_limit) {
        m->voltage_sensor_align = m->voltage_limit * FOC_ALIGN_VOLTAGE_RATIO;
        ESP_LOGW(TAG, "对齐电压超过相电压上限，已改为 %.2f V", (double)m->voltage_sensor_align);
    }
}

esp_err_t foc_motor_init(foc_motor_t *m, const foc_motor_cfg_t *cfg,
                         const foc_driver_t *driver,
                         const foc_sensor_t *sensor,
                         const foc_current_t *current)
{
    ESP_RETURN_ON_FALSE(m && cfg && driver, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    ESP_RETURN_ON_FALSE(!m->initialized, ESP_ERR_INVALID_STATE, TAG, "已初始化");
    ESP_RETURN_ON_FALSE(cfg->pole_pairs > 0, ESP_ERR_INVALID_ARG, TAG, "极对数必须 > 0");

    memset(m, 0, sizeof(*m));
    m->driver = driver;
    m->sensor = sensor;
    m->current_sense = current;

    m->pole_pairs           = cfg->pole_pairs;
    m->voltage_limit        = cfg->voltage_limit;
    m->current_limit        = cfg->current_limit;
    m->velocity_limit       = cfg->velocity_limit;
    m->voltage_sensor_align = cfg->voltage_sensor_align;
    m->align_zero_offset    = cfg->align_zero_offset;
    m->torque_ctrl          = cfg->torque_ctrl;
    m->motion               = cfg->motion;
    m->modulation           = cfg->modulation;
    m->modulation_centered  = cfg->modulation_centered;

    /* 方向未知 / 电角度零点未知 -> 需要 align */
    m->sensor_direction    = 0.0f;
    m->zero_electric_angle = 0.0f;
    m->aligned             = false;

    /* ---- 控制器 ---- */
    foc_pid_init(&m->pid_current_q, cfg->pid_cur_p, cfg->pid_cur_i, cfg->pid_cur_d,
                 cfg->pid_cur_ramp, cfg->pid_cur_limit, FOC_NOT_SET);
    foc_pid_init(&m->pid_current_d, cfg->pid_cur_p, cfg->pid_cur_i, cfg->pid_cur_d,
                 cfg->pid_cur_ramp, cfg->pid_cur_limit, FOC_NOT_SET);
    foc_lpf_init(&m->lpf_current_q, cfg->lpf_current_tf, FOC_NOT_SET);
    foc_lpf_init(&m->lpf_current_d, cfg->lpf_current_tf, FOC_NOT_SET);

    foc_pid_init(&m->pid_velocity, cfg->pid_vel_p, cfg->pid_vel_i, cfg->pid_vel_d,
                 cfg->pid_vel_ramp, cfg->pid_vel_limit, FOC_NOT_SET);
    /* 位置环只用 P（SimpleFOC: P_angle{P, 0, 0, 0, velocity_limit}） */
    foc_pid_init(&m->p_angle, cfg->p_angle_p, 0.0f, 0.0f, 0.0f, cfg->velocity_limit, FOC_NOT_SET);

    foc_lpf_init(&m->lpf_velocity, cfg->lpf_velocity_tf, FOC_NOT_SET);
    foc_lpf_init(&m->lpf_angle, cfg->lpf_angle_tf, FOC_NOT_SET);

    /* 防御：把各控制器限幅钳到实际可用相电压内（见函数里的说明）*/
    clamp_limits_to_voltage(m);

    m->open_loop_timestamp = foc_micros();
    m->initialized         = true;

    ESP_LOGI(TAG, "FOC 初始化: pole_pairs=%d  vlimit=%.1fV  ilimit=%.1fA  wlimit=%.1frad/s  "
                  "torque=%d motion=%d mod=%d%s",
             m->pole_pairs, (double)m->voltage_limit, (double)m->current_limit,
             (double)m->velocity_limit, (int)m->torque_ctrl, (int)m->motion,
             (int)m->modulation, m->modulation_centered ? "(centered)" : "");
    return ESP_OK;
}

esp_err_t foc_motor_deinit(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    if (m->driver && m->driver->enable) {
        (void)m->driver->enable(m->driver->ctx, false);
    }
    m->initialized = false;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  参数设置                                                                   */
/* -------------------------------------------------------------------------- */

void foc_motor_set_torque_ctrl(foc_motor_t *m, foc_torque_ctrl_t t)
{
    if (m) m->torque_ctrl = t;
}

void foc_motor_set_motion(foc_motor_t *m, foc_motion_t v)
{
    if (!m || m->motion == v) {
        return;
    }
    m->motion = v;
    /* 切换运动方式时把控制器状态清掉，避免残留积分 */
    foc_pid_reset(&m->pid_velocity);
    foc_pid_reset(&m->p_angle);
    foc_lpf_reset(&m->lpf_velocity);
}

void foc_motor_set_modulation(foc_motor_t *m, foc_modulation_t mod, bool centered)
{
    if (!m) return;
    m->modulation          = mod;
    m->modulation_centered = centered;
}

void foc_motor_set_voltage_limit(foc_motor_t *m, float v)
{
    if (!m) {
        return;
    }
    m->voltage_limit = v;
    clamp_limits_to_voltage(m);
}

void foc_motor_set_current_limit(foc_motor_t *m, float a)
{
    if (m) m->current_limit = a;
}

void foc_motor_set_velocity_limit(foc_motor_t *m, float w)
{
    if (!m) return;
    m->velocity_limit = w;
    m->p_angle.limit  = w;
}

void foc_motor_set_velocity_pid(foc_motor_t *m, float p, float i, float d)
{
    if (m) foc_pid_set_gains(&m->pid_velocity, p, i, d);
}

void foc_motor_set_current_pid(foc_motor_t *m, float p, float i, float d)
{
    if (!m) return;
    foc_pid_set_gains(&m->pid_current_q, p, i, d);
    foc_pid_set_gains(&m->pid_current_d, p, i, d);
}

void foc_motor_set_angle_p(foc_motor_t *m, float p)
{
    if (m) m->p_angle.P = p;
}

/* -------------------------------------------------------------------------- */
/*  电角度                                                                     */
/* -------------------------------------------------------------------------- */

float foc_motor_electrical_angle(const foc_motor_t *m)
{
    if (!m || !m->sensor) {
        return m ? m->electrical_angle : 0.0f;
    }
    /* SimpleFOC: normalize( sensor_direction * pole_pairs * getMechanicalAngle() - zero_electric_angle ) */
    const float mech = m->sensor->get_mechanical_angle(m->sensor->ctx);
    return foc_normalize_angle((m->sensor_direction * (float)m->pole_pairs) * mech
                               - m->zero_electric_angle);
}

/* -------------------------------------------------------------------------- */
/*  调制（BLDCMotor::setPhaseVoltage）                                          */
/* -------------------------------------------------------------------------- */

esp_err_t foc_motor_set_phase_voltage(foc_motor_t *m, float uq, float ud, float angle_el)
{
    ESP_RETURN_ON_FALSE(m && m->driver, ESP_ERR_INVALID_STATE, TAG, "无驱动接口");

    float s, c;
    foc_sincos(angle_el, &s, &c);

    /* 反 Park */
    const foc_ab_t u_ab = {
        .alpha = c * ud - s * uq,
        .beta  = s * ud + c * uq,
    };

    /* 反 Clarke */
    foc_abc_t u = foc_inverse_clarke(u_ab);

    const float vlimit = m->driver->voltage_limit;

    if (m->modulation_centered) {
        float center = vlimit * FOC_MOD_CENTER_RATIO;
        if (m->modulation == FOC_MOD_SPACE_VECTOR_PWM) {
            /* 中点钳位：center 再减去 (Umax+Umin)/2 */
            const float umin = fminf(u.a, fminf(u.b, u.c));
            const float umax = fmaxf(u.a, fmaxf(u.b, u.c));
            center -= (umax + umin) * FOC_MOD_CENTER_RATIO;
        }
        u.a += center;
        u.b += center;
        u.c += center;
    } else {
        /* 非中点对齐：把最低相拉到 0 */
        const float umin = fminf(u.a, fminf(u.b, u.c));
        u.a -= umin;
        u.b -= umin;
        u.c -= umin;
    }

    return m->driver->set_phase_voltage(m->driver->ctx, u.a, u.b, u.c, vlimit);
}

/* -------------------------------------------------------------------------- */
/*  四个环                                                                     */
/* -------------------------------------------------------------------------- */

float foc_loop_torque(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m, 0.0f, TAG, "句柄为空");
    /* SimpleFOC MotionControlType::torque: current_sp = target */
    m->current_sp = m->target;
    return m->current_sp;
}

/**
 * @brief 只读不控：把三相电流读出来、做 Clarke+Park、低通后存进 m->current
 *
 * 用于电压力矩模式下监视 Iq/Id（SimpleFOC test_current 的做法：
 * 验证电流采样时电流环是【不闭合】的，电流只用来观察）。
 */
static void foc_read_current_for_monitor(foc_motor_t *m)
{
    if (!m->current_sense) {
        return;
    }
    foc_abc_t iabc = { FOC_ZERO, FOC_ZERO, FOC_ZERO };
    if (!m->current_sense->read_phase(m->current_sense->ctx, &iabc)) {
        return;
    }
    m->current_abc = iabc;

    const foc_dq_t idq = foc_park(foc_clarke(iabc), m->electrical_angle);
    m->current.q = foc_lpf_run(&m->lpf_current_q, idq.q);
    m->current.d = foc_lpf_run(&m->lpf_current_d, idq.d);
}

float foc_loop_current_dq(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m && m->current_sense, 0.0f, TAG, "无电流采样接口");

    /* 目标电流限幅 + 前馈省略 */
    m->current_sp = foc_clampf(m->current_sp, -m->current_limit, m->current_limit);

    /* 采样三相电流 -> αβ -> dq */
    foc_abc_t iabc = { 0.0f, 0.0f, 0.0f };
    if (!m->current_sense->read_phase(m->current_sense->ctx, &iabc)) {
        return 0.0f;
    }
    m->current_abc = iabc;
    m->current = foc_park(foc_clarke(iabc), m->electrical_angle);

    /* 滤波 */
    m->current.q = foc_lpf_run(&m->lpf_current_q, m->current.q);
    m->current.d = foc_lpf_run(&m->lpf_current_d, m->current.d);

    /* d/q 各一个 PI */
    m->voltage.q = foc_pid_run(&m->pid_current_q, m->current_sp - m->current.q);
    m->voltage.d = foc_pid_run(&m->pid_current_d, -m->current.d);
    return m->voltage.q;
}

float foc_loop_current_dc(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m && m->current_sense, 0.0f, TAG, "无电流采样接口");

    m->current_sp = foc_clampf(m->current_sp, -m->current_limit, m->current_limit);

    foc_abc_t iabc = { 0.0f, 0.0f, 0.0f };
    if (!m->current_sense->read_phase(m->current_sense->ctx, &iabc)) {
        return 0.0f;
    }
    m->current_abc = iabc;
    /* 电流幅值：用 Park 变换后的 q 分量近似（与 SimpleFOC getDCCurrent 同思路） */
    const foc_dq_t idq = foc_park(foc_clarke(iabc), m->electrical_angle);
    m->current_magnitude = sqrtf(idq.d * idq.d + idq.q * idq.q);
    m->current.q = foc_lpf_run(&m->lpf_current_q, m->current_magnitude);
    m->current.d = 0.0f;

    m->voltage.q = foc_pid_run(&m->pid_current_q, m->current_sp - m->current.q);
    m->voltage.d = 0.0f;
    return m->voltage.q;
}

float foc_loop_velocity(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m, 0.0f, TAG, "句柄为空");
    /* SimpleFOC MotionControlType::velocity: current_sp = PID_velocity(w_sp - w) */
    m->shaft_velocity_sp = m->target;
    m->current_sp = foc_pid_run(&m->pid_velocity, m->shaft_velocity_sp - m->shaft_velocity);
    return m->current_sp;
}

float foc_loop_angle(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m, 0.0f, TAG, "句柄为空");

    /* 位置环：P 外环把角度误差变成速度指令 */
    m->shaft_angle_sp = m->target;
    m->shaft_velocity_sp = m->feed_forward_velocity
                         + foc_pid_run(&m->p_angle,
                                       m->shaft_angle_sp - foc_lpf_run(&m->lpf_angle, m->shaft_angle));
    m->shaft_velocity_sp = foc_clampf(m->shaft_velocity_sp, -m->velocity_limit, m->velocity_limit);

    /* 速度内环 */
    m->current_sp = foc_pid_run(&m->pid_velocity, m->shaft_velocity_sp - m->shaft_velocity);
    return m->current_sp;
}

float foc_loop_angle_nocascade(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m, 0.0f, TAG, "句柄为空");
    m->shaft_angle_sp = m->target;
    m->current_sp = foc_pid_run(&m->p_angle,
                                m->shaft_angle_sp - foc_lpf_run(&m->lpf_angle, m->shaft_angle));
    return m->current_sp;
}

/* -------------------------------------------------------------------------- */
/*  开环两种方式（SimpleFOC velocityOpenloop / angleOpenloop）                  */
/* -------------------------------------------------------------------------- */

static float loop_dt(foc_motor_t *m)
{
    const int64_t now = foc_micros();
    float ts = (float)(now - m->open_loop_timestamp) * 1e-6f;
    if (ts <= 0.0f || ts > FOC_OPENLOOP_MAX_DT) {
        ts = FOC_OPENLOOP_FALLBACK_DT;
    }
    m->open_loop_timestamp = now;
    return ts;
}

static float foc_motor_velocity_openloop(foc_motor_t *m, float target_velocity)
{
    const float ts = loop_dt(m);
    m->shaft_angle    = foc_normalize_angle(m->shaft_angle + target_velocity * ts);
    m->shaft_velocity = target_velocity;

    return (m->torque_ctrl == FOC_TORQUE_VOLTAGE) ? m->voltage_limit : m->current_limit;
}

static float foc_motor_angle_openloop(foc_motor_t *m, float target_angle)
{
    const float ts = loop_dt(m);

    if (fabsf(target_angle - m->shaft_angle) > fabsf(m->velocity_limit * ts)) {
        const float dir = (float)foc_signf(target_angle - m->shaft_angle);
        m->shaft_angle += dir * fabsf(m->velocity_limit) * ts;
        m->shaft_velocity = (dir > 0.0f) ? m->velocity_limit : -m->velocity_limit;
    } else {
        m->shaft_angle    = target_angle;
        m->shaft_velocity = 0.0f;
    }
    return (m->torque_ctrl == FOC_TORQUE_VOLTAGE) ? m->voltage_limit : m->current_limit;
}

/* -------------------------------------------------------------------------- */
/*  loopFOC                                                                    */
/* -------------------------------------------------------------------------- */

void foc_motor_loop_foc(foc_motor_t *m)
{
    if (!m || !m->initialized) {
        return;
    }

    /* 1) 传感器（开环也要更新，避免切回闭环时丢圈） */
    if (m->sensor) {
        m->sensor->update(m->sensor->ctx);
    }

    /* 2) 电角度 */
    const bool openloop = (m->motion == FOC_MOTION_VELOCITY_OPENLOOP) ||
                          (m->motion == FOC_MOTION_ANGLE_OPENLOOP);
    if (openloop) {
        m->electrical_angle = foc_electrical_angle(m->shaft_angle, m->pole_pairs);
    } else {
        m->electrical_angle = foc_motor_electrical_angle(m);
    }

    /* 3) 力矩/电流环 */
    switch (m->torque_ctrl) {
        case FOC_TORQUE_VOLTAGE:
            m->voltage.q = foc_clampf(m->current_sp, -m->voltage_limit, m->voltage_limit);
            m->voltage.d = 0.0f;
            /* 不闭环，但仍读电流供监视 */
            foc_read_current_for_monitor(m);
            break;
        case FOC_TORQUE_DC_CURRENT:
            if (!m->current_sense) return;
            (void)foc_loop_current_dc(m);
            break;
        case FOC_TORQUE_FOC_CURRENT:
            if (!m->current_sense) return;
            (void)foc_loop_current_dq(m);
            break;
        default:
            ESP_LOGE(TAG, "未选择力矩控制方式");
            return;
    }

    /* 4) 调制并写驱动 */
    (void)foc_motor_set_phase_voltage(m, m->voltage.q, m->voltage.d, m->electrical_angle);
}

/* -------------------------------------------------------------------------- */
/*  move                                                                       */
/* -------------------------------------------------------------------------- */

float foc_motor_move(foc_motor_t *m, float new_target)
{
    if (!m || !m->initialized) {
        return 0.0f;
    }
    if (foc_is_set(new_target)) {
        m->target = new_target;
    }

    /* 读回位置/速度（开环模式由下面对应函数自己更新） */
    const bool openloop = (m->motion == FOC_MOTION_VELOCITY_OPENLOOP) ||
                          (m->motion == FOC_MOTION_ANGLE_OPENLOOP);
    if (!openloop && m->sensor) {
        m->shaft_angle    = m->sensor->get_angle(m->sensor->ctx);
        m->shaft_velocity = foc_lpf_run(&m->lpf_velocity, m->sensor->get_velocity(m->sensor->ctx));
    }

    switch (m->motion) {
        case FOC_MOTION_TORQUE:
            (void)foc_loop_torque(m);
            break;
        case FOC_MOTION_VELOCITY:
            (void)foc_loop_velocity(m);
            break;
        case FOC_MOTION_ANGLE:
            (void)foc_loop_angle(m);
            break;
        case FOC_MOTION_ANGLE_NOCASCADE:
            (void)foc_loop_angle_nocascade(m);
            break;
        case FOC_MOTION_VELOCITY_OPENLOOP:
            m->shaft_velocity_sp = m->target;
            m->current_sp = foc_motor_velocity_openloop(m, m->shaft_velocity_sp);
            break;
        case FOC_MOTION_ANGLE_OPENLOOP:
            m->shaft_angle_sp = m->target;
            m->current_sp = foc_motor_angle_openloop(m, m->shaft_angle_sp);
            break;
        default:
            break;
    }
    return m->target;
}

/* -------------------------------------------------------------------------- */
/*  传感器对齐（SimpleFOC FOCMotor::alignSensor）                               */
/* -------------------------------------------------------------------------- */

#define FOC_MIN_ANGLE_DETECT_MOVEMENT   (FOC_2PI / 101.0f)

esp_err_t foc_motor_align_sensor(foc_motor_t *m)
{
    ESP_RETURN_ON_FALSE(m && m->initialized, ESP_ERR_INVALID_STATE, TAG, "未初始化");
    ESP_RETURN_ON_FALSE(m->sensor, ESP_ERR_INVALID_ARG, TAG, "无传感器，无法对齐");

    const float voltage_align = m->voltage_sensor_align;
    const float step = FOC_2PI / (float)FOC_ALIGN_STEPS;

    if (m->sensor_direction == 0.0f) {
        ESP_LOGI(TAG, "对齐: 扫描电角度一个整圈，判定传感器方向...");

        /* 正向扫一个电周期 */
        for (int i = 0; i <= FOC_ALIGN_STEPS; i++) {
            (void)foc_motor_set_phase_voltage(m, voltage_align, 0.0f,
                                              FOC_3PI_2 + step * (float)i);
            m->sensor->update(m->sensor->ctx);
            vTaskDelay(pdMS_TO_TICKS(FOC_ALIGN_STEP_DELAY_MS));
        }
        m->sensor->update(m->sensor->ctx);
        const float mid_angle = m->sensor->get_angle(m->sensor->ctx);

        /* 反向扫回来 */
        for (int i = FOC_ALIGN_STEPS; i >= 0; i--) {
            (void)foc_motor_set_phase_voltage(m, voltage_align, 0.0f,
                                              FOC_3PI_2 + step * (float)i);
            m->sensor->update(m->sensor->ctx);
            vTaskDelay(pdMS_TO_TICKS(FOC_ALIGN_STEP_DELAY_MS));
        }
        m->sensor->update(m->sensor->ctx);
        const float end_angle = m->sensor->get_angle(m->sensor->ctx);
        vTaskDelay(pdMS_TO_TICKS(FOC_ALIGN_SETTLE_MS));

        const float moved = fabsf(mid_angle - end_angle);
        if (moved < FOC_MIN_ANGLE_DETECT_MOVEMENT) {
            ESP_LOGE(TAG, "对齐失败: 传感器没有检测到转动 (moved=%.4f rad)", (double)moved);
            (void)foc_motor_set_phase_voltage(m, 0.0f, 0.0f, 0.0f);
            return ESP_FAIL;
        }
        /* SimpleFOC: mid < end -> CCW(-1)，否则 CW(+1) */
        m->sensor_direction = (mid_angle < end_angle) ? -1.0f : 1.0f;

        /* 极对数自检（SimpleFOC 只告警不失败） */
        if (fabsf(moved * (float)m->pole_pairs - FOC_2PI) > FOC_PP_CHECK_TOLERANCE) {
            ESP_LOGW(TAG, "极对数自检未通过: 实测转动 %.4f rad, 按 %d 对极应为 %.4f rad",
                     (double)moved, m->pole_pairs, (double)FOC_2PI);
        } else {
            ESP_LOGI(TAG, "极对数自检通过");
        }
        ESP_LOGI(TAG, "传感器方向: %s", (m->sensor_direction > 0.0f) ? "CW(+1)" : "CCW(-1)");
    }

    /* 找电角度零点 */
    ESP_LOGI(TAG, "对齐: 施加 theta=270°，读电角度零点...");
    (void)foc_motor_set_phase_voltage(m, voltage_align, 0.0f, FOC_3PI_2);
    vTaskDelay(pdMS_TO_TICKS(700));

    m->sensor->update(m->sensor->ctx);
    m->zero_electric_angle = 0.0f;
    m->zero_electric_angle = foc_motor_electrical_angle(m);
    /* 补上 d/q 轴系的固定偏差 */
    m->zero_electric_angle = foc_normalize_angle(m->zero_electric_angle
                                                 + m->align_zero_offset);
    vTaskDelay(pdMS_TO_TICKS(20));

    (void)foc_motor_set_phase_voltage(m, 0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(FOC_ALIGN_SETTLE_MS));

    m->aligned = true;
    ESP_LOGI(TAG, "对齐完成: zero_electric_angle=%.4f rad (%.1f°)  direction=%+.0f  偏移=%+.1f°",
             (double)m->zero_electric_angle,
             (double)(m->zero_electric_angle * FOC_RAD_TO_DEG),
             (double)m->sensor_direction,
             (double)(m->align_zero_offset * FOC_RAD_TO_DEG));
    return ESP_OK;
}
