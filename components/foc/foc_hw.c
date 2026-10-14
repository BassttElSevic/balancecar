/*
 * ============================================================================
 *  foc_hw —— 实现（纯转发，无算法）
 * ============================================================================
 */

#include "esp_log.h"
#include "esp_check.h"

#include "board_pins.h"
#include "motor.h"

#include "foc_hw.h"

static const char *TAG = "foc_hw";

/** 每多少次写 PWM 打一次点（诊断用）*/
#define FOC_DRV_LOG_EVERY   200

/* -------------------------------------------------------------------------- */
/*  三相驱动适配                                                               */
/* -------------------------------------------------------------------------- */

/** motor.c 没有句柄，用一个文件内状态记录使能情况 */
static bool s_driver_enabled;

/**
 * @brief 相电压 -> 占空比并写 MCPWM
 * @note duty = U / vlimit，并限幅到 [0,1]（同 SimpleFOC 的 driver->setPwm）
 */
static esp_err_t drv_set_phase_voltage(void *ctx, float ua, float ub, float uc, float vlimit)
{
    (void)ctx;
    if (vlimit <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    const float da = foc_clampf(ua / vlimit, FOC_DUTY_MIN, FOC_DUTY_MAX);
    const float db = foc_clampf(ub / vlimit, FOC_DUTY_MIN, FOC_DUTY_MAX);
    const float dc = foc_clampf(uc / vlimit, FOC_DUTY_MIN, FOC_DUTY_MAX);

    /* 每 FOC_DRV_LOG_EVERY 次打一次点，确认电压真的下到了占空比 */
    static uint32_t s_calls;
    if ((s_calls++ % FOC_DRV_LOG_EVERY) == 0) {
        ESP_LOGI(TAG, "PWM: U/V/W = %+.2f/%+.2f/%+.2f V -> duty %.3f/%.3f/%.3f (vlimit %.1f, en=%d)",
                 (double)ua, (double)ub, (double)uc,
                 (double)da, (double)db, (double)dc,
                 (double)vlimit, (int)s_driver_enabled);
    }
    return motor_set_duty(da, db, dc);
}

static esp_err_t drv_enable(void *ctx, bool en)
{
    (void)ctx;
    motor_enable(en);
    s_driver_enabled = en;
    return ESP_OK;
}

static bool drv_is_enabled(void *ctx)
{
    (void)ctx;
    return s_driver_enabled;
}

static const foc_driver_t s_driver = {
    .ctx               = NULL,
    .voltage_limit     = MOTOR_SUPPLY_V,
    .set_phase_voltage = drv_set_phase_voltage,
    .enable            = drv_enable,
    .is_enabled        = drv_is_enabled,
};

const foc_driver_t *foc_hw_driver(void)
{
    return &s_driver;
}

/* -------------------------------------------------------------------------- */
/*  传感器适配                                                                 */
/* -------------------------------------------------------------------------- */

static void sen_update(void *ctx)
{
    (void)encoder_update((encoder_t *)ctx);
}

static float sen_get_angle(void *ctx)
{
    return encoder_get_angle((encoder_t *)ctx);
}

static float sen_get_mechanical_angle(void *ctx)
{
    return encoder_get_single_turn((encoder_t *)ctx);
}

static float sen_get_velocity(void *ctx)
{
    return encoder_get_velocity((encoder_t *)ctx);
}

static const foc_sensor_t s_sensor = {
    .ctx                  = NULL,   /* 由 foc_hw_sensor_init 填 enc */
    .update               = sen_update,
    .get_angle            = sen_get_angle,
    .get_mechanical_angle = sen_get_mechanical_angle,
    .get_velocity         = sen_get_velocity,
};

void foc_hw_sensor_init(foc_sensor_t *out, encoder_t *enc)
{
    if (!out || !enc) {
        ESP_LOGE(TAG, "参数为空");
        return;
    }
    *out = s_sensor;
    out->ctx = enc;
}

/* -------------------------------------------------------------------------- */
/*  电流采样适配                                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief 读三相电流
 * @note 会先刷新一次采样，所以调用者不需要额外调 update
 */
static bool cur_read_phase(void *ctx, foc_abc_t *out_abc)
{
    current_sense_t *cs = (current_sense_t *)ctx;
    if (!cs || !out_abc) {
        return false;
    }
    if (current_sense_update(cs) != ESP_OK) {
        return false;
    }
    out_abc->a = current_sense_get_a(cs);
    out_abc->b = current_sense_get_b(cs);
    out_abc->c = current_sense_get_c(cs);
    return true;
}

static void cur_update(void *ctx)
{
    (void)current_sense_update((current_sense_t *)ctx);
}

static void cur_set_sign_a(void *ctx, float sign) { current_sense_set_sign_a((current_sense_t *)ctx, sign); }
static void cur_set_sign_b(void *ctx, float sign) { current_sense_set_sign_b((current_sense_t *)ctx, sign); }
static void cur_set_swap(void *ctx, bool swap)    { current_sense_set_swap((current_sense_t *)ctx, swap); }

static const foc_current_t s_current = {
    .ctx        = NULL,   /* 由 foc_hw_current_init 填 cs */
    .update     = cur_update,
    .read_phase = cur_read_phase,
    .set_sign_a = cur_set_sign_a,
    .set_sign_b = cur_set_sign_b,
    .set_swap   = cur_set_swap,
};

void foc_hw_current_init(foc_current_t *out, current_sense_t *cs)
{
    if (!out || !cs) {
        ESP_LOGE(TAG, "参数为空");
        return;
    }
    *out = s_current;
    out->ctx = cs;
}
