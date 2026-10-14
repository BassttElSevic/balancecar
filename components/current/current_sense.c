/*
 * ============================================================================
 *  current_sense —— 实现
 *
 *  不使用堆：句柄存储由调用者提供。
 *  ADC 的使用方式（单元配置 / 每通道独立校准句柄）见头文件里的说明。
 * ============================================================================
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include "current_sense.h"

static const char *TAG = "current";

#define MV_PER_V            1000.0f
#define CURRENT_FALLBACK_ZERO_MV   1650.0f   /**< 未校准时的临时零点 (Vdd/2) */
#define ADC_RAW_FULL_SCALE  4095.0f
#define ADC_REF_MV          3300.0f

/* -------------------------------------------------------------------------- */
/*  默认值                                                                     */
/* -------------------------------------------------------------------------- */

static void cfg_apply_defaults(current_sense_cfg_t *c)
{
    if (c->cal_samples  == 0) c->cal_samples  = CURRENT_SENSE_CAL_SAMPLES_DEFAULT;
    if (c->cal_delay_ms == 0) c->cal_delay_ms = CURRENT_SENSE_CAL_DELAY_MS_DEFAULT;
    if (c->sign_a == 0)       c->sign_a       = CURRENT_SENSE_SIGN_POS;
    if (c->sign_b == 0)       c->sign_b       = CURRENT_SENSE_SIGN_POS;
}

/** @brief gain = sign / 灵敏度 (A/V, 带符号) */
static void apply_gains(current_sense_t *cs)
{
    cs->gain_a = cs->sign_a / cs->v_per_a;
    cs->gain_b = cs->sign_b / cs->v_per_a;
}

/* -------------------------------------------------------------------------- */
/*  内部：读一个通道（raw -> mV）                                              */
/* -------------------------------------------------------------------------- */

static esp_err_t read_one(current_sense_t *cs, adc_channel_t chan,
                          adc_cali_handle_t cali, int *out_raw, int *out_mv)
{
    int raw = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(cs->unit_h, chan, &raw), TAG,
                        "ADC 读取失败 (CH%d)", (int)chan);
    *out_raw = raw;

    if (cali) {
        int mv = 0;
        if (adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) {
            *out_mv = mv;
            return ESP_OK;
        }
    }
    /* 没有校准句柄时退化：按 12bit / 3.3V 粗算 */
    *out_mv = (int)((float)raw * ADC_REF_MV / ADC_RAW_FULL_SCALE);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  生命周期                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t current_sense_init(current_sense_t *cs, const current_sense_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(cs && cfg, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    ESP_RETURN_ON_FALSE(!cs->ready, ESP_ERR_INVALID_STATE, TAG, "电流采样已初始化");

    current_sense_cfg_t c = *cfg;
    cfg_apply_defaults(&c);

    memset(cs, 0, sizeof(*cs));
    cs->chan_a      = c.chan_a;
    cs->chan_b      = c.chan_b;
    cs->gpio_a      = c.gpio_a;
    cs->gpio_b      = c.gpio_b;
    cs->cal_samples = c.cal_samples;
    cs->v_per_a     = c.shunt_ohms * c.amp_gain;
    cs->sign_a      = c.sign_a;
    cs->sign_b      = c.sign_b;

    ESP_RETURN_ON_FALSE(cs->v_per_a > 0.0f, ESP_ERR_INVALID_ARG, TAG,
                        "灵敏度无效 (shunt=%f gain=%f)", (double)c.shunt_ohms, (double)c.amp_gain);
    apply_gains(cs);

    /* ---- 1) ADC 单元 ---- */
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = c.unit,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &cs->unit_h), TAG,
                        "创建 ADC 单元失败");

    /* ---- 2) 两个通道 ---- */
    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = c.atten,
        .bitwidth = c.bitwidth,
    };
    esp_err_t err = adc_oneshot_config_channel(cs->unit_h, c.chan_a, &chan_cfg);
    if (err == ESP_OK) {
        err = adc_oneshot_config_channel(cs->unit_h, c.chan_b, &chan_cfg);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "配置 ADC 通道失败: %s", esp_err_to_name(err));
        (void)adc_oneshot_del_unit(cs->unit_h);
        cs->unit_h = NULL;
        return err;
    }

    /* ---- 3) 每个通道各建一个校准句柄，必须显式填 .chan ---- */
    const adc_cali_curve_fitting_config_t cali_cfg_a = {
        .unit_id  = c.unit,
        .chan     = c.chan_a,
        .atten    = c.atten,
        .bitwidth = c.bitwidth,
    };
    const adc_cali_curve_fitting_config_t cali_cfg_b = {
        .unit_id  = c.unit,
        .chan     = c.chan_b,
        .atten    = c.atten,
        .bitwidth = c.bitwidth,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg_a, &cs->cali_a) != ESP_OK) {
        ESP_LOGW(TAG, "A 相 (CH%d) 校准句柄创建失败，该相退化为粗算", (int)c.chan_a);
        cs->cali_a = NULL;
    }
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg_b, &cs->cali_b) != ESP_OK) {
        ESP_LOGW(TAG, "B 相 (CH%d) 校准句柄创建失败，该相退化为粗算", (int)c.chan_b);
        cs->cali_b = NULL;
    }

    cs->off_a_mv = CURRENT_FALLBACK_ZERO_MV;
    cs->off_b_mv = CURRENT_FALLBACK_ZERO_MV;
    cs->ready    = true;

    ESP_LOGI(TAG, "就绪: A=GPIO%d(CH%d)  B=GPIO%d(CH%d)  灵敏度 %.2f V/A  校准句柄 A=%d B=%d",
             c.gpio_a, (int)c.chan_a, c.gpio_b, (int)c.chan_b, (double)cs->v_per_a,
             cs->cali_a ? 1 : 0, cs->cali_b ? 1 : 0);
    return ESP_OK;
}

esp_err_t current_sense_deinit(current_sense_t *cs)
{
    ESP_RETURN_ON_FALSE(cs, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    if (!cs->ready) {
        return ESP_OK;
    }
    if (cs->cali_a) {
        (void)adc_cali_delete_scheme_curve_fitting(cs->cali_a);
    }
    if (cs->cali_b) {
        (void)adc_cali_delete_scheme_curve_fitting(cs->cali_b);
    }
    if (cs->unit_h) {
        (void)adc_oneshot_del_unit(cs->unit_h);
    }
    memset(cs, 0, sizeof(*cs));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  使用                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t current_sense_calibrate(current_sense_t *cs)
{
    ESP_RETURN_ON_FALSE(cs && cs->ready, ESP_ERR_INVALID_STATE, TAG, "电流采样未初始化");

    int64_t sum_a = 0;
    int64_t sum_b = 0;

    for (int i = 0; i < cs->cal_samples; i++) {
        /* 直接写回句柄，这样校准完就能从 print_diag 看到当时的读数 */
        ESP_RETURN_ON_ERROR(read_one(cs, cs->chan_a, cs->cali_a, &cs->raw_a, &cs->mv_a), TAG,
                            "A 相校准读取失败");
        sum_a += cs->mv_a;
        ESP_RETURN_ON_ERROR(read_one(cs, cs->chan_b, cs->cali_b, &cs->raw_b, &cs->mv_b), TAG,
                            "B 相校准读取失败");
        sum_b += cs->mv_b;
        vTaskDelay(pdMS_TO_TICKS(CURRENT_SENSE_CAL_DELAY_MS_DEFAULT));
    }

    cs->off_a_mv   = (float)sum_a / (float)cs->cal_samples;
    cs->off_b_mv   = (float)sum_b / (float)cs->cal_samples;
    cs->calibrated = true;

    /* 用零点重新算一次三相，让校准后的诊断输出有意义 */
    /* 与 SimpleFOC 一致：i = (v - offset) * gain，gain 带符号 */
    cs->ia = ((float)cs->mv_a - cs->off_a_mv) / MV_PER_V * cs->gain_a;
    cs->ib = ((float)cs->mv_b - cs->off_b_mv) / MV_PER_V * cs->gain_b;
    cs->ic = -cs->ia - cs->ib;

    ESP_LOGI(TAG, "零点校准完成 (%d 次): A=%.1f mV  B=%.1f mV",
             cs->cal_samples, (double)cs->off_a_mv, (double)cs->off_b_mv);
    return ESP_OK;
}

esp_err_t current_sense_update(current_sense_t *cs)
{
    ESP_RETURN_ON_FALSE(cs && cs->ready, ESP_ERR_INVALID_STATE, TAG, "电流采样未初始化");

    ESP_RETURN_ON_ERROR(read_one(cs, cs->chan_a, cs->cali_a, &cs->raw_a, &cs->mv_a),
                        TAG, "读 A 相失败");
    ESP_RETURN_ON_ERROR(read_one(cs, cs->chan_b, cs->cali_b, &cs->raw_b, &cs->mv_b),
                        TAG, "读 B 相失败");

    /* 与 SimpleFOC 一致：i = (v - offset) * gain，gain 带符号 */
    cs->ia = ((float)cs->mv_a - cs->off_a_mv) / MV_PER_V * cs->gain_a;
    cs->ib = ((float)cs->mv_b - cs->off_b_mv) / MV_PER_V * cs->gain_b;
    cs->ic = -cs->ia - cs->ib;      /* 基尔霍夫 */
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  读取                                                                       */
/* -------------------------------------------------------------------------- */

float current_sense_get_a(const current_sense_t *cs) { return cs ? cs->ia : 0.0f; }
float current_sense_get_b(const current_sense_t *cs) { return cs ? cs->ib : 0.0f; }
float current_sense_get_c(const current_sense_t *cs) { return cs ? cs->ic : 0.0f; }

int current_sense_get_raw_a(const current_sense_t *cs) { return cs ? cs->raw_a : 0; }
int current_sense_get_raw_b(const current_sense_t *cs) { return cs ? cs->raw_b : 0; }
int current_sense_get_mv_a(const current_sense_t *cs)  { return cs ? cs->mv_a  : 0; }
int current_sense_get_mv_b(const current_sense_t *cs)  { return cs ? cs->mv_b  : 0; }

float current_sense_get_offset_mv_a(const current_sense_t *cs) { return cs ? cs->off_a_mv : 0.0f; }
float current_sense_get_offset_mv_b(const current_sense_t *cs) { return cs ? cs->off_b_mv : 0.0f; }

void current_sense_set_sign_a(current_sense_t *cs, float sign)
{
    if (!cs) return;
    cs->sign_a = (sign < 0.0f) ? CURRENT_SENSE_SIGN_NEG : CURRENT_SENSE_SIGN_POS;
    apply_gains(cs);
}

void current_sense_set_sign_b(current_sense_t *cs, float sign)
{
    if (!cs) return;
    cs->sign_b = (sign < 0.0f) ? CURRENT_SENSE_SIGN_NEG : CURRENT_SENSE_SIGN_POS;
    apply_gains(cs);
}

float current_sense_get_gain_a(const current_sense_t *cs) { return cs ? cs->gain_a : 0.0f; }
float current_sense_get_gain_b(const current_sense_t *cs) { return cs ? cs->gain_b : 0.0f; }
float current_sense_get_sign_a(const current_sense_t *cs) { return cs ? cs->sign_a : 0.0f; }
float current_sense_get_sign_b(const current_sense_t *cs) { return cs ? cs->sign_b : 0.0f; }

bool     current_sense_is_calibrated(const current_sense_t *cs) { return cs ? cs->calibrated : false; }
uint32_t current_sense_get_err_count(const current_sense_t *cs) { return cs ? cs->err_count : 0; }

void current_sense_print_diag(const current_sense_t *cs)
{
    if (!cs || !cs->ready) {
        return;
    }
    printf("[电流] A=GPIO%d(CH%d) 符号%+.0f   B=GPIO%d(CH%d) 符号%+.0f   灵敏度 %.2f V/A\n",
           cs->gpio_a, (int)cs->chan_a, (double)cs->sign_a,
           cs->gpio_b, (int)cs->chan_b, (double)cs->sign_b, (double)cs->v_per_a);
    printf("   A: raw=%4d  %4d mV  (零 %.1f)  Ia=%+7.3f A\n",
           cs->raw_a, cs->mv_a, (double)cs->off_a_mv, (double)cs->ia);
    printf("   B: raw=%4d  %4d mV  (零 %.1f)  Ib=%+7.3f A\n",
           cs->raw_b, cs->mv_b, (double)cs->off_b_mv, (double)cs->ib);
    printf("   C: Ic = -Ia-Ib = %+7.3f A      校准状态 %s\n",
           (double)cs->ic, cs->calibrated ? "已校准" : "未校准");
    printf("   空闲(无电流)时两路都应在零点附近。\n");
}
