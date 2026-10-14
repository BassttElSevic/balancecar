/*
 * ============================================================================
 *  balancecar — 双电阻在线电流采样 (INA240 x2 -> ADC1)
 *
 *  A 相 GPIO21 (ADC1_CH5) / B 相 GPIO20 (ADC1_CH4)，Ic = -Ia - Ib
 *  灵敏度 0.50 V/A (0.01Ω x 50)
 *
 *  ⚠️ 零点校准必须在电机不转、无电流时调用 current_sense_calibrate()
 * ============================================================================
 */

#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "board_pins.h"
#include "current_sense.h"

static const char *TAG = "current";

#define CS_ATTEN        ADC_ATTEN_DB_12
#define CS_BITWIDTH     ADC_BITWIDTH_12
#define CS_CAL_SAMPLES  64
#define CS_CAL_DELAY_MS 1
#define CS_V_PER_A      (BOARD_CS_SHUNT_OHMS * BOARD_CS_AMP_GAIN)

#define CH_A            ((adc_channel_t)BOARD_CS_A_CHANNEL)
#define CH_B            ((adc_channel_t)BOARD_CS_B_CHANNEL)

static adc_oneshot_unit_handle_t s_unit;
static adc_cali_handle_t         s_cali_a;
static adc_cali_handle_t         s_cali_b;
static bool  s_ready, s_calibrated;
static float s_off_a_mv, s_off_b_mv;
static int   s_raw_a, s_raw_b, s_mv_a, s_mv_b;
static float s_ia, s_ib, s_ic;

esp_err_t current_sense_init(void)
{
    const adc_oneshot_unit_init_cfg_t uc = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&uc, &s_unit), TAG, "创建 ADC 单元失败");

    const adc_oneshot_chan_cfg_t cc = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_unit, CH_A, &cc), TAG, "配置 CH%d 失败", CH_A);
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_unit, CH_B, &cc), TAG, "配置 CH%d 失败", CH_B);

    const adc_cali_curve_fitting_config_t ca = {
        .unit_id  = ADC_UNIT_1,
        .chan     = CH_A,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    const adc_cali_curve_fitting_config_t cb = {
        .unit_id  = ADC_UNIT_1,
        .chan     = CH_B,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&ca, &s_cali_a) != ESP_OK) {
        ESP_LOGW(TAG, "CH%d 校准创建失败", CH_A);
        s_cali_a = NULL;
    }
    if (adc_cali_create_scheme_curve_fitting(&cb, &s_cali_b) != ESP_OK) {
        ESP_LOGW(TAG, "CH%d 校准创建失败", CH_B);
        s_cali_b = NULL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "电流采样初始化完成: A=GPIO%d(ADC1_CH%d)  B=GPIO%d(ADC1_CH%d)  %.2f V/A",
             BOARD_CS_A_GPIO, (int)CH_A, BOARD_CS_B_GPIO, (int)CH_B, (double)CS_V_PER_A);
    return ESP_OK;
}

/** 读一路，返回 mV；失败返回 -1 */
static int poll_one(adc_channel_t ch, adc_cali_handle_t cali, int *out_raw)
{
    int raw = 0;
    if (adc_oneshot_read(s_unit, ch, &raw) != ESP_OK) {
        return -1;
    }
    *out_raw = raw;
    int mv = 0;
    if (cali && adc_cali_raw_to_voltage(cali, raw, &mv) == ESP_OK) {
        return mv;
    }
    return (int)((float)raw * 3300.0f / 4095.0f);
}

esp_err_t current_sense_calibrate(void)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "电流采样未初始化");

    int64_t sa = 0, sb = 0;
    for (int i = 0; i < CS_CAL_SAMPLES; i++) {
        const int ma = poll_one(CH_A, s_cali_a, &s_raw_a);
        const int mb = poll_one(CH_B, s_cali_b, &s_raw_b);
        ESP_RETURN_ON_FALSE(ma >= 0 && mb >= 0, ESP_FAIL, TAG, "校准读取失败");
        sa += ma;
        sb += mb;
        vTaskDelay(pdMS_TO_TICKS(CS_CAL_DELAY_MS));
    }
    s_off_a_mv = (float)sa / (float)CS_CAL_SAMPLES;
    s_off_b_mv = (float)sb / (float)CS_CAL_SAMPLES;
    s_calibrated = true;

    ESP_LOGI(TAG, "零点校准完成: A(raw=%d) %.1f mV   B(raw=%d) %.1f mV",
             s_raw_a, (double)s_off_a_mv, s_raw_b, (double)s_off_b_mv);
    return ESP_OK;
}

esp_err_t current_sense_update(void)
{
    ESP_RETURN_ON_FALSE(s_ready, ESP_ERR_INVALID_STATE, TAG, "电流采样未初始化");

    const int ma = poll_one(CH_A, s_cali_a, &s_raw_a);
    const int mb = poll_one(CH_B, s_cali_b, &s_raw_b);
    ESP_RETURN_ON_FALSE(ma >= 0 && mb >= 0, ESP_FAIL, TAG, "电流读取失败");
    s_mv_a = ma;
    s_mv_b = mb;

    const float za = s_calibrated ? s_off_a_mv : 1650.0f;
    const float zb = s_calibrated ? s_off_b_mv : 1650.0f;
    s_ia = ((float)ma - za) / 1000.0f / CS_V_PER_A;
    s_ib = ((float)mb - zb) / 1000.0f / CS_V_PER_A;
    s_ic = -s_ia - s_ib;
    return ESP_OK;
}

float current_get_a(void) { return s_ia; }
float current_get_b(void) { return s_ib; }
float current_get_c(void) { return s_ic; }
int   current_get_mv_a(void) { return s_mv_a; }
int   current_get_mv_b(void) { return s_mv_b; }
int   current_get_raw_a(void) { return s_raw_a; }
int   current_get_raw_b(void) { return s_raw_b; }
float current_get_offset_mv_a(void) { return s_off_a_mv; }
float current_get_offset_mv_b(void) { return s_off_b_mv; }

void current_sense_print_diag(void)
{
    printf("[电流自检]\n");
    printf("   A GPIO%-3d CH%d  raw %4d  %4d mV  零点 %7.1f  Ia %+7.3f A\n",
           BOARD_CS_A_GPIO, (int)CH_A, s_raw_a, s_mv_a,
           (double)(s_calibrated ? s_off_a_mv : 1650.0f), (double)s_ia);
    printf("   B GPIO%-3d CH%d  raw %4d  %4d mV  零点 %7.1f  Ib %+7.3f A\n",
           BOARD_CS_B_GPIO, (int)CH_B, s_raw_b, s_mv_b,
           (double)(s_calibrated ? s_off_b_mv : 1650.0f), (double)s_ib);
    printf("   空闲时应两路都在 1650 mV 附近。\n");
}
