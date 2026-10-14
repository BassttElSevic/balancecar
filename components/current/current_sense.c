/*
 * ============================================================================
 *  balancecar — 双电阻在线电流采样 (INA240 x2 -> ADC1)
 * ============================================================================
 */

#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "board_pins.h"
#include "current_sense.h"

static const char *TAG = "current";

#define CS_ATTEN        ADC_ATTEN_DB_12     /* 量程约 0 ~ 3.3V */
#define CS_BITWIDTH     ADC_BITWIDTH_12
#define CS_CAL_SAMPLES  256                 /* 零点校准采样次数 */

typedef struct {
    adc_oneshot_unit_handle_t unit;
    adc_cali_handle_t         cali;
    bool     ready;
    bool     calibrated;
    float    offset_a_mv;   /* A 相零点 (mV) */
    float    offset_b_mv;   /* B 相零点 (mV) */
    int      raw_mv_a;      /* A 相当前电压 (mV) */
    int      raw_mv_b;      /* B 相当前电压 (mV) */
    float    ia, ib, ic;    /* 三相电流 (A) */
} cs_ctx_t;

static cs_ctx_t s_cs;

/* -------------------------------------------------------------------------- */

esp_err_t current_sense_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = (adc_unit_t)BOARD_CS_ADC_UNIT,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_cs.unit), TAG, "创建 ADC 单元失败");

    const adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = CS_ATTEN,
        .bitwidth = CS_BITWIDTH,
    };
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(s_cs.unit, (adc_channel_t)BOARD_CS_A_CHANNEL, &chan_cfg),
        TAG, "配置 A 相 ADC 通道失败");
    ESP_RETURN_ON_ERROR(
        adc_oneshot_config_channel(s_cs.unit, (adc_channel_t)BOARD_CS_B_CHANNEL, &chan_cfg),
        TAG, "配置 B 相 ADC 通道失败");

    /* 校准句柄：P4 支持曲线拟合校准 */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = (adc_unit_t)BOARD_CS_ADC_UNIT,
        .atten    = CS_ATTEN,
        .bitwidth = CS_BITWIDTH,
    };
    const esp_err_t cali_err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cs.cali);
    if (cali_err != ESP_OK) {
        ESP_LOGW(TAG, "ADC 校准创建失败 (%s)，将退化为原始值换算", esp_err_to_name(cali_err));
        s_cs.cali = NULL;
    }

    s_cs.ready = true;
    ESP_LOGI(TAG, "电流采样初始化完成: A=GPIO%d(ADC1_CH%d)  B=GPIO%d(ADC1_CH%d)  灵敏度 %.2f V/A",
             BOARD_CS_A_GPIO, BOARD_CS_A_CHANNEL,
             BOARD_CS_B_GPIO, BOARD_CS_B_CHANNEL,
             (double)BOARD_CS_SENS_V_PER_A);
    return ESP_OK;
}

/* 读一个通道的电压 (mV) */
static esp_err_t read_mv(adc_channel_t chan, int *out_mv)
{
    int raw = 0;
    ESP_RETURN_ON_ERROR(adc_oneshot_read(s_cs.unit, chan, &raw), TAG, "ADC 读取失败");

    if (s_cs.cali) {
        int mv = 0;
        ESP_RETURN_ON_ERROR(adc_cali_raw_to_voltage(s_cs.cali, raw, &mv), TAG, "ADC 校准换算失败");
        *out_mv = mv;
    } else {
        /* 无校准：按 12bit / 3.3V 粗略换算 */
        *out_mv = (int)((float)raw * 3300.0f / 4095.0f);
    }
    return ESP_OK;
}

esp_err_t current_sense_calibrate(void)
{
    ESP_RETURN_ON_FALSE(s_cs.ready, ESP_ERR_INVALID_STATE, TAG, "电流采样未初始化");

    int64_t sum_a = 0, sum_b = 0;
    for (int i = 0; i < CS_CAL_SAMPLES; i++) {
        int mv = 0;
        ESP_RETURN_ON_ERROR(read_mv((adc_channel_t)BOARD_CS_A_CHANNEL, &mv), TAG, "A 相校准读取失败");
        sum_a += mv;
        ESP_RETURN_ON_ERROR(read_mv((adc_channel_t)BOARD_CS_B_CHANNEL, &mv), TAG, "B 相校准读取失败");
        sum_b += mv;
    }

    s_cs.offset_a_mv = (float)sum_a / (float)CS_CAL_SAMPLES;
    s_cs.offset_b_mv = (float)sum_b / (float)CS_CAL_SAMPLES;
    s_cs.calibrated  = true;

    ESP_LOGI(TAG, "零点校准完成: A=%.1f mV  B=%.1f mV  (共 %d 次采样)",
             (double)s_cs.offset_a_mv, (double)s_cs.offset_b_mv, CS_CAL_SAMPLES);
    return ESP_OK;
}

esp_err_t current_sense_update(void)
{
    ESP_RETURN_ON_FALSE(s_cs.ready, ESP_ERR_INVALID_STATE, TAG, "电流采样未初始化");

    int mv_a = 0, mv_b = 0;
    ESP_RETURN_ON_ERROR(read_mv((adc_channel_t)BOARD_CS_A_CHANNEL, &mv_a), TAG, "读 A 相失败");
    ESP_RETURN_ON_ERROR(read_mv((adc_channel_t)BOARD_CS_B_CHANNEL, &mv_b), TAG, "读 B 相失败");

    s_cs.raw_mv_a = mv_a;
    s_cs.raw_mv_b = mv_b;

    /* 电压差 -> 电流。未校准时零点按 1650mV (3.3V 的一半) 估算 */
    const float zero_a = s_cs.calibrated ? s_cs.offset_a_mv : 1650.0f;
    const float zero_b = s_cs.calibrated ? s_cs.offset_b_mv : 1650.0f;

    s_cs.ia = (float)(mv_a - (int)zero_a) / 1000.0f / BOARD_CS_SENS_V_PER_A;
    s_cs.ib = (float)(mv_b - (int)zero_b) / 1000.0f / BOARD_CS_SENS_V_PER_A;
    s_cs.ic = -s_cs.ia - s_cs.ib;   /* 基尔霍夫: Ia + Ib + Ic = 0 */
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */

float current_get_a(void) { return s_cs.ia; }
float current_get_b(void) { return s_cs.ib; }
float current_get_c(void) { return s_cs.ic; }

int   current_get_mv_a(void) { return s_cs.raw_mv_a; }
int   current_get_mv_b(void) { return s_cs.raw_mv_b; }
float current_get_offset_mv_a(void) { return s_cs.offset_a_mv; }
float current_get_offset_mv_b(void) { return s_cs.offset_b_mv; }

void current_sense_print_diag(void)
{
    int mv_a = 0, mv_b = 0;
    const esp_err_t ea = read_mv((adc_channel_t)BOARD_CS_A_CHANNEL, &mv_a);
    const esp_err_t eb = read_mv((adc_channel_t)BOARD_CS_B_CHANNEL, &mv_b);

    printf("[电流自检] INA240 x2 -> ADC1\n");
    printf("   A 相  GPIO%-3d CH%d  ", BOARD_CS_A_GPIO, BOARD_CS_A_CHANNEL);
    if (ea == ESP_OK) {
        printf("电压 %4d mV   零点 %7.1f mV   电流 %+7.3f A\n",
               mv_a, (double)(s_cs.calibrated ? s_cs.offset_a_mv : 1650.0f),
               (double)s_cs.ia);
    } else {
        printf("读取失败 (%s)\n", esp_err_to_name(ea));
    }

    printf("   B 相  GPIO%-3d CH%d  ", BOARD_CS_B_GPIO, BOARD_CS_B_CHANNEL);
    if (eb == ESP_OK) {
        printf("电压 %4d mV   零点 %7.1f mV   电流 %+7.3f A\n",
               mv_b, (double)(s_cs.calibrated ? s_cs.offset_b_mv : 1650.0f),
               (double)s_cs.ib);
    } else {
        printf("读取失败 (%s)\n", esp_err_to_name(eb));
    }

    printf("   C 相 (推算)          Ic = -Ia - Ib = %+7.3f A\n", (double)s_cs.ic);
    printf("   灵敏度 %.2f V/A   校准状态 %s\n",
           (double)BOARD_CS_SENS_V_PER_A, s_cs.calibrated ? "✅ 已校准" : "⚠️ 未校准");
}
