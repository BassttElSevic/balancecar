/*
 * ============================================================================
 *  adc_ab_test —— 复现 / 定位 GPIO21(ADC1_CH5) 读数卡死
 *
 *  硬件（来自 components/board/include/board_pins.h）:
 *      A 相采样  INA240 -> P4 GPIO21 = ADC1_CH5   (BOARD_CS_A_CHANNEL = 5)
 *      B 相采样  INA240 -> P4 GPIO20 = ADC1_CH4   (BOARD_CS_B_CHANNEL = 4)
 *      电机三相  GPIO32 / GPIO27 / GPIO26,  使能 GPIO33 (MOTOR_EN)
 *
 *  已知现象（详见仓库根目录 ISSUE_ADC_GPIO21.md）:
 *      走 components/current/current_sense.c 这条路时，
 *      CH5 raw 恒为 3464~3469（≈3360mV），CH4 raw 恒为 1~60（≈0mV），
 *      而同样的两个引脚用官方例程写法读，是 ~1700（≈1650mV）且在动。
 *
 *  本工程把 4 组实验放在同一个固件里，便于逐组对比。
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "board_pins.h"
#include "motor.h"
#include "current_sense.h"

#define CH_A       ADC_CHANNEL_5      /* GPIO21 */
#define CH_B       ADC_CHANNEL_4      /* GPIO20 */
#define ATTEN      ADC_ATTEN_DB_12
#define N          32                 /* 每组采样次数 */

static const char *TAG = "adc_ab";

/* -------------------------------------------------------------------------- */
/*  通用统计                                                                   */
/* -------------------------------------------------------------------------- */

typedef struct {
    int min, max;
    long avg;
} stat_t;

static stat_t stat_of(const int *v, int n)
{
    stat_t s = { .min = v[0], .max = v[0], .avg = 0 };
    long sum = 0;
    for (int i = 0; i < n; i++) {
        if (v[i] < s.min) s.min = v[i];
        if (v[i] > s.max) s.max = v[i];
        sum += v[i];
    }
    s.avg = sum / n;
    return s;
}

static void report(const char *label, const int *a, const int *b,
                   adc_cali_handle_t ca, adc_cali_handle_t cb)
{
    stat_t sa = stat_of(a, N);
    stat_t sb = stat_of(b, N);
    int mva = -1, mvb = -1;
    if (ca) adc_cali_raw_to_voltage(ca, (int)sa.avg, &mva);
    if (cb) adc_cali_raw_to_voltage(cb, (int)sb.avg, &mvb);
    printf("  %-34s | CH5(GPIO21) avg=%5ld [%4d..%4d] %5dmV | CH4(GPIO20) avg=%5ld [%4d..%4d] %5dmV\n",
           label, sa.avg, sa.min, sa.max, mva, sb.avg, sb.min, sb.max, mvb);
    fflush(stdout);
}

/* -------------------------------------------------------------------------- */
/*  实验 1: 官方例程 examples/peripherals/adc/oneshot_read 的写法               */
/*          unit = { .unit_id }  (clk_src/ulp_mode 均为 0)                      */
/*          channel bitwidth = ADC_BITWIDTH_DEFAULT                            */
/* -------------------------------------------------------------------------- */

static void test1_official(void)
{
    printf("\n### 实验1: 官方例程写法 (unit={.unit_id}, bitwidth=DEFAULT) ###\n");

    adc_oneshot_unit_handle_t unit = NULL;
    adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc, &unit));

    adc_oneshot_chan_cfg_t ch = { .atten = ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit, CH_A, &ch));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit, CH_B, &ch));

    adc_cali_handle_t ca = NULL, cb = NULL;
    adc_cali_curve_fitting_config_t cca = {
        .unit_id = ADC_UNIT_1, .chan = CH_A, .atten = ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT };
    adc_cali_curve_fitting_config_t ccb = {
        .unit_id = ADC_UNIT_1, .chan = CH_B, .atten = ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT };
    bool oka = (adc_cali_create_scheme_curve_fitting(&cca, &ca) == ESP_OK);
    bool okb = (adc_cali_create_scheme_curve_fitting(&ccb, &cb) == ESP_OK);
    printf("  校准句柄: CH5=%s  CH4=%s\n", oka ? "OK" : "FAIL", okb ? "OK" : "FAIL");

    int a[N], b[N];
    for (int i = 0; i < N; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(unit, CH_A, &a[i]));
        ESP_ERROR_CHECK(adc_oneshot_read(unit, CH_B, &b[i]));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    report("官方例程写法", a, b, ca, cb);

    if (ca) adc_cali_delete_scheme_curve_fitting(ca);
    if (cb) adc_cali_delete_scheme_curve_fitting(cb);
    ESP_ERROR_CHECK(adc_oneshot_del_unit(unit));
}

/* -------------------------------------------------------------------------- */
/*  实验 2: 建完 unit 立刻读（两个通道都还没 config），再看 config 之后          */
/*          用来判断 ADC 是“一开始就不对”还是“配置之后才不对”                  */
/* -------------------------------------------------------------------------- */

static void probe(const adc_oneshot_unit_handle_t unit, const char *step)
{
    int a = -1, b = -1;
    const esp_err_t ea = adc_oneshot_read(unit, CH_A, &a);
    const esp_err_t eb = adc_oneshot_read(unit, CH_B, &b);
    printf("  [probe] %-26s -> CH5(raw)=%5d(%s)  CH4(raw)=%5d(%s)\n",
           step, a, esp_err_to_name(ea), b, esp_err_to_name(eb));
    fflush(stdout);
}

static void test2_step_by_step(void)
{
    printf("\n### 实验2: 逐步骤插桩 (unit 用【项目同款】配置) ###\n");

    adc_oneshot_unit_handle_t unit = NULL;
    adc_oneshot_unit_init_cfg_t uc = {
        .unit_id  = (adc_unit_t)BOARD_CS_ADC_UNIT,   /* 1 */
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&uc, &unit));
    probe(unit, "new_unit 之后(未config)");

    adc_oneshot_chan_cfg_t ch = {
        .atten    = ATTEN,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit, CH_A, &ch));
    probe(unit, "config CH5 之后");

    ESP_ERROR_CHECK(adc_oneshot_config_channel(unit, CH_B, &ch));
    probe(unit, "config CH4 之后");

    adc_cali_handle_t ca = NULL;
    adc_cali_curve_fitting_config_t cca = {
        .unit_id = ADC_UNIT_1, .chan = CH_A, .atten = ATTEN, .bitwidth = ADC_BITWIDTH_12 };
    adc_cali_create_scheme_curve_fitting(&cca, &ca);
    probe(unit, "cali 创建之后");

    if (ca) adc_cali_delete_scheme_curve_fitting(ca);
    ESP_ERROR_CHECK(adc_oneshot_del_unit(unit));
}

/* -------------------------------------------------------------------------- */
/*  实验 3: 走 components/current/current_sense.c                              */
/* -------------------------------------------------------------------------- */

static void test3_current_sense(void)
{
    printf("\n### 实验3: 走 components/current/current_sense.c ###\n");

    printf("  current_sense_init      : %s\n", esp_err_to_name(current_sense_init()));
    printf("  current_sense_calibrate : %s\n", esp_err_to_name(current_sense_calibrate()));

    printf("  零点校准: A=%.1f mV  B=%.1f mV\n",
           (double)current_get_offset_mv_a(), (double)current_get_offset_mv_b());

    printf("  连续读数 (原版 current_sense 只暴露 mV, 不暴露 raw):\n");
    for (int i = 0; i < 12; i++) {
        current_sense_update();
        printf("      A/CH5 = %4d mV    Ia=%+7.3f A   |   B/CH4 = %4d mV    Ib=%+7.3f A\n",
               current_get_mv_a(), (double)current_get_a(),
               current_get_mv_b(), (double)current_get_b());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* -------------------------------------------------------------------------- */
/*  实验 4: 先 motor_init()（MCPWM 30kHz 起振, EN 拉低），再走 current_sense    */
/* -------------------------------------------------------------------------- */

static void test4_motor_plus_current_sense(void)
{
    printf("\n### 实验4: motor_init() + current_sense 路径 ###\n");
    printf("  motor_init 之前 EN(GPIO%d) 电平未定义; motor_init 会把它拉低\n", MOTOR_EN_GPIO);
    ESP_ERROR_CHECK(motor_init());
    printf("  motor_init 之后, 驱动器失能(EN 低)\n");

    printf("  连续读数 (原版 current_sense 只暴露 mV, 不暴露 raw):\n");
    for (int i = 0; i < 12; i++) {
        current_sense_update();
        printf("      A/CH5 = %4d mV    Ia=%+7.3f A   |   B/CH4 = %4d mV    Ib=%+7.3f A\n",
               current_get_mv_a(), (double)current_get_a(),
               current_get_mv_b(), (double)current_get_b());
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* -------------------------------------------------------------------------- */

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("\n\n");
    printf("###########################################################################\n");
    printf("#  adc_ab_test: GPIO21(ADC1_CH5) / GPIO20(ADC1_CH4) 诊断                    #\n");
    printf("#  期望(引脚上无电流时 INA240 输出约中点): 两路 raw 都应在 1650~1750 附近    #\n");
    printf("###########################################################################\n");

    /* 实验1/2 都不碰电机, 所以先把电机使能脚固定为失能,
     * 避免 L6234 的 EN 悬空导致 INA240 输出偏离中点。 */
    gpio_config_t en = {
        .pin_bit_mask = 1ULL << MOTOR_EN_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&en);
    gpio_set_level(MOTOR_EN_GPIO, 0);
    printf("\n(已先把 MOTOR_EN(GPIO%d) 拉低 = 驱动器失能)\n", MOTOR_EN_GPIO);

    test1_official();
    test2_step_by_step();
    test3_current_sense();
    test4_motor_plus_current_sense();

    printf("\n\n---------------------------------------------------------------------------\n");
    printf("  结束。请把以上完整输出回贴。\n");
    printf("---------------------------------------------------------------------------\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
