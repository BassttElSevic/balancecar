/*
 * ============================================================================
 *  balancecar —— 主程序
 *
 *  本文件只负责"组装"：把各个组件的实例建起来、给它们配置、然后跑控制循环。
 *  引脚/地址/参数一律取自 components/board/include/board_pins.h，本文件不出现裸引脚号。
 *
 *  组件:
 *      i2c_bus        I2C 主机总线（存储由本文件提供）
 *      encoder        AS5600 磁编码器（挂在 i2c_bus 上）
 *      current_sense  INA240 x2 -> ADC1 双相电流采样
 *      motor          MCPWM 三相驱动（保持原样）
 * ============================================================================
 */

#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "board_pins.h"
#include "i2c_bus.h"
#include "motor.h"
#include "encoder.h"
#include "current_sense.h"

#define TWO_PI  6.28318530718f
#define RAD2DEG (180.0f / (float)M_PI)

/* ============================================================================
 *  实例（存储由本文件提供，组件内部不使用堆）
 * ============================================================================ */

static i2c_bus_t       s_i2c;
static encoder_t       s_enc;
static current_sense_t s_cs;

/* ============================================================================
 *  配置（全部取自 board_pins.h）
 * ============================================================================ */

static const i2c_bus_cfg_t s_i2c_cfg = {
    .port                   = BOARD_I2C_PORT,
    .sda_gpio               = BOARD_I2C_SDA_GPIO,
    .scl_gpio               = BOARD_I2C_SCL_GPIO,
    .freq_hz                = BOARD_I2C_FREQ_HZ,
    .timeout_ms             = I2C_BUS_DEFAULT_TIMEOUT_MS,
    .glitch_ignore_cnt      = I2C_BUS_DEFAULT_GLITCH_CNT,
    .enable_internal_pullup = true,
};

static const encoder_cfg_t s_enc_cfg = {
    .addr7     = ENCODER_I2C_ADDR,
    .direction = ENCODER_DIRECTION,
};

static const current_sense_cfg_t s_cs_cfg = {
    .unit       = BOARD_CS_ADC_UNIT,
    .chan_a     = BOARD_CS_A_CHANNEL,
    .chan_b     = BOARD_CS_B_CHANNEL,
    .atten      = BOARD_CS_ATTEN,
    .bitwidth   = BOARD_CS_BITWIDTH,
    .gpio_a     = BOARD_CS_A_GPIO,
    .gpio_b     = BOARD_CS_B_GPIO,
    .shunt_ohms = BOARD_CS_SHUNT_OHMS,
    .amp_gain   = BOARD_CS_AMP_GAIN,
};

/* ============================================================================
 *  编码器采样线程（独占 I2C）
 * ============================================================================ */

#define ENC_TASK_PERIOD_MS  2       /* 500 Hz */
#define ENC_TASK_STACK      4096
#define ENC_TASK_PRIO       5
#define ENC_TASK_CORE       1

typedef struct {
    volatile uint16_t raw_min;
    volatile uint16_t raw_max;
    volatile uint32_t count;
} enc_window_t;

static enc_window_t s_win;

static void enc_window_reset(void)
{
    s_win.raw_min = (uint16_t)AS5600_COUNTS_PER_REV;
    s_win.raw_max = 0;
    s_win.count   = 0;
}

static void encoder_task(void *arg)
{
    (void)arg;
    enc_window_reset();
    for (;;) {
        (void)encoder_update(&s_enc);
        const uint16_t r = encoder_get_raw_angle(&s_enc);
        if (r < s_win.raw_min) s_win.raw_min = r;
        if (r > s_win.raw_max) s_win.raw_max = r;
        s_win.count++;
        vTaskDelay(pdMS_TO_TICKS(ENC_TASK_PERIOD_MS));
    }
}

static void enc_window_report(const char *label)
{
    const uint16_t mn = s_win.raw_min;
    const uint16_t mx = s_win.raw_max;
    const uint32_t n  = s_win.count;
    const uint16_t span = (uint16_t)(mx - mn);

    printf("    %-26s raw %4u~%4u  极差 %5u  %s   单圈角 %7.2f°  多圈 %ld 圈   (%lu 次)\n",
           label, mn, mx, span, (span >= 20) ? "在动" : "不动",
           (double)(encoder_get_single_turn(&s_enc) * RAD2DEG),
           (long)encoder_get_full_rotations(&s_enc), (unsigned long)n);
    fflush(stdout);
}

/* ============================================================================ */

/** @brief 驱动电机做一次正弦换相，持续 run_ms 毫秒 */
static void motor_run_sine(float elec_hz, float amp, int run_ms)
{
    const int   steps  = run_ms / 5;
    const float dtheta = TWO_PI * elec_hz * 0.005f;
    static float th;

    for (int i = 0; i < steps; i++) {
        motor_set_uvw(amp * cosf(th),
                      amp * cosf(th - 2.0943951f),
                      amp * cosf(th + 2.0943951f));
        th += dtheta;
        if (th >= TWO_PI) th -= TWO_PI;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    printf("\n================ balancecar 启动 ================\n");

    /* ---- 1) I2C 总线 ---- */
    ESP_ERROR_CHECK(i2c_bus_init(&s_i2c, &s_i2c_cfg));

    /* ---- 2) 编码器 ---- */
    ESP_ERROR_CHECK(encoder_init(&s_enc, &s_i2c, &s_enc_cfg));
    encoder_dump_regs(&s_enc);
    encoder_refresh_diag(&s_enc);
    encoder_print_diag(&s_enc);

    /* ---- 3) 电流采样 ---- */
    ESP_ERROR_CHECK(current_sense_init(&s_cs, &s_cs_cfg));

    /* ---- 4) 电机 ---- */
    ESP_ERROR_CHECK(motor_init());
    motor_enable(false);

    /* ---- 5) 零点校准（此时电机静止、无电流） ---- */
    ESP_ERROR_CHECK(current_sense_calibrate(&s_cs));
    current_sense_print_diag(&s_cs);

    /* ---- 6) 启动编码器采样线程 ---- */
    xTaskCreatePinnedToCore(encoder_task, "enc", ENC_TASK_STACK, NULL,
                            ENC_TASK_PRIO, NULL, ENC_TASK_CORE);
    vTaskDelay(pdMS_TO_TICKS(300));

    /* ---- 7) 基线：电机不动 ---- */
    printf("\n---- 基线: 电机失能 (2s) ----\n");
    enc_window_reset();
    vTaskDelay(pdMS_TO_TICKS(2000));
    enc_window_report("电机失能");
    ESP_ERROR_CHECK(current_sense_update(&s_cs));
    printf("    电流: Ia=%+7.3f A   Ib=%+7.3f A   Ic=%+7.3f A\n",
           (double)current_sense_get_a(&s_cs),
           (double)current_sense_get_b(&s_cs),
           (double)current_sense_get_c(&s_cs));

    /* ---- 8) 驱动电机，慢慢升频 ---- */
    printf("\n---- 驱动: 正弦换相 0.5 -> 6 Hz ----\n");
    motor_enable(true);
    for (float f = 0.5f; f <= 6.0f; f += 0.5f) {
        enc_window_reset();
        motor_run_sine(f, 1.0f, 1000);
        ESP_ERROR_CHECK(current_sense_update(&s_cs));
        printf("    %5.2f Hz (%5.1f RPM)  ", (double)f, (double)(f / MOTOR_POLE_PAIRS * 60.0f));
        enc_window_report("");
        printf("        电流 Ia=%+7.3f A   Ib=%+7.3f A   Ic=%+7.3f A\n",
               (double)current_sense_get_a(&s_cs),
               (double)current_sense_get_b(&s_cs),
               (double)current_sense_get_c(&s_cs));
    }

    /* ---- 9) 停 ---- */
    motor_enable(false);
    printf("\n---- 收尾 ----\n");
    enc_window_reset();
    vTaskDelay(pdMS_TO_TICKS(2000));
    enc_window_report("已失能");
    encoder_refresh_diag(&s_enc);
    encoder_print_diag(&s_enc);
    current_sense_print_diag(&s_cs);

    printf("\n完成。\n");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
