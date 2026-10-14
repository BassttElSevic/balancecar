/*
 * ============================================================================
 *  balancecar — 电机开环旋转 + 编码器 + 电流
 *
 *  引脚:
 *     三相   GPIO32 / GPIO27 / GPIO26
 *     使能   GPIO33
 *     编码器 SDA=GPIO5  SCL=GPIO49
 *     电流   A=GPIO21  B=GPIO20
 * ============================================================================
 */

#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_chip_info.h"

#include "board_pins.h"
#include "motor.h"
#include "encoder.h"
#include "current_sense.h"

/* ============================================================================
 *                              测试参数
 * ============================================================================ */

#define TEST_ELEC_FREQ_HZ   20.0f   /* 电角度旋转频率（提高 4 倍） */
#define TEST_AMPLITUDE      0.50f   /* 相电压幅值 0~1（提高，保证高速时扭矩够） */
#define TEST_RAMP_SECONDS   2.0f
#define TEST_CTRL_HZ        1000
#define TEST_DIRECTION      (+1.0f)

#define TWO_PI              6.28318530718f
#define DEG_120_RAD         2.09439510239f

/* ============================================================================ */

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    esp_chip_info_t chip;
    esp_chip_info(&chip);

    printf("\n########################################################\n");
    printf("#  balancecar · 开环旋转 + 编码器 + 电流                #\n");
    printf("########################################################\n");
    printf("  三相 GPIO%d/%d/%d   使能 GPIO%d\n",
           MOTOR_PHASE_A_GPIO, MOTOR_PHASE_B_GPIO, MOTOR_PHASE_C_GPIO, MOTOR_EN_GPIO);
    printf("  PWM  %d kHz 中心对齐    电频率 %.1f Hz  幅值 %.2f\n",
           MOTOR_PWM_FREQ_HZ / 1000, TEST_ELEC_FREQ_HZ, TEST_AMPLITUDE);
    printf("  I2C  SDA=GPIO%d SCL=GPIO%d      电流 A=GPIO%d B=GPIO%d\n",
           BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO, BOARD_CS_A_GPIO, BOARD_CS_B_GPIO);
    printf("  ESP32-P4 rev v%d.%d\n", chip.revision / 100, chip.revision % 100);
    printf("########################################################\n\n");

    /* ---- 电流采样：电机静止时校准 ---- */
    ESP_ERROR_CHECK(motor_init());
    ESP_ERROR_CHECK(motor_zero());

    const bool cs_ok = (current_sense_init() == ESP_OK) &&
                       (current_sense_calibrate() == ESP_OK);

    /* ---- 编码器 ---- */
    const bool enc_ok = (encoder_init() == ESP_OK);

    /* ---- 使能电机 ---- */
    motor_enable(true);
    printf("\n>>> 电机开始旋转 <<<\n\n");

    const float  dt      = 1.0f / (float)TEST_CTRL_HZ;
    const float  d_theta = TWO_PI * TEST_ELEC_FREQ_HZ * dt * TEST_DIRECTION;
    const int64_t t0_us  = esp_timer_get_time();

    TickType_t period_ticks = pdMS_TO_TICKS(1000 / TEST_CTRL_HZ);
    if (period_ticks == 0) {
        period_ticks = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();

    float  theta     = 0.0f;
    double theta_tot = 0.0;
    float  amp       = 0.0f;
    int    last_log  = 0;

    /* 统计最近 5 秒编码器 raw 的波动范围，用于判断"到底动没动" */
    uint16_t raw_min  = 4095;
    uint16_t raw_max  = 0;

    while (1) {
        vTaskDelayUntil(&last_wake, period_ticks);

        const float t_sec = (float)(esp_timer_get_time() - t0_us) / 1e6f;

        amp = (TEST_RAMP_SECONDS > 0.0f) ? (TEST_AMPLITUDE * t_sec / TEST_RAMP_SECONDS)
                                         : TEST_AMPLITUDE;
        if (amp > TEST_AMPLITUDE) {
            amp = TEST_AMPLITUDE;
        }

        theta_tot += (double)d_theta;
        theta += d_theta;
        if (theta >= TWO_PI) {
            theta -= TWO_PI;
        } else if (theta < 0.0f) {
            theta += TWO_PI;
        }

        const float u = amp * cosf(theta);
        const float v = amp * cosf(theta - DEG_120_RAD);
        const float w = amp * cosf(theta + DEG_120_RAD);

        ESP_ERROR_CHECK(motor_set_uvw(u, v, w));

        if (enc_ok) {
            encoder_update();
            const uint16_t r = encoder_get_raw();
            if (r < raw_min) raw_min = r;
            if (r > raw_max) raw_max = r;
        }
        if (cs_ok)  current_sense_update();

        if ((int)t_sec > last_log) {
            last_log = (int)t_sec;

            printf("[%3ds] 电角 %8.1f° (%5.2f 圈)  U=%5.1f%% V=%5.1f%% W=%5.1f%%\n",
                   (int)t_sec, theta_tot * 180.0 / M_PI, theta_tot / TWO_PI,
                   (u * 0.5f + 0.5f) * 100.0f,
                   (v * 0.5f + 0.5f) * 100.0f,
                   (w * 0.5f + 0.5f) * 100.0f);

            if (enc_ok) {
                const float vel     = encoder_get_velocity();
                const float mech_hz = fabsf(vel) / TWO_PI;
                const uint32_t span = (uint32_t)raw_max - (uint32_t)raw_min;

                printf("        编码器 raw=%4u  raw0x0C=%4u  AGC=%3u  MAG=%4u  "
                       "单圈=%6.1f°  速度=%8.1f°/s  最近5秒Δ%u %s\n",
                       encoder_get_raw(),
                       encoder_get_raw_angle(),
                       encoder_get_agc(),
                       encoder_get_magnitude(),
                       encoder_get_single_turn() * 180.0f / (float)M_PI,
                       vel * 180.0f / (float)M_PI,
                       (unsigned)span,
                       (span >= 20) ? "✅ 在动" : "❌ 没动");

                if (mech_hz > 0.05f) {
                    printf("        ★ 极对数 = %.1f / %.3f = %.2f\n",
                           TEST_ELEC_FREQ_HZ, mech_hz, TEST_ELEC_FREQ_HZ / mech_hz);
                }

                /* 每 5 秒重置统计窗口 */
                if ((int)t_sec % 5 == 0) {
                    raw_min = 4095;
                    raw_max = 0;
                }
            }

            if (cs_ok) {
                printf("        电流     A:raw%4d %4dmV Ia=%+6.3fA   B:raw%4d %4dmV Ib=%+6.3fA   Ic=%+6.3fA\n",
                       current_get_raw_a(), current_get_mv_a(), (double)current_get_a(),
                       current_get_raw_b(), current_get_mv_b(), (double)current_get_b(),
                       (double)current_get_c());
            }
        }
    }
}
