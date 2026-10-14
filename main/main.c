/*
 * 临时诊断：
 *   线程 A (encoder_task, CPU1) —— 独占 I2C，500Hz 轮询 AS5600
 *   线程 B (app_main,   CPU0) —— 驱动电机 / 等待手转
 * 编码器读数逻辑照 DengFOC；I2C 初始化照官方例程。
 */
#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "board_pins.h"
#include "motor.h"
#include "encoder.h"

#define TWO_PI 6.28318530718f

static volatile uint16_t s_raw_min = 4095, s_raw_max = 0;
static volatile float    s_ang_min = 1e9f,  s_ang_max = -1e9f;
static volatile uint32_t s_n = 0;

static void encoder_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(20));
    while (1) {
        encoder_update();
        const uint16_t r = encoder_get_raw();
        const float    a = encoder_get_single_turn();      /* 0~2π */
        if (r < s_raw_min) s_raw_min = r;
        if (r > s_raw_max) s_raw_max = r;
        if (a < s_ang_min) s_ang_min = a;
        if (a > s_ang_max) s_ang_max = a;
        s_n++;
        vTaskDelay(pdMS_TO_TICKS(2));                      /* 500 Hz */
    }
}

static void win_reset(void)
{
    s_raw_min = 4095; s_raw_max = 0;
    s_ang_min = 1e9f; s_ang_max = -1e9f;
    s_n = 0;
}

static void win_report(const char *label)
{
    const uint16_t mn = s_raw_min, mx = s_raw_max;
    const float    amn = s_ang_min, amx = s_ang_max;
    printf("    %-24s raw %4u~%4u 极差 %5u  %s | 单圈角 %6.1f°~%6.1f° 跨 %6.1f° | %lu 次\n",
           label, mn, mx, (unsigned)(mx - mn),
           (mx - mn) >= 20 ? "✅动" : "❌不动",
           (double)(amn * 180.0f / (float)M_PI), (double)(amx * 180.0f / (float)M_PI),
           (double)((amx - amn) * 180.0f / (float)M_PI), (unsigned long)s_n);
    fflush(stdout);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    printf("\n[encoder_init] %s\n", esp_err_to_name(encoder_init()));
    encoder_dump_regs();

    xTaskCreatePinnedToCore(encoder_task, "enc", 4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("\n============ 阶段 1: 什么都不做，请你【用手转轴】 ============\n");
    win_reset();
    vTaskDelay(pdMS_TO_TICKS(8000));
    win_report("手转轴(8s)");

    printf("\n============ 阶段 2: 驱动电机 慢速升频 0.5→10Hz ============\n");
    printf("[motor_init] %s\n", esp_err_to_name(motor_init()));
    motor_enable(true);
    win_reset();

    float th = 0.0f;
    for (float f = 0.5f; f <= 10.0f; f += 0.5f) {
        const float dth = TWO_PI * f * 0.005f;
        for (int i = 0; i < 200; i++) {          /* 200*5ms = 1s */
            motor_set_uvw(cosf(th), cosf(th - 2.0943951f), cosf(th + 2.0943951f));
            th += dth;
            if (th >= TWO_PI) th -= TWO_PI;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        printf("    电频率 %5.2f Hz (%5.1f RPM)  ", f, f / 7.0f * 60.0f);
        win_report("");
        win_reset();
    }

    printf("\n============ 阶段 3: 失能，再手转一次 ============\n");
    motor_enable(false);
    win_reset();
    printf("    >>> 请再用手转一下轴 <<<\n");
    vTaskDelay(pdMS_TO_TICKS(8000));
    win_report("手转轴(8s)");

    printf("\n--------------- 汇总 ---------------\n");
    encoder_refresh_diag();
    encoder_print_diag();

    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
