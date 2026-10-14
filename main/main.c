/* 换线后重测: 编码器 500Hz 独立线程 + 电机驱动, 每秒报极差 */
#include <math.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board_pins.h"
#include "motor.h"
#include "encoder.h"

#define TWO_PI 6.28318530718f

static volatile uint16_t s_mn = 4095, s_mx = 0;
static volatile uint32_t s_n = 0;

static void enc_task(void *a)
{
    (void)a;
    vTaskDelay(pdMS_TO_TICKS(20));
    for (;;) {
        encoder_update();
        const uint16_t r = encoder_get_raw();
        if (r < s_mn) s_mn = r;
        if (r > s_mx) s_mx = r;
        s_n++;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
static void win(void) { s_mn = 4095; s_mx = 0; s_n = 0; }

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));
    printf("\n[encoder_init] %s\n", esp_err_to_name(encoder_init()));
    printf("[motor_init]   %s\n", esp_err_to_name(motor_init()));
    encoder_dump_regs();

    xTaskCreatePinnedToCore(enc_task, "enc", 4096, NULL, 5, NULL, 1);
    vTaskDelay(pdMS_TO_TICKS(300));

    printf("\n--- 基线: 电机失能 (3s) ---\n");
    win(); vTaskDelay(pdMS_TO_TICKS(3000));
    printf("    raw %4u~%4u 极差 %5u (%lu 次)\n", s_mn, s_mx,
           (unsigned)(s_mx - s_mn), (unsigned long)s_n);

    printf("\n--- 驱动: 正弦换相 0.5->6Hz, 每秒报一次 ---\n");
    motor_enable(true);
    float th = 0.0f;
    for (float f = 0.5f; f <= 6.0f; f += 0.5f) {
        const float dth = TWO_PI * f * 0.005f;
        win();
        for (int i = 0; i < 200; i++) {
            motor_set_uvw(cosf(th), cosf(th - 2.0943951f), cosf(th + 2.0943951f));
            th += dth; if (th >= TWO_PI) th -= TWO_PI;
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        printf("    %5.2f Hz (%5.1f RPM)  raw %4u~%4u 极差 %5u  %s\n",
               f, f / 7.0f * 60.0f, s_mn, s_mx, (unsigned)(s_mx - s_mn),
               (s_mx - s_mn) >= 20 ? "✅ 在转" : "❌ 没转");
        fflush(stdout);
    }
    motor_enable(false);
    printf("\n完成\n");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
