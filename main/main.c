/* 临时固件：持续以 5Hz 打印 AS5600 raw，方便手动转轴观察 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "board_pins.h"
#include "encoder.h"

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(200));

    printf("\n[encoder_init] %s\n", esp_err_to_name(encoder_init()));
    encoder_dump_regs();

    printf("\n==================================================================\n");
    printf("  现在开始连续打印 raw(0x0C)。请用手把电机轴转一整圈以上，\n");
    printf("  观察 raw 的值会不会变（应该从 0 扫到 4095）。\n");
    printf("==================================================================\n\n");
    fflush(stdout);

    uint16_t mn = 4095, mx = 0;
    for (int i = 0; ; i++) {
        encoder_update();
        const uint16_t r = encoder_get_raw();
        if (r < mn) mn = r;
        if (r > mx) mx = r;
        printf("[%4d]  raw=%4u (%.1f°)   0x0E=%4u   AGC=%3u  MAG=%4u  STATUS=0x%02X   | 累计范围 %u~%u 极差 %u\n",
               i, r, r * 360.0f / 4096.0f, encoder_get_raw_angle(),
               encoder_get_agc(), encoder_get_magnitude(), encoder_get_status(),
               mn, mx, (unsigned)(mx - mn));
        if (i % 50 == 49) { mn = 4095; mx = 0; }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
