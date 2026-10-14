/*
 * ============================================================================
 *  balancecar —— 电流采样验证（按 SimpleFOC test_current 的做法）
 *
 *  关键点（和文档一致）:
 *    * 验证电流采样时，力矩环仍然是【电压模式】、运动方式仍是【力矩】，
 *      电流环【不闭合】，电流只读出来监视 -> 用 m.current.d / m.current.q
 *    * 静测必须【用手把转子按住不动】，看 Iq 是否与电压成正比、Id 是否≈0
 *    * 转测松手，看 Iq 是否随转速上升（反电动势）而下降、Id 是否远小于 Iq
 *
 *  控制任务 1kHz：
 *      foc_motor_loop_foc(&m);
 *      foc_motor_move(&m, FOC_NOT_SET);
 * ============================================================================
 */

#include <math.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "board_pins.h"
#include "i2c_bus.h"
#include "motor.h"
#include "encoder.h"
#include "current_sense.h"
#include "foc_motor.h"
#include "foc_hw.h"
#include "app_cfg.h"

#define CTRL_HZ             APP_CTRL_HZ
#define CTRL_TASK_STACK     APP_CTRL_TASK_STACK
#define CTRL_TASK_PRIO      APP_CTRL_TASK_PRIO
#define CTRL_TASK_CORE      APP_CTRL_TASK_CORE
#define TELEMETRY_MS        APP_TELEMETRY_MS
#define RAD2DEG             APP_RAD_TO_DEG

/* ============================================================================
 *  实例（存储由本文件提供）
 * ============================================================================ */

static i2c_bus_t       s_i2c;
static encoder_t       s_enc;
static current_sense_t s_cs;
static foc_motor_t     s_foc;
static foc_sensor_t    s_sensor_if;
static foc_current_t   s_current_if;

static volatile bool     s_ctrl_paused;
static volatile uint32_t s_ctrl_loops;

/* ============================================================================
 *  配置
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
    .sign_a     = BOARD_CS_A_SIGN,
    .sign_b     = BOARD_CS_B_SIGN,
};

/* ============================================================================
 *  控制任务
 * ============================================================================ */

static void foc_ctrl_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CTRL_HZ);

    for (;;) {
        if (!s_ctrl_paused) {
            foc_motor_loop_foc(&s_foc);
            (void)foc_motor_move(&s_foc, FOC_NOT_SET);
            s_ctrl_loops++;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

/* ============================================================================
 *  遥测
 * ============================================================================ */

static void wait_operator(const char *prompt, int timeout_ms);

static void print_header(void)
{
    printf("      %-9s %-8s %-8s %-8s %-8s %-8s %-8s %-8s %-8s\n",
           "θ", "ω", "Ia", "Ib", "Ic", "Id", "Iq", "Uq", "θe");
    fflush(stdout);
}

static void print_row(void)
{
    printf("      %+7.1f° %+7.2f  %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f %+7.3f %+6.1f°\n",
           (double)(s_foc.shaft_angle * RAD2DEG),
           (double)s_foc.shaft_velocity,
           (double)s_foc.current_abc.a, (double)s_foc.current_abc.b, (double)s_foc.current_abc.c,
           (double)s_foc.current.d, (double)s_foc.current.q,
           (double)s_foc.voltage.q,
           (double)(s_foc.electrical_angle * RAD2DEG));
    fflush(stdout);
}

/** @brief 输出一段遥测，并返回这段时间内 Iq 的平均值 */
static float stage(const char *label, float uq, int duration_ms, const char *prompt)
{
    printf("\n    ---- %s   (Uq = %+.2f V) ----\n", label, (double)uq);
    if (prompt) {
        wait_operator(prompt, APP_CS_PROMPT_MS);
    }
    s_foc.target = uq;

    print_header();
    const int n = duration_ms / TELEMETRY_MS;
    float sum_q = 0.0f, sum_d = 0.0f;
    for (int i = 0; i < n; i++) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_MS));
        print_row();
        sum_q += s_foc.current.q;
        sum_d += s_foc.current.d;
    }

    const float avg_q = sum_q / (float)n;
    const float avg_d = sum_d / (float)n;
    printf("      => 平均 Iq=%+7.3f A  平均 Id=%+7.3f A\n", (double)avg_q, (double)avg_d);
    fflush(stdout);
    return avg_q;
}

/*
 * 等操作员按回车再继续（超时也会继续，方便无人值守跑）
 * 之前用的是固定延时，操作员还没准备好时间就到了。
 */
static void wait_operator(const char *prompt, int timeout_ms)
{
    printf("\n    >>> %s\n", prompt);
    printf("    >>> 按回车继续（不按的话 %d 秒后自动继续）<<<\n", timeout_ms / 1000);
    fflush(stdout);

    const int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (fl >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);
    }

    const int64_t t0 = esp_timer_get_time();
    bool pressed = false;
    while (((esp_timer_get_time() - t0) / 1000) < (int64_t)timeout_ms) {
        const int ch = fgetc(stdin);
        if (ch != EOF) {
            if (ch == '\n' || ch == '\r') {
                pressed = true;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(APP_OPERATOR_POLL_MS));
    }

    if (fl >= 0) {
        (void)fcntl(STDIN_FILENO, F_SETFL, fl);
    }
    printf("    >>> 继续（%s）<<<\n\n", pressed ? "操作员已确认" : "超时");
    fflush(stdout);
}


#if APP_RUN_OFFSET_SWEEP
/*
 * 极性判定（在零点偏移标定之后）
 *
 * 偏移调到 |Id| 最小之后，电流矢量已经落在 q 轴上，但可能在 -q 方向。
 * 全局翻转两路符号等价于把轴系再转 180°，所以这一步只需要判断 Iq 的符号。
 */
static void polarity_fix(foc_motor_t *m)
{
    printf("\n---- 极性判定 (Uq = %+.2f V) ----\n", (double)APP_OFFSET_V);

    s_ctrl_paused = true;
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
    m->motion      = FOC_MOTION_TORQUE;
    m->torque_ctrl = FOC_TORQUE_VOLTAGE;
    m->target      = APP_OFFSET_V;
    s_ctrl_paused  = false;
    vTaskDelay(pdMS_TO_TICKS(APP_OFFSET_SETTLE_MS));

    const int n = APP_OFFSET_AVG_MS / APP_TELEMETRY_MS;
    float sd = 0.0f, sq = 0.0f;
    for (int i = 0; i < n; i++) {
        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_MS));
        sd += m->current.d;
        sq += m->current.q;
    }
    s_ctrl_paused = true;
    m->target = APP_CS_V_ZERO;
    vTaskDelay(pdMS_TO_TICKS(APP_OFFSET_SETTLE_MS));

    const float id_avg = sd / (float)n;
    const float iq_avg = sq / (float)n;
    printf("      Id=%+.3f  Iq=%+.3f  ->  %s\n", (double)id_avg, (double)iq_avg,
           (iq_avg < FOC_ZERO) ? "Iq 为负, 翻转两路极性" : "Iq 为正, 极性正确");

    if (iq_avg < FOC_ZERO) {
        m->current_sense->set_sign_a(m->current_sense->ctx, -1.0f);
        m->current_sense->set_sign_b(m->current_sense->ctx, -1.0f);
        printf("      已把 sign_a / sign_b 都翻成 -1\n");
    }

    s_ctrl_paused = false;
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
}

/*
 * 零点偏移标定（扫一圈找 |Id| 最小）
 *
 * 现象: 不管把 zero_electric_angle 补多少, 命令 q 轴电压时电流总落在 d 轴上。
 * 说明偏差不是能一口说准的固定角, 也不能靠猜。
 *
 * 做法: 施加直流矢量、转子按住不动, 让 zero_electric_angle 从 base 起
 * 逐点 +N 度。每换一个偏移, 同一个电流矢量在 dq 坐标系里就转 N 度。
 * 找到【|Id| 最小 / Iq 最大】的那一点, 就是正确的零点。
 *
 * 注意: 这个标定必须让转子【不动】。转子一动, 电压矢量就跟着编码器转,
 *       测到的就不是纯电阻响应了。
 */
static float offset_sweep(foc_motor_t *m)
{
    printf("\n---- 零点偏移标定 (直流矢量 %.2f V, 转子请按住不动) ----\n",
           (double)APP_OFFSET_V);
    printf("      %-10s %-10s %-10s %-10s %s\n", "偏移(°)", "Id", "Iq", "|I|", "评定");

    s_ctrl_paused = true;
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));

    const float base      = m->zero_electric_angle;
    const float step      = APP_OFFSET_STEP_DEG / RAD2DEG;
    const int   n         = APP_OFFSET_AVG_MS / APP_TELEMETRY_MS;

    float best_off = 0.0f, best_id = 1e9f, best_iq = 0.0f;
    bool  have = false;

    for (int k = 0; k < APP_OFFSET_STEPS; k++) {
        const float off = step * (float)k;
        m->zero_electric_angle = foc_normalize_angle(base + off);

        /* 施加直流矢量（运动方式仍是力矩 -> 电压跟着编码器走；
         * 转子被按住不动，所以场是静止的） */
        m->motion      = FOC_MOTION_TORQUE;
        m->torque_ctrl = FOC_TORQUE_VOLTAGE;
        m->target      = APP_OFFSET_V;
        s_ctrl_paused  = false;
        vTaskDelay(pdMS_TO_TICKS(APP_OFFSET_SETTLE_MS));

        float sd = 0.0f, sq = 0.0f;
        for (int i = 0; i < n; i++) {
            vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_MS));
            sd += m->current.d;
            sq += m->current.q;
        }
        s_ctrl_paused = true;
        m->target = APP_CS_V_ZERO;
        vTaskDelay(pdMS_TO_TICKS(APP_OFFSET_SETTLE_MS));

        const float ad = sd / (float)n;
        const float aq = sq / (float)n;
        const float mag = sqrtf(ad * ad + aq * aq);

        printf("      %+8.0f  %+9.3f  %+9.3f  %+9.3f  %s\n",
               (double)(off * RAD2DEG), (double)ad, (double)aq, (double)mag,
               (fabsf(ad) < 0.05f) ? "Id 接近 0" : "");
        fflush(stdout);

        /* 判据: |Id| 最小（q 轴对齐）；|Id| 相同时取 |Iq| 大的 */
        const float abs_d = fabsf(ad);
        if (!have || abs_d < fabsf(best_id) - 1e-6f
                  || (fabsf(abs_d - fabsf(best_id)) < 1e-6f && fabsf(aq) > fabsf(best_iq))) {
            best_off = off;
            best_id  = ad;
            best_iq  = aq;
            have     = true;
        }
    }

    m->zero_electric_angle = foc_normalize_angle(base + best_off);
    printf("\n      => 最佳偏移 %+.0f°  (判据 |Id| 最小)   (Id=%+.3f  Iq=%+.3f)\n",
           (double)(best_off * RAD2DEG), (double)best_id, (double)best_iq);
    printf("         原始对齐值 %.1f° + 偏移 %+.0f° = %.1f°\n",
           (double)(base * RAD2DEG), (double)(best_off * RAD2DEG),
           (double)(foc_normalize_angle(base + best_off) * RAD2DEG));

    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
    s_ctrl_paused = false;
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
    return best_off;
}
#endif /* APP_RUN_OFFSET_SWEEP */

/*
 * 相电流极性自动判定
 *
 * 思路（对应 SimpleFOC 注释里的 "this should be automated later"）:
 *   施加一个【固定电角度】的直流电压矢量，转子会被磁场吸住并停在该位置。
 *   等它把摆动衰减掉之后，在【同一个电角度】上做 Clarke+Park 取平均：
 *       正确极性  ->  Iq 明显为正（电流方向与电压一致），|Id| 远小于 |Iq|
 *       极性接反  ->  dq 矢量被转错，Iq 会变小/变负，Id 变大
 *   对 4 种 (sign_a, sign_b) 组合各测一次，选 Iq 最大且 Id 最小的那个。
 */
typedef struct {
    float sign_a;
    float sign_b;
    float avg_d;
    float avg_q;
} sign_candidate_t;

#if APP_RUN_SIGN_CAL
static void sign_measure(foc_motor_t *m, float sign_a, float sign_b,
                         float *out_d, float *out_q)
{
    current_sense_set_sign_a(&s_cs, sign_a);
    current_sense_set_sign_b(&s_cs, sign_b);

    /* 施加固定电角度的直流矢量 */
    (void)foc_motor_set_phase_voltage(m, APP_SIGN_CAL_V, 0.0f, APP_SIGN_CAL_ANGLE);
    vTaskDelay(pdMS_TO_TICKS(APP_SIGN_CAL_SETTLE_MS));

    const int n = APP_SIGN_CAL_AVG_MS / APP_TELEMETRY_MS;
    float sum_d = 0.0f, sum_q = 0.0f;
    for (int i = 0; i < n; i++) {
        vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_MS));
        ESP_ERROR_CHECK(current_sense_update(&s_cs));
        const foc_abc_t iabc = {
            current_sense_get_a(&s_cs),
            current_sense_get_b(&s_cs),
            current_sense_get_c(&s_cs),
        };
        const foc_dq_t idq = foc_park(foc_clarke(iabc), APP_SIGN_CAL_ANGLE);
        sum_d += idq.d;
        sum_q += idq.q;
    }
    *out_d = sum_d / (float)n;
    *out_q = sum_q / (float)n;

    /* 松开 */
    (void)foc_motor_set_phase_voltage(m, 0.0f, 0.0f, 0.0f);
    vTaskDelay(pdMS_TO_TICKS(APP_SIGN_CAL_OFF_MS));
}

static void sign_calibration_test(foc_motor_t *m)
{
    printf("\n---- 相电流极性自动判定 (固定电角度 %.0f°, 矢量 %.2f V) ----\n",
           (double)(APP_SIGN_CAL_ANGLE * APP_RAD_TO_DEG), (double)APP_SIGN_CAL_V);
    printf("      %-8s %-8s %-10s %-10s %-10s %s\n",
           "sign_a", "sign_b", "Id", "Iq", "|Id|/|Iq|", "评定");

    s_ctrl_paused = true;
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
    wait_operator("这一段不需要按住，请把手放开，让转子可以自由摆动", APP_CS_PROMPT_MS);

    sign_candidate_t best = { 0.0f, 0.0f, 0.0f, -1e9f };
    bool have_best = false;

    const float signs[2] = { CURRENT_SENSE_SIGN_POS, CURRENT_SENSE_SIGN_NEG };
    for (int ia = 0; ia < 2; ia++) {
        for (int ib = 0; ib < 2; ib++) {
            float d = 0.0f, q = 0.0f;
            sign_measure(m, signs[ia], signs[ib], &d, &q);

            const float ratio = (fabsf(q) > APP_SIGN_CAL_MIN_Q)
                              ? (fabsf(d) / fabsf(q)) : 999.0f;
            const bool ok = (q > APP_SIGN_CAL_MIN_Q) && (ratio < APP_SIGN_CAL_ID_RATIO);

            printf("      %+7.0f  %+7.0f  %+9.3f  %+9.3f  %9.2f  %s\n",
                   (double)signs[ia], (double)signs[ib],
                   (double)d, (double)q, (double)ratio,
                   ok ? "OK" : "");
            fflush(stdout);

            if (!have_best || q > best.avg_q) {
                best.sign_a = signs[ia];
                best.sign_b = signs[ib];
                best.avg_d  = d;
                best.avg_q  = q;
                have_best   = true;
            }
        }
    }

    printf("\n      => 判定结果: sign_a=%+.0f  sign_b=%+.0f   (Id=%+.3f  Iq=%+.3f)\n",
           (double)best.sign_a, (double)best.sign_b,
           (double)best.avg_d, (double)best.avg_q);
    printf("         把它写进 board_pins.h:  BOARD_CS_A_SIGN %+.0f   BOARD_CS_B_SIGN %+.0f\n",
           (double)best.sign_a, (double)best.sign_b);

    /* 采用判定结果 */
    current_sense_set_sign_a(&s_cs, best.sign_a);
    current_sense_set_sign_b(&s_cs, best.sign_b);

    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
    s_ctrl_paused = false;
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));
}
#endif /* APP_RUN_SIGN_CAL */

/* ============================================================================ */

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));

    printf("\n================ 电流采样验证 (SimpleFOC test_current 流程) ================\n");
    printf("母线 %.2f V  相电压上限 %.2f V  电流 LPF Tf=%.4f s\n",
           (double)MOTOR_SUPPLY_V, (double)MOTOR_VOLTAGE_LIMIT, (double)APP_CS_LPF_TF);

    /* ---- 外设 ---- */
    ESP_ERROR_CHECK(i2c_bus_init(&s_i2c, &s_i2c_cfg));
    ESP_ERROR_CHECK(encoder_init(&s_enc, &s_i2c, &s_enc_cfg));
    ESP_ERROR_CHECK(current_sense_init(&s_cs, &s_cs_cfg));
    ESP_ERROR_CHECK(motor_init());
    motor_enable(false);
    ESP_ERROR_CHECK(current_sense_calibrate(&s_cs));

    /* ---- FOC 对象：力矩环=电压，运动=力矩，电流环【不闭合】 ---- */
    foc_motor_cfg_t cfg;
    foc_motor_default_cfg(&cfg);
    cfg.pole_pairs           = MOTOR_POLE_PAIRS;
    cfg.voltage_limit        = MOTOR_VOLTAGE_LIMIT;
    cfg.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE_V;
    cfg.align_zero_offset    = APP_ALIGN_ZERO_OFFSET;
    cfg.sensor_direction     = APP_SENSOR_DIRECTION;
    cfg.current_limit        = MOTOR_CURRENT_LIMIT_A;
    cfg.velocity_limit       = MOTOR_VELOCITY_LIMIT_RADS;
    cfg.modulation           = FOC_MOD_SINE_PWM;
    cfg.modulation_centered  = true;
    cfg.torque_ctrl          = FOC_TORQUE_VOLTAGE;      /* ★ 仍然是电压模式 */
    cfg.motion               = FOC_MOTION_TORQUE;       /* ★ 力矩模式 */
    cfg.pid_cur_limit        = MOTOR_VOLTAGE_LIMIT;
    cfg.pid_cur_ramp         = FOC_NOT_SET;
    cfg.pid_vel_limit        = MOTOR_VOLTAGE_LIMIT;
    cfg.lpf_current_tf       = APP_CS_LPF_TF;           /* ★ 加大电流滤波 */

    foc_hw_sensor_init(&s_sensor_if, &s_enc);
    foc_hw_current_init(&s_current_if, &s_cs);
    ESP_ERROR_CHECK(foc_motor_init(&s_foc, &cfg, foc_hw_driver(), &s_sensor_if, &s_current_if));

    /* ---- 对齐 ---- */
    printf("\n---- 对齐 ----\n");
    motor_enable(true);
    const esp_err_t align_err = foc_motor_align_sensor(&s_foc);
    printf("    对齐: %s  direction=%+.0f  zero_electric_angle=%.4f rad (%.1f°)\n",
           esp_err_to_name(align_err), (double)s_foc.sensor_direction,
           (double)s_foc.zero_electric_angle,
           (double)(s_foc.zero_electric_angle * RAD2DEG));
    if (align_err == ESP_OK) {
        const esp_err_t cs_err = foc_motor_align_current_sense(&s_foc);
        printf("    电流采样对齐: %s\n", esp_err_to_name(cs_err));
        printf("    => 官方判定: swap=%d  sign_a=%+.0f  sign_b=%+.0f  gain_a=%.4f  gain_b=%.4f\n",
               (int)current_sense_get_swap(&s_cs),
               (double)current_sense_get_sign_a(&s_cs),
               (double)current_sense_get_sign_b(&s_cs),
               (double)current_sense_get_gain_a(&s_cs),
               (double)current_sense_get_gain_b(&s_cs));
    }
    if (align_err != ESP_OK) {
        printf("    对齐失败，停止。\n");
        motor_enable(false);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    xTaskCreatePinnedToCore(foc_ctrl_task, "foc", CTRL_TASK_STACK, NULL,
                            CTRL_TASK_PRIO, NULL, CTRL_TASK_CORE);
    vTaskDelay(pdMS_TO_TICKS(APP_CTRL_SETTLE_MS));

    /* ---- 零点偏移标定（需要按住转子） ---- */
    wait_operator("零点是歪的，请用手按住转子别让它动，然后按回车开始标定", APP_CS_PROMPT_MS);
#if APP_RUN_OFFSET_SWEEP
    (void)offset_sweep(&s_foc);
    polarity_fix(&s_foc);
#else
    printf("\n    （APP_RUN_OFFSET_SWEEP=0：不跑零点扫描，直接用官方对齐结果）\n");
#endif

    /* ---- 相电流极性自动判定 ---- */
#if APP_RUN_SIGN_CAL
    sign_calibration_test(&s_foc);
#else
    printf("    （APP_RUN_SIGN_CAL=0：不跑非官方的极性判定，沿用官方 alignCurrentSense 结果）\n");
#endif

    /* ---- A) 基线：使能但不给电压 ---- */
    const float iq_base = stage("A) 基线 (0 V，电机应静止)", APP_CS_V_ZERO,
                                APP_CS_STAGE_MS, NULL);

    /* ---- B) 静测 1V：需要按住转子 ---- */
    const float iq_v1 = stage("B) 静测 1.0 V", APP_CS_V1, APP_CS_STAGE_MS,
                              "用手按住转子别让它转！");

    /* ---- C) 静测 2V：继续按住 ---- */
    const float iq_v2 = stage("C) 静测 2.0 V", APP_CS_V2, APP_CS_STAGE_MS,
                              "继续按住转子！");

    /* ---- D) 静测 -1V：继续按住 ---- */
    const float iq_vn = stage("D) 静测 -1.0 V", APP_CS_VNEG, APP_CS_STAGE_MS,
                              "继续按住转子！");

    /* ---- E) 转测：松手 ---- */
    const float iq_run = stage("E) 转测 1.5 V (松手让它转)", APP_CS_VRUN,
                               APP_CS_RUN_MS, "现在松手，让它转起来");

    /* ---- 判定 ---- */
    printf("\n================ 判定 ================\n");
    printf("  基线 Iq            = %+7.3f A   (应当接近 0)\n", (double)iq_base);
    printf("  静测 +1.0V  -> Iq  = %+7.3f A\n", (double)iq_v1);
    printf("  静测 +2.0V  -> Iq  = %+7.3f A\n", (double)iq_v2);
    printf("  静测 -1.0V  -> Iq  = %+7.3f A\n", (double)iq_vn);

    const float ratio = (fabsf(iq_v1) > APP_CS_RATIO_EPS)
                      ? (fabsf(iq_v2) / fabsf(iq_v1)) : 0.0f;
    printf("\n  电压加倍后电流比 = %.2f   (文档判据: 2 倍电压应约 2 倍电流)\n", (double)ratio);
    printf("  +1V 与 -1V 的符号: %s\n",
           ((iq_v1 > APP_CS_RATIO_EPS && iq_vn < -APP_CS_RATIO_EPS) ||
            (iq_v1 < -APP_CS_RATIO_EPS && iq_vn > APP_CS_RATIO_EPS))
           ? "相反 ✓ (极性正确)" : "没有明显相反 ✗");
    printf("  转测 Iq            = %+7.3f A   (应随转速上升而下降)\n", (double)iq_run);
    printf("\n  按 |Z|=%.2fΩ 估算: 1V 应对应 Iq≈%.2fA\n",
           (double)APP_CS_Z_ESTIMATE, (double)(APP_CS_V1 / APP_CS_Z_ESTIMATE));
    printf("======================================\n");

    printf("\n收尾...\n");
    s_foc.target = APP_CS_V_ZERO;
    vTaskDelay(pdMS_TO_TICKS(APP_STOP_SETTLE_MS));
    motor_enable(false);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
