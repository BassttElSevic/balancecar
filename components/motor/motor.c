/*
 * ============================================================================
 *  balancecar — 三相无刷电机驱动 (MCPWM)
 *
 *  外设结构:
 *      1 个 MCPWM 定时器 (中心对齐 UP_DOWN)
 *        └── 3 个 operator 共享同一个定时器 (保证三相严格同频同相)
 *              └── 每个 operator: 1 个 comparator + 1 个 generator -> 1 个 GPIO
 *
 *  占空比换算 (中心对齐模式):
 *      计数器 0 -> peak -> 0，一个完整周期 = 2*peak tick
 *      动作: 上行到比较值 -> 拉低;  下行到比较值 -> 拉高
 *      因此输出高电平宽度 = 2*compare tick
 *      => duty = 2*compare / (2*peak) = compare / peak
 *      => compare = duty * peak
 * ============================================================================
 */

#include <math.h>

#include "esp_log.h"
#include "esp_check.h"
#include "driver/mcpwm_prelude.h"

#include "board_pins.h"
#include "motor.h"

static const char *TAG = "motor";

/* MOTOR_PWM_PERIOD_TICKS / MOTOR_PWM_PEAK_TICKS 定义在 board_pins.h */
typedef struct {
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper[3];
    mcpwm_cmpr_handle_t  cmpr[3];
    mcpwm_gen_handle_t   gen[3];
    bool                 ready;
} motor_ctx_t;

static motor_ctx_t s_motor;

/* 当前生效的三相引脚（扫描时会变，所以不能是 const） */
static int               s_phase_gpio[3] = { MOTOR_PHASE_A_GPIO, MOTOR_PHASE_B_GPIO, MOTOR_PHASE_C_GPIO };
static const char *const s_phase_name[3] = { "U(A)", "V(B)", "W(C)" };

/* -------------------------------------------------------------------------- */
/*                              使能脚                                        */
/* -------------------------------------------------------------------------- */

static esp_err_t motor_en_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << MOTOR_EN_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "EN 引脚 (GPIO%d) 配置失败", MOTOR_EN_GPIO);

    /* 上电先失能，避免驱动芯片意外输出 */
    gpio_set_level(MOTOR_EN_GPIO, MOTOR_EN_ACTIVE_LEVEL ? 0 : 1);
    return ESP_OK;
}

void motor_enable(bool en)
{
    gpio_set_level(MOTOR_EN_GPIO, en ? (MOTOR_EN_ACTIVE_LEVEL ? 1 : 0)
                                     : (MOTOR_EN_ACTIVE_LEVEL ? 0 : 1));
    ESP_LOGI(TAG, "驱动器 %s", en ? "使能" : "失能");
}

/* -------------------------------------------------------------------------- */
/*                              初始化                                        */
/* -------------------------------------------------------------------------- */

esp_err_t motor_init(void)
{
    ESP_RETURN_ON_FALSE(!s_motor.ready, ESP_ERR_INVALID_STATE, TAG, "motor 已经初始化过了");

    ESP_LOGI(TAG, "三相引脚: U=GPIO%d  V=GPIO%d  W=GPIO%d  EN=GPIO%d",
             MOTOR_PHASE_A_GPIO, MOTOR_PHASE_B_GPIO, MOTOR_PHASE_C_GPIO, MOTOR_EN_GPIO);

    ESP_RETURN_ON_ERROR(motor_en_gpio_init(), TAG, "使能脚初始化失败");

    /* ---------------- 1) 定时器 ---------------- */
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,   /* P4 上即 PLL_F160M */
        .resolution_hz = MOTOR_PWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks  = MOTOR_PWM_PERIOD_TICKS,
        .flags.update_period_on_empty = true,
    };
    ESP_RETURN_ON_ERROR(mcpwm_new_timer(&timer_cfg, &s_motor.timer), TAG, "创建 MCPWM 定时器失败");

    /* ---------------- 2) 三个 operator 共用该定时器 ---------------- */
    mcpwm_operator_config_t    oper_cfg  = { .group_id = 0 };
    mcpwm_comparator_config_t  cmpr_cfg  = { .flags.update_cmp_on_tez = true };
    mcpwm_generator_config_t   gen_cfg   = { .gen_gpio_num = -1 };

    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(mcpwm_new_operator(&oper_cfg, &s_motor.oper[i]),
                            TAG, "创建 operator %d 失败", i);
        ESP_RETURN_ON_ERROR(mcpwm_operator_connect_timer(s_motor.oper[i], s_motor.timer),
                            TAG, "operator %d 连接定时器失败", i);

        ESP_RETURN_ON_ERROR(mcpwm_new_comparator(s_motor.oper[i], &cmpr_cfg, &s_motor.cmpr[i]),
                            TAG, "创建 comparator %d 失败", i);

        gen_cfg.gen_gpio_num = s_phase_gpio[i];
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_motor.oper[i], &gen_cfg, &s_motor.gen[i]),
                            TAG, "创建 generator %d (GPIO%d) 失败", i, s_phase_gpio[i]);

        /* 中心对齐 PWM: 上行过比较值拉低, 下行过比较值拉高 */
        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(s_motor.gen[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_motor.cmpr[i], MCPWM_GEN_ACTION_LOW)),
            TAG, "%s 上行比较动作设置失败", s_phase_name[i]);

        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(s_motor.gen[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, s_motor.cmpr[i], MCPWM_GEN_ACTION_HIGH)),
            TAG, "%s 下行比较动作设置失败", s_phase_name[i]);
    }

    /* ---------------- 3) 先输出零矢量, 再启动定时器 ---------------- */
    /* 先置就绪标志，否则 motor_zero() 会因为"未初始化"而拒绝执行 */
    s_motor.ready = true;

    ESP_RETURN_ON_ERROR(motor_zero(), TAG, "零矢量设置失败");

    ESP_RETURN_ON_ERROR(mcpwm_timer_enable(s_motor.timer), TAG, "MCPWM 定时器使能失败");
    ESP_RETURN_ON_ERROR(mcpwm_timer_start_stop(s_motor.timer, MCPWM_TIMER_START_NO_STOP),
                        TAG, "MCPWM 定时器启动失败");

    ESP_LOGI(TAG, "初始化完成: %d kHz 中心对齐, 分辨率 %d 级 (peak=%d ticks)",
             MOTOR_PWM_FREQ_HZ / 1000, MOTOR_PWM_PEAK_TICKS, MOTOR_PWM_PEAK_TICKS);
    ESP_LOGI(TAG, "注意: 驱动器当前处于【失能】状态, 需调用 motor_enable(true)");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                          运行时切换三相引脚                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief 把三相 PWM 输出切换到另一组 GPIO
 *
 * 实现方式：删除原 generator（会释放 GPIO 占用），再在同一 operator 上重建。
 * 用于排查"到底哪组引脚才是真正的三相输出"。
 */
esp_err_t motor_set_phase_pins(int gpio_a, int gpio_b, int gpio_c)
{
    ESP_RETURN_ON_FALSE(s_motor.ready, ESP_ERR_INVALID_STATE, TAG, "motor 尚未初始化");

    const int g[3] = { gpio_a, gpio_b, gpio_c };
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = -1 };

    for (int i = 0; i < 3; i++) {
        ESP_RETURN_ON_ERROR(mcpwm_del_generator(s_motor.gen[i]),
                            TAG, "删除 generator %d 失败", i);

        gen_cfg.gen_gpio_num = g[i];
        ESP_RETURN_ON_ERROR(mcpwm_new_generator(s_motor.oper[i], &gen_cfg, &s_motor.gen[i]),
                            TAG, "在 GPIO%d 上重建 generator %d 失败", g[i], i);

        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(s_motor.gen[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_motor.cmpr[i], MCPWM_GEN_ACTION_LOW)),
            TAG, "%s 上行比较动作设置失败", s_phase_name[i]);

        ESP_RETURN_ON_ERROR(
            mcpwm_generator_set_action_on_compare_event(s_motor.gen[i],
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, s_motor.cmpr[i], MCPWM_GEN_ACTION_HIGH)),
            TAG, "%s 下行比较动作设置失败", s_phase_name[i]);

        s_phase_gpio[i] = g[i];
    }

    ESP_LOGI(TAG, "三相引脚已切换为 U=GPIO%d  V=GPIO%d  W=GPIO%d", gpio_a, gpio_b, gpio_c);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                              占空比控制                                    */
/* -------------------------------------------------------------------------- */

esp_err_t motor_set_duty(float du, float dv, float dw)
{
    ESP_RETURN_ON_FALSE(s_motor.ready, ESP_ERR_INVALID_STATE, TAG, "motor 尚未初始化");

    float duty[3] = { du, dv, dw };
    for (int i = 0; i < 3; i++) {
        if (!isfinite(duty[i])) {
            return ESP_ERR_INVALID_ARG;
        }
        /* 限幅到 0.0 ~ 1.0 */
        if (duty[i] < MOTOR_DUTY_MIN) duty[i] = MOTOR_DUTY_MIN;
        if (duty[i] > MOTOR_DUTY_MAX) duty[i] = MOTOR_DUTY_MAX;

        uint32_t cmp = (uint32_t)lrintf(duty[i] * (float)MOTOR_PWM_PEAK_TICKS);
        ESP_RETURN_ON_ERROR(mcpwm_comparator_set_compare_value(s_motor.cmpr[i], cmp),
                            TAG, "设置 %s 占空比失败", s_phase_name[i]);
    }
    return ESP_OK;
}

esp_err_t motor_set_uvw(float u, float v, float w)
{
    /* 归一化相电压 -1~+1 映射到占空比，0V 对应中点 */
    return motor_set_duty(MOTOR_DUTY_MIDPOINT + MOTOR_DUTY_GAIN * u,
                          MOTOR_DUTY_MIDPOINT + MOTOR_DUTY_GAIN * v,
                          MOTOR_DUTY_MIDPOINT + MOTOR_DUTY_GAIN * w);
}

esp_err_t motor_zero(void)
{
    return motor_set_duty(MOTOR_DUTY_MIDPOINT, MOTOR_DUTY_MIDPOINT, MOTOR_DUTY_MIDPOINT);
}

/* -------------------------------------------------------------------------- */
/*                              安全停机                                      */
/* -------------------------------------------------------------------------- */

void motor_coast(void)
{
    motor_enable(false);
    ESP_LOGI(TAG, "电机自由滑行");
}
