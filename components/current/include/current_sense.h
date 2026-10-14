/*
 * ============================================================================
 *  current_sense —— 双电阻在线电流采样 (INA240 x2 -> ADC1)
 *
 *  A 相 / B 相各一路 INA240 输出接到 ADC1 的两个通道，第三相由基尔霍夫定律得出:
 *      Ic = -Ia - Ib
 *
 *  ADC 使用方式（这几条是使读数正确的关键，不要改）:
 *    * adc_oneshot_unit_init_cfg_t 用 ADC_UNIT_1 / ADC_RTC_CLK_SRC_DEFAULT /
 *      ADC_ULP_MODE_DISABLE
 *    * 每个通道各建一个 curve-fitting 校准句柄，并显式填 .chan
 *      （P4 上 CONFIG_SOC_ADC_CALIB_CHAN_COMPENS_SUPPORTED=y，校准是按通道补偿的）
 *    * 读的时候 A、B 顺序读；顺序读之间的时间差远小于电周期，可忽略
 *
 *  存储由调用者提供（不使用堆）:
 *      static current_sense_t s_cs;
 *      current_sense_init(&s_cs, &cfg);
 *      current_sense_calibrate(&s_cs);     // 必须在电机静止、无电流时调用
 *      current_sense_update(&s_cs);
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

/* -------------------------------------------------------------------------- */
/*  默认值                                                                     */
/* -------------------------------------------------------------------------- */

#define CURRENT_SENSE_CAL_SAMPLES_DEFAULT   64   /**< 零点校准采样次数 */
#define CURRENT_SENSE_CAL_DELAY_MS_DEFAULT  1    /**< 每次采样间隔 (ms) */

/* 每相极性允许的取值 */
#define CURRENT_SENSE_SIGN_POS              1.0f
#define CURRENT_SENSE_SIGN_NEG             (-1.0f)

/* -------------------------------------------------------------------------- */
/*  配置                                                                       */
/* -------------------------------------------------------------------------- */

/** @brief 电流采样配置（各项建议直接取 board_pins.h 的宏） */
typedef struct {
    adc_unit_t     unit;         /**< BOARD_CS_ADC_UNIT */
    adc_channel_t  chan_a;       /**< BOARD_CS_A_CHANNEL */
    adc_channel_t  chan_b;       /**< BOARD_CS_B_CHANNEL */
    adc_atten_t    atten;        /**< BOARD_CS_ATTEN */
    adc_bitwidth_t bitwidth;     /**< BOARD_CS_BITWIDTH */
    int            gpio_a;       /**< BOARD_CS_A_GPIO，只用于日志 */
    int            gpio_b;       /**< BOARD_CS_B_GPIO，只用于日志 */
    float          shunt_ohms;   /**< BOARD_CS_SHUNT_OHMS */
    float          amp_gain;     /**< BOARD_CS_AMP_GAIN */
    float          sign_a;       /**< A 相极性 +1/-1；0 -> +1 */
    float          sign_b;       /**< B 相极性 +1/-1；0 -> +1 */
    int            cal_samples;  /**< 0 -> 64 */
    int            cal_delay_ms; /**< 0 -> 1 */
} current_sense_cfg_t;

/* -------------------------------------------------------------------------- */
/*  句柄（成员内部使用）                                                        */
/* -------------------------------------------------------------------------- */

/** @brief 电流采样句柄（成员内部使用） */
typedef struct {
    adc_oneshot_unit_handle_t unit_h;      /**< 内部 */
    adc_cali_handle_t         cali_a;      /**< A 相专用校准句柄 */
    adc_cali_handle_t         cali_b;      /**< B 相专用校准句柄 */

    adc_channel_t  chan_a;                 /**< A 相通道 */
    adc_channel_t  chan_b;                 /**< B 相通道 */
    int            gpio_a;                 /**< A 相引脚（日志用） */
    int            gpio_b;                 /**< B 相引脚（日志用） */

    bool           swap_channels;          /**< 交换前两相（对齐用）*/
    bool           ready;                  /**< 内部 */
    bool           calibrated;             /**< 是否已做过零点校准 */

    float          v_per_a;                /**< 灵敏度 (V/A)，正数 */
    float          sign_a;                 /**< A 相极性 +1/-1 */
    float          sign_b;                 /**< B 相极性 +1/-1 */
    float          gain_a;                 /**< 通道 A 增益 (A/V)，带符号（硬件层，来自 board_pins）*/
    float          gain_b;                 /**< 通道 B 增益 (A/V)，带符号（硬件层，来自 board_pins）*/
    float          phase_sign_a;           /**< 相 A 符号 (+1/-1)，作用在 ia 上，与 swap 无关 */
    float          phase_sign_b;           /**< 相 B 符号 (+1/-1)，作用在 ib 上，与 swap 无关 */
    float          off_a_mv;               /**< A 相零点 (mV) */
    float          off_b_mv;               /**< B 相零点 (mV) */
    int            cal_samples;            /**< 校准采样次数 */

    /* 最近一次读数 */
    int            raw_a, raw_b;           /**< 12bit 原始值 */
    int            mv_a,  mv_b;            /**< 电压 (mV) */
    float          ia, ib, ic;             /**< 三相电流 (A) */
    uint32_t       err_count;              /**< 累计读失败次数 */
} current_sense_t;

/* -------------------------------------------------------------------------- */
/*  生命周期                                                                   */
/* -------------------------------------------------------------------------- */

/** @brief 初始化（配置 ADC 单元、两个通道、两个校准句柄） */
esp_err_t current_sense_init(current_sense_t *cs, const current_sense_cfg_t *cfg);

/** @brief 反初始化 */
esp_err_t current_sense_deinit(current_sense_t *cs);

/* -------------------------------------------------------------------------- */
/*  使用                                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief 零点校准
 * @note 必须在【电机不转、无电流】时调用
 */
esp_err_t current_sense_calibrate(current_sense_t *cs);

/** @brief 读一次两相电流并算出第三相 */
esp_err_t current_sense_update(current_sense_t *cs);

/* -------------------------------------------------------------------------- */
/*  读取                                                                       */
/* -------------------------------------------------------------------------- */

float current_sense_get_a(const current_sense_t *cs);
float current_sense_get_b(const current_sense_t *cs);
float current_sense_get_c(const current_sense_t *cs);

int   current_sense_get_raw_a(const current_sense_t *cs);
int   current_sense_get_raw_b(const current_sense_t *cs);
int   current_sense_get_mv_a(const current_sense_t *cs);
int   current_sense_get_mv_b(const current_sense_t *cs);

float current_sense_get_offset_mv_a(const current_sense_t *cs);
float current_sense_get_offset_mv_b(const current_sense_t *cs);

/**
 * @brief 设置某一相的极性 (+1/-1)
 * @note 与 SimpleFOC 的 gain_a/gain_b 同义：相电流接反时把它翻过来。
 *       改完立即生效（会重算 gain）。
 */
/** @brief 交换前两相（SimpleFOC 对齐时若发现通道与相不匹配会交换） */
void   current_sense_set_swap(current_sense_t *cs, bool swap);
bool   current_sense_get_swap(const current_sense_t *cs);

/**
 * @brief 设置相 A/B 的符号 (+1/-1)
 * @note  这是**相级**的：无论 swap 与否都作用在 ia / ib 上。
 *        与通道增益 gain_a/gain_b 分开，避免 swap 之后翻错通道。
 */
void current_sense_set_sign_a(current_sense_t *cs, float sign);
void current_sense_set_sign_b(current_sense_t *cs, float sign);

/** @brief 读回当前每相增益 (A/V，带符号) */
float current_sense_get_gain_a(const current_sense_t *cs);
float current_sense_get_gain_b(const current_sense_t *cs);
float current_sense_get_sign_a(const current_sense_t *cs);
float current_sense_get_sign_b(const current_sense_t *cs);

bool     current_sense_is_calibrated(const current_sense_t *cs);
uint32_t current_sense_get_err_count(const current_sense_t *cs);

/** @brief 打印一行自检信息 */
void current_sense_print_diag(const current_sense_t *cs);
