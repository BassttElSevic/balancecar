/*
 * ============================================================================
 *  encoder —— AS5600 实现
 *
 *  I2C 走 components/i2c_bus；读角度/多圈/速度的逻辑逐行对齐 DengFOC。
 * ============================================================================
 */

#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "encoder.h"

static const char *TAG = "encoder";

#define TWO_PI_F            6.28318530718f
#define RAD_TO_DEG          (180.0f / (float)M_PI)

/* DengFOC getSensorAngle() 里的位定义:
 *   _bit_resolution = 12
 *   _bits_used_msb  = 11 - 7 = 4
 *   lsb_used        = 12 - 4 = 8
 *   lsb_mask        = (2 << 8) - 1  -> 强转 uint8_t 后为 0xFF
 *   msb_mask        = (2 << 4) - 1  -> 0x1F
 */
#define AS5600_BITS_MSB     (11 - 7)
#define AS5600_LSB_USED     (AS5600_ANGLE_BITS - AS5600_BITS_MSB)

/* DengFOC 的过零判据：单次角度跳变超过 0.8 圈认为跨过了 0/2π */
#define AS5600_WRAP_THRESHOLD   (0.8f * TWO_PI_F)

#define ENCODER_DEFAULT_SETTLE_MS   500u

/* -------------------------------------------------------------------------- */
/*  默认值                                                                     */
/* -------------------------------------------------------------------------- */

static void cfg_apply_defaults(encoder_cfg_t *c)
{
    if (c->settle_ms      == 0) c->settle_ms      = ENCODER_SETTLE_MS_DEFAULT;
    if (c->probe_retry    == 0) c->probe_retry    = ENCODER_PROBE_RETRY_DEFAULT;
    if (c->probe_gap_ms   == 0) c->probe_gap_ms   = ENCODER_PROBE_GAP_MS_DEFAULT;
    if (c->counts_per_rev == 0) c->counts_per_rev = (float)AS5600_COUNTS_PER_REV;
    if (c->direction      == 0) c->direction      = 1.0f;
}

/* -------------------------------------------------------------------------- */
/*  读角度（DengFOC getSensorAngle 的等价实现）                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief 读 0x0C 两字节，按 DengFOC 的掩码拼出 12bit，换算成 0 ~ 2π
 */
static esp_err_t as5600_read_sensor_angle(encoder_t *enc, uint16_t *out_raw,
                                          float *out_rad)
{
    uint8_t buf[2] = {0};
    ESP_RETURN_ON_ERROR(i2c_slave_read_reg(&enc->slave, AS5600_REG_RAW_ANGLE, buf, 2),
                        TAG, "读 0x%02X 失败", AS5600_REG_RAW_ANGLE);

    /* 与 DengFOC 逐字一致 */
    const uint8_t lsb_mask = (uint8_t)((2u << AS5600_LSB_USED) - 1u);
    const uint8_t msb_mask = (uint8_t)((2u << AS5600_BITS_MSB) - 1u);

    uint16_t read_value  = (uint16_t)(buf[1] & lsb_mask);
    read_value += (uint16_t)((buf[0] & msb_mask) << AS5600_LSB_USED);

    *out_raw = read_value & AS5600_ANGLE_MASK;
    *out_rad = ((float)read_value / enc->counts_per_rev) * TWO_PI_F * enc->direction;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  生命周期                                                                   */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_init(encoder_t *enc, i2c_bus_t *bus, const encoder_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(enc && bus && cfg, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    ESP_RETURN_ON_FALSE(!enc->ready, ESP_ERR_INVALID_STATE, TAG, "编码器已初始化");
    ESP_RETURN_ON_FALSE(cfg->addr7 != 0, ESP_ERR_INVALID_ARG, TAG, "从机地址不能为 0");

    encoder_cfg_t c = *cfg;
    cfg_apply_defaults(&c);

    /* ---- 1) 挂载从机 ---- */
    ESP_RETURN_ON_ERROR(i2c_slave_init(&enc->slave, bus, c.addr7, c.freq_hz),
                        TAG, "挂载 AS5600 (0x%02X) 失败", c.addr7);

    /* ---- 2) DengFOC: Wire.begin() 之后 delay(500) ---- */
    vTaskDelay(pdMS_TO_TICKS(c.settle_ms));

    /* ---- 3) 探测（带重试） ---- */
    const esp_err_t probe = i2c_bus_probe_retry(bus, c.addr7, c.probe_retry, c.probe_gap_ms);
    if (probe != ESP_OK) {
        (void)i2c_slave_deinit(&enc->slave);
        return probe;
    }

    enc->counts_per_rev = c.counts_per_rev;
    enc->direction      = c.direction;
    enc->ready          = true;
    enc->err_count      = 0;

    /* ---- 4) DengFOC Sensor_init() 的连读序列 ----
     *     getSensorAngle();  delayMicroseconds(1);
     *     vel_angle_prev = getSensorAngle();  vel_angle_prev_ts = micros();
     *     delay(1);
     *     getSensorAngle();  delayMicroseconds(1);
     *     angle_prev = getSensorAngle();      angle_prev_ts = micros();
     */
    uint16_t raw = 0;
    float    a   = 0.0f;

    (void)as5600_read_sensor_angle(enc, &raw, &a);
    esp_rom_delay_us(1);

    (void)as5600_read_sensor_angle(enc, &raw, &a);
    enc->vel_angle_prev     = a;
    enc->vel_angle_prev_ts  = esp_timer_get_time();
    enc->vel_full_rotations = 0;

    vTaskDelay(pdMS_TO_TICKS(1));
    (void)as5600_read_sensor_angle(enc, &raw, &a);
    esp_rom_delay_us(1);

    (void)as5600_read_sensor_angle(enc, &raw, &a);
    enc->angle_prev     = a;
    enc->angle_prev_ts  = esp_timer_get_time();
    enc->full_rotations = 0;

    enc->raw_angle = raw;
    enc->ok        = true;

    (void)encoder_refresh_diag(enc);

    ESP_LOGI(TAG, "AS5600 就绪: 0x%02X  0x0C=%u  初始单圈角 %.2f°  STATUS=0x%02X",
             c.addr7, enc->raw_angle,
             (double)(enc->angle_prev * RAD_TO_DEG), enc->status);
    return ESP_OK;
}

esp_err_t encoder_deinit(encoder_t *enc)
{
    ESP_RETURN_ON_FALSE(enc, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    const esp_err_t err = i2c_slave_deinit(&enc->slave);
    enc->ready = false;
    enc->ok    = false;
    return err;
}

/* -------------------------------------------------------------------------- */
/*  更新（DengFOC Sensor_update）                                              */
/* -------------------------------------------------------------------------- */

float encoder_update(encoder_t *enc)
{
    ESP_RETURN_ON_FALSE(enc && enc->ready, enc ? enc->angle_prev : 0.0f,
                        TAG, "编码器未初始化");

    uint16_t raw = 0;
    float    val = 0.0f;
    if (as5600_read_sensor_angle(enc, &raw, &val) != ESP_OK) {
        enc->err_count++;
        enc->ok = false;
        return enc->angle_prev;
    }

    /* ---- DengFOC Sensor_update() ---- */
    enc->angle_prev_ts = esp_timer_get_time();

    const float d_angle = val - enc->angle_prev;
    if (fabsf(d_angle) > AS5600_WRAP_THRESHOLD) {
        enc->full_rotations += (d_angle > 0.0f) ? -1 : 1;
    }
    enc->angle_prev = val;
    enc->raw_angle  = raw;
    enc->ok         = true;
    return enc->angle_prev;
}

/* -------------------------------------------------------------------------- */
/*  导出                                                                       */
/* -------------------------------------------------------------------------- */

float encoder_get_single_turn(const encoder_t *enc)
{
    return enc ? enc->angle_prev : 0.0f;
}

float encoder_get_angle(const encoder_t *enc)
{
    return enc ? ((float)enc->full_rotations * TWO_PI_F + enc->angle_prev) : 0.0f;
}

float encoder_get_velocity(encoder_t *enc)
{
    ESP_RETURN_ON_FALSE(enc && enc->ready, 0.0f, TAG, "编码器未初始化");

    float ts = (float)(enc->angle_prev_ts - enc->vel_angle_prev_ts) * 1e-6f;
    if (ts <= 0.0f) {
        ts = 1e-3f;
    }
    const float vel = ((float)(enc->full_rotations - enc->vel_full_rotations) * TWO_PI_F
                       + (enc->angle_prev - enc->vel_angle_prev)) / ts;

    enc->vel_angle_prev     = enc->angle_prev;
    enc->vel_full_rotations = enc->full_rotations;
    enc->vel_angle_prev_ts  = enc->angle_prev_ts;
    enc->velocity           = vel;
    return vel;
}

int32_t encoder_get_full_rotations(const encoder_t *enc)
{
    return enc ? enc->full_rotations : 0;
}

bool encoder_is_ok(const encoder_t *enc)
{
    return enc ? enc->ok : false;
}

uint16_t encoder_get_raw_angle(const encoder_t *enc) { return enc ? enc->raw_angle : 0; }
uint16_t encoder_get_angle_reg(const encoder_t *enc) { return enc ? enc->angle_reg : 0; }
uint8_t  encoder_get_status(const encoder_t *enc)    { return enc ? enc->status    : 0; }
uint8_t  encoder_get_agc(const encoder_t *enc)       { return enc ? enc->agc       : 0; }
uint16_t encoder_get_magnitude(const encoder_t *enc) { return enc ? enc->magnitude : 0; }
uint32_t encoder_get_err_count(const encoder_t *enc) { return enc ? enc->err_count : 0; }

void encoder_set_zero(encoder_t *enc)
{
    if (!enc) {
        return;
    }
    enc->full_rotations = 0;
    enc->angle_prev     = 0.0f;
    ESP_LOGI(TAG, "多圈角已清零");
}

/* -------------------------------------------------------------------------- */
/*  诊断                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_refresh_diag(encoder_t *enc)
{
    ESP_RETURN_ON_FALSE(enc && enc->ready, ESP_ERR_INVALID_STATE, TAG, "编码器未初始化");

    uint16_t v16 = 0;
    uint8_t  v8  = 0;

    if (i2c_slave_read_u16_be(&enc->slave, AS5600_REG_ANGLE, &v16) == ESP_OK) {
        enc->angle_reg = v16 & AS5600_ANGLE_MASK;
    }
    if (i2c_slave_read_u8(&enc->slave, AS5600_REG_STATUS, &v8) == ESP_OK) {
        enc->status = v8;
    }
    if (i2c_slave_read_u8(&enc->slave, AS5600_REG_AGC, &v8) == ESP_OK) {
        enc->agc = v8;
    }
    if (i2c_slave_read_u16_be(&enc->slave, AS5600_REG_MAGNITUDE, &v16) == ESP_OK) {
        enc->magnitude = v16;
    }
    return ESP_OK;
}

void encoder_print_diag(const encoder_t *enc)
{
    if (!enc) {
        return;
    }
    const uint8_t st = enc->status;
    const int md = (st >> AS5600_STATUS_MD_SHIFT) & 1;
    const int ml = (st >> AS5600_STATUS_ML_SHIFT) & 1;
    const int mh = (st >> AS5600_STATUS_MH_SHIFT) & 1;

    printf("[编码器] AS5600 @0x%02X  SDA=GPIO%d SCL=GPIO%d\n",
           (int)enc->slave.addr7, i2c_bus_get_sda(enc->slave.bus),
           i2c_bus_get_scl(enc->slave.bus));
    printf("   STATUS=0x%02X  MD=%d ML=%d MH=%d  -> %s\n", st, md, ml, mh,
           !md ? "未检测到磁铁" : (ml ? "磁场太弱" : (mh ? "磁场太强" : "磁铁正常")));
    printf("   0x0C=%u (%.2f°)   0x0E=%u   AGC=%u   MAG=%u\n",
           enc->raw_angle, (double)(enc->raw_angle * 360.0f / (float)AS5600_COUNTS_PER_REV),
           enc->angle_reg, enc->agc, enc->magnitude);
    printf("   单圈角 %.4f rad   多圈 %ld 圈   角速度 %.4f rad/s   读失败 %lu 次\n",
           (double)enc->angle_prev, (long)enc->full_rotations,
           (double)enc->velocity, (unsigned long)enc->err_count);
}

static void dump_u16(const encoder_t *enc, const char *name, uint8_t reg)
{
    uint16_t v = 0;
    if (i2c_slave_read_u16_be(&enc->slave, reg, &v) != ESP_OK) {
        printf("   %-11s (0x%02X)  读取失败\n", name, reg);
        return;
    }
    printf("   %-11s (0x%02X)  = 0x%04X  (%u)\n", name, reg, v, v);
}

static void dump_u8(const encoder_t *enc, const char *name, uint8_t reg, bool is_status)
{
    uint8_t v = 0;
    if (i2c_slave_read_u8(&enc->slave, reg, &v) != ESP_OK) {
        printf("   %-11s (0x%02X)  读取失败\n", name, reg);
        return;
    }
    if (is_status) {
        printf("   %-11s (0x%02X)  = 0x%02X  MD=%d ML=%d MH=%d\n", name, reg, v,
               (v >> AS5600_STATUS_MD_SHIFT) & 1,
               (v >> AS5600_STATUS_ML_SHIFT) & 1,
               (v >> AS5600_STATUS_MH_SHIFT) & 1);
    } else {
        printf("   %-11s (0x%02X)  = 0x%02X  (%u)\n", name, reg, v, v);
    }
}

void encoder_dump_regs(const encoder_t *enc)
{
    if (!enc || !enc->ready) {
        return;
    }
    printf("\n===== AS5600 寄存器 dump @0x%02X =====\n", (int)enc->slave.addr7);
    dump_u8 (enc, "ZMCO",        AS5600_REG_ZMCO,       false);
    dump_u16(enc, "ZPOS",        AS5600_REG_ZPOS);
    dump_u16(enc, "MPOS",        AS5600_REG_MPOS);
    dump_u16(enc, "MANG",        AS5600_REG_MANG);
    dump_u16(enc, "CONF",        AS5600_REG_CONF);
    dump_u8 (enc, "STATUS",      AS5600_REG_STATUS,     true);
    dump_u16(enc, "RAW_ANGLE",   AS5600_REG_RAW_ANGLE);
    dump_u16(enc, "ANGLE",       AS5600_REG_ANGLE);
    dump_u8 (enc, "AGC",         AS5600_REG_AGC,        false);
    dump_u16(enc, "MAGNITUDE",   AS5600_REG_MAGNITUDE);
    dump_u8 (enc, "BURN",        AS5600_REG_BURN,       false);
    printf("====================================\n\n");
    fflush(stdout);
}
