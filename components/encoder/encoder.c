/*
 * ============================================================================
 *  balancecar — AS5600 磁编码器驱动 (I2C)
 * ============================================================================
 */

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "board_pins.h"
#include "encoder.h"

static const char *TAG = "encoder";

/* AS5600 寄存器 */
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_ANGLE    0x0E    /* 12bit 角度（经零点/方向编程），高字节在前 */
#define AS5600_REG_RAW      0x0C    /* 12bit 磁场原始角度（不受编程影响） */
#define AS5600_REG_AGC      0x1A    /* 自动增益 */
#define AS5600_REG_MAG      0x1B    /* 磁场幅值 (2 字节) */

/* 速度低通滤波时间常数 (秒) */
#define ENCODER_VEL_TAU     0.01f

/* 每圈计数 */
#define AS5600_COUNTS       4096.0f
/* 计数 -> 弧度 */
#define AS5600_RAD_PER_CNT  (2.0f * (float)M_PI / AS5600_COUNTS)

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    bool     ready;
    bool     ok;            /* 最近一次通信是否成功 */
    uint16_t raw;           /* 12bit 原始值 */
    uint16_t last_raw;
    uint16_t raw_angle;     /* 0x0C 磁场原始角度 */
    uint8_t  agc;           /* 0x1A 自动增益 */
    uint16_t magnitude;     /* 0x1B 磁场幅值 */
    float    angle;         /* 多圈累计机械角 (rad) */
    float    single_turn;   /* 单圈机械角 0~2π */
    float    velocity;      /* 低通后的机械角速度 (rad/s) */
    int64_t  last_us;
    uint32_t err_count;
} encoder_ctx_t;

static encoder_ctx_t s_enc;

/* -------------------------------------------------------------------------- */
/*                              I2C 底层读写                                  */
/* -------------------------------------------------------------------------- */

static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_enc.dev, &reg, 1, buf, len, 100);
}

static esp_err_t as5600_read_angle(uint16_t *out)
{
    uint8_t buf[2] = {0};
    ESP_RETURN_ON_ERROR(as5600_read_reg(AS5600_REG_ANGLE, buf, 2), TAG, "读角度寄存器失败");
    *out = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    return ESP_OK;
}

/** 读 0x0C 磁场原始角度 */
static esp_err_t as5600_read_raw_angle(uint16_t *out)
{
    uint8_t buf[2] = {0};
    ESP_RETURN_ON_ERROR(as5600_read_reg(AS5600_REG_RAW, buf, 2), TAG, "读原始角度失败");
    *out = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    return ESP_OK;
}

/** 刷新 AGC 与磁场幅值 */
static void as5600_refresh_field(void)
{
    uint8_t agc = 0;
    if (as5600_read_reg(AS5600_REG_AGC, &agc, 1) == ESP_OK) {
        s_enc.agc = agc;
    }
    uint8_t mag[2] = {0};
    if (as5600_read_reg(AS5600_REG_MAG, mag, 2) == ESP_OK) {
        s_enc.magnitude = (uint16_t)(((uint16_t)mag[0] << 8) | mag[1]);
    }
}

/* -------------------------------------------------------------------------- */
/*                                 初始化                                     */
/* -------------------------------------------------------------------------- */

esp_err_t encoder_init(void)
{
    return encoder_init_pins(BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
}

esp_err_t encoder_deinit(void)
{
    if (s_enc.dev) {
        i2c_master_bus_rm_device(s_enc.dev);
        s_enc.dev = NULL;
    }
    if (s_enc.bus) {
        i2c_del_master_bus(s_enc.bus);
        s_enc.bus = NULL;
    }
    s_enc.ready = false;
    s_enc.ok    = false;
    return ESP_OK;
}

esp_err_t encoder_init_pins(int sda_gpio, int scl_gpio)
{
    ESP_LOGI(TAG, "初始化 AS5600: SDA=GPIO%d  SCL=GPIO%d  %d Hz",
             sda_gpio, scl_gpio, BOARD_I2C_FREQ_HZ);

    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = BOARD_I2C_PORT,
        .sda_io_num        = sda_gpio,
        .scl_io_num        = scl_gpio,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_enc.bus), TAG, "创建 I2C 总线失败");

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ENCODER_I2C_ADDR,
        .scl_speed_hz    = BOARD_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_enc.bus, &dev_cfg, &s_enc.dev),
                        TAG, "挂载 AS5600 失败");

    /* 探测器件是否存在 */
    const esp_err_t probe = i2c_master_probe(s_enc.bus, ENCODER_I2C_ADDR, 100);
    if (probe != ESP_OK) {
        ESP_LOGW(TAG, "AS5600 (0x%02X) 在 SDA=GPIO%d/SCL=GPIO%d 上无应答: %s",
                 ENCODER_I2C_ADDR, sda_gpio, scl_gpio, esp_err_to_name(probe));
        return probe;
    }

    s_enc.ready   = true;
    s_enc.last_us = esp_timer_get_time();

    /* 读一次确立初始值 */
    uint16_t raw = 0;
    if (as5600_read_angle(&raw) == ESP_OK) {
        s_enc.raw = s_enc.last_raw = raw;
        s_enc.ok = true;
    }

    ESP_LOGI(TAG, "AS5600 初始化完成 (地址 0x%02X, SDA=GPIO%d SCL=GPIO%d)",
             ENCODER_I2C_ADDR, sda_gpio, scl_gpio);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                                  更新                                      */
/* -------------------------------------------------------------------------- */

float encoder_update(void)
{
    if (!s_enc.ready) {
        return 0.0f;
    }

    const int64_t now = esp_timer_get_time();
    float dt = (float)(now - s_enc.last_us) / 1e6f;
    if (dt <= 0.0f) {
        dt = 1e-4f;
    }

    uint16_t raw = 0;
    if (as5600_read_angle(&raw) != ESP_OK) {
        s_enc.err_count++;
        s_enc.ok = false;
        return s_enc.angle;
    }

    /* 同步刷新原始角度与磁场信息（用于诊断） */
    as5600_read_raw_angle(&s_enc.raw_angle);
    if (s_enc.err_count == 0) {
        as5600_refresh_field();
    }

    /* ---- 多圈累加：处理 0 <-> 4095 的跳变 ---- */
    int32_t delta = (int32_t)raw - (int32_t)s_enc.last_raw;
    if (delta >  (int32_t)(AS5600_COUNTS / 2.0f)) {
        delta -= (int32_t)AS5600_COUNTS;
    } else if (delta < -(int32_t)(AS5600_COUNTS / 2.0f)) {
        delta += (int32_t)AS5600_COUNTS;
    }

    const float d_angle = (float)delta * AS5600_RAD_PER_CNT * (float)ENCODER_DIRECTION;
    s_enc.angle      += d_angle;
    s_enc.single_turn = (float)raw * AS5600_RAD_PER_CNT * (float)ENCODER_DIRECTION;
    s_enc.raw         = raw;
    s_enc.last_raw    = raw;
    s_enc.last_us     = now;
    s_enc.ok          = true;

    /* ---- 角速度：差分 + 一阶低通 ---- */
    const float vel_raw = d_angle / dt;
    const float alpha   = dt / (ENCODER_VEL_TAU + dt);
    s_enc.velocity += alpha * (vel_raw - s_enc.velocity);

    return s_enc.angle;
}

/* -------------------------------------------------------------------------- */
/*                                 访问器                                     */
/* -------------------------------------------------------------------------- */

uint16_t encoder_get_raw_angle(void)   { return s_enc.raw_angle; }
uint8_t  encoder_get_agc(void)         { return s_enc.agc; }
uint16_t encoder_get_magnitude(void)   { return s_enc.magnitude; }

float    encoder_get_angle(void)       { return s_enc.angle; }
float    encoder_get_velocity(void)    { return s_enc.velocity; }
float    encoder_get_single_turn(void) { return s_enc.single_turn; }
uint16_t encoder_get_raw(void)         { return s_enc.raw; }
bool     encoder_is_ok(void)           { return s_enc.ok; }

void encoder_set_zero(void)
{
    s_enc.angle = 0.0f;
    ESP_LOGI(TAG, "机械角零点已设置");
}

/* -------------------------------------------------------------------------- */
/*                                 诊断                                       */
/* -------------------------------------------------------------------------- */

void encoder_print_diag(void)
{
    uint8_t st = 0;
    uint16_t raw = 0;

    const esp_err_t e1 = as5600_read_reg(AS5600_REG_STATUS, &st, 1);
    const esp_err_t e2 = as5600_read_angle(&raw);

    printf("[编码器自检] AS5600 @0x%02X (SDA=GPIO%d SCL=GPIO%d)\n",
           ENCODER_I2C_ADDR, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);

    if (e1 != ESP_OK || e2 != ESP_OK) {
        printf("   状态寄存器读取失败 (status=%s, angle=%s)\n",
               esp_err_to_name(e1), esp_err_to_name(e2));
        return;
    }

    const int md = (st >> 5) & 1;   /* 检测到磁铁 */
    const int ml = (st >> 4) & 1;   /* 磁铁太弱 */
    const int mh = (st >> 3) & 1;   /* 磁铁太强 */

    printf("   STATUS   0x%02X  MD=%d ML=%d MH=%d\n", st, md, ml, mh);
    printf("   磁铁     %s\n",
           !md ? "❌ 未检测到磁铁" : (ml ? "⚠️ 磁场太弱（磁铁太远）"
                                       : (mh ? "⚠️ 磁场太强（磁铁太近）" : "✅ 正常")));
    printf("   原始值   %u / 4095   (%.2f°)\n", raw, raw * 360.0f / AS5600_COUNTS);
    printf("   错误次数 %u\n", (unsigned)s_enc.err_count);
}
