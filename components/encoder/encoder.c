/*
 * ============================================================================
 *  balancecar — AS5600 磁编码器驱动 (I2C)
 *
 *  ┌─ I2C 初始化：严格照 ESP-IDF 官方例程 ────────────────────────────────
 *  │   examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
 *  │     * i2c_new_master_bus() + i2c_master_bus_add_device()
 *  │     * bus:    clk_source = I2C_CLK_SRC_DEFAULT
 *  │               glitch_ignore_cnt = 7
 *  │               flags.enable_internal_pullup = true
 *  │     * device: dev_addr_length = I2C_ADDR_BIT_LEN_7
 *  │               scl_speed_hz = 400000
 *  │     * 读寄存器: i2c_master_transmit_receive(dev, &reg, 1, buf, len, 1000)
 *  │               （超时 = 官方例程的 1000 ms）
 *  └──────────────────────────────────────────────────────────────────────
 *
 *  ┌─ 读角度 + 多圈 + 速度的逻辑：照 DengFOC ─────────────────────────────
 *  │   ToanTech/DengFOC_Lib → DengFOC 库/V0.6 支持SVPWM算法/AS5600.cpp
 *  │     getSensorAngle()  读 0x0C, 掩码 (buf[1]&0x1FF) + ((buf[0]&0x1F)<<8)
 *  │                       返回 (readValue / 4096) * 2π
 *  │     Sensor_init()     begin → delay(500) → 连读 3 次
 *  │     Sensor_update()   d_angle = val - angle_prev;
 *  │                       |d_angle| > 0.8*2π 时圈数 ±1
 *  │     getAngle()        full_rotations*2π + angle_prev
 *  │     getVelocity()     (Δfull_rotations*2π + Δangle) / Ts
 *  └──────────────────────────────────────────────────────────────────────
 * ============================================================================
 */

#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

#include "board_pins.h"
#include "encoder.h"

static const char *TAG = "encoder";

/* ---- AS5600 寄存器 ---- */
#define AS5600_REG_ZMCO     0x00
#define AS5600_REG_ZPOS     0x01
#define AS5600_REG_MPOS     0x03
#define AS5600_REG_MANG     0x05
#define AS5600_REG_CONF     0x07
#define AS5600_REG_STATUS   0x0B
#define AS5600_REG_RAW      0x0C    /* DengFOC 用的就是这一个 */
#define AS5600_REG_ANGLE    0x0E
#define AS5600_REG_AGC      0x1A
#define AS5600_REG_MAG      0x1B
#define AS5600_REG_BURN     0xFF

/* ---- 时序（DengFOC） ---- */
#define AS5600_SETTLE_MS    500
#define AS5600_PROBE_RETRY  10
#define AS5600_PROBE_GAP_MS 50

/* ---- I2C（官方例程） ---- */
#define AS5600_I2C_TIMEOUT_MS 1000

#define TWO_PI_F            6.28318530718f

/* DengFOC getSensorAngle() 里的位定义 */
#define AS5600_BIT_RES      12
#define AS5600_BITS_MSB     (11 - 7)                        /* 4 */
#define AS5600_CPR          ((float)(1 << AS5600_BIT_RES))   /* 4096 */
#define AS5600_LSB_USED     (AS5600_BIT_RES - AS5600_BITS_MSB)  /* 8 */

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    bool     ready;
    bool     ok;

    /* ---- DengFOC Sensor_AS5600 同款状态 ---- */
    float    angle_prev;         /* 上次机械角 0 ~ 2π */
    int64_t  angle_prev_ts;      /* 上次时间戳 (us) */
    int32_t  full_rotations;     /* 累计圈数 */
    float    vel_angle_prev;     /* 上次算速度时的角度 */
    int64_t  vel_angle_prev_ts;
    int32_t  vel_full_rotations;

    /* ---- 诊断（不影响主流程） ---- */
    uint16_t raw;                /* 0x0C 原始 12bit */
    uint16_t ang_reg;            /* 0x0E 参考 */
    uint8_t  status;
    uint8_t  agc;
    uint16_t magnitude;
    float    velocity;
    uint32_t err_count;
} encoder_ctx_t;

static encoder_ctx_t s_enc;

/* -------------------------------------------------------------------------- */
/*          I2C 底层（形态照官方例程 i2c_basic）                              */
/* -------------------------------------------------------------------------- */

static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_enc.dev, &reg, 1, buf, len, AS5600_I2C_TIMEOUT_MS);
}

/**
 * @brief DengFOC 的 getSensorAngle()
 *        读 0x0C 两个字节，按 DengFOC 的掩码拼出 12bit，换算成 0 ~ 2π
 */
static esp_err_t as5600_get_sensor_angle(float *out_rad)
{
    uint8_t readArray[2] = {0};
    ESP_RETURN_ON_ERROR(as5600_read_reg(AS5600_REG_RAW, readArray, 2), TAG, "读 0x0C 失败");

    /* 与 DengFOC 逐字一致 */
    const uint8_t lsb_mask = (uint8_t)((2 << AS5600_LSB_USED) - 1);       /* 0x1FF */
    const uint8_t msb_mask = (uint8_t)((2 << AS5600_BITS_MSB) - 1);       /* 0x1F  */

    uint16_t readValue  = (uint16_t)(readArray[1] & lsb_mask);
    readValue += (uint16_t)((readArray[0] & msb_mask) << AS5600_LSB_USED);

    s_enc.raw = readValue & 0x0FFF;
    *out_rad = ((float)readValue / AS5600_CPR) * TWO_PI_F;
    return ESP_OK;
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

    /* ---- 1) 总线（官方例程 i2c_basic 同款） ---- */
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = BOARD_I2C_PORT,
        .sda_io_num        = sda_gpio,
        .scl_io_num        = scl_gpio,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_enc.bus), TAG, "创建 I2C 总线失败");

    /* ---- 2) 挂载从机（官方例程 i2c_basic 同款） ---- */
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = ENCODER_I2C_ADDR,
        .scl_speed_hz    = BOARD_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_enc.bus, &dev_cfg, &s_enc.dev),
                        TAG, "挂载 AS5600 失败");

    /* ---- 3) DengFOC Sensor_init(): wire->begin(); delay(500) ---- */
    vTaskDelay(pdMS_TO_TICKS(AS5600_SETTLE_MS));

    /* ---- 4) 探测（带重试） ---- */
    esp_err_t probe = ESP_FAIL;
    for (int i = 0; i < AS5600_PROBE_RETRY; i++) {
        probe = i2c_master_probe(s_enc.bus, ENCODER_I2C_ADDR, AS5600_I2C_TIMEOUT_MS);
        if (probe == ESP_OK) {
            ESP_LOGI(TAG, "AS5600 探测成功 (第 %d 次尝试)", i + 1);
            break;
        }
        ESP_LOGW(TAG, "AS5600 第 %d/%d 次探测无应答: %s",
                 i + 1, AS5600_PROBE_RETRY, esp_err_to_name(probe));
        vTaskDelay(pdMS_TO_TICKS(AS5600_PROBE_GAP_MS));
    }
    if (probe != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 (0x%02X) 在 SDA=GPIO%d/SCL=GPIO%d 上无应答: %s",
                 ENCODER_I2C_ADDR, sda_gpio, scl_gpio, esp_err_to_name(probe));
        return probe;
    }

    s_enc.ready = true;

    /* ---- 5) DengFOC Sensor_init() 的连读序列 ---- */
    float a = 0.0f;

    (void)as5600_get_sensor_angle(&a);                       /* getSensorAngle();        */
    esp_rom_delay_us(1);                                     /* delayMicroseconds(1);    */

    (void)as5600_get_sensor_angle(&a);                       /* vel_angle_prev = ...     */
    s_enc.vel_angle_prev    = a;
    s_enc.vel_angle_prev_ts = esp_timer_get_time();
    s_enc.vel_full_rotations = 0;

    vTaskDelay(pdMS_TO_TICKS(1));                            /* delay(1);                */
    (void)as5600_get_sensor_angle(&a);                       /* getSensorAngle();        */
    esp_rom_delay_us(1);                                     /* delayMicroseconds(1);    */

    (void)as5600_get_sensor_angle(&a);                       /* angle_prev = ...         */
    s_enc.angle_prev    = a;
    s_enc.angle_prev_ts = esp_timer_get_time();
    s_enc.full_rotations = 0;
    s_enc.ok = true;

    ESP_LOGI(TAG, "AS5600 初始化完成: 0x%02X  raw(0x0C)=%u  angle0=%.4f rad (%.1f°)",
             ENCODER_I2C_ADDR, s_enc.raw, (double)s_enc.angle_prev,
             (double)(s_enc.angle_prev * 180.0f / (float)M_PI));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*            更新（照 DengFOC 的 Sensor_update()）                            */
/* -------------------------------------------------------------------------- */

float encoder_update(void)
{
    if (!s_enc.ready) {
        return s_enc.angle_prev;
    }

    float val = 0.0f;
    if (as5600_get_sensor_angle(&val) != ESP_OK) {
        s_enc.err_count++;
        s_enc.ok = false;
        return s_enc.angle_prev;
    }

    /* ---- DengFOC Sensor_update() ---- */
    s_enc.angle_prev_ts = esp_timer_get_time();

    const float d_angle = val - s_enc.angle_prev;
    /* 圈数检测：越过 0.8 圈认为跨过了 0/2π */
    if (fabsf(d_angle) > (0.8f * TWO_PI_F)) {
        s_enc.full_rotations += (d_angle > 0.0f) ? -1 : 1;
    }
    s_enc.angle_prev = val;
    s_enc.ok = true;
    return s_enc.angle_prev;
}

/* -------------------------------------------------------------------------- */
/*                                 访问器                                     */
/* -------------------------------------------------------------------------- */

/** DengFOC getMechanicalAngle()：单圈机械角 0 ~ 2π */
float encoder_get_single_turn(void)
{
    return s_enc.angle_prev;
}

/** DengFOC getAngle()：多圈累计机械角 (rad) */
float encoder_get_angle(void)
{
    return (float)s_enc.full_rotations * TWO_PI_F + s_enc.angle_prev;
}

/** DengFOC getVelocity()：rad/s */
float encoder_get_velocity(void)
{
    float Ts = (float)(s_enc.angle_prev_ts - s_enc.vel_angle_prev_ts) * 1e-6f;
    if (Ts <= 0.0f) {
        Ts = 1e-3f;
    }
    const float vel = ((float)(s_enc.full_rotations - s_enc.vel_full_rotations) * TWO_PI_F
                       + (s_enc.angle_prev - s_enc.vel_angle_prev)) / Ts;

    s_enc.vel_angle_prev     = s_enc.angle_prev;
    s_enc.vel_full_rotations = s_enc.full_rotations;
    s_enc.vel_angle_prev_ts  = s_enc.angle_prev_ts;
    s_enc.velocity = vel;
    return vel;
}

uint16_t encoder_get_raw(void)        { return s_enc.raw; }
uint16_t encoder_get_raw_angle(void)  { return s_enc.ang_reg; }
uint8_t  encoder_get_agc(void)        { return s_enc.agc; }
uint16_t encoder_get_magnitude(void)  { return s_enc.magnitude; }
uint8_t  encoder_get_status(void)     { return s_enc.status; }
bool     encoder_is_ok(void)          { return s_enc.ok; }
int32_t  encoder_get_full_rotations(void) { return s_enc.full_rotations; }

void encoder_set_zero(void)
{
    s_enc.full_rotations = 0;
    s_enc.angle_prev     = 0.0f;
    ESP_LOGI(TAG, "机械角零点已设置");
}

/* -------------------------------------------------------------------------- */
/*                                 诊断                                       */
/* -------------------------------------------------------------------------- */

/** 刷新诊断寄存器（0x0E / STATUS / AGC / MAG）—— 不参与角度计算 */
void encoder_refresh_diag(void)
{
    uint8_t b[2] = {0};
    if (as5600_read_reg(AS5600_REG_ANGLE, b, 2) == ESP_OK) {
        s_enc.ang_reg = (uint16_t)(((uint16_t)b[0] << 8 | b[1]) & 0x0FFF);
    }
    if (as5600_read_reg(AS5600_REG_STATUS, &s_enc.status, 1) != ESP_OK) {
        s_enc.status = 0;
    }
    if (as5600_read_reg(AS5600_REG_AGC, &s_enc.agc, 1) != ESP_OK) {
        s_enc.agc = 0;
    }
    if (as5600_read_reg(AS5600_REG_MAG, b, 2) == ESP_OK) {
        s_enc.magnitude = (uint16_t)((uint16_t)b[0] << 8 | b[1]);
    }
}

void encoder_print_diag(void)
{
    const uint8_t st = s_enc.status;
    printf("[编码器自检] AS5600 @0x%02X (SDA=GPIO%d SCL=GPIO%d)\n",
           ENCODER_I2C_ADDR, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
    printf("   STATUS=0x%02X  MD=%d ML=%d MH=%d  -> %s\n",
           st, (st >> 5) & 1, (st >> 4) & 1, (st >> 3) & 1,
           !((st >> 5) & 1) ? "❌ 未检测到磁铁"
                            : (((st >> 4) & 1) ? "⚠️ 磁场太弱"
                                                : (((st >> 3) & 1) ? "⚠️ 磁场太强" : "✅ 磁铁正常")));
    printf("   raw(0x0C)=%u (%.2f°)   angle(0x0E)=%u   AGC=%u   MAG=%u\n",
           s_enc.raw, s_enc.raw * 360.0f / 4096.0f, s_enc.ang_reg, s_enc.agc, s_enc.magnitude);
    printf("   机械角 %.4f rad   圈数 %ld   速度 %.4f rad/s   错误 %u\n",
           (double)s_enc.angle_prev, (long)s_enc.full_rotations,
           (double)s_enc.velocity, (unsigned)s_enc.err_count);
}

static void dump_u16(const char *name, uint8_t reg)
{
    uint8_t b[2] = {0};
    if (as5600_read_reg(reg, b, 2) != ESP_OK) {
        printf("   %-10s (0x%02X)  读取失败\n", name, reg);
        return;
    }
    printf("   %-10s (0x%02X)  = 0x%02X%02X  (%u)\n",
           name, reg, b[0], b[1], (unsigned)((uint16_t)b[0] << 8 | b[1]));
}

static void dump_u8(const char *name, uint8_t reg, int is_status)
{
    uint8_t v = 0;
    if (as5600_read_reg(reg, &v, 1) != ESP_OK) {
        printf("   %-10s (0x%02X)  读取失败\n", name, reg);
        return;
    }
    if (is_status) {
        printf("   %-10s (0x%02X)  = 0x%02X  MD=%d ML=%d MH=%d\n",
               name, reg, v, (v >> 5) & 1, (v >> 4) & 1, (v >> 3) & 1);
    } else {
        printf("   %-10s (0x%02X)  = 0x%02X  (%u)\n", name, reg, v, v);
    }
}

void encoder_dump_regs(void)
{
    printf("\n===== AS5600 寄存器 dump @0x%02X =====\n", ENCODER_I2C_ADDR);
    dump_u8 ("ZMCO",      AS5600_REG_ZMCO,   0);
    dump_u16("ZPOS",      AS5600_REG_ZPOS);
    dump_u16("MPOS",      AS5600_REG_MPOS);
    dump_u16("MANG",      AS5600_REG_MANG);
    dump_u16("CONF",      AS5600_REG_CONF);
    dump_u8 ("STATUS",    AS5600_REG_STATUS, 1);
    dump_u16("RAW_ANGLE", AS5600_REG_RAW);
    dump_u16("ANGLE",     AS5600_REG_ANGLE);
    dump_u8 ("AGC",       AS5600_REG_AGC,    0);
    dump_u16("MAGNITUDE", AS5600_REG_MAG);
    dump_u8 ("BURN",      AS5600_REG_BURN,   0);
    printf("====================================\n\n");
    fflush(stdout);
}
