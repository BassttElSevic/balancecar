/*
 * ============================================================================
 *  balancecar — AS5600 磁编码器驱动 (I2C)
 *
 *  硬件:
 *      SDA = BOARD_I2C_SDA_GPIO (GPIO5)
 *      SCL = BOARD_I2C_SCL_GPIO (GPIO49)
 *      地址 0x36, 400 kHz
 *
 *  ── I2C 用法：严格照 ESP-IDF 官方例程 ───────────────────────────────────
 *      examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
 *        * i2c_new_master_bus()  + i2c_master_bus_add_device()
 *        * bus 配置: clk_source = I2C_CLK_SRC_DEFAULT
 *                    glitch_ignore_cnt = 7
 *                    flags.enable_internal_pullup = true
 *        * device 配置: dev_addr_length = I2C_ADDR_BIT_LEN_7
 *                       scl_speed_hz = 400000
 *        * 读寄存器统一用:
 *            i2c_master_transmit_receive(dev, &reg, 1, buf, len, 1000)
 *          （超时用官方的 1000 ms）
 *
 *  ── 时序/延时：照 DengFOC ────────────────────────────────────────────────
 *      ToanTech/DengFOC_Lib → DengFOC 库/V0.6 支持SVPWM算法/AS5600.cpp
 *        * Wire.begin() 之后 delay(500) 才开始第一次读
 *        * Sensor_init() 里连读 3 次建立初值，读之间 delayMicroseconds(1) / delay(1)
 *        * 角度取 0x0C (RAW ANGLE)，不读 0x0E
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
#define AS5600_REG_RAW      0x0C    /* 12bit 磁场原始角度 —— DengFOC/x_foc 都用这个 */
#define AS5600_REG_ANGLE    0x0E    /* 12bit 经零点/方向编程的角度 */
#define AS5600_REG_AGC      0x1A
#define AS5600_REG_MAG      0x1B
#define AS5600_REG_BURN     0xFF

/* ---- 时序（DengFOC） ---- */
#define AS5600_SETTLE_MS    500     /* Wire.begin() 之后的稳定时间 */
#define AS5600_INIT_READS   3       /* 初始化连读次数 */
#define AS5600_INIT_GAP_US  1000    /* 连读之间的间隔 */

/* ---- 探测重试 ---- */
#define AS5600_PROBE_RETRY  10
#define AS5600_PROBE_GAP_MS 50

/* ---- I2C（官方例程） ---- */
#define AS5600_I2C_TIMEOUT_MS 1000

/* 速度低通滤波时间常数 (秒) */
#define ENCODER_VEL_TAU     0.01f

#define AS5600_COUNTS       4096.0f
#define AS5600_RAD_PER_CNT  (2.0f * (float)M_PI / AS5600_COUNTS)

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    bool     ready;
    bool     ok;
    uint16_t raw;           /* 0x0C 磁场原始角度（主用值） */
    uint16_t last_raw;
    uint16_t ang_reg;       /* 0x0E 经编程的角度（参考） */
    uint8_t  status;        /* 0x0B */
    uint8_t  agc;           /* 0x1A */
    uint16_t magnitude;     /* 0x1B */
    float    angle;         /* 多圈累计机械角 (rad) */
    float    single_turn;
    float    velocity;
    int64_t  last_us;
    uint32_t err_count;
} encoder_ctx_t;

static encoder_ctx_t s_enc;

/* -------------------------------------------------------------------------- */
/*                     I2C 底层（形态照官方例程）                              */
/* -------------------------------------------------------------------------- */

static esp_err_t as5600_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_enc.dev, &reg, 1, buf, len, AS5600_I2C_TIMEOUT_MS);
}

static esp_err_t as5600_read_u16(uint8_t reg, uint16_t *out)
{
    uint8_t b[2] = {0};
    ESP_RETURN_ON_ERROR(as5600_read_reg(reg, b, 2), TAG, "读寄存器 0x%02X 失败", reg);
    *out = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return ESP_OK;
}

/** 读 0x0C 磁场原始角度（主用值，DengFOC 同款） */
static esp_err_t as5600_read_raw_angle(uint16_t *out)
{
    uint16_t v = 0;
    ESP_RETURN_ON_ERROR(as5600_read_u16(AS5600_REG_RAW, &v), TAG, "读 0x0C 失败");
    *out = v & 0x0FFF;
    return ESP_OK;
}

/** 读 0x0E 经编程的角度（参考用） */
static esp_err_t as5600_read_angle_reg(uint16_t *out)
{
    uint16_t v = 0;
    ESP_RETURN_ON_ERROR(as5600_read_u16(AS5600_REG_ANGLE, &v), TAG, "读 0x0E 失败");
    *out = v & 0x0FFF;
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

    /* ---- 3) DengFOC: Wire.begin() 之后 delay(500) ---- */
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

    /* ---- 5) DengFOC: 连读 3 次建立初值 ---- */
    for (int i = 0; i < AS5600_INIT_READS; i++) {
        uint16_t v = 0;
        (void)as5600_read_raw_angle(&v);
        esp_rom_delay_us(AS5600_INIT_GAP_US);
    }

    s_enc.ready   = true;
    s_enc.last_us = esp_timer_get_time();

    if (as5600_read_raw_angle(&s_enc.raw) == ESP_OK) {
        s_enc.last_raw = s_enc.raw;
        (void)as5600_read_angle_reg(&s_enc.ang_reg);
        (void)as5600_read_reg(AS5600_REG_STATUS, &s_enc.status, 1);
        s_enc.ok = true;
    }

    ESP_LOGI(TAG, "AS5600 初始化完成: 地址 0x%02X  raw(0x0C)=%u  angle(0x0E)=%u  STATUS=0x%02X",
             ENCODER_I2C_ADDR, s_enc.raw, s_enc.ang_reg, s_enc.status);
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
    if (as5600_read_raw_angle(&raw) != ESP_OK) {
        s_enc.err_count++;
        s_enc.ok = false;
        return s_enc.angle;
    }
    s_enc.raw = raw;

    /* 参考值：0x0E / STATUS / AGC / MAG（诊断用，不影响主流程） */
    (void)as5600_read_angle_reg(&s_enc.ang_reg);
    (void)as5600_read_reg(AS5600_REG_STATUS, &s_enc.status, 1);
    if (s_enc.err_count == 0) {
        uint8_t agc = 0;
        uint16_t mag = 0;
        if (as5600_read_reg(AS5600_REG_AGC, &agc, 1) == ESP_OK) {
            s_enc.agc = agc;
        }
        if (as5600_read_u16(AS5600_REG_MAG, &mag) == ESP_OK) {
            s_enc.magnitude = mag;
        }
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
    s_enc.last_raw    = raw;
    s_enc.last_us     = now;
    s_enc.ok          = true;

    const float vel_raw = d_angle / dt;
    const float alpha   = dt / (ENCODER_VEL_TAU + dt);
    s_enc.velocity += alpha * (vel_raw - s_enc.velocity);

    return s_enc.angle;
}

/* -------------------------------------------------------------------------- */
/*                                 访问器                                     */
/* -------------------------------------------------------------------------- */

uint16_t encoder_get_raw(void)        { return s_enc.raw; }
uint16_t encoder_get_raw_angle(void)  { return s_enc.ang_reg; }
uint8_t  encoder_get_agc(void)        { return s_enc.agc; }
uint16_t encoder_get_magnitude(void)  { return s_enc.magnitude; }
uint8_t  encoder_get_status(void)     { return s_enc.status; }

float    encoder_get_angle(void)       { return s_enc.angle; }
float    encoder_get_velocity(void)    { return s_enc.velocity; }
float    encoder_get_single_turn(void) { return s_enc.single_turn; }
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
    uint8_t st = s_enc.status;
    const int md = (st >> 5) & 1;
    const int ml = (st >> 4) & 1;
    const int mh = (st >> 3) & 1;

    printf("[编码器自检] AS5600 @0x%02X (SDA=GPIO%d SCL=GPIO%d)\n",
           ENCODER_I2C_ADDR, BOARD_I2C_SDA_GPIO, BOARD_I2C_SCL_GPIO);
    printf("   STATUS=0x%02X  MD=%d ML=%d MH=%d  -> %s\n", st, md, ml, mh,
           !md ? "❌ 未检测到磁铁" : (ml ? "⚠️ 磁场太弱" : (mh ? "⚠️ 磁场太强" : "✅ 磁铁正常")));
    printf("   raw(0x0C)=%u (%.2f°)   angle(0x0E)=%u   AGC=%u   MAG=%u\n",
           s_enc.raw, s_enc.raw * 360.0f / AS5600_COUNTS, s_enc.ang_reg,
           s_enc.agc, s_enc.magnitude);
    printf("   错误次数 %u\n", (unsigned)s_enc.err_count);
}

static void dump_u16(const char *name, uint8_t reg)
{
    uint16_t v = 0;
    if (as5600_read_u16(reg, &v) != ESP_OK) {
        printf("   %-10s (0x%02X)  读取失败\n", name, reg);
        return;
    }
    printf("   %-10s (0x%02X)  = 0x%04X  (%u)\n", name, reg, v, v);
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
