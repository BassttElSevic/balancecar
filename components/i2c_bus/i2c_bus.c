/*
 * ============================================================================
 *  i2c_bus —— 实现
 *
 *  不使用堆：句柄存储由调用者提供（i2c_bus_t / i2c_slave_t）。
 * ============================================================================
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"

#include "i2c_bus.h"

static const char *TAG = "i2c_bus";

/** @brief 把配置里没填的字段补上默认值 */
static void cfg_apply_defaults(i2c_bus_cfg_t *cfg)
{
    if (cfg->freq_hz           == 0) cfg->freq_hz           = I2C_BUS_DEFAULT_FREQ_HZ;
    if (cfg->timeout_ms        == 0) cfg->timeout_ms        = I2C_BUS_DEFAULT_TIMEOUT_MS;
    if (cfg->glitch_ignore_cnt == 0) cfg->glitch_ignore_cnt = I2C_BUS_DEFAULT_GLITCH_CNT;
}

/* -------------------------------------------------------------------------- */
/*  总线                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t i2c_bus_init(i2c_bus_t *bus, const i2c_bus_cfg_t *cfg)
{
    ESP_RETURN_ON_FALSE(bus && cfg, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    ESP_RETURN_ON_FALSE(!bus->ready, ESP_ERR_INVALID_STATE, TAG, "总线已初始化");
    ESP_RETURN_ON_FALSE(cfg->sda_gpio >= 0 && cfg->scl_gpio >= 0,
                        ESP_ERR_INVALID_ARG, TAG, "引脚无效");
    ESP_RETURN_ON_FALSE(cfg->sda_gpio != cfg->scl_gpio,
                        ESP_ERR_INVALID_ARG, TAG, "SDA 与 SCL 不能是同一条");

    memset(bus, 0, sizeof(*bus));
    bus->cfg = *cfg;
    cfg_apply_defaults(&bus->cfg);

    /* 总线配置：照官方例程 i2c_basic */
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = bus->cfg.port,
        .sda_io_num        = bus->cfg.sda_gpio,
        .scl_io_num        = bus->cfg.scl_gpio,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = bus->cfg.glitch_ignore_cnt,
        .flags.enable_internal_pullup = bus->cfg.enable_internal_pullup,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus->handle),
                        TAG, "创建总线失败");
    bus->ready = true;

    ESP_LOGI(TAG, "总线就绪: port=%d  SDA=GPIO%d  SCL=GPIO%d  %lu Hz  超时 %lu ms",
             (int)bus->cfg.port, bus->cfg.sda_gpio, bus->cfg.scl_gpio,
             (unsigned long)bus->cfg.freq_hz, (unsigned long)bus->cfg.timeout_ms);
    return ESP_OK;
}

esp_err_t i2c_bus_deinit(i2c_bus_t *bus)
{
    ESP_RETURN_ON_FALSE(bus, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    if (!bus->ready) {
        return ESP_OK;
    }
    i2c_del_master_bus(bus->handle);   /* 框架会把其下从机一并清掉 */
    memset(bus, 0, sizeof(*bus));
    return ESP_OK;
}

esp_err_t i2c_bus_probe(const i2c_bus_t *bus, uint8_t addr7)
{
    ESP_RETURN_ON_FALSE(bus && bus->ready, ESP_ERR_INVALID_STATE, TAG, "总线未初始化");
    return i2c_master_probe(bus->handle, addr7, bus->cfg.timeout_ms);
}

esp_err_t i2c_bus_probe_retry(const i2c_bus_t *bus, uint8_t addr7,
                              int retries, uint32_t gap_ms)
{
    ESP_RETURN_ON_FALSE(bus && bus->ready, ESP_ERR_INVALID_STATE, TAG, "总线未初始化");
    ESP_RETURN_ON_FALSE(retries > 0, ESP_ERR_INVALID_ARG, TAG, "重试次数必须 > 0");

    esp_err_t err = ESP_FAIL;
    for (int i = 1; i <= retries; i++) {
        err = i2c_bus_probe(bus, addr7);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "0x%02X 探测成功 (第 %d/%d 次)", addr7, i, retries);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "0x%02X 第 %d/%d 次无应答: %s", addr7, i, retries, esp_err_to_name(err));
        if (i < retries && gap_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(gap_ms));
        }
    }
    ESP_LOGE(TAG, "0x%02X 探测失败: %s", addr7, esp_err_to_name(err));
    return err;
}

int i2c_bus_scan(const i2c_bus_t *bus)
{
    if (!bus || !bus->ready) {
        return -1;
    }
    printf("I2C 扫描 port=%d  SDA=GPIO%d  SCL=GPIO%d  %lu Hz\n",
           (int)bus->cfg.port, bus->cfg.sda_gpio, bus->cfg.scl_gpio,
           (unsigned long)bus->cfg.freq_hz);

    int found = 0;
    for (uint8_t a = I2C_ADDR_7BIT_MIN; a <= I2C_ADDR_7BIT_MAX; a++) {
        if (i2c_master_probe(bus->handle, a, bus->cfg.timeout_ms) == ESP_OK) {
            printf("    0x%02X  ACK\n", a);
            found++;
        }
    }
    printf("    共找到 %d 个设备\n", found);
    fflush(stdout);
    return found;
}

/* -------------------------------------------------------------------------- */
/*  只读访问器                                                                 */
/* -------------------------------------------------------------------------- */

i2c_port_num_t i2c_bus_get_port(const i2c_bus_t *bus)
{
    return bus ? bus->cfg.port : (i2c_port_num_t)-1;
}

int i2c_bus_get_sda(const i2c_bus_t *bus)
{
    return bus ? bus->cfg.sda_gpio : -1;
}

int i2c_bus_get_scl(const i2c_bus_t *bus)
{
    return bus ? bus->cfg.scl_gpio : -1;
}

uint32_t i2c_bus_get_freq(const i2c_bus_t *bus)
{
    return bus ? bus->cfg.freq_hz : 0;
}

uint32_t i2c_bus_get_timeout_ms(const i2c_bus_t *bus)
{
    return bus ? bus->cfg.timeout_ms : 0;
}

/* -------------------------------------------------------------------------- */
/*  从机                                                                       */
/* -------------------------------------------------------------------------- */

esp_err_t i2c_slave_init(i2c_slave_t *slave, i2c_bus_t *bus,
                         uint8_t addr7, uint32_t freq_hz)
{
    ESP_RETURN_ON_FALSE(slave && bus, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    ESP_RETURN_ON_FALSE(bus->ready, ESP_ERR_INVALID_STATE, TAG, "总线未初始化");
    ESP_RETURN_ON_FALSE(!slave->ready, ESP_ERR_INVALID_STATE, TAG, "从机已初始化");
    ESP_RETURN_ON_FALSE(addr7 <= I2C_ADDR_7BIT_MAX, ESP_ERR_INVALID_ARG, TAG, "7bit 地址越界");

    memset(slave, 0, sizeof(*slave));
    slave->bus   = bus;
    slave->addr7 = addr7;

    /* 从机配置：照官方例程 i2c_basic */
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr7,
        .scl_speed_hz    = freq_hz ? freq_hz : bus->cfg.freq_hz,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus->handle, &dev_cfg, &slave->handle),
                        TAG, "挂载 0x%02X 失败", addr7);
    slave->ready = true;

    ESP_LOGI(TAG, "从机就绪: 0x%02X @ %lu Hz", addr7, (unsigned long)dev_cfg.scl_speed_hz);
    return ESP_OK;
}

esp_err_t i2c_slave_deinit(i2c_slave_t *slave)
{
    ESP_RETURN_ON_FALSE(slave, ESP_ERR_INVALID_ARG, TAG, "句柄为空");
    if (!slave->ready) {
        return ESP_OK;
    }
    i2c_master_bus_rm_device(slave->handle);
    memset(slave, 0, sizeof(*slave));
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*  寄存器读写                                                                 */
/* -------------------------------------------------------------------------- */

esp_err_t i2c_slave_read_reg(const i2c_slave_t *slave, uint8_t reg,
                             uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(slave && slave->ready, ESP_ERR_INVALID_STATE, TAG, "从机未就绪");
    ESP_RETURN_ON_FALSE(buf && len > 0, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    /* 官方例程同款：写寄存器地址 + 重复起始 + 读数据 */
    return i2c_master_transmit_receive(slave->handle, &reg, 1, buf, len,
                                       slave->bus->cfg.timeout_ms);
}

esp_err_t i2c_slave_read_u8(const i2c_slave_t *slave, uint8_t reg, uint8_t *out)
{
    return i2c_slave_read_reg(slave, reg, out, 1);
}

esp_err_t i2c_slave_read_u16_be(const i2c_slave_t *slave, uint8_t reg, uint16_t *out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    uint8_t b[2] = {0};
    ESP_RETURN_ON_ERROR(i2c_slave_read_reg(slave, reg, b, 2), TAG, "读 0x%02X 失败", reg);
    *out = (uint16_t)(((uint16_t)b[0] << 8) | b[1]);
    return ESP_OK;
}

esp_err_t i2c_slave_write_reg(const i2c_slave_t *slave, uint8_t reg,
                              const uint8_t *buf, size_t len)
{
    ESP_RETURN_ON_FALSE(slave && slave->ready, ESP_ERR_INVALID_STATE, TAG, "从机未就绪");
    ESP_RETURN_ON_FALSE(buf && len > 0, ESP_ERR_INVALID_ARG, TAG, "参数为空");
    ESP_RETURN_ON_FALSE(len <= I2C_SLAVE_WRITE_MAX_LEN, ESP_ERR_INVALID_ARG,
                        TAG, "写入过长 (%u > %d)", (unsigned)len, I2C_SLAVE_WRITE_MAX_LEN);

    uint8_t tmp[I2C_SLAVE_WRITE_MAX_LEN + 1];
    tmp[0] = reg;
    memcpy(&tmp[1], buf, len);
    return i2c_master_transmit(slave->handle, tmp, len + 1, slave->bus->cfg.timeout_ms);
}

esp_err_t i2c_slave_write_u8(const i2c_slave_t *slave, uint8_t reg, uint8_t val)
{
    return i2c_slave_write_reg(slave, reg, &val, 1);
}
