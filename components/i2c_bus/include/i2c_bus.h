/*
 * ============================================================================
 *  i2c_bus —— I2C 主机总线 / 从机 / 寄存器读写的通用封装
 *
 *  形态严格照 ESP-IDF 官方例程:
 *      examples/peripherals/i2c/i2c_basic/main/i2c_basic_example_main.c
 *    * i2c_new_master_bus() + i2c_master_bus_add_device()
 *    * 总线:  clk_source = I2C_CLK_SRC_DEFAULT
 *            glitch_ignore_cnt = 7
 *            flags.enable_internal_pullup = true
 *    * 从机:  dev_addr_length = I2C_ADDR_BIT_LEN_7
 *            scl_speed_hz   = 默认 400000
 *    * 读寄存器: i2c_master_transmit_receive(dev, &reg, 1, buf, len, timeout)
 *            （超时默认用官方例程的 1000 ms）
 *
 *  存储由调用者提供（不使用堆）:
 *      static i2c_bus_t   s_bus;
 *      static i2c_slave_t s_dev;
 *
 *      i2c_bus_init(&s_bus, &cfg);
 *      i2c_slave_init(&s_dev, &s_bus, addr, freq);
 *      i2c_slave_read_u16_be(&s_dev, reg, &raw);
 *
 *  注意: 句柄结构体的成员仅供本组件内部使用，外部请只通过下面的函数访问。
 * ============================================================================
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

/* -------------------------------------------------------------------------- */
/*  默认值                                                                     */
/* -------------------------------------------------------------------------- */

#define I2C_BUS_DEFAULT_FREQ_HZ       400000u   /**< 官方例程的默认 SCL 频率 */
#define I2C_BUS_DEFAULT_TIMEOUT_MS    1000u     /**< 官方例程的传输超时 */
#define I2C_BUS_DEFAULT_GLITCH_CNT    7         /**< 官方例程的毛刺滤波 */

#define I2C_ADDR_7BIT_MIN             0x08      /**< 7bit 地址扫描下限 */
#define I2C_ADDR_7BIT_MAX             0x77      /**< 7bit 地址扫描上限 */

#define I2C_SLAVE_WRITE_MAX_LEN       16        /**< 单次寄存器写最大字节数 */

/* -------------------------------------------------------------------------- */
/*  句柄（成员内部使用）                                                        */
/* -------------------------------------------------------------------------- */

/** @brief I2C 主机总线配置。freq_hz/timeout_ms/glitch_ignore_cnt 为 0 时取默认值 */
typedef struct {
    i2c_port_num_t port;                    /**< 端口号，如 I2C_NUM_0 */
    int            sda_gpio;                /**< SDA 引脚（用 board_pins.h 的宏） */
    int            scl_gpio;                /**< SCL 引脚（用 board_pins.h 的宏） */
    uint32_t       freq_hz;                 /**< 默认速率；0 -> 400000 */
    uint32_t       timeout_ms;              /**< 单次传输超时；0 -> 1000 */
    uint8_t        glitch_ignore_cnt;       /**< 毛刺滤波；0 -> 7 */
    bool           enable_internal_pullup;  /**< 是否打开芯片内部上拉 */
} i2c_bus_cfg_t;

/** @brief I2C 主机总线句柄（成员内部使用） */
typedef struct {
    i2c_master_bus_handle_t handle;         /**< 内部 */
    i2c_bus_cfg_t           cfg;            /**< 已填过默认值的配置 */
    bool                    ready;          /**< 内部 */
} i2c_bus_t;

/** @brief I2C 从机句柄（成员内部使用） */
typedef struct {
    i2c_master_dev_handle_t handle;         /**< 内部 */
    i2c_bus_t              *bus;            /**< 所属总线 */
    uint8_t                 addr7;          /**< 7bit 地址 */
    bool                    ready;          /**< 内部 */
} i2c_slave_t;

/* -------------------------------------------------------------------------- */
/*  总线                                                                       */
/* -------------------------------------------------------------------------- */

/** @brief 初始化主机总线（存储由调用者提供） */
esp_err_t i2c_bus_init(i2c_bus_t *bus, const i2c_bus_cfg_t *cfg);

/** @brief 反初始化（其下从机必须已 del） */
esp_err_t i2c_bus_deinit(i2c_bus_t *bus);

/** @brief 探测某个 7bit 地址上是否有从机应答 */
esp_err_t i2c_bus_probe(const i2c_bus_t *bus, uint8_t addr7);

/** @brief 带重试的探测（用于处理上电瞬间的偶发无应答） */
esp_err_t i2c_bus_probe_retry(const i2c_bus_t *bus, uint8_t addr7,
                              int retries, uint32_t gap_ms);

/** @brief 扫描 ADDR_7BIT_MIN ~ ADDR_7BIT_MAX，打印应答地址，返回个数（-1 表示参数错） */
int i2c_bus_scan(const i2c_bus_t *bus);

/* ---- 只读访问器（日志/诊断用） ---- */
i2c_port_num_t i2c_bus_get_port(const i2c_bus_t *bus);
int            i2c_bus_get_sda(const i2c_bus_t *bus);
int            i2c_bus_get_scl(const i2c_bus_t *bus);
uint32_t       i2c_bus_get_freq(const i2c_bus_t *bus);
uint32_t       i2c_bus_get_timeout_ms(const i2c_bus_t *bus);

/* -------------------------------------------------------------------------- */
/*  从机                                                                       */
/* -------------------------------------------------------------------------- */

/** @brief 挂载一个从机。freq_hz 为 0 时用总线的默认速率 */
esp_err_t i2c_slave_init(i2c_slave_t *slave, i2c_bus_t *bus,
                         uint8_t addr7, uint32_t freq_hz);

/** @brief 摘掉从机 */
esp_err_t i2c_slave_deinit(i2c_slave_t *slave);

/* -------------------------------------------------------------------------- */
/*  寄存器读写                                                                 */
/* -------------------------------------------------------------------------- */

/** @brief 读一段寄存器（内部为"写地址 + 重复起始 + 读数据"，官方例程同款） */
esp_err_t i2c_slave_read_reg(const i2c_slave_t *slave, uint8_t reg,
                             uint8_t *buf, size_t len);

/** @brief 读 1 个字节 */
esp_err_t i2c_slave_read_u8(const i2c_slave_t *slave, uint8_t reg, uint8_t *out);

/** @brief 读 2 个字节，高字节在前 */
esp_err_t i2c_slave_read_u16_be(const i2c_slave_t *slave, uint8_t reg, uint16_t *out);

/** @brief 写一段寄存器（首字节为寄存器地址） */
esp_err_t i2c_slave_write_reg(const i2c_slave_t *slave, uint8_t reg,
                              const uint8_t *buf, size_t len);

/** @brief 写 1 个字节 */
esp_err_t i2c_slave_write_u8(const i2c_slave_t *slave, uint8_t reg, uint8_t val);
