# 问题报告：ESP32-P4 `ADC1_CH5`(GPIO21) 采样值卡死

> 本文只写**实测数据**。凡是没有实测支撑的，一律放在"待验证假设"一节并明确标注。
> 原始串口日志见 `diagnostics/logs/`，可复现工程见 `diagnostics/adc_ab_test/`。

---

## 0. 一句话结论（当前状态）

在**同一块板、同一个固件**里，同样的两个引脚：

| ADC 代码路径 | GPIO21 (ADC1_CH5) | GPIO20 (ADC1_CH4) |
| --- | --- | --- |
| 官方例程 `examples/peripherals/adc/oneshot_read` 的写法 | raw ≈ **1701**（正常） | raw ≈ **1697**（正常） |
| 本仓库 `components/current/current_sense.c` | raw ≈ **3467**，**卡死** | raw ≈ **5** |

**根因尚未确定。** 已排除：硬件/接线（用户示波器实测两脚有波形）、引脚↔通道映射、MCPWM、FreeRTOS、sdkconfig。
**未解决的矛盾**见第 7 节。

---

## 1. 环境

| 项 | 值 |
| --- | --- |
| 芯片 | **ESP32-P4**，revision **v3.2**，Dual Core + LP Core |
| eFuse block rev | v1.2 |
| MAC | `e8:f6:0a:e8:1c:31` |
| 晶振 | 40 MHz |
| Flash | 32 MB，GigaDevice (`detected chip: gd`)，DIO，80 MHz |
| CPU 频率 | 400 MHz |
| ESP-IDF | **v6.1-dev-6354-gd0b65797c30**（`/home/bassttelsevic/esp32/esp-idf`） |
| esptool | 5.5.0 |
| 下载口 | `/dev/ttyACM0`，USB-UART 桥 `1a86:55d3`（WCH USB Single Serial），460800 烧录 / 115200 控制台 |
| uart 控制台 | UART0，GPIO37/38，115200 |
| `CONFIG_FREERTOS_HZ` | 1000 |
| `CONFIG_SOC_ADC_CALIB_CHAN_COMPENS_SUPPORTED` | **y** |
| `CONFIG_SOC_ADC_SHARED_POWER` | y |

> 注：P4 该版本**不是** "ADC 不支持校准" 的那批（IDF issue #18140 里官方答复"该版本 ADC 没支持校准，请升级到 3.1 以上"，本板为 v3.2）。

---

## 2. 硬件接线（来自 `components/board/include/board_pins.h`）

| 信号 | P4 引脚 | ADC 通道 | 板上网络名 |
| --- | --- | --- | --- |
| A 相电流 | **GPIO21** | **ADC1_CH5** | `M0_OUT1_CS`（INA240A2 输出） |
| B 相电流 | **GPIO20** | **ADC1_CH4** | `M0_OUT2_CS`（INA240A2 输出） |
| 编码器 I2C SDA | GPIO5 | — | AS5600 @ 0x36，400 kHz |
| 编码器 I2C SCL | GPIO49 | — | — |
| 三相 PWM | GPIO32 / GPIO27 / GPIO26 | — | 灯哥 FOC V3（L6234） |
| 驱动器使能 EN | GPIO33 | — | 高电平使能 |

引脚↔通道映射已用 IDF 头文件核对，**正确**：

```
components/soc/esp32p4/include/soc/adc_channel.h
    #define ADC1_GPIO20_CHANNEL     4
    #define ADC1_GPIO21_CHANNEL     5
```

**无电流时** INA240 输出应位于中点，`atten=ADC_ATTEN_DB_12` 下 raw ≈ 1700（≈1650 mV）。
**这是本报告判定"正常/异常"的基准。**

---

## 3. 现象

### 3.1 现象 A：`ADC1_CH5` 读数卡死（主要问题）

跑原始 `balancecar` 固件（`motor_init()` → `current_sense_init()` → `current_sense_calibrate()`），
**7 次独立上电/复位**，结果完全一致：

| 启动 | 零点校准 A | 循环中 A(CH5) | 循环中 B(CH4) |
| --- | --- | --- | --- |
| 烧录后第 1 次 | 3360.7 mV | 3359 ~ 3363 mV | 0 ~ 85 mV |
| 复位 1 | — | 3360 / 3363 mV | 3 / 55 mV |
| 复位 2 | — | 3361 / 3359 mV | 50 / 58 mV |
| 复位 3 | — | 3361 / 3360 mV | 57 / 6 mV |
| 复位 4 | — | 3362 / 3362 mV | 8 / 16 mV |
| 复位 5 | — | 3361 / 3359 mV | 4 / 4 mV |
| 按官方例程改成"每通道一个校准句柄"后 | 3359 ~ 3363 mV | 不变 | 不变 |

要点：

* A 路**极差只有 1~4 LSB**（raw 3464~3469），15 个采样点跨度 3 mV → **是死值，不是纹波**。
* B 路偶尔跳到 20~85 mV，但绝大部分时间贴 0。
* 换成每通道独立校准句柄（官方例程写法）**没有任何改善** → 说明问题不在校准换算，而在 raw。
* 后来把 raw 值也打印出来，确认 **raw 本身就是 3465**（不是 mV 换算错）。

### 3.2 现象 B：AS5600 I2C 探测偶发失败

连跑 6 次上电/复位，探测 `0x36`：

* **5 次成功**
* **1 次失败**：`W (628) encoder: AS5600 (0x36) 在 SDA=GPIO5/SCL=GPIO49 上无应答: ESP_ERR_NOT_FOUND`（见 `logs/09_i2c_probe_5_resets.log`）

探测成功时寄存器都能读：`raw(0x0E)` 249~254、`raw(0x0C)` 249~254、`AGC(0x1A)=107` 恒定、`MAG(0x1B)` 2096~2100、单圈 21.9°~22.3°。
固件的"最近 5 秒 Δ"统计只有 1~6，因此打印 `❌ 没动`。
（**该现象是否与现象 A 同源，未验证。**）

---

## 4. 复现步骤

```bash
# 1) 主工程（复现现象 A）
cd balancecar
source ~/esp32/esp-idf/export.sh
idf.py -p /dev/ttyACM0 flash
# 观察串口：A 恒 3360mV / B 恒 ~0mV

# 2) 诊断工程（同固件内 4 组对比）
cd diagnostics/adc_ab_test
idf.py set-target esp32p4
idf.py -p /dev/ttyACM0 flash monitor
```

---

## 5. 关键实验与原始数据

### 实验 1：同固件 A/B（`logs/02_same_firmware_AB.log`）

**同一个二进制**，先跑官方写法、`del_unit()`，再跑 `current_sense` 路径。`motor_init()` 已执行、EN 为低。

```
[A] 官方 oneshot_read 例程写法 (unit 只有 .unit_id, bitwidth=DEFAULT)
    校准句柄: CH5=OK  CH4=OK
  A) 官方例程写法    | A/CH5(GPIO21) avg=1701 [1651..1810] 1655mV | B/CH4(GPIO20) avg=1697 [1522..1810] 1652mV
[B] 项目 current_sense.c 写法
I (1139) current: ADC 校准句柄: A 相 0x0x4ffbb780 (chan=5)  B 相 0x0x4ffbb7b4 (chan=4)
    current_sense_init: ESP_OK
    current_sense_calibrate: ESP_OK
  B) 项目写法          | A/CH5(GPIO21) avg=3467 [3465..3469] 3363mV | B/CH4(GPIO20) avg=   5 [   1..  24]    3mV
  B) 项目写法(转起来) | A/CH5(GPIO21) avg=3466 [3465..3469] 3362mV | B/CH4(GPIO20) avg=   9 [   0..  37]   16mV
```

→ **同一次启动内，两条路径结果不同。问题定位到 `current_sense.c` 这条路径。**

### 实验 2：纯 ADC + 电机（`logs/05_pure_adc_plus_motor.log`）

去掉 I2C/编码器/`current_sense`，只留 ADC（官方写法）+ 真实 `motor` 组件：

```
  0) 基线: 只有 ADC         | CH5 avg=1694 [1588..1791] 1649mV | CH4 avg=1692 [1476..1710] 1647mV
>>> motor_init()  (MCPWM 30kHz 定时器启动, EN 仍为低)
  1) motor_init() 之后      | CH5 avg=1706 [1682..1845] 1660mV | CH4 avg=1708 [1686..1934] 1662mV
>>> motor_enable(true) + motor_zero()
  2) 驱动器使能(零矢量)之后 | CH5 avg=1703 [1674..1772] 1657mV | CH4 avg=1696 [1597..1739] 1651mV
>>> 正弦调制 1 秒
  3) 正弦调制中            | CH5 avg=1825 [1747..1987] 1776mV | CH4 avg=1498 [1288..1525] 1458mV
>>> motor_enable(false)
  4) 失能之后              | CH5 avg=1698 [1658..1732] 1653mV | CH4 avg=1702 [1688..1795] 1656mV
```

→ **ADC 外设、GPIO21/GPIO20、MCPWM 起振、驱动器使能、电机真实旋转，全都不影响读数。**
→ 正弦调制时 CH5 在 1747~1987、CH4 在 1288~1525 之间摆动，**两路都在动**。

### 实验 3：ADC 配置穷举（`logs/06_config_matrix.log`）

逐个字段拆开 `unit` 的 `clk_src/ulp_mode` 与通道的 `bitwidth`：

```
  1) unit={unit_id}             bw=DEFAULT           | CH5 avg=1705 [1692..1806] 1659mV | CH4 avg=1696 1651mV
  2) unit={unit_id}             bw=ADC_BITWIDTH_12   | CH5 avg=1700 [1647..1752] 1654mV | CH4 avg=1697 1652mV
  3) unit={unit_id,clk_src,ulp} bw=DEFAULT           | CH5 avg=1698 [1661..1714] 1653mV | CH4 avg=1700 1654mV
  4) unit={unit_id,clk_src,ulp} bw=ADC_BITWIDTH_12   | CH5 avg=1697 [1691..1715] 1652mV | CH4 avg=1684 1639mV
  5) unit={unit_id}             bw=DEFAULT  (复查)    | CH5 avg=1699 [1691..1711] 1653mV | CH4 avg=1702 1656mV
  6) bw=ADC_BITWIDTH_9   | CH5=4095 恒  | cali=ESP_ERR_INVALID_ARG
  7) bw=ADC_BITWIDTH_10  | CH5=4095 恒  | cali=ESP_ERR_INVALID_ARG
  8) bw=ADC_BITWIDTH_11  | CH5=4095 恒  | cali=ESP_ERR_INVALID_ARG
```

→ **这 4 种组合（含第 4 组 = 项目同款配置）全部读到 ~1700。配置字段本身不是原因。**
→ 附带发现：`bitwidth=9/10/11` 时 `adc_oneshot_read` 返回 4095 且 curve-fitting 校准创建报 `ESP_ERR_INVALID_ARG`（属另一件事，P4 只支持 12 bit）。

### 实验 4：手写等价配置（`logs/07_handwritten_project_config_x4.log`）

在**独立固件**里手写和 `current_sense.c` 逐字段相同的 ADC 初始化，`motor_init()` 在前，**首次创建单元**，
用同一个二进制连跑 4 次：

```
A) 首次创建, 项目配置(bw=12) | CH5 avg= 1710 [1609..1962] 1664mV | CH4 avg= 1711 1665mV
A) 首次创建, 项目配置(bw=12) | CH5 avg= 1699 [1692..1714] 1653mV | CH4 avg= 1688 1643mV
A) 首次创建, 项目配置(bw=12) | CH5 avg= 1710 [1690..1846] 1664mV | CH4 avg= 1711 1665mV
A) 首次创建, 项目配置(bw=12) | CH5 avg= 1698 [1688..1719] 1653mV | CH4 avg= 1708 1662mV
```

→ 手写版本**稳定正常**（4/4）。

### 实验 5：插桩定位（`logs/03_current_sense_instrumented.log`）

在 `current_sense_init()` 的每一步之后立刻读一次 raw：

```
I (498) current:   [probe] new_unit 之后        -> CH5(raw)=4095  CH4(raw)= 109
I (498) current:   [probe] config CH5 之后      -> CH5(raw)=3465  CH4(raw)=  59
I (506) current:   [probe] config CH4 之后      -> CH5(raw)=3464  CH4(raw)=  39
I (522) current:   [probe] cali 创建之后        -> CH5(raw)=3467  CH4(raw)=  17
I (530) current:   [probe] 再读一次             -> CH5(raw)=3466  CH4(raw)=  36
I (538) current:   [probe] 再读二次             -> CH5(raw)=3464  CH4(raw)=   2
I (557) current:   [cal] #0:   A raw=3464 3359mV   B raw=   9   10mV
I (563) current:   [cal] #1:   A raw=3465 3360mV   B raw=  22   23mV
I (569) current:   [cal] #2:   A raw=3465 3360mV   B raw=  44   44mV
I (637) current:   [cal] #253: A raw=3465 3360mV   B raw=  23   24mV
I (638) current:   [cal] #254: A raw=3468 3363mV   B raw=   1    0mV
I (639) current:   [cal] #255: A raw=3465 3360mV   B raw=   1    0mV
I (644) current: 零点校准完成: A=3360.9 mV  B=18.5 mV  (共 256 次采样)
```

→ **`new_unit` 一返回就已经不对**（CH5 读满量程 4095），配置成 `DB_12/12bit` 之后固定为 3465。
→ **不是"采样若干次之后才坏"，是一开始就坏。**
→ 256 次校准采样全程稳定在 3460~3468。

### 实验 6：把主工程剥到最小（`logs/04_minimal_current_sense.log`）

`main.c` 只留 `current_sense_init/calibrate/update`，**不调 `motor_init()`、不调 `encoder_init()`**：

```
### minimal test, rev v3.2 ###
current_sense_init: ESP_OK
current_sense_calibrate: ESP_OK
I (574) current: 零点校准完成: A=3360.1 mV  B=16.3 mV  (共 256 次采样)
    A/CH5(GPIO21) raw=3466 3361mV    B/CH4(GPIO20) raw=   1    0mV
    A/CH5(GPIO21) raw=3465 3360mV    B/CH4(GPIO20) raw=  20   21mV
```

→ **不需要电机、不需要 I2C，单独跑 `current_sense` 就能复现。**

### 实验 7：把同一份最小代码搬进另一个 ESP-IDF 工程

把上面那份 `main.c` 原样放进一个**独立新建的** ESP-IDF 工程（同样的 target / 同样的 components 副本）编译：

```
### minimal test (temp 工程), rev v3.2 ###
I (574) current: 零点校准完成: A=3360.1 mV  B=16.3 mV  (共 256 次采样)
    A/CH5(GPIO21) raw=3466 3361mV    B/CH4(GPIO20) raw=   1    0mV
```

→ **同样复现**，排除"主工程的构建/链接环境"这一类因素。

---

## 6. 已排除项（均有实测支撑）

| # | 排除项 | 依据 |
| --- | --- | --- |
| 1 | 硬件 / 接线 / INA240 输出 | 用户示波器实测 GPIO20、GPIO21 两脚都有波形；且官方写法在同板同引脚读到 1650mV 中点和 1288~1987 的摆动 |
| 2 | 引脚 ↔ ADC 通道映射 | 对照 `soc/esp32p4/include/soc/adc_channel.h`：GPIO20=CH4、GPIO21=CH5，与 `board_pins.h` 一致 |
| 3 | ADC 外设本身 | 实验 2：隔离固件下 CH4/CH5 都能读到会动的值 |
| 4 | MCPWM / 电机起振 / 驱动器使能 / 电机旋转 | 实验 2：`motor_init`、`motor_enable`、正弦调制全程读数正常 |
| 5 | 需要电机或 I2C 参与 | 实验 6：都不调用，仍然复现 |
| 6 | `adc_oneshot` 的 `clk_src` / `ulp_mode` / `bitwidth` 字段 | 实验 3：4 种组合全部正常 |
| 7 | 校准句柄（单句柄未填 `.chan`） | 按官方例程改成每通道一个句柄、填 `.chan` 后，raw 仍是 3465 |
| 8 | 零点校准流程 / 换算 | 实验 5：`new_unit` 后立刻就是 4095；raw 本身就坏 |
| 9 | `sdkconfig` 差异 | 主工程与诊断工程的 `sdkconfig` 均 1256 行 `CONFIG_`，`diff` 为空 |
| 10 | C 源码层配置差异 | 预处理 `current_sense.c`（`gcc -E`）逐字段核对：`unit_id=1 / clk_src=ADC_RTC_CLK_SRC_DEFAULT / ulp_mode=DISABLE / atten=DB_12 / bitwidth=12`，与手写版**完全一致** |
| 11 | "先读未配置通道会破坏 ADC" | 实验 5 里确实在 config 前读到 4095，但原始 `current_sense.c` **没有**这个提前读（实验 6），同样是 3466 |
| 12 | `current_sense_init()` 相对于 `motor_init()` 的调用顺序 | 把 `current_sense_init()` 挪到 `motor_init()` 之前（`logs/08_order_change_no_effect.log`），仍是 `A:raw3465 3360mV` |

---

## 7. 未解决的核心矛盾（请从这里接手）

> **同一块板、同一次启动、逐字段相同的 ADC 初始化，`current_sense.c` 得到 3467，手写等价代码得到 1701。**

具体：

| 实验 | ADC 初始化配置 | 结果 |
| --- | --- | --- |
| 官方例程写法（实验 1，同固件内） | `unit={.unit_id}` + `bw=DEFAULT` | **1701** ✅ |
| `current_sense.c`（实验 1，同固件内） | `unit={.unit_id, .clk_src=RTC_DEFAULT, .ulp_mode=DISABLE}` + `bw=12` | **3467** ❌ |
| 手写同配置（实验 4，独立固件，4 次） | 与上面**逐字段相同** | **1699~1710** ✅ |
| 配置穷举（实验 3） | 4 种组合都试过 | 全部 **~1700** ✅ |

预处理后的取值完全一致，`sdkconfig` 完全一致，但行为不同。
**尚未定位到差异点。** 我最后想做的"同二进制内 `new_unit` 后立即插桩 + 手写路径对照"那次编译失败了（`diagnostics` 工程 `motor.h` 路径问题，已修），没有拿到数据。

剩余待查方向（**均为假设，未经实测**）：

1. `current_sense.c` 里 `const adc_oneshot_chan_cfg_t chan_cfg`（`.rodata`）与手写版非 `const`（栈）的实参形态差异是否影响驱动取配置。
2. `current_sense.c` 使用 `static cs_ctx_t s_cs`（大结构体在 `.bss`）间接持有 `unit` 句柄，与全局裸句柄是否有差异。
3. 驱动层是否有依赖"调用前 ADC 电源域/RTC 时钟状态"的分支；两者在 `new_unit` 之前的**运行历史**（`printf`、`esp_chip_info`、`vTaskDelay` 次数与时序）不同。
4. 是否与 `CONFIG_SOC_ADC_SHARED_POWER=y` 下的 ADC 上电时序竞态有关（`new_unit` 一返回就读到 4095 满量程，符合"采样保持未正常建立"的表现，但这是推测）。

**建议的第一步**：写一个固件，在**同一次启动内**依次做
`(a) 手写路径 → 读 → del_unit → (b) current_sense 路径 → 读`，
并打印 `adc_oneshot_new_unit()` / `adc_oneshot_config_channel()` 的返回码、以及每一步之后的 raw。
如果 (a) 正常 (b) 不正常，再逐个把 `current_sense_init()` 改写成 (a) 的形式（先去掉 `const`、再换掉 `s_cs` 间接句柄），二分出翻转点。

---

## 8. 证据文件索引

| 文件 | 内容 |
| --- | --- |
| `diagnostics/logs/01_project_boot_original.log` | 原始 `balancecar` 完整启动日志（现象 A + 现象 B） |
| `diagnostics/logs/02_same_firmware_AB.log` | 同固件内 官方写法 vs `current_sense` 写法（实验 1） |
| `diagnostics/logs/03_current_sense_instrumented.log` | 逐步骤插桩（实验 5） |
| `diagnostics/logs/04_minimal_current_sense.log` | 主工程剥到最小（实验 6） |
| `diagnostics/logs/05_pure_adc_plus_motor.log` | 纯 ADC + 真实电机组件（实验 2） |
| `diagnostics/logs/06_config_matrix.log` | ADC 配置穷举（实验 3） |
| `diagnostics/logs/07_handwritten_project_config_x4.log` | 手写等价配置 ×4（实验 4） |
| `diagnostics/logs/08_order_change_no_effect.log` | 调整 `current_sense_init` 调用顺序，无效 |
| `diagnostics/logs/09_i2c_probe_5_resets.log` | AS5600 探测 5 次成功 / 1 次失败（现象 B） |
| `diagnostics/adc_ab_test/` | **可直接编译运行**的诊断工程（4 组实验在同一个固件内） |

---

## 9. 未验证 / 信息缺失

* 没有示波器波形截图或数据（现象依赖用户口述"GPIO20、GPIO21 都有波"）。
* 没有读过 P4 的 ADC 寄存器（`APB_SARADC_*` / `SENS_*`）来比对两种路径下的实际 atten、采样周期、mux 设置。
* 没有在 broken 状态下用官方写法读**同一个** `adc_oneshot_unit_handle_t`（受"同 unit_id 只能存在一个单元"限制，只能 del/new，已验证 del/new 有效）。
* 现象 B（I2C 偶发 `ESP_ERR_NOT_FOUND`）与现象 A 是否同源，未做任何关联实验。
* 未在 `ADC_UNIT_2`（GPIO49~54）上做对照。
