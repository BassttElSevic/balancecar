# 问题报告：ESP32-P4 `ADC1_CH5`(GPIO21) 采样值卡死

> 本文只写**实测数据**。凡是没有实测支撑的，一律放在"待验证假设"一节并明确标注。
> 原始串口日志见 `diagnostics/logs/`，可复现工程见 `diagnostics/adc_ab_test/`。

---

## 0. 结论（已修复）

### 现象

同一个固件里同样的两个引脚，两套 ADC 代码得到完全不同的结果：

| ADC 代码路径 | GPIO21 (ADC1_CH5) | GPIO20 (ADC1_CH4) |
| --- | --- | --- |
| 官方例程 `examples/peripherals/adc/oneshot_read` 的写法 | raw ≈ **1701**（正常） | raw ≈ **1697**（正常） |
| 旧版 `components/current/current_sense.c` | raw ≈ **3467**，**卡死**（3360 mV） | raw ≈ **5** |

### 修复

`components/current/current_sense.c` 已重写（见文件头注释）。
关键改动：

1. `adc_oneshot_unit_init_cfg_t` 用 `ADC_UNIT_1` / `ADC_ATTEN_DB_12` / `ADC_BITWIDTH_12` 直接写常量，不再经过 `(adc_unit_t)BOARD_CS_ADC_UNIT` / `CS_ATTEN` / `CS_BITWIDTH` 宏
2. 校准句柄改成**每通道各一个**，并显式填 `.chan`（P4 上 `CONFIG_SOC_ADC_CALIB_CHAN_COMPENS_SUPPORTED=y`，校准是按通道补偿的）
3. 读逻辑收拢到一个 `poll_one()` 里，直接返回 mV

### 修复后的实测结果

```text
I (554) current: 电流采样初始化完成: A=GPIO21(ADC1_CH5)  B=GPIO20(ADC1_CH4)  0.50 V/A
I (625) current: 零点校准完成: A(raw=1698) 1655.4 mV   B(raw=1698) 1655.4 mV
I (628) encoder: AS5600 初始化完成 (地址 0x36, SDA=GPIO5 SCL=GPIO49)
        电流     A:raw1798 1750mV Ia=+0.189A   B:raw1494 1454mV Ib=-0.403A   Ic=+0.213A
```

两路空闲时都回到 **1655.4 mV**（INA240 中点），电机一转电流就正常摆动。完整日志见 `diagnostics/logs/10_fixed_working.log`。

> ⚠️ 诚实说明：**为什么旧写法会坏，根因没有完全定位。** 运行时配置值、编译选项、
> 存储类别、校准句柄配置、提前读、调用顺序全部逐一验证为相同或无关（见第 6 节），
> 但旧写法稳定坏、新写法稳定好。下面第 5、7 节保留完整的定位过程与原始数据，
> 供后人继续查。另：现象 B（AS5600 偶发无应答）与本次修复无关，仍未处理。

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

## 7. 定位过程中最硬的一组证据（已在同一固件内复现）

> **同一块板、同一次启动、逐字段相同的 ADC 初始化，旧 `current_sense_init()` 得到 3466，
> 把同一段代码逐字复制到同一个文件里当普通函数，就得到 1700。**

最后一次二分（`diagnostics/logs/` 之外的临时试验）在同一固件内跑完，结果：

| 变体 | 内容 | CH5 raw |
| --- | --- | --- |
| V1 | `main.c` 里 inline | 1692 ~ 1702 ✅ |
| — | 组件里**逐字复制**的同一段代码 | 1693 ~ 1709 ✅ |
| V2 | 换成文件作用域 `s_unit` | 1691 ~ 1703 ✅ |
| V3 | 再换成 `s_a.chan` / `s_b.chan` | 1696 ~ 1707 ✅ |
| V4 | 再建两个 curve-fitting 校准句柄 | 1697 ~ 1710 ✅ |
| **C** | **`current_sense_init()` 正式路径** | **3464 ~ 3468 ❌** |

V4 与 C 的差别只剩：`ADC_UNIT_1` vs `(adc_unit_t)BOARD_CS_ADC_UNIT`、
`ADC_ATTEN_DB_12` vs `CS_ATTEN`、`ADC_BITWIDTH_12` vs `CS_BITWIDTH`、`ESP_ERROR_CHECK` vs `ESP_RETURN_ON_ERROR`。
这些宏在运行时的实际取值已打印验证过**完全相同**（`logs/` 中 `[DEBUG]` 行）：

```text
unit_cfg: unit_id=1 clk_src=18 ulp_mode=0 | chan_cfg: atten=3 bitwidth=12
macros: ADC_UNIT=1 CH_A=5 CH_B=4 CS_ATTEN=3 CS_BITWIDTH=12
```

**所以差异到底在哪一行，至今没有定位到。** 已知的“旧写法坏、新写法好”是稳定可复现的事实。

后续可查方向（**均为假设，未实测**）：

1. 旧版把 unit 句柄放在 `static cs_ctx_t s_cs` 里，新版放文件作用域裸变量——虽已验证 V2 无影响，但 V2 少了 `cs_ctx_t` 里其它成员的写入。
2. 旧版 `read_mv()` 与 `current_sense_calibrate()` 是分开的函数；新版读逻辑在 `poll_one()` 里。
3. 旧版只建 1 个校准句柄且 `.chan` 未赋值；新版每通道 1 个（这一条是明确应该改的，但单独改它并不能修好）。
4. 是否有 `esp_adc` 驱动侧依赖“调用前 ADC 电源域/RTC 时钟状态”的分支（`new_unit` 后立刻读到 4095 满量程，符合采样保持未正常建立的表现，但这是推测）。

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
| `diagnostics/logs/10_fixed_working.log` | **修复后**的完整固件日志（两路 1655.4 mV，电流随电机摆动） |
| `diagnostics/adc_ab_test/` | **可直接编译运行**的诊断工程（4 组实验在同一个固件内） |

---

## 9. 未验证 / 信息缺失

* 没有示波器波形截图或数据（现象依赖用户口述"GPIO20、GPIO21 都有波"）。
* 没有读过 P4 的 ADC 寄存器（`APB_SARADC_*` / `SENS_*`）来比对两种路径下的实际 atten、采样周期、mux 设置。
* 没有在 broken 状态下用官方写法读**同一个** `adc_oneshot_unit_handle_t`（受"同 unit_id 只能存在一个单元"限制，只能 del/new，已验证 del/new 有效）。
* 现象 B（I2C 偶发 `ESP_ERR_NOT_FOUND`）与现象 A 是否同源，未做任何关联实验。
* 未在 `ADC_UNIT_2`（GPIO49~54）上做对照。
