# ESP32-C6 Temperature Controller

基于 nanoESP32-C6 v1.0 的双传感器温控项目（MAX6675 + MAX31865 PT100）。

## 🎯 当前功能

- ✅ 主温度传感器：MAX6675（用于控制）
- ✅ 第二温度传感器：MAX31865 + PT100（安全联锁）
- ✅ SSR 控制联锁：仅当第二传感器温度 `< 150°C` 时，第一传感器才允许控制 SSR
- ✅ OLED 实时显示（首页大字体温度 + 状态区）
- ✅ 菜单防误按：二级确认菜单
- ✅ Factory Reset：清空保存参数并恢复默认
- ✅ 参数持久化：目标温度/滞后/安全上下限保存到 NVS
- ✅ 重启策略：系统重启后 `SYS` 固定为 `ON`
- ✅ 屏幕保护：无操作自动息屏，支持在 Settings 开关并保存
- ✅ RGB LED 报警序列

## 📦 硬件需求

- nanoESP32-C6 v1.0
- MAX6675 + K 型热电偶
- MAX31865 + PT100
- SSD1306 OLED（I2C）
- SSR/继电器（加热输出）
- 旋转编码器（CLK/DT/SW）
- 两个按键（Menu / Back）

## 🔌 引脚分配（当前）

| GPIO | 功能 |
|------|------|
| 4 | MAX6675 SO |
| 5 | MAX6675 CS |
| 6 | MAX6675 SCK |
| 7 | SSR 输出 |
| 8 | 板载 WS2812 RGB |
| 10 | 编码器 CLK |
| 11 | 编码器 DT |
| 12 | Menu 键（打开弹出菜单） |
| 13 | Back 键 |
| 21 | 编码器 SW（确认/执行） |
| 22 | I2C SDA（OLED） |
| 23 | I2C SCL（OLED） |
| 9 | MAX31865 CS |
| 15 | MAX31865 MOSI |
| 4 | MAX31865 MISO（与 MAX6675 SO 共用） |
| 6 | MAX31865 SCK（与 MAX6675 SCK 共用） |

## 🧠 控制逻辑

1. 第一传感器 `T1`（MAX6675）负责温控滞后算法。
2. 第二传感器 `T2`（PT100）负责联锁：
   - `T2 >= 150°C` 或读取无效 → 强制关闭 SSR
   - `T2 < 150°C` 且有效 → 允许 T1 控制 SSR
3. 安全边界：当前温度超出 `[Low, High]` 会触发保护并关闭系统。

## 🎛️ 菜单操作（当前交互）

### 首页
- 显示大字体温度（当前/设定）与状态信息
- `Menu` 键（GPIO12）打开弹出菜单

### 弹出菜单
- 旋钮选择条目，`SW` 执行
- 包含：
  - `Set Target`
  - `SYS: ON/OFF`
  - `Settings`
  - `Factory Reset`

### 二级确认（防误按）
- 对 `SYS` 切换和 `Factory Reset` 进入确认页
- `SW: YES`，`BACK: NO`

### Settings
- 可调参数：
  - `Hys`
  - `Low`
  - `High`
  - `T2Mx`
  - `T2Hy`
  - `ScrSv`（屏保开关）
- `SW` 进入/退出编辑
- 参数会自动持久化到 NVS

## 💾 持久化与重启行为

保存到 NVS 的参数：
- `targetTemp`
- `tempHysteresis`
- `safetyLowerTemp`
- `safetyUpperTemp`
- `secondaryTempLimit`
- `secondaryTempHysteresis`
- `screenSaverEnabled`

重启后：
- 上述参数会恢复
- `systemEnabled` 强制设为 `ON`

## 🎨 LED 状态

- 高温报警：红闪两下 + 蓝闪一下（循环）
- 低温报警：蓝闪两下 + 红闪一下（循环）
- 传感器错误：紫色闪烁
- 加热中：橙色/红色状态指示（按当前逻辑）

## 🚀 构建与上传

```bash
pio run -e esp32-c6-devkitc-1
pio run -e esp32-c6-devkitc-1 -t upload
pio device monitor
```

## 📚 依赖库

由 PlatformIO 自动安装（见 `platformio.ini`）：
- adafruit/MAX6675 library
- adafruit/Adafruit MAX31865 library
- adafruit/Adafruit GFX Library
- adafruit/Adafruit SSD1306
- adafruit/Adafruit BusIO
- adafruit/Adafruit NeoPixel

## 🧩 关键配置文件

- `include/config.h`：引脚、阈值、间隔、默认参数
- `src/main.cpp`：菜单、温控、联锁、报警、持久化逻辑
- `platformio.ini`：板卡与库依赖

## 📄 License

MIT
