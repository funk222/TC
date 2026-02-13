# ESP32-C6 Temperature Controller

基于 nanoESP32-C6 v1.0 的温度控制器项目

## 🎯 项目特点

- ✅ **精确温控** - MAX6675热电偶，0-1024°C测量范围
- ✅ **直观显示** - I2C OLED (128x64) 实时显示温度和状态
- ✅ **便捷操作** - 旋转编码器+2个独立按键（编码器SW可选）
- ✅ **智能报警** - WS2812 RGB LED颜色指示系统状态
- ✅ **安全保护** - 过温保护，自动断电
- ✅ **滞后控制** - ±2°C滞后带，避免频繁开关

## 📦 硬件需求

### 主控板
- **nanoESP32-C6 v1.0** (ESP32-C6)
- 板载WS2812 RGB LED (GPIO 8)

### 传感器和执行器
- MAX6675 热电偶模块 (K型)
- SSD1306 OLED显示屏 (128x64, I2C)
- 继电器模块 (5V/3.3V)

### 输入设备
- 旋转编码器 (KY-040或类似，SW按键可不接)
- 2个独立按键 (确认+返回)

## 🔌 引脚连接

| GPIO | 功能 | 说明 |
|------|------|------|
| 4 | MAX6675 SO | 数据输出 |
| 5 | MAX6675 CS | 片选 |
| 6 | MAX6675 SCK | 时钟 |
| 7 | 继电器 IN | 控制输出 |
| **8** | **RGB LED** | **板载WS2812** ⭐ |
| 10 | 编码器 CLK | 旋转检测A |
| 11 | 编码器 DT | 旋转检测B |
| 12 | 确认按键 | 主要操作键 |
| 13 | 返回按键 | 返回/取消 |
| - | 编码器 SW | 可选按键（默认不接） |
| **22** | **I2C SDA** | OLED数据 ⚠️ |
| **23** | **I2C SCL** | OLED时钟 ⚠️ |

⚠️ **重要提示**：I2C引脚已从GPIO 8/9更改为GPIO 22/23，避免与板载RGB LED冲突。

## 🚀 快速开始

### 1. 安装PlatformIO

- VS Code + PlatformIO Extension
- 或使用PlatformIO CLI

### 2. 克隆/打开项目

```bash
cd D:\ESP32-C6-TempController
code .
```

### 3. 编译项目

```bash
pio run -e esp32-c6-devkitc-1
```

### 4. 上传固件

```bash
pio run -e esp32-c6-devkitc-1 -t upload
```

### 5. 查看串口输出

```bash
pio device monitor
```

## 📋 依赖库

所有库已在 `platformio.ini` 中配置，PlatformIO会自动下载：

- Adafruit MAX6675 library @ ^1.2.0
- Adafruit GFX Library @ ^1.11.3
- Adafruit SSD1306 @ ^2.5.7
- Adafruit BusIO @ ^1.14.1
- Adafruit NeoPixel @ ^1.10.7

## 🎮 操作说明

### 主菜单
- **旋转编码器** - 选择菜单项
- **确认键** - 进入选中的菜单
- **返回键** - 返回上级菜单

### 设置目标温度
1. 主菜单选择 "Set Target"
2. 按确认键进入
3. 旋转编码器调整温度 (±0.5°C/步)
4. 按确认键保存

### 系统控制
1. 主菜单选择 "System Ctrl"
2. 按确认键切换系统开/关

## 🎨 LED状态指示

| 状态 | 颜色 | 说明 |
|------|------|------|
| 正常待机 | 🟢 绿色 | 系统关闭 |
| 加热中 | 🔴 红色 | 加热器工作 |
| 保温待机 | 🟠 橙色 | 系统开启未加热 |
| 过热报警 | 🔴 红闪 | 温度>350°C |
| 传感器错误 | 💜 紫闪 | 读数异常 |

## 🔧 硬件测试

项目包含完整的硬件测试程序：

### 运行测试
1. 重命名文件：
   ```
   src/main.cpp → src/main.cpp.bak
   src/test_hardware.cpp.disabled → src/main.cpp
   ```
2. 编译上传
3. 查看串口输出和OLED显示

### 测试项目
- ✓ 串口通信
- ✓ OLED显示
- ✓ MAX6675温度传感器
- ✓ 旋转编码器
- ✓ 2个按键（编码器SW可选）
- ✓ 继电器输出
- ✓ RGB LED

## 📁 项目结构

```
ESP32-C6-TempController/
├── platformio.ini              # PlatformIO配置
├── include/
│   └── config.h                # 引脚和参数配置
├── src/
│   ├── main.cpp                # 主程序
│   └── test_hardware.cpp.disabled  # 硬件测试程序
├── docs/                       # 文档目录
└── README.md                   # 本文件
```

## ⚙️ 配置参数

在 `include/config.h` 中可调整：

```cpp
// 温度控制
#define DEFAULT_TARGET_TEMP  25.0   // 默认目标温度
#define TEMP_HYSTERESIS      2.0    // 控制滞后带
#define MAX_TEMP             300.0  // 最大设定温度
#define MAX_SAFE_TEMP        350.0  // 安全保护温度

// RGB LED
#define RGB_LED_BRIGHTNESS  50      // 亮度 (0-255)

// 更新间隔
#define TEMP_UPDATE_INTERVAL     500   // 温度读取间隔(ms)
#define DISPLAY_UPDATE_INTERVAL  100   // 显示刷新间隔(ms)
```

## 🐛 故障排除

### OLED不显示
- 检查I2C接线 (SDA=GPIO22, SCL=GPIO23)
- 尝试更改地址：`config.h` 中 `SCREEN_ADDRESS` (0x3C ↔ 0x3D)

### MAX6675读取失败
- 检查接线 (SO=4, CS=5, SCK=6)
- 确认热电偶极性
- 等待传感器稳定 (~1秒)

### RGB LED不亮
- 板载LED在GPIO 8，无需外接
- 检查固件是否正确编译
- 查看串口输出确认初始化

### 编译错误
- 确保PlatformIO已安装
- 运行 `pio pkg install` 更新库
- 清理项目：`pio run -t clean`

## 📚  相关链接

- [nanoESP32-C6官方仓库](https://github.com/wuxx/nanoESP32-C6)
- [ESP32-C6官方文档](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/)
- [PlatformIO文档](https://docs.platformio.org/)

## 📝 版本历史

### v1.2 - 2026-02-12
- ✅ 修正RGB LED到GPIO 8（板载）
- ✅ I2C更改到GPIO 22/23
- ✅ 添加RGB LED报警功能
- ✅ 完整硬件测试程序

### v1.1 - 2026-02-12
- ✅ 3个按键支持（2主要+1可选）
- ✅ 改进菜单导航
- ✅ 添加系统开关控制

### v1.0 - 初始版本
- ✅ 基础温度控制功能
- ✅ OLED显示界面
- ✅ 旋转编码器输入

## 📄 许可证

MIT License - 自由使用和修改

## 🤝 贡献

欢迎提交Issue和Pull Request！

---

**项目路径**: `D:\ESP32-C6-TempController`  
**最后更新**: 2026-02-12  
**硬件版本**: nanoESP32-C6 v1.0
