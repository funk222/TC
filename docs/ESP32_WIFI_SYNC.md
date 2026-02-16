# ESP32 WiFi 同步显示（副ESP32）

本项目主控已经新增同步接口：

- `GET /api/sync/status`
- 返回字段：`t1,target,t2Valid,t2,heaterOn,systemEnabled,safeLow,safeHigh,t2Limit,rssi,uptimeMs`

## 同一仓库切换编译目标

已在 `platformio.ini` 新增两个独立环境：

- `esp32-c6-devkitc-1`：主控固件（`src/main.cpp`）
- `esp8266-hw-364a`：副机固件（`src/receiver_main.cpp`）

命令行切换：

```bash
# 编译主控
pio run -e esp32-c6-devkitc-1

# 上传主控
pio run -e esp32-c6-devkitc-1 -t upload

# 编译副机
pio run -e esp8266-hw-364a

# 上传副机
pio run -e esp8266-hw-364a -t upload

# 监视串口（按目标切换）
pio device monitor -e esp32-c6-devkitc-1
pio device monitor -e esp8266-hw-364a
```

在 VS Code（PlatformIO 插件）中切换：

1. 打开左侧 PlatformIO 面板
2. 在 `PROJECT TASKS` 下选择对应环境
3. 点击该环境下的 `Build / Upload / Monitor`

## 1) 主控ESP32配置

编辑 `include/config.h`：

- `WIFI_SYNC_ENDPOINT_ENABLED`：是否启用同步接口
- `WIFI_SYNC_API_KEY`：同步密钥（可留空）

示例：

```cpp
#define WIFI_SYNC_ENDPOINT_ENABLED true
#define WIFI_SYNC_API_KEY "my-sync-key"
```

> 如果 `WIFI_SYNC_API_KEY` 为空，则副ESP32可直接访问同步接口（仅建议在可信局域网中使用）。

## 2) 副ESP32程序

参考文件：`docs/ESP32_WIFI_MIRROR_RECEIVER.ino`

需要修改以下参数：

- `WIFI_SSID`
- `WIFI_PASSWORD`
- `PRIMARY_IP`（主控ESP32的IP）
- `SYNC_KEY`（与主控 `WIFI_SYNC_API_KEY` 一致）

副ESP32会每秒拉取一次状态并显示在 SSD1306 OLED 上。

### HW-364A OLED 接线（你当前这块）

- SDA: D6 / GPIO14
- SCL: D5 / GPIO12
- I2C 地址: 0x3C

示例程序已按上述引脚与地址默认配置。

## 3) 快速验证

在同一局域网中，先确保主控在线，然后用浏览器测试：

- 无密钥：`http://<主控IP>/api/sync/status`
- 有密钥：`http://<主控IP>/api/sync/status?key=<你的密钥>`

能返回 JSON 即说明同步链路可用。
