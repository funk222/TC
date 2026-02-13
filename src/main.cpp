/*
 * nanoESP32-C6 Temperature Controller
 * 
 * 硬件配置：
 * - MAX6675 热电偶传感器
 * - I2C OLED 显示屏 (SSD1306, 128x64)
 * - 旋转编码器 + 3个按键
 * - 继电器控制输出
 * - WS2812 RGB LED (板载GPIO 8)
 * 
 * 引脚分配：
 * - GPIO 4,5,6: MAX6675 (SO, CS, SCK)
 * - GPIO 7: 继电器输出
 * - GPIO 8: RGB LED (板载WS2812)
 * - GPIO 10,11: 编码器 (CLK, DT)
 * - GPIO 12,13: 独立按键 (确认, 返回)
 * - 编码器按键: 可选（默认未连接）
 * - GPIO 22,23: I2C (SDA, SCL)
 * 
 * Project: ESP32-C6 Temperature Controller
 * Date: 2026-02-12
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include "max6675.h"
#include "config.h"

// 全局对象
MAX6675 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel rgbLED(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences settingsStore;

const char* SETTINGS_NAMESPACE = "tempctrl";
const char* KEY_TARGET_TEMP = "target";
const char* KEY_HYSTERESIS = "hys";
const char* KEY_SAFE_LOW = "safe_low";
const char* KEY_SAFE_HIGH = "safe_high";

// 温度控制变量
float currentTemp = 0.0;
float targetTemp = DEFAULT_TARGET_TEMP;
float tempHysteresis = TEMP_HYSTERESIS;
float safetyLowerTemp = DEFAULT_SAFE_LOWER_TEMP;
float safetyUpperTemp = DEFAULT_SAFE_UPPER_TEMP;
bool heaterOn = false;
bool systemEnabled = true;

// 菜单系统
enum MenuState {
  MENU_MAIN,          // 首页状态显示
  MENU_POPUP,         // 弹出菜单
  MENU_CONFIRM_ACTION,// 二级确认菜单
  MENU_SET_TARGET,    // 设置目标温度
  MENU_SYSTEM_CONTROL,// 系统开关控制
  MENU_SETTINGS       // 设置菜单
};

enum PendingAction {
  ACTION_NONE,
  ACTION_TOGGLE_SYS,
  ACTION_FACTORY_RESET
};

MenuState currentMenu = MENU_MAIN;
int menuSelection = 0;
int settingsSelection = 0;
bool settingsEditing = false;
int mainScrollOffset = 0;
PendingAction pendingAction = ACTION_NONE;

const int MAIN_TOTAL_LINES = 5;

// 编码器变量
volatile int encoderPos = 0;
volatile int lastEncoded = 0;
unsigned long lastEncoderTime = 0;
const unsigned long debounceDelay = ENCODER_DEBOUNCE_DELAY;

// 按键防抖
unsigned long lastBtnConfirmTime = 0;
unsigned long lastBtnBackTime = 0;
unsigned long lastBtnEncoderSwTime = 0;
const unsigned long btnDebounceDelay = BUTTON_DEBOUNCE_DELAY;

// 更新间隔
unsigned long lastTempUpdate = 0;
const unsigned long tempUpdateInterval = TEMP_UPDATE_INTERVAL;
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = DISPLAY_UPDATE_INTERVAL;
unsigned long lastLEDUpdate = 0;
const unsigned long ledUpdateInterval = 100;  // LED更新间隔
unsigned long alarmBlinkTimer = 0;
bool alarmLEDState = false;
uint8_t alarmSequenceStep = 0;

enum AlarmMode {
  ALARM_NONE,
  ALARM_HIGH_TEMP,
  ALARM_LOW_TEMP,
  ALARM_SENSOR_ERROR
};

AlarmMode activeAlarmMode = ALARM_NONE;

// 函数声明
void updateEncoder();
void handleButtons();
void readTemperature();
void controlHeater();
void updateDisplay();
void drawMainMenu();
void drawPopupMenu();
void drawConfirmActionMenu();
void drawSetTargetMenu();
void drawSystemControlMenu();
void handleMenuNavigation();
void encoderISR();
void setLEDColor(uint32_t color);
void updateLEDStatus();
void sanitizeRuntimeSettings();
void loadSettings();
void saveSettings();
void factoryResetSettings();

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  #if ENABLE_SERIAL_DEBUG
  Serial.println("\n\n=================================");
  Serial.println("ESP32-C6 温度控制器启动中...");
  Serial.println("=================================");
  #endif

  loadSettings();

  // 初始化 I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  // 初始化显示屏
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    #if ENABLE_SERIAL_DEBUG
    Serial.println(F("错误：SSD1306 初始化失败！"));
    Serial.println(F("请检查 I2C 连接和地址"));
    #endif
    // 尝试备用地址
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
      for (;;); // 永久循环
    }
  }
  
  // 显示启动画面
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("ESP32-C6"));
  display.println(F("Temperature"));
  display.println(F("Controller"));
  display.println();
  display.println(F("Initializing..."));
  display.display();
  
  // 初始化引脚
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  if (ENCODER_SW >= 0) {
    pinMode(ENCODER_SW, INPUT_PULLUP);  // 编码器按键（备用）
  }
  pinMode(BTN_CONFIRM, INPUT_PULLUP); // 确认键（主要）
  pinMode(BTN_BACK, INPUT_PULLUP);    // 返回键
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // 初始化RGB LED
  rgbLED.begin();
  rgbLED.setBrightness(RGB_LED_BRIGHTNESS);
  rgbLED.show();  // 初始化为关闭
  setLEDColor(COLOR_NORMAL);  // 设置为绿色表示正常

  // 设置编码器中断
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);

  #if ENABLE_SERIAL_DEBUG
  Serial.println("引脚配置完成");
  Serial.printf("MAX6675: CS=%d, SCK=%d, SO=%d\n", MAXCS, MAXCLK, MAXDO);
  Serial.printf("I2C: SDA=%d, SCL=%d (避免与板载LED冲突)\n", I2C_SDA, I2C_SCL);
  if (ENCODER_SW >= 0) {
    Serial.printf("编码器: CLK=%d, DT=%d, SW=%d(备用)\n", ENCODER_CLK, ENCODER_DT, ENCODER_SW);
  } else {
    Serial.printf("编码器: CLK=%d, DT=%d, SW=未使用\n", ENCODER_CLK, ENCODER_DT);
  }
  Serial.printf("按键: CONFIRM=%d, BACK=%d\n", BTN_CONFIRM, BTN_BACK);
  Serial.printf("继电器: HEATER=%d\n", RELAY_PIN);
  Serial.printf("RGB LED: PIN=%d (板载WS2812)\n", RGB_LED_PIN);
  #endif

  // 等待 MAX6675 稳定
  delay(500);

  #if ENABLE_SERIAL_DEBUG
  Serial.println("初始化完成！");
  Serial.println("=================================\n");
  #endif
  
  delay(1000);
}

void loop() {
  unsigned long currentMillis = millis();

  // 定期读取温度
  if (currentMillis - lastTempUpdate >= tempUpdateInterval) {
    lastTempUpdate = currentMillis;
    readTemperature();
    
    // 根据温度控制加热器
    if (systemEnabled) {
      controlHeater();
    } else {
      digitalWrite(RELAY_PIN, LOW);
      heaterOn = false;
    }
  }

  // 处理用户输入
  handleButtons();
  handleMenuNavigation();

  // 更新显示
  if (currentMillis - lastDisplayUpdate >= displayUpdateInterval) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }

  // 更新RGB LED状态
  if (currentMillis - lastLEDUpdate >= ledUpdateInterval) {
    lastLEDUpdate = currentMillis;
    updateLEDStatus();
  }
}

// 编码器中断服务程序
void IRAM_ATTR encoderISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastEncoderTime < debounceDelay) return;
  lastEncoderTime = currentTime;

  int MSB = digitalRead(ENCODER_CLK);
  int LSB = digitalRead(ENCODER_DT);

  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  // 使用格雷码解码旋转方向
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }

  lastEncoded = encoded;
}

void handleButtons() {
  unsigned long currentTime = millis();

  // 菜单按键（原 CONFIRM 键）
  if (digitalRead(BTN_CONFIRM) == LOW) {
    if (currentTime - lastBtnConfirmTime > btnDebounceDelay) {
      lastBtnConfirmTime = currentTime;
      if (currentMenu == MENU_MAIN) {
        currentMenu = MENU_POPUP;
        menuSelection = 0;
      }
    }
  }

  // 编码器按键（确认/选择）
  if (ENCODER_SW >= 0 && digitalRead(ENCODER_SW) == LOW) {
    if (currentTime - lastBtnEncoderSwTime > btnDebounceDelay) {
      lastBtnEncoderSwTime = currentTime;

      switch (currentMenu) {
        case MENU_POPUP:
          // 在弹出菜单执行操作
          if (menuSelection == 0) currentMenu = MENU_SET_TARGET;
          else if (menuSelection == 1) {
            pendingAction = ACTION_TOGGLE_SYS;
            currentMenu = MENU_CONFIRM_ACTION;
          }
          else if (menuSelection == 2) {
            currentMenu = MENU_SETTINGS;
            settingsSelection = 0;
            settingsEditing = false;
          }
          else if (menuSelection == 3) {
            pendingAction = ACTION_FACTORY_RESET;
            currentMenu = MENU_CONFIRM_ACTION;
          }
          break;

        case MENU_CONFIRM_ACTION:
          if (pendingAction == ACTION_TOGGLE_SYS) {
            systemEnabled = !systemEnabled;
            saveSettings();
            #if ENABLE_SERIAL_DEBUG
            Serial.printf("系统 %s\n", systemEnabled ? "开启" : "关闭");
            #endif
            currentMenu = MENU_POPUP;
          } else if (pendingAction == ACTION_FACTORY_RESET) {
            factoryResetSettings();
            #if ENABLE_SERIAL_DEBUG
            Serial.println("已恢复出厂设置");
            #endif
            currentMenu = MENU_MAIN;
          }
          pendingAction = ACTION_NONE;
          break;

        case MENU_SET_TARGET:
          // 确认目标温度
          saveSettings();
          currentMenu = MENU_MAIN;
          menuSelection = 0;
          #if ENABLE_SERIAL_DEBUG
          Serial.printf("目标温度设定为: %.1f°C\n", targetTemp);
          #endif
          break;

        case MENU_SYSTEM_CONTROL:
          // 切换系统开关
          systemEnabled = !systemEnabled;
          #if ENABLE_SERIAL_DEBUG
          Serial.printf("系统 %s\n", systemEnabled ? "开启" : "关闭");
          #endif
          break;

        case MENU_SETTINGS:
          // 确认键：进入/退出编辑模式
          settingsEditing = !settingsEditing;
          if (!settingsEditing) {
            saveSettings();
          }
          #if ENABLE_SERIAL_DEBUG
          if (!settingsEditing) {
            Serial.printf("设置已更新: Hys=%.1f°C, Low=%.1f°C, High=%.1f°C\n", tempHysteresis, safetyLowerTemp, safetyUpperTemp);
          }
          #endif
          break;

        default:
          break;
      }
    }
  }

  // 返回按钮
  if (digitalRead(BTN_BACK) == LOW) {
    if (currentTime - lastBtnBackTime > btnDebounceDelay) {
      lastBtnBackTime = currentTime;
      
      if (currentMenu != MENU_MAIN) {
        if (currentMenu == MENU_SETTINGS && settingsEditing) {
          settingsEditing = false;
          saveSettings();
        } else if (currentMenu == MENU_CONFIRM_ACTION) {
          pendingAction = ACTION_NONE;
          currentMenu = MENU_POPUP;
        } else if (currentMenu == MENU_POPUP) {
          currentMenu = MENU_MAIN;
        } else {
          currentMenu = MENU_POPUP;
        }
        #if ENABLE_SERIAL_DEBUG
        Serial.println("返回上级菜单");
        #endif
      }
    }
  }
}

void handleMenuNavigation() {
  static int lastEncoderPos = 0;
  
  if (encoderPos != lastEncoderPos) {
    int delta = encoderPos - lastEncoderPos;
    lastEncoderPos = encoderPos;

    switch (currentMenu) {
      case MENU_MAIN:
        // 首页滚屏
        mainScrollOffset += delta;
        if (mainScrollOffset < 0) mainScrollOffset = 0;
        {
          int totalHeight = 0;
          for (int lineIndex = 0; lineIndex < MAIN_TOTAL_LINES; lineIndex++) {
            totalHeight += (lineIndex == 0) ? 16 : 8;
          }

          int maxScroll = 0;
          if (totalHeight > SCREEN_HEIGHT) {
            maxScroll = MAIN_TOTAL_LINES - 1;
          }
          if (maxScroll < 0) maxScroll = 0;
          if (mainScrollOffset > maxScroll) mainScrollOffset = maxScroll;
        }
        break;

      case MENU_POPUP:
        // 弹出菜单导航
        menuSelection += delta;
        if (menuSelection < 0) menuSelection = 3;
        if (menuSelection > 3) menuSelection = 0;
        break;
        
      case MENU_SET_TARGET:
        // 调节目标温度
        targetTemp += delta * TEMP_ADJUST_STEP;
        {
          float targetMin = (safetyLowerTemp > MIN_TEMP) ? safetyLowerTemp : MIN_TEMP;
          float targetMax = (safetyUpperTemp < MAX_TEMP) ? safetyUpperTemp : MAX_TEMP;
          if (targetTemp < targetMin) targetTemp = targetMin;
          if (targetTemp > targetMax) targetTemp = targetMax;
        }
        break;
        
      case MENU_SYSTEM_CONTROL:
        // 保留给未来扩展
        break;
        
      case MENU_SETTINGS:
        if (settingsEditing) {
          if (settingsSelection == 0) {
            // 调节滞后参数
            tempHysteresis += delta * HYSTERESIS_ADJUST_STEP;
            if (tempHysteresis < MIN_HYSTERESIS) tempHysteresis = MIN_HYSTERESIS;
            if (tempHysteresis > MAX_HYSTERESIS) tempHysteresis = MAX_HYSTERESIS;
          } else if (settingsSelection == 1) {
            // 调节安全下限
            safetyLowerTemp += delta * SAFETY_BOUNDARY_STEP;
            if (safetyLowerTemp < SAFETY_BOUNDARY_MIN) safetyLowerTemp = SAFETY_BOUNDARY_MIN;
            if (safetyLowerTemp > safetyUpperTemp - MIN_SAFE_RANGE_GAP) {
              safetyLowerTemp = safetyUpperTemp - MIN_SAFE_RANGE_GAP;
            }
          } else if (settingsSelection == 2) {
            // 调节安全上限
            safetyUpperTemp += delta * SAFETY_BOUNDARY_STEP;
            if (safetyUpperTemp > SAFETY_BOUNDARY_MAX) safetyUpperTemp = SAFETY_BOUNDARY_MAX;
            if (safetyUpperTemp < safetyLowerTemp + MIN_SAFE_RANGE_GAP) {
              safetyUpperTemp = safetyLowerTemp + MIN_SAFE_RANGE_GAP;
            }
          }

          // 目标温度始终限制在安全范围内
          {
            float targetMin = (safetyLowerTemp > MIN_TEMP) ? safetyLowerTemp : MIN_TEMP;
            float targetMax = (safetyUpperTemp < MAX_TEMP) ? safetyUpperTemp : MAX_TEMP;
            if (targetTemp < targetMin) targetTemp = targetMin;
            if (targetTemp > targetMax) targetTemp = targetMax;
          }
        } else {
          // 未编辑时，旋钮切换设置项
          settingsSelection += delta;
          if (settingsSelection < 0) settingsSelection = 2;
          if (settingsSelection > 2) settingsSelection = 0;
        }
        break;
    }
  }
}

void readTemperature() {
  currentTemp = thermocouple.readCelsius();
  
  if (isnan(currentTemp)) {
    #if ENABLE_SERIAL_DEBUG
    Serial.println("错误：热电偶读取失败！");
    #endif
    currentTemp = 0.0;
  }
  
  #if ENABLE_OVERHEAT_PROTECTION
  // 安全边界保护
  if (currentTemp > safetyUpperTemp || currentTemp < safetyLowerTemp) {
    digitalWrite(RELAY_PIN, LOW);
    heaterOn = false;
    systemEnabled = false;
    #if ENABLE_SERIAL_DEBUG
    Serial.printf("!!! 安全保护激活 !!! 当前温度 %.2f°C 超出范围 [%.1f, %.1f]\n", currentTemp, safetyLowerTemp, safetyUpperTemp);
    #endif
  }
  #endif
  
  #if ENABLE_SERIAL_DEBUG
  if (currentTemp > 0) {
    Serial.printf("温度: %.2f°C | 目标: %.1f°C | Hys: %.1f | Safe:[%.1f,%.1f] | 加热: %s | 系统: %s\n", 
                  currentTemp, targetTemp, 
                  tempHysteresis, safetyLowerTemp, safetyUpperTemp,
                  heaterOn ? "ON " : "OFF", 
                  systemEnabled ? "ON " : "OFF");
  }
  #endif
}

void controlHeater() {
  // 简单的滞后控制算法
  if (currentTemp < targetTemp - tempHysteresis) {
    // 温度低于下限，开启加热
    digitalWrite(RELAY_PIN, HIGH);
    heaterOn = true;
  } else if (currentTemp > targetTemp + tempHysteresis) {
    // 温度高于上限，关闭加热
    digitalWrite(RELAY_PIN, LOW);
    heaterOn = false;
  }
  // 在滞后带内保持当前状态
}

void sanitizeRuntimeSettings() {
  if (tempHysteresis < MIN_HYSTERESIS) tempHysteresis = MIN_HYSTERESIS;
  if (tempHysteresis > MAX_HYSTERESIS) tempHysteresis = MAX_HYSTERESIS;

  if (safetyLowerTemp < SAFETY_BOUNDARY_MIN) safetyLowerTemp = SAFETY_BOUNDARY_MIN;
  if (safetyLowerTemp > SAFETY_BOUNDARY_MAX) safetyLowerTemp = SAFETY_BOUNDARY_MAX;
  if (safetyUpperTemp < SAFETY_BOUNDARY_MIN) safetyUpperTemp = SAFETY_BOUNDARY_MIN;
  if (safetyUpperTemp > SAFETY_BOUNDARY_MAX) safetyUpperTemp = SAFETY_BOUNDARY_MAX;

  if (safetyUpperTemp < safetyLowerTemp + MIN_SAFE_RANGE_GAP) {
    safetyUpperTemp = safetyLowerTemp + MIN_SAFE_RANGE_GAP;
    if (safetyUpperTemp > SAFETY_BOUNDARY_MAX) {
      safetyUpperTemp = SAFETY_BOUNDARY_MAX;
      safetyLowerTemp = safetyUpperTemp - MIN_SAFE_RANGE_GAP;
      if (safetyLowerTemp < SAFETY_BOUNDARY_MIN) {
        safetyLowerTemp = SAFETY_BOUNDARY_MIN;
      }
    }
  }

  float targetMin = (safetyLowerTemp > MIN_TEMP) ? safetyLowerTemp : MIN_TEMP;
  float targetMax = (safetyUpperTemp < MAX_TEMP) ? safetyUpperTemp : MAX_TEMP;
  if (targetTemp < targetMin) targetTemp = targetMin;
  if (targetTemp > targetMax) targetTemp = targetMax;
}

void loadSettings() {
  settingsStore.begin(SETTINGS_NAMESPACE, true);
  targetTemp = settingsStore.getFloat(KEY_TARGET_TEMP, DEFAULT_TARGET_TEMP);
  tempHysteresis = settingsStore.getFloat(KEY_HYSTERESIS, TEMP_HYSTERESIS);
  safetyLowerTemp = settingsStore.getFloat(KEY_SAFE_LOW, DEFAULT_SAFE_LOWER_TEMP);
  safetyUpperTemp = settingsStore.getFloat(KEY_SAFE_HIGH, DEFAULT_SAFE_UPPER_TEMP);
  settingsStore.end();

  // 按需求：重启后系统始终为 ON
  systemEnabled = true;

  sanitizeRuntimeSettings();

  #if ENABLE_SERIAL_DEBUG
  Serial.printf("已加载设置: T=%.1f Hys=%.1f Low=%.1f High=%.1f Sys=%s\n",
                targetTemp,
                tempHysteresis,
                safetyLowerTemp,
                safetyUpperTemp,
                systemEnabled ? "ON" : "OFF");
  #endif
}

void saveSettings() {
  sanitizeRuntimeSettings();

  settingsStore.begin(SETTINGS_NAMESPACE, false);
  settingsStore.putFloat(KEY_TARGET_TEMP, targetTemp);
  settingsStore.putFloat(KEY_HYSTERESIS, tempHysteresis);
  settingsStore.putFloat(KEY_SAFE_LOW, safetyLowerTemp);
  settingsStore.putFloat(KEY_SAFE_HIGH, safetyUpperTemp);
  settingsStore.end();
}

void factoryResetSettings() {
  settingsStore.begin(SETTINGS_NAMESPACE, false);
  settingsStore.clear();
  settingsStore.end();

  targetTemp = DEFAULT_TARGET_TEMP;
  tempHysteresis = TEMP_HYSTERESIS;
  safetyLowerTemp = DEFAULT_SAFE_LOWER_TEMP;
  safetyUpperTemp = DEFAULT_SAFE_UPPER_TEMP;
  systemEnabled = true;

  settingsSelection = 0;
  settingsEditing = false;
  menuSelection = 0;
  mainScrollOffset = 0;

  sanitizeRuntimeSettings();
  saveSettings();
}

void updateDisplay() {
  display.clearDisplay();
  
  switch (currentMenu) {
    case MENU_MAIN:
      drawMainMenu();
      break;
    case MENU_POPUP:
      drawPopupMenu();
      break;
    case MENU_CONFIRM_ACTION:
      drawConfirmActionMenu();
      break;
    case MENU_SET_TARGET:
      drawSetTargetMenu();
      break;
    case MENU_SYSTEM_CONTROL:
      drawSystemControlMenu();
      break;
    case MENU_SETTINGS:
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.println(F("=== SETTINGS ==="));
      display.printf("%s Hys : %.1fC\n", settingsSelection == 0 ? ">" : " ", tempHysteresis);
      display.printf("%s Low : %.1fC\n", settingsSelection == 1 ? ">" : " ", safetyLowerTemp);
      display.printf("%s High: %.1fC\n", settingsSelection == 2 ? ">" : " ", safetyUpperTemp);
      display.println();
      display.println(settingsEditing ? F("Mode: EDIT") : F("Mode: SELECT"));
      display.println(settingsEditing ? F("Rotate: adjust") : F("Rotate: choose"));
      display.println(F("Press: edit/save"));
      break;
  }
  
  display.display();
}

void drawMainMenu() {
  String lines[MAIN_TOTAL_LINES];
  lines[0] = String(currentTemp, 1) + "/" + String(targetTemp, 1);
  lines[1] = String("Low : ") + String(safetyLowerTemp, 1) + "C";
  lines[2] = String("Hi: ") + String(safetyUpperTemp, 1) + "C";
  lines[3] = String("Heat:") + (heaterOn ? "ON" : "OFF") + " Sys:" + (systemEnabled ? "ON" : "OFF");
  lines[4] = "Press MENU for popup";

  int y = 0;
  for (int contentIndex = mainScrollOffset; contentIndex < MAIN_TOTAL_LINES; contentIndex++) {
    if (contentIndex == 0) {
      display.setTextSize(2);
      if (y + 16 > SCREEN_HEIGHT) break;
      display.setCursor(0, y);
      display.print(F(""));
      display.println(lines[contentIndex]);
      display.drawLine(0, y + 16, SCREEN_WIDTH - 1, y + 16, SSD1306_WHITE);
      y += 18;
    } else {
      display.setTextSize(1);
      if (y + 8 > SCREEN_HEIGHT) break;
      display.setCursor(0, y);
      display.println(lines[contentIndex]);
      y += 8;
    }
  }
}

void drawPopupMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("=== MENU ==="));
  display.println();
  display.println(menuSelection == 0 ? "> Set Target" : "  Set Target");
  display.println(menuSelection == 1
                    ? (systemEnabled ? "> SYS: ON" : "> SYS: OFF")
                    : (systemEnabled ? "  SYS: ON" : "  SYS: OFF"));
  display.println(menuSelection == 2 ? "> Settings" : "  Settings");
  display.println(menuSelection == 3 ? "> Factory Reset" : "  Factory Reset");
  display.println();
  display.println(F("SW: Select/Toggle"));
  display.println(F("BACK: Close"));
}

void drawConfirmActionMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("=== CONFIRM ==="));
  display.println();

  if (pendingAction == ACTION_TOGGLE_SYS) {
    display.println(systemEnabled ? F("Turn SYS OFF ?") : F("Turn SYS ON ?"));
  } else if (pendingAction == ACTION_FACTORY_RESET) {
    display.println(F("Factory Reset ?"));
    display.println(F("(clear saved cfg)"));
  }

  display.println();
  display.println(F("SW: YES"));
  display.println(F("BACK: NO"));
}

void drawSetTargetMenu() {
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(F("TARGET"));
  
  display.println();
  display.setTextSize(3);
  display.printf("%.1f", targetTemp);
  display.setTextSize(2);
  display.println(F("C"));
  
  display.setTextSize(1);
  display.println();
  display.println(F("Rotate: adjust"));
  display.println(F("Press SW: confirm"));
}

void drawSystemControlMenu() {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("== SYS CTRL =="));
  display.println();
  
  display.setTextSize(2);
  display.printf("Curr: %.1fC\n", currentTemp);
  display.printf("Targ: %.1fC\n", targetTemp);
  
  display.setTextSize(2);
  display.println();
  display.printf("SYS: %s\n", systemEnabled ? " ON" : "OFF");
  
  display.setTextSize(1);
  display.println(F("Press SW"));
  display.println(F("to toggle SYS"));
}

// ===================================
// RGB LED 控制函数
// ===================================

/**
 * 设置RGB LED颜色
 * @param color 24位RGB颜色值 (0xRRGGBB)
 */
void setLEDColor(uint32_t color) {
  rgbLED.setPixelColor(0, color);
  rgbLED.show();
}

/**
 * 根据系统状态更新RGB LED
 * 颜色逻辑：
 * - 红色闪烁: 过热报警 (Temp > MAX_SAFE_TEMP)
 * - 紫色闪烁: 传感器错误或读数异常
 * - 红色: 加热中 (Heater ON)
 * - 橙色: 系统开启，但未加热 (Standby)
 * - 绿色: 系统关闭，正常状态
 */
void updateLEDStatus() {
  unsigned long currentMillis = millis();

  AlarmMode desiredAlarmMode = ALARM_NONE;
  if (currentTemp > safetyUpperTemp) {
    desiredAlarmMode = ALARM_HIGH_TEMP;
  } else if (currentTemp < safetyLowerTemp) {
    desiredAlarmMode = ALARM_LOW_TEMP;
  } else if (isnan(currentTemp) || currentTemp < -50) {
    desiredAlarmMode = ALARM_SENSOR_ERROR;
  }

  if (desiredAlarmMode != activeAlarmMode) {
    activeAlarmMode = desiredAlarmMode;
    alarmSequenceStep = 0;
    alarmLEDState = false;
    alarmBlinkTimer = currentMillis;
  }
  
  // 温度报警序列
  // 低温: 蓝闪两下 + 红闪一下
  // 高温: 红闪两下 + 蓝闪一下
  if (activeAlarmMode == ALARM_HIGH_TEMP || activeAlarmMode == ALARM_LOW_TEMP) {
    if (currentMillis - alarmBlinkTimer >= ALARM_BLINK_INTERVAL) {
      alarmBlinkTimer = currentMillis;
      alarmSequenceStep = (alarmSequenceStep + 1) % 6;
    }

    if (alarmSequenceStep % 2 == 1) {
      setLEDColor(COLOR_OFF);
    } else {
      uint8_t flashIndex = alarmSequenceStep / 2;
      if (activeAlarmMode == ALARM_HIGH_TEMP) {
        setLEDColor(flashIndex < 2 ? COLOR_OVERHEAT : COLOR_UNDERHEAT);
      } else {
        setLEDColor(flashIndex < 2 ? COLOR_UNDERHEAT : COLOR_OVERHEAT);
      }
    }

    return;
  }

  // 传感器错误闪烁
  if (activeAlarmMode == ALARM_SENSOR_ERROR) {
    if (currentMillis - alarmBlinkTimer >= ALARM_BLINK_INTERVAL) {
      alarmBlinkTimer = currentMillis;
      alarmLEDState = !alarmLEDState;
      setLEDColor(alarmLEDState ? COLOR_ERROR : COLOR_OFF);
    }
    return;
  }
  
  // 正常状态显示
  if (systemEnabled) {
    if (heaterOn) {
      // 加热中 - 红色
      setLEDColor(COLOR_HEATING);
    } else {
      // 待机 - 橙色
      setLEDColor(COLOR_COOLING);
    }
  } else {
    // 系统关闭 - 绿色
    setLEDColor(COLOR_NORMAL);
  }
}
