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
#include <Adafruit_MAX31865.h>
#include <Preferences.h>
#include "max6675.h"
#include "config.h"

// 全局对象
MAX6675 thermocouple(MAXCLK, MAXCS, MAXDO);
Adafruit_MAX31865 pt100Sensor(PT100_CS, PT100_MOSI, PT100_MISO, PT100_SCK);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel rgbLED(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);
Preferences settingsStore;

const char* SETTINGS_NAMESPACE = "tempctrl";
const char* KEY_TARGET_TEMP = "target";
const char* KEY_HYSTERESIS = "hys";
const char* KEY_SAFE_LOW = "safe_low";
const char* KEY_SAFE_HIGH = "safe_high";
const char* KEY_T2_MAX = "t2_max";
const char* KEY_T2_HYS = "t2_hys";
const char* KEY_SCREEN_SAVER = "scr_sav";

// 温度控制变量
float currentTemp = 0.0;
float secondaryTemp = 0.0;
float targetTemp = DEFAULT_TARGET_TEMP;
float tempHysteresis = TEMP_HYSTERESIS;
float safetyLowerTemp = DEFAULT_SAFE_LOWER_TEMP;
float safetyUpperTemp = DEFAULT_SAFE_UPPER_TEMP;
float secondaryTempLimit = DEFAULT_SECOND_SENSOR_MAX_TEMP;
float secondaryTempHysteresis = SECOND_SENSOR_HYSTERESIS;
bool heaterOn = false;
bool systemEnabled = true;
bool secondarySensorValid = false;
bool secondaryInterlockBlocked = false;
bool safetyTripActive = false;
uint8_t pt100FaultCount = 0;
unsigned long lastPt100ValidMs = 0;
float lastPt100Temp = NAN;

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
bool screenSaverActive = false;
bool screenSaverEnabled = SCREEN_SAVER_ENABLED;
unsigned long lastUserActivityMs = 0;
int screenSaverEncoderPos = 0;

const int MAIN_TOTAL_LINES = 4;

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
  display.cp437(true);

  // 初始化 MAX31865 (PT100)
  if (PT100_WIRES == 2) {
    pt100Sensor.begin(MAX31865_2WIRE);
  } else if (PT100_WIRES == 4) {
    pt100Sensor.begin(MAX31865_4WIRE);
  } else {
    pt100Sensor.begin(MAX31865_3WIRE);
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
  Serial.printf("MAX31865: CS=%d, SCK=%d, MISO=%d, MOSI=%d\n", PT100_CS, PT100_SCK, PT100_MISO, PT100_MOSI);
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

  lastUserActivityMs = millis();
  screenSaverEncoderPos = encoderPos;
}

void loop() {
  unsigned long currentMillis = millis();

  if (!screenSaverEnabled && screenSaverActive) {
    screenSaverActive = false;
    display.ssd1306_command(SSD1306_DISPLAYON);
    updateDisplay();
  }

  if (screenSaverEnabled && !screenSaverActive) {
    if (currentMillis - lastUserActivityMs >= SCREEN_SAVER_TIMEOUT_MS) {
      screenSaverActive = true;
      display.ssd1306_command(SSD1306_DISPLAYOFF);
    }
  }

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
  bool wakeInput = (digitalRead(BTN_CONFIRM) == LOW)
                  || (digitalRead(BTN_BACK) == LOW)
                  || (ENCODER_SW >= 0 && digitalRead(ENCODER_SW) == LOW)
                  || (encoderPos != screenSaverEncoderPos);

  if (screenSaverActive) {
    if (wakeInput) {
      screenSaverActive = false;
      display.ssd1306_command(SSD1306_DISPLAYON);
      lastUserActivityMs = currentTime;
      screenSaverEncoderPos = encoderPos;
      lastBtnConfirmTime = currentTime;
      lastBtnBackTime = currentTime;
      lastBtnEncoderSwTime = currentTime;
      updateDisplay();
    }
    return;
  }

  if (wakeInput) {
    lastUserActivityMs = currentTime;
    screenSaverEncoderPos = encoderPos;
  }

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
          // ScrSv 项支持一键切换（无需进入编辑模式）
          if (!settingsEditing && settingsSelection == 5) {
            screenSaverEnabled = !screenSaverEnabled;
            saveSettings();
            #if ENABLE_SERIAL_DEBUG
            Serial.printf("屏保 %s\n", screenSaverEnabled ? "开启" : "关闭");
            #endif
            break;
          }

          // 其他项：确认键进入/退出编辑模式
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
    lastUserActivityMs = millis();
    screenSaverEncoderPos = encoderPos;

    switch (currentMenu) {
      case MENU_MAIN:
        // 首页滚屏
        mainScrollOffset += delta;
        if (mainScrollOffset < 0) mainScrollOffset = 0;
        {
          int totalHeight = 0;
          for (int lineIndex = 0; lineIndex < MAIN_TOTAL_LINES; lineIndex++) {
            totalHeight += (lineIndex <= 2) ? 16 : 8;
          }
          if (MAIN_TOTAL_LINES > 1) {
            totalHeight += 2; // separator under second large line
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
          } else if (settingsSelection == 3) {
            // 调节 T2 联锁上限
            secondaryTempLimit += delta * SECOND_SENSOR_MAX_STEP;
            if (secondaryTempLimit < MIN_SECOND_SENSOR_MAX_TEMP) secondaryTempLimit = MIN_SECOND_SENSOR_MAX_TEMP;
            if (secondaryTempLimit > MAX_SECOND_SENSOR_MAX_TEMP) secondaryTempLimit = MAX_SECOND_SENSOR_MAX_TEMP;
          } else if (settingsSelection == 4) {
            // 调节 T2 联锁回差
            secondaryTempHysteresis += delta * SECOND_SENSOR_HYSTERESIS_STEP;
            if (secondaryTempHysteresis < MIN_SECOND_SENSOR_HYSTERESIS) secondaryTempHysteresis = MIN_SECOND_SENSOR_HYSTERESIS;
            if (secondaryTempHysteresis > MAX_SECOND_SENSOR_HYSTERESIS) secondaryTempHysteresis = MAX_SECOND_SENSOR_HYSTERESIS;
          } else if (settingsSelection == 5) {
            // 屏幕保护开关
            if (delta != 0) screenSaverEnabled = !screenSaverEnabled;
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
          if (settingsSelection < 0) settingsSelection = 5;
          if (settingsSelection > 5) settingsSelection = 0;
        }
        break;
    }
  }
}

void readTemperature() {
  unsigned long now = millis();
  currentTemp = thermocouple.readCelsius();
  uint8_t pt100Fault = pt100Sensor.readFault();
  bool newPt100Valid = false;

  if (pt100Fault == 0) {
    float measuredPt100 = pt100Sensor.temperature(PT100_RNOMINAL, PT100_RREF);
    if (!isnan(measuredPt100)) {
      if (isnan(lastPt100Temp) || fabsf(measuredPt100 - lastPt100Temp) <= PT100_MAX_JUMP_PER_SAMPLE) {
        secondaryTemp = measuredPt100;
        lastPt100Temp = measuredPt100;
        lastPt100ValidMs = now;
        pt100FaultCount = 0;
        newPt100Valid = true;
      } else {
        // 单次突变过大，视为瞬时干扰
        pt100FaultCount++;
      }
    } else {
      pt100FaultCount++;
    }
  } else {
    pt100FaultCount++;
    pt100Sensor.clearFault();
  }

  if (newPt100Valid) {
    secondarySensorValid = true;
  } else {
    unsigned long validAge = now - lastPt100ValidMs;
    if (!isnan(lastPt100Temp) && validAge <= PT100_VALID_HOLD_MS && pt100FaultCount < PT100_FAULT_DEBOUNCE_COUNT) {
      // 接触抖动期间短时保留最后有效值
      secondaryTemp = lastPt100Temp;
      secondarySensorValid = true;
    } else {
      secondarySensorValid = false;
    }
  }
  
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
    safetyTripActive = true;
    #if ENABLE_SERIAL_DEBUG
    Serial.printf("!!! 安全保护激活 !!! 当前温度 %.2f°C 超出范围 [%.1f, %.1f]\n", currentTemp, safetyLowerTemp, safetyUpperTemp);
    #endif
  } else if (safetyTripActive) {
    // 仅对“安全保护触发导致的关机”自动恢复
    safetyTripActive = false;
    systemEnabled = true;
    #if ENABLE_SERIAL_DEBUG
    Serial.printf("*** 温度恢复安全区间，系统自动恢复开启 (%.2f°C 在 [%.1f, %.1f])\n", currentTemp, safetyLowerTemp, safetyUpperTemp);
    #endif
  }
  #endif
  
  #if ENABLE_SERIAL_DEBUG
  if (currentTemp > 0) {
    if (secondarySensorValid) {
      Serial.printf("T1: %.2f°C | T2: %.2f°C | T2Lim: %.1f°C | 目标: %.1f°C | Hys: %.1f | Safe:[%.1f,%.1f] | 加热: %s | 系统: %s\n", 
                    currentTemp, secondaryTemp, secondaryTempLimit,
                    targetTemp,
                    tempHysteresis, safetyLowerTemp, safetyUpperTemp,
                    heaterOn ? "ON " : "OFF", 
                    systemEnabled ? "ON " : "OFF");
    } else {
      Serial.printf("T1: %.2f°C | T2: ERROR(fc=%u) | T2Lim: %.1f°C | 目标: %.1f°C | Hys: %.1f | Safe:[%.1f,%.1f] | 加热: %s | 系统: %s\n", 
                  currentTemp, pt100FaultCount,
                  secondaryTempLimit,
                  targetTemp, 
                  tempHysteresis, safetyLowerTemp, safetyUpperTemp,
                  heaterOn ? "ON " : "OFF", 
                  systemEnabled ? "ON " : "OFF");
    }
  }
  #endif
}

void controlHeater() {
  // 第二传感器联锁（带回差）：
  // - T2 >= 上限时锁定切断
  // - 锁定后需降到 (上限 - 回差) 才解除
  // - T2 无效/报错时不参与控制，交由 T1 正常控制
  if (secondarySensorValid) {
    float t2ReleaseTemp = secondaryTempLimit - secondaryTempHysteresis;
    if (secondaryInterlockBlocked) {
      if (secondaryTemp <= t2ReleaseTemp) {
        secondaryInterlockBlocked = false;
      }
    } else if (secondaryTemp >= secondaryTempLimit) {
      secondaryInterlockBlocked = true;
    }

    if (secondaryInterlockBlocked) {
      digitalWrite(RELAY_PIN, LOW);
      heaterOn = false;
      return;
    }
  } else {
    secondaryInterlockBlocked = false;
  }

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
  if (secondaryTempHysteresis < MIN_SECOND_SENSOR_HYSTERESIS) secondaryTempHysteresis = MIN_SECOND_SENSOR_HYSTERESIS;
  if (secondaryTempHysteresis > MAX_SECOND_SENSOR_HYSTERESIS) secondaryTempHysteresis = MAX_SECOND_SENSOR_HYSTERESIS;
  if (secondaryTempLimit < MIN_SECOND_SENSOR_MAX_TEMP) secondaryTempLimit = MIN_SECOND_SENSOR_MAX_TEMP;
  if (secondaryTempLimit > MAX_SECOND_SENSOR_MAX_TEMP) secondaryTempLimit = MAX_SECOND_SENSOR_MAX_TEMP;

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
  secondaryTempLimit = settingsStore.getFloat(KEY_T2_MAX, DEFAULT_SECOND_SENSOR_MAX_TEMP);
  secondaryTempHysteresis = settingsStore.getFloat(KEY_T2_HYS, SECOND_SENSOR_HYSTERESIS);
  screenSaverEnabled = settingsStore.getBool(KEY_SCREEN_SAVER, SCREEN_SAVER_ENABLED);
  settingsStore.end();

  // 按需求：重启后系统始终为 ON
  systemEnabled = true;

  sanitizeRuntimeSettings();

  #if ENABLE_SERIAL_DEBUG
  Serial.printf("已加载设置: T=%.1f Hys=%.1f Low=%.1f High=%.1f T2Lim=%.1f T2Hys=%.1f ScrSav=%s Sys=%s\n",
                targetTemp,
                tempHysteresis,
                safetyLowerTemp,
                safetyUpperTemp,
                secondaryTempLimit,
                secondaryTempHysteresis,
                screenSaverEnabled ? "ON" : "OFF",
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
  settingsStore.putFloat(KEY_T2_MAX, secondaryTempLimit);
  settingsStore.putFloat(KEY_T2_HYS, secondaryTempHysteresis);
  settingsStore.putBool(KEY_SCREEN_SAVER, screenSaverEnabled);
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
  secondaryTempLimit = DEFAULT_SECOND_SENSOR_MAX_TEMP;
  secondaryTempHysteresis = SECOND_SENSOR_HYSTERESIS;
  screenSaverEnabled = SCREEN_SAVER_ENABLED;
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
      display.printf("%s Hys : %.1f\xF8" "C\n", settingsSelection == 0 ? ">" : " ", tempHysteresis);
      display.printf("%s Low : %.1f\xF8" "C\n", settingsSelection == 1 ? ">" : " ", safetyLowerTemp);
      display.printf("%s High: %.1f\xF8" "C\n", settingsSelection == 2 ? ">" : " ", safetyUpperTemp);
      display.printf("%s T2Mx: %.0f\xF8" "C\n", settingsSelection == 3 ? ">" : " ", secondaryTempLimit);
      display.printf("%s T2Hy: %.1f\xF8" "C\n", settingsSelection == 4 ? ">" : " ", secondaryTempHysteresis);
      display.printf("%s ScrSv: %s\n", settingsSelection == 5 ? ">" : " ", screenSaverEnabled ? "ON" : "OFF");
      display.println();
      display.println(settingsEditing ? F("Mode: EDIT") : F("Mode: SELECT"));
      display.println(settingsEditing ? F("Rotate: adjust") : F("Rotate: choose"));
      display.println(F("Press: edit/save"));
      break;
  }
  
  display.display();
}

void drawMainMenu() {
  const int boxW = 64;
  const int boxH = 26;
  const int bottomY = 52;

  auto drawCenteredValue = [&](const String& text, int boxX, int boxY, int textY) {
    int16_t x1, y1;
    uint16_t w, h;
    display.setTextSize(2);
    display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
    int textX = boxX + (boxW - (int)w) / 2;
    if (textX < boxX + 1) textX = boxX + 1;
    display.setCursor(textX, textY);
    display.print(text);
  };

  // 2x2 分栏边框
  display.drawRect(0, 0, boxW, boxH, SSD1306_WHITE);     // T1
  display.drawRect(64, 0, boxW, boxH, SSD1306_WHITE);    // T2
  display.drawRect(0, 26, boxW, boxH, SSD1306_WHITE);    // Heat
  display.drawRect(64, 26, boxW, boxH, SSD1306_WHITE);   // Sys
  display.drawRect(0, bottomY, 128, 12, SSD1306_WHITE);  // 安全信息条

  // T1（大字）
  display.setTextSize(1);
  display.setCursor(2, 1);
  display.print(F("T1"));
  drawCenteredValue(String(currentTemp, 1), 0, 0, 10);

  // T2（大字）
  display.setTextSize(1);
  display.setCursor(66, 1);
  display.print(F("T2"));
  drawCenteredValue(secondarySensorValid ? String(secondaryTemp, 1) : String("ERR"), 64, 0, 10);

  // Heat（大字）
  display.setTextSize(1);
  display.setCursor(2, 27);
  display.print(F("HEAT"));
  drawCenteredValue(heaterOn ? String("ON") : String("OFF"), 0, 26, 36);

  // Sys（大字）
  display.setTextSize(1);
  display.setCursor(66, 27);
  display.print(F("SYS"));
  drawCenteredValue(systemEnabled ? String("ON") : String("OFF"), 64, 26, 36);

  // 底部小字：T1设定 / Low / High / T2Max
  display.setTextSize(1);
  display.setCursor(2, 54);
  display.print(String("T1 ") + String(targetTemp, 1)
              + " L" + String(safetyLowerTemp, 0)
              + " H" + String(safetyUpperTemp, 0)
              + " M" + String(secondaryTempLimit, 0));
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
  display.println(F("\xF8" "C"));
  
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
  
  // 正常状态：呼吸灯（用于显示系统仍在运行）
  uint32_t baseColor;
  if (systemEnabled) {
    baseColor = heaterOn ? COLOR_HEATING : COLOR_COOLING;
  } else {
    baseColor = COLOR_NORMAL;
  }

  const uint16_t breathPeriodMs = 3000;
  const uint8_t breathMin = 10;
  const uint8_t breathMax = RGB_LED_BRIGHTNESS;
  uint16_t phase = currentMillis % breathPeriodMs;
  uint16_t halfPeriod = breathPeriodMs / 2;
  uint8_t breathLevel;
  if (phase < halfPeriod) {
    breathLevel = map(phase, 0, halfPeriod, breathMin, breathMax);
  } else {
    breathLevel = map(phase, halfPeriod, breathPeriodMs, breathMax, breathMin);
  }

  uint8_t r = (baseColor >> 16) & 0xFF;
  uint8_t g = (baseColor >> 8) & 0xFF;
  uint8_t b = baseColor & 0xFF;
  r = (uint16_t)r * breathLevel / 255;
  g = (uint16_t)g * breathLevel / 255;
  b = (uint16_t)b * breathLevel / 255;
  setLEDColor(((uint32_t)r << 16) | ((uint32_t)g << 8) | b);
}
