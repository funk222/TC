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
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <time.h>
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
WebServer webServer(WEB_DASHBOARD_PORT);
const char* LOG_FILE_PATH = "/status.csv";
const char* LOG_OLD_FILE_PATH = "/status.old.csv";
const char* LOG_CSV_HEADER = "date,time,T1,T1set,T2,Heat";

const char* SETTINGS_NAMESPACE = "tempctrl";
const char* KEY_TARGET_TEMP = "target";
const char* KEY_HYSTERESIS = "hys";
const char* KEY_SAFE_LOW = "safe_low";
const char* KEY_SAFE_HIGH = "safe_high";
const char* KEY_T2_MAX = "t2_max";
const char* KEY_T2_HYS = "t2_hys";
const char* KEY_SCREEN_SAVER = "scr_sav";
const char* KEY_SCREEN_SAVER_TIMEOUT = "scr_to";
const char* KEY_WEB_PASSWORD = "web_pwd";

String webPassword = "";

struct AuthClientState {
  String ip;
  uint8_t failCount;
  unsigned long blockedUntilMs;
  bool loggedIn;
};

const int MAX_AUTH_CLIENTS = 8;
AuthClientState authClients[MAX_AUTH_CLIENTS];

// 温度控制变量
float currentTemp = 0.0;
float currentTempRaw = 0.0;
float secondaryTemp = 0.0;
float secondaryTempRaw = 0.0;
float targetTemp = DEFAULT_TARGET_TEMP;
float tempHysteresis = TEMP_HYSTERESIS;
float safetyLowerTemp = DEFAULT_SAFE_LOWER_TEMP;
float safetyUpperTemp = DEFAULT_SAFE_UPPER_TEMP;
float secondaryTempLimit = DEFAULT_SECOND_SENSOR_MAX_TEMP;
float secondaryTempHysteresis = SECOND_SENSOR_HYSTERESIS;
bool heaterOn = false;
bool systemEnabled = true;
bool secondarySensorValid = false;
bool safetyTripActive = false;
uint8_t pt100FaultCount = 0;
unsigned long lastPt100ValidMs = 0;
float lastPt100Temp = NAN;
bool t1FilterInitialized = false;
bool t2FilterInitialized = false;

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
uint32_t screenSaverTimeoutMs = DEFAULT_SCREEN_SAVER_TIMEOUT_MS;
unsigned long lastUserActivityMs = 0;
int screenSaverEncoderPos = 0;
unsigned long lastWebLogWriteMs = 0;

const int MAIN_TOTAL_LINES = 4;

// 编码器变量
volatile int encoderPos = 0;
volatile uint8_t lastEncoded = 0;
volatile int8_t encoderTransitionAcc = 0;
volatile unsigned long lastEncoderTimeUs = 0;

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
const uint32_t screenSaverTimeoutMinMinutes = MIN_SCREEN_SAVER_TIMEOUT_MS / 60000UL;
const uint32_t screenSaverTimeoutMaxMinutes = MAX_SCREEN_SAVER_TIMEOUT_MS / 60000UL;
const uint32_t screenSaverTimeoutStepMinutes = SCREEN_SAVER_TIMEOUT_STEP_MS / 60000UL;

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
void initWiFi();
void setupWebServer();
void handleWebRoot();
void handleWebStatus();
void handleWebControl();
void handleWebLogDownload();
void handleWebLogClear();
void handleWebAuthState();
void handleWebAuthSetup();
void handleWebAuthLogin();
void handleWebAuthLogout();
void handleWebSettings();
void handleWebSettingsUpdate();
AuthClientState* getAuthClientState(const String& ip, bool createIfMissing);
String getWebClientIp();
bool isWebClientBlocked(const String& ip, unsigned long* remainingMs = nullptr);
bool isWebClientLoggedIn(const String& ip);
void markWebLoginFailure(const String& ip);
void markWebLoginSuccess(const String& ip);
void markWebLogout(const String& ip);
bool ensureAuthenticated();
String jsonEscape(const String& input);
void rotateLogsIfNeeded();
void formatLogDateAndTime(String& datePart, String& timePart);
void appendLogLine();
void appendPeriodicStatusLog();
void migrateLegacyLogFormatIfNeeded();

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  #if ENABLE_SERIAL_DEBUG
  Serial.println("\n\n=================================");
  Serial.println("ESP32-C6 温度控制器启动中...");
  Serial.println("=================================");
  #endif

  LittleFS.begin(true);
  migrateLegacyLogFormatIfNeeded();

  initWiFi();
  setupWebServer();

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

  lastEncoded = (static_cast<uint8_t>(digitalRead(ENCODER_CLK)) << 1)
              | static_cast<uint8_t>(digitalRead(ENCODER_DT));

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
    if (currentMillis - lastUserActivityMs >= screenSaverTimeoutMs) {
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

  if (WiFi.status() == WL_CONNECTED) {
    webServer.handleClient();
  }

  appendPeriodicStatusLog();
}

// 编码器中断服务程序
void IRAM_ATTR encoderISR() {
  static const int8_t transitionTable[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0
  };

  unsigned long nowUs = micros();
  if (nowUs - lastEncoderTimeUs < 300) return;
  lastEncoderTimeUs = nowUs;

  uint8_t msb = static_cast<uint8_t>(digitalRead(ENCODER_CLK));
  uint8_t lsb = static_cast<uint8_t>(digitalRead(ENCODER_DT));
  uint8_t encoded = (msb << 1) | lsb;
  uint8_t transition = (lastEncoded << 2) | encoded;
  int8_t movement = transitionTable[transition & 0x0F];

  if (movement != 0) {
    int8_t nextAcc = static_cast<int8_t>(encoderTransitionAcc + movement);
    if (nextAcc >= 4) {
      encoderPos = encoderPos + 1;
      nextAcc = 0;
    } else if (nextAcc <= -4) {
      encoderPos = encoderPos - 1;
      nextAcc = 0;
    }
    encoderTransitionAcc = nextAcc;
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
        menuSelection += (delta > 0) ? 1 : -1;
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
          } else if (settingsSelection == 6) {
            // 屏幕保护时长（分钟步进，内部毫秒）
            int32_t nextValue = static_cast<int32_t>(screenSaverTimeoutMs) + (delta * SCREEN_SAVER_TIMEOUT_STEP_MS);
            if (nextValue < MIN_SCREEN_SAVER_TIMEOUT_MS) nextValue = MIN_SCREEN_SAVER_TIMEOUT_MS;
            if (nextValue > MAX_SCREEN_SAVER_TIMEOUT_MS) nextValue = MAX_SCREEN_SAVER_TIMEOUT_MS;
            screenSaverTimeoutMs = static_cast<uint32_t>(nextValue);
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
          settingsSelection += (delta > 0) ? 1 : -1;
          if (settingsSelection < 0) settingsSelection = 6;
          if (settingsSelection > 6) settingsSelection = 0;
        }
        break;
    }
  }
}

void readTemperature() {
  unsigned long now = millis();
  bool newPrimaryValid = false;
  uint8_t pt100Fault = pt100Sensor.readFault();

  if (pt100Fault == 0) {
    float measuredT1 = pt100Sensor.temperature(PT100_RNOMINAL, PT100_RREF);
    if (!isnan(measuredT1)) {
      if (isnan(lastPt100Temp) || fabsf(measuredT1 - lastPt100Temp) <= PT100_MAX_JUMP_PER_SAMPLE) {
        currentTempRaw = measuredT1;
        lastPt100Temp = measuredT1;
        lastPt100ValidMs = now;
        pt100FaultCount = 0;
        newPrimaryValid = true;
      } else {
        pt100FaultCount++;
      }
    } else {
      pt100FaultCount++;
    }
  } else {
    pt100FaultCount++;
    pt100Sensor.clearFault();
  }

  if (!newPrimaryValid) {
    unsigned long validAge = now - lastPt100ValidMs;
    if (!isnan(lastPt100Temp) && validAge <= PT100_VALID_HOLD_MS && pt100FaultCount < PT100_FAULT_DEBOUNCE_COUNT) {
      currentTempRaw = lastPt100Temp;
    } else {
      #if ENABLE_SERIAL_DEBUG
      Serial.println("错误：PT100(T1) 读取失败！");
      #endif
      if (t1FilterInitialized) {
        currentTempRaw = currentTemp;
      } else {
        currentTempRaw = 0.0f;
      }
    }
  }

  float smoothingAlpha = T1_SMOOTHING_ALPHA;
  if (smoothingAlpha < 0.0f) smoothingAlpha = 0.0f;
  if (smoothingAlpha > 1.0f) smoothingAlpha = 1.0f;

  if (!t1FilterInitialized) {
    currentTemp = currentTempRaw;
    t1FilterInitialized = true;
  } else {
    currentTemp = (smoothingAlpha * currentTempRaw) + ((1.0f - smoothingAlpha) * currentTemp);
  }

  float measuredT2 = thermocouple.readCelsius();
  if (!isnan(measuredT2)) {
    secondaryTempRaw = measuredT2;

    float secondarySmoothingAlpha = T2_SMOOTHING_ALPHA;
    if (secondarySmoothingAlpha < 0.0f) secondarySmoothingAlpha = 0.0f;
    if (secondarySmoothingAlpha > 1.0f) secondarySmoothingAlpha = 1.0f;

    if (!t2FilterInitialized) {
      secondaryTemp = secondaryTempRaw;
      t2FilterInitialized = true;
    } else {
      secondaryTemp = (secondarySmoothingAlpha * secondaryTempRaw) + ((1.0f - secondarySmoothingAlpha) * secondaryTemp);
    }

    secondarySensorValid = true;
  } else {
    secondarySensorValid = false;
    #if ENABLE_SERIAL_DEBUG
    Serial.println("警告：MAX6675(T2) 读取失败，联锁暂不可用");
    #endif
  }

  #if ENABLE_OVERHEAT_PROTECTION
  // 安全边界保护（基于 T1 原始值）
  if (currentTempRaw > safetyUpperTemp || currentTempRaw < safetyLowerTemp) {
    digitalWrite(RELAY_PIN, LOW);
    heaterOn = false;
    systemEnabled = false;
    safetyTripActive = true;
    #if ENABLE_SERIAL_DEBUG
    Serial.printf("!!! 安全保护激活 !!! 当前温度 %.2f°C 超出范围 [%.1f, %.1f]\n", currentTempRaw, safetyLowerTemp, safetyUpperTemp);
    #endif
  } else if (safetyTripActive) {
    // 仅对“安全保护触发导致的关机”自动恢复
    safetyTripActive = false;
    systemEnabled = true;
    #if ENABLE_SERIAL_DEBUG
    Serial.printf("*** 温度恢复安全区间，系统自动恢复开启 (%.2f°C 在 [%.1f, %.1f])\n", currentTempRaw, safetyLowerTemp, safetyUpperTemp);
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
      Serial.printf("T1: %.2f°C | T2: ERROR | T2Lim: %.1f°C | 目标: %.1f°C | Hys: %.1f | Safe:[%.1f,%.1f] | 加热: %s | 系统: %s\n", 
                  currentTemp,
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
  // 第二传感器联锁：
  // - T2 有效且 >= 上限：强制关闭 SSR
  // - T2 有效且 < 上限，或 T2 无效：允许 T1 控制
  if (secondarySensorValid && secondaryTempRaw >= secondaryTempLimit) {
    digitalWrite(RELAY_PIN, LOW);
    heaterOn = false;
    return;
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
  if (screenSaverTimeoutMs < MIN_SCREEN_SAVER_TIMEOUT_MS) screenSaverTimeoutMs = MIN_SCREEN_SAVER_TIMEOUT_MS;
  if (screenSaverTimeoutMs > MAX_SCREEN_SAVER_TIMEOUT_MS) screenSaverTimeoutMs = MAX_SCREEN_SAVER_TIMEOUT_MS;

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
  screenSaverTimeoutMs = settingsStore.getULong(KEY_SCREEN_SAVER_TIMEOUT, DEFAULT_SCREEN_SAVER_TIMEOUT_MS);
  webPassword = settingsStore.getString(KEY_WEB_PASSWORD, "");
  settingsStore.end();

  // 按需求：重启后系统始终为 ON
  systemEnabled = true;

  sanitizeRuntimeSettings();

  #if ENABLE_SERIAL_DEBUG
  Serial.printf("已加载设置: T=%.1f Hys=%.1f Low=%.1f High=%.1f T2Lim=%.1f T2Hys=%.1f ScrSav=%s ScrTm=%lumin Sys=%s\n",
                targetTemp,
                tempHysteresis,
                safetyLowerTemp,
                safetyUpperTemp,
                secondaryTempLimit,
                secondaryTempHysteresis,
                screenSaverEnabled ? "ON" : "OFF",
                static_cast<unsigned long>(screenSaverTimeoutMs / 60000),
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
  settingsStore.putULong(KEY_SCREEN_SAVER_TIMEOUT, screenSaverTimeoutMs);
  settingsStore.putString(KEY_WEB_PASSWORD, webPassword);
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
  screenSaverTimeoutMs = DEFAULT_SCREEN_SAVER_TIMEOUT_MS;
  webPassword = "";
  systemEnabled = true;

  settingsSelection = 0;
  settingsEditing = false;
  menuSelection = 0;
  mainScrollOffset = 0;

  sanitizeRuntimeSettings();
  saveSettings();
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
  }

  #if ENABLE_SERIAL_DEBUG
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected: %s | IP: %s\n", WIFI_SSID, WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("WiFi connect failed: %s\n", WIFI_SSID);
  }
  #endif

  if (WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  }
}

void setupWebServer() {
  webServer.on("/", HTTP_GET, handleWebRoot);
  webServer.on("/api/auth/state", HTTP_GET, handleWebAuthState);
  webServer.on("/api/auth/setup", HTTP_POST, handleWebAuthSetup);
  webServer.on("/api/auth/login", HTTP_POST, handleWebAuthLogin);
  webServer.on("/api/auth/logout", HTTP_POST, handleWebAuthLogout);
  webServer.on("/api/status", HTTP_GET, handleWebStatus);
  webServer.on("/api/settings", HTTP_GET, handleWebSettings);
  webServer.on("/api/settings/update", HTTP_POST, handleWebSettingsUpdate);
  webServer.on("/api/control", HTTP_GET, handleWebControl);
  webServer.on("/logs/download", HTTP_GET, handleWebLogDownload);
  webServer.on("/api/logs/clear", HTTP_POST, handleWebLogClear);
  webServer.onNotFound([]() {
    webServer.send(404, "text/plain", "Not Found");
  });
  webServer.begin();
}

void handleWebRoot() {
  const char* html = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Temp Controller Status</title>
  <style>
    body { font-family: Arial, sans-serif; background: #111; color: #eee; margin: 0; padding: 16px; }
    h1 { margin: 0 0 12px; font-size: 20px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    .card { background: #1b1b1b; border: 1px solid #333; border-radius: 8px; padding: 10px; }
    .k { color: #a0a0a0; font-size: 12px; margin-bottom: 4px; }
    .v { font-size: 24px; font-weight: 700; }
    .line { margin-top: 12px; background: #1b1b1b; border: 1px solid #333; border-radius: 8px; padding: 10px; }
    .ops { margin-top: 12px; display: grid; gap: 8px; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; }
    button, input { background: #222; color: #eee; border: 1px solid #444; border-radius: 6px; padding: 8px; }
    a { color: #7ecbff; }
    .hidden { display: none; }
    .auth { margin-top: 12px; background: #1b1b1b; border: 1px solid #333; border-radius: 8px; padding: 10px; }
    .warn { color: #ff9f9f; }
    .sec { color: #a0a0a0; font-size: 12px; margin: 4px 0 2px; }
    .field { display: flex; flex-direction: column; min-width: 120px; }
    .field label { font-size: 12px; color: #a0a0a0; margin-bottom: 4px; }
  </style>
</head>
<body>
  <h1>ESP32-C6 Status</h1>
  <div id="authPanel" class="auth">
    <div id="authMsg">Checking auth...</div>
    <div id="setupBox" class="hidden">
      <p>First use: create web password</p>
      <input id="setupPwd" type="password" placeholder="New password" />
      <button onclick="setupPassword()">Create Password</button>
    </div>
    <div id="loginBox" class="hidden">
      <p>Login required</p>
      <input id="loginPwd" type="password" placeholder="Password" />
      <button onclick="login()">Login</button>
    </div>
  </div>

  <div id="mainPanel" class="hidden">
  <div class="grid">
    <div class="card"><div class="k">T1</div><div class="v" id="t1">--</div></div>
    <div class="card"><div class="k">T2</div><div class="v" id="t2">--</div></div>
    <div class="card"><div class="k">HEAT</div><div class="v" id="heat">--</div></div>
    <div class="card"><div class="k">SYS</div><div class="v" id="sys">--</div></div>
  </div>
  <div class="line" id="limits">--</div>
  <div class="ops">
    <div class="sec">Control</div>
    <div class="row">
      <button onclick="setSys('toggle')">SYS TOGGLE</button>
    </div>
    <div class="row">
      <input id="targetInput" type="number" step="0.1" min="0" max="300" placeholder="Target °C"/>
      <button onclick="setTarget()">Set Target</button>
    </div>
    <div class="sec">Settings</div>
    <div class="row">
      <div class="field"><label for="hysInput">Hys</label><input id="hysInput" type="number" step="0.1" min="0" max="20" placeholder="Hys"/></div>
      <div class="field"><label for="lowInput">Low</label><input id="lowInput" type="number" step="0.1" min="0" max="200" placeholder="Low"/></div>
      <div class="field"><label for="highInput">High</label><input id="highInput" type="number" step="0.1" min="0" max="200" placeholder="High"/></div>
    </div>
    <div class="row">
      <div class="field"><label for="t2maxInput">T2Max</label><input id="t2maxInput" type="number" step="0.1" min="0" max="200" placeholder="T2Max"/></div>
      <div class="field"><label for="t2hyInput">T2Hy</label><input id="t2hyInput" type="number" step="0.1" min="0" max="20" placeholder="T2Hy"/></div>
      <div class="field"><label for="scrsvInput">ScrSv</label><label><input id="scrsvInput" type="checkbox"/> Enable</label></div>
      <div class="field"><label for="scrtmInput">ScrTm(min)</label><input id="scrtmInput" type="number" step="1" min="1" max="100" placeholder="ScrTm"/></div>
      <button onclick="saveSettings()">Save Settings</button>
    </div>
    <div class="row">
      <a href="/logs/download" download>Download Logs</a>
      <button onclick="clearLogs()">Clear Logs</button>
      <button onclick="logout()">Logout</button>
      <span id="opResult"></span>
    </div>
  </div>
  </div>
  <script>
    let loggedIn = false;
    let authWasLoggedIn = false;

    function isEditingInputs() {
      const active = document.activeElement;
      if (!active) return false;
      return ['targetInput', 'hysInput', 'lowInput', 'highInput', 't2maxInput', 't2hyInput', 'scrtmInput'].includes(active.id);
    }

    function bindEnterToApply() {
      const targetInput = document.getElementById('targetInput');
      targetInput.addEventListener('keydown', async (e) => {
        if (e.key === 'Enter') {
          e.preventDefault();
          await setTarget();
        }
      });

      ['hysInput', 'lowInput', 'highInput', 't2maxInput', 't2hyInput', 'scrtmInput'].forEach((id) => {
        const element = document.getElementById(id);
        element.addEventListener('keydown', async (e) => {
          if (e.key === 'Enter') {
            e.preventDefault();
            await saveSettings();
          }
        });
      });
    }

    async function checkAuth() {
      const res = await fetch('/api/auth/state');
      const s = await res.json();
      const authMsg = document.getElementById('authMsg');
      const setupBox = document.getElementById('setupBox');
      const loginBox = document.getElementById('loginBox');
      const mainPanel = document.getElementById('mainPanel');

      setupBox.classList.add('hidden');
      loginBox.classList.add('hidden');
      mainPanel.classList.add('hidden');

      if (s.blocked) {
        authMsg.innerHTML = `<span class="warn">Blocked for ${s.remainingSec}s due to failed logins</span>`;
        return;
      }

      if (!s.hasPassword) {
        authMsg.textContent = 'Create password to continue';
        setupBox.classList.remove('hidden');
        return;
      }

      if (!s.loggedIn) {
        authMsg.textContent = 'Please login';
        loginBox.classList.remove('hidden');
        authWasLoggedIn = false;
        return;
      }

      loggedIn = true;
      authMsg.textContent = 'Authenticated';
      mainPanel.classList.remove('hidden');
      if (!authWasLoggedIn) {
        await refreshStatus();
        await loadSettings();
        bindEnterToApply();
        authWasLoggedIn = true;
      }
    }

    async function setupPassword() {
      const password = document.getElementById('setupPwd').value;
      const body = new URLSearchParams({ password });
      const res = await fetch('/api/auth/setup', { method: 'POST', body });
      const data = await res.json();
      document.getElementById('opResult').textContent = data.message || '';
      await checkAuth();
    }

    async function login() {
      const password = document.getElementById('loginPwd').value;
      const body = new URLSearchParams({ password });
      const res = await fetch('/api/auth/login', { method: 'POST', body });
      const data = await res.json();
      document.getElementById('opResult').textContent = data.message || '';
      await checkAuth();
    }

    async function logout() {
      await fetch('/api/auth/logout', { method: 'POST' });
      loggedIn = false;
      await checkAuth();
    }

    async function refreshStatus() {
      if (!loggedIn) return;
      try {
        const res = await fetch('/api/status');
        if (res.status === 401 || res.status === 403 || res.status === 429) {
          loggedIn = false;
          await checkAuth();
          return;
        }
        const s = await res.json();
        document.getElementById('t1').textContent = `${s.t1.toFixed(1)}°C / ${s.target.toFixed(1)}°C`;
        document.getElementById('t2').textContent = s.t2Valid ? `${s.t2.toFixed(1)}°C` : 'ERROR';
        document.getElementById('heat').textContent = s.heaterOn ? 'ON' : 'OFF';
        document.getElementById('sys').textContent = s.systemEnabled ? 'ON' : 'OFF';
        document.getElementById('limits').textContent = `Low ${s.safeLow.toFixed(1)}°C | High ${s.safeHigh.toFixed(1)}°C | T2Max ${s.t2Limit.toFixed(1)}°C`;
        if (!isEditingInputs()) {
          document.getElementById('targetInput').value = s.target.toFixed(1);
        }
      } catch (e) {
        document.getElementById('limits').textContent = 'Status fetch failed';
      }
    }

    async function loadSettings() {
      if (!loggedIn) return;
      const res = await fetch('/api/settings');
      if (!res.ok) return;
      const s = await res.json();
      if (isEditingInputs()) return;
      const scrtmInput = document.getElementById('scrtmInput');
      if (Number.isFinite(s.scrtmMin)) scrtmInput.min = String(Math.round(s.scrtmMin));
      if (Number.isFinite(s.scrtmMax)) scrtmInput.max = String(Math.round(s.scrtmMax));
      if (Number.isFinite(s.scrtmStep)) scrtmInput.step = String(Math.round(s.scrtmStep));
      document.getElementById('targetInput').value = s.target.toFixed(1);
      document.getElementById('hysInput').value = s.hys.toFixed(1);
      document.getElementById('lowInput').value = s.low.toFixed(1);
      document.getElementById('highInput').value = s.high.toFixed(1);
      document.getElementById('t2maxInput').value = s.t2max.toFixed(1);
      document.getElementById('t2hyInput').value = s.t2hy.toFixed(1);
      document.getElementById('scrsvInput').checked = !!s.scrsv;
      scrtmInput.value = String(Math.round(s.scrtm));
    }

    async function setSys(mode) {
      const res = await fetch(`/api/control?sys=${mode}`);
      const data = await res.json();
      document.getElementById('opResult').textContent = data.message || 'OK';
      refreshStatus();
    }

    async function setTarget() {
      const value = document.getElementById('targetInput').value;
      const res = await fetch(`/api/control?target=${encodeURIComponent(value)}`);
      const data = await res.json();
      document.getElementById('opResult').textContent = data.message || 'OK';
      refreshStatus();
    }

    async function saveSettings() {
      const body = new URLSearchParams({
        hys: document.getElementById('hysInput').value,
        low: document.getElementById('lowInput').value,
        high: document.getElementById('highInput').value,
        t2max: document.getElementById('t2maxInput').value,
        t2hy: document.getElementById('t2hyInput').value,
        scrsv: document.getElementById('scrsvInput').checked ? '1' : '0',
        scrtm: document.getElementById('scrtmInput').value
      });
      const res = await fetch('/api/settings/update', { method: 'POST', body });
      const data = await res.json();
      document.getElementById('opResult').textContent = data.message || 'OK';
      await refreshStatus();
      await loadSettings();
    }

    async function clearLogs() {
      const yes = window.confirm('Clear all logs now?');
      if (!yes) return;
      const res = await fetch('/api/logs/clear', { method: 'POST' });
      const data = await res.json();
      document.getElementById('opResult').textContent = data.message || 'OK';
    }

    checkAuth();
    setInterval(checkAuth, 3000);
    setInterval(refreshStatus, 2000);
  </script>
</body>
</html>
)HTML";

  webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  webServer.sendHeader("Pragma", "no-cache");
  webServer.sendHeader("Expires", "0");
  webServer.send(200, "text/html", html);
}

void handleWebAuthState() {
  String ip = getWebClientIp();
  unsigned long remainingMs = 0;
  bool blocked = isWebClientBlocked(ip, &remainingMs);
  bool hasPassword = webPassword.length() > 0;
  bool loggedIn = hasPassword && isWebClientLoggedIn(ip);

  String json = "{";
  json += "\"hasPassword\":" + String(hasPassword ? "true" : "false") + ",";
  json += "\"loggedIn\":" + String(loggedIn ? "true" : "false") + ",";
  json += "\"blocked\":" + String(blocked ? "true" : "false") + ",";
  json += "\"remainingSec\":" + String((remainingMs + 999) / 1000);
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleWebAuthSetup() {
  if (webPassword.length() > 0) {
    webServer.send(400, "application/json", "{\"ok\":false,\"message\":\"Password already exists\"}");
    return;
  }

  if (!webServer.hasArg("password")) {
    webServer.send(400, "application/json", "{\"ok\":false,\"message\":\"Missing password\"}");
    return;
  }

  String password = webServer.arg("password");
  if (password.length() < 4) {
    webServer.send(400, "application/json", "{\"ok\":false,\"message\":\"Password too short\"}");
    return;
  }

  webPassword = password;
  saveSettings();
  markWebLoginSuccess(getWebClientIp());
  appendLogLine();
  webServer.send(200, "application/json", "{\"ok\":true,\"message\":\"Password created\"}");
}

void handleWebAuthLogin() {
  String ip = getWebClientIp();
  unsigned long remainingMs = 0;
  if (isWebClientBlocked(ip, &remainingMs)) {
    String json = String("{\"ok\":false,\"message\":\"Blocked\",\"remainingSec\":") + String((remainingMs + 999) / 1000) + "}";
    webServer.send(429, "application/json", json);
    return;
  }

  if (webPassword.length() == 0) {
    webServer.send(403, "application/json", "{\"ok\":false,\"message\":\"Password setup required\"}");
    return;
  }

  if (!webServer.hasArg("password")) {
    webServer.send(400, "application/json", "{\"ok\":false,\"message\":\"Missing password\"}");
    return;
  }

  if (webServer.arg("password") == webPassword && webPassword.length() > 0) {
    markWebLoginSuccess(ip);
    appendLogLine();
    webServer.send(200, "application/json", "{\"ok\":true,\"message\":\"Login success\"}");
  } else {
    markWebLoginFailure(ip);
    appendLogLine();
    webServer.send(401, "application/json", "{\"ok\":false,\"message\":\"Invalid password\"}");
  }
}

void handleWebAuthLogout() {
  markWebLogout(getWebClientIp());
  webServer.send(200, "application/json", "{\"ok\":true,\"message\":\"Logged out\"}");
}

void handleWebStatus() {
  if (!ensureAuthenticated()) return;

  String json = "{";
  json += "\"t1\":" + String(currentTemp, 2) + ",";
  json += "\"target\":" + String(targetTemp, 2) + ",";
  json += "\"t2Valid\":" + String(secondarySensorValid ? "true" : "false") + ",";
  json += "\"t2\":" + (secondarySensorValid ? String(secondaryTemp, 2) : String("null")) + ",";
  json += "\"heaterOn\":" + String(heaterOn ? "true" : "false") + ",";
  json += "\"systemEnabled\":" + String(systemEnabled ? "true" : "false") + ",";
  json += "\"safeLow\":" + String(safetyLowerTemp, 2) + ",";
  json += "\"safeHigh\":" + String(safetyUpperTemp, 2) + ",";
  json += "\"t2Limit\":" + String(secondaryTempLimit, 2);
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleWebSettings() {
  if (!ensureAuthenticated()) return;

  String json = "{";
  json += "\"target\":" + String(targetTemp, 2) + ",";
  json += "\"hys\":" + String(tempHysteresis, 2) + ",";
  json += "\"low\":" + String(safetyLowerTemp, 2) + ",";
  json += "\"high\":" + String(safetyUpperTemp, 2) + ",";
  json += "\"t2max\":" + String(secondaryTempLimit, 2) + ",";
  json += "\"t2hy\":" + String(secondaryTempHysteresis, 2) + ",";
  json += "\"scrsv\":" + String(screenSaverEnabled ? "true" : "false") + ",";
  json += "\"scrtm\":" + String(static_cast<unsigned long>(screenSaverTimeoutMs / 60000)) + ",";
  json += "\"scrtmMin\":" + String(screenSaverTimeoutMinMinutes) + ",";
  json += "\"scrtmMax\":" + String(screenSaverTimeoutMaxMinutes) + ",";
  json += "\"scrtmStep\":" + String(screenSaverTimeoutStepMinutes);
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleWebSettingsUpdate() {
  if (!ensureAuthenticated()) return;

  if (webServer.hasArg("hys")) tempHysteresis = webServer.arg("hys").toFloat();
  if (webServer.hasArg("low")) safetyLowerTemp = webServer.arg("low").toFloat();
  if (webServer.hasArg("high")) safetyUpperTemp = webServer.arg("high").toFloat();
  if (webServer.hasArg("t2max")) secondaryTempLimit = webServer.arg("t2max").toFloat();
  if (webServer.hasArg("t2hy")) secondaryTempHysteresis = webServer.arg("t2hy").toFloat();
  if (webServer.hasArg("scrsv")) screenSaverEnabled = (webServer.arg("scrsv") == "1");
  if (webServer.hasArg("scrtm")) {
    long timeoutMin = webServer.arg("scrtm").toInt();
    screenSaverTimeoutMs = static_cast<uint32_t>(timeoutMin) * 60000UL;
  }
  sanitizeRuntimeSettings();
  saveSettings();

  appendLogLine();
  webServer.send(200, "application/json", "{\"ok\":true,\"message\":\"Settings updated\"}");
}

void handleWebControl() {
  if (!ensureAuthenticated()) return;

  bool changed = false;
  String message = "No change";

  if (webServer.hasArg("sys")) {
    String mode = webServer.arg("sys");
    if (mode == "on") {
      systemEnabled = true;
      changed = true;
      message = "SYS ON";
    } else if (mode == "off") {
      systemEnabled = false;
      changed = true;
      message = "SYS OFF";
    } else if (mode == "toggle") {
      systemEnabled = !systemEnabled;
      changed = true;
      message = String("SYS ") + (systemEnabled ? "ON" : "OFF");
    }
  }

  if (webServer.hasArg("target")) {
    float requested = webServer.arg("target").toFloat();
    targetTemp = requested;
    sanitizeRuntimeSettings();
    changed = true;
    message = String("Target set to ") + String(targetTemp, 1) + "C";
  }

  if (changed) {
    saveSettings();
    appendLogLine();
  }

  String json = "{";
  json += "\"ok\":true,";
  json += "\"message\":\"" + jsonEscape(message) + "\"";
  json += "}";
  webServer.send(200, "application/json", json);
}

void handleWebLogDownload() {
  if (!ensureAuthenticated()) return;

  if (!LittleFS.exists(LOG_FILE_PATH)) {
    webServer.send(200, "text/plain", "No log file yet\n");
    return;
  }

  File file = LittleFS.open(LOG_FILE_PATH, "r");
  if (!file) {
    webServer.send(500, "text/plain", "Failed to open log file\n");
    return;
  }

  webServer.sendHeader("Content-Disposition", "attachment; filename=status.csv");
  webServer.streamFile(file, "text/plain");
  file.close();
}

void handleWebLogClear() {
  if (!ensureAuthenticated()) return;

  if (LittleFS.exists(LOG_FILE_PATH)) {
    LittleFS.remove(LOG_FILE_PATH);
  }
  if (LittleFS.exists(LOG_OLD_FILE_PATH)) {
    LittleFS.remove(LOG_OLD_FILE_PATH);
  }

  lastWebLogWriteMs = millis();
  webServer.send(200, "application/json", "{\"ok\":true,\"message\":\"Logs cleared\"}");
}

AuthClientState* getAuthClientState(const String& ip, bool createIfMissing) {
  int firstEmptyIndex = -1;

  for (int index = 0; index < MAX_AUTH_CLIENTS; index++) {
    if (authClients[index].ip == ip) {
      return &authClients[index];
    }
    if (firstEmptyIndex < 0 && authClients[index].ip.length() == 0) {
      firstEmptyIndex = index;
    }
  }

  if (!createIfMissing) return nullptr;

  int slot = (firstEmptyIndex >= 0) ? firstEmptyIndex : 0;
  authClients[slot].ip = ip;
  authClients[slot].failCount = 0;
  authClients[slot].blockedUntilMs = 0;
  authClients[slot].loggedIn = false;
  return &authClients[slot];
}

String getWebClientIp() {
  return webServer.client().remoteIP().toString();
}

bool isWebClientBlocked(const String& ip, unsigned long* remainingMs) {
  AuthClientState* client = getAuthClientState(ip, false);
  if (!client) {
    if (remainingMs) *remainingMs = 0;
    return false;
  }

  unsigned long now = millis();
  if (client->blockedUntilMs == 0 || now >= client->blockedUntilMs) {
    client->blockedUntilMs = 0;
    client->failCount = 0;
    if (remainingMs) *remainingMs = 0;
    return false;
  }

  if (remainingMs) *remainingMs = client->blockedUntilMs - now;
  return true;
}

bool isWebClientLoggedIn(const String& ip) {
  AuthClientState* client = getAuthClientState(ip, false);
  return client && client->loggedIn;
}

void markWebLoginFailure(const String& ip) {
  AuthClientState* client = getAuthClientState(ip, true);
  if (!client) return;

  client->loggedIn = false;
  if (client->failCount < 255) client->failCount++;
  if (client->failCount >= WEB_AUTH_MAX_FAILS) {
    client->blockedUntilMs = millis() + WEB_AUTH_BLOCK_MS;
    client->failCount = 0;
  }
}

void markWebLoginSuccess(const String& ip) {
  AuthClientState* client = getAuthClientState(ip, true);
  if (!client) return;

  client->loggedIn = true;
  client->failCount = 0;
  client->blockedUntilMs = 0;
}

void markWebLogout(const String& ip) {
  AuthClientState* client = getAuthClientState(ip, false);
  if (!client) return;
  client->loggedIn = false;
}

bool ensureAuthenticated() {
  String ip = getWebClientIp();
  unsigned long remainingMs = 0;

  if (isWebClientBlocked(ip, &remainingMs)) {
    String json = String("{\"ok\":false,\"message\":\"Blocked\",\"remainingSec\":") + String((remainingMs + 999) / 1000) + "}";
    webServer.send(429, "application/json", json);
    return false;
  }

  if (webPassword.length() == 0) {
    webServer.send(403, "application/json", "{\"ok\":false,\"message\":\"Password setup required\"}");
    return false;
  }

  if (!isWebClientLoggedIn(ip)) {
    webServer.send(401, "application/json", "{\"ok\":false,\"message\":\"Authentication required\"}");
    return false;
  }

  return true;
}

String jsonEscape(const String& input) {
  String output = input;
  output.replace("\\", "\\\\");
  output.replace("\"", "\\\"");
  return output;
}

void rotateLogsIfNeeded() {
  if (!LittleFS.exists(LOG_FILE_PATH)) return;

  File logFile = LittleFS.open(LOG_FILE_PATH, "r");
  if (!logFile) return;
  size_t fileSize = logFile.size();
  logFile.close();

  if (fileSize <= LOG_MAX_FILE_SIZE) return;

  if (LittleFS.exists(LOG_OLD_FILE_PATH)) {
    LittleFS.remove(LOG_OLD_FILE_PATH);
  }
  LittleFS.rename(LOG_FILE_PATH, LOG_OLD_FILE_PATH);
}

void formatLogDateAndTime(String& datePart, String& timePart) {
  time_t now = time(nullptr);
  struct tm tmNow;

  if (now > 100000 && localtime_r(&now, &tmNow)) {
    char dateBuffer[11];
    char timeBuffer[9];
    snprintf(dateBuffer, sizeof(dateBuffer), "%04d/%02d/%02d",
             tmNow.tm_year + 1900,
             tmNow.tm_mon + 1,
             tmNow.tm_mday);
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d",
             tmNow.tm_hour,
             tmNow.tm_min,
             tmNow.tm_sec);
    datePart = String(dateBuffer);
    timePart = String(timeBuffer);
    return;
  }

  unsigned long seconds = millis() / 1000;
  unsigned long hh = (seconds / 3600) % 24;
  unsigned long mm = (seconds / 60) % 60;
  unsigned long ss = seconds % 60;
  char fallbackTime[9];
  snprintf(fallbackTime, sizeof(fallbackTime), "%02lu:%02lu:%02lu", hh, mm, ss);
  datePart = "1970/01/01";
  timePart = String(fallbackTime);
}

void appendLogLine() {
  File logFile = LittleFS.open(LOG_FILE_PATH, "a");
  if (!logFile) return;

  if (logFile.size() == 0) {
    logFile.println(LOG_CSV_HEADER);
  }

  String datePart;
  String timePart;
  formatLogDateAndTime(datePart, timePart);
  int t1Int = (int)lroundf(currentTemp);
  int t1SetInt = (int)lroundf(targetTemp);
  String t2Field = secondarySensorValid ? String((int)lroundf(secondaryTemp)) : String("");
  char heatState = heaterOn ? 'O' : 'F';
  logFile.printf("%s,%s,%d,%d,%s,%c\n", datePart.c_str(), timePart.c_str(), t1Int, t1SetInt, t2Field.c_str(), heatState);
  logFile.close();
  rotateLogsIfNeeded();
}

void appendPeriodicStatusLog() {
  unsigned long now = millis();
  if (now - lastWebLogWriteMs < WEB_LOG_INTERVAL_MS) return;
  lastWebLogWriteMs = now;

  appendLogLine();
}

void migrateLegacyLogFormatIfNeeded() {
  if (!LittleFS.exists(LOG_FILE_PATH)) return;

  File logFile = LittleFS.open(LOG_FILE_PATH, "r");
  if (!logFile) return;

  String firstLine = logFile.readStringUntil('\n');
  firstLine.trim();

  if (firstLine == LOG_CSV_HEADER) {
    firstLine = logFile.readStringUntil('\n');
  }

  logFile.close();
  firstLine.trim();
  if (firstLine.length() == 0) return;

  int commaCount = 0;
  for (size_t i = 0; i < firstLine.length(); i++) {
    if (firstLine[i] == ',') commaCount++;
  }

  int firstComma = firstLine.indexOf(',');
  int secondComma = (firstComma >= 0) ? firstLine.indexOf(',', firstComma + 1) : -1;
  bool dateOk = false;
  bool timeOk = false;

  if (firstComma > 0 && secondComma > firstComma + 1) {
    String dateField = firstLine.substring(0, firstComma);
    String timeField = firstLine.substring(firstComma + 1, secondComma);

    dateOk = dateField.length() == 10 && dateField[4] == '/' && dateField[7] == '/';
    if (dateOk) {
      for (int i = 0; i < 10; i++) {
        if (i == 4 || i == 7) continue;
        if (dateField[i] < '0' || dateField[i] > '9') {
          dateOk = false;
          break;
        }
      }
    }

    timeOk = timeField.length() == 8 && timeField[2] == ':' && timeField[5] == ':';
    if (timeOk) {
      for (int i = 0; i < 8; i++) {
        if (i == 2 || i == 5) continue;
        if (timeField[i] < '0' || timeField[i] > '9') {
          timeOk = false;
          break;
        }
      }
    }
  }

  bool formatOk = (commaCount == 5) && dateOk && timeOk;
  if (formatOk) return;

  if (LittleFS.exists(LOG_OLD_FILE_PATH)) {
    LittleFS.remove(LOG_OLD_FILE_PATH);
  }
  LittleFS.rename(LOG_FILE_PATH, LOG_OLD_FILE_PATH);
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
      display.printf("%s ScrTm: %lumin\n", settingsSelection == 6 ? ">" : " ", static_cast<unsigned long>(screenSaverTimeoutMs / 60000));
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
  String wifiState = (WiFi.status() == WL_CONNECTED) ? "CONNECTED" : "DISCONNECTED";
  String ipText = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : "-";

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("=== MENU ==="));
  display.println(menuSelection == 0 ? "> Set Target" : "  Set Target");
  display.println(menuSelection == 1
                    ? (systemEnabled ? "> SYS: ON" : "> SYS: OFF")
                    : (systemEnabled ? "  SYS: ON" : "  SYS: OFF"));
  display.println(menuSelection == 2 ? "> Settings" : "  Settings");
  display.println(menuSelection == 3 ? "> Factory Reset" : "  Factory Reset");
  display.println(String("WiFi: ") + wifiState);
  display.println(String("IP: ") + ipText);
  display.println(F("SW:SEL BACK:EXIT"));
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
