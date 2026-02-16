#include <Arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#if defined(__has_include)
#  if __has_include("receiver_secrets.h")
#    include "receiver_secrets.h"
#  endif
#endif

// ====== User Config ======
#ifndef RECEIVER_WIFI_SSID
#define RECEIVER_WIFI_SSID "YOUR_WIFI_SSID"
#endif

#ifndef RECEIVER_WIFI_PASSWORD
#define RECEIVER_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

const char* PRIMARY_IP = "192.168.4.111";  // Main controller IP
const uint16_t PRIMARY_PORT = 80;
const char* SYNC_KEY = ""; // Must match WIFI_SYNC_API_KEY in primary config.h
const unsigned long SYNC_INTERVAL_MS = 1000;
const unsigned long DISPLAY_REFRESH_MS = 100;
const unsigned long BLINK_INTERVAL_MS = 300;

const char* WIFI_SSID = RECEIVER_WIFI_SSID;
const char* WIFI_PASSWORD = RECEIVER_WIFI_PASSWORD;

// HW-364A OLED (SSD1306, 128x64)
static const int SCREEN_WIDTH = 128;
static const int SCREEN_HEIGHT = 64;
static const int OLED_RESET = -1;
static const int I2C_SDA_PIN = 14; // HW-364A D6
static const int I2C_SCL_PIN = 12; // HW-364A D5
static const uint8_t OLED_ADDR = 0x3C;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

struct SyncState {
  float t1 = NAN;
  float target = NAN;
  float t2 = NAN;
  float safeLow = NAN;
  float safeHigh = NAN;
  bool t2Valid = false;
  bool heaterOn = false;
  bool systemEnabled = false;
  int rssi = -127;
  bool valid = false;
  unsigned long updatedMs = 0;
};

SyncState state;
unsigned long lastSyncMs = 0;
unsigned long lastDisplayMs = 0;
bool connectionErrorActive = true;
String connectionErrorReason = "WiFi";

int findKeyStart(const String& json, const char* key) {
  String token = String("\"") + key + "\":";
  return json.indexOf(token);
}

String readJsonValueRaw(const String& json, const char* key) {
  int start = findKeyStart(json, key);
  if (start < 0) return "";

  start = json.indexOf(':', start);
  if (start < 0) return "";
  start += 1;

  while (start < (int)json.length() && (json[start] == ' ' || json[start] == '\t')) {
    start++;
  }

  int end = start;
  bool quoted = (start < (int)json.length() && json[start] == '"');
  if (quoted) {
    start++;
    end = start;
    while (end < (int)json.length() && json[end] != '"') {
      end++;
    }
    return json.substring(start, end);
  }

  while (end < (int)json.length()) {
    char ch = json[end];
    if (ch == ',' || ch == '}' || ch == '\r' || ch == '\n') {
      break;
    }
    end++;
  }

  String value = json.substring(start, end);
  value.trim();
  return value;
}

bool readJsonBool(const String& json, const char* key, bool defaultValue) {
  String value = readJsonValueRaw(json, key);
  if (value.equalsIgnoreCase("true")) return true;
  if (value.equalsIgnoreCase("false")) return false;
  return defaultValue;
}

float readJsonFloat(const String& json, const char* key, float defaultValue) {
  String value = readJsonValueRaw(json, key);
  if (value.length() == 0 || value.equalsIgnoreCase("null")) return defaultValue;
  return value.toFloat();
}

int readJsonInt(const String& json, const char* key, int defaultValue) {
  String value = readJsonValueRaw(json, key);
  if (value.length() == 0 || value.equalsIgnoreCase("null")) return defaultValue;
  return value.toInt();
}

bool fetchSyncState() {
  if (WiFi.status() != WL_CONNECTED) {
    connectionErrorActive = true;
    connectionErrorReason = "WiFi";
    return false;
  }

  String url = String("http://") + PRIMARY_IP + ":" + String(PRIMARY_PORT) + "/api/sync/status";
  if (strlen(SYNC_KEY) > 0) {
    url += "?key=";
    url += SYNC_KEY;
  }

  WiFiClient wifiClient;
  HTTPClient http;
  if (!http.begin(wifiClient, url)) {
    connectionErrorActive = true;
    connectionErrorReason = "HTTPInit";
    return false;
  }
  http.setTimeout(2500);

  int code = http.GET();
  if (code != 200) {
    connectionErrorActive = true;
    if (code < 0) {
      connectionErrorReason = "Timeout";
    } else {
      connectionErrorReason = String("HTTP") + String(code);
    }
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  state.t1 = readJsonFloat(payload, "t1", NAN);
  state.target = readJsonFloat(payload, "target", NAN);
  state.t2 = readJsonFloat(payload, "t2", NAN);
  state.safeLow = readJsonFloat(payload, "safeLow", NAN);
  state.safeHigh = readJsonFloat(payload, "safeHigh", NAN);
  state.t2Valid = readJsonBool(payload, "t2Valid", false);
  state.heaterOn = readJsonBool(payload, "heaterOn", false);
  state.systemEnabled = readJsonBool(payload, "systemEnabled", false);
  state.rssi = readJsonInt(payload, "rssi", -127);
  state.valid = true;
  state.updatedMs = millis();
  connectionErrorActive = false;
  connectionErrorReason = "";
  return true;
}

void drawState() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  bool blinkOn = ((millis() / BLINK_INTERVAL_MS) % 2UL) == 0;

  const int statusY = 0;
  const int labelY = 18;
  const int valueY = 28;
  const int bottomY = 56;

  bool hasT1 = state.valid && !isnan(state.t1);
  bool hasTarget = !isnan(state.target);
  bool hasSafeRange = !isnan(state.safeLow) && !isnan(state.safeHigh);

  bool tooLow = hasT1 && hasSafeRange && (state.t1 < state.safeLow);
  bool tooHigh = hasT1 && hasSafeRange && (state.t1 > state.safeHigh);
  bool outOfSafeRange = tooLow || tooHigh;
  bool targetGapTooLarge = hasT1 && hasTarget && (fabsf(state.t1 - state.target) > 10.0f);
  bool warning = outOfSafeRange || targetGapTooLarge;

  String warningText = "";
  if (warning) {
    if (outOfSafeRange && targetGapTooLarge) {
      warningText = tooLow ? "Too Low + dT" : "Too High + dT";
    } else if (tooLow) {
      warningText = "Too Low";
    } else if (tooHigh) {
      warningText = "Too High";
    } else {
      warningText = "dT>10C";
    }
  }

  display.setTextSize(2);
  display.setCursor(0, statusY);
  if (connectionErrorActive) {
    if (blinkOn) display.print("LINK");
  } else if (warning) {
    if (blinkOn) display.print(warningText);
  }

  display.setTextSize(1);
  display.setCursor(2, labelY);
  display.print("T1");
  display.setCursor(66, labelY);
  display.print("T2");

  if (connectionErrorActive) {
    display.setTextSize(2);
    display.setCursor(0, valueY);
    display.print(connectionErrorReason);
  } else {
    display.setTextSize(3);
    display.setCursor(0, valueY);
    if (hasT1) {
      display.print((int)lroundf(state.t1));
    } else {
      display.print("--");
    }

    display.setCursor(64, valueY);
    if (!state.t2Valid || isnan(state.t2)) {
      display.print("ERR");
    } else {
      display.print((int)lroundf(state.t2));
    }
  }

  display.setTextSize(1);
  display.setCursor(0, bottomY);
  display.print("Heat:");
  display.print(state.heaterOn ? "ON" : "OFF");
  display.print(" Sys:");
  display.print(state.systemEnabled ? "ON" : "OFF");

  display.display();
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(300);
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    for (;;) {
      delay(1000);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESP8266 Mirror Boot");
  display.display();

  connectWiFi();
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }

  unsigned long now = millis();
  if (now - lastSyncMs >= SYNC_INTERVAL_MS) {
    lastSyncMs = now;
    fetchSyncState();
  }

  if (now - lastDisplayMs >= DISPLAY_REFRESH_MS) {
    lastDisplayMs = now;
    drawState();
  }

  delay(20);
}
