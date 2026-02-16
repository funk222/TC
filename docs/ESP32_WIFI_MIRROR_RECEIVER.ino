#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ====== User Config ======
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* PRIMARY_IP = "192.168.1.100";  // Main controller IP
const uint16_t PRIMARY_PORT = 80;
const char* SYNC_KEY = ""; // Must match WIFI_SYNC_API_KEY in primary config.h
const unsigned long SYNC_INTERVAL_MS = 1000;

// OLED SSD1306 (128x64)
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
  bool t2Valid = false;
  bool heaterOn = false;
  bool systemEnabled = false;
  int rssi = -127;
  bool valid = false;
  unsigned long updatedMs = 0;
};

SyncState state;
unsigned long lastSyncMs = 0;

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
    return false;
  }

  String url = String("http://") + PRIMARY_IP + ":" + String(PRIMARY_PORT) + "/api/sync/status";
  if (strlen(SYNC_KEY) > 0) {
    url += "?key=";
    url += SYNC_KEY;
  }

  HTTPClient http;
  http.begin(url);
  http.setTimeout(2500);

  int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  state.t1 = readJsonFloat(payload, "t1", NAN);
  state.target = readJsonFloat(payload, "target", NAN);
  state.t2 = readJsonFloat(payload, "t2", NAN);
  state.t2Valid = readJsonBool(payload, "t2Valid", false);
  state.heaterOn = readJsonBool(payload, "heaterOn", false);
  state.systemEnabled = readJsonBool(payload, "systemEnabled", false);
  state.rssi = readJsonInt(payload, "rssi", -127);
  state.valid = true;
  state.updatedMs = millis();
  return true;
}

void drawState() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Mirror: ");
  display.println(state.valid ? "OK" : "WAIT");

  display.setTextSize(1);
  display.setCursor(0, 12);
  display.print("T1:");
  if (isnan(state.t1)) {
    display.print("--");
  } else {
    display.print(state.t1, 1);
  }
  display.print("  Tg:");
  if (isnan(state.target)) {
    display.print("--");
  } else {
    display.print(state.target, 1);
  }

  display.setCursor(0, 24);
  display.print("T2:");
  if (!state.t2Valid || isnan(state.t2)) {
    display.print("ERR");
  } else {
    display.print(state.t2, 1);
  }

  display.setCursor(0, 36);
  display.print("Heat:");
  display.print(state.heaterOn ? "ON" : "OFF");
  display.print(" Sys:");
  display.print(state.systemEnabled ? "ON" : "OFF");

  display.setCursor(0, 48);
  display.print("WiFi RSSI:");
  display.print(state.rssi);
  display.print("dBm");

  display.setCursor(0, 58);
  if (state.valid) {
    display.print("Age:");
    display.print((millis() - state.updatedMs) / 1000);
    display.print("s");
  } else {
    display.print("No data");
  }

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
  display.println("ESP32 Mirror Boot");
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
    drawState();
  }

  delay(20);
}
