/**
 * Configuration for nanoESP32-C6 Temperature Controller
 * Hardware: MAX6675 + I2C OLED + Rotary Encoder + Buttons + RGB LED
 * 
 * Project: ESP32-C6 Temperature Controller
 * Board: nanoESP32-C6 v1.0
 * Date: 2026-02-12
 */

#ifndef CONFIG_H
#define CONFIG_H

// ========================
// Pin Configuration
// ========================

// MAX6675 Thermocouple Interface (3 pins)
#define MAXDO   4   // SO pin (Serial Out)
#define MAXCS   5   // CS pin (Chip Select)
#define MAXCLK  6   // SCK pin (Serial Clock)

// MAX31865 PT100 Interface (software SPI)
// Use available pins and share SCK/MISO with MAX6675 to avoid unavailable GPIO16/17
#define PT100_CS    9       // CS pin
#define PT100_MOSI  15      // SDI pin
#define PT100_MISO  MAXDO   // SDO pin (shared with MAX6675 SO)
#define PT100_SCK   MAXCLK  // CLK pin (shared with MAX6675 SCK)

// I2C OLED Display SSD1306 (2 pins)
#define I2C_SDA 22  // Changed from GPIO 8 to avoid conflict with onboard RGB LED
#define I2C_SCL 23  // Changed from GPIO 9

// Rotary Encoder (2 pins for rotation only)
#define ENCODER_CLK 10  // CLK (A pin)
#define ENCODER_DT  11  // DT (B pin)
#define ENCODER_SW  21  // SW (Button) - used as confirm/select key

// Control Buttons (2 independent buttons)
#define BTN_CONFIRM 12  // Menu key (open popup menu)
#define BTN_BACK    13  // Back button

// Output Control (2 additional pins)
#define RELAY_PIN   7   // Relay/SSR control for heater
#define RGB_LED_PIN 8   // WS2812 RGB LED (onboard LED on nanoESP32-C6)

// ========================
// RGB LED Settings (WS2812)
// ========================
#define RGB_LED_COUNT   1       // Number of LEDs
#define RGB_LED_BRIGHTNESS  50      // Brightness 0-255 (50 = 20%)

// RGB LED Colors for different states
#define COLOR_OFF       0x000000  // LED off
#define COLOR_NORMAL    0x00FF00  // Green - normal operation
#define COLOR_HEATING   0xFF8000  // Orange - heating
#define COLOR_COOLING   0x0080FF  // Light blue - cooling (standby)
#define COLOR_OVERHEAT  0xFF0000  // Red - overheat alarm
#define COLOR_UNDERHEAT 0x0000FF  // Blue - underheat alarm
#define COLOR_ERROR     0xFF00FF  // Magenta - sensor error

// ========================
// Display Settings
// ========================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C  // Some displays use 0x3D
#define SCREEN_SAVER_ENABLED true
#define SCREEN_SAVER_TIMEOUT_MS 60000  // Auto turn off display after inactivity (ms)

// ========================
// Temperature Control
// ========================
#define DEFAULT_TARGET_TEMP  18.0   // Default target (°C)
#define TEMP_HYSTERESIS      2.0    // Control hysteresis (±°C)
#define MIN_HYSTERESIS       0.1    // Minimum hysteresis (°C)
#define MAX_HYSTERESIS       20.0   // Maximum hysteresis (°C)
#define HYSTERESIS_ADJUST_STEP 0.1  // Hysteresis adjust step
#define DEFAULT_SAFE_LOWER_TEMP  0.0    // Default lower safety boundary (°C)
#define DEFAULT_SAFE_UPPER_TEMP  MAX_SAFE_TEMP // Default upper safety boundary (°C)
#define SAFETY_BOUNDARY_MIN     5.0   // Hard minimum boundary value
#define SAFETY_BOUNDARY_MAX     95.0   // Hard maximum boundary value
#define SAFETY_BOUNDARY_STEP      1.0   // Safety boundary adjust step (°C)
#define MIN_SAFE_RANGE_GAP        5.0   // Minimum gap between lower and upper boundary (°C)
#define MIN_TEMP                 5// Minimum settable temp
#define MAX_TEMP             95.0  // Maximum settable temp
#define TEMP_ADJUST_STEP     0.5    // Adjustment step per encoder click

// Second temperature sensor (MAX31865 + PT100)
#define DEFAULT_SECOND_SENSOR_MAX_TEMP 150.0  // Interlock threshold default (°C)
#define SECOND_SENSOR_HYSTERESIS        2.0    // Interlock hysteresis (°C), release at (T2Lim - Hys)
#define MIN_SECOND_SENSOR_HYSTERESIS    0.0    // T2 hysteresis min (°C)
#define MAX_SECOND_SENSOR_HYSTERESIS    20.0   // T2 hysteresis max (°C)
#define SECOND_SENSOR_HYSTERESIS_STEP   0.1    // T2 hysteresis adjust step (°C)
#define MIN_SECOND_SENSOR_MAX_TEMP     0.0  // Adjustable lower limit (°C)
#define MAX_SECOND_SENSOR_MAX_TEMP     200.0  // Adjustable upper limit (°C)
#define SECOND_SENSOR_MAX_STEP           1.0  // Adjustment step (°C)
#define PT100_RNOMINAL       100.0    // PT100 nominal resistance at 0°C
#define PT100_RREF           430.0    // Reference resistor value on MAX31865 board
#define PT100_WIRES          3        // RTD wiring: 2/3/4
#define PT100_FAULT_DEBOUNCE_COUNT 3  // Consecutive faults required before marking T2 invalid
#define PT100_VALID_HOLD_MS    3000   // Keep last valid T2 for short contact glitches
#define PT100_MAX_JUMP_PER_SAMPLE 12.0 // Reject unrealistic single-sample jump (°C)

// ========================
// Timing Parameters
// ========================
#define TEMP_UPDATE_INTERVAL     500   // Temperature read interval (ms)
#define DISPLAY_UPDATE_INTERVAL  500   // Display refresh (ms)
#define ENCODER_DEBOUNCE_DELAY   5     // Encoder debounce (ms)
#define BUTTON_DEBOUNCE_DELAY    200   // Button debounce (ms)

// ========================
// Safety Settings
// ========================
#define MAX_SAFE_TEMP        95  // Emergency shutoff temp
#define ENABLE_OVERHEAT_PROTECTION  true
#define ALARM_BLINK_INTERVAL 125    // Alarm LED blink interval (ms), 2x faster blinking

// ========================
// Debug
// ========================
#define SERIAL_BAUD_RATE     115200
#define ENABLE_SERIAL_DEBUG  true

#endif // CONFIG_H
