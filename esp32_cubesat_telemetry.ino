/*
  ============================================================================
  CUBESAT DASHBOARD FIRMWARE - ESP32
  MPU6050 + BMP280 (raw registers) + NEO-6M GPS + MQ-135, on 0.96" OLED,
  PLUS a continuous JSON telemetry stream over USB Serial for a laptop
  dashboard (connects via the same USB cable used to program the board -
  no extra wiring needed for the dashboard part).
  ============================================================================

  WHAT'S NEW vs. the OLED-only version:
      - All sensors are now read into a set of global variables every
        ~200ms, regardless of which OLED screen is showing.
      - Every ~200ms, one line of JSON is printed to Serial (115200 baud)
        with everything the dashboard needs. Example line:

          {"tiltX":2.31,"tiltY":-14.72,"tiltZ":8.05,"temp":26.4,
           "pressure":1011.3,"baroAlt":42.7,"ppm":480,"quality":"Good",
           "lat":26.912345,"lon":75.787345,"alt":431.2,"sats":7,"fix":true}

      - The OLED + button behavior is unchanged - it still cycles through
        MPU6050 / GPS / MQ-135 / BMP280 screens on each press. It just now
        reads from the same cached values instead of re-reading sensors
        itself, so nothing is read twice.
      - BMP280 has no humidity sensor (unlike BME280), so humidity has
        been dropped entirely. In its place, barometric altitude is now
        computed from pressure - see SEALEVEL_HPA_REFERENCE below if you
        want to calibrate it to your local QNH for more accurate altitude.

      - IMPORTANT: keep Serial.println() debug text OUT of loop() other
        than the JSON line - the dashboard expects each line it reads to
        be valid JSON, and stray text lines would fail to parse (they're
        just skipped, but keep it clean regardless).

  WIRING: identical to the previous version - see the wiring block below.
  ============================================================================

  OLED (SSD1306, I2C):      VCC->3.3V  GND->GND  SDA->21  SCL->22
  MPU6050 (I2C):            VCC->5V    GND->GND  SDA->21  SCL->22  AD0->GND
  BMP280 (I2C):              VCC->3.3V  GND->GND  SDA->21  SCL->22
  NEO-6M GPS (UART2):        VCC->5V    GND->GND  TX->16   RX->17
  MQ-135 (Analog):           VCC->3.3V  GND->GND  AOUT->34
  Push Button:               Leg1->27   Leg2->GND (internal pull-up)

  Libraries needed: Adafruit GFX, Adafruit SSD1306, TinyGPSPlus
  (MPU6050 and BMP280 use raw I2C registers - no library needed for either.
   Note: BMP280 is temperature + pressure only, no humidity register.)
  ============================================================================
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>

// ================= OLED =================
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT  64
#define OLED_RESET     -1
#define OLED_I2C_ADDR  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= MPU6050 (raw registers) =================
#define MPU_ADDR       0x68
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
const float ACCEL_SCALE = 16384.0;
const float GYRO_SCALE  = 131.0;
unsigned long lastTiltUpdate = 0;

// ================= GPS =================
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD   9600
HardwareSerial GPSserial(2);
TinyGPSPlus gps;

// ================= MQ-135 =================
#define MQ135_PIN 34
const float VOLTAGE_RESOLUTION = 3.3;
const float ADC_RESOLUTION     = 4095.0;
const float RL_VALUE = 20.0;
const float RO_CLEAN_AIR_FACTOR = 3.6;
const int   CALIBRATION_SAMPLES = 50;
const int   CALIBRATION_INTERVAL_MS = 500;
const float PARA = 116.6020682;
const float PARB = -2.769034857;
float Ro = 10.0;

// ================= BMP280 (raw registers) =================
#define BMP280_ADDR 0x76
uint16_t bmp_dig_T1; int16_t bmp_dig_T2, bmp_dig_T3;
uint16_t bmp_dig_P1; int16_t bmp_dig_P2, bmp_dig_P3, bmp_dig_P4, bmp_dig_P5, bmp_dig_P6, bmp_dig_P7, bmp_dig_P8, bmp_dig_P9;
int32_t  bmp_t_fine;

// Local sea-level pressure reference (hPa) used to convert pressure -> altitude.
// 1013.25 is the standard atmosphere default. For better accuracy, set this to
// the actual current QNH/sea-level pressure for your area/day if you know it.
const float SEALEVEL_HPA_REFERENCE = 1013.25;

// ================= Push button =================
#define BUTTON_PIN 27
int lastButtonReading = HIGH;
int stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// ================= Screen / timing state =================
int screenIndex = 0;
const int NUM_SCREENS = 4;
unsigned long lastOledRefresh = 0;
const unsigned long oledRefreshInterval = 500;
unsigned long lastSensorUpdate = 0;
const unsigned long sensorUpdateInterval = 200;   // also the JSON send rate

// ================= Cached latest readings (shared by OLED + JSON) =================
float g_tiltX = 0, g_tiltY = 0, g_tiltZ = 0;
float g_temp = 0, g_pressure = 0, g_baroAlt = 0;
float g_ppm = 0;
String g_quality = "Unknown";
double g_lat = 0, g_lon = 0, g_alt = 0;
int g_sats = 0;
bool g_fix = false;

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(MQ135_PIN, INPUT);

  GPSserial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    while (true) delay(10);   // OLED is required for this build - halt if missing
  }

  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(100);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Calibrating MQ-135..."));
  display.println(F("Keep in clean air"));
  display.display();
  Ro = calibrateMQ135();

  bmpWriteReg(0xF4, 0x27);
  bmpWriteReg(0xF5, 0xA0);
  bmpReadCalibration();

  lastTiltUpdate = millis();
  showStartupScreen();
}

void loop() {
  while (GPSserial.available() > 0) {
    gps.encode(GPSserial.read());
  }

  handleButton();

  unsigned long now = millis();

  if (now - lastSensorUpdate > sensorUpdateInterval) {
    lastSensorUpdate = now;
    updateAllSensors();
    sendTelemetryJSON();
  }

  if (now - lastOledRefresh > oledRefreshInterval) {
    lastOledRefresh = now;
    renderCurrentScreen();
  }
}

// ============================================================================
// Reads every sensor once and stores results in the g_* globals
// ============================================================================
void updateAllSensors() {
  // ---- MPU6050 tilt ----
  int16_t rawAccX, rawAccY, rawAccZ, rawGyroZ;
  readMPURaw(rawAccX, rawAccY, rawAccZ, rawGyroZ);

  float accX_g = rawAccX / ACCEL_SCALE;
  float accY_g = rawAccY / ACCEL_SCALE;
  float accZ_g = rawAccZ / ACCEL_SCALE;
  float gyroZ_dps = rawGyroZ / GYRO_SCALE;

  unsigned long now = millis();
  float dt = (now - lastTiltUpdate) / 1000.0;
  lastTiltUpdate = now;

  g_tiltX = atan2(accY_g, accZ_g) * 180.0 / PI;
  g_tiltY = atan2(-accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * 180.0 / PI;
  g_tiltZ += gyroZ_dps * dt;   // integrated estimate, drifts (no magnetometer)

  // ---- BMP280 ----
  g_temp     = bmpReadTemperature();
  g_pressure = bmpReadPressure();
  // Standard barometric formula (international altitude formula).
  g_baroAlt  = 44330.0 * (1.0 - pow(g_pressure / SEALEVEL_HPA_REFERENCE, 0.1903));

  // ---- MQ-135 ----
  float rs = getMQ135Resistance();
  float ratio = rs / Ro;
  g_ppm = PARA * pow(ratio, PARB);
  g_quality = classifyAirQuality(g_ppm);

  // ---- GPS ----
  g_fix = gps.location.isValid();
  if (g_fix) {
    g_lat = gps.location.lat();
    g_lon = gps.location.lng();
  }
  if (gps.altitude.isValid()) g_alt = gps.altitude.meters();
  g_sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

// ============================================================================
// Prints one JSON line with all current readings - this is what the laptop
// dashboard reads over the USB serial connection.
// ============================================================================
void sendTelemetryJSON() {
  Serial.print(F("{"));
  Serial.print(F("\"tiltX\":"));   Serial.print(g_tiltX, 2);
  Serial.print(F(",\"tiltY\":"));  Serial.print(g_tiltY, 2);
  Serial.print(F(",\"tiltZ\":"));  Serial.print(g_tiltZ, 2);
  Serial.print(F(",\"temp\":"));   Serial.print(g_temp, 1);
  Serial.print(F(",\"pressure\":")); Serial.print(g_pressure, 1);
  Serial.print(F(",\"baroAlt\":")); Serial.print(g_baroAlt, 1);
  Serial.print(F(",\"ppm\":"));    Serial.print(g_ppm, 0);
  Serial.print(F(",\"quality\":\"")); Serial.print(g_quality); Serial.print(F("\""));
  Serial.print(F(",\"lat\":"));    Serial.print(g_lat, 6);
  Serial.print(F(",\"lon\":"));    Serial.print(g_lon, 6);
  Serial.print(F(",\"alt\":"));    Serial.print(g_alt, 1);
  Serial.print(F(",\"sats\":"));   Serial.print(g_sats);
  Serial.print(F(",\"fix\":"));    Serial.print(g_fix ? "true" : "false");
  Serial.println(F("}"));
}

// ============================================================================
// BUTTON HANDLING
// ============================================================================
void handleButton() {
  int reading = digitalRead(BUTTON_PIN);
  if (reading != lastButtonReading) lastDebounceTime = millis();

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableButtonState) {
      stableButtonState = reading;
      if (stableButtonState == LOW) {
        screenIndex = (screenIndex + 1) % NUM_SCREENS;
        renderCurrentScreen();
      }
    }
  }
  lastButtonReading = reading;
}

void renderCurrentScreen() {
  switch (screenIndex) {
    case 0: showMPUScreen();   break;
    case 1: showGPSScreen();   break;
    case 2: showMQ135Screen(); break;
    case 3: showBMPScreen();   break;
  }
}

void showStartupScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Sensor Suite Ready"));
  display.println();
  display.println(F("Press button to"));
  display.println(F("cycle screens"));
  display.println();
  display.println(F("Streaming JSON on"));
  display.println(F("USB Serial @115200"));
  display.display();
}

// ---- OLED screens (render from cached g_* values, no sensor reads here) ----
void showMPUScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("-- MPU6050 Tilt --"));
  display.print(F("Tilt X: ")); display.print(g_tiltX, 1); display.println(F(" deg"));
  display.print(F("Tilt Y: ")); display.print(g_tiltY, 1); display.println(F(" deg"));
  display.print(F("Tilt Z: ")); display.print(g_tiltZ, 1); display.println(F(" deg"));
  display.println();
  display.println(F("(Z is gyro-estimated,"));
  display.println(F("drifts over time)"));
  display.display();
}

void showGPSScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("-- GPS (NEO-6M) --"));
  if (g_fix) {
    display.print(F("Lat: ")); display.println(g_lat, 6);
    display.print(F("Lon: ")); display.println(g_lon, 6);
  } else {
    display.println(F("Lat: no fix"));
    display.println(F("Lon: no fix"));
  }
  display.print(F("Alt: ")); display.print(g_alt, 1); display.println(F(" m"));
  display.print(F("Sats: ")); display.println(g_sats);
  if (!g_fix) display.println(F("Searching..."));
  display.display();
}

void showMQ135Screen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("-- MQ-135 Air --"));
  display.print(F("PPM (est): ")); display.println(g_ppm, 0);
  display.print(F("Quality: ")); display.println(g_quality);
  display.println();
  display.println(F("(Approx. reading,"));
  display.println(F("not official AQI)"));
  display.display();
}

void showBMPScreen() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("-- BMP280 --"));
  display.print(F("Temp: ")); display.print(g_temp, 1); display.println(F(" C"));
  display.print(F("Pressure: ")); display.print(g_pressure, 1); display.println(F(" hPa"));
  display.print(F("Baro Alt: ")); display.print(g_baroAlt, 1); display.println(F(" m"));
  display.display();
}

// ============================================================================
// MPU6050 - raw register access
// ============================================================================
void writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readMPURaw(int16_t &accX, int16_t &accY, int16_t &accZ, int16_t &gyroZ) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();
  accZ = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();   // discard temperature
  Wire.read(); Wire.read();   // gyro X (unused)
  Wire.read(); Wire.read();   // gyro Y (unused)
  gyroZ = (Wire.read() << 8) | Wire.read();
}

// ============================================================================
// MQ-135
// ============================================================================
float getMQ135Resistance() {
  long sum = 0;
  const int samples = 5;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(MQ135_PIN);
    delay(10);
  }
  float raw = sum / (float)samples;
  float voltage = raw * (VOLTAGE_RESOLUTION / ADC_RESOLUTION);
  if (voltage <= 0.0) voltage = 0.01;
  return ((VOLTAGE_RESOLUTION * RL_VALUE) / voltage) - RL_VALUE;
}

float calibrateMQ135() {
  float total = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    total += getMQ135Resistance();
    delay(CALIBRATION_INTERVAL_MS);
  }
  return (total / CALIBRATION_SAMPLES) / RO_CLEAN_AIR_FACTOR;
}

String classifyAirQuality(float ppm) {
  if (ppm < 400)  return "Good";
  if (ppm < 1000) return "Moderate";
  if (ppm < 2000) return "Poor";
  if (ppm < 5000) return "Unhealthy";
  return "Hazardous";
}

// ============================================================================
// BMP280 - raw register access + Bosch compensation formulas
// ============================================================================
void bmpWriteReg(byte reg, byte val) {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t bmpReadReg(byte reg) {
  Wire.beginTransmission(BMP280_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDR, 1);
  return Wire.read();
}

void bmpReadCalibration() {
  bmp_dig_T1 = bmpReadReg(0x88) | (bmpReadReg(0x89) << 8);
  bmp_dig_T2 = bmpReadReg(0x8A) | (bmpReadReg(0x8B) << 8);
  bmp_dig_T3 = bmpReadReg(0x8C) | (bmpReadReg(0x8D) << 8);

  bmp_dig_P1 = bmpReadReg(0x8E) | (bmpReadReg(0x8F) << 8);
  bmp_dig_P2 = bmpReadReg(0x90) | (bmpReadReg(0x91) << 8);
  bmp_dig_P3 = bmpReadReg(0x92) | (bmpReadReg(0x93) << 8);
  bmp_dig_P4 = bmpReadReg(0x94) | (bmpReadReg(0x95) << 8);
  bmp_dig_P5 = bmpReadReg(0x96) | (bmpReadReg(0x97) << 8);
  bmp_dig_P6 = bmpReadReg(0x98) | (bmpReadReg(0x99) << 8);
  bmp_dig_P7 = bmpReadReg(0x9A) | (bmpReadReg(0x9B) << 8);
  bmp_dig_P8 = bmpReadReg(0x9C) | (bmpReadReg(0x9D) << 8);
  bmp_dig_P9 = bmpReadReg(0x9E) | (bmpReadReg(0x9F) << 8);
}

float bmpReadTemperature() {
  int32_t adc_T = ((uint32_t)bmpReadReg(0xFA) << 12) |
                  ((uint32_t)bmpReadReg(0xFB) << 4)  |
                  (bmpReadReg(0xFC) >> 4);

  int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_dig_T1 << 1))) * bmp_dig_T2) >> 11;
  int32_t var2 = (((((adc_T >> 4) - (int32_t)bmp_dig_T1) *
                    ((adc_T >> 4) - (int32_t)bmp_dig_T1)) >> 12) * bmp_dig_T3) >> 14;
  bmp_t_fine = var1 + var2;
  return (bmp_t_fine * 5 + 128) / 25600.0;
}

float bmpReadPressure() {
  int32_t adc_P = ((uint32_t)bmpReadReg(0xF7) << 12) |
                  ((uint32_t)bmpReadReg(0xF8) << 4)  |
                  (bmpReadReg(0xF9) >> 4);

  int64_t var1 = (int64_t)bmp_t_fine - 128000;
  int64_t var2 = var1 * var1 * bmp_dig_P6;
  var2 = var2 + ((var1 * bmp_dig_P5) << 17);
  var2 = var2 + ((int64_t)bmp_dig_P4 << 35);
  var1 = ((var1 * var1 * bmp_dig_P3) >> 8) + ((var1 * bmp_dig_P2) << 12);
  var1 = (((int64_t)1 << 47) + var1) * bmp_dig_P1 >> 33;
  if (var1 == 0) return 0;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = ((int64_t)bmp_dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = ((int64_t)bmp_dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + ((int64_t)bmp_dig_P7 << 4);
  return p / 25600.0;
}
