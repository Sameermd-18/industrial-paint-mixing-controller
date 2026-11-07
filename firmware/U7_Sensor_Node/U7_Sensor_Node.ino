/* ESP32 #2 (U7): flow and DS18B20 telemetry to U2 over HTTP.
   Set WIFI_PASSWORD to the same local password configured on U2.
   FS1 G33 blue/flush, FS2 G25 yellow, FS3 G26 green.
   TS1 G32 yellow, TS2 G13 green.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

constexpr uint8_t FS_BLUE_FLUSH = 33, FS_YELLOW = 25, FS_GREEN = 26;
constexpr uint8_t PIN_TS1_YELLOW = 32, PIN_TS2_GREEN = 13;
constexpr char WIFI_NAME[] = "Lam_Teal_Mixer";
constexpr char WIFI_PASSWORD[] = "CHANGE_ME_123";
constexpr char FLOW_URL[] = "http://192.168.4.1/api/flow";
constexpr uint32_t SEND_INTERVAL_MS = 250, LOG_INTERVAL_MS = 1000;
constexpr uint32_t RETRY_INTERVAL_MS = 3000, TEMP_PERIOD_MS = 1000;
constexpr uint32_t TEMP_CONV_MS = 200;

volatile uint32_t blueFlushPulses = 0, yellowPulses = 0, greenPulses = 0;
OneWire owYellow(PIN_TS1_YELLOW), owGreen(PIN_TS2_GREEN);
DallasTemperature tsYellow(&owYellow), tsGreen(&owGreen);
bool dualOnPin32 = false;
float tempYellow = NAN, tempGreen = NAN;
bool tempRequested = false;
uint32_t lastTempReq = 0, bootId = 0, lastSend = 0, lastLog = 0, lastRetry = 0;
int lastHttpStatus = 0;

void IRAM_ATTR countBlueFlush() { ++blueFlushPulses; }
void IRAM_ATTR countYellow() { ++yellowPulses; }
void IRAM_ATTR countGreen() { ++greenPulses; }

void updateTemps(uint32_t now) {
  if (!tempRequested && now - lastTempReq >= TEMP_PERIOD_MS) {
    tsYellow.requestTemperatures();
    if (!dualOnPin32) tsGreen.requestTemperatures();
    lastTempReq = now;
    tempRequested = true;
  } else if (tempRequested && now - lastTempReq >= TEMP_CONV_MS) {
    if (dualOnPin32) {
      float t1 = tsYellow.getTempCByIndex(0);
      float t2 = tsYellow.getTempCByIndex(1);
      tempYellow = t1 == DEVICE_DISCONNECTED_C ? NAN : t1;
      tempGreen  = t2 == DEVICE_DISCONNECTED_C ? NAN : t2;
    } else {
      float y = tsYellow.getTempCByIndex(0);
      float g = tsGreen.getTempCByIndex(0);
      tempYellow = y == DEVICE_DISCONNECTED_C ? NAN : y;
      tempGreen  = g == DEVICE_DISCONNECTED_C ? NAN : g;
    }
    tempRequested = false;
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("U7 SENSOR NODE — INITIALIZING BUSSES");
  pinMode(PIN_TS1_YELLOW, INPUT_PULLUP);
  pinMode(PIN_TS2_GREEN, INPUT_PULLUP);
  pinMode(FS_BLUE_FLUSH, INPUT);
  pinMode(FS_YELLOW, INPUT);
  pinMode(FS_GREEN, INPUT);
  attachInterrupt(digitalPinToInterrupt(FS_BLUE_FLUSH), countBlueFlush, RISING);
  attachInterrupt(digitalPinToInterrupt(FS_YELLOW), countYellow, RISING);
  attachInterrupt(digitalPinToInterrupt(FS_GREEN), countGreen, RISING);
  tsYellow.begin(); tsGreen.begin();
  tsYellow.setResolution(10); tsGreen.setResolution(10);
  tsYellow.setWaitForConversion(false); tsGreen.setWaitForConversion(false);
  int countY = tsYellow.getDeviceCount(), countG = tsGreen.getDeviceCount();
  Serial.printf("GPIO32: %d device(s), GPIO13: %d device(s)\n", countY, countG);
  if (countG == 0 && countY >= 2) {
    dualOnPin32 = true;
    Serial.println("Both sensors found on GPIO32; index 0=yellow, index 1=green");
  } else if (countG == 0) {
    Serial.println("WARNING: no sensor detected on GPIO13; check wiring and pull-up");
  }
  bootId = esp_random();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);
  Serial.println("Connecting to U2 Wi-Fi...");
}

void loop() {
  uint32_t now = millis();
  updateTemps(now);
  if (WiFi.status() != WL_CONNECTED && now - lastRetry >= RETRY_INTERVAL_MS) {
    lastRetry = now;
    WiFi.reconnect();
    Serial.println("U7: reconnecting to U2 Wi-Fi...");
  }
  uint32_t blue, yellow, green;
  noInterrupts();
  blue = blueFlushPulses;
  yellow = yellowPulses;
  green = greenPulses;
  interrupts();
  if (WiFi.status() == WL_CONNECTED && now - lastSend >= SEND_INTERVAL_MS) {
    lastSend = now;
    HTTPClient http;
    http.setTimeout(650);
    http.begin(FLOW_URL);
    http.addHeader("Content-Type", "application/x-www-form-urlencoded");
    String body = "boot=" + String(bootId) + "&b=" + String(blue)
                + "&y=" + String(yellow) + "&g=" + String(green);
    if (!isnan(tempYellow)) body += "&ty=" + String(tempYellow, 2);
    if (!isnan(tempGreen))  body += "&tg=" + String(tempGreen, 2);
    lastHttpStatus = http.POST(body);
    http.end();
  }
  if (now - lastLog >= LOG_INTERVAL_MS) {
    lastLog = now;
    Serial.printf("FS1 Blue: %lu | FS2 Yellow: %lu | FS3 Green: %lu | TS1: %s | TS2: %s | WiFi: %s | HTTP: %d\n",
      (unsigned long)blue, (unsigned long)yellow, (unsigned long)green,
      isnan(tempYellow) ? "DISCONNECTED" : (String(tempYellow, 1) + " C").c_str(),
      isnan(tempGreen) ? "DISCONNECTED" : (String(tempGreen, 1) + " C").c_str(),
      WiFi.status() == WL_CONNECTED ? "OK" : "NO-WIFI", lastHttpStatus);
  }
  delay(1);
}
