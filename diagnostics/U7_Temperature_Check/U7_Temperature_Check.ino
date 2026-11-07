/* U7 standalone DS18B20 checkout. Replaces the normal U7 firmware while testing.
   TS1 GPIO32, TS2 GPIO13. Serial Monitor 115200 baud.
   Requires OneWire and DallasTemperature libraries. */
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>

constexpr uint8_t TS1_PIN = 32, TS2_PIN = 13;
OneWire bus1(TS1_PIN), bus2(TS2_PIN);
DallasTemperature ts1(&bus1), ts2(&bus2);

bool valid(float c) {
  return isfinite(c) && c != DEVICE_DISCONNECTED_C && c != 85.0f &&
         c >= -10.0f && c <= 100.0f;
}
void setup() {
  Serial.begin(115200);
  ts1.begin(); ts2.begin();
  ts1.setResolution(12); ts2.setResolution(12);
  Serial.printf("TS1 devices=%d, TS2 devices=%d\n",
                ts1.getDeviceCount(), ts2.getDeviceCount());
  Serial.println("Time_ms,TS1_C,TS2_C,Delta_C");
}
void loop() {
  ts1.requestTemperatures(); ts2.requestTemperatures();
  float a = ts1.getTempCByIndex(0), b = ts2.getTempCByIndex(0);
  if (valid(a) && valid(b))
    Serial.printf("%lu,%.2f,%.2f,%.2f\n", millis(), a, b, fabsf(a-b));
  else
    Serial.printf("%lu,INVALID (TS1=%.2f TS2=%.2f)\n", millis(), a, b);
  delay(250);
}
