#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);              // รอ USB CDC enumerate ไม่งั้นบรรทัดแรกหาย
  Serial.println("ESP32-S3 alive");
  Serial.printf("PSRAM: %d bytes\n", ESP.getPsramSize());
}

void loop() {
  Serial.println(millis());
  delay(1000);
}