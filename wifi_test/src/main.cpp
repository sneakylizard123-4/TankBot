#include <Arduino.h>
#include <ESP8266WiFi.h>

#define PIN_IN1 14
#define PIN_IN2 12
#define PIN_IN3 13
#define PIN_IN4 15

#define AP_SSID "TankBot"
#define AP_PASS "12345678"

void setup(void) {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== wifi_test boot ===");
  Serial.print("reset reason=");
  Serial.println(ESP.getResetReason());

  for (int pin : { PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 }) {
    Serial.print("init gpio ");
    Serial.println(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  Serial.println("pins ok");

  Serial.print("WiFi.mode(AP) ");
  WiFi.mode(WIFI_AP);
  Serial.println("done");

  Serial.print("softAP start: ");
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println(ok ? "OK" : "FAILED");

  Serial.print("open http://");
  Serial.println(WiFi.softAPIP());

  WiFi.setOutputPower(20.5);
  Serial.println("setup done");
}

void loop(void) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 2000) {
    last = now;
    Serial.print("alive ");
    Serial.print(now);
    Serial.print("ms heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" stations=");
    Serial.println(WiFi.softAPgetStationNum());
  }
  delay(5);
}