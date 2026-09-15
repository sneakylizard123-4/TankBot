// pins-only test: no WiFi libs at all. If this loops -> hardware/wiring.
#include <Arduino.h>

#define PIN_IN1 14
#define PIN_IN2 12
#define PIN_IN3 13
#define PIN_IN4 15

void setup(void) {
  Serial.begin(115200);
  Serial.println();
  Serial.println("=== pinstest boot ===");
  Serial.print("reset reason=");
  Serial.println(ESP.getResetReason());

  for (int pin : { PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 }) {
    Serial.print("init gpio ");
    Serial.println(pin);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    Serial.print("gpio ");
    Serial.print(pin);
    Serial.println(" ok");
  }
  Serial.println("ALL PINS OK");
}

void loop(void) {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 1000) {
    last = now;
    Serial.print("alive ");
    Serial.print(now);
    Serial.print("ms heap=");
    Serial.println(ESP.getFreeHeap());
  }
  yield();
}