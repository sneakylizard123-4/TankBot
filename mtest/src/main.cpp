#include <Arduino.h>

// Motor channel sweep test. Same L298N pins as the tank sketch.
// Watches: which motor moves, and in which direction, per phase.

#define PIN_IN1 14
#define PIN_IN2 12
#define PIN_IN3 13
#define PIN_IN4 15

#define ON 1.6f
#define OFF 0.6f

static void setA(int a, int b) { digitalWrite(PIN_IN1, a); digitalWrite(PIN_IN2, b); }
static void setB(int a, int b) { digitalWrite(PIN_IN3, a); digitalWrite(PIN_IN4, b); }
static void allLow() { setA(LOW, LOW); setB(LOW, LOW); }

void setup() {
  Serial.begin(115200);
  for (int p : { PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 }) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  Serial.println("=== motor sweep ===");
  Serial.println("phase list: A-FWD / B-FWD / A-BACK / B-BACK / A-BWD / B-BWD");
}

void loop() {
  Serial.println("> A-FWD"); setA(HIGH, LOW);           delay((int)(ON * 1000)); allLow();
  Serial.println("> A-BACK"); setA(LOW, HIGH);          delay((int)(ON * 1000)); allLow();
  delay((int)(OFF * 1000));
  Serial.println("> B-FWD"); setB(HIGH, LOW);           delay((int)(ON * 1000)); allLow();
  Serial.println("> B-BACK"); setB(LOW, HIGH);          delay((int)(ON * 1000)); allLow();
  delay((int)(OFF * 1000));
}