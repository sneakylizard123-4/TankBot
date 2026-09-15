#include <Arduino.h>

#if defined(ESP32)
  #include <WiFi.h>
  #include <WebServer.h>
  typedef WebServer HttpServer;
#else
  #include <ESP8266WiFi.h>
  #include <ESP8266WebServer.h>
  typedef ESP8266WebServer HttpServer;
#endif

// ============================================================================
//  Wemos D1 TankBot - WiFi AP + web D-pad controller for an L298N (blue board)
//
//  The board creates its own WiFi network:
//     SSID : TankBot
//     pass : 12345678
//  Connect a phone/laptop to it, open  http://192.168.4.1  and drive the tank.
//
//  L298N wiring (D1 -> L298N):
//    D13 (GPIO14) -> IN1   (motor A direction 1)
//    D12 (GPIO12) -> IN2   (motor A direction 2)
//    D11 (GPIO13) -> IN3   (motor B direction 1)
//    D10 (GPIO15) -> IN4   (motor B direction 2)
//    ENA, ENB    -> 5V (jumper, always on -> full speed)
//    GND         -> GND shared with the board
//    7-12V       -> VMS motor supply
//
//  (The SPI row is often labeled D5-D8 = SCK/MISO/MOSI/SS; the pins are the
//   same GPIOs: D5=14, D6=12, D7=13, D8=15.)
//
//  Edit the pin numbers below to match your wiring.
// ============================================================================

#define PIN_IN1 14
#define PIN_IN2 12
#define PIN_IN3 13
#define PIN_IN4 15

#define AP_SSID "TankBot"
#define AP_PASS "12345678"

#define MOTOR_A 0                       // e.g. left
#define MOTOR_B 1                       // e.g. right

static HttpServer server(80);

static void printResetInfo(const char* tag) {
  Serial.print("[");
  Serial.print(tag);
  Serial.print("] reset reason=");
  Serial.println(ESP.getResetReason());
  Serial.print("[");
  Serial.print(tag);
  Serial.print("] reset info=");
  Serial.println(ESP.getResetInfo());
}

static void motorForward(int motor) {
  if (motor == MOTOR_A) { digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, LOW); }
  else                 { digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW); }
}

static void motorBackward(int motor) {
  if (motor == MOTOR_A) { digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH); }
  else                 { digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH); }
}

static void motorStop(int motor) {
  if (motor == MOTOR_A) { digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); }
  else                 { digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); }
}

static void driveFwd()  { motorForward(MOTOR_A); motorForward(MOTOR_B); }
static void driveBack() { motorBackward(MOTOR_A); motorBackward(MOTOR_B); }
static void driveLeft() { motorForward(MOTOR_A); motorBackward(MOTOR_B); }   // spin left
static void driveRight(){ motorBackward(MOTOR_A); motorForward(MOTOR_B); }   // spin right
static void driveStop() { motorStop(MOTOR_A); motorStop(MOTOR_B); }

static void helloPage() {
  static const char PAGE[] = R"HTML(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body{margin:0;background:#222;color:#fff;font-family:sans-serif;text-align:center}
h1{font-size:18px;padding:10px}
.pad{display:grid;grid-template:repeat(2,110px)/repeat(3,100px);gap:8px;justify-content:center}
button{border:none;border-radius:14px;background:#3a6ea5;color:#fff;font-size:30px}
button:active{background:#ff9800}
.up{grid-column:2;grid-row:1}
.left{grid-column:1;grid-row:2}
.back{grid-column:2;grid-row:2}
.right{grid-column:3;grid-row:2}
.stop{width:90%;height:70px;margin-top:14px;background:#a53a3a;font-size:26px}
</style></head><body>
<h1>Wemos D1 TankBot</h1>
<div class="pad">
<button class="up"   onpointerdown="go('fwd')">&#9650;</button>
<button class="left" onpointerdown="go('left')">&#9664;</button>
<button class="back" onpointerdown="go('back')">&#9660;</button>
<button class="right"onpointerdown="go('right')">&#9654;</button>
</div>
<button class="stop" onpointerdown="go('stop')">STOP</button>
<script>function go(c){fetch('/cmd/'+c).catch(function(){});}</script>
</body></html>)HTML";
  server.send(200, "text/html", PAGE);
}

static void sendCmd() {
  String c = server.uri();
  c.replace("/cmd/", "");
  if      (c == "fwd")  driveFwd();
  else if (c == "back") driveBack();
  else if (c == "left") driveLeft();
  else if (c == "right")driveRight();
  else if (c == "stop") driveStop();
  server.send(200, "text/plain", "ok");
}

void setup(void) {
  Serial.begin(115200);
  Serial.println();
  Serial.println(F("=== TankBot boot ==="));
  printResetInfo("setup");

  Serial.print(F("flash:"));
  Serial.println(ESP.getFlashChipRealSize());
  Serial.print(F("free heap:"));
  Serial.println(ESP.getFreeHeap());

  for (int p : { PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 }) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
  Serial.println(F("motor pins set LOW"));

  Serial.print(F("WiFi.mode(AP) "));
  WiFi.mode(WIFI_AP);
  Serial.println(F("done"));

  Serial.print(F("softAP start: "));
  bool ok = WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println(ok ? F("OK") : F("FAILED"));

  Serial.print(F("server.begin "));
  server.on("/", helloPage);
  server.on("/cmd/fwd", sendCmd);
  server.on("/cmd/back", sendCmd);
  server.on("/cmd/left", sendCmd);
  server.on("/cmd/right", sendCmd);
  server.on("/cmd/stop", sendCmd);
  server.begin();
  Serial.println(F("done"));

  Serial.println(F("TankBot AP ready"));
  Serial.println(F("L298N IN1/2/3/4 -> GPIO 14/12/13/15 (D13/D12/D11/D10)"));
  Serial.print(F("SSID: ")); Serial.println(AP_SSID);
  Serial.print(F("pass: ")); Serial.println(AP_PASS);
  Serial.print(F("Open http://"));
  Serial.println(WiFi.softAPIP());
  delay(500);
}

static unsigned long lastTicker = 0;

void loop(void) {
  unsigned long now = millis();
  if (now - lastTicker >= 2000) {
    lastTicker = now;
    Serial.print("alive t=");
    Serial.print(now);
    Serial.print("ms heap=");
    Serial.print(ESP.getFreeHeap());
    Serial.print(" resetreason=");
    Serial.println(ESP.getResetReason());
  }
  server.handleClient();
  delay(5);
}