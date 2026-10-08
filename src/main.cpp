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

#include <WebSocketsServer.h>

// ============================================================================
//  Wemos D1 TankBot - WiFi AP + web D-pad controller for an L298N (blue board)
//
//  The board creates its own WiFi network:
//     SSID : TankBot
//     pass : 12345678
//  Connect a phone/laptop to it, open  http://192.168.4.1  and drive the tank.
//
//  Drive control: HOLD-to-drive over a WebSocket (port 81). The web page pushes
//  the held command every 100ms; the tank auto-stops if ~350ms pass without a
//  command (lost packet / phone dropped / tab hidden).
//
//  Optional ELRS control (ExpressLRS receiver -> CRSF serial):
//    receiver TX pad -> D1 (GPIO3)   [ESP UART0 RX @ 420000 baud]
//    receiver RX pad -> D4 (GPIO2)   [ESP UART1 TX, idle-high; telemetry-ready]
//  RC channels 1(steer) & 3(throttle), AETR mix, centre ~992. Sticks past the
//  deadband drive the tank (incl. diagonals). RC takes priority over the web
//  pad while a stick is pushed; centre the sticks to give the web control back.
//  CRC-8 is verified per frame; a lost RC link auto-stops in ~350ms.
//
//  L298N wiring (D1 -> L298N):
//    D5 (GPIO14) -> IN1   (motor A direction 1)
//    D6 (GPIO12) -> IN2   (motor A direction 2)
//    D7 (GPIO13) -> IN3   (motor B direction 1)
//    D8 (GPIO15) -> IN4   (motor B direction 2)
//    ENA, ENB    -> 5V (jumper, always on -> full speed)
//    GND         -> GND shared with the board
//    7-12V       -> VMS motor supply
//
//  Note: GPIO15 must be LOW at reset (boot strapping pin) - check the L298N
//  input does not hold D8 high or the board may fail to boot.
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

#define WS_PORT 81                      // WebSocket server (heartbeat control)
#define CMD_TIMEOUT_MS 350              // stop motors if no command in this long

// ---- ELRS / CRSF -----------------------------------------------------------
#define CRSF_BAUD       420000          // default ExpressLRS CRSF baud rate
#define CRSF_SYNC       0xC8            // RC receiver -> flight controller frame
#define CRSF_FRAME_RC   0x16            // RC channels packed frame type
#define RC_CH_STEER     0               // 0-based channel (ELRS AETR => ch1)
#define RC_CH_THROTTLE  2               // 0-based channel (ELRS AETR => ch3)
#define RC_CENTER       992             // 172..1811 range, centre value
#define RC_DEAD         100             // deadband around centre (~60us)
#define RC_HOLD_MS      700             // how long RC keeps control after a push

// ELRS receiver wiring (values kept for clarity; the UART pins are hardwired:
// UART0 RX is always GPIO3, UART1 TX is always GPIO2).
#define ELRS_RX_PIN     3               // ELRS receiver TX pad -> D1 GPIO3 (board "RX")
#define ELRS_TX_PIN     2               // ELRS receiver RX pad -> D1 GPIO2 (board D4)

static HttpServer server(80);
static WebSocketsServer webSocket(WS_PORT);

static String activeCmd = "stop";
static unsigned long lastCmdMs = 0;
static unsigned long rcActiveUntil = 0;

// ---- ELRS / CRSF state (elrsPage() HTTP route needs it early) --------------
static uint32_t crsfGood = 0, crsfBad = 0, crsfOther = 0, crsfFrames = 0;
static uint32_t crsfBytes = 0, crsfSyncs = 0;
static uint16_t crsfCh[16];
static unsigned long lastCrsfMs = 0;    // last valid frame
static bool rcLinkAlive = false;        // receiving valid CRSF frames

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
  else                  { digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, LOW); }
}

static void motorBackward(int motor) {
  if (motor == MOTOR_A) { digitalWrite(PIN_IN1, LOW);  digitalWrite(PIN_IN2, HIGH); }
  else                  { digitalWrite(PIN_IN3, LOW);  digitalWrite(PIN_IN4, HIGH); }
}

static void motorCmd(int motor, int dir) {          // dir: +1 fwd, -1 back, 0 stop
  if (dir > 0)      motorForward(motor);
  else if (dir < 0) motorBackward(motor);
  else              { digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW);
                      if (motor == MOTOR_B) { digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW); } }
}

static void driveFwd()      { motorCmd(MOTOR_A,  1); motorCmd(MOTOR_B,  1); }
static void driveBack()     { motorCmd(MOTOR_A, -1); motorCmd(MOTOR_B, -1); }
static void driveLeft()     { motorCmd(MOTOR_A,  1); motorCmd(MOTOR_B, -1); }   // spin left
static void driveRight()    { motorCmd(MOTOR_A, -1); motorCmd(MOTOR_B,  1); }   // spin right
static void driveFwdLeft()  { motorCmd(MOTOR_A,  1); motorCmd(MOTOR_B,  0); }   // curve left
static void driveFwdRight() { motorCmd(MOTOR_A,  0); motorCmd(MOTOR_B,  1); }   // curve right
static void driveBackLeft() { motorCmd(MOTOR_A,  0); motorCmd(MOTOR_B, -1); }
static void driveBackRight(){ motorCmd(MOTOR_A, -1); motorCmd(MOTOR_B,  0); }
static void driveStop()     { motorCmd(MOTOR_A,  0); motorCmd(MOTOR_B,  0); }

static void applyCommand(const String& c, bool fromRc = false) {
  if (!fromRc && millis() < rcActiveUntil) return;   // RC currently has control
  if (fromRc && c != "stop") rcActiveUntil = millis() + RC_HOLD_MS;
  if      (c == "fwd")   driveFwd();
  else if (c == "back")  driveBack();
  else if (c == "left")  driveLeft();
  else if (c == "right") driveRight();
  else if (c == "fwdleft")  driveFwdLeft();
  else if (c == "fwdright") driveFwdRight();
  else if (c == "backleft") driveBackLeft();
  else if (c == "backright")driveBackRight();
  else if (c == "stop")  driveStop();
  else return;                          // unknown - keep previous failsafe state
  activeCmd = c;
  lastCmdMs = millis();
}

static void helloPage() {
  server.client().setNoDelay(true);
  static const char PAGE[] = R"HTML(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<style>
body{margin:0;background:#222;color:#fff;font-family:sans-serif;text-align:center;touch-action:none;-webkit-touch-callout:none;-webkit-user-select:none;user-select:none}
h1{font-size:18px;padding:10px}
.pad{display:grid;grid-template:repeat(2,110px)/repeat(3,100px);gap:8px;justify-content:center}
button{border:none;border-radius:14px;background:#3a6ea5;color:#fff;font-size:30px;touch-action:none;-webkit-user-select:none;user-select:none}
button:active{background:#ff9800}
.up{grid-column:2;grid-row:1}
.left{grid-column:1;grid-row:2}
.back{grid-column:2;grid-row:2}
.right{grid-column:3;grid-row:2}
.stop{width:90%;height:70px;margin-top:14px;background:#a53a3a;font-size:26px}
</style></head><body>
<h1>Wemos D1 TankBot</h1>
<div class="pad">
<button class="up"   data-cmd="fwd">&#9650;</button>
<button class="left" data-cmd="left">&#9664;</button>
<button class="back" data-cmd="back">&#9660;</button>
<button class="right"data-cmd="right">&#9654;</button>
</div>
</body></html>)HTML";
  server.send(200, "text/html", PAGE);
}

static void sendCmd() {
  server.client().setNoDelay(true);
  String c = server.uri();
  c.replace("/cmd/", "");
  applyCommand(c);
  server.send(200, "text/plain", "ok");
}

static void elrsPage() {
  server.client().setNoDelay(true);
  String out;
  out  = "crsf bytes="; out += crsfBytes;
  out += " syncs=";     out += crsfSyncs;
  out += " good=";      out += crsfGood;
  out += " other=";     out += crsfOther;
  out += " bad=";       out += crsfBad;
  out += " frames=";    out += crsfFrames;
  out += "\nlink=";     out += (millis() - lastCrsfMs < 1000) ? "alive" : (rcLinkAlive ? "lost<1s" : "down");
  out += " cmd=";       out += activeCmd;
  out += "\nCH1:";      out += crsfCh[0];
  for (int c = 1; c < 16; c++) { out += " CH"; out += c + 1; out += ":"; out += crsfCh[c]; }
  server.send(200, "text/plain", out);
}

static void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[ws] #%u connected from ", num);
      Serial.println(webSocket.remoteIP(num).toString());
      driveStop();
      activeCmd = "stop";
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[ws] #%u disconnected\n", num);
      driveStop();
      activeCmd = "stop";
      break;
    case WStype_TEXT: {
      String c;
      c.concat((const char*)payload, length);
      applyCommand(c);
      break;
    }
    default:
      break;
  }
}

// ---- CRSF (ExpressLRS receiver) --------------------------------------------
static uint8_t crsfState = 0;           // 0=sync, 1=length, 2=data
static uint8_t crsfLen = 0, crsfIdx = 0;
static uint8_t crsfBuf[64];
static String lastRcCmd = "stop";       // last command derived from the sticks

static uint8_t crsfCrc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void rcToCommand(const uint16_t ch[]) {
  int thr = ch[RC_CH_THROTTLE];
  int st  = ch[RC_CH_STEER];
  bool fwd  = thr > RC_CENTER + RC_DEAD;
  bool back = thr < RC_CENTER - RC_DEAD;
  bool left = st  < RC_CENTER - RC_DEAD;
  bool right= st  > RC_CENTER + RC_DEAD;

  const char* cmd;
  if      (fwd  && left)  cmd = "fwdleft";
  else if (fwd  && right) cmd = "fwdright";
  else if (back && left)  cmd = "backleft";
  else if (back && right) cmd = "backright";
  else if (fwd)           cmd = "fwd";
  else if (back)          cmd = "back";
  else if (left)          cmd = "left";
  else if (right)         cmd = "right";
  else                    cmd = "stop";
  applyCommand(cmd, true);

  if (lastRcCmd == cmd) return;                       // nothing changed, stay quiet
  if (cmd == "stop") {
    Serial.println(F("[rc] released control"));
  } else {
    if (lastRcCmd == "stop") Serial.println(F("[rc] acquired control"));
    Serial.printf("[rc] %s  steer=%u thr=%u\n", cmd, ch[RC_CH_STEER], ch[RC_CH_THROTTLE]);
  }
  lastRcCmd = cmd;
}

static void crsfPump() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    crsfBytes++;
    switch (crsfState) {
      case 0:                                        // waiting for sync 0xC8
        if (b == CRSF_SYNC) { crsfSyncs++; crsfState = 1; }
        break;
      case 1:                                        // read length byte
        if (b < 4 || b > 60) { crsfState = 0; break; }
        crsfLen = b; crsfIdx = 0; crsfState = 2;
        break;
      case 2:                                        // read frame body
        crsfBuf[crsfIdx++] = b;
        if (crsfIdx >= crsfLen) {
          crsfState = 0;
          crsfFrames++;
          if (crsfCrc8(crsfBuf, crsfLen - 1) != crsfBuf[crsfLen - 1]) {
            crsfBad++;
            break;
          }
          if (crsfBuf[0] != CRSF_FRAME_RC) {        // valid frame, e.g. link stats 0x14
            crsfOther++;
            break;
          }
          crsfGood++;
          if (!rcLinkAlive) {
            rcLinkAlive = true;
            Serial.println(F("[rc] link acquired (valid CRSF frames)"));
          }
          lastCrsfMs = millis();
          // payload starts at crsfBuf[1] (22 bytes = 16ch * 11bit, LE-packed)
          for (int c = 0; c < 16; c++) {
            uint8_t b0 = (c * 11) >> 3, bi = (c * 11) & 7;
            uint16_t v = (crsfBuf[1 + b0] >> bi) | (crsfBuf[2 + b0] << (8 - bi));
            if (bi > 5) v |= crsfBuf[3 + b0] << (16 - bi);
            crsfCh[c] = v & 0x7FF;
          }
          rcToCommand(crsfCh);
        }
        break;
    }
  }
}

void setup(void) {
  Serial.begin(CRSF_BAUD);              // UART0: RX=GPIO3 feeds ELRS (TX=GPIO1 still debugs to USB)
  Serial.println();
  Serial.println(F("=== TankBot boot ==="));
  printResetInfo("setup");

  Serial1.begin(CRSF_BAUD, SERIAL_8N1, SERIAL_TX_ONLY, ELRS_TX_PIN);   // drives ELRS RX pad (GPIO2) idle-high
  Serial.print(F("crsf baud:"));
  Serial.println(CRSF_BAUD);
  Serial.print(F("elrs pins: RX=GPIO"));
  Serial.print(ELRS_RX_PIN);
  Serial.print(F("  TX=GPIO"));
  Serial.println(ELRS_TX_PIN);

  Serial.print(F("flash:"));
  Serial.println(ESP.getFlashChipRealSize());
  Serial.print(F("free heap:"));
  Serial.println(ESP.getFreeHeap());

  static const int motorPins[] = { PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4 };
  for (size_t i = 0; i < sizeof(motorPins) / sizeof(motorPins[0]); i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  Serial.println(F("motor pins set LOW"));

#if defined(ESP8266)
  WiFiClient::setDefaultNoDelay(true);  // disable Nagle -> snappy responses
#endif
  Serial.println(F("TCP_NODELAY on"));

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
  server.on("/elrs", elrsPage);
  server.begin();
  Serial.println(F("done"));

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.printf("WS server begin (port %d)\n", WS_PORT);

  Serial.println(F("TankBot AP ready"));
  Serial.println(F("ELRS: RX pad -> D1(GPIO3), TX pad -> D4(GPIO2)"));
  Serial.println(F("L298N IN1/2/3/4 -> GPIO 14/12/13/15 (D5/D6/D7/D8)"));
  Serial.print(F("SSID: ")); Serial.println(AP_SSID);
  Serial.print(F("pass: ")); Serial.println(AP_PASS);
  Serial.print(F("Open http://"));
  Serial.println(WiFi.softAPIP());
  delay(500);
}

void loop(void) {
  static unsigned long lastStatus = 0, lastElrs = 0, lastHint = 0;
  static uint32_t prevFrames = 0;
  unsigned long now = millis();
  crsfPump();
  webSocket.loop();
  server.handleClient();
  if (activeCmd != "stop" && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    Serial.println(F("[safe] no command for 350ms -> stop"));
    driveStop();
    activeCmd = "stop";
  }
  if (rcLinkAlive && now - lastCrsfMs > 1000) {
    rcLinkAlive = false;
    rcActiveUntil = 0;                                // release control to the web pad
    Serial.println(F("[rc] link LOST (no CRSF frames)"));
  }
  if (now - lastElrs >= 1000) {
    lastElrs = now;
    Serial.printf("[elrs] fps=%u bytes=%lu syncs=%lu good=%lu other=%lu bad=%lu frames=%lu\n",
                  (unsigned)(crsfFrames - prevFrames), crsfBytes, crsfSyncs,
                  crsfGood, crsfOther, crsfBad, crsfFrames);
    Serial.print(F("   "));
    for (int c = 0; c < 16; c++) {
      Serial.printf("%sCH%d:%u", c ? " " : "", c + 1, crsfCh[c]);
    }
    Serial.println(F(" cmd=") + activeCmd);
    prevFrames = crsfFrames;
  }
  if (now - lastHint >= 5000) {
    lastHint = now;
    if (crsfBytes == 0)
      Serial.println(F("   hint: NO serial data on GPIO3 - receiver TX pad MUST go to the pin silkscreened RX (GPIO3). If USB is plugged, the CH340 holds GPIO3 high and blocks CRSF - unplug USB once flashed, power from L298N 5V."));
    else if (crsfGood == 0)
      Serial.println(F("   hint: bytes arrive but no valid RC frames - check CRSF baud (420000) and TX16S bind"));
  }
  if (now - lastStatus >= 5000) {
    lastStatus = now;
    Serial.printf("[status] t=%lu heap=%u cmd=%s crsf good=%lu bad=%lu frames=%lu rc=%s\n",
                  now, ESP.getFreeHeap(), activeCmd.c_str(),
                  crsfGood, crsfBad, crsfFrames,
                  millis() < rcActiveUntil ? "active" : "idle");
  }
  delay(1);
}