#include <Wire.h>
#include <Encoder.h>
#include <Servo.h>

// ===== I2C =====
#define I2C_ADDR 0x08        // 아두이노 슬레이브 주소

// ===== 모터 핀 =====
#define IN1 4
#define IN2 5
#define ENA 6
#define IN3 8
#define IN4 9
#define ENB 11

// ===== 엔코더 핀 =====
#define LEFT_Y 2
#define LEFT_G 7

// ===== 서보 핀 =====
#define SERVO_PIN 3

Encoder myEnc(LEFT_Y, LEFT_G);
Servo servo;

// ===== 파라미터 =====
int SPEED = 95;
int BIGSPEED = 120;
int SMALLSPEED = 20;

// ===== I2C 상태 =====
volatile bool g_stopRequested = false;  
volatile bool g_hasCmd = false;         
String g_lastCmd = "";                  
volatile bool g_completeFlag = false;   

// ===== 함수 프로토타입 =====
void forward();
void backward();
void stopMotors();
void right();
void left();
void moveServoToAndBack(int angle);
void handleCommand(const String& cmd);

void onI2CReceive(int howMany) {
  char buf[33];
  int i = 0;
  while (Wire.available() && i < 32) {
    buf[i++] = (char)Wire.read();
  }
  buf[i] = '\0';

  String cmd = String(buf);
  cmd.trim();
  if (!cmd.length()) return;

  if (cmd.equalsIgnoreCase("STOP")) {
    g_stopRequested = true;
    return;
  }

  g_lastCmd = cmd;
  g_hasCmd = true;
}

void onI2CRequest() {
  if (g_completeFlag) {
    Wire.write("COMPLETE");
    g_completeFlag = false;
  } else {
    Wire.write("");
  }
}

void setup() {
  Wire.begin(I2C_ADDR);
  Wire.setClock(400000);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(0);

  stopMotors();
}

void loop() {
  if (g_hasCmd) {
    noInterrupts();
    String cmd = g_lastCmd;
    g_hasCmd = false;
    interrupts();
    handleCommand(cmd);
  }
}

void handleCommand(const String& cmd) {
  if (!cmd.length()) return;

  if (cmd.equalsIgnoreCase("w")) { forward(); return; }
  if (cmd.equalsIgnoreCase("s")) { backward(); return; }
  if (cmd.equalsIgnoreCase("a")) { left(); return; }
  if (cmd.equalsIgnoreCase("d")) { right(); return; }
  if (cmd.equalsIgnoreCase("q")) { stopMotors(); return; }

  if (cmd.length() >= 2 && (cmd[0]=='V' || cmd[0]=='v')) {
    int v = cmd.substring(1).toInt();
    SPEED = constrain(v, 0, 255);
    return;
  }

  if (cmd[0]=='b') {
    int deg = cmd.substring(1).toInt();
    moveServoToAndBack(deg);
    return;
  }

  bool allDigits = true;
  for (uint16_t i=0;i<cmd.length();++i) {
    if (!isDigit(cmd[i])) { allDigits = false; break; }
  }
  if (allDigits) {
    moveServoToAndBack(cmd.toInt());
    return;
  }
}

/* =========================
 * 모터 제어 함수
 * ========================= */
void forward() {
  digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW); analogWrite(ENA, SPEED);
  digitalWrite(IN3, HIGH);  digitalWrite(IN4, LOW); analogWrite(ENB, SPEED);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  analogWrite(ENA, SPEED+5);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  analogWrite(ENB, SPEED+5);
}

void stopMotors() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  analogWrite(ENB, 0);
}

void right() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, BIGSPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, SMALLSPEED);
}

void left() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, SMALLSPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, BIGSPEED);
}

/* =========================
 * 서보 제어
 * ========================= */
void moveServoToAndBack(int angle) {
  angle = constrain(angle, 0, 180);

  for (int a = 0; a <= angle; ++a) {
    if (g_stopRequested) { g_stopRequested = false; return; }
    servo.write(a);
    delay(30);
  }
  for (int a = angle; a >= 0; --a) {
    if (g_stopRequested) { g_stopRequested = false; return; }
    servo.write(a);
    delay(30);
  }

  g_completeFlag = true;
}
