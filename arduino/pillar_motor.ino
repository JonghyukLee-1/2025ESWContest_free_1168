#include <Encoder.h>
#include <Servo.h>

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

// ===== 상태 =====
volatile bool g_stopRequested = false;
volatile bool g_hasCmd = false;
String g_lastCmd = "";
volatile bool g_completeFlag = false;  // (UART에서는 COMPLETE를 즉시 println)

// ===== 프로토타입 =====
void forward();
void backward();
void stopMotors();
void right();
void left();
void moveServoToAndBack(int angle);
void handleCommand(const String& cmd);

// ---- UART 라인 리더: \n 기준으로 한 줄 읽기
bool readSerialLine(String &out) {
  static char buf[64];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;    // 빈 줄 무시
      buf[idx] = '\0';
      out = String(buf);
      idx = 0;
      return true;
    }
    if (idx < sizeof(buf) - 1) buf[idx++] = c;
  }
  return false;
}

void setup() {
  // ==== UART 시작 (보레이트 필요 시 호스트와 동일하게) ====
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(0);

  stopMotors();
}

void loop() {
  // 새 명령 수신
  String cmd;
  if (readSerialLine(cmd)) {
    cmd.trim();
    if (cmd.length()) {
      noInterrupts();
      g_lastCmd = cmd;
      g_hasCmd = true;
      interrupts();
    }
  }

  if (g_hasCmd) {
    noInterrupts();
    String c = g_lastCmd;
    g_hasCmd = false;
    interrupts();
    handleCommand(c);
  }
}

void handleCommand(const String& cmd) {
  if (!cmd.length()) return;

  if (cmd.equalsIgnoreCase("w")) { forward(); return; }
  if (cmd.equalsIgnoreCase("s")) { backward(); return; }
  if (cmd.equalsIgnoreCase("a")) { left(); return; }
  if (cmd.equalsIgnoreCase("d")) { right(); return; }
  if (cmd.equalsIgnoreCase("q")) { stopMotors(); return; }

  if (cmd.equalsIgnoreCase("STOP")) {
    g_stopRequested = true;
    return;
  }

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
 * 모터 제어
 * ========================= */
void forward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);  analogWrite(ENA, SPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  analogWrite(ENB, SPEED);
}

void backward() {
  digitalWrite(IN1, HIGH);   digitalWrite(IN2, LOW); analogWrite(ENA, SPEED+5);
  digitalWrite(IN3, HIGH);   digitalWrite(IN4, LOW); analogWrite(ENB, SPEED+5);
}

void stopMotors() {
  digitalWrite(IN1, LOW);   digitalWrite(IN2, LOW);  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);   digitalWrite(IN4, LOW);  analogWrite(ENB, 0);
}

void right() {
  digitalWrite(IN1, LOW);   digitalWrite(IN2, HIGH); analogWrite(ENA, BIGSPEED);
  digitalWrite(IN3, LOW);   digitalWrite(IN4, HIGH); analogWrite(ENB, SMALLSPEED);
}

void left() {
  digitalWrite(IN1, LOW);   digitalWrite(IN2, HIGH); analogWrite(ENA, SMALLSPEED);
  digitalWrite(IN3, LOW);   digitalWrite(IN4, HIGH); analogWrite(ENB, BIGSPEED);
}

/* =========================
 * 서보 제어
 * - 완료 후 "COMPLETE" 한 줄 출력
 * - 동작 중 "STOP" 수신 시 즉시 중단
 * ========================= */
void moveServoToAndBack(int angle) {
  angle = constrain(angle, 0, 180);

  for (int a = 0; a <= angle; ++a) {
    // 동작 중 STOP 수신 체크
    String s;
    if (readSerialLine(s)) {
      s.trim();
      if (s.equalsIgnoreCase("STOP")) g_stopRequested = true;
    }
    if (g_stopRequested) { g_stopRequested = false; return; }

    servo.write(a);
    delay(30);
  }
  for (int a = angle; a >= 0; --a) {
    String s;
    if (readSerialLine(s)) {
      s.trim();
      if (s.equalsIgnoreCase("STOP")) g_stopRequested = true;
    }
    if (g_stopRequested) { g_stopRequested = false; return; }

    servo.write(a);
    delay(30);
  }

  // UART에서는 요청-응답이 없으므로 완료를 즉시 알림
  Serial.println("COMPLETE");
}
