// ============================
// Arduino Final Code (R-motor: Stepper.h, Theta-motor: L298N)
// ============================
#include <Stepper.h>

const int REV = 2048;
const float DEG_PER_STEP = 1.8;
const float CM_PER_STEP = 1.0 / 450.0;
const float MIN_DELAY_MS = 2.0;

const int trigPin = 2;
const int echoPin = 3;
const int solenoidPin = 12;

// r motor (ULN2003A, Stepper 라이브러리)
Stepper m1(REV, 8, 10, 9, 11);

// theta motor (L298N) - 남은 핀에 맞춤
const int in1Pin = 4;
const int in2Pin = 5;
const int in3Pin = 13;
const int in4Pin = 7;
const int theta_enablePin = 6; // L298N의 ENA 또는 ENB 핀

String input = "";
String sendBuffer = "";

bool ready1 = false, ready2 = false;
int remain1 = 0, remain2 = 0;
int dir1 = 0, dir2 = 0;
float delay1 = 0, delay2 = 0;
unsigned long lastStep1 = 0, lastStep2 = 0;

bool measuring = true;
bool triggered = false;
unsigned long lastSendTime = 0;

// L298N 스텝 순서
int step_sequence[4][4] = {
  {HIGH, LOW, HIGH, LOW},
  {LOW, HIGH, HIGH, LOW},
  {LOW, HIGH, LOW, HIGH},
  {HIGH, LOW, LOW, HIGH}
};
int currentStep = 0;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);

  // L298N 핀 설정
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  pinMode(theta_enablePin, OUTPUT);
  digitalWrite(theta_enablePin, HIGH); // L298N 드라이버 활성화
}

void loop() {
  readSerial();
  updateMotors();
  if (measuring) sendDistance();

  if (sendBuffer != "") {
    Serial.println(sendBuffer);
    sendBuffer = "";
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input == "STOP") {
        measuring = false;
      } else if (input == "GO" || input == "ABNORMAL") {
        measuring = true;
      } else if (input == "HIT") {
        digitalWrite(solenoidPin, HIGH);
        delay(400);
        digitalWrite(solenoidPin, LOW);
      } else if (input == "QUIT") {
        measuring = false;
        ready1 = ready2 = false;
        remain1 = remain2 = 0;
        for (int i = 4; i <= 11; ++i) digitalWrite(i, LOW);
      } else {
        processCommand(input);
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void processCommand(String cmd) {
  int i1 = cmd.indexOf(',');
  int i2 = cmd.indexOf(',', i1 + 1);
  int i3 = cmd.indexOf(',', i2 + 1);
  if (i1 < 0 || i2 < 0 || i3 < 0) return;
  int id = cmd.substring(0, i1).toInt();
  char dir = cmd.charAt(i1 + 1);
  float speed = cmd.substring(i2 + 1, i3).toFloat();
  float distance = cmd.substring(i3 + 1).toFloat();

  Serial.println("Received ID: " + String(id));
  Serial.println("Received Direction: " + String(dir));
  Serial.println("Received Speed: " + String(speed));
  Serial.println("Received Distance: " + String(distance));

  if (id == 1) {
    remain1 = int(distance / CM_PER_STEP + 0.5);
    delay1 = max(1000.0 * CM_PER_STEP / speed, MIN_DELAY_MS);
    dir1 = (dir == 'F') ? -1 : 1;
    lastStep1 = millis();
    ready1 = true;
  } else if (id == 2) {
    remain2 = int(distance / DEG_PER_STEP + 0.5);
    delay2 = max(1000.0 * DEG_PER_STEP / speed, MIN_DELAY_MS);
    dir2 = (dir == 'F') ? 1 : -1; // L298N 방향
    lastStep2 = millis();
    ready2 = true;
  }
}

void updateMotors() {
  unsigned long nowMs = millis();

  // R-motor
  if (ready1 && remain1 > 0 && (nowMs - lastStep1) >= delay1) {
    m1.step(dir1);
    remain1--;
    lastStep1 = nowMs;
  }

  // Theta-motor (L298N)
  if (ready2 && remain2 > 0 && (nowMs - lastStep2) >= delay2) {
    if (dir2 == 1) { // 정방향
      currentStep = (currentStep + 1) % 4;
    } else { // 역방향
      currentStep = (currentStep - 1 + 4) % 4;
    }
    digitalWrite(in1Pin, step_sequence[currentStep][0]);
    digitalWrite(in2Pin, step_sequence[currentStep][1]);
    digitalWrite(in3Pin, step_sequence[currentStep][2]);
    digitalWrite(in4Pin, step_sequence[currentStep][3]);
    remain2--;
    lastStep2 = nowMs;
  }

  if (remain1 <= 0) ready1 = false;
  if (remain2 <= 0) ready2 = false;

  if (!ready1 && !ready2) {
    for (int i = 4; i <= 11; i++) digitalWrite(i, LOW);
  }
}

void sendDistance() {
  if (millis() - lastSendTime > 100) {
    long duration;
    float distance;
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 20000);
    distance = duration * 0.034 / 2.0;
    Serial.print("DIST,");
    Serial.println(distance, 2);
    lastSendTime = millis();
  }
}
