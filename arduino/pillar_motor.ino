// ============================
// UART 버전 (I2C → UART 치환)
// ============================

#define I2C_ADDR 0x18   // (사용 안 함, 호환을 위해 남겨둠)

// ===== Pins =====
const uint8_t ENA = 5;   // PWM
const uint8_t IN1 = 6;
const uint8_t IN2 = 7;
const uint8_t ENB = 10;  // PWM
const uint8_t IN3 = 8;
const uint8_t IN4 = 9;

// ===== RX buffers =====
char     cmd_buf[32];   // 처리용 버퍼

// ===== Motor helpers =====
inline void pillarUp()    { analogWrite(ENA, 255); digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  }
inline void pillarDown()  { analogWrite(ENA, 255); digitalWrite(IN1, HIGH);  digitalWrite(IN2, LOW); }
inline void pillarStop()  { analogWrite(ENA, 0);   digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
inline void shootStart()  { analogWrite(ENB, 255); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
inline void shootStop()   { analogWrite(ENB, 0);   digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
inline void shootReverse(){ analogWrite(ENB, 0);   digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }

// ---- utils ----
static void sanitize(char* s, uint8_t& n) {
  while (n && (s[n-1]=='\r' || s[n-1]=='\n' || s[n-1]==' ' || s[n-1]=='\t')) n--;
  s[n] = '\0';
  for (uint8_t i=0; i<n; ++i) { char c=s[i]; if (c>='A' && c<='Z') s[i]=c-'A'+'a'; }
}
static bool equals(const char* a, const char* b) {
  while (*a && *b) { if (*a++ != *b++) return false; }
  return *a==0 && *b==0;
}

// ---- UART line reader (\n / \r\n 기준) ----
bool readSerialLine(char* out, uint8_t out_size, uint8_t& out_len) {
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;         // 빈 줄 무시
      uint8_t n = idx;
      if (n >= out_size) n = out_size - 1;
      for (uint8_t i=0; i<n; ++i) out[i] = buf[i];
      out[n] = '\0';
      out_len = n;
      idx = 0;
      return true;
    }

    if (idx < sizeof(buf) - 1) buf[idx++] = c;
  }
  return false;
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pillarStop(); shootStop();

  Serial.begin(9600);  // 호스트와 동일 보레이트로 설정
}

void loop() {
  uint8_t n = 0;
  if (readSerialLine(cmd_buf, sizeof(cmd_buf), n)) {
    sanitize(cmd_buf, n);

    if      (equals(cmd_buf, "up"))           pillarUp();
    else if (equals(cmd_buf, "down"))         pillarDown();
    else if (equals(cmd_buf, "pillar_stop"))  pillarStop();
    else if (equals(cmd_buf, "shoot"))        shootStart();
    else if (equals(cmd_buf, "shoot_stop"))   shootStop();
    else if (equals(cmd_buf, "shoot_rev"))    shootReverse();
    // 그 외는 무시
  }
}
